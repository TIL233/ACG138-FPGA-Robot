"""OV5647 preview, recognition, USB recording and FPGA control on HDMI."""
import json
import os
import select
import signal
from pathlib import Path
import subprocess
import threading
import time
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GdkPixbuf, GLib
from protocol import PosixTelemetryLink, PosixControlLink, CameraDiagnostic, MotionTelemetry, PredictiveTelemetry, LidarTelemetry, LidarPlanningTelemetry, NavTelemetry, RANGE_FAR, RANGE_INVALID
from lidar_planning import planning_summary, planning_details, planning_snapshot
from lidar_map_view import MAP_SIZE, MAP_RADIUS, render_map
from camera_capture import CameraConfig, CaptureReader, capture_command, load_config, run_command, ov5647_only
from video_recording import VideoRecorder, RemoteRecordTrigger, load_recording_config, recording_presentation
from recording_overlay import TextOverlayRenderer
from usb_lidar import LidarUsbBridge,load_config as load_usb_lidar_config

ROOT = Path(__file__).resolve().parent
SERIAL = '/dev/serial/by-path/platform-fc800000.usb-usb-0:1.3:1.0-port0'
WIDTH, HEIGHT = 1280, 720


def distance_text(value, valid):
    if not valid or value == RANGE_INVALID:return '无有效数据'
    if value == RANGE_FAR:return '超量程/无回波（按远距）'
    return f'{value/10:.1f} cm'


def concise_status(*, camera_name, source, view_live, telemetry, live, objects,
                   detection_live, following=False, navigation_text=None,
                   mission_text=None, motion=None, predictive=None, notice='',follow_speed=None,recording=None,
                   lidar_planning=None, lidar_planning_live=False):
    """Presentation only: five normal rows plus task/exception rows as needed."""
    t = telemetry if live else None
    source_name = camera_name+'画面' if source == 'usb' else 'OV5647画面' if source == 'csi' else 'FPGA采集画面'
    mode = '任务6' if mission_text else '路径导航' if navigation_text else '人员跟随' if following else '手动' if t and t.health & 64 else '待命'
    lines = [f'{source_name}  |  {mode}  |  '+('控制在线' if t else '等待控制数据')]
    action = {0:'停止',1:'前进',2:'后退',3:'左转',4:'右转',5:'定点左转',6:'定点右转'}
    if t:
        status = '绕障中' if 1 <= t.state <= 6 else action.get(t.command, '未知')
        if t.health & 32: status = '已停车'
        speed_text=f'跟随档 {follow_speed}' if following and follow_speed is not None else f'速度档 {t.speed}'
        lines.append(f'{status}  ·  {speed_text}  |  轮速 左 {t.velocity_l}  右 {t.velocity_r}')
        def distance(value, bit):
            if not t.health & bit or value == RANGE_INVALID: return '--'
            return '远' if value == RANGE_FAR else f'{value/10:.1f}cm'
        ranges = f'距离 左 {distance(t.distance_l,4)}  右 {distance(t.distance_r,8)}'
        pose = (f'横滚 {t.roll/100:.1f}°  俯仰 {t.pitch/100:.1f}°  航向 {t.yaw/100:.1f}°'
                if t.health & 2 else '姿态：等待有效数据')
        if not t.health & 2 and isinstance(motion,MotionTelemetry):
            if motion.flags & 0x110 == 0x110:
                pose = '姿态：航向异常锁定，放稳后按5清除'
            elif motion.flags & 0x90 == 0x90:
                pose = '姿态：编码器异常锁定，检查后按5清除'
    else:
        lines.append('运动状态：等待数据')
        ranges, pose = '距离 左 --  右 --', '姿态：等待有效数据'
    recognition = (f'人 {sum(o[0]==0 for o in objects)}  瓶子 {sum(o[0]==39 for o in objects)}'
                   if detection_live else '识别等待 / 已超时')
    lines.extend([ranges+'  |  '+recognition, pose])
    planning_text = planning_summary(lidar_planning, lidar_planning_live)
    if planning_text:
        lines.append(planning_text)
    task = mission_text or navigation_text
    if task: lines.append(('任务6：' if mission_text else '导航：')+str(task).replace('\n',' ')[:48])
    warnings = []
    if t:
        if isinstance(predictive,PredictiveTelemetry) and predictive.fault: warnings.append('预测控制器异常，按5清除')
        if t.fault: warnings.append(f'故障 0x{t.fault:04X}，查看详情')
        if not t.health & 2: warnings.append('姿态反馈异常')
        if not t.health & 16: warnings.append('停止许可未满足')
        if isinstance(motion,MotionTelemetry):
            if motion.flags & 128: warnings.append('编码器跳变')
            if motion.flags & 256: warnings.append('IMU航向跳变')
            if motion.disagreement: warnings.append('轮速与航向反馈不一致')
        distances=[v for bit,v in ((4,t.distance_l),(8,t.distance_r)) if t.health&bit and 0<v<RANGE_FAR]
        if distances and following and min(distances)<=600:
            warnings.append('跟随距离≤60cm，保持停车')
        elif distances and min(distances)<200: warnings.append('前方近障')
        if not t.health & 1 and not t.fault: warnings.append('先按5准备')
    if not view_live: warnings.append('当前画面无信号 / 已超时')
    if warnings: lines.append('提示：'+'；'.join(warnings[:3]))
    if notice: lines.append('提示：'+str(notice).replace('\n',' ')[:48])
    if recording:
        lines.append(recording_presentation(recording)['text'])
    lines.append('0 跟随   5 停车   7 路径寻瓶   9 录像   VOL± 调速')
    return lines


def motion_status(t, following=False):
    """Explain control restrictions without confusing them with UART loss."""
    if t is None:
        return '控制状态：等待遥测'
    reasons = []
    if not t.health & 1:
        reasons.append('控制未准备：先按5，再点按方向键')
    # Health bit 1 describes the complete motion estimator, not UART activity.
    # IMU samples can remain fresh while a latched yaw/encoder fault stops it.
    if not t.health & 2:
        reasons.append('姿态反馈不可用，查看诊断')
    if not t.health & 16:
        reasons.append('停止许可未满足')
    if t.fault & 32:
        reasons.append('编码器/电机反馈故障：检查接线后按5')
    if t.fault & 0xFF00:
        reasons.append('绕障失败/超时：按5停车重置')
    if t.fault & 0x1F:
        reasons.append('故障已锁定：条件恢复后仍需按5重置')
    ranges=[d for bit,d in ((4,t.distance_l),(8,t.distance_r)) if t.health&bit and 0<d<65535]
    if ranges and following and min(ranges)<=600:
        reasons.append('跟随距离≤60cm：保持停车，不自动绕障')
    elif ranges and not following and min(ranges)<200:
        reasons.append('距离<20cm：前进/前向转弯受限')
    sonar_notes='；'.join(name+'无有效数据（不阻止运动）' for bit,name in ((4,'左测距'),(8,'右测距')) if not t.health&bit)
    suffix=('；'+sonar_notes) if sonar_notes else ''
    if reasons:
        return '限制：'+'；'.join(reasons)+suffix
    return '控制状态：'+('点按持续运动，按5停车' if t.health & 64 else '停车；首次/故障先按5，方向键点按启动，VOL±调速')+suffix

class Viewer:
    def __init__(self, follow_category=None, route_path=None, mission_path=None, camera_config=None):
        self.usb_lidar=None
        self.usb_lidar_error=''
        self.camera_config = ov5647_only(camera_config or load_config(ROOT/'camera_config.json'))
        self.camera_name = {'usb':'USB','ov5647':'OV5647','fpga':'FPGA DVP'}[self.camera_config.primary]
        self.csi_reader = None
        self.recorder = None
        self.remote_record = RemoteRecordTrigger()
        self.recording_config = load_recording_config(ROOT/'recording_config.json')
        self.fpga_receiver = None
        self.fpga_video_error = ''
        self.fpga_monitor_requested = self.camera_config.fpga_monitor or self.camera_config.primary == 'fpga'
        self.fpga_retry = 0.
        self.fpga_last_displayed = None
        self.preview_source = ('csi' if self.camera_config.preview == 'ov5647'
                               and self.camera_config.primary != 'ov5647' else None)
        self.follower = None
        self.navigator = None
        self.mission = None
        if sum(value is not None for value in (follow_category,route_path,mission_path))>1:
            raise ValueError('follow, navigation and mission modes are mutually exclusive')
        if mission_path is not None:
            from mission import MissionSession,load_mission
            self.mission=MissionSession(**load_mission(mission_path,include_semantics=True))
        self.semantic_path=ROOT/'mission_semantic_example.json'
        if not self.semantic_path.exists():
            self.semantic_path=ROOT.parent/'robot_rebuild/rk/mission_semantic_example.json'
        if mission_path is not None:
            try:
                if 'locations' in json.loads(Path(mission_path).read_text(encoding='utf-8-sig')):
                    self.semantic_path=Path(mission_path)
            except (OSError,ValueError):pass
        self.ui_notice=''
        self.control_stop_pending=False
        self.semantic_dialog_open=False
        from follow import RemoteFollowTrigger
        self.remote_path_mode=RemoteFollowTrigger('nav_mode_count')
        self.navigation_route_path=Path(route_path) if route_path is not None else ROOT/'route.json'
        if route_path is None and not self.navigation_route_path.exists():
            self.navigation_route_path=ROOT/'route_example.json'
            if not self.navigation_route_path.exists():
                self.navigation_route_path=ROOT.parent/'robot_rebuild/rk/route_example.json'
        if route_path is not None:
            from navigation import NavigationSession, load_route
            self.navigator=NavigationSession(load_route(route_path),remote_enabled=True)
        if follow_category is not None:
            from follow import TargetFollower, RemoteFollowTrigger
            self.follower = TargetFollower()
            self.remote_follow = RemoteFollowTrigger()
        self.link = PosixControlLink(SERIAL) if self.follower or self.navigator or self.mission else PosixTelemetryLink(SERIAL)
        self.follow_last_send = float('-inf')
        self.follow_was_armed = False
        self.follow_feedback_deadline = 0.
        self.follow_feedback_seq = None
        self.follow_expected = None
        self.follow_disarm_pending = bool(self.follower)
        self.follow_start_pending = None
        self.camera_frames = 0
        self.camera_lock = threading.Lock()
        self.camera_latest = (None, 0.0, 0, '')
        self.last_displayed = 0
        self.display_source = 'usb'
        self.stopping = False
        self.stop_event = threading.Event()
        self.process = None
        self.log = None
        self.thread = None
        self.timer = None
        self.last_report = 0
        self.control_events = []
        self.show_details = False
        self.previous_control_state = None
        self.recognition = (None, 0.0, 0, [], 0.0, '识别模型加载中')
        self.inference_thread = threading.Thread(target=self.recognize, daemon=True)
        self.window = Gtk.Window(title=self.camera_name+' Camera + FPGA Telemetry')
        self.window.set_default_size(1920, 1080)
        self.window.set_decorated(False)
        self.window.set_resizable(False)
        self.window.move(0, 0)
        self.window.connect('destroy', self.close)
        self.window.connect('key-press-event', self.key)
        self.window.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        signal.signal(signal.SIGTERM, lambda *_: self.window.destroy())
        signal.signal(signal.SIGINT, lambda *_: self.window.destroy())
        overlay = Gtk.Overlay()
        self.image = Gtk.Image()
        overlay.add(self.image)
        self.label = Gtk.Label()
        self.label.set_line_wrap(True)
        self.label.set_max_width_chars(90)
        self.label.set_xalign(0)
        self.label.set_halign(Gtk.Align.START)
        self.label.set_valign(Gtk.Align.END)
        self.label.set_margin_start(24)
        self.label.set_margin_bottom(24)
        self.label.set_name('telemetry')
        overlay.add_overlay(self.label)
        self.record_indicator=Gtk.Label(label='未录像｜按9开始录像')
        self.record_indicator.set_halign(Gtk.Align.START)
        self.record_indicator.set_valign(Gtk.Align.START)
        self.record_indicator.set_margin_start(24)
        self.record_indicator.set_margin_top(16)
        self.record_indicator.set_name('recording-status')
        overlay.add_overlay(self.record_indicator)
        sources = Gtk.Box(spacing=8)
        sources.set_halign(Gtk.Align.END)
        sources.set_valign(Gtk.Align.START)
        sources.set_margin_top(16)
        sources.set_margin_end(16)
        source_buttons = [(self.camera_name+'画面', 'usb')]
        if self.preview_source:
            source_buttons.append(('OV5647', self.preview_source))
        for title, source in source_buttons:
            button = Gtk.Button(label=title)
            button.connect('clicked', lambda _, selected=source: self.set_source(selected))
            sources.pack_start(button, False, False, 0)
        if self.fpga_monitor_requested:
            button = Gtk.Button(label='FPGA画面')
            button.connect('clicked', lambda _: self.set_source('fpga'))
            sources.pack_start(button, False, False, 0)
        for title,callback in [('任务6',self.show_semantic_dialog),
                               ('执行任务6',self.start_semantic_task),
                               ('跟随模式',self.activate_follow_mode),('停车',self.stop_controls)]:
            button=Gtk.Button(label=title)
            button.connect('clicked',callback)
            sources.pack_start(button,False,False,0)
        self.record_button=Gtk.Button(label='开始录像（9）')
        self.record_button.connect('clicked',self.toggle_recording)
        sources.pack_start(self.record_button,False,False,0)
        self.details_button=Gtk.ToggleButton(label='详情')
        self.details_button.connect('toggled',self.toggle_details)
        sources.pack_start(self.details_button,False,False,0)
        self.lidar_map_button = Gtk.Button(label='雷达地图')
        self.lidar_map_button.connect('clicked', self.show_lidar_map)
        self.lidar_map_button.set_no_show_all(True)
        sources.pack_start(self.lidar_map_button, False, False, 0)
        overlay.add_overlay(sources)
        # Independent task-1 evidence: never substitute the RK image for this pane.
        self.fpga_pane = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        self.fpga_pane.set_halign(Gtk.Align.START)
        self.fpga_pane.set_valign(Gtk.Align.START)
        self.fpga_pane.set_margin_start(24)
        self.fpga_pane.set_margin_top(80)
        self.fpga_label = Gtk.Label(label='FPGA DVP采集：等待独立图像')
        self.fpga_label.set_name('fpga-monitor')
        self.fpga_image = Gtk.Image()
        self.fpga_image.set_size_request(320, 240)
        self.fpga_pane.pack_start(self.fpga_label, False, False, 0)
        self.fpga_pane.pack_start(self.fpga_image, False, False, 0)
        overlay.add_overlay(self.fpga_pane)
        css = Gtk.CssProvider()
        css.load_from_data(b'window {background: black;} #telemetry {background: rgba(0,0,0,0.72); color: white; font: 23px "Noto Sans CJK SC"; padding: 14px 20px; border-radius: 8px;} #fpga-monitor {background: rgba(0,0,0,0.8); color: white; font: 18px "Noto Sans CJK SC"; padding: 4px 8px;}')
        record_css=Gtk.CssProvider()
        record_css.load_from_data(b'#recording-status {background: rgba(30,30,30,0.92); color: white; font: bold 27px "Noto Sans CJK SC"; padding: 12px 18px; border-radius: 8px;} #recording-status.recording, #recording-status.error {background: #b91c1c;} #recording-status.preparing, #recording-status.saving {background: #854d0e;} #recording-status.saved {background: #166534;} #lidar-map {background: #13181d; padding: 16px;} #lidar-map label, #lidar-map drawingarea {color: white; font: 21px "Noto Sans CJK SC";} #lidar-map button label {color: inherit;}')
        Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), css, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
        Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), record_css, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
        self.view_stack = Gtk.Stack()
        self.view_stack.add_named(overlay, 'camera')
        self.view_stack.add_named(self.make_map_page(), 'map')
        self.window.add(self.view_stack)
        self.window.show_all()
        self.show_camera_page()
        self.fpga_pane.set_visible(self.fpga_monitor_requested)
        # Match the HDMI output without changing the desktop's display layout.
        self.window.resize(1920, 1080)
        self.window.move(0, 0)
        try:
            self.ensure_fpga_monitor(time.monotonic())
            self.recorder=VideoRecorder(self.recording_config,WIDTH,HEIGHT)
            self.log = open(ROOT/'camera-pipeline.log', 'wb')
            self.thread = threading.Thread(target=self.read_camera, daemon=True)
            self.thread.start()
            self.inference_thread.start()
            self.timer = GLib.timeout_add(50, self.update)
            try:
                self.usb_lidar=LidarUsbBridge(lambda:self.link,load_usb_lidar_config(ROOT/'usb_lidar_config.json'))
            except (OSError,ValueError,RuntimeError) as exc:
                self.usb_lidar_error=str(exc)
        except BaseException:
            self.close()
            raise

    def hdmi_monitor(self):
        screen = Gdk.Screen.get_default()
        for n in range(screen.get_n_monitors()):
            if screen.get_monitor_plug_name(n) == 'HDMI-2': return n
        return screen.get_primary_monitor()

    def toggle_details(self, button):
        # Presentation only; do not stop, ARM or reconfigure any controller.
        self.show_details=bool(button.get_active())
        button.set_label('收起详情' if self.show_details else '详情')

    def make_map_page(self):
        page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        page.set_name('lidar-map')
        header = Gtk.Box(spacing=12)
        header.pack_start(Gtk.Label(label='雷达地图　｜　车身周围 8 米'), False, False, 0)
        for title, action in [('返回摄像头', self.show_camera_page), ('停车', self.stop_controls)]:
            button = Gtk.Button(label=title)
            button.connect('clicked', action)
            header.pack_end(button, False, False, 0)
        page.pack_start(header, False, False, 0)
        self.lidar_map_canvas = Gtk.DrawingArea()
        self.lidar_map_canvas.connect('draw', self.draw_lidar_map)
        page.pack_start(self.lidar_map_canvas, True, True, 0)
        self.lidar_map_caption = Gtk.Label()
        page.pack_end(self.lidar_map_caption, False, False, 0)
        self.lidar_map_key = None
        self.lidar_map_pixbuf = None
        return page

    def show_lidar_map(self, *_):
        if self.link.lidar_map_telemetry is not None:
            self.view_stack.set_visible_child_name('map')
            self.update_lidar_map(time.monotonic())

    def show_camera_page(self, *_):
        self.view_stack.set_visible_child_name('camera')

    def draw_lidar_map(self, canvas, cr):
        if self.lidar_map_pixbuf is None:
            return False
        from gi.repository import PangoCairo
        width, height = canvas.get_allocated_width(), canvas.get_allocated_height()
        side = max(1, min(width - 80, height - 56))
        cx, cy = width/2, height/2
        cr.save()
        cr.translate(cx-side/2, cy-side/2)
        cr.scale(side/MAP_SIZE, side/MAP_SIZE)
        Gdk.cairo_set_source_pixbuf(cr, self.lidar_map_pixbuf, 0, 0)
        cr.paint()
        cr.restore()
        r = side * MAP_RADIUS/MAP_SIZE
        labels = [('前 · 车头', cx, cy-r-18), ('后', cx, cy+r+18),
                  ('左', cx-r-18, cy), ('右', cx+r+18, cy)]
        # Mark the rings on a ruler outside the scan, so labels never hide a nearby return.
        ruler_x = cx + r + 8
        cr.set_source_rgb(.65, .7, .74)
        cr.set_line_width(1)
        cr.move_to(ruler_x, cy-r/4)
        cr.line_to(ruler_x, cy-r)
        for ring, metres in enumerate((1, 2, 4, 8), 1):
            y = cy - r*ring/4
            cr.move_to(ruler_x, y)
            cr.line_to(ruler_x+6, y)
            labels.append((f'{metres} m', ruler_x+26, y))
        cr.stroke()
        for text, x, y in labels:
            layout = canvas.create_pango_layout(text)
            w, h = layout.get_pixel_size()
            cr.set_source_rgba(.07, .09, .11, .85)
            cr.rectangle(x-w/2-2, y-h/2, w+4, h)
            cr.fill()
            cr.set_source_rgb(1, 1, 1)
            cr.move_to(x-w/2, y-h/2)
            PangoCairo.show_layout(cr, layout)
        return False

    def update_lidar_map(self, now):
        if not hasattr(self, 'view_stack'):
            return
        sample = self.link.lidar_map_telemetry
        self.lidar_map_button.set_visible(sample is not None)
        if self.view_stack.get_visible_child_name() != 'map':
            return
        live = sample is not None and self.link.lidar_map_live(now)
        mask, ranges = (sample.known_mask, sample.ranges_mm) if live else (0, ())
        key = (live, mask, ranges)
        if self.lidar_map_key == key:
            return
        rgb = GLib.Bytes.new(render_map(mask, ranges))
        pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(rgb, GdkPixbuf.Colorspace.RGB, False, 8,
                                                MAP_SIZE, MAP_SIZE, MAP_SIZE*3)
        self.lidar_map_pixbuf = pixbuf
        self.lidar_map_canvas.queue_draw()
        legend = '橙点为障碍　灰区未知' if live else '等待新扫描　灰区未知'
        self.lidar_map_caption.set_text(legend + '　｜　距离圈由内向外：1、2、4、8 米')
        self.lidar_map_key = key

    def toggle_recording(self,*_):
        recorder=getattr(self,'recorder',None)
        if recorder is not None:return recorder.toggle()

    def offer_recording(self,rgb,captured,lines,now,*,fallback_rgb=None,fallback_captured=0.):
        """Publish one frame, its capture time and the visible data panel together."""
        recorder=getattr(self,'recorder',None)
        if recorder is None:return
        if rgb is None:
            recorder.offer(None,0.)
            return
        if now-getattr(self,'last_recording_offer',float('-inf'))<.1:return
        self.last_recording_offer=now
        try:
            renderer=getattr(self,'recording_overlay_renderer',None)
            if renderer is None:
                renderer=self.recording_overlay_renderer=TextOverlayRenderer(WIDTH,HEIGHT)
            overlay=renderer.render('\n'.join(lines))
            recorder.offer(rgb,captured,overlay,fallback_rgb=fallback_rgb,
                           fallback_captured=fallback_captured)
        except Exception as exc:
            recorder.offer(None,0.)
            recorder.fail('录像数据叠加失败：'+str(exc))
        return False

    def remote_record_step(self,now):
        trigger=getattr(self,'remote_record',None)
        if trigger is not None and trigger.observe(getattr(self.link,'nav_telemetry',None),self.link.nav_live(now)):
            self.toggle_recording()

    def _stop_controllers(self,reason):
        for name in ('follower','navigator','mission'):
            controller=getattr(self,name,None)
            if controller is not None:controller.stop(reason)
        self.follow_start_pending=None
        self.follow_was_armed=False
        self.follow_disarm_pending=False
        if not callable(getattr(self.link,'send',None)):
            # Explicitly selecting a task enables writing; ordinary viewing
            # still creates only the receive-only link.
            self.link.close()
            self.link=PosixControlLink(SERIAL)
            self.link.poll(time.monotonic())
        try:
            self.link.send()
            self.control_stop_pending=False
        except OSError:
            self.control_stop_pending=True

    def stop_controls(self,*_):
        self._stop_controllers('操作者停车；任务需重新生成或明确启动')
        self.ui_notice='已请求停车；遥控5可直接在FPGA停车'

    def prepare_semantic_task(self,instruction):
        from mission import MissionSession,load_mission
        config=load_mission(self.semantic_path,instruction=instruction,include_semantics=True)
        new_task=MissionSession(**config)  # Validate before changing controllers.
        self._stop_controllers('切换语义任务，已请求停车')
        self.follower=self.navigator=None
        self.remote_follow=None
        self.mission=new_task
        self.set_source('usb')
        names=' → '.join(new_task.location_names)
        self.ui_notice='已生成：'+instruction+'；地点：'+names+'。自动以当前车位和车头为起点，点一次“执行任务6”，停车准备后自动执行'
        return new_task

    def start_semantic_task(self,*_):
        if getattr(self,'semantic_dialog_open',False) or getattr(self,'control_stop_pending',False):
            self.ui_notice='等待输入结束及停车指令送达'
            return False
        mission=getattr(self,'mission',None)
        if mission is None:
            self.ui_notice='请先点“任务6：语义导航”生成任务'
            return False
        if getattr(self,'display_source','usb')!='usb':
            self.ui_notice='请切回识别画面后明确启动任务'
            return False
        started=mission.request_run(self.link,time.monotonic(),source='button')
        self.ui_notice='' if started else mission.text
        return started

    def activate_follow_mode(self,*_):
        from follow import TargetFollower,RemoteFollowTrigger
        self._stop_controllers('切换人员跟随，已请求停车')
        self.mission=self.navigator=None
        self.follower=TargetFollower()
        self.remote_follow=RemoteFollowTrigger()
        self.follow_disarm_pending=True
        self.follow_expected=None
        self.follow_last_send=float('-inf')
        self.set_source('usb')
        self.ui_notice='人员跟随模式：识别到人后点按遥控0启动，5停车'

    def activate_navigation_mode(self,now=None):
        """Key7 explicitly starts one waypoint bottle-search mission."""
        from navigation import load_route
        from mission import MissionSession
        now=time.monotonic() if now is None else now
        if getattr(self,'semantic_dialog_open',False) or getattr(self,'control_stop_pending',False):
            self.ui_notice='7键请求未排队：等待输入结束及停车指令送达后重新按7'
            return False
        mission=getattr(self,'mission',None)
        if (mission is not None and getattr(mission,'remote_path_search',False)
                and (mission.enabled or mission.run_request is not None)):
            self.ui_notice='路径寻瓶正在准备或执行；5停车'
            return True
        try:
            mission=MissionSession(load_route(self.navigation_route_path),action='find_bottle')
            mission.remote_path_search=True
        except (OSError,ValueError) as exc:
            self._stop_controllers('7键选择路径寻瓶，但路线配置无效，保持停车')
            self.ui_notice='搜索未启动：'+str(exc)
            return False
        # Capture the live STOP generation before a receive-only link is
        # replaced. The new controller waits for fresh data before ARM.
        accepted=mission.request_run(self.link,now)
        self._stop_controllers('7键切换路径寻瓶，先停车再准备')
        self.follower=self.navigator=self.remote_follow=None
        self.mission=mission
        if getattr(self,'remote_path_mode',None) is not None:self.remote_path_mode.previous=None
        self.set_source('usb')
        self.ui_notice='已收到7：沿路线逐点搜索水瓶，无需再按0' if accepted else mission.text
        return accepted

    def remote_path_mode_step(self,now):
        from follow import RemoteFollowTrigger
        trigger=getattr(self,'remote_path_mode',None)
        if trigger is None:
            trigger=self.remote_path_mode=RemoteFollowTrigger('nav_mode_count')
        nav=getattr(self.link,'nav_telemetry',None)
        event=trigger.observe(nav,self.link.live(now) and self.link.nav_live(now))
        if event=='stop':
            navigator=getattr(self,'navigator',None)
            mission=getattr(self,'mission',None)
            if mission is not None and getattr(mission,'remote_path_search',False):
                mission.stop('遥控停车/连接异常，搜索已取消；重新按7启动')
                if trigger.remote_stop:
                    mission._pending_disarm=mission.navigator.pending_disarm=False
            if trigger.remote_stop:
                self.control_stop_pending=False
                if navigator is not None:
                    navigator.stop('遥控5停车，7键请求已取消')
                    navigator.pending_disarm=False
                    navigator.stop_epoch=nav.stop_epoch
            elif navigator is not None and (navigator.run_request is not None or navigator.enabled or navigator.start_at is not None):
                navigator.stop('遥控事件/连接异常，7键请求已取消；恢复后重新按7')
        elif event=='start':
            self.activate_navigation_mode(now)

    def show_semantic_dialog(self,*_):
        # A modal input window must never obscure an active autonomous drive.
        self.stop_controls()
        self.semantic_dialog_open=True
        dialog=None
        try:
            from mission import load_mission
            load_mission(self.semantic_path)
            data=json.loads(self.semantic_path.read_text(encoding='utf-8-sig'))
            locations=data.get('locations')
            if not locations:raise ValueError('语义任务需要locations地点配置')
            dialog=Gtk.Dialog(title='任务6：中文语义导航',transient_for=self.window,
                              flags=Gtk.DialogFlags.MODAL)
            dialog.add_button('取消',Gtk.ResponseType.CANCEL)
            dialog.add_button('生成任务（不启动）',Gtk.ResponseType.OK)
            dialog.set_default_size(760,360)
            area=dialog.get_content_area()
            intro=Gtk.Label(label='点选下面的地点任务，或输入中文指令。\n先放好小车，生成新任务时自动以当前车位和车头为起点；同一任务各点共用起点和初始车头，不随途中转向改变。地点以以下毫米坐标配置为准，+X为初始前方、+Y为初始左方。以下按钮只填入指令。')
            intro.set_line_wrap(True);area.pack_start(intro,False,False,8)
            entry=Gtk.Entry();entry.set_max_length(256)
            entry.set_text(data.get('instruction','去'+next(iter(locations))+'找人'))
            area.pack_start(entry,False,False,8)
            for name,point in locations.items():
                row=Gtk.Box(spacing=8)
                row.pack_start(Gtk.Label(label=f'{name}：({point[0]}, {point[1]}) mm'),False,False,8)
                for title,wording in (('导航','去'+name),('找人','去'+name+'找人'),('找水瓶','去'+name+'找水瓶')):
                    button=Gtk.Button(label=title)
                    button.connect('clicked',lambda _,text=wording:entry.set_text(text))
                    row.pack_start(button,False,False,0)
                area.pack_start(row,False,False,4)
            dialog.show_all()
            if dialog.run()==Gtk.ResponseType.OK:
                self.prepare_semantic_task(entry.get_text())
        except (OSError,ValueError,TypeError) as exc:
            self.ui_notice='语义任务未生成：'+str(exc)
        finally:
            if dialog is not None:dialog.destroy()
            self.semantic_dialog_open=False
            if getattr(self,'remote_path_mode',None) is not None:self.remote_path_mode.previous=None
            if getattr(self,'remote_follow',None) is not None:
                # Ignore any start event still in flight from the input window.
                self.remote_follow.previous=None

    def set_source(self, source):
        if source not in ('usb', 'ov5647', 'csi', 'fpga'):
            raise ValueError('unknown display source')
        if source=='fpga':
            self.ui_notice='当前摄像头功能统一使用OV5647'
            return False
        # 'usb' is the old internal primary-view token, not a UVC capture path.
        source='usb'
        self.display_source = source
        self.last_displayed = None
        self.image.clear()
        if source != 'csi' and getattr(self, 'csi_reader', None) is not None:
            self.csi_reader.close()
            self.csi_reader = None
        return True

    def ensure_fpga_monitor(self, now):
        if (not getattr(self, 'fpga_monitor_requested', False)
                or getattr(self, 'fpga_receiver', None) is not None
                or now < getattr(self, 'fpga_retry', 0.)):
            return
        self.fpga_retry = now+2
        try:
            from video import VideoReceiver
            self.fpga_receiver = VideoReceiver()
            self.fpga_video_error = ''
        except (OSError, ImportError) as exc:
            self.fpga_video_error = str(exc)

    def update_fpga_monitor(self, now):
        self.ensure_fpga_monitor(now)
        receiver = getattr(self, 'fpga_receiver', None)
        image, stamp = receiver.snapshot() if receiver else (None, 0.)
        valid = image is not None and 0 <= now-stamp < .5
        error = getattr(receiver, 'error', None) or getattr(self, 'fpga_video_error', '')
        widget, caption = getattr(self, 'fpga_image', None), getattr(self, 'fpga_label', None)
        if widget is not None:
            if valid and stamp != getattr(self, 'fpga_last_displayed', None):
                rgb = image[:, :, ::-1].tobytes()
                buf = GdkPixbuf.Pixbuf.new_from_bytes(GLib.Bytes.new(rgb),GdkPixbuf.Colorspace.RGB,False,8,640,480,1920)
                widget.set_from_pixbuf(buf.scale_simple(320,240,GdkPixbuf.InterpType.BILINEAR))
                self.fpga_last_displayed = stamp
            elif not valid:
                widget.clear()
                self.fpga_last_displayed = None
        if caption is not None:
            caption.set_text('FPGA DVP采集：'+('实时图像' if valid else error or '等待独立图像 / 已超时'))
        pane = getattr(self, 'fpga_pane', None)
        if pane is not None:
            pane.set_visible(bool(getattr(self,'fpga_monitor_requested',False))
                             and getattr(self,'display_source','usb')!='fpga'
                             and (valid or getattr(self,'show_details',False)))
        return image, stamp, valid, error

    def read_camera(self):
        size = WIDTH*HEIGHT*3
        while not self.stop_event.is_set():
            process = None
            try:
                def checked_run(argv):
                    if self.stop_event.is_set(): raise InterruptedError('closing capture')
                    return run_command(argv)
                command = capture_command(ov5647_only(getattr(self, 'camera_config', CameraConfig())),
                                          width=WIDTH, height=HEIGHT, run=checked_run)
                if self.stop_event.is_set(): break
                process = subprocess.Popen(command,
                    stdout=subprocess.PIPE, stderr=self.log, bufsize=0)
                self.process = process
                pending = bytearray()
                last_data = time.monotonic()
                while not self.stop_event.is_set():
                    if time.monotonic()-last_data > 3:
                        raise TimeoutError('摄像头读取超时')
                    ready, _, _ = select.select([process.stdout], [], [], 0.2)
                    if not ready:
                        continue
                    chunk = os.read(process.stdout.fileno(), size-len(pending))
                    if not chunk:
                        raise OSError('摄像头断开或采集程序已退出')
                    pending.extend(chunk)
                    if len(pending) == size:
                        last_data = time.monotonic()
                        self.camera_frames += 1
                        rgb=bytes(pending)
                        with self.camera_lock:
                            self.camera_latest = (rgb, last_data, self.camera_frames, '')
                        pending.clear()
            except Exception as exc:
                if not self.stop_event.is_set():
                    with self.camera_lock:
                        self.camera_latest = (None, 0.0, self.camera_frames, str(exc))
                    recorder=getattr(self,'recorder',None)
                    if recorder is not None:recorder.offer(None,0.)
            finally:
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=1)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=1)
                    process.stdout.close()
                self.process = None
            self.stop_event.wait(2)

    def read_fpga_camera(self):
        """Reuse the one UDP owner for recognition, preserving its capture stamp."""
        last = None
        try:
            import cv2
            import numpy as np
            while not self.stop_event.is_set():
                receiver = getattr(self, 'fpga_receiver', None)
                image, stamp = receiver.snapshot() if receiver else (None, 0.)
                now = time.monotonic()
                if image is None or not 0 <= now-stamp < .5:
                    error = getattr(receiver,'error',None) or getattr(self,'fpga_video_error','') or 'FPGA采集无图像 / 已超时'
                    with self.camera_lock:
                        self.camera_latest = (None,0.,self.camera_frames,error)
                elif stamp != last:
                    # The DVP stream is 4:3. Preserve aspect ratio with side bars.
                    canvas = np.zeros((HEIGHT,WIDTH,3),dtype=np.uint8)
                    width = min(WIDTH,HEIGHT*640//480)
                    height = min(HEIGHT,WIDTH*480//640)
                    rgb = cv2.resize(image[:,:,::-1],(width,height))
                    x,y = (WIDTH-width)//2,(HEIGHT-height)//2
                    canvas[y:y+height,x:x+width] = rgb
                    last = stamp
                    self.camera_frames += 1
                    with self.camera_lock:
                        self.camera_latest = (canvas.tobytes(),stamp,self.camera_frames,'')
                self.stop_event.wait(.02)
        except Exception as exc:
            with self.camera_lock:
                self.camera_latest = (None,0.,self.camera_frames,str(exc))

    def recognize(self):
        # Retry a failed inference owner without restarting the UI or controls.
        while not self.stop_event.is_set():
            try:
                import numpy as np
                from detector import Detector
                detector = Detector(ROOT/'yolov8n.onnx')
                last = 0
                while not self.stop_event.is_set():
                    with self.camera_lock:
                        data, stamp, frame, error = self.camera_latest
                    if data is None or frame == last or time.monotonic()-stamp > 1:
                        self.stop_event.wait(.03)
                        continue
                    last = frame
                    rgb = np.frombuffer(data, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()
                    start = time.monotonic()
                    objects = detector.detect(rgb[:, :, ::-1].copy())
                    elapsed = (time.monotonic()-start)*1000
                    for category, confidence, box in objects:
                        x1,y1,x2,y2 = map(int, box)
                        color = (40,255,80) if category == 0 else (255,200,40)
                        detector.cv.rectangle(rgb,(x1,y1),(x2,y2),color,3)
                        detector.cv.putText(rgb, f'{"person" if category == 0 else "bottle"} {confidence:.0%}',
                            (x1,max(25,y1-8)),detector.cv.FONT_HERSHEY_SIMPLEX,.8,color,2)
                    with self.camera_lock:
                        self.recognition = (rgb.tobytes(), stamp, frame, objects, elapsed, '')
            except Exception as exc:
                with self.camera_lock:
                    self.recognition = (None,0.0,0,[],0.0,str(exc))
                if self.stop_event.wait(1.):
                    return

    def update(self):
        if self.stopping: return False
        now = time.monotonic()
        self.link.poll(now)
        self.remote_record_step(now)
        self.remote_path_mode_step(now)
        if getattr(self,'control_stop_pending',False):
            try:
                self.link.send()
                self.control_stop_pending=False
            except OSError:pass
        with self.camera_lock:
            data, camera_stamp, camera_frames, camera_error = self.camera_latest
        with self.camera_lock:
            annotated, detection_stamp, detection_frame, objects, inference_ms, detection_error = getattr(self, 'recognition', (None,0.0,0,[],0.0,'识别未启用'))
        # Workers can publish during serial polling. Compare copied frames with
        # a time sampled AFTER both snapshots, never the start-of-update time.
        now = time.monotonic()
        camera_live = data is not None and 0 <= now-camera_stamp < 1
        self.update_lidar_map(now)
        raw_recording_frame = data if camera_live else None
        capture_frames = camera_frames
        detection_live = camera_live and annotated is not None and 0 <= now-detection_stamp < 1
        record_stamp=camera_stamp
        if detection_live:
            data, camera_frames = annotated, detection_frame
            record_stamp=detection_stamp
        source = getattr(self, 'display_source', 'usb')
        fpga_image, fpga_stamp, fpga_live, fpga_error = self.update_fpga_monitor(now)
        view_data, view_frame, view_live = data, camera_frames, camera_live
        view_width, view_height = WIDTH, HEIGHT
        csi_data, csi_stamp, csi_frame, csi_error = (None, 0., 0, '')
        csi_reader = getattr(self, 'csi_reader', None)
        if csi_reader is not None:
            csi_data, csi_stamp, csi_frame, csi_error = csi_reader.snapshot()
        csi_live = csi_data is not None and 0 <= now-csi_stamp < .5
        if source == 'csi':
            view_data, view_frame, view_live = csi_data, ('csi', csi_frame), csi_live
        elif source == 'fpga':
            view_data = fpga_image[:, :, ::-1].tobytes() if fpga_live else None
            view_frame, view_live = ('fpga', fpga_stamp), fpga_live
            view_width, view_height = 640, 480
        if view_live and view_frame != self.last_displayed:
            pixels = GLib.Bytes.new(view_data)
            buf = GdkPixbuf.Pixbuf.new_from_bytes(pixels,GdkPixbuf.Colorspace.RGB,False,8,view_width,view_height,view_width*3)
            w,h = self.window.get_size()
            scale = min(w/view_width,h/view_height)
            self.image.set_from_pixbuf(buf.scale_simple(max(1,int(view_width*scale)),max(1,int(view_height*scale)),GdkPixbuf.InterpType.BILINEAR))
            self.last_displayed = view_frame
        elif not view_live:
            self.image.clear()  # Never display an old image as a live stream.
        live = self.link.live(now)
        t = self.link.telemetry if live else None
        follower = getattr(self, 'follower', None)
        navigator = getattr(self, 'navigator', None)
        mission = getattr(self, 'mission', None)
        mission_text=mission.step(self.link,now,objects,detection_stamp,WIDTH,
            camera_live and detection_live and not detection_error,height=HEIGHT,
            remote_allowed=(source=='usb' and not getattr(self,'semantic_dialog_open',False)
                            and not getattr(self,'control_stop_pending',False))) if mission else None
        navigation_text = navigator.step(self.link,now,remote_allowed=(source=='usb'
            and not getattr(self,'semantic_dialog_open',False)
            and not getattr(self,'control_stop_pending',False))) if navigator else None
        follow_text = None
        if follower is not None:
            try:
                self.remote_follow_step(now, t, live, objects, detection_stamp)
                self.remote_follow_speed_step(now,live)
                if not follower.enabled:
                    follower.select_best(objects,detection_stamp,now)
                # Track the exact detection frame; never use boxes on another frame.
                decision=follower.update(objects, detection_stamp, now, WIDTH, t, live)
                if decision.armed and t is not None:
                    if self.follow_was_armed and now>self.follow_feedback_deadline and t.seq!=self.follow_feedback_seq and (not t.health&64 or (t.command,t.speed)!=self.follow_expected):
                        follower.stop('remote STOP/control takeover; press remote 0 to restart')
                        decision=follower.update(objects,detection_stamp,now,WIDTH,t,live)
                    self.follow_feedback_seq=t.seq
                if not decision.armed and self.follow_was_armed:
                    self.follow_disarm_pending=True
                # DISARM once on startup/stop, never repeatedly cancel remote STOP.
                if now-self.follow_last_send>=.1:
                    if self.follow_disarm_pending:
                        self.link.send()
                        self.follow_disarm_pending=False
                    elif decision.armed:
                        self.link.send(True,decision.command,decision.speed,follow=True)
                        expected=(decision.command,decision.speed)
                        if expected!=self.follow_expected:
                            self.follow_expected=expected
                            self.follow_feedback_deadline=now+.35
                    self.follow_last_send=now
                if decision.armed and not self.follow_was_armed:
                    self.follow_feedback_deadline=now+.35
                self.follow_was_armed=decision.armed
                follow_text=f'FOLLOW {"person" if follower.category==0 else "bottle"}: {decision.reason} | 跟随档 {follower.speed}（VOL±）'
            except Exception as exc:
                self.follow_start_pending=None
                follower.stop('serial failure; press remote 0 after recovery',clear=True)
                self.follow_disarm_pending=True
                self.follow_was_armed=False
                follow_text='FOLLOW stopped: '+str(exc)
        if t is not None:
            # Sonar validity can chatter without changing control or safety.
            # Keep complete snapshots, but do not let it evict turn/fault/STOP history.
            control_state = (t.command, t.speed, t.health & 0x73, t.state, t.fault)
            if control_state != self.previous_control_state:
                event=dict(time=round(now, 3), **vars(t))
                # Retain startup targets and controller faults alongside the
                # legacy transition, even after a later DISARM clears them.
                for name,kind in (('motion',MotionTelemetry),('predictive',PredictiveTelemetry),('nav',NavTelemetry)):
                    sample=getattr(self.link, name+'_telemetry',None)
                    event[name]=vars(sample).copy() if isinstance(sample,kind) else None
                self.control_events.append(event)
                self.control_events[:] = self.control_events[-64:]
                self.previous_control_state = control_state
        def distance(v,bit): return distance_text(v, bool(t.health&bit))
        camera_name = getattr(self, 'camera_name', 'USB')
        lines = [((camera_name+'识别画面') if source == 'usb' else 'OV5647 CSI采集画面' if source=='csi' else 'FPGA DVP采集画面')+'  |  '+('串口在线' if live else '串口无数据 / 已超时'),
                 '人员、瓶子识别与跟随使用'+camera_name+'；预览切换后需明确重新启动']
        diagnostic = getattr(self.link, 'camera_telemetry', None)
        diagnostic = diagnostic if isinstance(diagnostic,CameraDiagnostic) and 0 <= now-self.link.last_camera_rx < .5 else None
        if getattr(self,'fpga_monitor_requested',False) or source=='fpga':
            lines.append('FPGA采集：'+('独立实时图像' if fpga_live else fpga_error or '等待图像 / 已超时；RK画面不替代FPGA采集'))
            if diagnostic is not None:
                lines += [diagnostic.summary, diagnostic.flow_summary]
        if source == 'csi':
            lines.append('OV5647：'+('实时图像' if csi_live else csi_error or '等待CSI/ISP图像 / 已超时'))
        if follow_text:lines += [follow_text,'自动选择置信度最高的人员 → 遥控0启动；空格/遥控5停车']
        if navigation_text:
            lines.append('路径导航：'+navigation_text)
            lines.append('遥控7切换路径寻瓶，5停车')
            nt=self.link.nav_telemetry if self.link.nav_live(now) else None
            if nt:lines.append(f'位置 X {nt.x_mm}mm  Y {nt.y_mm}mm  朝向 {nt.heading/100:.1f}°')
        if mission_text:
            hint=('遥控7直接开始路径寻瓶；5取消；重新按7重启'
                  if getattr(mission,'remote_path_search',False) else 'E开始整项任务；空格/遥控5取消；R重新准备')
            lines += ['任务6：'+mission_text,hint]
            mt=self.link.nav_telemetry if self.link.nav_live(now) else None
            if mt:lines.append(f'位置 X {mt.x_mm}mm  Y {mt.y_mm}mm  朝向 {mt.heading/100:.1f}°')
        if detection_live:
            people = sum(obj[0] == 0 for obj in objects)
            bottles = sum(obj[0] == 39 for obj in objects)
            lines.append(f'识别：人 {people}  瓶子 {bottles}  |  推理 {inference_ms:.0f} ms')
        else:
            lines.append('识别：'+(detection_error or '等待有效画面 / 推理结果已超时'))
        if not camera_live: lines.insert(0, camera_error or '摄像头无画面 / 已超时')
        lines += [f'距离  左：{distance(t.distance_l,4)}    右：{distance(t.distance_r,8)}' if t else '距离  左：--    右：--',
                  f'轮速  左：{t.velocity_l}    右：{t.velocity_r}  脉冲/20ms' if t else '轮速  --',
                  f'横滚 {t.roll/100:.2f}°   俯仰 {t.pitch/100:.2f}°   航向 {t.yaw/100:.2f}°' if t and t.health&2 else '姿态  无有效数据',
                  ('控制请求：'+{0:'停止',1:'前进',2:'后退',3:'普通左转（前进转弯）',4:'普通右转（前进转弯）',5:'定点左转（原地旋转）',6:'定点右转（原地旋转）'}.get(t.command,'未知')+f'   速度档 {t.speed}') if t else '控制请求：--',
                  f'状态码 {t.state}   制动：{"开启" if t.health&32 else "关闭"}   故障 0x{t.fault:04X}' if t else '状态  --',
                  motion_status(t, following=bool((follower and follower.enabled) or
                    (mission and mission.state in ('approach_wait','approaching','completing')))),
                  ('任务6：地点导航' if mission and mission.action=='navigate' else '任务6：搜索'+mission.target_name if mission else '路径导航模式' if navigator else '跟随控制模式' if follower else '只读显示')+'  |  遥控7路径寻瓶，5停车  |  Esc 退出']
        motion = getattr(self.link, 'motion_telemetry', None)
        motion_live = isinstance(motion, MotionTelemetry) and self.link.motion_live(now)
        if motion_live:
            mode = '编码器+IMU融合' if motion.fused else 'IMU航向（轮距未标定）'
            lines.append(f'FPGA {mode}：{motion.heading/100:.1f}°  '
                         f'目标 {motion.goal/100:.1f}°  纠偏误差 {motion.error/100:.1f}°'
                         if motion.holding else f'FPGA {mode}：{motion.heading/100:.1f}°，航向保持未介入')
            if motion.disagreement:
                lines.append('运动诊断：编码器与IMU持续不一致，请检查标定、安装和地面打滑')
        elif live and isinstance(motion, MotionTelemetry):
            lines.append('FPGA运动反馈：故障锁定，按5清除' if motion.flags & 16
                         else 'FPGA运动反馈：无有效更新 / 已超时')
            if motion.flags & 128:lines.append('运动诊断：编码器计数或推算转角跳变，检查后按5清除')
            if motion.flags & 256:lines.append('运动诊断：IMU航向异常跳变，检查安装及数据后按5清除')
        predictive=getattr(self.link,'predictive_telemetry',None)
        predictive_live=isinstance(predictive,PredictiveTelemetry) and self.link.predictive_live(now)
        if predictive_live:
            profile='已标定' if predictive.validated else '名义模型，待实测标定'
            lines.append(f'FPGA预测控制：{profile}，候选 {predictive.candidate}/8，'
                         f'时域 {predictive.horizon*20}ms，计算 {predictive.solve_cycles} 周期')
            if predictive.fault:lines.append('预测控制器异常：检查输入/计算期限，按5清除')
        lidar=getattr(self.link,'lidar_telemetry',None)
        if not isinstance(lidar,LidarTelemetry):lidar=None
        lidar_live=bool(lidar is not None and self.link.lidar_live(now))
        if lidar is not None:
            if lidar_live:
                ranges=lidar.distances()
                values=' / '.join('--' if ranges[k] is None else f'{ranges[k]/10:.0f}'
                                  for k in ('front','left','right'))
                mode='参与避障' if lidar.control_enabled else '仅观察'
                lines.append(f'激光雷达：{mode}，前/左/右 {values} cm')
            else:
                lines.append('激光雷达：未连接或无完整扫描')
        usb_bridge=getattr(self,'usb_lidar',None)
        usb_status=usb_bridge.snapshot() if usb_bridge is not None else None
        if usb_status is not None:
            state_text={'waiting_firmware':'等待支持USB雷达的新固件','waiting_device':'等待CP2102雷达',
                        'starting':'启动与数据同步中','streaming':'原始数据转发中',
                        'backpressure':'等待串口发送空间','disabled':'已停用','closed':'已关闭'}
            text=state_text.get(usb_status.get('state'),'等待数据')
            if usb_status.get('error'):text+='，'+usb_status['error']
            lines.append('USB雷达：'+text)
        elif getattr(self,'usb_lidar_error',''):
            lines.append('USB雷达：'+self.usb_lidar_error)
        lidar_planning=getattr(self.link,'lidar_planning_telemetry',None)
        if not isinstance(lidar_planning,LidarPlanningTelemetry):lidar_planning=None
        lidar_planning_live=bool(lidar_planning is not None and self.link.lidar_planning_live(now))
        lines.extend(planning_details(lidar_planning,lidar_planning_live))
        if getattr(self,'ui_notice',''):lines.append(self.ui_notice)
        recorder=getattr(self,'recorder',None)
        recording=recorder.snapshot() if recorder is not None else None
        if recording:
            presentation=recording_presentation(recording)
            lines.append(presentation['text'])
            if getattr(self,'record_button',None) is not None:
                self.record_button.set_label(presentation['button'])
                self.record_button.set_sensitive(not presentation['busy'])
            indicator=getattr(self,'record_indicator',None)
            if indicator is not None:
                indicator.set_text(presentation['text'].replace('｜','\n',1))
                context=indicator.get_style_context()
                for name in ('idle','preparing','recording','saving','saved','error'):context.remove_class(name)
                context.add_class(presentation['style'])
        if self.link.error:
            lines.append('串口：'+self.link.error)
        if not getattr(self,'show_details',False):
            lines=concise_status(camera_name=camera_name,source=source,view_live=view_live,
                telemetry=t,live=live,objects=objects,detection_live=detection_live,
                following=bool(follower and follower.enabled),navigation_text=navigation_text,
                mission_text=mission_text,motion=motion,predictive=predictive,notice=getattr(self,'ui_notice',''),
                follow_speed=follower.speed if follower else None,recording=recording,
                lidar_planning=lidar_planning,lidar_planning_live=lidar_planning_live)
        self.label.set_text('\n'.join(lines))
        self.offer_recording(data if camera_live else None,record_stamp,lines,now,
                            fallback_rgb=raw_recording_frame,fallback_captured=camera_stamp)
        if now-self.last_report > 2:
            try:
                target = ROOT/'camera-status.json'
                temporary = target.with_suffix('.tmp')
                temporary.write_text(json.dumps({'camera_frames':capture_frames,'camera_live':camera_live,
                    'telemetry_frames':self.link.frames,'telemetry_live':live,'crc_errors':self.link.crc_errors,
                    'camera_error':camera_error,'serial_error':self.link.error,
                    'display_source':'ov5647',
                    'display_details':bool(getattr(self,'show_details',False)),
                    'lidar_live':lidar_live,
                    'lidar_planning':planning_snapshot(lidar_planning,lidar_planning_live),
                    'lidar_planning_live':lidar_planning_live,
                    'usb_lidar':usb_status,
                    'lidar':None if lidar is None else {
                        'scan_id':lidar.scan_id,'flags':lidar.flags,
                        'distances_mm':lidar.distances() if lidar_live else dict.fromkeys(('front','left','right')),
                        'good_packets':lidar.good_packets,'bad_packets':lidar.bad_packets,
                        'control_enabled':lidar.control_enabled},
                    'display_text':'\n'.join(lines),
                    'fpga_video_live':fpga_live,'fpga_video_timestamp':fpga_stamp,
                    'fpga_video_error':fpga_error,
                    'fpga_monitor_enabled':bool(getattr(self,'fpga_monitor_requested',False)),
                    'fpga_complete_frames':getattr(getattr(getattr(self,'fpga_receiver',None),'assembler',None),'complete_frames',0),
                    'camera_diagnostic':vars(diagnostic) if diagnostic is not None else None,
                    'camera_kind':getattr(self, 'camera_config', CameraConfig()).primary,
                    'csi_index':getattr(self, 'camera_config', CameraConfig()).csi_index,
                    'csi_preview_live':csi_live,'csi_preview_error':csi_error,
                    'telemetry':vars(t) if t is not None else None,
                    'motion':vars(motion) if isinstance(motion,MotionTelemetry) else None,
                    'motion_live':motion_live,
                    'predictive':vars(predictive) if isinstance(predictive,PredictiveTelemetry) else None,
                    'predictive_live':predictive_live,
                    'recognition_live':detection_live,'recognition_error':detection_error,
                    'follow_enabled':bool(follower and follower.enabled),'follow_status':follow_text,
                    'follow_speed':follower.speed if follower else None,
                    'recording':recording,
                    'remote_follow_count':getattr(getattr(self.link,'nav_telemetry',None),'follow_count',None),
                    'follow_start_pending':bool(getattr(self,'follow_start_pending',None)),
                    'navigation_enabled':bool(navigator and navigator.enabled),'navigation_status':navigation_text,
                    'navigation_start_requested':bool(navigator and navigator.run_request is not None),
                    'navigation':vars(self.link.nav_telemetry) if (navigator or mission) and self.link.nav_live(now) else None,
                    'mission':mission.report if mission else None,
                    'semantic_notice':getattr(self,'ui_notice',''),
                    'semantic_dialog_open':getattr(self,'semantic_dialog_open',False),
                    'control_stop_pending':getattr(self,'control_stop_pending',False),
                    'detections':objects if detection_live else [],'inference_ms':inference_ms,
                    'control_events':self.control_events}),encoding='utf-8')
                temporary.replace(target)
            except OSError as exc:
                print(f'Cannot write camera status: {exc}', flush=True)
            self.last_report = now
        return True

    def remote_follow_speed_step(self,now,live):
        follower=getattr(self,'follower',None)
        if follower is None or not live or not self.link.nav_live(now):return
        speed=getattr(getattr(self.link,'nav_telemetry',None),'follow_speed',None)
        if (not isinstance(speed,bool) and isinstance(speed,int)
                and follower.MIN_SPEED<=speed<=follower.MAX_SPEED):
            follower.set_speed(speed)

    def remote_follow_step(self, now, telemetry, live, objects, stamp):
        trigger = getattr(self, 'remote_follow', None)
        if trigger is None:return
        nav = getattr(self.link, 'nav_telemetry', None)
        event = trigger.observe(nav, live and self.link.nav_live(now))
        follower = self.follower
        if getattr(self,'semantic_dialog_open',False):
            self.follow_start_pending=None
            follower.stop('正在输入语义任务；关闭后重新按0启动')
            return
        if event == 'stop':
            was_active = (follower.enabled or getattr(self, 'follow_was_armed', False)
                          or getattr(self, 'follow_start_pending', None) is not None)
            self.follow_start_pending = None
            follower.stop('remote STOP/link reset; press remote 0 to restart')
            if trigger.remote_stop:
                # Key5 already brakes in FPGA and prepares manual IR motion.
                # A UART DISARM here would steal that ownership and undo key5.
                self.follow_disarm_pending = False
                self.follow_was_armed = False
            elif was_active:
                self.follow_disarm_pending = True
        elif event == 'start' and not follower.enabled:
            # Consume an unsafe start rather than saving it until conditions recover.
            ready = (getattr(self, 'display_source', 'usb') == 'usb'
                     and not self.follow_disarm_pending and live and telemetry is not None
                     and telemetry.health & 0x12 == 0x12
                     and not telemetry.fault and telemetry.state == 0)
            self.link.send()  # Release manual IR ownership and stop before selecting.
            if not ready:
                self.follow_start_pending = None
                follower.stop('start rejected: recognition view/sensors/control not ready; press remote 0 again')
                return
            if not follower.select_best(objects, stamp, now):
                self.follow_start_pending = None
                return
            # The idle command heartbeat may have expired. Wait for fresh stopped
            # telemetry after DISARM rather than arming against the old snapshot.
            self.follow_start_pending = (now + .6, telemetry.seq, nav.seq)
        pending = getattr(self, 'follow_start_pending', None)
        if pending is not None:
            deadline, previous_seq, previous_nav_seq = pending
            if (now > deadline or not live or telemetry is None or telemetry.fault
                    or telemetry.health & 0x12 != 0x12 or telemetry.state != 0
                    or (telemetry.seq != previous_seq and (telemetry.health & 64 or telemetry.command != 0))
                    or getattr(self, 'display_source', 'usb') != 'usb'
                    or self.follow_disarm_pending):
                self.follow_start_pending = None
                follower.stop('start cancelled; press remote 0 again')
            elif (telemetry.seq != previous_seq and nav is not None and nav.seq != previous_nav_seq
                  and telemetry.sensors_ready and not telemetry.health & 64 and telemetry.command == 0):
                self.follow_start_pending = None
                follower.select_best(objects, stamp, now)
                follower.start(now)
                self.follow_expected = None

    def key(self, window, event):
        if event.keyval == Gdk.KEY_v:
            preview = getattr(self, 'preview_source', 'csi')
            if preview:
                self.set_source(preview if getattr(self,'display_source','usb') == 'usb' else 'usb')
            return
        if event.keyval in (Gdk.KEY_e,Gdk.KEY_r,Gdk.KEY_l) and getattr(self,'display_source','usb') != 'usb':
            return  # Autonomous controls require their recognition image to be visible.
        mission = getattr(self, 'mission', None)
        if mission:
            if event.keyval==Gdk.KEY_space:
                mission.stop('操作者取消任务')
                try:self.link.send()
                except OSError:pass
            elif event.keyval==Gdk.KEY_e:self.start_semantic_task()
            elif event.keyval==Gdk.KEY_r:
                self._stop_controllers('重置任务，已请求停车')
                mission.reset()
        navigator = getattr(self, 'navigator', None)
        if navigator:
            if event.keyval==Gdk.KEY_space:navigator.stop('操作者停车')
            elif event.keyval==Gdk.KEY_e:navigator.start(self.link,time.monotonic())
            elif event.keyval==Gdk.KEY_r:navigator.reset_pose()
            elif event.keyval==Gdk.KEY_l:navigator.upload()
        follower = getattr(self, 'follower', None)
        if follower:
            if event.keyval==Gdk.KEY_space:
                self.follow_start_pending=None
                follower.stop('operator stopped')
                self.follow_disarm_pending=True
        if event.keyval in (Gdk.KEY_Escape,Gdk.KEY_q): self.window.destroy()

    def close(self, *_):
        if self.stopping:
            return
        self.stopping = True
        if getattr(self, 'mission', None):
            self.mission.stop('closing')
            try:self.link.send()
            except OSError:pass
        if getattr(self, 'navigator', None):
            self.navigator.stop('closing')
            try:self.link.send()
            except OSError:pass
        if getattr(self, 'follower', None):
            self.follower.stop('closing')
            try:self.link.send()
            except OSError:pass
        self.stop_event.set()
        usb_bridge=getattr(self,'usb_lidar',None)
        if usb_bridge is not None:
            try:usb_bridge.close()
            except Exception as exc:print('USB雷达关闭错误：'+str(exc),flush=True)
            finally:self.usb_lidar=None
        if getattr(self, 'fpga_receiver', None) is not None:
            try:
                self.fpga_receiver.close()
            except Exception as exc:
                print('FPGA monitor cleanup failed: '+str(exc), flush=True)
            finally:
                self.fpga_receiver = None
        if getattr(self, 'csi_reader', None) is not None:
            try:
                self.csi_reader.close()
            except Exception as exc:
                print('CSI cleanup failed: '+str(exc), flush=True)
            finally:
                self.csi_reader = None
        if getattr(self,'recorder',None) is not None:
            try:self.recorder.close()
            except Exception as exc:print('录像结束错误：'+str(exc),flush=True)
        if self.timer is not None:
            GLib.source_remove(self.timer)
            self.timer = None
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(3)
        inference = getattr(self, 'inference_thread', None)
        if inference is not None and inference.is_alive():
            inference.join(3)
        if self.log is not None:
            self.log.close()
        try:
            self.link.close()
        finally:
            Gtk.main_quit()

if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--camera-config', help='camera JSON; default is camera_config.json beside this script')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--follow',choices=['person'],help='highest-confidence person; remote 0 starts, SPACE/remote 5 stops')
    modes.add_argument('--navigate',metavar='ROUTE_JSON',help='FPGA navigation UI; remote0 start/reprepare, remote5 stop; no keyboard required')
    modes.add_argument('--mission',metavar='MISSION_JSON',help='semantic navigation/person/bottle task UI; E starts, SPACE cancels')
    args=parser.parse_args()
    Viewer(None if args.follow is None else 0,args.navigate,args.mission,
           load_config(args.camera_config, missing_ok=False) if args.camera_config else None)
    Gtk.main()
