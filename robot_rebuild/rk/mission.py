"""Named navigation and person/bottle search over FPGA navigation and vision.

Creating/configuring a mission never grants motion. One explicit start permits
the bounded sequence; every failure latches until reset and another start.
"""
import json
import math
from pathlib import Path
import re

from follow import TargetFollower
from navigation import NavigationSession, navigation_failure_reason
from protocol import NAV_WRITE, RANGE_FAR, navigation_packet


class MissionSession:
    REQUEST_TIMEOUT_S = 12.
    VISION_RECOVERY_S = 2.
    STOPPED_S = .25
    SWITCH_TIMEOUT_S = 4.
    SEND_S = .1
    DISARM_S = .2
    POST_DISARM_S = .12
    FINAL_MIN_MM = 350
    FINAL_MAX_MM = 600

    def __init__(self, points, dwell_s=3., confirm_frames=5, confirm_s=.4,
                 total_timeout_s=240., approach_timeout_s=30.,
                 min_person_width=.25, approach_speed=6, action='find_person',
                 visual_stop_width=None, instruction='', location_names=None,initialize_pose=True,
                 min_bottle_height=.25,visual_stop_height=.60):
        # NavigationSession applies the protocol's integer-coordinate limits.
        if not isinstance(points, (list, tuple)):
            raise ValueError('mission points must be a list')
        points = list(points)
        if not 1 <= len(points) <= 8:
            raise ValueError('mission requires 1..8 search points')
        for index, point in enumerate(points):
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError('each search point must be [x_mm, y_mm]')
            navigation_packet(0, NAV_WRITE, index=index, x_mm=point[0], y_mm=point[1])
        for name, value, lower, upper in (
                ('dwell_s', dwell_s, .5, 30.),
                ('confirm_s', confirm_s, .2, 3.),
                ('total_timeout_s', total_timeout_s, 10., 600.),
                ('approach_timeout_s', approach_timeout_s, 2., 60.),
                ('min_person_width', min_person_width, .15, .6)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f'{name} must be finite and within {lower}..{upper}')
        if isinstance(confirm_frames, bool) or not isinstance(confirm_frames, int) or not 3 <= confirm_frames <= 30:
            raise ValueError('confirm_frames must be an integer within 3..30')
        if not confirm_s < dwell_s:
            raise ValueError('confirm_s must be shorter than dwell_s')
        if action!='navigate' and approach_timeout_s >= total_timeout_s:
            raise ValueError('approach timeout must be shorter than total timeout')
        if isinstance(approach_speed, bool) or not isinstance(approach_speed, int) or not 4 <= approach_speed <= 8:
            raise ValueError('approach_speed must be an integer within 4..8')
        if action not in ('find_person','find_bottle','navigate'):
            raise ValueError('action must be find_person, find_bottle or navigate')
        self.target_category=39 if action=='find_bottle' else 0
        self.target_name='水瓶' if self.target_category==39 else '人员'
        for name,value,lo,hi in (('min_bottle_height',min_bottle_height,.15,.6),
                                 ('visual_stop_height',visual_stop_height,.3,.8)):
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:
                raise ValueError(name+' must be a finite target image ratio')
        if visual_stop_height<min_bottle_height:
            raise ValueError('visual_stop_height must be at least min_bottle_height')
        self.min_bottle_height,self.visual_stop_height=min_bottle_height,visual_stop_height
        if not isinstance(initialize_pose,bool) and initialize_pose!='auto':
            raise ValueError('initialize_pose must be boolean or auto')
        self.initialize_pose=initialize_pose
        if visual_stop_width is None:visual_stop_width=max(.45,min_person_width)
        if (isinstance(visual_stop_width,bool) or not isinstance(visual_stop_width,(int,float))
                or not math.isfinite(visual_stop_width) or not max(.3,min_person_width)<=visual_stop_width<=.65):
            raise ValueError('visual_stop_width must be finite, at least min_person_width, within .3..65')
        if not isinstance(instruction,str) or len(instruction)>256:
            raise ValueError('instruction must be a short string')
        if location_names is not None and (not isinstance(location_names,(list,tuple))
                or len(location_names)!=len(points) or any(not isinstance(n,str) or not n for n in location_names)):
            raise ValueError('location_names must match mission points')
        self.points = [tuple(p) for p in points]
        self.action,self.instruction=action,instruction
        self.location_names=list(location_names) if location_names is not None else []
        self.visual_stop_width=visual_stop_width
        self.dwell_s, self.confirm_frames, self.confirm_s = dwell_s, confirm_frames, confirm_s
        self.total_timeout_s, self.approach_timeout_s = total_timeout_s, approach_timeout_s
        self.min_person_width, self.approach_speed = min_person_width, approach_speed
        self.reset()

    @property
    def selected_box(self):
        return self.follower.box

    @property
    def report(self):
        return dict(state=self.state, text=self.text, enabled=self.enabled,
                    point_index=self.point_index, point_count=len(self.points),
                    selected_box=list(self.selected_box) if self.selected_box else None,
                    confirmed_frames=self._confirmed_frames,
                    elapsed_s=max(0., self._now-self._started) if self._started is not None else 0.,
                    navigation_text=self.navigator.text,
                    min_person_width=self.min_person_width,
                    final_distance_mm=[self.FINAL_MIN_MM, self.FINAL_MAX_MM],
                    action=self.action,instruction=self.instruction,location_names=self.location_names,
                    target_category=self.target_category,target_name=self.target_name,
                    remote_path_search=bool(getattr(self,'remote_path_search',False)),
                    start_requested=self.run_request is not None,
                    target_size_metric='height' if self.target_category==39 else 'width',
                    completion_basis=self.completion_basis,visual_stop_width=self.visual_stop_width,
                    min_bottle_height=self.min_bottle_height,visual_stop_height=self.visual_stop_height,
                    measured_distance_mm=self.measured_distance_mm,
                    origin_policy=self.initialize_pose,pose_reset_requested=self.pose_reset_requested,
                    failure_snapshot=self.failure_snapshot,
                    vision_waiting=self._vision_missing_since is not None,
                    vision_wait_seconds=(max(0.,self._now-self._vision_missing_since)
                                         if self._vision_missing_since is not None else 0.))

    def reset(self,*,reset_origin=False):
        """Discard progress; default tasks use their current placement as origin."""
        self.run_request = None
        self.request_source = None
        self.failure_snapshot = None
        self.follower = TargetFollower(speed=self.approach_speed,category=self.target_category)
        self.point_index = 0
        origin=True if reset_origin else self.initialize_pose
        self._pose_choice_pending=origin=='auto'
        self.pose_reset_requested=origin is True
        self.navigator = NavigationSession([self.points[0]], initialize_pose=origin is True)
        self.state, self.text, self.enabled = 'preparing', '停车配置搜索路线；E明确启动任务', False
        self._started = self._approach_started = None
        self._now = 0.
        self._last_send = float('-inf')
        self._pending_disarm = True
        self._last_capture = float('-inf')
        self._camera_good = False
        self._camera_at = float('-inf')
        self._vision_missing_since = None
        self._observation_started = None
        self._confirm_started = None
        self._confirmed_frames = 0
        self._confirmed_capture = float('-inf')
        self._stop_at = self._stop_seq = None
        self._still_since = self._first_still_seq = None
        self._expected = self._feedback_seq = None
        self._feedback_until = 0.
        self._seen_armed = False
        self._stop_epoch = None
        self.completion_basis=None
        self.measured_distance_mm=None
        self._frame_height=None

    def stop(self, reason='任务已取消；R重设后按E重新启动'):
        self._halt('stopped', reason)

    def _halt(self, state, text):
        self.run_request = None
        self.enabled = False
        self.state, self.text = state, text
        self.navigator.stop(text)
        self.follower.stop(text)
        self._pending_disarm = True

    def _fail(self, text):
        restart=('；任务已取消，检查后重新按7启动'
                 if getattr(self,'remote_path_search',False) else
                 '；请重新生成任务' if self.request_source=='button' else '；任务已锁定，R重设后E重新启动')
        self._halt('failed', text+restart)

    def _fail_vision(self, text, t, n, now, captured):
        stamp=(captured if not isinstance(captured,bool) and isinstance(captured,(int,float))
               and math.isfinite(captured) else None)
        self.failure_snapshot=dict(telemetry=vars(t).copy(),navigation=vars(n).copy(),
            vision=dict(state=self.state,camera_good=self._camera_good,now=now,
                        captured=stamp,age_s=now-stamp if stamp is not None else None))
        self._fail(text)

    @staticmethod
    def _telemetry(link, now):
        return link.telemetry if link.live(now) else None

    @staticmethod
    def _navigation(link, now):
        return link.nav_telemetry if link.nav_live(now) else None

    @staticmethod
    def _measured_distance(t):
        # Invalid/stale/FAR readings do not represent target depth. A missing
        # probe never gates the task or retains a previously observed near value.
        values=[d for bit,d in ((4,t.distance_l),(8,t.distance_r))
                if t.health&bit and 0<d<RANGE_FAR]
        return min(values) if values else None

    def start(self, link, now):
        """E starts the entire sequence; no ARM is sent from this method."""
        if self.state != 'ready' or self.enabled:
            return False
        t, n = self._telemetry(link, now), self._navigation(link, now)
        if (not isinstance(now, (int, float)) or not math.isfinite(now)
                or (self.action!='navigate' and (not self._camera_good or not 0 <= now-self._camera_at <= TargetFollower.MAX_AGE))
                or t is None or n is None or not n.supported
                or not getattr(n, 'mission_supported', False)
                or not t.sensors_ready or t.fault
                or t.state != 0 or t.health & 64 or t.velocity_l or t.velocity_r):
            self.text = ('暂不能启动：需要有效识别、IMU、控制许可和任务停车事件遥测，并保持停车'
                         if self.action!='navigate' else
                         '暂不能启动：需要IMU、控制许可和任务停车事件遥测，并保持停车')
            return False
        if not self.navigator.start(link, now):
            self.text = self.navigator.text
            return False
        self.enabled = True
        if self.request_source != 'button':self.run_request = None
        self._stop_epoch = n.stop_epoch
        self.state, self.text = 'navigating', '任务启动：前往第1个目标地点'
        self._started, self._now = now, now
        return True

    def request_run(self, link, now, *, source='key7'):
        """One explicit key7/button request permits one bounded start cycle."""
        if isinstance(now,bool) or not isinstance(now,(int,float)) or not math.isfinite(now):
            self.stop('执行请求时钟无效，请重新生成任务' if source=='button' else
                      '7键搜索请求时钟无效，保持停车'); return False
        if self.enabled or self.run_request is not None:return True
        if self.state not in ('preparing','ready'):
            self.text='任务已结束或取消，请重新生成任务' if source=='button' else self.text
            return False
        t,n=self._telemetry(link,now),self._navigation(link,now)
        if source=='button' and t is None:t=self._recent_button_telemetry(link,now)
        if (t is None or n is None or not n.supported or not n.mission_supported
                or (source=='button' and (getattr(link,'connected',True) is False or getattr(link,'fd',0) is None))
                or t.health&0x12!=0x12 or t.fault):
            self.stop('执行请求未接受：等待遥测、IMU和控制许可后重新生成任务' if source=='button' else
                      '7键搜索请求未接受：检查遥测、IMU、停止许可和故障，再重新按7')
            return False
        if source=='button' and (t.state or t.command or t.health&64 or n.selected):
            self.text='当前有其他运动/控制请求，请先停车，再点执行任务6'
            return False
        self.request_source=source
        self.run_request=(now+self.REQUEST_TIMEOUT_S,now,n.stop_epoch,source=='button' and self._telemetry(link,now) is not None,n.nav_mode_count)
        self.text=('执行请求已收到：停车准备，条件齐后自动启动' if source=='button' else
                   '已收到7：停车准备路线，随后自动开始逐点搜索水瓶')
        return True

    @staticmethod
    def _recent_button_telemetry(link,now):
        # A single ordinary snapshot can be late while the STOP stream is
        # current. Never bridge an actual disconnect or an unknown baseline.
        if getattr(link,'connected',True) is False or getattr(link,'fd',0) is None:return None
        stamp=getattr(link,'last_rx',None)
        if (isinstance(stamp,bool) or not isinstance(stamp,(int,float))
                or not math.isfinite(stamp) or not 0<=now-stamp<.65):return None
        return getattr(link,'telemetry',None)

    def _advance_run_request(self, link, now, allowed, *,allow_start=False):
        if self.run_request is None:return
        deadline,requested,epoch,seen,generation=self.run_request
        if self.request_source=='button' and self.navigator.enabled:
            self.run_request=None
            return
        t,n=self._telemetry(link,now),self._navigation(link,now)
        button=self.request_source=='button'
        if button and n is not None and n.stop_epoch!=epoch:
            self.stop('遥控5停车，任务已取消；请重新生成任务')
            self._pending_disarm=self.navigator.pending_disarm=False
            return
        if button and ((t is not None and (t.state or t.command or t.health&64))
                or (n is not None and (n.selected or (generation is not None and n.nav_mode_count!=generation)))):
            self.stop('控制模式或运动请求已改变；请重新生成任务')
            self._pending_disarm=self.navigator.pending_disarm=False
            return
        if button and (getattr(link,'connected',True) is False or getattr(link,'fd',0) is None):
            self.stop('执行请求连接中断；请重新生成任务');return
        if not allowed or now>=deadline:
            self.stop('执行请求已取消/超时；请重新生成任务' if button else
                      '7键搜索请求已取消/超时；准备好后重新按7'); return
        if button and t is None:t=self._recent_button_telemetry(link,now)
        if t is None or n is None:
            # Upgrading a read-only serial owner needs its first fresh snapshot.
            if not button and not seen and 0<=now-requested<.5:return
            self.stop('执行请求遥测中断；请重新生成任务' if button else
                      '7键搜索请求遥测中断；恢复后重新按7'); return
        if button and self.state=='navigating' and (n.done or n.failed):
            self.stop('启动确认期间导航状态异常；请重新生成任务');return
        reset=generation is not None and (n.nav_mode_count is None or n.nav_mode_count<generation)
        if (reset or n.stop_epoch!=epoch or not n.supported or not n.mission_supported
                or t.health&0x12!=0x12 or t.fault or (self.state=='ready' and n.failed)):
            self.stop('执行条件失效，任务取消；请重新生成任务' if button else
                      '7键搜索请求条件失效或已按5；重新按7才可启动')
            if n.stop_epoch!=epoch:
                self._pending_disarm=self.navigator.pending_disarm=False
            return
        self.run_request=(deadline,requested,epoch,True,generation)
        if allow_start and self.state=='ready':
            if button:
                reason=('普通遥测' if self._telemetry(link,now) is None else
                        '停车心跳' if not t.health&1 else
                        '车轮停稳' if t.velocity_l or t.velocity_r else
                        '实时识别帧' if self.action!='navigate' and not self._camera_good else None)
                if reason is not None:
                    self.text='执行请求等待'+reason+'；取消可终止'
                    return
            if self.action=='navigate' or self._camera_good:self.start(link,now)

    def _send_disarm(self, link, now, force=False):
        if force or self._pending_disarm or now-self._last_send >= self.DISARM_S:
            link.send()
            self._last_send = now
            self._pending_disarm = False

    @classmethod
    def _post_disarm_feedback(cls, link, sent_at, now):
        # DISARM releases IR ownership in the FPGA. Before any later ARM, both
        # streams must report a snapshot acquired after that release; otherwise
        # an IR STOP epoch could still be in flight in the navigation frame.
        # Allow one 100 ms snapshot period plus a UART in-flight margin.
        for name in ('last_rx', 'last_nav_rx'):
            stamp = getattr(link, name, None)
            if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                    or not math.isfinite(stamp) or not sent_at+cls.POST_DISARM_S <= stamp <= now):
                return False
        return True

    def _begin_wait(self, link, now, state, text):
        # Record before DISARM; a cached stopped snapshot cannot authorize ARM.
        t = self._telemetry(link, now)
        self._stop_at, self._stop_seq = now, t.seq if t else None
        self._still_since = self._first_still_seq = None
        self.navigator.stop(text)
        self.follower.stop(text)
        self.state, self.text = state, text
        self._send_disarm(link, now, force=True)

    def _stopped(self, link, t, now):
        if t.health & 64 or t.command != 0 or t.state != 0 or t.velocity_l or t.velocity_r:
            self._still_since = self._first_still_seq = None
            return False
        if t.seq == self._stop_seq:
            return False
        # Confirm release of motion ownership at this transition. Periodic
        # DISARM heartbeats must not keep moving the feedback deadline forward.
        if not self._post_disarm_feedback(link, self._stop_at, now):
            return False
        if self._still_since is None:
            self._still_since, self._first_still_seq = now, t.seq
        return now-self._still_since >= self.STOPPED_S and t.seq != self._first_still_seq

    def _targets(self,objects):
        people = []
        try:
            for category, score, box in objects:
                if category == self.target_category and math.isfinite(score) and score >= .45 and TargetFollower.valid_box(box):
                    people.append((category, score, tuple(box)))
        except (TypeError, ValueError, OverflowError):
            raise ValueError('invalid detection data')
        return people

    def _visual(self, people, captured, now, width):
        if len(people) != 1:
            self._fail('出现多个'+self.target_name+'，目标不明确' if people else '指定'+self.target_name+'丢失')
            return False
        if not self.follower.track(people, captured, now):
            self._fail('指定'+self.target_name+'丢失或关联不明确')
            return False
        box = self.follower.box
        if box[0] < 0 or box[2] > width or (self._frame_height is not None and (box[1]<0 or box[3]>self._frame_height)):
            self._fail(self.target_name+'框超出图像范围')
            return False
        return True

    def _arrival_view(self, width, minimum=None):
        box = self.follower.box
        center = (box[0]+box[2]) / (2*width)
        if self.target_category==39:
            if self._frame_height is None:return False
            size=(box[3]-box[1])/self._frame_height
            minimum=self.min_bottle_height if minimum is None else minimum
        else:
            size=(box[2]-box[0])/width
            minimum=self.min_person_width if minimum is None else minimum
        return .42 <= center <= .58 and size>=minimum

    def _arrival_basis(self, distance, width):
        if distance is not None:
            return 'range_and_vision' if distance<=self.FINAL_MAX_MM and self._arrival_view(width) else None
        threshold=self.visual_stop_height if self.target_category==39 else self.visual_stop_width
        return 'vision_only' if self._arrival_view(width,threshold) else None

    def _next_point(self):
        self.point_index+=1
        self.navigator=NavigationSession([self.points[self.point_index]],initialize_pose=False)
        self.state,self.text='next_preparing','停车配置下一个目标地点；保留原起点'

    def _restart_button_prepare(self):
        # Keep the original request deadline; replace only the unarmed
        # navigation confirmation so all final feedback follows a new DISARM.
        self.navigator.start_at=None
        self.navigator.start_sent_at=None
        self.navigator.start_deadline=None
        self.navigator.start_baseline=None
        self.navigator.pending_disarm=True
        self.enabled=False
        self.state='ready'

    def step(self, link, now, objects, captured, width, camera_live, *,height=None,remote_allowed=True):
        """Tick one owner of the same serial link; no camera/serial reader inside."""
        try:
            if isinstance(now,bool) or not isinstance(now,(int,float)) or not math.isfinite(now) or now<self._now:
                raise ValueError('invalid monotonic clock')
            self._advance_run_request(link,now,remote_allowed)
            if (self.request_source=='button' and self.run_request is not None
                    and self._telemetry(link,now) is None
                    and self._recent_button_telemetry(link,now) is not None):
                self._now=now
                if self.navigator.start_at is not None:self._restart_button_prepare()
                self._send_disarm(link,now)
                self.text='执行请求等待普通遥测；取消可终止'
            else:self._step(link, now, objects, captured, width, camera_live,height=height)
            self._advance_run_request(link,now,remote_allowed,allow_start=True)
        except (OSError, ValueError, TypeError, OverflowError) as exc:
            self._fail('任务通信/输入错误：'+str(exc))
        if self._pending_disarm:
            try:
                self._send_disarm(link, now)
            except (OSError, ValueError):
                # Leave pending until transport recovers; never auto ARM.
                self._pending_disarm = True
        return self.text

    def _step(self, link, now, objects, captured, width, camera_live, *,height=None):
        if not isinstance(now, (int, float)) or not math.isfinite(now):
            raise ValueError('invalid monotonic clock')
        if now < self._now:
            raise ValueError('monotonic clock moved backwards')
        self._now = now
        height_good=(not isinstance(height,bool) and isinstance(height,(int,float)) and math.isfinite(height) and height>0)
        self._frame_height=height if height_good else None
        self._camera_good = (bool(camera_live) and isinstance(captured, (int, float))
                             and math.isfinite(captured) and 0 <= now-captured <= TargetFollower.MAX_AGE
                             and isinstance(width, (int, float)) and math.isfinite(width) and width > 0
                             and (self.target_category!=39 or height_good))
        if self._camera_good:
            self._camera_at = captured
        if self.state in ('complete', 'failed', 'stopped'):
            return
        if (self.request_source=='button' and self.run_request is not None
                and self.state=='navigating' and self.navigator.start_at is not None):
            t=self._telemetry(link,now)
            waiting=(t is not None and (not t.health&1 or t.velocity_l or t.velocity_r
                     or (self.action!='navigate' and not self._camera_good)))
            if waiting:
                if abs(t.velocity_l)<=1 and abs(t.velocity_r)<=1:
                    self._restart_button_prepare()
                else:
                    self.stop('启动确认期间出现运动，请重新生成任务');return
        if self.state in ('preparing', 'ready'):
            if self.request_source=='button' and self.run_request is not None:self._send_disarm(link,now)
            if self._pose_choice_pending:
                n=self._navigation(link,now)
                if n is None:
                    self.text='等待导航坐标系状态；不会自动启动运动'
                    return
                self.pose_reset_requested=not n.pose_ready
                self.navigator=NavigationSession([self.points[0]],initialize_pose=self.pose_reset_requested)
                self._pose_choice_pending=False
            self.navigator.step(link, now)
            self._pending_disarm = self.navigator.pending_disarm
            if self.navigator.prepared:
                self.state, self.text = 'ready', ('停车等待启动条件；取消可终止' if self.run_request is not None else
                    '路线已准备：点执行任务6启动，取消可终止')
            else:
                self.state, self.text = 'preparing', self.navigator.text
            if self.run_request is not None and self.action!='navigate' and not self._camera_good:
                # The explicit key7 request already has a fixed 12s deadline.
                # Prepare only while disarmed; a late recognition frame must
                # neither cancel that request nor authorize ARM. STOP/fault/
                # link/deadline checks run before this branch on every tick.
                self._send_disarm(link, now)
                self.text=('执行请求等待实时识别帧；取消可终止' if self.request_source=='button' else
                           '已收到7：保持停车，等待实时识别画面；超时后需重新按7')
            return
        t, n = self._telemetry(link, now), self._navigation(link, now)
        if t is None or n is None or not n.supported or not getattr(n, 'mission_supported', False):
            self._fail('串口/导航遥测超时或固件不支持导航'); return
        if n.stop_epoch != self._stop_epoch:
            self._fail('遥控5停车事件，任务已取消')
            # FPGA already braked and prepared manual IR ownership. Do not
            # undo key5 by sending a later UART DISARM from this task.
            self._pending_disarm=False
            self.navigator.pending_disarm=False
            return
        if t.fault or not t.sensors_ready or n.failed:
            self.failure_snapshot = dict(telemetry=vars(t).copy(), navigation=vars(n).copy())
            self._fail(navigation_failure_reason(t,n)); return
        if now-self._started >= self.total_timeout_s:
            self._fail('搜索任务总超时'); return
        if self.action!='navigate':
            if self._camera_good:
                if captured < self._last_capture:
                    self._fail_vision('摄像头/识别时间戳倒退',t,n,now,captured);return
                if self._vision_missing_since is not None:
                    gap=now-self._vision_missing_since
                    if gap>=self.VISION_RECOVERY_S:
                        self._fail_vision('摄像头/识别持续超时，已停车',t,n,now,captured);return
                    # Missing frames do not consume the stationary observation
                    # interval or contribute to a target confirmation streak.
                    if self.state=='observing' and self._observation_started is not None:
                        self._observation_started+=gap
                    self._vision_missing_since=None
                self._last_capture = captured
            else:
                # FPGA waypoint motion has its own current pose, range and
                # STOP checks. A single late vision frame must not cancel it.
                # Visual approach still requires fresh frames on every tick.
                if self.state in ('approach_wait','approaching','completing'):
                    self._fail_vision('接近目标时摄像头/识别超时，已停车',t,n,now,captured);return
                if self._vision_missing_since is None:self._vision_missing_since=now
                if now-self._vision_missing_since>=self.VISION_RECOVERY_S:
                    self._fail_vision('摄像头/识别持续超时，已停车',t,n,now,captured);return
        if self.state == 'navigating':
            if not self._camera_good and self.action!='navigate' and not self.navigator.enabled:
                self._send_disarm(link,now)
                self.text='等待实时画面恢复后启动路径导航'
                return
            # The shared navigation session owns DISARM, fresh feedback and
            # STOP-generation checks for both standalone and mission launches.
            self.navigator.step(link, now)
            if n.done:
                self._begin_wait(link, now, 'settling', '到达搜索点；等待停稳后观察')
            elif not self.navigator.enabled and self.navigator.start_at is None:
                self._fail('导航已停止：'+self.navigator.text)
            else:
                self.text = f'搜索点 {self.point_index+1}/{len(self.points)}：'+self.navigator.text
                if self._vision_missing_since is not None:self.text+='；视觉短暂延迟，等待恢复'
            return
        if self.state == 'next_preparing':
            self.navigator.step(link, now)
            if self.navigator.prepared:
                if self.action!='navigate' and not self._camera_good:
                    self._send_disarm(link,now)
                    self.text='路线已准备，停车等待实时画面恢复'
                    return
                if not self.navigator.start(link, now):
                    self._fail('下一搜索点启动条件失效'); return
                self.state, self.text = 'navigating', '前往下一个搜索点'
            elif not self.navigator.queue and not self.navigator.waiting:
                self._fail('下一搜索点配置失败：'+self.navigator.text)
            else:
                self.text = '停车配置下一个搜索点：'+self.navigator.text
            return
        if t.state != 0:
            self._fail('停车/接近阶段出现其他运动状态'); return
        people = self._targets(objects) if self.action!='navigate' else []
        if self.state == 'settling':
            if self._stopped(link, t, now):
                if self.action=='navigate':
                    if self.point_index+1<len(self.points):self._next_point()
                    else:
                        self.completion_basis='waypoint_and_stopped'
                        self._halt('complete','语义导航完成：已到达目标地点并停稳')
                    return
                self.state, self.text = 'observing', '停稳观察；只确认画面中唯一的'+self.target_name
                self._observation_started = now
                self._confirm_started = None
                self._confirmed_frames = 0
                self.follower.stop('观察'+self.target_name, clear=True)
            elif now-self._stop_at >= self.SWITCH_TIMEOUT_S:
                self._fail('到点后未能确认停车')
            else:
                self._send_disarm(link, now)
            return
        if self.state == 'observing':
            self._send_disarm(link, now)
            if t.health & 64 or t.velocity_l or t.velocity_r:
                self._fail('观察期间出现外部运动请求'); return
            if not self._camera_good:
                self.follower.stop('等待实时画面恢复',clear=True)
                self._confirmed_frames,self._confirm_started=0,None
                self._confirmed_capture=float('-inf')
                self.text='停车观察：视觉短暂延迟，等待恢复'
                return
            if len(people) > 1:
                self._fail('观察到多个'+self.target_name+'，不能自动指定目标'); return
            if captured != self._confirmed_capture:
                self._confirmed_capture = captured
                if not people:
                    self.follower.stop('未发现'+self.target_name, clear=True)
                    self._confirmed_frames, self._confirm_started = 0, None
                elif self.follower.box is None or not self.follower.track(people, captured, now):
                    box = people[0][2]
                    self.follower.select(people, (box[0]+box[2])/2, (box[1]+box[3])/2, captured, now)
                    self._confirmed_frames, self._confirm_started = 1, captured
                else:
                    self._confirmed_frames += 1
            if (self._confirmed_frames >= self.confirm_frames and self._confirm_started is not None
                    and captured-self._confirm_started >= self.confirm_s):
                self._approach_started = now
                self._begin_wait(link, now, 'approach_wait', self.target_name+'已确认；停车确认后低速接近')
            elif now-self._observation_started >= self.dwell_s:
                if self.point_index+1 >= len(self.points):
                    self._fail('所有搜索点巡查完成，未确认唯一'+self.target_name)
                else:
                    self._next_point()
            else:
                self.text = f'观察搜索点 {self.point_index+1}：{self.target_name}确认 {self._confirmed_frames}/{self.confirm_frames} 帧'
            return
        if self.state in ('approach_wait', 'approaching', 'completing'):
            if not self._visual(people, captured, now, width): return
            if now-self._approach_started >= self.approach_timeout_s:
                self._fail(self.target_name+'接近超时'); return
            distance = self._measured_distance(t)
            self.measured_distance_mm=distance
            basis=self._arrival_basis(distance,width)
            if distance is not None and distance < self.FINAL_MIN_MM:
                self._fail('前方距离小于35厘米，停止接近'); return
            if n.selected:
                self._fail('接近阶段出现导航控制权'); return
            if self.state == 'approach_wait':
                if self._stopped(link, t, now):
                    if distance is not None and distance <= self.FINAL_MAX_MM and not basis:
                        self._fail('前方已近，但'+self.target_name+'未居中或视觉尺寸不足'); return
                    if basis:
                        self.completion_basis=basis
                        self._begin_wait(link, now, 'completing', '已接近'+self.target_name+'；等待停车确认')
                    elif self.follower.start(now):
                        self.state, self.text = 'approaching', '低速接近已确认'+self.target_name
                        self._expected = self._feedback_seq = None
                        self._seen_armed = False
                        decision = self.follower.update(people, captured, now, width, t, True)
                        if not decision.armed or decision.command == 0:
                            self._fail('视觉接近启动条件失效'); return
                        link.send(True, decision.command, decision.speed,follow=True)
                        self._last_send, self._feedback_until = now, now+.35
                        self._expected, self._feedback_seq = (decision.command, decision.speed), t.seq
                    else:
                        self._fail(self.target_name+'选择已失效')
                elif now-self._stop_at >= self.SWITCH_TIMEOUT_S:
                    self._fail('切换接近前未能确认停车')
                else:
                    self._send_disarm(link, now)
                return
            if self.state == 'completing':
                if basis is None:
                    if distance is None and self._arrival_view(width):
                        self.completion_basis=None
                        self._begin_wait(link,now,'approach_wait','测距失效；确认停稳后转为视觉接近')
                        return
                    self._fail('停车后'+self.target_name+'/距离条件失效'); return
                if self._stopped(link, t, now) and captured > self._stop_at:
                    self.completion_basis=basis
                    self._halt('complete', '任务完成：找到'+self.target_name+'并停稳（'+('有效测距及视觉确认' if basis=='range_and_vision' else '仅视觉确认，无有效测距')+'）')
                elif now-self._stop_at >= self.SWITCH_TIMEOUT_S:
                    self._fail('接近后未能确认停车')
                else:
                    self._send_disarm(link, now)
                return
            if self._expected is not None and t.seq != self._feedback_seq:
                self._feedback_seq = t.seq
                if t.health & 64:
                    self._seen_armed = True
                elif self._seen_armed or now >= self._feedback_until:
                    self._fail('遥控停车/控制权切换，任务取消'); return
                if now >= self._feedback_until and (t.command, t.speed) != self._expected:
                    self._fail('电机请求未被接受或已被其他控制覆盖'); return
            if distance is not None and distance <= self.FINAL_MAX_MM and not basis:
                self._fail('前方已近，但'+self.target_name+'未居中或视觉尺寸不足'); return
            if basis:
                self.completion_basis=basis
                self._begin_wait(link, now, 'completing', '已接近'+self.target_name+'；等待停车确认')
                return
            decision = self.follower.update(people, captured, now, width, t, True)
            if not decision.armed or decision.command == 0:
                self._fail('视觉接近已停止：'+decision.reason); return
            if now-self._last_send >= self.SEND_S:
                link.send(True, decision.command, decision.speed,follow=True)
                expected = (decision.command, decision.speed)
                if expected != self._expected:
                    self._expected, self._feedback_until = expected, now+.35
                self._last_send = now
            self.text = ('低速接近'+self.target_name+'：有效测距及视觉确认后停车' if distance is not None
                         else '低速接近'+self.target_name+'：无有效测距，依据居中及视觉尺寸确认停车')


def parse_semantic_instruction(instruction, locations):
    """Resolve bounded Chinese navigation/search wording to configured locations.

    Text never executes code or grants motion. No guessed coordinates or
    substring matching: every requested location must match a configured name.
    """
    if not isinstance(instruction,str) or not 1<=len(instruction)<=256:
        raise ValueError('指令长度须为1至256个字符')
    if not isinstance(locations,dict) or not 1<=len(locations)<=8:
        raise ValueError('请配置1至8个地点及毫米坐标')
    name_pattern=r'[\u3400-\u9fffA-Za-z0-9_-]{1,24}'
    for index,(name,point) in enumerate(locations.items()):
        if (not isinstance(name,str) or re.fullmatch(name_pattern,name) is None
                or any(word in name for word in ('找人','找瓶','找水瓶','寻找','依次','停车','停下'))):
            raise ValueError('地点名称包含歧义或任务语法')
        if not isinstance(point,(list,tuple)) or len(point)!=2:
            raise ValueError('地点必须使用[x_mm,y_mm]坐标')
        navigation_packet(0,NAV_WRITE,index=index,x_mm=point[0],y_mm=point[1])
    text=instruction.strip()
    if any(ch in text for ch in ('\n','\r','\x00')):
        raise ValueError('请输入单行中文导航指令')
    text=re.sub(r'[ \t]+','',text)
    if text.endswith(('。','!','！')):text=text[:-1]
    if text.startswith('请'):text=text[1:]
    action='navigate'
    for suffix,target_action in (('寻找人员','find_person'),('寻找人','find_person'),('找人','find_person'),
                                 ('寻找水瓶','find_bottle'),('寻找瓶子','find_bottle'),('找水瓶','find_bottle'),('找瓶子','find_bottle')):
        if text.endswith(suffix):
            text=text[:-len(suffix)];action=target_action;break
    if action=='navigate':
        for suffix in ('停下','停车'):
            if text.endswith(suffix):text=text[:-len(suffix)];break
    sequential=False
    for prefix in ('依次前往','依次导航到','依次去','依次到','导航到','移动到','前往','去','到'):
        if text.startswith(prefix):
            text=text[len(prefix):];sequential=prefix.startswith('依次');break
    else:
        raise ValueError('例如：去前方停车、去左边找人、去右边找水瓶、依次去前方、左前方找人')
    names=text.replace('，','、').replace(',','、').split('、')
    if (not 1<=len(names)<=8 or (sequential and len(names)<2)
            or (not sequential and len(names)!=1)
            or any(re.fullmatch(name_pattern,n) is None for n in names)):
        raise ValueError('多个地点请使用“依次去地点1、地点2”，地点不能为空')
    if len(set(names))!=len(names):raise ValueError('同一地点不可在一个任务中重复')
    unknown=[n for n in names if n not in locations]
    if unknown:raise ValueError('未知地点：'+'、'.join(unknown))
    return dict(action=action,points=[tuple(locations[n]) for n in names],
                location_names=names,instruction=instruction.strip())


def parse_instruction(instruction, locations):
    """Compatibility API returning the points from a validated semantic plan."""
    return parse_semantic_instruction(instruction,locations)['points']


def _unique_json_object(pairs):
    data = {}
    for key, value in pairs:
        if key in data:
            raise ValueError('duplicate JSON fields or locations are ambiguous')
        data[key] = value
    return data


def load_mission(path, *, instruction=None, include_semantics=False):
    """Load coordinates or named navigation/search wording; never start motion."""
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'), object_pairs_hook=_unique_json_object)
    allowed = {'units', 'points', 'instruction', 'locations', 'dwell_s', 'confirm_frames', 'confirm_s',
               'total_timeout_s', 'approach_timeout_s', 'min_person_width', 'approach_speed',
               'visual_stop_width','action','initialize_pose','min_bottle_height','visual_stop_height'}
    if not isinstance(data, dict) or data.get('units') != 'mm' or set(data)-allowed:
        raise ValueError('mission requires units="mm" and known configuration fields only')
    if instruction is not None:
        if 'locations' not in data or 'points' in data:
            raise ValueError('现场指令需要具名地点配置，不能使用裸坐标路线')
        data['instruction']=instruction
        data.pop('action',None)  # An explicit new instruction selects its action.
    semantic = 'instruction' in data or 'locations' in data
    if semantic and 'points' in data:
        raise ValueError('choose explicit points or instruction/locations, not both')
    config = {key: value for key, value in data.items() if key not in ('units', 'instruction', 'locations')}
    if semantic:
        if 'instruction' not in data or 'locations' not in data:
            raise ValueError('semantic missions require both instruction and locations')
        plan=parse_semantic_instruction(data['instruction'],data['locations'])
        if 'action' in config and config['action']!=plan['action']:
            raise ValueError('action与中文指令的任务类型冲突')
        config['points']=plan['points']
        config.setdefault('initialize_pose',True)
        if plan['action']!='find_person' or 'action' in config:config['action']=plan['action']
        if include_semantics:
            config.update(instruction=plan['instruction'],location_names=plan['location_names'])
    elif 'points' not in data or not isinstance(data['points'], list):
        raise ValueError('mission requires points=[[x_mm,y_mm],...]')
    MissionSession(**config)  # Validate all limits before the UI enables writing.
    return config


if __name__=='__main__':
    import argparse
    import sys
    if hasattr(sys.stdout,'reconfigure') and not sys.stdout.isatty():
        sys.stdout.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser(description='离线预览语义任务，不连接串口、不启动运动')
    parser.add_argument('config',nargs='?',default=str(Path(__file__).with_name('mission_semantic_example.json')))
    parser.add_argument('--instruction',help='覆盖配置中的中文指令，仅预览，不修改配置文件')
    args=parser.parse_args()
    try:
        config=load_mission(args.config,instruction=args.instruction,include_semantics=True)
    except (OSError,ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps({'starts_motion':False,'units':'mm','task':config},ensure_ascii=False,indent=2))
