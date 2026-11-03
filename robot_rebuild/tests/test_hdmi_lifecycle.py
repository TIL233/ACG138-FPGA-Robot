"""GTK-independent tests of the actual HDMI viewer's worker/cleanup paths."""
import importlib.util
from pathlib import Path
import sys
import threading
import json
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rk'))


class HdmiTests(unittest.TestCase):
    def setUp(self):
        self.gtk, self.glib = mock.Mock(), mock.Mock()
        gi = SimpleNamespace(require_version=mock.Mock())
        repository = SimpleNamespace(Gtk=self.gtk, Gdk=mock.Mock(), GdkPixbuf=mock.Mock(), GLib=self.glib)
        spec = importlib.util.spec_from_file_location(
            'hdmi_under_test', Path(__file__).resolve().parents[2] / 'ch341_driver_work/hdmi_camera.py')
        self.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {'gi': gi, 'gi.repository': repository}):
            spec.loader.exec_module(self.module)
        # CSI discovery is hardware I/O. Routing itself is tested separately;
        # worker lifecycle tests must never probe this Windows host's devices.
        capture_patch=mock.patch.object(self.module,'capture_command',return_value=['OV5647-test-pipeline'])
        self.capture_command=capture_patch.start();self.addCleanup(capture_patch.stop)

    def viewer(self):
        viewer = self.module.Viewer.__new__(self.module.Viewer)
        viewer.stopping = False
        viewer.stop_event = threading.Event()
        viewer.timer = None
        viewer.thread = None
        viewer.log = mock.Mock()
        viewer.link = mock.Mock(telemetry=None, error='')
        viewer.link.live.return_value = False
        viewer.camera_lock = threading.Lock()
        viewer.camera_latest = (None, 0, 0, '')
        viewer.camera_frames = 0
        viewer.last_displayed = None
        viewer.process = None
        viewer.image = mock.Mock()
        viewer.label = mock.Mock()
        viewer.last_report = float('inf')
        viewer.control_events = []
        viewer.previous_control_state = None
        # Existing lifecycle assertions exercise the optional full diagnostics.
        viewer.show_details = True
        return viewer

    def follow_viewer(self):
        from follow import TargetFollower,RemoteFollowTrigger
        v=self.viewer();v.follower=TargetFollower();v.remote_follow=RemoteFollowTrigger()
        v.follow_disarm_pending=False;v.follow_expected=None;v.display_source='usb'
        v.follow_was_armed=False;v.follow_start_pending=None
        v.link.nav_live.return_value=True
        return v

    def test_follow_speed_uses_only_fresh_valid_fpga_preset_and_never_starts(self):
        v=self.follow_viewer()
        v.link.nav_telemetry=SimpleNamespace(follow_speed=12)
        v.remote_follow_speed_step(10,True)
        self.assertEqual(v.follower.speed,12)
        self.assertFalse(v.follower.enabled);v.link.send.assert_not_called()
        for bad in (None,True,0,3,13,64,8.5):
            v.link.nav_telemetry.follow_speed=bad
            v.remote_follow_speed_step(10.1,True)
            self.assertEqual(v.follower.speed,12)
        v.link.nav_telemetry=SimpleNamespace(follow_speed=4)
        v.link.nav_live.return_value=False
        v.remote_follow_speed_step(10.2,True);self.assertEqual(v.follower.speed,12)
        v.link.nav_live.return_value=True
        v.remote_follow_speed_step(10.3,False);self.assertEqual(v.follower.speed,12)
        v.link.nav_telemetry=SimpleNamespace()  # Legacy v1/v2/v3 still work.
        v.remote_follow_speed_step(10.4,True);self.assertEqual(v.follower.speed,12)
        v.link.nav_telemetry=SimpleNamespace(follow_speed=4)
        v.follower.enabled=True;v.follow_expected=(1,12)
        v.remote_follow_speed_step(10.5,True)
        self.assertEqual(v.follower.speed,4);self.assertTrue(v.follower.enabled)
        self.assertEqual(v.follow_expected,(1,12));v.link.send.assert_not_called()

    def test_compact_follow_hold_shows_selected_preset_instead_of_stop_speed(self):
        t=SimpleNamespace(command=0,speed=0,health=0x7f,state=0,fault=0,
            velocity_l=0,velocity_r=0,distance_l=500,distance_r=1000,
            roll=0,pitch=0,yaw=0)
        lines=self.module.concise_status(camera_name='USB',source='usb',view_live=True,
            telemetry=t,live=True,objects=[],detection_live=True,following=True,follow_speed=12)
        self.assertIn('跟随档 12',lines[1]);self.assertIn('已停车',lines[1])
        self.assertTrue(any('≤60cm' in line for line in lines))

    def test_compact_distinguishes_latched_yaw_fault_from_missing_imu(self):
        from dataclasses import fields
        motion = self.module.MotionTelemetry(**{
            f.name: (0x718 if f.name == 'flags' else 0)
            for f in fields(self.module.MotionTelemetry)})
        t=SimpleNamespace(command=0,speed=0,health=0x3d,state=0,fault=0,
            velocity_l=0,velocity_r=0,distance_l=500,distance_r=1000,
            roll=51,pitch=-43,yaw=-17804)
        lines=self.module.concise_status(camera_name='OV5647',source='csi',view_live=True,
            telemetry=t,live=True,objects=[],detection_live=True,motion=motion)
        self.assertIn('航向异常锁定',lines[3])
        self.assertIn('按5',lines[3])
        self.assertNotIn('等待有效数据',lines[3])
        # A stale last fault must not be presented when the whole link is lost.
        offline=self.module.concise_status(camera_name='OV5647',source='csi',view_live=True,
            telemetry=t,live=False,objects=[],detection_live=True,motion=motion)
        self.assertNotIn('航向异常锁定','\n'.join(offline))

    def test_control_unready_does_not_claim_imu_has_no_data(self):
        t=SimpleNamespace(health=0x3d,state=0,fault=0,distance_l=500,distance_r=1000)
        self.assertNotIn('IMU无有效数据',self.module.motion_status(t))
        self.assertIn('姿态反馈',self.module.motion_status(t))

    def test_remote0_starts_only_after_baseline_and_fresh_person(self):
        v=self.follow_viewer();t=SimpleNamespace(sensors_ready=True,fault=0,state=0,health=0x1f,seq=1,command=0)
        objects=[(0,.9,(100,100,400,600))]
        v.link.nav_telemetry=SimpleNamespace(follow_count=3,stop_epoch=0,seq=1)
        v.remote_follow_step(10,t,True,objects,9.9)
        self.assertFalse(v.follower.enabled)
        v.link.nav_telemetry.follow_count=4
        v.remote_follow_step(10,t,True,objects,9.9)
        self.assertFalse(v.follower.enabled)
        t.seq=2;v.link.nav_telemetry.seq=2
        v.remote_follow_step(10.1,t,True,objects,10)
        self.assertTrue(v.follower.enabled);v.link.send.assert_called_once_with()
        v.link.nav_telemetry.stop_epoch=1
        v.remote_follow_step(10,t,True,objects,9.9)
        self.assertFalse(v.follower.enabled);self.assertFalse(v.follow_disarm_pending)

    def test_remote5_preserves_manual_control_while_idle_and_after_follow(self):
        for active in (False,True):
            with self.subTest(active=active):
                v=self.follow_viewer();v.follower.enabled=active;v.follow_was_armed=active
                t=SimpleNamespace(sensors_ready=True,fault=0,state=0,health=0x3f,seq=1,command=0)
                v.link.nav_telemetry=SimpleNamespace(follow_count=2,stop_epoch=0,seq=1)
                v.remote_follow_step(10,t,True,[],9.9)
                v.follow_start_pending=(11,1,1)
                v.link.nav_telemetry.stop_epoch=1
                v.remote_follow_step(10.1,t,True,[],10)
                self.assertFalse(v.follower.enabled)
                self.assertFalse(v.follow_was_armed)
                self.assertFalse(v.follow_disarm_pending)
                self.assertIsNone(v.follow_start_pending)
                v.link.send.assert_not_called()
                # Subsequent idle updates must not schedule a delayed UART stop.
                v.remote_follow_step(10.2,t,True,[],10.1)
                self.assertFalse(v.follow_disarm_pending)

    def test_link_loss_disarms_active_follow_but_does_not_steal_idle_manual_control(self):
        for active in (False,True):
            with self.subTest(active=active):
                v=self.follow_viewer();v.follower.enabled=active
                t=SimpleNamespace(sensors_ready=True,fault=0,state=0,health=0x3f,seq=1,command=0)
                v.link.nav_telemetry=SimpleNamespace(follow_count=2,stop_epoch=0,seq=1)
                v.remote_follow_step(10,t,True,[],9.9)
                v.link.nav_live.return_value=False
                v.remote_follow_step(10.1,t,True,[],10)
                self.assertFalse(v.follower.enabled)
                self.assertEqual(v.follow_disarm_pending,active)

    def test_viewer_update_does_not_undo_remote5_or_subsequent_manual_forward(self):
        from protocol import Telemetry
        for active in (False,True):
            with self.subTest(active=active):
                v=self.follow_viewer();objects=[(0,.9,(100,100,400,600))]
                v.follower.select_best(objects,9.9,10)
                if active:v.follower.start(10)
                v.follow_was_armed=active;v.follow_last_send=9
                v.follow_feedback_deadline=11;v.follow_feedback_seq=1
                v.recognition=(None,10,1,objects,75,'')
                v.link.live.return_value=True
                v.link.telemetry=Telemetry(2,0,0x3f,0,16,1000,1000,0,0,0,0,0,0)
                v.link.nav_telemetry=SimpleNamespace(follow_count=2,stop_epoch=0,seq=1)
                v.remote_follow_step(10,v.link.telemetry,True,objects,9.9)
                v.link.nav_telemetry.stop_epoch=1
                with mock.patch.object(self.module.time,'monotonic',return_value=10.1):
                    v.update()
                self.assertFalse(v.follower.enabled)
                v.link.send.assert_not_called()
                # New manual forward feedback must remain owned by the remote.
                v.link.telemetry=Telemetry(3,0,0x5f,1,16,1000,1000,0,0,0,0,0,0)
                with mock.patch.object(self.module.time,'monotonic',return_value=10.2):
                    v.update()
                v.link.send.assert_not_called()

    def test_unsafe_remote_start_consumed_without_delayed_arming(self):
        for problem in ('sensors','preview','pending','no_person','stale'):
            with self.subTest(problem=problem):
                v=self.follow_viewer();t=SimpleNamespace(sensors_ready=problem!='sensors',fault=0,state=0,health=0x1d if problem=='sensors' else 0x1f,seq=1,command=0)
                objects=[] if problem=='no_person' else [(0,.9,(100,100,400,600))]
                stamp=0 if problem=='stale' else 9.9
                v.display_source='csi' if problem=='preview' else 'usb'
                v.follow_disarm_pending=problem=='pending'
                v.link.nav_telemetry=SimpleNamespace(follow_count=0,stop_epoch=0,seq=1)
                v.remote_follow_step(10,t,True,objects,stamp)
                v.link.nav_telemetry.follow_count=1
                v.remote_follow_step(10,t,True,objects,stamp)
                self.assertFalse(v.follower.enabled)
                v.display_source='usb';v.follow_disarm_pending=False;t.sensors_ready=True;t.health=0x1f;t.seq=2
                v.remote_follow_step(10,t,True,[(0,.9,(100,100,400,600))],9.9)
                self.assertFalse(v.follower.enabled)

    def test_remote0_idle_heartbeat_and_ack_timeout(self):
        v=self.follow_viewer();t=SimpleNamespace(sensors_ready=False,fault=0,state=0,health=0x1e,seq=1,command=0)
        objects=[(0,.9,(100,100,400,600))]
        v.link.nav_telemetry=SimpleNamespace(follow_count=0,stop_epoch=0,seq=1)
        v.remote_follow_step(10,t,True,objects,9.9)
        v.link.nav_telemetry.follow_count=1
        v.remote_follow_step(10,t,True,objects,9.9)
        self.assertFalse(v.follower.enabled)
        t.health=0x1f;t.sensors_ready=True;t.seq=2;v.link.nav_telemetry.seq=2
        v.remote_follow_step(10.1,t,True,objects,10)
        self.assertTrue(v.follower.enabled)
        v.follower.stop();v.link.nav_telemetry.follow_count=2
        v.remote_follow_step(11,t,True,objects,10.9)
        t.seq=3
        v.remote_follow_step(11.7,t,True,objects,11.6)
        self.assertFalse(v.follower.enabled)

    def test_remote0_can_start_without_sonar_but_still_waits_for_stopped_ack(self):
        from protocol import Telemetry
        v=self.follow_viewer();objects=[(0,.9,(100,100,400,600))]
        t=Telemetry(1,0,0x12,0,0,65535,65535,0,0,0,0,0,0)
        v.link.nav_telemetry=SimpleNamespace(follow_count=0,stop_epoch=0,seq=1)
        v.remote_follow_step(10,t,True,objects,9.9)
        v.link.nav_telemetry.follow_count=1
        v.remote_follow_step(10,t,True,objects,9.9)
        self.assertFalse(v.follower.enabled)
        t=Telemetry(2,0,0x13,0,0,65535,65535,0,0,0,0,0,0)
        v.link.nav_telemetry.seq=2
        v.remote_follow_step(10.1,t,True,objects,10)
        self.assertTrue(v.follower.enabled)

    def test_missing_sonar_is_displayed_without_claiming_a_motion_restriction(self):
        from protocol import Telemetry
        t=Telemetry(1,0,0x13,0,0,65535,65535,0,0,0,0,0,0)
        self.assertTrue(t.sensors_ready)
        text=self.module.motion_status(t)
        self.assertIn('不阻止运动',text)
        self.assertNotIn('限制：',text)

    def test_semantic_task_replaces_active_follow_only_after_validating_without_arm(self):
        with tempfile.TemporaryDirectory() as directory:
            v=self.follow_viewer();v.semantic_path=Path(directory)/'map.json'
            v.semantic_path.write_text(json.dumps({'units':'mm','locations':{'门口':[1000,0]},'instruction':'去门口找人'}),encoding='utf8')
            old=v.follower;old.enabled=True
            with self.assertRaises(ValueError):v.prepare_semantic_task('去未知地点')
            self.assertIs(v.follower,old);self.assertTrue(old.enabled)
            v.link.send.assert_not_called()
            mission=v.prepare_semantic_task('请前往门口')
            self.assertFalse(old.enabled);self.assertIsNone(v.follower)
            self.assertIsNone(v.navigator);self.assertIsNone(v.remote_follow)
            self.assertIs(v.mission,mission);self.assertFalse(mission.enabled)
            self.assertEqual(mission.action,'navigate');self.assertEqual(mission.location_names,['门口'])
            self.assertTrue(mission.navigator.queue)
            v.link.send.assert_called_once_with()
            v.link.configure_navigation.assert_not_called()
            self.assertIs(mission.initialize_pose,True)
            self.assertTrue(mission.pose_reset_requested)
            self.assertIn(3,[opcode for opcode,_ in mission.navigator.queue])
            # Returning to following cancels the task and requires a new key0.
            v.activate_follow_mode()
            self.assertIsNone(v.mission);self.assertIsNone(v.navigator)
            self.assertFalse(v.follower.enabled)
            self.assertIsNone(v.remote_follow.previous)
            self.assertTrue(all(not call.args for call in v.link.send.call_args_list))

    def test_semantic_dialog_generates_without_starting_and_blocks_hidden_key0(self):
        with tempfile.TemporaryDirectory() as directory:
            v=self.follow_viewer();v.window=mock.Mock();v.semantic_path=Path(directory)/'map.json'
            v.semantic_path.write_text(json.dumps({'units':'mm','locations':{'门口':[1000,0]},'instruction':'去门口找人'}),encoding='utf8')
            dialog=self.gtk.Dialog.return_value
            self.gtk.Entry.return_value.get_text.return_value='去门口找人'
            def run_dialog():
                t=SimpleNamespace(sensors_ready=True,fault=0,state=0,health=0x1f,seq=1,command=0)
                v.link.nav_telemetry=SimpleNamespace(follow_count=0,stop_epoch=0,seq=1)
                v.remote_follow_step(10,t,True,[(0,.9,(100,100,400,600))],9.9)
                v.link.nav_telemetry.follow_count=1
                v.remote_follow_step(10.1,t,True,[(0,.9,(100,100,400,600))],10)
                self.assertFalse(v.follower.enabled);self.assertIsNone(v.follow_start_pending)
                return self.gtk.ResponseType.OK
            dialog.run.side_effect=run_dialog
            v.show_semantic_dialog()
            self.assertFalse(v.semantic_dialog_open)
            self.assertIsNotNone(v.mission);self.assertFalse(v.mission.enabled)
            self.assertTrue(all(not call.args for call in v.link.send.call_args_list))
            dialog.destroy.assert_called_once()

    def test_cancelled_semantic_input_does_not_replay_key0_received_in_dialog(self):
        with tempfile.TemporaryDirectory() as directory:
            v=self.follow_viewer();v.window=mock.Mock();v.semantic_path=Path(directory)/'map.json'
            v.semantic_path.write_text(json.dumps({'units':'mm','locations':{'门口':[1000,0]},'instruction':'去门口找人'}),encoding='utf8')
            t=SimpleNamespace(sensors_ready=True,fault=0,state=0,health=0x1f,seq=1,command=0)
            objects=[(0,.9,(100,100,400,600))]
            v.link.nav_telemetry=SimpleNamespace(follow_count=3,stop_epoch=0,seq=1)
            def cancel_dialog():
                v.remote_follow_step(10,t,True,objects,9.9)
                v.link.nav_telemetry.follow_count=4
                v.remote_follow_step(10.1,t,True,objects,10)
                return self.gtk.ResponseType.CANCEL
            self.gtk.Dialog.return_value.run.side_effect=cancel_dialog
            v.show_semantic_dialog()
            self.assertIsNone(v.remote_follow.previous)
            self.assertIsNone(getattr(v,'mission',None))
            v.remote_follow_step(10.2,t,True,objects,10.1)
            self.assertFalse(v.follower.enabled);self.assertIsNone(v.follow_start_pending)
            self.assertTrue(all(not call.args for call in v.link.send.call_args_list))

    def test_failed_toolbar_stop_is_retried_without_any_motion(self):
        v=self.follow_viewer();v.follow_last_send=9
        v.link.send.side_effect=OSError('offline')
        v.stop_controls();self.assertTrue(v.control_stop_pending)
        v.link.send.side_effect=None
        with mock.patch.object(self.module.time,'monotonic',return_value=10):v.update()
        self.assertFalse(v.control_stop_pending)
        self.assertTrue(all(not call.args for call in v.link.send.call_args_list))

    def test_real_task_button_preparing_navigate_runs_without_camera(self):
        from test_mission import SimulatedLink
        from mission import MissionSession
        from dataclasses import replace
        v=self.follow_viewer();v.link=SimulatedLink()
        v.mission=MissionSession([(1000,0)],action='navigate')
        v.link.telemetry=replace(v.link.telemetry,health=0x13,distance_l=65535,distance_r=65535)
        with mock.patch.object(self.module.time,'monotonic',return_value=0.):
            self.assertTrue(v.start_semantic_task())
            self.assertTrue(v.start_semantic_task())
        for tick in range(1,30):
            now=tick*.1;v.link.fresh(now)
            v.mission.step(v.link,now,(),None,640,False,height=None)
        self.assertTrue(v.mission.navigator.enabled,v.mission.text)
        self.assertEqual(v.mission.point_index,0)
        self.assertIsNone(v.mission.run_request)

    def test_task_button_uses_bounded_mission_request_and_clears_snapshot_notice(self):
        v=self.follow_viewer();v.mission=mock.Mock()
        v.mission.request_run.return_value=True
        v.ui_notice='stale notice'
        with mock.patch.object(self.module.time,'monotonic',return_value=10):
            self.assertTrue(v.start_semantic_task())
        v.mission.request_run.assert_called_once_with(v.link,10,source='button')
        v.mission.start.assert_not_called()
        self.assertEqual(v.ui_notice,'')

    def test_task_start_button_does_not_start_while_input_or_stop_is_pending(self):
        v=self.follow_viewer();v.mission=mock.Mock()
        v.semantic_dialog_open=True
        self.assertFalse(v.start_semantic_task())
        v.semantic_dialog_open=False;v.control_stop_pending=True
        self.assertFalse(v.start_semantic_task());v.mission.start.assert_not_called()

    def test_visual_follow_sends_mode_flag_even_without_sonar(self):
        from protocol import Telemetry
        v=self.follow_viewer();objects=[(0,.9,(500,100,780,600))]
        v.follower.select_best(objects,10,10);v.follower.start(10)
        v.follow_last_send=9;v.follow_feedback_deadline=11;v.follow_feedback_seq=1
        v.recognition=(None,10,1,objects,75,'')
        v.link.live.return_value=True
        v.link.telemetry=Telemetry(2,0,0x13,0,0,65535,65535,0,0,0,0,0,0)
        v.link.nav_telemetry=SimpleNamespace(follow_count=0,stop_epoch=0,seq=1)
        with mock.patch.object(self.module.time,'monotonic',return_value=10.1):v.update()
        v.link.send.assert_called_once_with(True,1,8,follow=True)

    def test_E_no_longer_starts_person_following(self):
        v=self.follow_viewer()
        v.key(None,SimpleNamespace(keyval=self.module.Gdk.KEY_e))
        self.assertFalse(v.follower.enabled)
        v.link.send.assert_not_called()

    def test_pending_start_waits_for_new_STOP_snapshot_and_STOP_wins(self):
        v=self.follow_viewer();t=SimpleNamespace(sensors_ready=True,fault=0,state=0,health=0x1f,seq=1,command=0)
        objects=[(0,.9,(100,100,400,600))]
        v.link.nav_telemetry=SimpleNamespace(follow_count=0,stop_epoch=0,seq=1)
        v.remote_follow_step(10,t,True,objects,9.9)
        v.link.nav_telemetry.follow_count=1
        v.remote_follow_step(10,t,True,objects,9.9)
        t.seq=2
        v.remote_follow_step(10.1,t,True,objects,10)
        self.assertFalse(v.follower.enabled)  # New regular telemetry alone is insufficient.
        v.link.nav_telemetry.seq=2;v.link.nav_telemetry.stop_epoch=1
        v.remote_follow_step(10.2,t,True,objects,10.1)
        self.assertFalse(v.follower.enabled);self.assertIsNone(v.follow_start_pending)

    def test_remote_timeout_is_not_reported_as_serial_loss(self):
        t = SimpleNamespace(health=0x3E, fault=1, distance_l=125, distance_r=39)
        text = self.module.motion_status(t)
        self.assertIn('控制未准备', text)
        self.assertIn('先按5', text)
        self.assertIn('距离<20cm', text)
        self.assertNotIn('IMU无有效数据', text)

    def test_no_echo_display_does_not_claim_a_numeric_measured_distance(self):
        self.assertIn('按远距',self.module.distance_text(65534,True))
        self.assertEqual(self.module.distance_text(65534,False),'无有效数据')
        self.assertEqual(self.module.distance_text(65535,True),'无有效数据')
        self.assertEqual(self.module.distance_text(150,True),'15.0 cm')

    def test_missing_sensor_and_encoder_fault_are_explained(self):
        t = SimpleNamespace(health=0x39, fault=0x22, distance_l=65535, distance_r=300)
        text = self.module.motion_status(t)
        self.assertIn('姿态反馈不可用', text)
        self.assertIn('左测距无有效数据', text)
        self.assertIn('编码器/电机反馈故障', text)
        self.assertNotIn('距离<20cm', text)

    def test_recovered_sensor_does_not_hide_latched_fault(self):
        t = SimpleNamespace(health=0x7F, fault=2, distance_l=800, distance_r=800)
        text = self.module.motion_status(t)
        self.assertIn('故障已锁定', text)
        self.assertNotIn('点按持续运动', text)

    def test_close_before_camera_started_and_repeated_close_are_safe(self):
        viewer = self.viewer()
        viewer.close()
        viewer.close()
        viewer.log.close.assert_called_once()
        viewer.link.close.assert_called_once()
        self.gtk.main_quit.assert_called_once()

    def test_usb_lidar_worker_stops_before_serial_close_without_motion_request(self):
        viewer=self.viewer();events=[]
        viewer.usb_lidar=mock.Mock()
        viewer.usb_lidar.close.side_effect=lambda:events.append('usb')
        viewer.link.close.side_effect=lambda:events.append('serial')
        viewer.close()
        self.assertEqual(events,['usb','serial'])
        viewer.link.send.assert_not_called()

    def test_usb_lidar_details_distinguish_firmware_wait_from_valid_ranges(self):
        viewer=self.viewer();viewer.usb_lidar=mock.Mock()
        viewer.usb_lidar.snapshot.return_value={'state':'waiting_firmware','error':'','device':''}
        viewer.update()
        self.assertIn('USB雷达：等待支持USB雷达的新固件',viewer.label.set_text.call_args.args[0])
        viewer.link.send.assert_not_called()

    def test_serial_close_error_still_exits_gtk(self):
        viewer = self.viewer()
        viewer.link.close.side_effect = OSError('USB unplugged')
        with self.assertRaises(OSError):
            viewer.close()
        self.gtk.main_quit.assert_called_once()

    def test_stale_camera_clears_picture(self):
        viewer = self.viewer()
        viewer.camera_latest = (b'old picture', 0, 1, '')
        self.assertTrue(viewer.update())
        viewer.image.clear.assert_called_once()
        self.assertIn('已超时', viewer.label.set_text.call_args.args[0])
        viewer.link.poll.assert_called_once()

    def test_control_history_keeps_wheel_targets_after_fault_clear(self):
        from dataclasses import replace
        from protocol import Telemetry,MotionTelemetry,NavTelemetry
        viewer=self.viewer();viewer.link.live.return_value=True
        viewer.link.telemetry=Telemetry(1,7,0x73,1,12,900,900,0,0,0,0,0,32)
        motion=MotionTelemetry(1,0x4b,1000,100,120,120,0,0,-120,0,0,12,12,0,0,100,100)
        viewer.link.motion_telemetry=motion
        viewer.link.nav_telemetry=NavTelemetry(1,7,0,1,0xcc,0,0,0,0,0)
        viewer.update()
        event=viewer.control_events[-1]
        self.assertEqual(event['motion']['target_l'],12)
        self.assertEqual(event['nav']['state'],7)
        viewer.link.motion_telemetry=replace(motion,target_l=0,target_r=0)
        viewer.link.telemetry=replace(viewer.link.telemetry,seq=2,fault=0,state=0,command=0)
        viewer.update()
        self.assertEqual(event['fault'],32)
        self.assertEqual(event['motion']['target_l'],12)
        self.assertEqual(viewer.control_events[-1]['motion']['target_l'],0)

    def test_sonar_validity_chatter_preserves_turn_fault_stop_history(self):
        from protocol import Telemetry
        viewer=self.viewer();viewer.link.live.return_value=True
        stages=[(2,1,0x53,0,-8,8),(7,1,0x73,0x400,0,0),(0,0,0x33,0,0,0)]
        snapshots=[]
        with mock.patch.object(self.module.time,'monotonic') as clock:
            for stage,(state,command,health,fault,vl,vr) in enumerate(stages):
                for sample in range(200):
                    # More transitions than the64-entry ring can retain if
                    # raw sonar validity is incorrectly treated as control.
                    sonar_validity=(sample+3)%4
                    t=Telemetry(sample%256,state,health|(sonar_validity<<2),command,16,
                        100+sample,200+sample,vl,vr,1,2,1000+sample,fault)
                    viewer.link.telemetry=t;clock.return_value=10+stage+sample*.001
                    self.assertTrue(viewer.update())
                    if sample==0:snapshots.append(vars(t))
        self.assertEqual(len(viewer.control_events),3)
        self.assertEqual([event['state'] for event in viewer.control_events],[2,7,0])
        self.assertEqual([event['fault'] for event in viewer.control_events],[0,0x400,0])
        # The deduplication mask must not strip real health/distance/yaw from
        # the retained evidence, including both valid-sonar bits in this sample.
        for event,expected in zip(viewer.control_events,snapshots):
            self.assertEqual({key:event[key] for key in expected},expected)
        viewer.link.send.assert_not_called()

    def test_control_history_still_records_safety_arm_and_command_changes(self):
        from dataclasses import replace
        from protocol import Telemetry
        viewer=self.viewer();viewer.link.live.return_value=True
        t=Telemetry(0,0,0x13,0,16,1000,1000,0,0,0,0,0,0)
        samples=[t]
        # Link, IMU, stop permission, brake and ARM changes remain evidence.
        for bit in (0x01,0x02,0x10,0x20,0x40):
            t=replace(t,health=t.health^bit);samples.append(t)
        for change in (dict(command=1),dict(speed=18),dict(state=2),dict(fault=0x400)):
            t=replace(t,**change);samples.append(t)
        with mock.patch.object(self.module.time,'monotonic') as clock:
            for index,t in enumerate(samples):
                viewer.link.telemetry=t;clock.return_value=10+index*.1
                self.assertTrue(viewer.update())
        self.assertEqual(len(viewer.control_events),len(samples))
        self.assertEqual([event['health'] for event in viewer.control_events],
                         [t.health for t in samples])
        self.assertEqual(viewer.control_events[-1]['fault'],0x400)
        self.assertEqual(viewer.control_events[-1]['command'],1)
        viewer.link.send.assert_not_called()

    def test_motion_diagnostics_are_read_only_and_calibration_is_truthful(self):
        from dataclasses import replace
        from protocol import MotionTelemetry
        viewer=self.viewer()
        motion=MotionTelemetry(1,0x4b,1000,100,120,120,0,0,-120,8,8,8,8,0,0,100,100)
        viewer.link.motion_telemetry=motion
        viewer.link.motion_live.return_value=True
        viewer.update()
        self.assertIn('轮距未标定',viewer.label.set_text.call_args.args[0])
        self.assertIn('纠偏误差 -1.2°',viewer.label.set_text.call_args.args[0])
        viewer.link.motion_telemetry=replace(motion,flags=0x2f)
        viewer.update()
        self.assertIn('编码器+IMU融合',viewer.label.set_text.call_args.args[0])
        self.assertIn('持续不一致',viewer.label.set_text.call_args.args[0])
        viewer.link.motion_live.return_value=False
        viewer.link.live.return_value=True
        viewer.link.motion_telemetry=replace(motion,flags=0x190)
        viewer.update()
        text=viewer.label.set_text.call_args.args[0]
        self.assertIn('编码器计数或推算转角跳变',text)
        self.assertIn('IMU航向异常跳变',text)
        self.assertNotIn('纠偏误差',text)
        viewer.link.send.assert_not_called()

    def test_offline_details_do_not_present_old_motion_fault_as_current(self):
        from dataclasses import fields
        viewer=self.viewer()
        viewer.link.motion_live.return_value=False
        viewer.link.motion_telemetry=self.module.MotionTelemetry(**{
            f.name: (0x718 if f.name=='flags' else 0)
            for f in fields(self.module.MotionTelemetry)})
        viewer.update()
        text=viewer.label.set_text.call_args.args[0]
        self.assertNotIn('故障锁定',text)
        self.assertNotIn('IMU航向异常跳变',text)
        viewer.link.send.assert_not_called()

    def test_partial_camera_pipe_reads_are_assembled_before_publication(self):
        viewer = self.viewer()
        viewer.camera_config=self.module.CameraConfig(primary='usb',csi_index=1,rotation=180)
        viewer.recorder=mock.Mock()
        process = mock.Mock()
        process.poll.return_value = 0
        process.stdout.fileno.return_value = 17
        chunks = [b'ab', b'cdef']

        def read(fd, count):
            chunk = chunks.pop(0)
            if not chunks:
                viewer.stop_event.set()
            return chunk

        with mock.patch.object(self.module, 'WIDTH', 2), mock.patch.object(self.module, 'HEIGHT', 1), \
             mock.patch.object(self.module.subprocess, 'Popen', return_value=process), \
             mock.patch.object(self.module.select, 'select', return_value=([17], [], [])), \
             mock.patch.object(self.module.os, 'read', side_effect=read):
            viewer.read_camera()
        self.assertEqual(viewer.camera_frames, 1)
        self.assertEqual(viewer.camera_latest[0], b'abcdef')
        config=self.capture_command.call_args.args[0]
        self.assertEqual((config.primary,config.csi_index,config.rotation),('ov5647',1,180))
        # Publication waits for the UI to select the matching recognition frame
        # and compute the displayed telemetry text.
        viewer.recorder.offer.assert_not_called()
        process.stdout.close.assert_called_once()

    def test_recognition_recovers_after_transient_inference_failure(self):
        import numpy  # Load before patch.dict restores the module inventory.
        viewer = self.viewer()
        viewer.camera_latest = (b'abcdef', 10., 1, '')
        viewer.recognition = (b'old', 9., 0, [(0,.9,(0,0,1,1))], 1., '')
        detector = mock.Mock()
        def detect(_):
            if detector.detect.call_count == 1:
                raise RuntimeError('temporary inference failure')
            viewer.stop_event.set()
            return []
        detector.detect.side_effect = detect
        def wait(_):
            self.assertEqual(viewer.recognition[3], [])
            self.assertIn('temporary inference failure', viewer.recognition[5])
            return False
        with mock.patch.dict(sys.modules, {'detector':SimpleNamespace(Detector=mock.Mock(return_value=detector))}), \
             mock.patch.object(self.module, 'WIDTH', 2), mock.patch.object(self.module, 'HEIGHT', 1), \
             mock.patch.object(self.module.time, 'monotonic', return_value=10.), \
             mock.patch.object(viewer.stop_event, 'wait', side_effect=wait):
            viewer.recognize()
        self.assertEqual(detector.detect.call_count, 2)
        self.assertEqual(viewer.recognition[2], 1)
        self.assertEqual(viewer.recognition[5], '')
        viewer.link.send.assert_not_called()

    def test_recognition_retry_can_be_cancelled_on_close(self):
        import numpy
        viewer = self.viewer()
        factory = mock.Mock(side_effect=RuntimeError('model unavailable'))
        with mock.patch.dict(sys.modules, {'detector':SimpleNamespace(Detector=factory)}), \
             mock.patch.object(viewer.stop_event, 'wait', return_value=True) as wait:
            viewer.recognize()
        wait.assert_called_once_with(1.)
        factory.assert_called_once()
        viewer.link.send.assert_not_called()

    def test_recording_uses_primary_boxed_frame_timestamp_and_displayed_text_when_preview_switches(self):
        for details in (False,True):
            with self.subTest(details=details):
                viewer=self.viewer();viewer.show_details=details
                viewer.camera_name='OV5647';viewer.display_source='csi'
                viewer.camera_latest=(b'primary raw',9.9,9,'')
                viewer.recognition=(b'primary boxes',9.8,8,[(0,.9,(1,2,3,4))],75,'')
                viewer.csi_reader=mock.Mock()
                viewer.csi_reader.snapshot.return_value=(b'alternate preview',9.95,3,'')
                viewer.window=mock.Mock();viewer.window.get_size.return_value=(1920,1080)
                viewer.recorder=mock.Mock()
                viewer.recorder.snapshot.return_value=dict(state='recording',active=True,path='/usb/test.avi',segments=1,error='')
                patch=object();renderer=mock.Mock();renderer.render.return_value=patch
                with mock.patch.object(self.module,'TextOverlayRenderer',return_value=renderer), \
                     mock.patch.object(self.module.time,'monotonic',return_value=10):
                    self.assertTrue(viewer.update())
                self.glib.Bytes.new.assert_called_with(b'alternate preview')
                renderer.render.assert_called_once_with(viewer.label.set_text.call_args.args[0])
                viewer.recorder.offer.assert_called_once_with(b'primary boxes',9.8,patch,
                    fallback_rgb=b'primary raw',fallback_captured=9.9)
                viewer.recorder.fail.assert_not_called();viewer.link.send.assert_not_called()

    def test_recording_falls_back_to_fresh_raw_frame_with_its_timestamp(self):
        viewer=self.viewer();viewer.last_displayed=0
        viewer.camera_latest=(b'fresh primary raw',11.9,10,'')
        viewer.recognition=(b'old boxes',9.8,8,[(0,.9,(1,2,3,4))],75,'')
        viewer.window=mock.Mock();viewer.window.get_size.return_value=(1920,1080)
        viewer.recorder=mock.Mock()
        viewer.recorder.snapshot.return_value=dict(state='recording',active=True,path='/usb/test.avi',segments=1,error='')
        patch=object();renderer=mock.Mock();renderer.render.return_value=patch
        with mock.patch.object(self.module,'TextOverlayRenderer',return_value=renderer), \
             mock.patch.object(self.module.time,'monotonic',return_value=12):
            viewer.update()
        viewer.recorder.offer.assert_called_once_with(b'fresh primary raw',11.9,patch,
            fallback_rgb=b'fresh primary raw',fallback_captured=11.9)
        self.assertIn('推理结果已超时',renderer.render.call_args.args[0])
        viewer.link.send.assert_not_called()

    def test_capture_arriving_during_poll_is_not_reported_as_disconnected(self):
        viewer=self.viewer();viewer.last_displayed=0
        viewer.window=mock.Mock();viewer.window.get_size.return_value=(1920,1080)
        viewer.recorder=mock.Mock()
        viewer.recorder.snapshot.return_value=dict(state='recording',active=True,error='')
        renderer=mock.Mock();patch=renderer.render.return_value
        with mock.patch.object(self.module.time,'monotonic',return_value=10.) as clock, \
             mock.patch.object(self.module,'TextOverlayRenderer',return_value=renderer):
            def poll(_):
                # Capture/inference workers publish while serial work runs.
                viewer.camera_latest=(b'new raw',10.01,10,'')
                viewer.recognition=(b'new boxes',10.005,9,[],10.,'')
                clock.return_value=10.02
            viewer.link.poll.side_effect=poll
            viewer.update()
        viewer.recorder.offer.assert_called_once_with(b'new boxes',10.005,patch,
            fallback_rgb=b'new raw',fallback_captured=10.01)
        self.assertNotIn('摄像头无画面',viewer.label.set_text.call_args.args[0])

    def test_recording_rejects_stale_primary_even_with_live_preview_and_recent_boxes(self):
        viewer=self.viewer();viewer.display_source='csi'
        viewer.camera_latest=(b'stale primary',8.9,9,'')
        viewer.recognition=(b'recent boxes',9.9,8,[(0,.9,(1,2,3,4))],75,'')
        viewer.csi_reader=mock.Mock()
        viewer.csi_reader.snapshot.return_value=(b'live preview',9.95,3,'')
        viewer.window=mock.Mock();viewer.window.get_size.return_value=(1920,1080)
        viewer.recorder=mock.Mock()
        viewer.recorder.snapshot.return_value=dict(state='recording',active=True,path='/usb/test.avi',segments=1,error='')
        with mock.patch.object(self.module,'TextOverlayRenderer') as renderer, \
             mock.patch.object(self.module.time,'monotonic',return_value=10):
            viewer.update()
        self.glib.Bytes.new.assert_called_with(b'live preview')
        self.assertEqual(viewer.recorder.offer.call_args.args[:2],(None,0.))
        renderer.assert_not_called();viewer.link.send.assert_not_called()

    def test_recording_text_render_is_bounded_and_frame_loss_is_published_immediately(self):
        viewer=self.viewer();viewer.recorder=mock.Mock()
        viewer.recorder.snapshot.return_value=dict(state='recording',active=True)
        patch=object();renderer=mock.Mock();renderer.render.return_value=patch
        with mock.patch.object(self.module,'TextOverlayRenderer',return_value=renderer):
            viewer.offer_recording(b'frame one',9.9,['距离 左 20cm'],10)
            viewer.offer_recording(b'frame two',9.91,['距离 左 21cm'],10.01)
            viewer.offer_recording(b'frame three',10.01,['距离 左 22cm'],10.11)
            self.assertEqual(renderer.render.call_count,2)
            self.assertEqual(viewer.recorder.offer.call_count,2)
            viewer.offer_recording(None,0.,['摄像头无画面'],10.12)
        self.assertEqual(viewer.recorder.offer.call_args.args[:2],(None,0.))
        self.assertEqual(renderer.render.call_count,2);viewer.link.send.assert_not_called()

    def test_recording_render_failure_reports_error_without_interrupting_control(self):
        viewer=self.viewer();viewer.recorder=mock.Mock()
        viewer.recorder.snapshot.return_value=dict(state='recording',active=True)
        renderer=mock.Mock();renderer.render.side_effect=RuntimeError('CJK font rendering failed')
        with mock.patch.object(self.module,'TextOverlayRenderer',return_value=renderer):
            viewer.offer_recording(b'primary frame',9.9,['距离 左 20cm'],10)
        self.assertEqual(viewer.recorder.offer.call_args.args[:2],(None,0.))
        self.assertIn('CJK font rendering failed',str(viewer.recorder.fail.call_args.args[0]))
        viewer.link.send.assert_not_called()

    def test_remote9_only_toggles_recording_and_never_sends_motor_commands(self):
        from video_recording import RemoteRecordTrigger
        viewer=self.viewer();viewer.recorder=mock.Mock();viewer.remote_record=RemoteRecordTrigger()
        viewer.link.nav_live.return_value=True
        viewer.link.nav_telemetry=SimpleNamespace(record_count=4)
        viewer.remote_record_step(10);viewer.recorder.toggle.assert_not_called()
        viewer.link.nav_telemetry.record_count=5
        viewer.remote_record_step(10.1);viewer.remote_record_step(10.2)
        viewer.recorder.toggle.assert_called_once_with();viewer.link.send.assert_not_called()
        viewer.link.nav_live.return_value=False
        viewer.link.nav_telemetry.record_count=6;viewer.remote_record_step(10.3)
        viewer.link.nav_live.return_value=True;viewer.remote_record_step(10.4)
        viewer.recorder.toggle.assert_called_once_with()

    def test_recorder_cleanup_failure_still_closes_control_link(self):
        viewer=self.viewer();viewer.recorder=mock.Mock()
        viewer.recorder.close.side_effect=TimeoutError('write blocked')
        with mock.patch('builtins.print'):viewer.close()
        viewer.link.close.assert_called_once();self.gtk.main_quit.assert_called_once()

    def test_recognition_displays_the_matching_frame_and_expires(self):
        viewer = self.viewer()
        viewer.last_displayed = 0
        viewer.window = mock.Mock()
        viewer.window.get_size.return_value = (1920,1080)
        viewer.camera_latest = (b'new camera', 9.9, 9, '')
        viewer.recognition = (b'boxed frame', 9.8, 8, [(0,.9,(1,2,3,4))],75,'')
        with mock.patch.object(self.module.time,'monotonic',return_value=10):
            viewer.update()
        self.glib.Bytes.new.assert_called_with(b'boxed frame')
        self.assertIn('人 1', viewer.label.set_text.call_args.args[0])
        viewer.camera_latest = (b'fresh camera',11.9,10,'')
        with mock.patch.object(self.module.time,'monotonic',return_value=12):
            viewer.update()
        self.glib.Bytes.new.assert_called_with(b'fresh camera')
        self.assertIn('推理结果已超时',viewer.label.set_text.call_args.args[0])

    def test_remote_stop_disarms_follow_once_without_auto_restart(self):
        from follow import TargetFollower
        from protocol import Telemetry
        viewer=self.viewer()
        viewer.follower=TargetFollower()
        objects=[(0,.9,(400,100,700,650))]
        viewer.follower.select(objects,500,200,10,10)
        viewer.follower.start(10)
        viewer.camera_latest=(None,0,0,'')
        viewer.recognition=(None,10,1,objects,75,'')
        viewer.follow_was_armed=True
        viewer.follow_disarm_pending=False
        viewer.follow_last_send=9
        viewer.follow_feedback_deadline=9
        viewer.follow_feedback_seq=1
        viewer.follow_expected=(1,8)
        viewer.link.live.return_value=True
        viewer.link.telemetry=Telemetry(2,0,0x3f,0,0,1000,1000,0,0,0,0,0,0)
        with mock.patch.object(self.module.time,'monotonic',return_value=10):
            viewer.update()
        self.assertFalse(viewer.follower.enabled)
        viewer.link.send.assert_called_once_with()
        viewer.recognition=(None,10.1,2,objects,75,'')
        with mock.patch.object(self.module.time,'monotonic',return_value=10.1):
            viewer.update()
        viewer.link.send.assert_called_once_with()

    def test_mission_uses_the_same_detection_frame_and_no_second_control_owner(self):
        viewer=self.viewer()
        viewer.last_displayed=7
        viewer.camera_latest=(b'raw',10,8,'')
        objects=[(0,.9,(100,100,400,700))]
        viewer.recognition=(b'annotated',9.9,7,objects,75,'')
        viewer.mission=mock.Mock()
        viewer.mission.action='find_person';viewer.mission.target_name='人员'
        viewer.mission.step.return_value='正在巡查'
        viewer.mission.report={'state':'navigating','enabled':True}
        viewer.follower=viewer.navigator=None
        viewer.link.nav_live.return_value=False
        with mock.patch.object(self.module.time,'monotonic',return_value=10):
            self.assertTrue(viewer.update())
        viewer.mission.step.assert_called_once_with(viewer.link,10,objects,9.9,1280,True,height=720,remote_allowed=True)
        viewer.link.send.assert_not_called()
        self.assertIn('任务6：正在巡查',viewer.label.set_text.call_args.args[0])

    def test_navigation_remote0_runs_through_actual_viewer_without_keyboard(self):
        from dataclasses import replace
        from navigation import NavigationSession
        from protocol import NavTelemetry,Telemetry
        viewer=self.viewer()
        viewer.navigator=NavigationSession([(500,0)],remote_enabled=True)
        navigator=viewer.navigator
        navigator.queue.clear();navigator.pending_disarm=False
        navigator.route_confirmed=navigator.prepared=True
        viewer.link.live.return_value=viewer.link.nav_live.return_value=True
        viewer.link.telemetry=Telemetry(1,0,0x32,0,0,65535,65535,0,0,0,0,0,0)
        viewer.link.nav_telemetry=NavTelemetry(1,0,0,1,128|64|32|8,0,0,0,0,0,follow_count=3)
        for now,seq,count in ((10.,1,3),(10.1,1,4),(10.3,2,4),(10.6,3,4)):
            viewer.link.telemetry=replace(viewer.link.telemetry,seq=seq,health=0x32 if seq==1 else 0x33)
            viewer.link.nav_telemetry=replace(viewer.link.nav_telemetry,seq=seq,follow_count=count)
            viewer.link.last_rx=viewer.link.last_nav_rx=now
            with mock.patch.object(self.module.time,'monotonic',return_value=now):
                viewer.update()
        self.assertTrue(navigator.enabled)
        viewer.link.send.assert_called_with(True,1,8,nav=True)
        self.assertIn('遥控7切换路径寻瓶，5停车',viewer.label.set_text.call_args.args[0])

    def test_key7_switches_follow_and_starts_bottle_mission_without_zero_or_e(self):
        from dataclasses import replace
        from protocol import NavTelemetry,Telemetry,NAV_CLEAR,NAV_COMMIT,NAV_POSE_RESET
        with tempfile.TemporaryDirectory() as directory:
            viewer=self.follow_viewer()
            viewer.window=mock.Mock()
            viewer.window.get_size.return_value=(1280,720)
            viewer.navigation_route_path=Path(directory)/'route.json'
            viewer.navigation_route_path.write_text('{"units":"mm","points":[[500,0]]}',encoding='utf-8')
            old=viewer.follower;old.enabled=True
            viewer.follow_was_armed=True
            viewer.link.live.return_value=viewer.link.nav_live.return_value=True
            viewer.link.telemetry=Telemetry(1,0,0x73,1,8,65535,65535,8,8,0,0,0,0)
            viewer.link.nav_telemetry=NavTelemetry(1,0,0,0,128|64,0,0,0,0,255,
                                                   follow_count=0,nav_mode_count=0)
            viewer.link.configure_navigation.side_effect=range(10,100)
            viewer.remote_path_mode_step(10.)  # Establish baseline, no switch.
            viewer.link.nav_telemetry=replace(viewer.link.nav_telemetry,nav_mode_count=1,follow_count=1)
            viewer.camera_latest=(b'rgb',10.1,1,'')
            viewer.recognition=(b'rgb',10.1,1,[],0.,'')
            with mock.patch.object(self.module.time,'monotonic',return_value=10.1):viewer.update()
            self.assertFalse(old.enabled)
            self.assertIsNone(viewer.follower)
            self.assertIsNone(viewer.navigator)
            self.assertEqual(viewer.mission.action,'find_bottle')
            self.assertIsNotNone(viewer.mission.run_request)
            self.assertFalse(any(c.args for c in viewer.link.send.call_args_list))
            for i in range(35):
                now=10.2+i*.15;n=viewer.link.nav_telemetry;flags=n.flags;ack={}
                navigator=viewer.mission.navigator
                if navigator.waiting:
                    seq,*_=navigator.waiting;op=navigator.queue[0][0]
                    if op==NAV_CLEAR:flags=128|64
                    if op==NAV_COMMIT:flags|=8
                    if op==NAV_POSE_RESET:flags|=32
                    ack=dict(ack_seq=seq,ack_status=0)
                viewer.link.nav_telemetry=replace(n,seq=n.seq+1,flags=flags,**ack)
                viewer.link.telemetry=replace(viewer.link.telemetry,seq=i+2,health=0x33,
                                               command=0,velocity_l=0,velocity_r=0)
                viewer.link.last_rx=viewer.link.last_nav_rx=now
                viewer.camera_latest=(b'rgb',now,i+2,'')
                viewer.recognition=(b'rgb',now,i+2,[],0.,'')
                with mock.patch.object(self.module.time,'monotonic',return_value=now):viewer.update()
                if viewer.mission.navigator.enabled:break
            self.assertTrue(viewer.mission.navigator.enabled)
            self.assertTrue(viewer.mission.enabled)
            viewer.link.send.assert_called_with(True,1,8,nav=True)
            self.assertEqual(viewer.link.nav_telemetry.follow_count,1)  # No later 0.

    def test_key7_in_dialog_is_consumed_and_not_replayed_when_closed(self):
        from protocol import NavTelemetry
        from dataclasses import replace
        viewer=self.follow_viewer()
        viewer.link.live.return_value=True
        viewer.link.nav_telemetry=NavTelemetry(1,0,0,0,128|64,0,0,0,0,0,
                                               follow_count=0,nav_mode_count=0)
        viewer.remote_path_mode_step(10.)
        viewer.semantic_dialog_open=True
        viewer.link.nav_telemetry=replace(viewer.link.nav_telemetry,nav_mode_count=1)
        viewer.remote_path_mode_step(10.1)
        viewer.semantic_dialog_open=False
        viewer.remote_path_mode_step(10.2)
        self.assertIsNone(getattr(viewer,'navigator',None))
        viewer.link.send.assert_not_called()

    def test_repeated_key7_preserves_running_search_and_does_not_reset_origin(self):
        from mission import MissionSession
        viewer=self.follow_viewer()
        viewer.mission=MissionSession([(500,0)],action='find_bottle')
        viewer.mission.remote_path_search=True;viewer.mission.enabled=True
        old=viewer.mission
        self.assertTrue(viewer.activate_navigation_mode(10.))
        self.assertIs(viewer.mission,old)
        viewer.link.send.assert_not_called()

    def test_key5_cancels_key7_search_before_start_without_undoing_manual_readiness(self):
        from dataclasses import replace
        from mission import MissionSession
        from protocol import NavTelemetry,Telemetry
        viewer=self.viewer()
        viewer.link.live.return_value=viewer.link.nav_live.return_value=True
        viewer.link.telemetry=Telemetry(1,0,0x33,0,0,65535,65535,0,0,0,0,0,0)
        viewer.link.nav_telemetry=NavTelemetry(1,0,0,0,128|64,0,0,0,0,0,
                                               follow_count=0,nav_mode_count=1)
        viewer.remote_path_mode_step(10.)
        viewer.mission=MissionSession([(500,0)],action='find_bottle')
        viewer.mission.remote_path_search=True
        self.assertTrue(viewer.mission.request_run(viewer.link,10.))
        viewer.link.nav_telemetry=replace(viewer.link.nav_telemetry,stop_epoch=1)
        viewer.remote_path_mode_step(10.1)
        viewer.mission.step(viewer.link,10.1,[],10.1,1280,True,height=720)
        self.assertEqual(viewer.mission.state,'stopped')
        self.assertIsNone(viewer.mission.run_request)
        viewer.link.send.assert_not_called()

    def test_simultaneous_key7_and_stop_does_not_switch_or_undo_manual_readiness(self):
        from dataclasses import replace
        from protocol import NavTelemetry,Telemetry
        viewer=self.follow_viewer();viewer.follower.enabled=True
        viewer.follow_was_armed=True;viewer.follow_last_send=9.
        viewer.follow_feedback_deadline=11.;viewer.follow_feedback_seq=1
        viewer.link.live.return_value=True
        viewer.link.telemetry=Telemetry(1,0,0x33,0,0,65535,65535,0,0,0,0,0,0)
        viewer.link.nav_telemetry=NavTelemetry(1,0,0,0,128|64,0,0,0,0,0,
                                               follow_count=0,nav_mode_count=0)
        viewer.remote_path_mode_step(10.)
        viewer.remote_follow_step(10.,viewer.link.telemetry,True,[],10.)
        viewer.link.nav_telemetry=replace(viewer.link.nav_telemetry,nav_mode_count=1,stop_epoch=1)
        with mock.patch.object(self.module.time,'monotonic',return_value=10.1):viewer.update()
        self.assertIsNone(getattr(viewer,'navigator',None))
        self.assertFalse(viewer.follower.enabled)
        viewer.link.send.assert_not_called()

    def test_invalid_key7_route_stops_old_controller_and_does_not_start_other_mode(self):
        from protocol import NavTelemetry,Telemetry
        viewer=self.follow_viewer();viewer.follower.enabled=True
        viewer.navigation_route_path=Path('missing-waypoint-route-for-test.json')
        viewer.link.live.return_value=True
        viewer.link.telemetry=Telemetry(1,0,0x33,0,0,65535,65535,0,0,0,0,0,0)
        viewer.link.nav_telemetry=NavTelemetry(1,0,0,0,128|64,0,0,0,0,0,
                                               follow_count=0,nav_mode_count=1)
        self.assertFalse(viewer.activate_navigation_mode(10.))
        self.assertFalse(viewer.follower.enabled)
        self.assertIsNone(getattr(viewer,'navigator',None))
        viewer.link.send.assert_called_once_with()

    def test_key7_request_upgrades_receive_only_owner_and_stop_during_reopen_cancels(self):
        from dataclasses import replace
        from protocol import NavTelemetry,Telemetry
        with tempfile.TemporaryDirectory() as directory:
            viewer=self.viewer();viewer.navigation_route_path=Path(directory)/'route.json'
            viewer.navigation_route_path.write_text('{"units":"mm","points":[[500,0]]}',encoding='utf-8')
            old=viewer.link;old.send=None;old.live.return_value=old.nav_live.return_value=True
            old.telemetry=Telemetry(1,0,0x32,0,0,65535,65535,0,0,0,0,0,0)
            old.nav_telemetry=NavTelemetry(1,0,0,0,128|64,0,0,0,0,0,
                                          follow_count=0,nav_mode_count=1)
            writer=mock.Mock();writer.live.return_value=writer.nav_live.return_value=False
            with mock.patch.object(self.module,'PosixControlLink',return_value=writer):
                self.assertTrue(viewer.activate_navigation_mode(10.))
            old.close.assert_called_once()
            viewer.mission.step(writer,10.1,[],10.1,1280,True,height=720)
            self.assertIsNotNone(viewer.mission.run_request)
            writer.live.return_value=writer.nav_live.return_value=True
            writer.telemetry=old.telemetry
            writer.nav_telemetry=replace(old.nav_telemetry,stop_epoch=1)
            viewer.mission.step(writer,10.2,[],10.2,1280,True,height=720)
            self.assertIsNone(viewer.mission.run_request)
            self.assertFalse(any(c.args for c in writer.send.call_args_list))

    def test_mission_close_stops_before_camera_cleanup(self):
        viewer=self.viewer()
        viewer.mission=mock.Mock()
        events=[]
        viewer.mission.stop.side_effect=lambda *_:events.append('stop mission')
        viewer.link.send.side_effect=lambda:events.append('disarm')
        viewer.log.close.side_effect=lambda:events.append('close camera log')
        viewer.close()
        self.assertEqual(events[:3],['stop mission','disarm','close camera log'])


    def test_csi_alias_keeps_single_ov5647_stream_and_existing_follow(self):
        from follow import TargetFollower
        viewer = self.viewer()
        viewer.camera_config = self.module.CameraConfig()
        viewer.follower = TargetFollower()
        viewer.follower.select_best([(0,.9,(100,100,400,600))],10,10)
        viewer.follower.start(10)
        with mock.patch.object(self.module, 'CaptureReader') as open_reader:
            viewer.set_source('csi')
            self.assertTrue(viewer.follower.enabled)
            viewer.link.send.assert_not_called()
            open_reader.assert_not_called()
            viewer.set_source('usb')
            open_reader.return_value.close.assert_not_called()
        self.assertTrue(viewer.follower.enabled)
        viewer.link.send.assert_not_called()

    def test_csi_preview_uses_its_own_rgb_frame_not_usb_boxes_and_clears_stale(self):
        viewer = self.viewer(); viewer.display_source = 'csi'
        viewer.last_displayed = None; viewer.window = mock.Mock()
        viewer.window.get_size.return_value = (1920,1080)
        viewer.camera_latest = (b'usb raw',9.9,1,'')
        viewer.recognition = (b'usb boxes',9.9,1,[(0,.9,(1,2,3,4))],75,'')
        viewer.csi_reader = mock.Mock()
        viewer.csi_reader.snapshot.return_value = (b'csi RGB',9.9,3,'')
        with mock.patch.object(self.module.time,'monotonic',return_value=10): viewer.update()
        self.glib.Bytes.new.assert_called_with(b'csi RGB')
        self.assertIn('OV5647 CSI采集画面',viewer.label.set_text.call_args.args[0])
        with mock.patch.object(self.module.time,'monotonic',return_value=11): viewer.update()
        viewer.image.clear.assert_called_once()
        self.assertIn('已超时',viewer.label.set_text.call_args.args[0])

    def test_primary_ov5647_reuses_inference_frame_and_does_not_open_same_camera_twice(self):
        viewer = self.viewer()
        viewer.camera_config = self.module.CameraConfig(primary='ov5647')
        viewer.camera_name = 'OV5647'
        viewer.camera_latest = (b'OV raw',9.9,2,'')
        viewer.recognition = (b'OV boxes',9.9,2,[(0,.9,(1,2,3,4))],75,'')
        viewer.window = mock.Mock(); viewer.window.get_size.return_value = (1920,1080)
        with mock.patch.object(self.module,'CaptureReader') as open_reader:
            viewer.set_source('csi')
            open_reader.assert_not_called()
        self.assertEqual(viewer.display_source,'usb')  # Legacy token means primary recognition view.
        with mock.patch.object(self.module.time,'monotonic',return_value=10): viewer.update()
        self.glib.Bytes.new.assert_called_with(b'OV boxes')
        self.assertIn('OV5647识别画面',viewer.label.set_text.call_args.args[0])
        viewer.link.send.assert_not_called()

    def test_csi_cleanup_failure_still_closes_serial_and_quits(self):
        viewer = self.viewer(); viewer.csi_reader = mock.Mock()
        viewer.csi_reader.close.side_effect = TimeoutError('worker blocked')
        with mock.patch('builtins.print'): viewer.close()
        viewer.link.close.assert_called_once()
        self.gtk.main_quit.assert_called_once()

    def test_fpga_inset_keeps_its_own_pixels_while_primary_has_detection_boxes(self):
        import numpy as np
        viewer=self.viewer();viewer.fpga_monitor_requested=True
        viewer.fpga_receiver=mock.Mock(error=None)
        frame=np.zeros((480,640,3),dtype=np.uint8);frame[0,0]=(10,20,30)
        viewer.fpga_receiver.snapshot.return_value=(frame,9.9)
        viewer.fpga_image=mock.Mock();viewer.fpga_label=mock.Mock()
        viewer.camera_latest=(b'primary raw',9.9,2,'')
        viewer.recognition=(b'primary boxes',9.9,2,[(0,.9,(100,100,400,600))],75,'')
        viewer.window=mock.Mock();viewer.window.get_size.return_value=(1920,1080)
        with mock.patch.object(self.module.time,'monotonic',return_value=10):viewer.update()
        calls=self.glib.Bytes.new.call_args_list
        self.assertEqual(calls[0].args[0][:3],bytes((30,20,10)))
        self.assertEqual(calls[-1].args[0],b'primary boxes')
        viewer.fpga_image.set_from_pixbuf.assert_called_once()
        viewer.link.send.assert_not_called()
        self.assertIn('独立实时图像',viewer.label.set_text.call_args.args[0])

    def test_stale_fpga_inset_clears_without_substituting_primary_image(self):
        import numpy as np
        viewer=self.viewer();viewer.fpga_monitor_requested=True
        viewer.fpga_receiver=mock.Mock(error=None)
        viewer.fpga_receiver.snapshot.return_value=(np.zeros((480,640,3),dtype=np.uint8),9.)
        viewer.fpga_image=mock.Mock();viewer.fpga_label=mock.Mock()
        viewer.camera_latest=(b'fresh primary',9.9,3,'')
        viewer.window=mock.Mock();viewer.window.get_size.return_value=(1920,1080)
        with mock.patch.object(self.module.time,'monotonic',return_value=10):viewer.update()
        viewer.fpga_image.clear.assert_called_once()
        viewer.fpga_image.set_from_pixbuf.assert_not_called()
        self.glib.Bytes.new.assert_called_once_with(b'fresh primary')
        self.assertIn('不替代FPGA采集',viewer.label.set_text.call_args.args[0])

    def test_legacy_fpga_decoder_is_isolated_from_the_ov5647_capture_entry(self):
        import numpy as np
        viewer=self.viewer();viewer.fpga_receiver=mock.Mock(error=None)
        viewer.camera_config=self.module.CameraConfig(primary='fpga')
        frame=np.zeros((480,640,3),dtype=np.uint8);frame[0,0]=(10,20,30)
        viewer.fpga_receiver.snapshot.return_value=(frame,9.9)
        cv=mock.Mock();cv.resize.return_value=np.full((720,960,3),7,dtype=np.uint8)
        with mock.patch.dict(sys.modules,{'cv2':cv}), \
             mock.patch.object(self.module.time,'monotonic',return_value=10), \
             mock.patch.object(viewer.stop_event,'wait',side_effect=lambda _:viewer.stop_event.set()), \
             mock.patch('video.VideoReceiver') as open_receiver, \
             mock.patch.object(self.module.subprocess,'Popen') as open_uvc:
            viewer.read_fpga_camera()
        self.assertEqual(tuple(cv.resize.call_args.args[0][0,0]),(30,20,10))
        rgb=np.frombuffer(viewer.camera_latest[0],dtype=np.uint8).reshape(720,1280,3)
        self.assertTrue(np.all(rgb[:,:160]==0));self.assertTrue(np.all(rgb[:,160:1120]==7))
        self.assertEqual(viewer.camera_latest[1],9.9)
        self.assertEqual(viewer.camera_frames,1)
        open_receiver.assert_not_called();open_uvc.assert_not_called()

    def test_primary_fpga_frame_loss_invalidates_recognition_input_without_usb_fallback(self):
        viewer=self.viewer();viewer.camera_latest=(b'old',9.9,1,'')
        viewer.fpga_receiver=mock.Mock(error=None)
        viewer.fpga_receiver.snapshot.return_value=(None,0.)
        with mock.patch.dict(sys.modules,{'cv2':mock.Mock()}), \
             mock.patch.object(viewer.stop_event,'wait',side_effect=lambda _:viewer.stop_event.set()), \
             mock.patch.object(self.module.subprocess,'Popen') as open_uvc:
            viewer.read_fpga_camera()
        self.assertIsNone(viewer.camera_latest[0]);self.assertEqual(viewer.camera_latest[1],0.)
        self.assertIn('已超时',viewer.camera_latest[3]);open_uvc.assert_not_called()

    def test_fpga_monitor_failure_on_close_still_releases_serial(self):
        viewer=self.viewer();viewer.fpga_receiver=mock.Mock()
        viewer.fpga_receiver.close.side_effect=OSError('socket failure')
        with mock.patch('builtins.print'):viewer.close()
        viewer.link.close.assert_called_once();self.gtk.main_quit.assert_called_once()


    def test_fpga_preview_is_rejected_without_opening_old_camera_or_changing_motion(self):
        from follow import TargetFollower
        viewer=self.viewer()
        viewer.follower=TargetFollower()
        viewer.follower.select_best([(0,.9,(100,100,400,600))],10,10)
        viewer.follower.start(10)
        with mock.patch('video.VideoReceiver') as receiver:
            viewer.set_source('fpga')
            self.assertTrue(viewer.follower.enabled)
            viewer.link.send.assert_not_called()
            receiver.assert_not_called()
            viewer.set_source('usb')
            self.assertTrue(viewer.follower.enabled)
            viewer.link.send.assert_not_called()


    def test_fpga_view_uses_its_geometry_rgb_order_and_expires(self):
        import numpy as np
        viewer=self.viewer()
        viewer.display_source='fpga'
        viewer.last_displayed=None
        viewer.window=mock.Mock()
        viewer.window.get_size.return_value=(1920,1080)
        viewer.camera_latest=(b'usb',9.9,1,'')
        image=np.zeros((480,640,3),dtype=np.uint8)
        image[0,0]=(10,20,30)
        viewer.fpga_receiver=mock.Mock(error=None)
        viewer.fpga_receiver.snapshot.return_value=(image,9.9)
        with mock.patch.object(self.module.time,'monotonic',return_value=10):
            viewer.update()
        self.assertEqual(self.glib.Bytes.new.call_args.args[0][:3],bytes((30,20,10)))
        self.assertEqual(self.module.GdkPixbuf.Pixbuf.new_from_bytes.call_args.args[4:],(640,480,1920))
        self.assertIn('实时图像',viewer.label.set_text.call_args.args[0])
        with mock.patch.object(self.module.time,'monotonic',return_value=11):
            viewer.update()
        viewer.image.clear.assert_called_once()
        self.assertIn('已超时',viewer.label.set_text.call_args.args[0])


    def test_fpga_receiver_failure_does_not_prevent_return_to_usb(self):
        viewer=self.viewer()
        with mock.patch('video.VideoReceiver',side_effect=OSError('port occupied')):
            self.assertFalse(viewer.set_source('fpga'))
        self.assertIn('OV5647',viewer.ui_notice)
        viewer.set_source('usb')
        self.assertEqual(viewer.display_source,'usb')
        viewer.link.send.assert_not_called()


    def test_lidar_details_do_not_start_motion_and_stale_ranges_are_hidden(self):
        v=self.viewer()
        v.link.lidar_telemetry=self.module.LidarTelemetry(1,63,7,500,1200,900,2000,123,2,0)
        v.link.lidar_live.return_value=True
        with mock.patch.object(self.module.time,'monotonic',return_value=10):v.update()
        self.assertIn('激光雷达：参与避障',v.label.set_text.call_args.args[0])
        self.assertIn('前/左/右 50 / 120 / 90 cm',v.label.set_text.call_args.args[0])
        self.assertNotIn('前/左/右/后',v.label.set_text.call_args.args[0])
        v.link.send.assert_not_called()
        v.link.lidar_live.return_value=False
        with mock.patch.object(self.module.time,'monotonic',return_value=11):v.update()
        self.assertIn('激光雷达：未连接或无完整扫描',v.label.set_text.call_args.args[0])
        self.assertNotIn('前/左/右 50 / 120 / 90 cm',v.label.set_text.call_args.args[0])
        v.link.send.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
