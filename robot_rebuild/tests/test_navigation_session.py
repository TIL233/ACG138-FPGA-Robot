from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
from navigation import NavigationSession,load_route
from protocol import NavTelemetry,Telemetry,NAV_COMMIT,NAV_POSE_RESET


class NavigationSessionTests(unittest.TestCase):
    def setUp(self):
        self.link=Mock()
        self.link.live.return_value=True
        self.link.nav_live.return_value=True
        self.link.telemetry=Telemetry(1,0,0x3f,0,0,1000,1000,0,0,0,0,0,0)
        self.link.nav_telemetry=NavTelemetry(1,0,0,0,128,0,0,0,0,255)
        self.link.last_rx=self.link.last_nav_rx=0.
        self.link.configure_navigation.side_effect=range(10,100)
        self.s=NavigationSession([(1000,0),(1000,1000)])

    def ready(self):
        self.s.queue.clear();self.s.waiting=None;self.s.pending_disarm=False;self.s.prepared=True;self.s.route_confirmed=True
        self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=128|8|32,count=2)

    def test_old_firmware_never_gets_arm(self):
        self.link.nav_live.return_value=False
        self.s.step(self.link,0)
        self.assertFalse(self.s.start(self.link,.1))
        self.s.step(self.link,1)
        self.link.send.assert_called_once_with()
        self.link.configure_navigation.assert_not_called()

    def test_missing_startup_serial_preserves_upload_but_never_starts_motion(self):
        self.link.send.side_effect=OSError('not connected')
        self.s.step(self.link,0)
        self.assertTrue(self.s.queue)
        self.assertFalse(self.s.enabled)
        self.link.send.side_effect=None
        self.s.step(self.link,1);self.s.step(self.link,1.3)
        self.assertTrue(self.s.waiting)
        self.assertTrue(all(not c.args for c in self.link.send.call_args_list))

    def test_upload_needs_ack_and_never_arms(self):
        self.s.step(self.link,0);self.s.step(self.link,.3)
        first=self.link.configure_navigation.call_count
        self.s.step(self.link,.4)
        self.assertEqual(self.link.configure_navigation.call_count,first)
        for i in range(8):
            if self.s.waiting:
                seq,_,_,_=self.s.waiting
                opcode=self.s.queue[0][0]
                flags=self.link.nav_telemetry.flags
                if opcode==NAV_COMMIT:flags|=8
                if opcode==NAV_POSE_RESET:flags|=32
                self.link.nav_telemetry=replace(self.link.nav_telemetry,seq=i+2,ack_seq=seq,ack_status=0,flags=flags,count=2)
            self.s.step(self.link,.5+i*.15)
        self.assertTrue(self.s.prepared)
        self.assertFalse(self.s.enabled)
        self.assertTrue(all(not c.args for c in self.link.send.call_args_list))

    def test_configuration_ack_timeout_stops_without_arm(self):
        self.s.step(self.link,0);self.s.step(self.link,.3);self.s.step(self.link,1.5)
        self.assertFalse(self.s.queue)
        self.assertFalse(self.s.enabled)
        self.assertIn('无确认',self.s.text)

    def test_failed_upload_cannot_enable_a_previously_stored_route(self):
        self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=128|8|32,count=8)
        self.s.step(self.link,0);self.s.step(self.link,.3);self.s.step(self.link,1.5)
        self.s.step(self.link,1.7)
        self.assertFalse(self.s.prepared)
        self.assertFalse(self.s.start(self.link,1.8))

    def test_stale_ack_cannot_advance_configuration(self):
        self.link.nav_telemetry=replace(self.link.nav_telemetry,ack_seq=10,ack_status=0)
        self.s.step(self.link,0);self.s.step(self.link,.3)
        waiting=self.s.waiting
        self.s.step(self.link,.4)
        self.assertEqual(self.s.waiting,waiting)
        self.assertEqual(self.s.queue[0][0],4)

    def test_explicit_start_then_remote_stop_latches(self):
        self.ready()
        self.assertTrue(self.s.start(self.link,1))
        self.s.step(self.link,1)
        self.assertFalse(self.s.enabled)
        self.link.telemetry=replace(self.link.telemetry,seq=2)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,seq=2)
        self.link.last_rx=self.link.last_nav_rx=1.3
        self.s.step(self.link,1.3)
        self.link.send.assert_called_with(True,1,8,nav=True)
        self.assertTrue(self.s.enabled)
        # A remote STOP removes selected/armed flags even though sensors are OK.
        self.s.step(self.link,1.8)
        self.assertFalse(self.s.enabled)
        arms=sum(bool(c.args and c.args[0]) for c in self.link.send.call_args_list)
        self.s.step(self.link,2)
        self.assertEqual(arms,sum(bool(c.args and c.args[0]) for c in self.link.send.call_args_list))

    def test_stale_sensor_fault_and_done_stop(self):
        for failure in ('stale','sensor','done','fault'):
            self.setUp();self.ready();self.s.enabled=True;self.s.grace_until=9
            if failure=='stale':self.link.nav_live.return_value=False
            elif failure=='sensor':self.link.telemetry=replace(self.link.telemetry,health=0x3d)
            elif failure=='fault':self.link.telemetry=replace(self.link.telemetry,fault=32)
            else:self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=128|8|32|2)
            self.s.step(self.link,1)
            self.assertFalse(self.s.enabled)

    def test_motor_protection_reason_survives_fault_clear_without_restart(self):
        self.ready();self.s.enabled=True;self.s.grace_until=9
        self.link.telemetry=replace(self.link.telemetry,fault=32)
        self.s.step(self.link,1)
        self.assertIn('电机闭环',self.s.text)
        self.assertIn('0x0020',self.s.text)
        self.assertNotIn('传感器/导航故障',self.s.text)
        self.link.send.reset_mock()
        self.link.telemetry=replace(self.link.telemetry,fault=0)
        self.s.step(self.link,1.1)
        self.assertIn('0x0020',self.s.text)
        self.assertFalse(self.s.enabled)
        self.assertTrue(all(not c.args for c in self.link.send.call_args_list))

    def test_start_requires_both_new_streams_after_the_actual_disarm_send(self):
        self.ready();self.assertTrue(self.s.start(self.link,1))
        self.s.step(self.link,1.4)  # A delayed first tick may only send DISARM.
        self.assertFalse(self.s.enabled)
        self.assertTrue(all(not c.args for c in self.link.send.call_args_list))
        self.link.telemetry=replace(self.link.telemetry,seq=2)
        self.link.last_rx=self.link.last_nav_rx=1.6
        self.s.step(self.link,1.6)
        self.assertFalse(self.s.enabled)  # New ordinary frame alone is insufficient.
        self.link.nav_telemetry=replace(self.link.nav_telemetry,seq=2)
        self.link.last_nav_rx=1.41
        self.s.step(self.link,1.7)
        self.assertFalse(self.s.enabled)  # Old/in-flight nav frame is insufficient.
        self.link.last_rx=self.link.last_nav_rx=1.8
        self.s.step(self.link,1.8)
        self.assertTrue(self.s.enabled)
        self.link.send.assert_called_with(True,1,8,nav=True)

    def test_stopped_start_rejects_external_motion_or_encoder_activity(self):
        for fields in (dict(health=0x7f),dict(command=1),dict(state=2),dict(velocity_r=1)):
            with self.subTest(fields=fields):
                self.setUp();self.ready()
                self.link.telemetry=replace(self.link.telemetry,**fields)
                self.assertFalse(self.s.start(self.link,1))
                self.link.send.assert_not_called()

    def test_physical_stop_wins_pending_start_without_undoing_ir_preparation(self):
        self.ready();self.assertTrue(self.s.start(self.link,1))
        self.link.nav_telemetry=replace(self.link.nav_telemetry,seq=2,stop_epoch=1)
        self.s.step(self.link,1.3)
        self.assertFalse(self.s.enabled);self.assertIsNone(self.s.start_at)
        self.link.send.assert_not_called()
        self.s.step(self.link,2)
        self.link.send.assert_not_called()

    def test_physical_stop_cancels_configuration_without_another_uart_stop(self):
        self.s.step(self.link,0);self.s.step(self.link,.3)
        sends=self.link.send.call_count;configs=self.link.configure_navigation.call_count
        self.link.nav_telemetry=replace(self.link.nav_telemetry,seq=2,stop_epoch=1)
        self.s.step(self.link,.4);self.s.step(self.link,.8)
        self.assertFalse(self.s.queue);self.assertFalse(self.s.enabled)
        self.assertEqual(self.link.send.call_count,sends)
        self.assertEqual(self.link.configure_navigation.call_count,configs)

    def test_start_confirmation_timeout_never_rearms_from_recovered_old_frames(self):
        self.ready();self.assertTrue(self.s.start(self.link,1));self.s.step(self.link,1)
        self.s.step(self.link,5.1)
        self.assertIsNone(self.s.start_at);self.assertFalse(self.s.enabled)
        before=sum(bool(c.args and c.args[0]) for c in self.link.send.call_args_list)
        self.link.telemetry=replace(self.link.telemetry,seq=2)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,seq=2)
        self.link.last_rx=self.link.last_nav_rx=5.3;self.s.step(self.link,5.3)
        self.assertEqual(sum(bool(c.args and c.args[0]) for c in self.link.send.call_args_list),before)

    def test_json_units_bounds_and_point_count(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'route.json'
            for data in ({'units':'m','points':[[1,0]]},{'units':'mm','points':[[10001,0]]},{'units':'mm','points':[[True,0]]},{'units':'mm','points':[]}):
                p.write_text(json.dumps(data))
                with self.assertRaises(ValueError):load_route(p)
            p.write_text(json.dumps({'units':'mm','points':[[1000,0]]}))
            self.assertEqual(load_route(p),[(1000,0)])

    def test_mission_next_leg_preserves_the_original_coordinate_frame(self):
        first=NavigationSession([(1000,0)])
        later=NavigationSession([(1000,1000)],initialize_pose=False)
        self.assertIn(NAV_POSE_RESET,[op for op,_ in first.queue])
        self.assertNotIn(NAV_POSE_RESET,[op for op,_ in later.queue])
        self.assertFalse(later.prepared)
        self.assertFalse(later.start(self.link,0))


if __name__=='__main__':unittest.main()
