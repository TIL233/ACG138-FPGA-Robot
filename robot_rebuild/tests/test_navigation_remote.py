"""Real navigation session driven by the existing FPGA remote0 generations."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
from navigation import NavigationSession
from protocol import NavTelemetry,Telemetry,NAV_POSE_RESET,NAV_CLEAR


class NavigationRemoteTests(unittest.TestCase):
    def setUp(self):
        self.link=Mock()
        self.link.live.return_value=self.link.nav_live.return_value=True
        self.link.telemetry=Telemetry(1,0,0x32,0,0,65535,65535,0,0,0,0,0,0)
        self.link.nav_telemetry=NavTelemetry(1,0,0,1,128|64|32|8,0,0,0,0,255,follow_count=10)
        self.link.last_rx=self.link.last_nav_rx=10.
        self.link.configure_navigation.side_effect=range(10,100)
        self.session=NavigationSession([(500,0)],remote_enabled=True)
        self.prepared()
        self.session.step(self.link,10)  # First snapshot is a baseline, not a click.

    def prepared(self):
        s=self.session
        s.queue.clear();s.waiting=None;s.pending_disarm=False
        s.prepared=s.route_confirmed=True

    def click(self,now=10.1,**kwargs):
        n=self.link.nav_telemetry
        self.link.nav_telemetry=replace(n,follow_count=n.follow_count+1)
        return self.session.step(self.link,now,**kwargs)

    def feedback(self,now):
        self.link.telemetry=replace(self.link.telemetry,seq=self.link.telemetry.seq+1,health=0x33)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,seq=self.link.nav_telemetry.seq+1)
        self.link.last_rx=self.link.last_nav_rx=now
        self.session.step(self.link,now)

    def arms(self):
        return [call for call in self.link.send.call_args_list if call.args and call.args[0]]

    def test_remote0_renews_expired_idle_link_then_requires_fresh_stopped_feedback(self):
        self.link.send.assert_not_called()
        self.click()
        self.link.send.assert_called_once_with()
        self.assertIsNotNone(self.session.remote_start_pending)
        self.assertFalse(self.arms())
        self.feedback(10.25)  # Authorizes only the shared navigation handshake.
        self.assertIsNotNone(self.session.start_at)
        self.assertFalse(self.arms())
        self.feedback(10.55)
        self.assertTrue(self.session.enabled)
        self.link.send.assert_called_with(True,1,8,nav=True)
        self.assertEqual(len(self.arms()),1)

    def test_remote0_during_configuration_is_consumed_not_queued(self):
        self.session.upload()
        self.click()
        self.assertIsNone(self.session.remote_start_pending)
        self.prepared()
        self.feedback(10.5)
        self.assertIsNone(self.session.start_at)
        self.assertFalse(self.arms())

    def test_same_generation_and_completed_route_do_not_replay_start(self):
        self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=128|64|32|8|2)
        self.click()
        self.assertEqual([op for op,_ in self.session.queue],[NAV_POSE_RESET])
        self.assertIsNone(self.session.remote_start_pending)
        self.assertFalse(self.arms())
        self.prepared()
        self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=128|64|32|8)
        self.feedback(10.5)
        self.assertIsNone(self.session.start_at)
        self.assertFalse(self.arms())
        self.click(10.6)
        self.assertIsNotNone(self.session.remote_start_pending)

    def test_missing_route_reuploads_without_starting_or_reusing_old_route(self):
        self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=128|64)
        self.click()
        self.assertEqual(self.session.queue[0][0],NAV_CLEAR)
        self.assertFalse(self.session.prepared)
        self.assertIsNone(self.session.remote_start_pending)
        self.assertFalse(self.arms())

    def test_stop_and_click_in_one_snapshot_stop_wins_without_uart_disarm(self):
        n=self.link.nav_telemetry
        self.link.nav_telemetry=replace(n,follow_count=11,stop_epoch=1)
        self.session.step(self.link,10.1)
        self.session.step(self.link,10.2)
        self.link.send.assert_not_called()
        self.assertIsNone(self.session.remote_start_pending)
        self.assertIsNone(self.session.start_at)

    def test_stop_cancels_remote_confirmation_and_preserves_manual_ir_preparation(self):
        self.click()
        self.link.send.reset_mock()
        self.link.nav_telemetry=replace(self.link.nav_telemetry,stop_epoch=1)
        self.session.step(self.link,10.2)
        self.session.step(self.link,10.3)
        self.feedback(10.5)
        self.link.send.assert_not_called()
        self.assertIsNone(self.session.remote_start_pending)
        self.assertFalse(self.session.enabled)

    def test_blocked_view_dialog_or_stop_delivery_consumes_hidden_click(self):
        self.click(remote_allowed=False)
        self.feedback(10.4)
        self.link.send.assert_not_called()
        self.assertIsNone(self.session.remote_start_pending)
        self.assertIsNone(self.session.start_at)

    def test_remote0_after_stop_resumes_route_without_resetting_origin_or_index(self):
        self.click();self.feedback(10.25);self.feedback(10.55)
        self.assertTrue(self.session.enabled)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,stop_epoch=1,x_mm=230)
        self.session.step(self.link,10.65)
        self.session.step(self.link,10.7)
        self.assertFalse(self.session.enabled)
        self.click(10.8);self.feedback(10.95);self.feedback(11.25)
        self.assertTrue(self.session.enabled)
        self.assertEqual(self.link.nav_telemetry.x_mm,230)
        self.link.configure_navigation.assert_not_called()
        self.assertEqual(len(self.arms()),2)

    def test_blocking_view_after_click_cancels_pending_start(self):
        self.click()
        self.session.step(self.link,10.2,remote_allowed=False)
        self.feedback(10.5)
        self.assertIsNone(self.session.remote_start_pending)
        self.assertIsNone(self.session.start_at)
        self.assertFalse(self.arms())

    def test_unsafe_click_is_not_saved_until_sensors_or_motion_recover(self):
        for fields in (dict(health=0x30),dict(fault=32),dict(health=0x72),
                       dict(command=1),dict(state=2),dict(velocity_l=1)):
            with self.subTest(fields=fields):
                self.setUp()
                self.link.telemetry=replace(self.link.telemetry,**fields)
                self.click()
                self.link.telemetry=replace(self.link.telemetry,health=0x33,fault=0,
                                            command=0,state=0,velocity_l=0)
                self.feedback(10.5)
                self.assertIsNone(self.session.remote_start_pending)
                self.assertIsNone(self.session.start_at)
                self.link.send.assert_not_called()

    def test_timeout_requires_another_click_even_when_new_frames_later_arrive(self):
        self.click()
        self.link.telemetry=replace(self.link.telemetry,seq=2,health=0x33)
        self.link.last_rx=self.link.last_nav_rx=10.4
        self.session.step(self.link,10.4)  # Navigation sequence is still cached.
        self.assertIsNone(self.session.start_at)
        self.session.step(self.link,11.)
        self.feedback(11.2)
        self.assertIsNone(self.session.remote_start_pending)
        self.assertIsNone(self.session.start_at)
        self.assertFalse(self.arms())

    def test_disconnect_reset_and_missed_burst_cannot_queue_a_start(self):
        self.click()
        self.link.nav_live.return_value=False
        self.session.step(self.link,10.2)
        self.link.nav_live.return_value=True
        self.link.nav_telemetry=replace(self.link.nav_telemetry,follow_count=3)
        self.feedback(10.5)  # Reconnection establishes a new baseline.
        self.link.nav_telemetry=replace(self.link.nav_telemetry,follow_count=5)
        self.feedback(10.6)  # Missed/coalesced clicks cannot start.
        self.assertIsNone(self.session.remote_start_pending)
        self.assertIsNone(self.session.start_at)
        self.assertFalse(self.arms())

    def test_remote_control_is_opt_in_so_task6_does_not_treat_zero_as_navigation(self):
        session=NavigationSession([(500,0)])
        self.assertIsNone(session.remote_trigger)
        session.queue.clear();session.waiting=None;session.pending_disarm=False
        session.prepared=session.route_confirmed=True
        self.link.nav_telemetry=replace(self.link.nav_telemetry,follow_count=11)
        session.step(self.link,10.2)
        self.assertIsNone(session.remote_start_pending)
        self.assertIsNone(session.start_at)
        self.link.send.assert_not_called()


if __name__=='__main__':unittest.main()
