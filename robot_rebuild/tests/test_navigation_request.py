"""One key7 request prepares/configures and starts, without a later key0."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
from navigation import NavigationSession
from protocol import NavTelemetry,Telemetry,NAV_CLEAR,NAV_COMMIT,NAV_POSE_RESET


class NavigationRequestTests(unittest.TestCase):
    def setUp(self):
        self.link=Mock()
        self.link.live.return_value=self.link.nav_live.return_value=True
        self.link.telemetry=Telemetry(1,0,0x32,0,0,65535,65535,0,0,0,0,0,0)
        self.link.nav_telemetry=NavTelemetry(1,0,0,0,128|64,0,0,0,0,255,follow_count=8,nav_mode_count=1)
        self.link.last_rx=self.link.last_nav_rx=10.
        self.link.configure_navigation.side_effect=range(10,100)
        self.session=NavigationSession([(500,0)],remote_enabled=True)

    def tick(self,now,**kwargs):
        n=self.link.nav_telemetry;flags=n.flags;ack={}
        if self.session.waiting:
            seq,*_=self.session.waiting
            op=self.session.queue[0][0]
            if op==NAV_CLEAR:flags=128|64
            if op==NAV_COMMIT:flags|=8
            if op==NAV_POSE_RESET:flags=(flags|32)&~(2|4)
            ack=dict(ack_seq=seq,ack_status=0)
        self.link.nav_telemetry=replace(n,seq=n.seq+1,flags=flags,**ack)
        self.link.telemetry=replace(self.link.telemetry,seq=self.link.telemetry.seq+1,health=0x33)
        self.link.last_rx=self.link.last_nav_rx=now
        self.session.step(self.link,now,**kwargs)

    def arms(self):return [c for c in self.link.send.call_args_list if c.args and c.args[0]]

    def test_single_request_configures_and_starts_with_no_key0_change(self):
        self.assertTrue(self.session.request_run(self.link,10.))
        for i in range(30):
            self.tick(10.+i*.15)
            if self.session.enabled:break
        self.assertTrue(self.session.enabled)
        self.assertEqual(self.link.nav_telemetry.follow_count,8)
        self.assertEqual(len(self.arms()),1)
        self.link.send.assert_called_with(True,1,8,nav=True)
        self.assertIsNone(self.session.run_request)

    def test_switch_from_active_follow_waits_until_disarmed_and_wheels_still(self):
        self.link.telemetry=replace(self.link.telemetry,health=0x73,command=1,velocity_l=8,velocity_r=8)
        self.assertTrue(self.session.request_run(self.link,10.))
        self.session.step(self.link,10.)
        self.session.step(self.link,10.3)
        self.link.configure_navigation.assert_not_called()
        self.assertFalse(self.arms())
        self.link.telemetry=replace(self.link.telemetry,health=0x33,command=0,velocity_l=0,velocity_r=0)
        for i in range(30):
            self.tick(10.4+i*.15)
            if self.session.enabled:break
        self.assertTrue(self.session.enabled)

    def test_physical_stop_cancels_preparation_without_undoing_ir(self):
        self.session.request_run(self.link,10.);self.tick(10.)
        self.link.send.reset_mock()
        self.link.nav_telemetry=replace(self.link.nav_telemetry,stop_epoch=1)
        self.session.step(self.link,10.2)
        self.session.step(self.link,10.4)
        self.assertIsNone(self.session.run_request)
        self.link.send.assert_not_called()
        self.assertFalse(self.arms())

    def test_hidden_view_cancels_before_due_start_could_arm(self):
        self.session.queue.clear();self.session.pending_disarm=False
        self.session.prepared=self.session.route_confirmed=True
        self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=128|64|32|8)
        self.link.telemetry=replace(self.link.telemetry,health=0x33)
        self.session.request_run(self.link,10.);self.tick(10.)
        self.assertIsNotNone(self.session.start_at)
        self.tick(10.4,remote_allowed=False)
        self.assertFalse(self.arms())
        self.assertIsNone(self.session.start_at)
        self.assertIsNone(self.session.run_request)

    def test_disconnect_during_preparation_cancels_request_not_delayed_start(self):
        self.session.request_run(self.link,10.);self.tick(10.)
        self.link.nav_live.return_value=False
        self.session.step(self.link,10.2)
        self.link.nav_live.return_value=True
        for i in range(12):self.tick(10.4+i*.15)
        self.assertIsNone(self.session.run_request)
        self.assertFalse(self.arms())

    def test_request_timeout_and_repeated_click_do_not_extend_authorization(self):
        self.session.request_run(self.link,10.)
        original=self.session.run_request
        self.assertTrue(self.session.request_run(self.link,11.))
        self.assertEqual(self.session.run_request,original)
        self.session.step(self.link,22.1)
        self.assertIsNone(self.session.run_request)
        self.assertFalse(self.arms())

    def test_fpga_reset_before_first_new_owner_snapshot_cancels_old_key7_intent(self):
        self.session.request_run(self.link,10.)
        self.link.live.return_value=self.link.nav_live.return_value=False
        self.session.step(self.link,10.1)
        self.assertIsNotNone(self.session.run_request)
        self.link.live.return_value=self.link.nav_live.return_value=True
        self.link.nav_telemetry=replace(self.link.nav_telemetry,nav_mode_count=0)
        self.tick(10.3)
        self.assertIsNone(self.session.run_request)
        self.assertFalse(self.arms())

    def test_completed_route_can_prepare_and_run_with_one_new_request(self):
        self.session.queue.clear();self.session.pending_disarm=False
        self.session.prepared=self.session.route_confirmed=True
        self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=128|64|32|8|2)
        self.assertTrue(self.session.request_run(self.link,10.))
        self.assertEqual(self.session.queue[0][0],NAV_POSE_RESET)
        for i in range(25):
            self.tick(10.+i*.15)
            if self.session.enabled:break
        self.assertTrue(self.session.enabled)
        self.assertEqual(len(self.arms()),1)

    def test_fault_at_click_or_later_during_upload_cancels_until_another_key7(self):
        self.link.telemetry=replace(self.link.telemetry,fault=32)
        self.assertFalse(self.session.request_run(self.link,10.))
        self.link.telemetry=replace(self.link.telemetry,fault=0)
        self.tick(10.1)
        self.assertFalse(self.arms())
        self.assertTrue(self.session.request_run(self.link,10.2))
        self.link.telemetry=replace(self.link.telemetry,health=0x30)
        self.session.step(self.link,10.3)
        self.assertIsNone(self.session.run_request)
        for i in range(10):self.tick(10.4+i*.15)
        self.assertFalse(self.arms())


if __name__=='__main__':unittest.main()
