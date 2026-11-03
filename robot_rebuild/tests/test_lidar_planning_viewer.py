"""Run the real GTK update and recording paths with hardware boundaries stubbed."""
import importlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from robot_rebuild.tests import test_hdmi_lifecycle as lifecycle
from robot_rebuild.tests.test_lidar_planning import frame


class PlanningViewerTests(unittest.TestCase):
    def setUp(self):
        self.harness=lifecycle.HdmiTests();self.harness.setUp()
        self.addCleanup(self.harness.doCleanups)
        self.module=self.harness.module

    def viewer(self, details=False):
        viewer=self.harness.viewer();viewer.show_details=details
        viewer.link.nav_telemetry=None
        wire=importlib.import_module('protocol')
        # patch.dict(sys.modules) in the GTK harness can restore an older
        # protocol import. Use the viewer's actual class, as lifecycle tests do.
        decoded=wire.LidarPlanningTelemetryParser().feed(frame())[0]
        viewer.link.lidar_planning_telemetry=self.module.LidarPlanningTelemetry(**vars(decoded))
        viewer.link.lidar_planning_live.return_value=True
        return viewer

    def test_compact_one_row_details_complete_and_never_sends_control(self):
        for details in (False,True):
            with self.subTest(details=details):
                viewer=self.viewer(details)
                with mock.patch.object(self.module.time,'monotonic',return_value=10):viewer.update()
                text=viewer.label.set_text.call_args.args[0]
                self.assertIn('雷达规划：观察｜建议左弧｜安全62cm',text)
                self.assertEqual(sum('雷达规划：' in line for line in text.splitlines()),1)
                self.assertEqual('候选 3/9' in text,details)
                if details:
                    for value in ('275ms','301','未验收','0x018','0x100','595ms'):
                        self.assertIn(value,text)
                viewer.link.send.assert_not_called()

    def test_recording_reuses_display_text_and_primary_detection_frame(self):
        for details in (False,True):
            viewer=self.viewer(details)
            viewer.camera_latest=(b'primary raw',9.9,9,'')
            viewer.recognition=(b'primary boxes',9.8,8,[(0,.9,(1,2,3,4))],75,'')
            viewer.window=mock.Mock();viewer.window.get_size.return_value=(1280,720)
            viewer.recorder=mock.Mock()
            viewer.recorder.snapshot.return_value=dict(state='recording',active=True,path='/usb/test.avi',segments=1,error='')
            renderer=mock.Mock();overlay=object();renderer.render.return_value=overlay
            with mock.patch.object(self.module,'TextOverlayRenderer',return_value=renderer),mock.patch.object(self.module.time,'monotonic',return_value=10):viewer.update()
            text=renderer.render.call_args.args[0]
            self.assertEqual(text,viewer.label.set_text.call_args.args[0])
            self.assertIn('雷达规划：观察｜建议左弧',text)
            viewer.recorder.offer.assert_called_once_with(b'primary boxes',9.8,overlay,
                fallback_rgb=b'primary raw',fallback_captured=9.9)
            viewer.link.send.assert_not_called()

    def test_status_json_planning_is_explicit_and_timeout_removes_old_targets(self):
        with tempfile.TemporaryDirectory() as directory:
            viewer=self.viewer();viewer.link.frames=4;viewer.link.crc_errors=0
            with mock.patch.object(self.module,'ROOT',Path(directory)):
                for now,live in ((10,True),(13,False)):
                    viewer.last_report=0;viewer.link.lidar_planning_live.return_value=live
                    with mock.patch.object(self.module.time,'monotonic',return_value=now):viewer.update()
                    status=json.loads((Path(directory)/'camera-status.json').read_text(encoding='utf8'))
                    self.assertIn('lidar_planning',status)
                    plan=status['lidar_planning']
                    self.assertEqual(status['lidar_planning_live'],live)
                    self.assertFalse(plan['actual_takeover'])
                    self.assertEqual(plan['candidate'],3 if live else None)
                    self.assertEqual(plan['target_l'],3 if live else None)
                    if not live:
                        self.assertIn('等待',status['display_text'])
                        self.assertNotIn('建议左弧',status['display_text'])
            viewer.link.send.assert_not_called()

    def test_old_firmware_has_no_planning_row_or_synthetic_json(self):
        with tempfile.TemporaryDirectory() as directory:
            viewer=self.harness.viewer();viewer.show_details=False
            viewer.link.nav_telemetry=None
            viewer.link.lidar_planning_telemetry=None
            viewer.link.frames=4;viewer.link.crc_errors=0;viewer.last_report=0
            with mock.patch.object(self.module,'ROOT',Path(directory)),mock.patch.object(self.module.time,'monotonic',return_value=10):viewer.update()
            status=json.loads((Path(directory)/'camera-status.json').read_text(encoding='utf8'))
            self.assertNotIn('雷达规划：',status['display_text'])
            self.assertIn('lidar_planning',status)
            self.assertIsNone(status['lidar_planning'])
            self.assertFalse(status['lidar_planning_live'])
            viewer.link.send.assert_not_called()

    def test_protective_stop_is_recorded_as_actual_takeover_without_suggested_route(self):
        wire=importlib.import_module('protocol')
        samples=wire.LidarPlanningTelemetryParser().feed(frame(flags=0x23a7,candidate=255,target_l=0,target_r=0))
        self.assertEqual(len(samples),1)
        for details in (False,True):
            viewer=self.viewer(details)
            viewer.link.lidar_planning_telemetry=self.module.LidarPlanningTelemetry(**vars(samples[0]))
            viewer.camera_latest=(b'primary raw',9.9,9,'')
            viewer.window=mock.Mock();viewer.window.get_size.return_value=(1280,720)
            viewer.recorder=mock.Mock()
            viewer.recorder.snapshot.return_value=dict(state='recording',active=True,path='/usb/test.avi',segments=1,error='')
            renderer=mock.Mock();overlay=object();renderer.render.return_value=overlay
            with mock.patch.object(self.module,'TextOverlayRenderer',return_value=renderer),mock.patch.object(self.module.time,'monotonic',return_value=10):viewer.update()
            text=viewer.label.set_text.call_args.args[0]
            self.assertIn('雷达规划：接管｜保护停车',text)
            self.assertIn('无可行路',text)
            self.assertNotIn('建议左弧',text)
            renderer.render.assert_called_once_with(text)
            viewer.recorder.offer.assert_called_once_with(b'primary raw',9.9,overlay,
                fallback_rgb=b'primary raw',fallback_captured=9.9)
            viewer.link.send.assert_not_called()


if __name__=='__main__':unittest.main()
