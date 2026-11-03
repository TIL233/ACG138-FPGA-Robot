"""Exercise the real deployment loop with an in-memory remote filesystem."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


class RemoteFile(io.BytesIO):
    def __init__(self, files, path, mode):
        self.files,self.path,self.mode=files,path,mode
        if 'r' in mode and path not in files:raise FileNotFoundError(path)
        super().__init__(files.get(path,b'') if 'r' in mode else b'')

    def write(self,data):
        return super().write(data.encode() if isinstance(data,str) else data)

    def close(self):
        if not self.closed and 'w' in self.mode:self.files[self.path]=self.getvalue()
        super().close()


class SemanticDeploymentTests(unittest.TestCase):
    def test_code_update_preserves_calibrated_locations_backup_and_follow_startup(self):
        self.exercise_deployment(False)

    def test_explicit_camera_config_switches_source_without_overwriting_routes(self):
        self.exercise_deployment(True)

    def test_old_usb_config_is_migrated_to_ov5647_preserving_csi_rotation_and_disk(self):
        self.exercise_deployment(False,old_usb=True)

    def test_no_mode_preserves_existing_follow_navigation_mission_or_read_only(self):
        for options in ('--follow person','--navigate route.json','--mission mission.json',''):
            with self.subTest(options=options):
                startup=('#!/bin/sh\n# existing local display settings\nexec python3 -B hdmi_camera.py '+options+'\n').encode()
                self.exercise_deployment(False,modes=(),startup=startup,expected_startup=startup)

    def test_first_deployment_without_mode_stays_read_only(self):
        self.exercise_deployment(False,modes=(),expected_options=b'')

    def test_explicit_read_only_replaces_existing_follow_mode(self):
        self.exercise_deployment(False,modes=('--read-only',),
            startup=b'#!/bin/sh\nexec python3 -B hdmi_camera.py --follow person\n',expected_options=b'')

    def test_explicit_follow_replaces_existing_read_only(self):
        self.exercise_deployment(False,startup=b'#!/bin/sh\nexec python3 -B hdmi_camera.py\n')

    def exercise_deployment(self, override_camera,old_usb=False,modes=('--follow',),startup=None,
                            expected_startup=None,expected_options=b'--follow person'):
        source=Path(__file__).resolve().parents[2]/'ch341_driver_work/sync_hdmi.py'
        spec=importlib.util.spec_from_file_location('sync_under_test',source)
        module=importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules,{'board_access':SimpleNamespace(connect=mock.Mock())}):
            spec.loader.exec_module(module)
        remote=module.REMOTE
        calibrated=json.dumps({'units':'mm','locations':{'实际门口':[2500,1300]},
                               'instruction':'去实际门口找人'},ensure_ascii=False).encode()
        camera_config=b'{"primary":"ov5647","csi_index":1,"rotation":180,"preview":"off"}'
        if old_usb:camera_config=b'{"primary":"usb","csi_index":1,"rotation":180,"preview":"ov5647","fpga_monitor":true}'
        recording_config=b'{"fps":6,"usb_mount":"/media/ubuntu/ROBOT"}'
        files={remote+'/mission_semantic_example.json':calibrated,
               remote+'/video.py':b'# retired receiver\n',
               remote+'/uvc_capture.py':b'# retired UVC capture\n',
               remote+'/camera_config.json':camera_config,
               remote+'/recording_config.json':recording_config,
               remote+'/route_example.json':b'{"units":"mm","points":[[750,100]]}',
               remote+'/hdmi_camera.py':b'# old viewer\n',
               '/home/ubuntu/.config/autostart/robot-usb-camera.desktop':b'old startup'}
        if startup is not None:files[remote+'/start-camera-viewer.sh']=startup
        sftp=mock.Mock()
        sftp.file.side_effect=lambda path,mode:RemoteFile(files,path,mode)
        def rename(old,new):files[new]=files.pop(old)
        sftp.posix_rename.side_effect=rename
        sftp.remove.side_effect=lambda path:files.pop(path)
        client=mock.Mock();client.open_sftp.return_value=sftp
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)/'host';root.mkdir()
            rk=root.parent/'robot_rebuild/rk';(rk/'models').mkdir(parents=True)
            for name in ('hdmi_camera.py','hdmi_telemetry.py','browser_viewer.py'):
                (root/name).write_bytes(b'# updated viewer\n')
            for name in ('protocol.py','detector.py','video.py','follow.py','navigation.py','mission.py',
                         'camera_capture.py','prepare_ov5647.py','rkaiq_single_camera.c','ov5647_raw_capture.py','robot-ov5647.service','csi_capture.py','video_recording.py','recording_overlay.py','usb_lidar.py','lidar_planning.py',
                         'mission_example.json','mission_semantic_example.json','mission_bottle_example.json','route_example.json'):
                (rk/name).write_bytes(b'# new source\n')
            (rk/'camera_config.json').write_bytes(b'{"primary":"usb"}')
            (rk/'recording_config.json').write_bytes(b'{"fps":10}')
            (rk/'usb_lidar_config.json').write_bytes(b'{"enabled":true}')
            (rk/'lidar_map_view.py').write_bytes(b'# radar map renderer\n')
            (rk/'models/yolov8n.onnx').write_bytes(b'test model')
            module.ROOT=root
            argv=['sync_hdmi.py',*modes]
            if override_camera: argv += ['--camera-config',str(rk/'camera_config.json')]
            with mock.patch.object(module,'connect',return_value=client), \
                 mock.patch.object(module,'run',return_value=('ok','')), \
                 mock.patch.object(sys,'argv',argv),mock.patch('builtins.print'):
                module.main()
            report=json.loads((root/'hdmi-sync-result.json').read_text(encoding='utf8'))
        self.assertEqual(files[remote+'/mission_semantic_example.json'],calibrated)
        self.assertEqual(files[report['backup']+'/mission_semantic_example.json'],calibrated)
        self.assertEqual(files[remote+'/hdmi_camera.py'],b'# updated viewer\n')
        self.assertEqual(files[report['backup']+'/hdmi_camera.py'],b'# old viewer\n')
        self.assertEqual(files[remote+'/video.py'],b'# new source\n')
        self.assertEqual(files[remote+'/recording_overlay.py'],b'# new source\n')
        self.assertEqual(files[remote+'/usb_lidar.py'],b'# new source\n')
        self.assertEqual(files[remote+'/lidar_planning.py'],b'# new source\n')
        self.assertEqual(files.get(remote+'/lidar_map_view.py'),b'# radar map renderer\n')
        self.assertEqual(report['files']['lidar_planning.py'],hashlib.sha256(b'# new source\n').hexdigest())
        self.assertEqual(json.loads(files[remote+'/usb_lidar_config.json']),{'enabled':True})
        self.assertEqual(report['files']['recording_overlay.py'],hashlib.sha256(b'# new source\n').hexdigest())
        self.assertEqual(files[report['backup']+'/video.py'],b'# retired receiver\n')
        self.assertEqual(report['removed_files'],['uvc_capture.py'])
        self.assertNotIn(remote+'/uvc_capture.py',files)
        self.assertEqual(files[report['backup']+'/uvc_capture.py'],b'# retired UVC capture\n')
        self.assertIn('mission_semantic_example.json',report['preserved_configuration'])
        self.assertIn('route_example.json',report['preserved_configuration'])
        self.assertEqual(files[remote+'/route_example.json'],b'{"units":"mm","points":[[750,100]]}')
        final_camera=json.loads(files[remote+'/camera_config.json'])
        self.assertEqual(final_camera['primary'],'ov5647')
        self.assertEqual(final_camera['csi_index'],0 if override_camera else 1)
        self.assertEqual(final_camera['rotation'],0 if override_camera else 180)
        self.assertEqual(files[remote+'/recording_config.json'],recording_config)
        self.assertEqual(files[report['backup']+'/recording_config.json'],recording_config)
        self.assertIn('recording_config.json',report['preserved_configuration'])
        self.assertEqual(files[report['backup']+'/camera_config.json'],camera_config)
        self.assertEqual('camera_config.json' in report['preserved_configuration'],not override_camera)
        self.assertEqual(report['files']['mission_semantic_example.json'],hashlib.sha256(calibrated).hexdigest())
        script=files[remote+'/start-camera-viewer.sh']
        if expected_startup is not None:
            self.assertEqual(script,expected_startup)
            self.assertIn('start-camera-viewer.sh',report['preserved_configuration'])
        elif expected_options:
            self.assertIn(expected_options,script)
        else:
            for option in (b'--follow',b'--navigate',b'--mission'):self.assertNotIn(option,script)
        if startup is not None:self.assertEqual(files[report['backup']+'/start-camera-viewer.sh'],startup)
        self.assertEqual(report['files']['start-camera-viewer.sh'],hashlib.sha256(script).hexdigest())


if __name__=='__main__':unittest.main()
