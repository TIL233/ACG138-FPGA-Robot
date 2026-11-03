"""Exercise CSI routing, capture framing and failure behavior without hardware."""
from pathlib import Path
import json
import sys
import tempfile
import threading
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'rk'))
import camera_capture as capture
import prepare_ov5647 as prepare


class RoutingTests(unittest.TestCase):
    def test_capture_waits_for_matching_live_isp_owners_not_a_stale_ready_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);runtime=root/'run';runtime.mkdir();proc=root/'proc'
            config=capture.CameraConfig(csi_backend='isp')
            with self.assertRaisesRegex(RuntimeError,'ISP not ready'):
                capture.require_isp_ready(config,runtime=runtime,proc_root=proc)
            state=dict(index=0,width=1296,height=972,backend='isp',parent_pid=100,aiq_pid=101)
            (runtime/'ready.json').write_text(json.dumps(state))
            for pid,cmd in [(100,b'python3\0prepare_ov5647.py\0--serve\0'),(101,b'rkaiq-single-camera\0')]:
                (proc/str(pid)).mkdir(parents=True);(proc/str(pid)/'cmdline').write_bytes(cmd)
            capture.require_isp_ready(config,runtime=runtime,proc_root=proc)
            for invalid in [dict(state,width=640),dict(state,parent_pid=True),dict(state,aiq_pid=102)]:
                (runtime/'ready.json').write_text(json.dumps(invalid))
                with self.assertRaisesRegex(RuntimeError,'ISP not ready'):
                    capture.require_isp_ready(config,runtime=runtime,proc_root=proc)
            (runtime/'ready.json').write_text(json.dumps(state))
            (proc/'101/cmdline').write_bytes(b'unrelated-process\0')
            with self.assertRaisesRegex(RuntimeError,'ISP not ready'):
                capture.require_isp_ready(config,runtime=runtime,proc_root=proc)

    def topology(self, index=0):
        suffix = '1' if index else ''
        graphs = {'/dev/media3': f'bus info platform:rkcif-mipi-lvds{suffix}\n- entity 7: m0{index}_b_ov5647 8-0036 (1 pad, 1 link)\n',
                  '/dev/media7': f'bus info platform:rkisp{index}-vir0\n'}
        def run(argv):
            if argv[-1] == '-p': return graphs[argv[2]]
            if argv[-1] == 'rkisp_mainpath': return '/dev/video42'
            if argv[-1] == 'stream_cif_mipi_id0': return '/dev/video17'
            raise AssertionError(argv)
        return graphs, run

    def test_both_supported_csi_ports_discover_dynamic_video_numbers(self):
        for index in (0, 1):
            graphs, run = self.topology(index)
            route = capture.discover_csi(index, run, list(graphs))
            self.assertEqual(route['video'], '/dev/video42')
            self.assertEqual(route['raw'], '/dev/video17')
            self.assertEqual(route['isp'], '/dev/media7')
            calls = []
            prepare.configure_route(route, lambda argv: calls.append(argv))
            self.assertIn('m0'+str(index)+'_b_ov5647', calls[0][-1])
            self.assertTrue(any('pixelformat=NV12' in arg for call in calls for arg in call))

    def test_missing_sensor_wrong_bus_and_ambiguous_bus_are_rejected(self):
        graphs, run = self.topology()
        graphs['/dev/media3'] = 'bus info platform:rkcif-mipi-lvds\n- entity 7: wrong_sensor (1 pad)'
        with self.assertRaisesRegex(RuntimeError, 'sensor not found'):
            capture.discover_csi(0, run, list(graphs))
        graphs['/dev/media3'] = 'bus info platform:rkcif-mipi-lvds1'
        with self.assertRaisesRegex(RuntimeError, 'found 0'):
            capture.discover_csi(0, run, list(graphs))
        graphs['/dev/media4'] = graphs['/dev/media3']
        with self.assertRaisesRegex(RuntimeError, 'found 2'):
            capture.find_media('rkcif-mipi-lvds1', run, list(graphs))

    def test_raw_node_cannot_be_substituted_for_isp_rgb_capture(self):
        route = dict(video='/dev/video42')
        with mock.patch.object(capture, 'discover_csi', return_value=route) as discovery:
            cmd = capture.capture_command(capture.CameraConfig(primary='ov5647', csi_index=1, rotation=180,csi_backend='isp'))
        discovery.assert_called_once_with(1, capture.run_command)
        self.assertIn('device=/dev/video42', cmd)
        self.assertIn('video/x-raw,format=NV12,width=1280,height=960', cmd)
        self.assertIn('method=rotate-180', cmd)
        self.assertNotIn('jpegdec', cmd)
        self.assertIn('leaky=downstream', cmd)
        self.assertIn('video/x-raw,format=RGB,width=1280,height=720,pixel-aspect-ratio=1/1', cmd)

    def test_old_usb_capture_is_disabled_and_configuration_migrates(self):
        with mock.patch.object(capture, 'discover_csi', side_effect=AssertionError('must not probe')):
            with self.assertRaisesRegex(ValueError,'Only OV5647'):
                capture.capture_command(capture.CameraConfig(primary='usb'))
        migrated=capture.ov5647_only(capture.CameraConfig(primary='usb',csi_index=1,rotation=180,fpga_monitor=True))
        self.assertEqual((migrated.primary,migrated.csi_index,migrated.rotation,migrated.preview,migrated.fpga_monitor),('ov5647',1,180,'off',False))

    def test_config_schema_rejects_wrong_channel_angle_unknown_keys(self):
        for data in ({'csi_index':2}, {'csi_index':True}, {'rotation':45}, {'primary':'raw'}, {'usb_device':'/dev/video0 ! fakesink'}, {'fpga_monitor':1}, {'typo':1}, {'csi_width':1280,'csi_height':720}, {'csi_width':True,'csi_height':480}):
            with self.subTest(data=data), tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'camera.json'
                path.write_text(json.dumps(data))
                with self.assertRaises((ValueError, TypeError)): capture.load_config(path)

    def test_source_size_is_consistent_across_sensor_raw_isp_and_rgb_pipeline(self):
        route=dict(cif='/dev/media0',isp='/dev/media2',sensor='m00_b_ov5647 9-0036',raw='/dev/video0',video='/dev/video22')
        calls=[]
        prepare.configure_route(route,lambda argv:calls.append(argv),width=640,height=480)
        self.assertTrue(any('fmt:SBGGR10_1X10/640x480' in arg for call in calls for arg in call))
        self.assertTrue(any('width=640,height=480,pixelformat=BG10' in arg for call in calls for arg in call))
        self.assertTrue(any('width=640,height=480,pixelformat=NV12' in arg for call in calls for arg in call))
        self.assertFalse(any('1296' in arg or '1920' in arg for call in calls for arg in call))
        with mock.patch.object(capture,'discover_csi',return_value=route):
            cmd=capture.capture_command(capture.CameraConfig(csi_width=640,csi_height=480,csi_backend='isp'))
        self.assertIn('video/x-raw,format=NV12,width=640,height=480',cmd)
        self.assertIn('video/x-raw,format=RGB,width=1280,height=720,pixel-aspect-ratio=1/1',cmd)

    def test_binned_native_mode_is_center_cropped_to_aligned_isp_output(self):
        route=dict(cif='/dev/media0',isp='/dev/media2',sensor='m00_b_ov5647 9-0036',raw='/dev/video0',video='/dev/video22')
        calls=[];prepare.configure_route(route,lambda argv:calls.append(argv))
        self.assertIn('"m00_b_ov5647 9-0036":0 [fmt:SBGGR10_1X10/1296x972]',calls[0])
        self.assertTrue(any('crop:(8,6)/1280x960' in arg for call in calls for arg in call))
        self.assertTrue(any('width=1296,height=972,pixelformat=BG10' in arg for call in calls for arg in call))
        self.assertTrue(any('width=1280,height=960,pixelformat=NV12' in arg for call in calls for arg in call))

    def test_explicit_missing_config_cannot_silently_select_usb(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'missing.json'
            self.assertEqual(capture.load_config(path).primary, 'ov5647')
            with self.assertRaises(FileNotFoundError): capture.load_config(path, missing_ok=False)

    def test_probe_is_read_only_and_apply_does_not_start_aiq_or_any_controls(self):
        route = dict(index=0, video='/dev/video42')
        with mock.patch.object(prepare, 'discover_csi', return_value=route), \
             mock.patch.object(prepare, 'configure_route') as configure, \
             mock.patch.object(prepare, 'configure_raw_route') as raw_configure, \
             mock.patch.object(prepare, 'serve') as serve, mock.patch('builtins.print'):
            prepare.main([])
            configure.assert_not_called(); serve.assert_not_called()
            prepare.main(['--apply'])
            raw_configure.assert_called_once_with(route,width=1296,height=972)
            configure.assert_not_called();serve.assert_not_called()

    def test_existing_iq_calibration_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            source = base/'bsp.json'; source.write_text('{"sensor":"OV5647"}')
            destination = prepare.install_iq(source, 0, base/'iq')
            prepare.install_iq(source, 0, base/'iq')
            destination.write_text('{"custom":"calibrated"}')
            with self.assertRaisesRegex(RuntimeError, 'Existing IQ differs'):
                prepare.install_iq(source, 0, base/'iq')
            self.assertEqual(destination.read_text(), '{"custom":"calibrated"}')

    def test_aiq_supervision_leaves_an_existing_service_untouched(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory)/'aiq'; executable.write_bytes(b'fake executable')
            with mock.patch.dict(sys.modules, {'fcntl':mock.Mock(LOCK_EX=2, LOCK_NB=4)}), \
                 mock.patch.object(prepare.os,'access',return_value=True), \
                 mock.patch.object(prepare,'existing_aiq',return_value=[123]), \
                 mock.patch.object(prepare,'configure_route') as configure, \
                 mock.patch.object(prepare.subprocess,'Popen') as start:
                with self.assertRaisesRegex(RuntimeError,'Another AIQ server'):
                    prepare.serve({'index':0},executable,'missing-iq',Path(directory)/'runtime')
                start.assert_not_called(); configure.assert_not_called()

    def test_aiq_exit_before_preparation_fails_and_restores_signal_handlers(self):
        route = dict(index=0,isp='/dev/media7',sensor='m00_b_ov5647 9-0036')
        process = mock.Mock(); process.poll.return_value = 1
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory)/'aiq'; executable.write_bytes(b'fake executable')
            with mock.patch.dict(sys.modules, {'fcntl':mock.Mock(LOCK_EX=2, LOCK_NB=4)}), \
                 mock.patch.object(prepare.os,'access',return_value=True), \
                 mock.patch.object(prepare,'existing_aiq',return_value=[]), \
                 mock.patch.object(prepare,'install_iq'), \
                 mock.patch.object(prepare,'configure_route'), \
                 mock.patch.object(prepare,'run_command',return_value='/dev/video30'), \
                 mock.patch.object(prepare.signal,'signal',return_value='old-handler') as signals, \
                 mock.patch.object(prepare.subprocess,'Popen',return_value=process):
                with self.assertRaisesRegex(RuntimeError,'AIQ exited'):
                    prepare.serve(route,executable,'iq',Path(directory)/'runtime')
                self.assertEqual(signals.call_count,4)
                process.terminate.assert_not_called()  # The process has already exited.

    def test_single_aiq_uses_discovered_sensor_and_params_node_without_global_library_override(self):
        route=dict(index=0,isp='/dev/media7',sensor='m00_b_ov5647 9-0036')
        run=mock.Mock(return_value='/dev/video30')
        env=prepare.aiq_environment(route,run)
        run.assert_called_once_with(['media-ctl','-d','/dev/media7','-e','rkisp-input-params'])
        self.assertEqual(env['ACG138_SENSOR'],route['sensor'])
        self.assertEqual(env['ACG138_ISP_PARAMS'],'/dev/video30')
        self.assertEqual(env['normal_no_read_back'],'0')
        self.assertEqual((env['ACG138_WIDTH'],env['ACG138_HEIGHT']),('1296','972'))
        self.assertEqual(env.get('LD_LIBRARY_PATH'),prepare.os.environ.get('LD_LIBRARY_PATH'))
        for invalid in ('/dev/video30 extra','/dev/v4l-subdev3','/tmp/video30',''):
            with self.assertRaises(ValueError):prepare.aiq_environment(route,lambda _:invalid)


class CaptureTests(unittest.TestCase):
    def worker(self):
        worker = capture.CaptureReader.__new__(capture.CaptureReader)
        worker.config = capture.CameraConfig()
        worker.kind = 'ov5647'; worker.width = 2; worker.height = 1
        worker.stop_event = threading.Event(); worker.lock = threading.Lock()
        worker.latest = (None, 0., 0, '')
        return worker

    def test_partial_reads_publish_only_a_full_frame(self):
        worker = self.worker(); process = mock.Mock()
        process.poll.return_value = 0; process.stdout.fileno.return_value = 17
        pieces = [b'ab', b'cdef']
        def read(*_):
            result = pieces.pop(0)
            if not pieces: worker.stop_event.set()
            return result
        with mock.patch.object(capture, 'capture_command', return_value=['fake']), \
             mock.patch.object(capture.subprocess, 'Popen', return_value=process), \
             mock.patch.object(capture.select, 'select', return_value=([17], [], [])), \
             mock.patch.object(capture.os, 'read', side_effect=read):
            worker._run()
        self.assertEqual(worker.snapshot()[0], b'abcdef')
        self.assertEqual(worker.snapshot()[2], 1)
        process.stdout.close.assert_called_once()

    def test_disconnect_clears_previous_frame_and_explains_failure(self):
        worker = self.worker(); worker.latest = (b'old', 1., 1, '')
        process = mock.Mock(); process.poll.return_value = 0
        def wait(_):
            worker.stop_event.set()
        with mock.patch.object(capture, 'capture_command', return_value=['fake']), \
             mock.patch.object(capture.subprocess, 'Popen', return_value=process), \
             mock.patch.object(capture.select, 'select', return_value=([17], [], [])), \
             mock.patch.object(capture.os, 'read', return_value=b''), \
             mock.patch.object(worker.stop_event, 'wait', side_effect=wait):
            worker._run()
        self.assertIsNone(worker.snapshot()[0])
        self.assertIn('disconnected', worker.snapshot()[3])

    def test_hung_pipe_is_terminated_after_frame_timeout(self):
        worker = self.worker(); process = mock.Mock(); process.poll.return_value = None
        def wait(_): worker.stop_event.set()
        with mock.patch.object(capture, 'capture_command', return_value=['fake']), \
             mock.patch.object(capture.subprocess, 'Popen', return_value=process), \
             mock.patch.object(capture.time, 'monotonic', side_effect=[10., 14.]), \
             mock.patch.object(worker.stop_event, 'wait', side_effect=wait):
            worker._run()
        process.terminate.assert_called_once()
        self.assertIn('timed out', worker.snapshot()[3])


if __name__ == '__main__': unittest.main()
