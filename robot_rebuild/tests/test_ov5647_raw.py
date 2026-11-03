"""Golden RAW10 packing, channel order, output framing and camera-only routing."""
from pathlib import Path
import sys,unittest,json,tempfile
from unittest import mock
import numpy as np,cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
from ov5647_raw_capture import unpack_raw10,fit_rgb,next_exposure
import camera_capture as capture


class RawCameraTests(unittest.TestCase):
    def test_compact_lsb_raw10_decodes_samples_and_ignores_row_padding(self):
        pixels=[[0,1023,512,1,64,128,256,768],[7,9,17,31,63,127,511,999]]
        data=bytearray()
        for row in pixels:
            for offset in (0,4):
                word=sum(row[offset+i]<<(10*i) for i in range(4))
                data.extend(word.to_bytes(5,'little'))
            data.extend(b'\xa5'*6)
        result=unpack_raw10(data,8,2,16)
        np.testing.assert_array_equal(result,np.array(pixels,dtype=np.uint16)>>2)
        self.assertEqual(result.dtype,np.uint8)
        data.clear()  # Returned pixels do not retain a view of the input pipe.
        self.assertEqual(int(result[0,1]),255)

    def test_partial_frames_invalid_stride_and_non_group_width_are_rejected(self):
        for payload,w,h,stride in [(b'1234',4,1,5),(b'12345',5,1,5),(b'1234',4,1,4),(b'',0,1,0)]:
            with self.assertRaises(ValueError):unpack_raw10(payload,w,h,stride)

    def test_bggr_sensor_is_converted_to_rgb_without_red_blue_swap(self):
        bayer=np.zeros((8,8),np.uint8)
        bayer[::2,::2]=20;bayer[::2,1::2]=40;bayer[1::2,::2]=40;bayer[1::2,1::2]=80
        rgb=cv2.cvtColor(bayer,cv2.COLOR_BayerBGGR2RGB)
        self.assertEqual(rgb[3,3].tolist(),[80,40,20])

    def test_letterbox_and_rotation_produce_exact_rgb_pipe_dimensions(self):
        rgb=np.full((8,8,3),100,np.uint8)
        for rotation in (0,90,180,270):
            output=fit_rgb(rgb,16,8,rotation)
            self.assertEqual(output.shape,(8,16,3));self.assertEqual(len(output.tobytes()),16*8*3)
            self.assertFalse(output[:,:4].any());self.assertFalse(output[:,12:].any())
            self.assertTrue((output[:,4:12]==100).all())

    def test_exposure_feedback_is_bounded_and_never_changes_frame_blanking(self):
        self.assertEqual(next_exposure(400,32,64,(4,1000),(16,128)),(400,32))
        exposure,gain=1000,120
        for _ in range(20):exposure,gain=next_exposure(exposure,gain,0,(4,1000),(16,128))
        self.assertEqual((exposure,gain),(1000,128))
        for _ in range(100):exposure,gain=next_exposure(exposure,gain,255,(4,1000),(16,128))
        self.assertEqual((exposure,gain),(4,16))

    def test_raw_backend_targets_sensor_and_cif_not_isp_capture(self):
        route=dict(cif='/dev/media0',isp='/dev/media2',sensor='m00_b_ov5647 9-0036',raw='/dev/video0',video='/dev/video22')
        with mock.patch.object(capture,'discover_csi',return_value=route),mock.patch.object(capture,'require_isp_ready'):
            cmd=capture.capture_command(capture.CameraConfig(rotation=180),width=640,height=360,run=lambda _:'/dev/v4l-subdev2')
        self.assertIn('/dev/video0',cmd);self.assertNotIn('/dev/video22',cmd)
        self.assertIn('/dev/v4l-subdev2',cmd);self.assertEqual(cmd[cmd.index('--rotation')+1],'180')
        self.assertTrue(any(arg.endswith('ov5647_raw_capture.py') for arg in cmd))

    def test_raw_ready_file_only_accepts_live_preparer_and_matching_backend(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);runtime=root/'run';runtime.mkdir();proc=root/'proc';(proc/'50').mkdir(parents=True)
            (proc/'50/cmdline').write_bytes(b'python3\0prepare_ov5647.py\0--serve\0')
            (runtime/'ready.json').write_text(json.dumps(dict(index=0,width=1296,height=972,backend='raw',parent_pid=50,aiq_pid=None)))
            capture.require_isp_ready(capture.CameraConfig(),runtime=runtime,proc_root=proc)
            with self.assertRaises(RuntimeError):capture.require_isp_ready(capture.CameraConfig(csi_backend='isp'),runtime=runtime,proc_root=proc)


if __name__=='__main__':unittest.main()
