import sys
from pathlib import Path
import struct
import unittest
import zlib
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'rk'))
from protocol import command_packet, crc16, TelemetryParser
from video import FrameAssembler, HEADER, rgb565_to_bgr
from detector import decode_predictions


class ProtocolTests(unittest.TestCase):
    def test_known_crc(self):
        self.assertEqual(crc16(b'123456789'), 0x29B1)
        self.assertEqual(len(command_packet(255, True, 6, 64)), 12)
        for cmd, speed in [(7, 0), (1, 65), (-1, 0)]:
            with self.assertRaises(ValueError):
                command_packet(0, True, cmd, speed)

    def test_telemetry_chunks_corruption_resync(self):
        body = struct.pack('<BBBBBBHHhhhhhH', 1, 254, 3, 0x9F, 1, 16,
                           199, 65535, -16, 16, -1234, 250, 17999, 0x800)
        frame = b'\x5a\xa5'+body+struct.pack('<H', crc16(body))+b'\r\n'
        self.assertEqual(len(frame), 28)
        for split in range(29):
            parser = TelemetryParser()
            broken = bytearray(frame)
            broken[8] ^= 1
            output = parser.feed(b'noise'+bytes(broken)+frame[:split])+parser.feed(frame[split:])
            self.assertEqual(len(output), 1)
            self.assertEqual(output[0].distance_l, 199)
            self.assertEqual(output[0].roll, -1234)
            self.assertEqual(output[0].fault, 0x800)
            self.assertTrue(output[0].sensors_ready)
        parser = TelemetryParser()
        parser.feed(bytes(100000))
        self.assertLess(len(parser.buffer), 2)


def row(seq, index, data=b'\x00\xf8\xe0\x07'):
    return HEADER.pack(b'RBV1', seq, index, 2, 2, 1, 0, 4)+data


class VideoTests(unittest.TestCase):
    def test_actual_rtl_ethernet_output(self):
        trace=Path(__file__).with_name('ethernet.hex')
        if not trace.exists():
            self.skipTest('run ModelSim tb_video first')
        assembler=FrameAssembler(4, 2)
        completed=[]
        for line in trace.read_text().splitlines():
            frame=bytes.fromhex(line)
            self.assertEqual(frame[:8], b'\x55'*7+b'\xd5')
            self.assertEqual(zlib.crc32(frame[8:-4]), int.from_bytes(frame[-4:], 'little'))
            ip=frame[22:42]
            checksum=sum(struct.unpack('!10H', ip))
            while checksum>>16:
                checksum=(checksum&65535)+(checksum>>16)
            self.assertEqual(checksum, 65535)
            self.assertEqual(int.from_bytes(frame[46:48], 'big'), 32)
            result=assembler.push(frame[50:-4])
            if result is not None:
                completed.append(result)
        self.assertEqual(len(completed), 1)
        self.assertEqual(completed[0], b'\x00\xf8\xe0\x07\x1f\x00\xff\xff'*2)

    def test_complete_and_colors(self):
        assembler = FrameAssembler(2, 2)
        self.assertIsNone(assembler.push(row(65535, 0), 0))
        data = assembler.push(row(65535, 1, b'\x1f\x00\xff\xff'), 0.01)
        colors = rgb565_to_bgr(data, 2, 2).tolist()
        self.assertEqual(colors, [[[0, 0, 255], [0, 255, 0]], [[255, 0, 0], [255, 255, 255]]])
        self.assertIsNone(assembler.push(row(0, 0), 0.02))
        self.assertIsNotNone(assembler.push(row(0, 1), 0.03))

    def test_loss_duplicates_mixed_frames_and_bad_lengths(self):
        a = FrameAssembler(2, 2)
        self.assertIsNone(a.push(row(1, 1), 0))
        a.push(row(2, 0), 0.1)
        self.assertIsNone(a.push(row(1, 1), 0.11))
        self.assertIsNotNone(a.push(row(2, 1), 0.12))
        self.assertIsNone(a.push(row(2, 0), 0.13))
        self.assertIsNone(a.push(row(2, 1), 0.14))
        a.push(row(3, 0), 0.2)
        self.assertIsNone(a.push(row(3, 1)+b'x', 0.21))
        self.assertIsNone(a.push(row(3, 1), 0.22))
        a.push(row(4, 0), 0.3)
        self.assertIsNone(a.push(row(4, 1), 0.9))
        a.push(row(0, 0), 3)  # sender reboot after idle
        self.assertIsNotNone(a.push(row(0, 1), 3.01))


class DetectorTests(unittest.TestCase):
    def test_only_person_bottle_no_color_and_nms(self):
        output = np.zeros((1, 84, 5), np.float32)
        output[0, :4, :] = np.array([[160, 160, 100, 120]]*5).T
        output[0, 4+0, 0] = 0.9
        output[0, 4+0, 1] = 0.8  # duplicate person
        output[0, 4+39, 2] = 0.85
        output[0, 4+41, 3] = 0.99  # cup excluded
        output[0, 4+39, 4] = 0.2  # low confidence excluded
        out = decode_predictions(output, 0.5, (0, 40), (480, 640, 3))
        self.assertEqual([d[0] for d in out], [0, 39])
        self.assertEqual(out[0][2], (220, 120, 420, 360))

    def test_reject_wrong_model_shape(self):
        with self.assertRaises(ValueError):
            decode_predictions(np.zeros((1, 300, 6)), 1, (0, 0), (480, 640, 3))


if __name__ == '__main__':
    unittest.main(verbosity=2)
