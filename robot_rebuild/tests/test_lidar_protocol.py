import struct
import unittest
from unittest import mock
from types import SimpleNamespace
from robot_rebuild.rk.protocol import LidarTelemetryParser, crc16, Link, PosixTelemetryLink


def frame(seq=1, flags=63, scan=7, distances=(500, 1200, 900, 2000), zero=0):
    body = struct.pack('<BB9H', 1, seq, flags, scan, *distances, 123, 2, zero)
    return b'\x5a\xb7' + body + struct.pack('<H', crc16(body)) + b'\r\n'


class LidarProtocolTests(unittest.TestCase):
    def test_only_front_left_right_are_exposed_even_with_legacy_rear_reading(self):
        t = LidarTelemetryParser().feed(frame())[0]
        self.assertEqual(t.distances(), dict(front=500, left=1200, right=900))

    def test_all_fragment_boundaries(self):
        raw = frame()
        for split in range(len(raw)+1):
            p = LidarTelemetryParser()
            result = p.feed(raw[:split]) + p.feed(raw[split:])
            self.assertEqual(len(result), 1)
            t = result[0]
            self.assertEqual(t.distances(), dict(front=500, left=1200, right=900))
            self.assertTrue(t.control_enabled)

    def test_bad_crc_recovers_with_mixed_telemetry_noise(self):
        raw = bytearray(frame()); raw[10] ^= 1
        p = LidarTelemetryParser()
        result = p.feed(b'\x5a\xe5noise' + raw + frame(seq=2))
        self.assertEqual([t.seq for t in result], [2])
        self.assertEqual(p.bad_frames, 1)

    def test_unknown_is_not_clear(self):
        t = LidarTelemetryParser().feed(frame(flags=16, distances=(65535,)*4))[0]
        self.assertEqual(t.distances(), dict.fromkeys(('front', 'left', 'right')))
        self.assertFalse(t.control_enabled)

    def test_stale_flags_hide_old_ranges(self):
        t = LidarTelemetryParser().feed(frame(flags=0))[0]
        self.assertFalse(t.fresh)
        self.assertIsNone(t.distances()['front'])

    def test_semantically_invalid_frames(self):
        for raw in (frame(flags=256), frame(zero=23040), frame(distances=(0,1200,900,2000))):
            p = LidarTelemetryParser()
            self.assertEqual(p.feed(raw), [])
            self.assertEqual(p.bad_frames, 1)

    def test_frozen_scan_does_not_stay_live(self):
        for cls in (Link, PosixTelemetryLink):
            link = cls.__new__(cls)
            link.lidar_telemetry = LidarTelemetryParser().feed(frame())[0]
            link.last_lidar_rx = 10.0
            self.assertTrue(link.lidar_live(10.2))
            self.assertFalse(link.lidar_live(10.5))
            self.assertFalse(link.lidar_live(9.9))

    def test_bounded_noise_and_partial_header(self):
        p = LidarTelemetryParser()
        self.assertEqual(p.feed(b'\x00'*100000 + b'\x5a'), [])
        self.assertEqual(len(p.buffer), 1)
        self.assertEqual(len(p.feed(frame()[1:])), 1)

    def test_disconnect_resets_lidar(self):
        # No serial I/O: fd=None. Use the production disconnect reset path.
        link = PosixTelemetryLink('unused')
        link.lidar_telemetry = LidarTelemetryParser().feed(frame())[0]
        link.last_lidar_rx = 10.0
        link.last_lidar_scan = 7
        with mock.patch.dict('sys.modules', {'termios': SimpleNamespace()}):
            link._disconnect()
        self.assertIsNone(link.lidar_telemetry)
        self.assertIsNone(link.last_lidar_scan)
        self.assertFalse(link.lidar_live(10.1))

    def test_poll_repeated_telemetry_cannot_refresh_stuck_scanner(self):
        port = mock.Mock(in_waiting=26)
        with mock.patch.dict('sys.modules', {'serial': SimpleNamespace(Serial=mock.Mock(return_value=port))}):
            link = Link('unused')
        for now, seq, scan in ((10.0,1,7),(10.2,2,7),(10.5,3,7),(10.6,4,8)):
            port.read.return_value = frame(seq=seq, scan=scan)
            with mock.patch('robot_rebuild.rk.protocol.time.monotonic', return_value=now):
                link.poll()
            self.assertEqual(link.lidar_live(now), now != 10.5)
        port.write.assert_not_called()


if __name__ == '__main__':
    unittest.main()
