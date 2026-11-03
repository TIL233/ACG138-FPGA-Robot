"""Exercise the FPGA map wire format through the two real reader entry points."""
import binascii
import struct
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

from robot_rebuild.rk import protocol as p


def envelope(kind, body):
    crc = binascii.crc_hqx(body, 0xffff)
    return bytes((0x5a, kind)) + body + struct.pack('<H', crc) + b'\r\n'


def map_frame(scan=0x1234, seq=9, age=100, flags=1, mask=1 | (1 << 35),
              distances=None, version=1):
    if distances is None:
        distances = (175,) + (65535,) * 34 + (7800,)
    body = (struct.pack('<BBBHH', version, seq, flags, scan, age)
            + mask.to_bytes(5, 'little') + struct.pack('<36H', *distances))
    return envelope(0xc9, body)


class MapWireTests(unittest.TestCase):
    def parser(self):
        self.assertTrue(hasattr(p, 'LidarMapTelemetryParser'), 'missing C9 map reader')
        return p.LidarMapTelemetryParser()

    def test_fragmented_snapshot_recovers_from_corruption_and_noise(self):
        frame = map_frame()
        bad = bytearray(frame)
        bad[21] ^= 1
        self.assertEqual(len(frame), 90)
        for split in range(len(frame) + 1):
            parser = self.parser()
            prefix = parser.feed(b'\x00' * 200 + bad + frame[:split])
            maps = prefix + parser.feed(frame[split:])
            self.assertEqual(len(maps), 1)
            m = maps[0]
            self.assertEqual((m.seq, m.flags, m.scan_id, m.age_ms), (9, 1, 0x1234, 100))
            self.assertEqual((m.known_mask, len(m.ranges_mm)), (0x800000001, 36))
            self.assertEqual((m.ranges_mm[0], m.ranges_mm[35]), (175, 7800))
            self.assertEqual(parser.bad_frames, 1)

    def test_reserved_fields_bad_ranges_and_delimiters(self):
        for frame in (map_frame(version=2), map_frame(mask=1 << 36), map_frame(flags=3),
                      map_frame(flags=0, mask=1), map_frame(age=351),
                      map_frame(mask=1, distances=(119,) * 36),
                      map_frame(mask=1, distances=(8001,) * 36),
                      map_frame()[:-2] + b'xx'):
            with self.subTest(frame=frame.hex()):
                parser = self.parser()
                self.assertEqual(parser.feed(frame + map_frame(scan=3))[0].scan_id, 3)
                self.assertEqual(parser.bad_frames, 1)

    def test_links_do_not_revive_frozen_scans_or_keep_map_after_disconnect(self):
        self.parser()
        port = mock.Mock(in_waiting=90)
        with mock.patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=lambda *a, **k: port)}):
            serial = p.Link('unused')
        posix = p.PosixTelemetryLink('unused')
        posix.fd = 7
        self.assertIsNone(posix.lidar_map_telemetry)
        modules = {'termios': SimpleNamespace(error=OSError), 'fcntl': SimpleNamespace(), 'tty': SimpleNamespace()}
        with mock.patch('os.read') as read, mock.patch('os.close'), mock.patch.dict(sys.modules, modules):
            for now, message, live in (
                    (10., map_frame(scan=65535, age=250), True),
                    (10.08, map_frame(scan=65535, seq=10, age=1), True),
                    (10.12, map_frame(scan=65535, seq=11, age=1), False),
                    (10.13, map_frame(scan=0, seq=12, age=120), True),
                    (10.16, map_frame(scan=0, seq=13, age=340), True),
                    (10.18, map_frame(scan=0, seq=14, age=120), False),
                    (10.19, map_frame(scan=1, flags=0, mask=0), False),
                    (10.20, map_frame(scan=2, mask=0), True),
                    # 90 bytes of 115200 baud 8N1 consume over 7 ms after latching.
                    (10.21, map_frame(scan=3, age=349), False)):
                port.read.return_value = read.return_value = message
                with mock.patch.object(p.time, 'monotonic', return_value=now):
                    serial.poll()
                posix.poll(now)
                for link in (serial, posix):
                    self.assertEqual(link.lidar_map_live(now), live, (now, type(link)))
                    self.assertFalse(link.lidar_map_live(9))
                    self.assertIsNotNone(link.lidar_map_telemetry)
            read.return_value = b''
            posix.poll(10.3)
            port.read.side_effect = OSError('device disconnected')
            with self.assertRaises(OSError):
                serial.poll()
            for link in (serial, posix):
                self.assertIsNone(link.lidar_map_telemetry)
                self.assertFalse(link.lidar_map_live(10.3))

    def test_map_payload_cannot_forge_navigation_or_delay_real_telemetry(self):
        self.parser()
        port = mock.Mock(in_waiting=256)
        with mock.patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=lambda *a, **k: port)}):
            link = p.Link('unused')
        nav_body = bytearray(18)
        nav_body[0:2] = (1, 7)
        valid_nav = envelope(0xc3, nav_body)
        # The contents of unknown bins are not information. Even CRC-valid navigation
        # bytes inside these bins must never appear as separate navigation telemetry.
        ranges = valid_nav + bytes(72 - len(valid_nav))
        packet = map_frame(flags=0, mask=0, distances=struct.unpack('<36H', ranges))
        for chunk in (packet[:17], packet[17:46], packet[46:]):
            port.read.return_value = chunk
            link.poll()
        self.assertIsNone(link.nav_telemetry)
        bad_crc = bytearray(packet)
        bad_crc[-3] ^= 1
        port.read.return_value = bad_crc
        link.poll()
        self.assertIsNone(link.nav_telemetry, 'discard the payload of a damaged complete map')
        # Conversely, a map sync sequence in a complete legacy frame must be harmless.
        nav_body[7:10] = b'\x5a\xc9\x01'
        valid_nav = envelope(0xc3, nav_body)
        port.read.return_value = valid_nav
        link.poll()
        self.assertEqual(link.nav_telemetry.seq, 7)

    def test_bad_or_truncated_map_cannot_replay_keys_and_resynchronizes(self):
        def keys(seq, count):
            # v5 includes the follow, navigation and recording remote-key counters.
            body = bytearray(25)
            body[0:2] = (5, seq)
            struct.pack_into('<HHBH', body, 18, count, count, 6, count)
            return envelope(0xc3, body)

        embedded = keys(73, 42)
        real = keys(74, 5)
        raw = map_frame(flags=0, mask=0, distances=struct.unpack('<36H',
                        embedded + bytes(72 - len(embedded))))
        for damaged in (raw[:-2] + b'xx', raw[:-4] + b'xxxx', raw[:-1], raw[:-2], raw[:69]):
            with self.subTest(damage=len(damaged)):
                parser = self.parser()
                legacy = bytearray()
                stream = damaged + real
                # Expose the false navigation envelope before the true frame arrives.
                for i in range(0, len(stream), 3):
                    self.assertEqual(parser.feed(stream[i:i+3], legacy=legacy), [])
                received = p.NavTelemetryParser().feed(legacy)
                self.assertEqual([(t.seq, t.follow_count, t.nav_mode_count, t.record_count)
                                  for t in received], [(74, 5, 5, 5)])
                self.assertEqual(parser.bad_frames, 1)


if __name__ == '__main__':
    unittest.main()
