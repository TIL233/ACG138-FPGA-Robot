"""Read-only motion snapshots: wire layout, coexistence and sample freshness."""
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rk'))
import protocol as p


def packet(seq=7, stamp=1000, flags=0x4f, version=1):
    body = struct.pack('<BBHI12hii', version, seq, flags, stamp,
                       -17900, 17900, 17800, 100, 17700, -200,
                       9, 11, 8, 12, 2, -900, 1000, -2000)
    return b'\x5a\xe5' + body + struct.pack('<H', p.crc16(body)) + b'\r\n'


class MotionTests(unittest.TestCase):
    def test_independent_encoder_marker_does_not_claim_global_fusion_or_holding(self):
        t=p.MotionTelemetryParser().feed(packet(flags=0x60b))[0]
        self.assertTrue(t.ready and t.encoder_available and t.braked)
        self.assertFalse(t.fused or t.holding)
        old=p.MotionTelemetryParser().feed(packet(flags=0x0b))[0]
        self.assertTrue(old.ready)
        self.assertFalse(old.encoder_available)

    def test_all_fragment_boundaries_and_signed_layout(self):
        wire = packet()
        self.assertEqual(len(wire), 46)
        for split in range(len(wire)+1):
            parser = p.MotionTelemetryParser()
            frames = parser.feed(wire[:split])+parser.feed(wire[split:])
            self.assertEqual(len(frames), 1)
            t = frames[0]
            self.assertEqual((t.seq, t.flags, t.timestamp_ms), (7, 0x4f, 1000))
            self.assertEqual((t.imu_heading, t.heading, t.encoder_heading), (-17900,17900,17800))
            self.assertEqual((t.goal,t.error,t.trim,t.yaw_rate), (17700,-200,2,-900))
            self.assertEqual((t.position_l,t.position_r), (1000,-2000))
            self.assertTrue(t.ready and t.fused and t.holding)
            self.assertFalse(t.disagreement)

    def test_crc_version_terminator_noise_and_recovery(self):
        for index in (0, 1, 2, 18, 42, 44, 45):
            bad = bytearray(packet());bad[index] ^= 1
            parser = p.MotionTelemetryParser()
            frames = parser.feed(b'noise'+bytes(bad)+packet(version=2)+packet(seq=8))
            self.assertEqual([t.seq for t in frames], [8])
        parser.feed(bytes(100000))
        self.assertLess(len(parser.buffer), 2)

    def test_new_stream_preserves_legacy_and_nav_parsers(self):
        legacy_body=struct.pack('<BBBBBBHHhhhhhH',1,2,0,0x1f,0,8,1000,1000,0,0,0,0,0,0)
        legacy=b'\x5a\xa5'+legacy_body+struct.pack('<H',p.crc16(legacy_body))+b'\r\n'
        nav_body=struct.pack('<BBBBBBiihBB',1,3,0,0,1,0xb8,0,0,0,0,0)
        nav=b'\x5a\xc3'+nav_body+struct.pack('<H',p.crc16(nav_body))+b'\r\n'
        stream=packet()+legacy+packet(seq=8)+nav
        for split in range(len(stream)+1):
            for parser, expected in ((p.MotionTelemetryParser(),[7,8]),
                                     (p.TelemetryParser(),[2]),(p.NavTelemetryParser(),[3])):
                frames=parser.feed(stream[:split])+parser.feed(stream[split:])
                self.assertEqual([t.seq for t in frames],expected)

    def link(self):
        port=mock.Mock(is_open=True,in_waiting=46)
        with mock.patch.dict(sys.modules, {'serial':SimpleNamespace(Serial=mock.Mock(return_value=port))}):
            link=p.Link('virtual',writable=False)
        return link,port

    def test_uart_sequence_cannot_keep_a_frozen_fpga_sample_fresh(self):
        link,port=self.link()
        port.read.return_value=packet(seq=1,stamp=1000)
        with mock.patch.object(p.time,'monotonic',return_value=1):
            link.poll();self.assertTrue(link.motion_live())
        port.read.return_value=packet(seq=2,stamp=1000)
        with mock.patch.object(p.time,'monotonic',return_value=2):
            link.poll();self.assertFalse(link.motion_live())
        port.read.return_value=packet(seq=3,stamp=1020)
        with mock.patch.object(p.time,'monotonic',return_value=2):
            link.poll();self.assertTrue(link.motion_live())
        port.write.assert_not_called()

    def test_invalid_flags_override_recent_timestamp(self):
        link,port=self.link()
        for flags in (0,1,3,0x19,0x1f):
            port.read.return_value=packet(flags=flags)
            with mock.patch.object(p.time,'monotonic',return_value=1):
                link.poll();self.assertFalse(link.motion_live())
        port.read.return_value=packet(flags=0x0b)
        with mock.patch.object(p.time,'monotonic',return_value=1):
            link.poll();self.assertTrue(link.motion_live())
            self.assertFalse(link.motion_telemetry.fused)

    def test_fpga_timestamp_wrap_is_a_new_sample(self):
        link,port=self.link()
        for now,stamp in ((1,0xfffffff0),(2,4)):
            port.read.return_value=packet(stamp=stamp)
            with mock.patch.object(p.time,'monotonic',return_value=now):
                link.poll();self.assertTrue(link.motion_live())

    def test_posix_disconnect_discards_motion_state(self):
        link=p.PosixTelemetryLink('virtual')
        link.motion_telemetry=p.MotionTelemetryParser().feed(packet())[0]
        link.last_motion_rx=1;link.last_motion_stamp=1000
        with mock.patch.dict(sys.modules, {'termios':SimpleNamespace()}):
            link._disconnect()
        self.assertIsNone(link.motion_telemetry)
        self.assertIsNone(link.last_motion_stamp)
        self.assertFalse(link.motion_live(1))


if __name__=='__main__':unittest.main()
