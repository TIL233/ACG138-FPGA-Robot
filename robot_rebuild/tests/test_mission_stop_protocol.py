"""STOP generations survive route/control mode transitions on the same wire."""
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rk'))
from protocol import NavTelemetry, NavTelemetryParser, TelemetryParser, crc16


def nav_frame(seq, state, epoch, flags=0xF8):
    body = struct.pack('<BBBBBBiihBB', 1, seq, (epoch << 4) | state, 0, 1,
                       flags, 123, -456, 789, 21, 0)
    return b'\x5a\xc3' + body + struct.pack('<H', crc16(body)) + b'\r\n'


class MissionStopProtocolTests(unittest.TestCase):
    def test_all_epoch_and_state_values_round_trip_without_changing_layout(self):
        parser = NavTelemetryParser()
        for epoch in range(16):
            for state in range(16):
                frame = nav_frame(epoch * 16 + state, state, epoch)
                self.assertEqual(len(frame), 24)
                t, = parser.feed(frame)
                self.assertEqual((t.state, t.stop_epoch), (state, epoch))
                self.assertTrue(t.supported and t.mission_supported)
                self.assertEqual((t.index, t.count, t.x_mm, t.y_mm, t.heading,
                                  t.ack_seq, t.ack_status), (0, 1, 123, -456, 789, 21, 0))

    def test_legacy_navigation_has_no_mission_capability_and_default_epoch(self):
        original = NavTelemetry(7, 2, 0, 1, 0xB8, 123, -456, 789, 21, 0)
        decoded, = NavTelemetryParser().feed(nav_frame(7, 2, 0, flags=0xB8))
        self.assertEqual(decoded, original)
        self.assertTrue(decoded.supported)
        self.assertFalse(decoded.mission_supported)
        self.assertEqual(decoded.stop_epoch, 0)

    def test_modulo_wrap_is_an_observable_new_stop(self):
        frames = nav_frame(254, 3, 15) + nav_frame(0, 3, 0)
        before, after = NavTelemetryParser().feed(frames)
        self.assertNotEqual(before.stop_epoch, after.stop_epoch)
        self.assertEqual((after.stop_epoch - before.stop_epoch) & 15, 1)

    def test_fragmentation_crc_recovery_and_legacy_parser_stay_independent(self):
        corrupted = bytearray(nav_frame(2, 4, 12))
        corrupted[4] ^= 0x10
        valid = nav_frame(3, 4, 13)
        stream = bytes(corrupted) + valid
        for split in range(len(stream) + 1):
            parser = NavTelemetryParser()
            got = parser.feed(stream[:split]) + parser.feed(stream[split:])
            self.assertEqual(len(got), 1)
            self.assertEqual((got[0].state, got[0].stop_epoch), (4, 13))
            self.assertEqual(parser.bad_frames, 1)
            self.assertEqual(TelemetryParser().feed(stream), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
