"""Navigation framing, capability/ACK freshness and receive-only ownership."""
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rk'))
import protocol as p


def nav_frame(seq=7, flags=0xB9, ack_seq=3, ack_status=0):
    body = struct.pack('<BBBBBBiihBB', 1, seq, 2, 1, 3, flags,
                       -1000, 2500, -9000, ack_seq, ack_status)
    return b'\x5a\xc3' + body + struct.pack('<H', p.crc16(body)) + b'\r\n'


def legacy_frame(seq=7):
    body = struct.pack('<BBBBBBHHhhhhhH', 1, seq, 0, 0x1F, 0, 8,
                       1000, 1000, 0, 0, 0, 0, 0, 0)
    return b'\x5a\xa5' + body + struct.pack('<H', p.crc16(body)) + b'\r\n'


class NavigationPackets(unittest.TestCase):
    def test_serial_link_live_accepts_navigation_clock_and_rejects_future_feedback(self):
        link = p.Link.__new__(p.Link)
        link.telemetry = object()
        link.last_rx = 10.
        self.assertTrue(link.live(10.2))
        self.assertFalse(link.live(10.5))
        self.assertFalse(link.live(9.9))
        with mock.patch.object(p.time, 'monotonic', return_value=10.2):
            self.assertTrue(link.live())
        link.telemetry = None
        self.assertFalse(link.live(10.2))

    def test_unknown_auxiliary_frames_do_not_block_control_or_navigation_telemetry(self):
        # The deployed FPGA may still emit retired auxiliary frames until updated.
        auxiliary = b'\x5a\xd4'+bytes((4,11))+bytes(28)+b'\x00\x00\r\n'
        stream = auxiliary+legacy_frame(7)+auxiliary+nav_frame(9)+auxiliary
        for split in range(len(stream)+1):
            ordinary, navigation = p.TelemetryParser(), p.NavTelemetryParser()
            got = ordinary.feed(stream[:split])+ordinary.feed(stream[split:])
            nav = navigation.feed(stream[:split])+navigation.feed(stream[split:])
            self.assertEqual([t.seq for t in got],[7])
            self.assertEqual([t.seq for t in nav],[9])

    def test_follow_flag_is_explicit_crc_protected_and_excludes_navigation(self):
        for armed,command in ((True,0),(True,1),(True,3),(True,4),(False,0)):
            packet=p.command_packet(1,armed,command,8,follow=True)
            self.assertEqual(packet[7],2)
            self.assertEqual(p.crc16(packet[2:8]),int.from_bytes(packet[8:10],'little'))
        for command in (2,5,6):
            with self.assertRaises(ValueError):p.command_packet(1,True,command,8,follow=True)
        with self.assertRaises(ValueError):p.command_packet(1,True,1,8,nav=True,follow=True)
        with self.assertRaises(ValueError):p.command_packet(1,False,1,8,follow=True)

    def test_posix_writer_preserves_follow_flag(self):
        reader=p.PosixTelemetryLink('virtual');reader.fd=17
        with mock.patch.object(p,'PosixTelemetryLink',return_value=reader):
            link=p.PosixControlLink('virtual')
        with mock.patch('os.write',side_effect=lambda fd,packet:len(packet)) as write:
            link.send(True,3,8,follow=True)
        self.assertEqual(write.call_args.args[1][7],2)

    def test_original_control_packet_unchanged(self):
        body = bytes((1, 23, 1, 3, 8, 0))
        self.assertEqual(p.command_packet(23, True, 3, 8),
                         b'\xa5\x5a' + body + struct.pack('<H', p.crc16(body)) + b'\r\n')

    def test_navigation_is_explicit_forward_or_disarm_only(self):
        for armed, command in [(True, 1), (False, 0)]:
            packet = p.command_packet(0, armed, command, 8, nav=True)
            self.assertEqual(packet[7], 1)
            self.assertEqual(p.crc16(packet[2:8]), int.from_bytes(packet[8:10], 'little'))
        for armed, command in [(True, 0), (True, 2), (True, 3), (False, 1)]:
            with self.assertRaises(ValueError):
                p.command_packet(0, armed, command, 8, nav=True)

    def test_config_layout_signed_units_crc_and_canonical_fields(self):
        frame = p.navigation_packet(255, p.NAV_WRITE, index=7, x_mm=-10000, y_mm=10000)
        self.assertEqual(len(frame), 20)
        self.assertEqual(frame[:2], b'\xa5\xc3')
        self.assertEqual(struct.unpack('<BBBBhhB5x', frame[2:16]), (1, 255, 1, 7, -10000, 10000, 0))
        self.assertEqual(frame[11:16], bytes(5))
        self.assertEqual(p.crc16(frame[2:16]), int.from_bytes(frame[16:18], 'little'))
        self.assertEqual(frame[-2:], b'\r\n')
        self.assertEqual(p.navigation_packet(1, p.NAV_COMMIT, count=8)[10], 8)
        for opcode, fields in [(p.NAV_WRITE, {'index': 8}), (p.NAV_WRITE, {'x_mm': 10001}),
                               (p.NAV_WRITE, {'y_mm': -10001}), (p.NAV_WRITE, {'x_mm': 1.5}),
                               (p.NAV_COMMIT, {}), (p.NAV_COMMIT, {'count': 9}),
                               (p.NAV_COMMIT, {'count': 1, 'x_mm': 1}),
                               (p.NAV_CLEAR, {'count': 1}), (p.NAV_POSE_RESET, {'index': 1}),
                               (0, {})]:
            with self.assertRaises(ValueError):
                p.navigation_packet(0, opcode, **fields)

    def test_interleaved_stream_all_splits_and_crc_recovery(self):
        frame = nav_frame()
        broken = bytearray(frame)
        broken[8] ^= 1
        stream = b'noise' + bytes(broken) + legacy_frame() + frame
        for split in range(len(stream) + 1):
            nav, old = p.NavTelemetryParser(), p.TelemetryParser()
            got = nav.feed(stream[:split]) + nav.feed(stream[split:])
            legacy = old.feed(stream[:split]) + old.feed(stream[split:])
            self.assertEqual(len(got), 1)
            self.assertEqual(len(legacy), 1)
            t = got[0]
            self.assertEqual((t.x_mm, t.y_mm, t.heading, t.ack_seq, t.ack_status), (-1000, 2500, -9000, 3, 0))
            self.assertTrue(t.supported and t.active and t.ready and t.selected and t.pose_ready)
            self.assertFalse(t.done or t.failed)
            self.assertEqual(nav.bad_frames, 1)
        nav.feed(bytes(100000))
        self.assertLess(len(nav.buffer), 2)

    def test_link_navigation_ack_is_fresh_only_on_new_telemetry_sequence(self):
        port = mock.Mock(is_open=True, in_waiting=52)
        port.read.return_value = legacy_frame() + nav_frame()
        port.write.side_effect = lambda packet: len(packet)
        with mock.patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=mock.Mock(return_value=port))}):
            link = p.Link('virtual', writable=True)
        with mock.patch.object(p.time, 'monotonic', return_value=10):
            link.poll()
        self.assertTrue(link.nav_live(10.1))
        self.assertFalse(link.nav_live(9))
        self.assertEqual(link.nav_telemetry.ack_seq, 3)
        with mock.patch.object(p.time, 'monotonic', return_value=11):
            link.poll()
        self.assertFalse(link.nav_live(11))
        link.seq = 3
        self.assertEqual(link.configure_navigation(p.NAV_CLEAR), 3)
        self.assertEqual(port.write.call_args.args[0][:2], b'\xa5\xc3')
        self.assertEqual(link.seq, 4)

    def test_receive_only_configuration_never_writes(self):
        port = mock.Mock(is_open=True)
        with mock.patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=mock.Mock(return_value=port))}):
            link = p.Link('virtual')
        with self.assertRaises(PermissionError):
            link.configure_navigation(p.NAV_CLEAR)
        port.write.assert_not_called()
        self.assertFalse(link.nav_live(10))

    def test_posix_writer_uses_same_fd_and_returns_ack_sequence(self):
        reader=p.PosixTelemetryLink('virtual');reader.fd=17
        with mock.patch.object(p, 'PosixTelemetryLink', return_value=reader):
            link = p.PosixControlLink('virtual')
        with mock.patch('os.write', side_effect=lambda fd, packet: len(packet)) as write:
            self.assertEqual(link.configure_navigation(p.NAV_WRITE, x_mm=1000), 0)
            link.send(True, 1, 8, nav=True)
        self.assertEqual(write.call_args_list[0].args[0], 17)
        self.assertEqual(write.call_args_list[0].args[1][:2], b'\xa5\xc3')
        self.assertEqual(write.call_args_list[1].args[1][7], 1)

    def test_posix_nav_freshness_and_disconnect_reset(self):
        link = p.PosixTelemetryLink('virtual')
        link.fd = 17
        modules = {'termios': SimpleNamespace(error=OSError),
                   'tty': SimpleNamespace(), 'fcntl': SimpleNamespace()}
        with mock.patch.dict(sys.modules, modules), mock.patch('os.read') as read, mock.patch('os.close') as close:
            read.return_value = legacy_frame() + nav_frame()
            link.poll(10)
            self.assertTrue(link.live(10) and link.nav_live(10))
            link.poll(11)
            self.assertFalse(link.nav_live(11))
            read.return_value = b''
            link.poll(12)
            self.assertIsNone(link.nav_telemetry)
            self.assertFalse(link.nav_live(12))
            self.assertIsNone(link.last_nav_seq)
            self.assertEqual(link.nav_parser.buffer, b'')
            close.assert_called_once_with(17)


if __name__ == '__main__':
    unittest.main(verbosity=2)
