"""Failure-path checks: receive-only control ownership and bounded cleanup."""
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rk'))
import main as app
import protocol
import detector


def telemetry(seq=1):
    body = struct.pack('<BBBBBBHHhhhhhH', 1, seq, 0, 0x1F, 0, 16,
                       600, 650, 0, 0, 0, 0, 0, 0)
    return b'\x5a\xa5' + body + struct.pack('<H', protocol.crc16(body)) + b'\r\n'


class LinkTests(unittest.TestCase):
    def make_link(self, writable=False):
        port = mock.Mock(is_open=True)
        port.write.side_effect = lambda packet: len(packet)
        with mock.patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=mock.Mock(return_value=port))}):
            link = protocol.Link('virtual', writable=writable)
        return link, port

    def test_receive_only_never_transmits_including_close(self):
        link, port = self.make_link()
        with self.assertRaises(PermissionError):
            link.send()
        link.close()
        port.write.assert_not_called()
        port.close.assert_called_once()

    def test_drive_close_disarms_and_closes_even_after_write_failure(self):
        link, port = self.make_link(True)
        with mock.patch.object(protocol.time, 'sleep'):
            link.close()
        self.assertEqual(port.write.call_count, 3)
        for call in port.write.call_args_list:
            packet = call.args[0]
            self.assertEqual(packet[4:8], bytes(4))
        broken, port = self.make_link(True)
        port.write.side_effect = OSError('USB unplugged')
        with self.assertRaises(OSError):
            broken.close()
        port.close.assert_called_once()

    def test_duplicate_telemetry_does_not_refresh_watchdog(self):
        link, port = self.make_link()
        port.in_waiting = 28
        port.read.return_value = telemetry()
        with mock.patch.object(protocol.time, 'monotonic', return_value=1):
            link.poll()
            self.assertTrue(link.live())
        with mock.patch.object(protocol.time, 'monotonic', return_value=2):
            link.poll()
            self.assertFalse(link.live())
        port.read.return_value = telemetry(2)
        with mock.patch.object(protocol.time, 'monotonic', return_value=2):
            link.poll()
            self.assertTrue(link.live())


class ApplicationTests(unittest.TestCase):
    def setUp(self):
        self.link = mock.Mock(telemetry=None)
        self.link.live.return_value = False
        self.detection = mock.Mock(error=None)
        self.detection.snapshot.return_value = (None, (), 0, 0)
        self.cv = mock.Mock()
        self.cv.waitKey.side_effect = [ord(' '), 27]
        self.cv.getWindowProperty.return_value = 1

    def run_app(self, argv):
        with mock.patch.dict(sys.modules, {'cv2': self.cv}), \
             mock.patch.object(app, 'Link', return_value=self.link) as open_link, \
             mock.patch.object(app, 'DetectionWorker', return_value=self.detection):
            result = app.main(argv)
        return result, open_link

    def test_display_space_and_exit_do_not_claim_remote_control(self):
        result, open_link = self.run_app(['--no-detection', '--serial', 'virtual'])
        self.assertEqual(result, 0)
        open_link.assert_called_once_with('virtual', writable=False)
        self.link.send.assert_not_called()
        self.link.close.assert_called_once()

    def test_headless_receive_only_has_no_display_or_transmit(self):
        result, _ = self.run_app(['--no-detection', '--serial', 'virtual', '--headless', '--duration', '.005'])
        self.assertEqual(result, 0)
        self.link.send.assert_not_called()
        self.cv.putText.assert_not_called()
        self.cv.namedWindow.assert_not_called()

    def test_drive_explicitly_claims_uart(self):
        result, open_link = self.run_app(['--enable-drive', '--no-detection', '--serial', 'virtual'])
        self.assertEqual(result, 0)
        open_link.assert_called_once_with('virtual', writable=True)
        self.assertGreaterEqual(self.link.send.call_count, 2)

    def test_worker_failure_and_serial_close_failure_still_close_everything(self):
        self.detection.error = 'camera disconnected'
        self.link.close.side_effect = OSError('serial disconnected')
        with self.assertLogs(level='ERROR'):
            result, _ = self.run_app(['--camera', 'virtual', '--serial', 'virtual', '--headless'])
        self.assertEqual(result, 1)
        self.detection.close.assert_called_once()


class PosixViewerTests(unittest.TestCase):
    def modules(self):
        class TermiosError(Exception):
            pass
        termios = SimpleNamespace(
            error=TermiosError, TCSANOW=0, B115200=115200, CLOCAL=1,
            CREAD=2, CRTSCTS=4, VMIN=6, VTIME=5,
            tcgetattr=mock.Mock(side_effect=lambda _: [0, 0, 0, 0, 0, 0, [0] * 32]),
            tcsetattr=mock.Mock())
        return {'termios': termios, 'tty': SimpleNamespace(setraw=mock.Mock()),
                'fcntl': SimpleNamespace(LOCK_EX=2, LOCK_NB=4, flock=mock.Mock())}

    def test_idle_is_not_disconnect_and_duplicates_expire(self):
        modules = self.modules()
        link = protocol.PosixTelemetryLink('virtual')
        with mock.patch.dict(sys.modules, modules), mock.patch('os.open', return_value=17), \
             mock.patch('os.close') as close, mock.patch('os.read') as read, \
             mock.patch('os.O_NOCTTY', 0, create=True), mock.patch('os.O_NONBLOCK', 0, create=True):
            read.side_effect = BlockingIOError()
            link.poll(1)
            self.assertEqual(link.fd, 17)
            self.assertFalse(link.live(1))
            cfg = modules['termios'].tcsetattr.call_args.args[2]
            self.assertEqual(cfg[6][6], 1)  # VMIN prevents idle b''.
            read.side_effect = None
            read.return_value = telemetry()
            link.poll(2)
            self.assertTrue(link.live(2))
            link.poll(3)
            self.assertFalse(link.live(3))
            read.return_value = b''
            link.poll(4)
            self.assertIsNone(link.fd)
            self.assertIsNone(link.telemetry)
            self.assertEqual(link.retry, 6)
            close.assert_called_once_with(17)

    def test_configuration_error_closes_fd_and_clears_previous_attributes(self):
        modules = self.modules()
        modules['termios'].tcgetattr.side_effect = modules['termios'].error('not a tty')
        link = protocol.PosixTelemetryLink('virtual')
        with mock.patch.dict(sys.modules, modules), mock.patch('os.open', return_value=17), \
             mock.patch('os.close') as close, mock.patch('os.O_NOCTTY', 0, create=True), \
             mock.patch('os.O_NONBLOCK', 0, create=True):
            link.poll(1)
            self.assertIsNone(link.fd)
            self.assertIsNone(link.old)
            self.assertIn('not a tty', link.error)
            close.assert_called_once_with(17)


class WorkerTests(unittest.TestCase):
    def test_video_socket_bind_failure_releases_socket(self):
        import video
        sock=mock.Mock()
        sock.bind.side_effect=OSError('port already bound')
        with mock.patch.object(video.socket,'socket',return_value=sock):
            with self.assertRaises(OSError):video.VideoReceiver()
        sock.close.assert_called_once()


    def worker(self):
        worker = detector.DetectionWorker.__new__(detector.DetectionWorker)
        worker.stop_event = threading.Event()
        worker.process = mock.Mock()
        worker.process.stdout.fileno.return_value = 17
        return worker

    def test_capture_frame_handles_partial_reads_and_original_timestamp(self):
        worker = self.worker()
        raw = struct.pack('<d', 123.25) + bytes(640 * 480 * 3)
        chunks = [raw[:3], raw[3:29], raw[29:]]
        with mock.patch.object(detector.select, 'select', return_value=([17], [], [])), \
             mock.patch.object(detector.os, 'read', side_effect=chunks):
            image, stamp = worker._read_frame(1)
        self.assertEqual(image.shape, (480, 640, 3))
        self.assertEqual(stamp, 123.25)

    def test_capture_disconnect_timeout_and_cancel(self):
        worker = self.worker()
        with mock.patch.object(detector.select, 'select', return_value=([17], [], [])), \
             mock.patch.object(detector.os, 'read', return_value=b''):
            with self.assertRaises(OSError):
                worker._read_frame(1)
        with mock.patch.object(detector.time, 'monotonic', side_effect=[0, 2]):
            with self.assertRaises(TimeoutError):
                worker._read_frame(1)
        worker.stop_event.set()
        self.assertIsNone(worker._read_frame(1))


if __name__ == '__main__':
    unittest.main(verbosity=2)
