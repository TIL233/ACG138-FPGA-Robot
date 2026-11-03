"""Raw USB bridge behavior using nonblocking fake endpoints, never hardware."""
from collections import deque
from contextlib import contextmanager, ExitStack
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rk'))
import usb_lidar as u


class Clock:
    def __init__(self): self.now = 0.0
    def __call__(self): return self.now
    def advance(self, seconds): self.now += seconds


class Reader:
    def __init__(self):
        self.chunks = deque(); self.dtr = []; self.closed = False; self.flushes = 0
    def set_dtr(self, asserted): self.dtr.append(asserted)
    def flush(self): self.flushes += 1; self.chunks.clear()
    def read(self, size):
        if not self.chunks: return b''
        item = self.chunks.popleft()
        if isinstance(item, Exception): raise item
        if len(item) > size: self.chunks.appendleft(item[size:])
        return item[:size]
    def close(self): self.closed = True


class Backend:
    def __init__(self):
        self.readers = []; self.error = None
    def discover(self, requested):
        if self.error: raise self.error
        return requested or '/dev/serial/by-id/usb-Silicon_Labs_CP2102_lidar'
    def open(self, device, config):
        reader = Reader(); self.readers.append(reader); return reader


class Link:
    # Deliberately offers no ARM/navigation/general control method.
    def __init__(self):
        self.ready = True; self.lidar_bridge_epoch = 0
        self.accept = True; self.error = None; self.attempts = []; self.sent = []
    def lidar_bridge_ready(self, now=None): return self.ready
    def send_lidar_bytes(self, payload, reset=False, now=None):
        self.attempts.append((payload, reset, now))
        if self.error: raise self.error
        if not self.accept: return False
        self.sent.append((payload, reset)); return True


class SelectionTests(unittest.TestCase):
    def test_automatic_selection_prefers_stable_alias_and_never_ch340(self):
        devices = [u.UsbDevice('/dev/ttyUSB0', 0x1a86, 0x7523, '/dev/ttyUSB0'),
                   u.UsbDevice('/dev/ttyUSB1', 0x10c4, 0xea60, '/dev/ttyUSB1'),
                   u.UsbDevice('/dev/serial/by-id/cp2102-lidar', 0x10c4, 0xea60, '/dev/ttyUSB1')]
        self.assertEqual(u.select_usb_device(devices), '/dev/serial/by-id/cp2102-lidar')

    def test_ambiguous_cp2102_requires_explicit_selection(self):
        devices = [u.UsbDevice('/dev/serial/by-id/a', 0x10c4, 0xea60, '/dev/ttyUSB1'),
                   u.UsbDevice('/dev/serial/by-id/b', 0x10c4, 0xea60, '/dev/ttyUSB2')]
        with self.assertRaisesRegex(u.DeviceSelectionError, 'multiple'):
            u.select_usb_device(devices)
        self.assertEqual(u.select_usb_device(devices, '/dev/serial/by-id/b'), '/dev/serial/by-id/b')

    def test_explicit_wrong_vid_and_missing_device_are_rejected(self):
        devices = [u.UsbDevice('/dev/ttyUSB0', 0x1a86, 0x7523, '/dev/ttyUSB0')]
        for requested in ('', '/dev/ttyUSB0', '/dev/serial/by-id/missing'):
            with self.subTest(requested=requested), self.assertRaises(u.DeviceSelectionError):
                u.select_usb_device(devices, requested)

    def test_sysfs_discovery_validates_usb_parent_not_port_number(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); sysroot = root/'sys'; devroot = root/'dev'
            for name, vid, pid in [('ttyUSB0', '1a86', '7523'), ('ttyUSB7', '10c4', 'ea60')]:
                parent = sysroot/name/'device'; parent.mkdir(parents=True)
                (parent/'idVendor').write_text(vid); (parent/'idProduct').write_text(pid)
            devices = u.discover_usb_devices(root/'missing-by-id', sysroot, devroot)
            self.assertEqual(u.select_usb_device(devices), str(devroot/'ttyUSB7'))

    def test_config_validates_types_polarity_and_fixed_baud(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'config.json'
            self.assertTrue(u.load_config(path).enabled)
            for value in ({'enabled': 1}, {'device': 'ttyUSB0'},
                          {'support_motor_dtr': 'false'}, {'baudrate': 230400}, {'unknown': 1}):
                path.write_text(json.dumps(value))
                with self.subTest(value=value), self.assertRaises(ValueError): u.load_config(path)
            path.write_text('{"enabled":true,"device":"","support_motor_dtr":false}')
            self.assertFalse(u.load_config(path).support_motor_dtr)


class NativeBackendTests(unittest.TestCase):
    class NativeTermiosError(Exception): pass

    @contextmanager
    def native(self, identity=(0x10c4, 0xea60)):
        attrs = [65535, 65535, 65535, 65535, 0, 0, [9]*32]
        termios = SimpleNamespace(CS8=48, CLOCAL=2048, CREAD=128, B115200=4098,
                                  VMIN=6, VTIME=5, TCSANOW=0, TCIFLUSH=0,
                                  TIOCEXCL=21516, TIOCMBIS=21526, TIOCMBIC=21527, TIOCM_DTR=2,
                                  error=self.NativeTermiosError,
                                  tcgetattr=mock.Mock(return_value=attrs), tcsetattr=mock.Mock(), tcflush=mock.Mock())
        fcntl = SimpleNamespace(LOCK_EX=2, LOCK_NB=4, flock=mock.Mock(), ioctl=mock.Mock())
        with ExitStack() as stack:
            stack.enter_context(mock.patch.dict(sys.modules, {'termios': termios, 'fcntl': fcntl}))
            opened = stack.enter_context(mock.patch.object(u.os, 'open', return_value=17))
            closed = stack.enter_context(mock.patch.object(u.os, 'close'))
            stack.enter_context(mock.patch.object(u.os, 'O_NOCTTY', create=True, new=256))
            stack.enter_context(mock.patch.object(u.os, 'O_NONBLOCK', create=True, new=2048))
            stack.enter_context(mock.patch.object(u.os, 'fstat', return_value=SimpleNamespace(st_rdev=48135)))
            stack.enter_context(mock.patch.object(u.os, 'major', create=True, return_value=188))
            stack.enter_context(mock.patch.object(u.os, 'minor', create=True, return_value=7))
            stack.enter_context(mock.patch.object(u, '_usb_identity', return_value=identity))
            stack.enter_context(mock.patch.object(u.os, 'write', side_effect=AssertionError('sensor UART write forbidden')))
            yield termios, fcntl, opened, closed

    def test_raw_8n1_nonblocking_exclusive_without_uart_writes(self):
        with self.native() as (t, f, opened, closed):
            reader = u.PosixLidarReader('/dev/serial/by-id/verified-cp2102')
            flags = opened.call_args.args[1]
            self.assertTrue(flags & u.os.O_NONBLOCK); self.assertTrue(flags & u.os.O_NOCTTY)
            f.flock.assert_called_once_with(17, 6)
            attrs = t.tcsetattr.call_args.args[2]
            self.assertEqual(attrs[:6], [0, 0, 2224, 0, 4098, 4098])
            # With O_NONBLOCK, VMIN=1 gives EAGAIN when empty. VMIN=0 may return
            # zero, making a healthy idle source look like a disconnected tty.
            self.assertEqual(attrs[6][6], 1); self.assertEqual(attrs[6][5], 0)
            reader.set_dtr(True); reader.set_dtr(False); reader.flush(); reader.close()
            self.assertEqual([call.args[:2] for call in f.ioctl.call_args_list],
                             [(17, 21516), (17, 21526), (17, 21527)])
            closed.assert_called_once_with(17)

    def test_opened_descriptor_wrong_usb_identity_is_rejected_before_dtr(self):
        with self.native((0x1a86, 0x7523)) as (t, f, opened, closed):
            with self.assertRaises(u.DeviceSelectionError): u.PosixLidarReader('/dev/serial/by-id/raced-alias')
            f.ioctl.assert_not_called(); t.tcsetattr.assert_not_called(); closed.assert_called_once_with(17)

    def test_empty_nonblocking_read_and_unplug_are_distinct(self):
        with self.native():
            reader = u.PosixLidarReader('/dev/ttyUSB7')
            with mock.patch.object(u.os, 'read', side_effect=BlockingIOError()):
                self.assertEqual(reader.read(128), b'')
            with mock.patch.object(u.os, 'read', return_value=b''):
                with self.assertRaisesRegex(OSError, 'disconnected'): reader.read(128)
            with mock.patch.object(u.os, 'read', return_value=b'\xaa\x55\x5a\x00'):
                self.assertEqual(reader.read(128), b'\xaa\x55\x5a\x00')
            reader.close()

    def test_termios_errors_are_io_errors_and_configuration_failure_closes_fd(self):
        with self.native() as (t, f, opened, closed):
            t.tcgetattr.side_effect = self.NativeTermiosError('device gone')
            with self.assertRaises(OSError): u.PosixLidarReader('/dev/ttyUSB7')
            closed.assert_called_once_with(17)
        with self.native() as (t, f, opened, closed):
            reader = u.PosixLidarReader('/dev/ttyUSB7')
            t.tcflush.side_effect = self.NativeTermiosError('device gone')
            with self.assertRaises(OSError): reader.flush()
            reader.close(); closed.assert_called_once_with(17)


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(); self.backend = Backend(); self.link = Link()
        self.current = self.link
        self.bridge = u.LidarUsbBridge(lambda: self.current, autostart=False,
                                      backend=self.backend, clock=self.clock)
        self.addCleanup(self.bridge.close)

    def step(self, elapsed=0):
        self.clock.advance(elapsed); self.bridge._step(self.clock())

    def connect(self):
        self.step(); self.step(.5)
        return self.backend.readers[-1]

    def test_old_firmware_cannot_open_or_start_usb_and_never_writes(self):
        self.link.ready = False
        for _ in range(4): self.step(.1)
        self.assertEqual(self.bridge.snapshot()['state'], 'waiting_firmware')
        self.assertEqual(self.backend.readers, [])
        self.assertEqual(self.link.attempts, [])
        self.bridge.close()
        self.assertEqual(self.link.attempts, [])

    def test_link_without_capability_api_waits_without_side_effect(self):
        self.current = object(); self.step()
        self.assertEqual(self.bridge.snapshot()['state'], 'waiting_firmware')
        self.assertEqual(self.backend.readers, [])

    def test_raw_bytes_are_preserved_and_partial_batches_wait_for_source_idle(self):
        reader = self.connect()
        raw = b'\xaa\x55\x5a\xa5\x7d\x00\xff\x0d\x0a'
        reader.chunks.append(raw); self.step()
        self.assertEqual(self.link.sent, [(b'', True)])
        self.step(u.PARTIAL_FLUSH-.001); self.assertEqual(len(self.link.sent), 1)
        self.step(.0011)
        self.assertEqual(self.link.sent, [(b'', True), (raw, False)])
        state = self.bridge.snapshot(); json.dumps(state)
        self.assertEqual((state['bytes_received'], state['bytes_forwarded'], state['dropped_bytes']), (9, 9, 0))

    def test_usb_20ms_bursts_do_not_end_each_continuous_scan_epoch(self):
        reader = self.connect(); raw = bytearray()
        for burst in range(50):
            chunk = bytes((len(raw)+j)&255 for j in range(188)); raw.extend(chunk)
            reader.chunks.append(chunk); self.step(.002)
            for _ in range(9): self.step(.002)
            self.assertEqual(sum(reset for _, reset in self.link.sent), 1,
                             '20ms USB delivery gaps must not repeatedly reset a continuous source')
            self.assertTrue(all(len(payload)==128 for payload, reset in self.link.sent if not reset))
            self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 0)
        self.step(u.PARTIAL_FLUSH)
        self.assertEqual(b''.join(payload for payload, reset in self.link.sent if not reset), raw)

    def test_continuous_9400_bytes_per_second_keeps_full_batches_until_source_idle(self):
        reader = self.connect(); raw = bytearray()
        # At the worker's 2ms cadence, 94 bytes per five reads models 9.4KB/s.
        # A 128-byte batch can leave newly received bytes behind: their source
        # is still active even when the original buffer age crosses 16ms.
        # Keep running beyond MAX_AGE: emitting a full prefix must advance
        # the oldest remaining byte rather than expiring the active stream.
        for tick in range(500):
            count = (18, 19, 19, 19, 19)[tick % 5]
            chunk = bytes((len(raw)+index) % 256 for index in range(count))
            raw.extend(chunk); reader.chunks.append(chunk); self.step(.002)
            sent = [payload for payload, reset in self.link.sent if not reset]
            self.assertTrue(all(len(payload) == 128 for payload in sent),
                            f'active source emitted a short batch at read {tick+1}: '
                            f'{[len(payload) for payload in sent]}')
            self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 0,
                             f'active source lost recent bytes at read {tick+1}')
            self.assertEqual(b''.join(sent), bytes(raw[:sum(map(len, sent))]))
        self.assertEqual(len(raw), 9400)
        self.assertEqual(sum(reset for _, reset in self.link.sent), 1,
                         'uninterrupted input needs only its initial RESET')
        self.assertEqual([len(payload) for payload, reset in self.link.sent if not reset],
                         [128]*73)
        before = list(self.link.sent); self.step(u.PARTIAL_FLUSH-.001)
        self.assertEqual(self.link.sent, before, 'source-idle deadline has not elapsed')
        self.step(.0011)
        sent = [payload for payload, reset in self.link.sent if not reset]
        self.assertEqual([len(payload) for payload in sent], [128]*73+[56])
        self.assertEqual(b''.join(sent), bytes(raw))
        self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 0)

    def test_intermittent_input_restarts_idle_deadline_without_flushing_a_recent_tail(self):
        reader = self.connect(); reader.chunks.append(b'first'); self.step()
        self.step(.010)
        reader.chunks.append(b'second'); self.step(.004)
        self.step(.003)
        self.assertEqual(self.link.sent, [(b'', True)],
                         '17ms buffer age must not flush bytes received 3ms ago')
        self.step(u.PARTIAL_FLUSH-.004)
        self.assertEqual(self.link.sent, [(b'', True)])
        self.step(.0011)
        self.assertEqual(self.link.sent, [(b'', True), (b'firstsecond', False)])
        self.step(.020)
        reader.chunks.append(b'third'); self.step()
        self.assertEqual(self.link.sent[-1], (b'', True))
        self.step(u.PARTIAL_FLUSH-.001)
        self.assertEqual([payload for payload, reset in self.link.sent if not reset], [b'firstsecond'])
        self.step(.0011)
        self.assertEqual(self.link.sent[-1], (b'third', False))

    def test_recent_input_does_not_extend_oldest_buffer_max_age_under_backpressure(self):
        reader = self.connect(); self.link.accept = False
        reader.chunks.append(b'A'*20); self.step()
        for _ in range(5):
            reader.chunks.append(b'B'*20); self.step(.019)
        self.assertEqual(self.bridge.snapshot()['buffered_bytes'], 120)
        self.link.accept = True
        reader.chunks.append(b'C'*19); self.step(.006)
        # The last B arrived only 6ms ago, but the oldest A is already 101ms
        # old. All queued old-epoch data must be dropped before RESET and C.
        self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 120)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True)])
        self.assertEqual(self.bridge.snapshot()['buffered_bytes'], 19)
        self.step(u.PARTIAL_FLUSH-.001)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True)])
        self.step(.0011)
        self.assertEqual(self.link.sent[-1], (b'C'*19, False))

    def test_batch_limit_and_single_attempt_per_service_iteration(self):
        reader = self.connect(); raw = bytes(range(256))*2
        reader.chunks.append(raw); before = len(self.link.attempts)
        self.step(); self.assertEqual(len(self.link.attempts), before+1)
        for _ in range(3): self.step(.002)
        data = [payload for payload, reset in self.link.sent if not reset]
        self.assertEqual([len(part) for part in data], [128, 128, 128, 128])
        self.assertEqual(b''.join(data), raw)

    def test_input_resuming_after_a_short_tail_resets_before_the_next_frame(self):
        reader = self.connect()
        reader.chunks.append(b'A'*20); self.step(); self.step(u.PARTIAL_FLUSH)
        self.assertEqual(self.link.sent, [(b'', True), (b'A'*20, False)])
        reader.chunks.append(b'B'*128); self.step(.002)
        self.assertEqual(self.link.sent[-1], (b'', True),
                         'a short tail must not be followed by another frame in the old epoch')
        self.step(.002)
        self.assertEqual(self.link.sent[-1], (b'B'*128, False))

    def test_backpressure_retains_recent_data_without_duplicate_delivery(self):
        reader = self.connect(); self.link.accept = False
        reader.chunks.append(b'A'*128); self.step()
        self.assertEqual(self.bridge.snapshot()['bytes_forwarded'], 0)
        self.step(.02); self.link.accept = True; self.step(.002)
        self.assertEqual(self.link.sent, [(b'', True), (b'A'*128, False)])

    def test_age_expiration_drops_stalled_bytes_and_resets_before_new_data(self):
        reader = self.connect(); self.link.accept = False
        reader.chunks.append(b'old'*43); self.step()
        self.step(.101); self.link.accept = True; self.step(.002)
        reader.chunks.append(b'\xaa\x55fresh'); self.step(.002); self.step(u.PARTIAL_FLUSH)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True), (b'\xaa\x55fresh', False)])
        self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 129)

    def test_overflow_never_exceeds_bound_and_discards_previous_epoch(self):
        reader = self.connect(); self.link.accept = False
        reader.chunks.append(b'A'*1024); self.step()
        reader.chunks.append(b'B'*128); self.step(.002)
        self.assertLessEqual(self.bridge.snapshot()['buffered_bytes'], 1024)
        self.link.accept = True; self.step(.002); self.step(.002)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True), (b'B'*128, False)])
        self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 1024)

    def test_worker_scheduling_stall_flushes_unknown_age_kernel_bytes_before_reset(self):
        reader = self.connect(); reader.chunks.append(b'stale-in-kernel')
        self.step(.101)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True)])
        self.assertEqual(reader.flushes, 3)
        reader.chunks.append(b'fresh'); self.step(.002); self.step(u.PARTIAL_FLUSH)
        self.assertEqual(self.link.sent[-1], (b'fresh', False))

    def test_usb_unplug_drops_pending_and_reconnects_with_fresh_reset(self):
        reader = self.connect(); reader.chunks.append(b'old'); self.step()
        reader.chunks.append(OSError('unplugged')); self.step(.002)
        self.assertTrue(reader.closed); self.assertEqual(reader.dtr, [True, False])
        self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 3)
        self.step(.5); new_reader = self.backend.readers[-1]
        self.assertIsNot(new_reader, reader)
        self.step(.5); new_reader.chunks.append(b'new'); self.step(); self.step(u.PARTIAL_FLUSH)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True), (b'', True), (b'new', False)])

    def test_usb_unplug_resets_fpga_while_source_remains_absent(self):
        reader = self.connect(); self.link.accept = False
        reader.chunks.append(OSError('unplugged')); before = len(self.link.attempts); self.step(.002)
        self.assertEqual(len(self.link.attempts), before+1)
        self.backend.error = OSError('no USB source'); self.link.accept = True
        self.step(.002)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True)])
        self.assertEqual(len(self.backend.readers), 1)
        self.assertEqual(self.bridge.snapshot()['state'], 'waiting_device')

    def test_gui_link_replacement_discards_old_data_and_resets_new_link(self):
        reader = self.connect(); reader.chunks.append(b'old'); self.step()
        self.current = Link(); self.step(.002)
        self.assertTrue(reader.closed)
        self.assertEqual(self.current.sent, [(b'', True)])
        self.assertEqual(self.link.sent, [(b'', True)])
        self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 3)

    def test_fpga_reconnect_epoch_change_requires_reset_even_same_link_object(self):
        reader = self.connect(); reader.chunks.append(b'old'); self.step()
        self.link.lidar_bridge_epoch += 1; self.step(.002)
        self.assertTrue(reader.closed)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True)])
        self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 3)

    def test_lost_fresh_capability_stops_usb_until_ready_again(self):
        reader = self.connect(); reader.chunks.append(b'old'); self.step()
        self.link.ready = False; self.step(.002)
        self.assertTrue(reader.closed); self.assertEqual(reader.dtr[-1], False)
        self.assertEqual(self.bridge.snapshot()['state'], 'waiting_firmware')
        self.assertEqual(self.link.sent, [(b'', True)])
        self.link.ready = True; self.step(.002)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True)])

    def test_send_error_invalidates_and_retries_instead_of_replaying(self):
        reader = self.connect(); reader.chunks.append(b'A'*128)
        self.link.error = OSError('FPGA disconnected'); self.step()
        self.assertTrue(reader.closed)
        self.assertEqual(self.bridge.snapshot()['dropped_bytes'], 128)
        self.link.error = None; self.step(.5)
        self.assertEqual(self.link.sent, [(b'', True), (b'', True)])

    def test_polarity_false_matches_sdk_clear_on_start_set_on_stop(self):
        self.bridge.close()
        self.bridge = u.LidarUsbBridge(lambda: self.current, {'support_motor_dtr': False},
                                      autostart=False, backend=self.backend, clock=self.clock)
        self.step(); reader = self.backend.readers[-1]; self.bridge.close()
        self.assertEqual(reader.dtr, [False, True])
        self.assertEqual(self.link.sent, [(b'', True)])

    def test_motor_start_settling_is_interruptible_and_stale_kernel_data_flushed(self):
        self.step(); reader = self.backend.readers[-1]
        reader.chunks.append(b'startup bytes'); self.step(.499)
        self.assertEqual(self.link.sent, [(b'', True)])
        self.step(.001)
        self.assertEqual(reader.flushes, 2)
        self.step(.02); self.assertEqual(self.link.sent, [(b'', True)])

    def test_ambiguous_source_reports_error_without_open_or_commands(self):
        self.backend.error = u.DeviceSelectionError('multiple CP2102 devices')
        self.step()
        self.assertIn('multiple', self.bridge.snapshot()['error'])
        self.assertEqual(self.backend.readers, []); self.assertEqual(self.link.sent, [])

    def test_disabled_configuration_has_no_io(self):
        self.bridge.close()
        self.bridge = u.LidarUsbBridge(lambda: self.current, {'enabled': False},
                                      autostart=False, backend=self.backend, clock=self.clock)
        self.step(); self.assertEqual(self.bridge.snapshot()['state'], 'disabled')
        self.assertEqual(self.backend.readers, []); self.assertEqual(self.link.sent, [])

    def test_background_close_interrupts_retry_wait_and_leaves_no_thread(self):
        self.bridge.close(); self.backend.error = OSError('not present')
        self.bridge = u.LidarUsbBridge(lambda: self.current, backend=self.backend)
        deadline = time.monotonic()+1
        while not self.bridge.snapshot()['error'] and time.monotonic() < deadline: time.sleep(.002)
        worker = self.bridge._thread; before = time.monotonic(); self.bridge.close()
        self.assertLess(time.monotonic()-before, .25)
        self.assertFalse(worker.is_alive()); self.assertEqual(self.bridge.snapshot()['state'], 'closed')

    def test_background_close_interrupts_motor_settle_and_only_stops_source_dtr(self):
        self.bridge.close()
        self.bridge = u.LidarUsbBridge(lambda: self.current, backend=self.backend)
        deadline = time.monotonic()+1
        while not self.backend.readers and time.monotonic() < deadline: time.sleep(.002)
        self.assertTrue(self.backend.readers)
        worker = self.bridge._thread; before = time.monotonic(); self.bridge.close()
        self.assertLess(time.monotonic()-before, .25); self.assertFalse(worker.is_alive())
        reader = self.backend.readers[-1]
        self.assertTrue(reader.closed); self.assertEqual(reader.dtr, [True, False])
        self.assertTrue(all(payload == b'' and reset for payload, reset in self.link.sent))

    def test_snapshot_does_not_wait_for_worker_usb_discovery(self):
        self.bridge.close(); entered = threading.Event(); release = threading.Event()
        captured = threading.Event(); result = []
        def discover(requested):
            entered.set(); release.wait(1); raise OSError('not present')
        self.backend.discover = discover
        self.bridge = u.LidarUsbBridge(lambda: self.current, backend=self.backend)
        self.assertTrue(entered.wait(1))
        def capture(): result.append(self.bridge.snapshot()); captured.set()
        snapshot_worker = threading.Thread(target=capture)
        snapshot_worker.start()
        try: self.assertTrue(captured.wait(.1), 'GUI status must not wait on filesystem/USB I/O')
        finally:
            release.set(); snapshot_worker.join(1)
        json.dumps(result[0])


if __name__ == '__main__': unittest.main()
