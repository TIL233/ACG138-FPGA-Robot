"""Forward CP2102 X3 raw bytes to the FPGA; all scan interpretation stays in FPGA.

No SDK, sensor UART writes, scan parser, navigation, or chassis commands. The
source starts automatically after power/rotation. DTR polarity follows the
bundled YDLidar SDK startMotor/stopMotor and the X3 support_motor_dtr parameter.
"""
from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path, PurePosixPath
import struct
import threading
import time


CP2102_VID = 0x10c4
CP2102_PID = 0xea60
BUFFER_LIMIT = 1024
MAX_AGE = .100
BATCH_SIZE = 128
# Real CP2102 delivery gaps reached ~27ms while the sensor kept streaming.
# Keep those bursts in one epoch; oldest-byte MAX_AGE remains independent.
PARTIAL_FLUSH = .050
MOTOR_SETTLE = .500
RETRY_DELAY = .500


@dataclass(frozen=True)
class UsbLidarConfig:
    enabled: bool = True
    device: str = ''
    support_motor_dtr: bool = True
    baudrate: int = 115200

    def __post_init__(self):
        for name in ('enabled', 'support_motor_dtr'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(name+' must be a boolean')
        if not isinstance(self.device, str) or (self.device and
                (not PurePosixPath(self.device).is_absolute() or
                 not self.device.startswith('/dev/') or '\x00' in self.device)):
            raise ValueError('device must be empty or an absolute /dev/ path')
        if type(self.baudrate) is not int or self.baudrate != 115200:
            raise ValueError('X3 USB baudrate must be 115200')


def _config(value):
    if value is None: return UsbLidarConfig()
    if isinstance(value, UsbLidarConfig): return value
    if not isinstance(value, dict): raise ValueError('USB lidar config must be an object')
    try: return UsbLidarConfig(**value)
    except TypeError as exc: raise ValueError(str(exc)) from exc


def load_config(path):
    path = Path(path)
    return _config(json.loads(path.read_text(encoding='utf-8-sig'))) if path.exists() else UsbLidarConfig()


class DeviceSelectionError(RuntimeError):
    pass


@dataclass(frozen=True)
class UsbDevice:
    path: str
    vid: int
    pid: int
    realpath: str


def _usb_identity(path):
    """Find USB VID/PID above a tty's device node, including USB interface nodes."""
    path = Path(path).resolve()
    for parent in (path, *path.parents):
        try:
            return (int((parent/'idVendor').read_text().strip(), 16),
                    int((parent/'idProduct').read_text().strip(), 16))
        except (OSError, ValueError):
            continue
    return None


def discover_usb_devices(by_id_root='/dev/serial/by-id', sys_tty_root='/sys/class/tty', dev_root='/dev'):
    """Prefer stable aliases, but verify every candidate through sysfs VID/PID."""
    by_id_root = Path(by_id_root); sys_tty_root = Path(sys_tty_root); dev_root = Path(dev_root)
    devices = []
    if by_id_root.is_dir():
        for alias in sorted(by_id_root.iterdir()):
            realpath = os.path.realpath(alias)
            identity = _usb_identity(sys_tty_root/Path(realpath).name/'device')
            if identity: devices.append(UsbDevice(str(alias), *identity, realpath))
    if sys_tty_root.is_dir():
        for tty in sorted(sys_tty_root.iterdir()):
            identity = _usb_identity(tty/'device')
            if identity:
                path = str(dev_root/tty.name)
                devices.append(UsbDevice(path, *identity, os.path.realpath(path)))
    return devices


def select_usb_device(devices, requested=''):
    devices = list(devices)
    if requested:
        realpath = os.path.realpath(requested)
        devices = [d for d in devices if d.path == requested or d.realpath == realpath]
    cp2102 = [d for d in devices if (d.vid, d.pid) == (CP2102_VID, CP2102_PID)]
    unique = {}
    for device in sorted(cp2102, key=lambda d: (not d.path.startswith('/dev/serial/by-id/'), d.path)):
        unique.setdefault(device.realpath, device.path)
    if not unique:
        raise DeviceSelectionError('no verified CP2102 10c4:ea60 lidar USB device'+(' at '+requested if requested else ''))
    if len(unique) != 1:
        raise DeviceSelectionError('multiple CP2102 devices; set device to a stable /dev/serial/by-id/ path')
    # Keep an explicit stable path; sysfs may have discovered only its tty target.
    return requested or next(iter(unique.values()))


class PosixLidarReader:
    """Exclusive, nonblocking 115200 8N1 source; intentionally has no write API."""
    def __init__(self, device):
        import fcntl
        import termios
        self._fcntl = fcntl; self._termios = termios; self.fd = None
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | getattr(os, 'O_CLOEXEC', 0))
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # Recheck the opened descriptor, not just a potentially replaced alias.
            info = os.fstat(fd)
            identity = _usb_identity(Path('/sys/dev/char')/f'{os.major(info.st_rdev)}:{os.minor(info.st_rdev)}'/'device')
            if identity != (CP2102_VID, CP2102_PID):
                raise DeviceSelectionError('opened device is not verified CP2102 10c4:ea60')
            if hasattr(termios, 'TIOCEXCL'): fcntl.ioctl(fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0; attrs[1] = 0; attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[3] = 0; attrs[4] = termios.B115200; attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 1; attrs[6][termios.VTIME] = 0
            # HUPCL is absent: close must not reverse the explicitly applied stop polarity.
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            self.fd = fd
        except BaseException as exc:
            os.close(fd)
            if isinstance(exc, termios.error): raise OSError(str(exc)) from exc
            raise

    def set_dtr(self, asserted):
        t = self._termios
        self._fcntl.ioctl(self.fd, t.TIOCMBIS if asserted else t.TIOCMBIC, struct.pack('I', t.TIOCM_DTR))

    def flush(self):
        try: self._termios.tcflush(self.fd, self._termios.TCIFLUSH)
        except self._termios.error as exc: raise OSError(str(exc)) from exc

    def read(self, size):
        try:
            payload = os.read(self.fd, size)
        except BlockingIOError:
            return b''
        if not payload: raise OSError('lidar USB disconnected')
        return payload

    def close(self):
        if self.fd is not None:
            fd = self.fd; self.fd = None; os.close(fd)


class PosixUsbBackend:
    def discover(self, requested): return select_usb_device(discover_usb_devices(), requested)
    def open(self, device, config): return PosixLidarReader(device)


class LidarUsbBridge:
    """One worker owns source I/O and a bounded recent-byte queue.

    link_getter is evaluated on every iteration because the GUI replaces links.
    The link owns capability freshness, serial framing, bandwidth backpressure,
    and serialized writes. An accepted reset always precedes a new byte epoch.
    Injected backends must provide nonblocking read/open operations.
    """
    def __init__(self, link_getter, config=None, autostart=True, *, backend=None, clock=None):
        self.config = _config(config); self._link_getter = link_getter
        self._backend = backend if backend is not None else PosixUsbBackend()
        self._clock = clock if clock is not None else time.monotonic
        self._lock = threading.RLock(); self._stop = threading.Event(); self._thread = None
        self._reader = None; self._link_signature = None; self._buffer = bytearray()
        self._buffer_chunks = deque()
        self._buffer_since = None; self._last_source_rx = None
        self._tail_sent = False
        self._needs_reset = True; self._retry_at = 0
        self._last_service_at = None
        self._reset_without_source = False
        self._settling_until = 0; self._settling = False
        self._status = dict(state='waiting_firmware' if self.config.enabled else 'disabled',
                            error='', device='', bytes_received=0, bytes_forwarded=0, dropped_bytes=0)
        if autostart and self.config.enabled:
            self._thread = threading.Thread(target=self._run, name='lidar-usb-bridge', daemon=True)
            self._thread.start()

    def snapshot(self):
        # A status read must not wait on USB/sysfs work held under the worker
        # lock. CPython copies this primitive-value dictionary under the GIL.
        return dict(self._status, buffered_bytes=len(self._buffer))

    def _drop_buffer(self):
        self._status['dropped_bytes'] += len(self._buffer)
        self._buffer.clear(); self._buffer_since = None; self._last_source_rx = None
        self._buffer_chunks.clear()
        self._tail_sent = False
        self._needs_reset = True

    def _close_source(self):
        reader = self._reader; self._reader = None; self._settling = False
        if reader is None: return
        try:
            # SDK: support_motor_dtr=true => start asserted, stop cleared; false => inverse.
            reader.set_dtr(not self.config.support_motor_dtr)
        except OSError:
            pass
        try: reader.flush()
        except OSError: pass
        try: reader.close()
        except OSError: pass

    def _source_failed(self, now, exc):
        self._reset_without_source = self._reset_without_source or self._reader is not None
        self._drop_buffer(); self._close_source(); self._retry_at = now+RETRY_DELAY
        self._status.update(state='waiting_device', error=str(exc))

    def _reset_absent_source(self, link, now):
        if not self._reset_without_source: return False
        try:
            if link.send_lidar_bytes(b'', reset=True, now=now): self._reset_without_source = False
        except (OSError, RuntimeError):
            pass
        return True

    def _step(self, now):
        with self._lock:
            if self._stop.is_set() or not self.config.enabled: return
            service_gap = 0 if self._last_service_at is None else now-self._last_service_at
            self._last_service_at = now
            try:
                link = self._link_getter()
                signature = (id(link), getattr(link, 'lidar_bridge_epoch', None))
                if signature != self._link_signature:
                    self._drop_buffer(); self._close_source(); self._retry_at = 0
                    self._link_signature = signature
                ready = callable(getattr(link, 'lidar_bridge_ready', None)) and link.lidar_bridge_ready(now=now)
            except (OSError, RuntimeError) as exc:
                self._source_failed(now, exc); self._status['state'] = 'waiting_firmware'; return
            if not ready:
                self._drop_buffer(); self._close_source(); self._retry_at = 0
                self._status.update(state='waiting_firmware', error='')
                return
            if self._reader is None:
                if self._reset_absent_source(link, now): return
                if now < self._retry_at: return
                try:
                    device = self._backend.discover(self.config.device)
                    self._reader = self._backend.open(device, self.config)
                    self._reader.flush()
                    self._reader.set_dtr(self.config.support_motor_dtr)
                    self._settling = True; self._settling_until = now+MOTOR_SETTLE
                    self._needs_reset = True
                    self._status.update(state='starting', error='', device=device)
                except (OSError, RuntimeError) as exc:
                    self._source_failed(now, exc); return
            if self._buffer_since is not None and now-self._buffer_since >= MAX_AGE:
                self._drop_buffer()
            try:
                if not self._settling and service_gap >= MAX_AGE:
                    # Bytes accumulated by the kernel while this worker was stalled
                    # have unknown age. Flush rather than stamp them as newly read.
                    self._drop_buffer(); self._reader.flush()
                if self._settling and now >= self._settling_until:
                    self._reader.flush(); self._settling = False
                if not self._settling:
                    payload = self._reader.read(BUFFER_LIMIT)
                    if payload:
                        if self._tail_sent:
                            # A short tail can still occupy an FPGA bank.
                            # Resume as a new epoch before another data frame,
                            # rather than enqueueing long/short/long back-to-back.
                            self._needs_reset = True; self._tail_sent = False
                        self._status['bytes_received'] += len(payload)
                        if len(self._buffer)+len(payload) > BUFFER_LIMIT:
                            self._drop_buffer()
                        if not self._buffer: self._buffer_since = now
                        self._buffer.extend(payload)
                        self._buffer_chunks.append((len(payload), now))
                        self._last_source_rx = now
            except (OSError, RuntimeError) as exc:
                self._source_failed(now, exc)
                self._reset_absent_source(link, now)
                return
            # Exactly one enqueue attempt per iteration, including resets/backpressure.
            if self._needs_reset:
                try: accepted = link.send_lidar_bytes(b'', reset=True, now=now)
                except (OSError, RuntimeError) as exc:
                    self._source_failed(now, exc); return
                if accepted: self._needs_reset = False
                self._status['state'] = 'starting' if self._settling else ('streaming' if accepted else 'backpressure')
                return
            if self._settling: return
            # Keep continuous input in full batches. An old buffer timestamp
            # must not flush a newly received short remainder: long/short/long
            # frames can occupy both FPGA banks before the first one drains.
            # Source-idle time controls tail flushing; oldest-byte age remains
            # independent and still expires through MAX_AGE above.
            if self._buffer and (len(self._buffer) >= BATCH_SIZE or now-self._last_source_rx >= PARTIAL_FLUSH):
                payload = bytes(self._buffer[:BATCH_SIZE])
                try: accepted = link.send_lidar_bytes(payload, reset=False, now=now)
                except (OSError, RuntimeError) as exc:
                    self._source_failed(now, exc); return
                if not accepted:
                    self._status['state'] = 'backpressure'; return
                del self._buffer[:len(payload)]
                remaining = len(payload)
                while remaining:
                    size, received_at = self._buffer_chunks.popleft()
                    if size > remaining:
                        self._buffer_chunks.appendleft((size-remaining, received_at))
                        break
                    remaining -= size
                self._buffer_since = self._buffer_chunks[0][1] if self._buffer_chunks else None
                self._tail_sent = len(payload) < BATCH_SIZE
                self._status['bytes_forwarded'] += len(payload)
            self._status.update(state='streaming', error='')

    def _run(self):
        try:
            while not self._stop.is_set():
                self._step(self._clock())
                self._stop.wait(.002 if self._reader is not None else .005)
        finally:
            with self._lock:
                self._drop_buffer(); self._close_source(); self._status['state'] = 'closed'

    def close(self):
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1)
            if self._thread.is_alive(): raise RuntimeError('lidar USB worker did not stop')
        with self._lock:
            self._drop_buffer(); self._close_source(); self._status['state'] = 'closed'
