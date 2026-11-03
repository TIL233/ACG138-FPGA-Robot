"""Wire protocol v1. No legacy register writes or implicit mode selection."""
from dataclasses import dataclass
import struct
import time
import threading
from typing import Optional


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def lidar_forward_packet(seq: int, payload: bytes, *, reset=False) -> bytes:
    """Escaped sensor-only tunnel; no A5 control headers occur inside its body."""
    if type(seq) is not int or not 0 <= seq <= 255 or type(reset) is not bool:
        raise ValueError('invalid lidar tunnel sequence or flags')
    if not isinstance(payload, bytes) or (reset and payload) or (not reset and not 1 <= len(payload) <= 128):
        raise ValueError('lidar payload requires 1..128 bytes, or an empty RESET')
    body=bytes((1,seq,int(reset),len(payload)))+payload
    encoded=bytearray()
    for byte in body+struct.pack('<H',crc16(body)):
        if byte in (0xa5,0x7d,0x7e):encoded.extend((0x7d,byte^0x20))
        else:encoded.append(byte)
    return b'\xa5\xb7'+encoded+b'\x7e'


def command_packet(seq: int, armed: bool, command: int, speed: int, *, nav=False, follow=False) -> bytes:
    if not 0 <= command <= 6 or not 0 <= speed <= 64:
        raise ValueError('command 0..6, speed 0..64 counts/20ms required')
    if nav and not ((armed and command == 1) or (not armed and command == 0)):
        raise ValueError('navigation accepts explicit ARM+FORWARD or DISARM+STOP only')
    if nav and follow:
        raise ValueError('navigation and visual follow are mutually exclusive')
    if follow and not ((armed and command in (0,1,3,4)) or (not armed and command == 0)):
        raise ValueError('visual follow accepts stop, forward and forward arcs only')
    body = bytes((1, seq & 255, int(bool(armed)), command, speed, int(bool(nav)) | (int(bool(follow)) << 1)))
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body)) + b'\r\n'


NAV_WRITE, NAV_COMMIT, NAV_POSE_RESET, NAV_CLEAR = 1, 2, 3, 4
RANGE_FAR, RANGE_INVALID = 0xfffe, 0xffff


@dataclass(frozen=True)
class LidarTelemetry:
    seq: int
    flags: int
    scan_id: int
    front_mm: int
    left_mm: int
    right_mm: int
    reserved_mm: int  # Former rear slot: ignored, kept for v1 wire compatibility.
    good_packets: int
    bad_packets: int
    front_zero_q6: int

    @property
    def fresh(self): return bool(self.flags & 16)

    @property
    def control_enabled(self): return bool(self.flags & 32)

    @property
    def usb_bridge_supported(self): return bool(self.flags & 64)

    @property
    def usb_source(self): return bool(self.flags & 128)

    def distances(self):
        values = (self.front_mm, self.left_mm, self.right_mm)
        return {name: value if self.fresh and self.flags & (1 << i) else None
                for i, (name, value) in enumerate(zip(('front', 'left', 'right'), values))}


class LidarTelemetryParser:
    """26 bytes: 5A B7, v1/seq + nine little-endian u16, CRC16, CR LF."""
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while len(self.buffer) >= 2:
            pos = self.buffer.find(b'\x5a\xb7')
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0x5a else b''
                break
            del self.buffer[:pos]
            if len(self.buffer) < 26:
                break
            frame = bytes(self.buffer[:26])
            body = frame[2:22]
            if (body[0] != 1 or frame[-2:] != b'\r\n'
                    or crc16(body) != int.from_bytes(frame[22:24], 'little')):
                self.bad_frames += 1
                del self.buffer[0]
                continue
            t = LidarTelemetry(*struct.unpack('<B9H', body[1:]))
            if t.flags & ~255 or t.front_zero_q6 >= 23040 or any(
                    t.flags & (1 << i) and not 120 <= d <= 8000
                    for i, d in enumerate((t.front_mm, t.left_mm, t.right_mm))):
                self.bad_frames += 1
            else:
                out.append(t)
            del self.buffer[:26]
        return out


@dataclass(frozen=True)
class LidarPlanningTelemetry:
    seq: int
    flags: int
    timestamp_ms: int
    scan_id: int
    generation: int
    age_ms: int
    stop_mm: int
    slow_mm: int
    clearance_mm: int
    speed_mm_s: int
    cycles: int
    candidate: int
    candidate_count: int
    target_l: int
    target_r: int
    admissible_mask: int
    unknown_mask: int
    goal_cd: int
    command_epoch: int
    latency_budget_ms: int

    @property
    def result_valid(self): return bool(self.flags & (1 << 4))

    @property
    def actual_takeover(self): return bool(self.flags & (1 << 5))


class LidarPlanningTelemetryParser:
    """Frozen C8 v1: 40-byte body, CRC16/CCITT-FALSE LE, CRLF (46B)."""
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    @staticmethod
    def _semantic_valid(t):
        if (t.flags & 0xc000 or t.candidate_count != 9
                or (t.admissible_mask | t.unknown_mask) & ~0x1ff):
            return False
        if t.actual_takeover:
            # Actual control includes protection braking while the planner has
            # no usable route. The model/control gates still apply to both.
            if t.flags & 0x2007 != 0x2007 or t.flags & 0x1000:
                return False
            if not t.result_valid and not t.flags & 0x80:
                return False
        if t.result_valid:
            if (not t.flags & 8 or t.flags & 0xd00 or not 0 <= t.candidate < 9
                    or not t.admissible_mask & (1 << t.candidate)
                    or t.unknown_mask & (1 << t.candidate)
                    or not 0 <= t.target_l <= 64 or not 0 <= t.target_r <= 64):
                return False
        elif t.candidate != 255 or t.target_l != 0 or t.target_r != 0:
            return False
        return True

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while len(self.buffer) >= 2:
            pos = self.buffer.find(b'\x5a\xc8')
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0x5a else b''
                break
            del self.buffer[:pos]
            if len(self.buffer) < 46:
                break
            frame = bytes(self.buffer[:46]); body = frame[2:42]
            if (body[0] != 1 or frame[-2:] != b'\r\n'
                    or crc16(body) != int.from_bytes(frame[42:44], 'little')):
                self.bad_frames += 1
                del self.buffer[0]
                continue
            t = LidarPlanningTelemetry(*struct.unpack('<BHI8HBBhhHHhHH', body[1:]))
            if self._semantic_valid(t):
                out.append(t)
            else:
                self.bad_frames += 1
            del self.buffer[:46]
        return out


@dataclass(frozen=True)
class LidarMapTelemetry:
    seq: int
    flags: int
    scan_id: int
    age_ms: int
    known_mask: int
    ranges_mm: tuple


class LidarMapTelemetryParser:
    """Remove complete C9 envelopes before the legacy readers see their range bytes."""
    # A C9 marker can also occur inside an older frame. Respect its full envelope,
    # including fragmented navigation and camera diagnostics of earlier versions.
    _legacy_sizes = {
        0xa5: (28,), 0xb7: (26,), 0xc3: (24, 26, 28, 29, 31),
        0xd4: (16, 28, 32, 36), 0xe5: (46,), 0xf6: (38,), 0xc8: (46,),
    }
    MAX_AGE_MS = 350
    FRAME_SIZE = 90
    UART_TIME_S = FRAME_SIZE * 10 / 115200  # 8N1, age latched before transmission

    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def _decode(self, body):
        version, seq, flags, scan_id, age_ms = struct.unpack_from('<BBBHH', body)
        known = int.from_bytes(body[7:12], 'little')
        ranges = struct.unpack_from('<36H', body, 12)
        if (version != 1 or flags > 1 or known >> 36 or (not flags and known)
                or (flags and age_ms > self.MAX_AGE_MS)
                or any(known & (1 << i) and not 120 <= mm <= 8000
                       for i, mm in enumerate(ranges))):
            return None
        return LidarMapTelemetry(seq, flags, scan_id, age_ms, known, ranges)

    def _wire_size(self, offset):
        kind, version = self.buffer[offset + 1:offset + 3]
        if kind == 0xc9:
            return self.FRAME_SIZE
        sizes = self._legacy_sizes.get(kind, ())
        return sizes[version - 1] if 1 <= version <= len(sizes) else 0

    @staticmethod
    def _integrity_ok(frame):
        return (frame[-2:] == b'\r\n' and
                crc16(frame[2:-4]) == int.from_bytes(frame[-4:-2], 'little'))

    def _resume_after_bad_map(self):
        # Ignore every envelope wholly inside the suspected map, even if its
        # checksum is valid. On byte loss, a genuine following frame may
        # straddle the expected boundary. Only its own checksum can identify it.
        end = self.FRAME_SIZE
        if self.buffer[end:end + 1] == b'\x5a':
            return end
        wait = False
        offset = self.buffer.find(b'\x5a', 2, end)
        while offset != -1:
            if offset + 3 > len(self.buffer):
                wait = True
            else:
                size = self._wire_size(offset)
                stop = offset + size
                if stop > len(self.buffer):
                    wait = True
                elif stop > end and self._integrity_ok(self.buffer[offset:stop]):
                    return offset
            offset = self.buffer.find(b'\x5a', offset + 1, end)
        return None if wait else end

    def feed(self, data, *, legacy=None):
        self.buffer.extend(data)
        maps = []
        while self.buffer:
            sync = self.buffer.find(b'\x5a')
            if sync != 0:
                count = len(self.buffer) if sync < 0 else sync
                if legacy is not None:
                    legacy.extend(self.buffer[:count])
                del self.buffer[:count]
                continue
            if len(self.buffer) < 3:
                break
            kind = self.buffer[1]
            length = self._wire_size(0)
            if not length:
                if legacy is not None:
                    legacy.append(self.buffer[0])
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                break
            frame = self.buffer[:length]
            if not self._integrity_ok(frame):
                if kind == 0xc9:
                    skip = self._resume_after_bad_map()
                    if skip is None:
                        break
                    self.bad_frames += 1
                    del self.buffer[:skip]
                    continue
                if legacy is not None:
                    legacy.append(self.buffer[0])
                del self.buffer[0]
                continue
            if kind == 0xc9:
                scan = self._decode(frame[2:-4])
                if scan is None:
                    self.bad_frames += 1
                else:
                    maps.append(scan)
            elif legacy is not None:
                legacy.extend(frame)
            del self.buffer[:length]
        return maps


class _LidarDiagnosticsReceiver:
    """Read-only FPGA planning and map diagnostics on both serial transports."""
    def _reset_lidar_diagnostics(self):
        self.lidar_map_parser = LidarMapTelemetryParser()
        self.lidar_map_telemetry = None
        self._map_first_rx = self._map_deadline = float('-inf')
        self.lidar_planning_parser = LidarPlanningTelemetryParser()
        self.lidar_planning_telemetry = None
        self.last_lidar_planning_stamp = None
        self.last_lidar_planning_rx = float('-inf')

    def _receive_lidar_diagnostics(self, data, now):
        legacy = bytearray()
        for scan in self.lidar_map_parser.feed(data, legacy=legacy):
            remaining = (self.lidar_map_parser.MAX_AGE_MS - scan.age_ms) / 1000
            deadline = now + remaining - self.lidar_map_parser.UART_TIME_S
            if self.lidar_map_telemetry is None or scan.scan_id != self.lidar_map_telemetry.scan_id:
                self._map_first_rx = now
                self._map_deadline = deadline
            else:
                # Repeating one scan can shorten its lifetime, never extend it.
                self._map_deadline = min(self._map_deadline, deadline)
            if not scan.flags:
                self._map_deadline = float('-inf')
            self.lidar_map_telemetry = scan
        for t in self.lidar_planning_parser.feed(legacy):
            self.lidar_planning_telemetry = t
            # A changing UART seq does not prove that the motion core advanced.
            if t.timestamp_ms != self.last_lidar_planning_stamp:
                self.last_lidar_planning_stamp = t.timestamp_ms
                self.last_lidar_planning_rx = now
        return legacy

    def lidar_map_live(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.lidar_map_telemetry is not None
                and self._map_first_rx <= now <= self._map_deadline)

    def lidar_planning_live(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.lidar_planning_telemetry is not None
                and 0 <= now - self.last_lidar_planning_rx < .4)


@dataclass(frozen=True)
class MotionTelemetry:
    seq: int
    flags: int
    timestamp_ms: int
    imu_heading: int
    heading: int
    encoder_heading: int
    innovation: int
    goal: int
    error: int
    velocity_l: int
    velocity_r: int
    target_l: int
    target_r: int
    trim: int
    yaw_rate: int
    position_l: int
    position_r: int

    @property
    def ready(self): return self.flags & 0x1b == 0x0b
    @property
    def fused(self): return bool(self.flags & 4)
    @property
    def disagreement(self): return bool(self.flags & 32)
    @property
    def holding(self): return bool(self.flags & 64)
    @property
    def encoder_available(self): return bool(self.flags & 1024)
    @property
    def braked(self): return bool(self.flags & 512)


class MotionTelemetryParser:
    """46 bytes: 5A E5, 40-byte v1 snapshot, CRC16 LE, CR LF.

    Separate from legacy/nav/camera parsers; never grants motion permission.
    """
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while len(self.buffer) >= 2:
            pos = self.buffer.find(b'\x5a\xe5')
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0x5a else b''
                break
            del self.buffer[:pos]
            if len(self.buffer) < 46:
                break
            frame = bytes(self.buffer[:46])
            body = frame[2:42]
            if (body[0] != 1 or frame[-2:] != b'\r\n'
                    or crc16(body) != int.from_bytes(frame[42:44], 'little')):
                self.bad_frames += 1
                del self.buffer[0]
                continue
            out.append(MotionTelemetry(*struct.unpack('<BHI12hii', body[1:])))
            del self.buffer[:46]
        return out


@dataclass(frozen=True)
class PredictiveTelemetry:
    seq: int
    flags: int
    timestamp_ms: int
    generation: int
    solve_cycles: int
    candidate: int
    horizon: int
    reference_l: int
    reference_r: int
    output_l: int
    output_r: int
    cost: int
    predicted_error: int
    yaw_gain_q8: int
    alpha_shifts: int
    slew_counts: int

    @property
    def fault(self): return bool(self.flags & 8)
    @property
    def validated(self): return bool(self.flags & 128)
    @property
    def angular_model(self): return bool(self.flags & 16)


class PredictiveTelemetryParser:
    """5A F6 + 32-byte body + CRC16 LE + CR LF; diagnostic only."""
    def __init__(self):
        self.buffer=bytearray()
        self.bad_frames=0

    def feed(self,data):
        self.buffer.extend(data)
        out=[]
        while len(self.buffer)>=2:
            pos=self.buffer.find(b'\x5a\xf6')
            if pos<0:
                self.buffer[:]=self.buffer[-1:] if self.buffer[-1]==0x5a else b''
                break
            del self.buffer[:pos]
            if len(self.buffer)<38:break
            frame=bytes(self.buffer[:38]);body=frame[2:34]
            if body[0]!=1 or frame[-2:]!=b'\r\n' or crc16(body)!=int.from_bytes(frame[34:36],'little'):
                self.bad_frames+=1;del self.buffer[0];continue
            out.append(PredictiveTelemetry(*struct.unpack('<BHIHHBBhhhhIhHBB',body[1:])))
            del self.buffer[:38]
        return out


@dataclass(frozen=True)
class CameraDiagnostic:
    seq: int
    phase: int
    index: int
    chip_id: int
    nacks: int
    flags: int
    last_nack_index: int
    pclk_ticks: Optional[int] = None
    lines: Optional[int] = None
    frames: Optional[int] = None
    packets_started: Optional[int] = None
    packets_done: Optional[int] = None
    dropped: Optional[int] = None
    href_ticks: Optional[int] = None
    vsync_ticks: Optional[int] = None
    line_bytes: Optional[int] = None
    line_max_bytes: Optional[int] = None

    @property
    def flow_summary(self):
        if self.pclk_ticks is None:
            return '无采集计数（旧诊断固件）'
        duty = '' if self.href_ticks is None else f'，H/V高电平 {self.href_ticks}/{self.vsync_ticks}×16'
        if self.line_bytes is not None:
            duty += f'，行字节 {self.line_bytes}/最大{self.line_max_bytes}'
        return (f'每100ms：PCLK {self.pclk_ticks}×256，行 {self.lines}，帧 {self.frames}，'
                f'发包 {self.packets_started}/{self.packets_done}，丢帧 {self.dropped}' + duty)

    @property
    def summary(self):
        if not self.flags & 1:
            return '开机芯片ID探测中'
        if self.flags & 128:
            return f'摄像头通信超时，阶段{self.phase}，进度{self.index}/253'
        if self.flags & 6 != 6:
            progress = '当前初始化完成' if self.flags & 16 else f'初始化{self.index}/253'
            return f'开机芯片ID读取无应答，高/低字节ACK={int(bool(self.flags&2))}/{int(bool(self.flags&4))}；{progress}，NACK={self.nacks}'
        if self.chip_id != 0x5640:
            return f'芯片ID=0x{self.chip_id:04X}，与OV5640不匹配'
        state = '初始化完成' if self.flags & 16 else f'初始化{self.index}/253'
        return f'ID=0x5640，{state}，NACK={self.nacks}'


class CameraDiagnosticParser:
    """CRC-protected diagnostics: v1=16, v2=28, v3=32, v4=36 bytes."""
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while len(self.buffer) >= 2:
            pos = self.buffer.find(b'\x5a\xd4')
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0x5a else b''
                break
            del self.buffer[:pos]
            if len(self.buffer) < 3:
                break
            version = self.buffer[2]
            if version not in (1,2,3,4):
                self.bad_frames += 1
                del self.buffer[0]
                continue
            length = {1:16,2:28,3:32,4:36}[version]
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            body = frame[2:-4]
            if (frame[-2:] != b'\r\n' or body[2] > 7
                    or crc16(body) != int.from_bytes(frame[-4:-2], 'little')):
                self.bad_frames += 1
                del self.buffer[0]
                continue
            out.append(CameraDiagnostic(body[1],body[2],body[3],
                int.from_bytes(body[4:6],'big'),int.from_bytes(body[6:8],'little'),body[8],body[9],
                *(struct.unpack({2:'<6H',3:'<8H',4:'<10H'}[version],body[10:]) if version >= 2 else ())))
            del self.buffer[:length]
        return out


def navigation_packet(seq, opcode, *, index=0, x_mm=0, y_mm=0, count=0):
    """Configuration only; FPGA independently rejects writes while moving/armed."""
    values = (seq, opcode, index, x_mm, y_mm, count)
    if not all(isinstance(v, int) and not isinstance(v, bool) for v in values):
        raise ValueError('navigation configuration requires integer millimetres')
    if opcode == NAV_WRITE:
        valid = 0 <= index < 8 and -10000 <= x_mm <= 10000 and -10000 <= y_mm <= 10000 and count == 0
    elif opcode == NAV_COMMIT:
        valid = index == x_mm == y_mm == 0 and 1 <= count <= 8
    elif opcode in (NAV_POSE_RESET, NAV_CLEAR):
        valid = index == x_mm == y_mm == count == 0
    else:
        valid = False
    if not valid:
        raise ValueError('invalid navigation opcode/index/coordinate/count')
    body = struct.pack('<BBBBhhB5x', 1, seq & 255, opcode, index, x_mm, y_mm, count)
    return b'\xa5\xc3' + body + struct.pack('<H', crc16(body)) + b'\r\n'


@dataclass(frozen=True)
class NavTelemetry:
    seq: int
    state: int
    index: int
    count: int
    flags: int
    x_mm: int
    y_mm: int
    heading: int
    ack_seq: int
    ack_status: int
    stop_epoch: int = 0
    follow_count: Optional[int] = None
    nav_mode_count: Optional[int] = None
    follow_speed: Optional[int] = None
    record_count: Optional[int] = None

    @property
    def active(self): return bool(self.flags & 1)
    @property
    def done(self): return bool(self.flags & 2)
    @property
    def failed(self): return bool(self.flags & 4)
    @property
    def ready(self): return bool(self.flags & 8)
    @property
    def selected(self): return bool(self.flags & 16)
    @property
    def pose_ready(self): return bool(self.flags & 32)
    @property
    def supported(self): return bool(self.flags & 128)
    @property
    def mission_supported(self): return bool(self.flags & 64)


class NavTelemetryParser:
    """Independent parser fed the same bytes as the unchanged legacy parser."""
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while len(self.buffer) >= 2:
            pos = self.buffer.find(b'\x5a\xc3')
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0x5A else b''
                break
            del self.buffer[:pos]
            if len(self.buffer) < 3:
                break
            version=self.buffer[2]
            if version not in (1,2,3,4,5):
                self.bad_frames+=1
                del self.buffer[0]
                continue
            length={1:24,2:26,3:28,4:29,5:31}[version]
            if len(self.buffer)<length:break
            frame = bytes(self.buffer[:length])
            body = frame[2:-4]
            if frame[-2:] != b'\r\n' or crc16(body) != int.from_bytes(frame[-4:-2], 'little'):
                self.bad_frames += 1
                del self.buffer[0]
                continue
            fields = list(struct.unpack('<BBBBBiihBB', body[1:18]))
            stop_epoch = fields[1] >> 4
            fields[1] &= 15
            out.append(NavTelemetry(*fields, stop_epoch=stop_epoch,
                follow_count=int.from_bytes(body[18:20],'little') if version>=2 else None,
                nav_mode_count=int.from_bytes(body[20:22],'little') if version>=3 else None,
                follow_speed=body[22] if version>=4 else None,
                record_count=int.from_bytes(body[23:25],'little') if version>=5 else None))
            del self.buffer[:length]
        return out


@dataclass(frozen=True)
class Telemetry:
    seq: int
    state: int
    health: int
    command: int
    speed: int
    distance_l: int
    distance_r: int
    velocity_l: int
    velocity_r: int
    roll: int
    pitch: int
    yaw: int
    fault: int

    @property
    def sensors_ready(self):
        # Sonar health remains truthful in bits2/3 but is not motion permission.
        return self.health & 0x13 == 0x13


class TelemetryParser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes):
        self.buffer.extend(data)
        out = []
        while len(self.buffer) >= 2:
            pos = self.buffer.find(b'\x5a\xa5')
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0x5A else b''
                break
            del self.buffer[:pos]
            if len(self.buffer) < 28:
                break
            frame = bytes(self.buffer[:28])
            body = frame[2:24]
            if frame[-2:] != b'\r\n' or body[0] != 1 or crc16(body) != int.from_bytes(frame[24:26], 'little'):
                self.bad_frames += 1
                del self.buffer[0]
                continue
            out.append(Telemetry(*struct.unpack('<BBBBBHHhhhhhH', body[1:])))
            del self.buffer[:28]
        return out


class Link(_LidarDiagnosticsReceiver):
    def __init__(self, device: str, *, writable=False):
        import serial
        self.port = serial.Serial(device, 115200, timeout=0, write_timeout=0.2, exclusive=True)
        self.writable = writable
        self.parser = TelemetryParser()
        self.nav_parser = NavTelemetryParser()
        self.camera_parser = CameraDiagnosticParser()
        self.motion_parser = MotionTelemetryParser()
        self.predictive_parser = PredictiveTelemetryParser()
        self.lidar_parser = LidarTelemetryParser()
        self._reset_lidar_diagnostics()
        self.lidar_telemetry = None
        self.last_lidar_rx = float('-inf')
        self.last_lidar_scan = None
        self.predictive_telemetry = None
        self.last_predictive_rx = float('-inf')
        self.last_predictive_seq = None
        self.motion_telemetry = None
        self.last_motion_rx = float('-inf')
        self.last_motion_stamp = None
        self.camera_telemetry = None
        self.last_camera_rx = float('-inf')
        self.last_camera_seq = None
        self.seq = 0
        self.telemetry = None
        self.last_rx = float('-inf')
        self.last_tele_seq = None
        self.nav_telemetry = None
        self.last_nav_rx = float('-inf')
        self.last_nav_seq = None

    def poll(self):
        try:
            data = self.port.read(min(self.port.in_waiting, 4096))
        except OSError:
            self._reset_lidar_diagnostics()
            raise
        data = self._receive_lidar_diagnostics(data, time.monotonic())
        for t in self.parser.feed(data):
            if t.seq != self.last_tele_seq:
                self.telemetry = t
                self.last_tele_seq = t.seq
                self.last_rx = time.monotonic()
        for t in self.lidar_parser.feed(data):
            self.lidar_telemetry = t
            if t.fresh and t.scan_id != self.last_lidar_scan:
                self.last_lidar_scan = t.scan_id
                self.last_lidar_rx = time.monotonic()
        for t in self.predictive_parser.feed(data):
            if t.seq != self.last_predictive_seq:
                self.predictive_telemetry = t
                self.last_predictive_seq = t.seq
                self.last_predictive_rx = time.monotonic()
        for t in self.motion_parser.feed(data):
            self.motion_telemetry = t
            if t.timestamp_ms != self.last_motion_stamp:
                self.last_motion_stamp = t.timestamp_ms
                self.last_motion_rx = time.monotonic()
        for t in self.camera_parser.feed(data):
            if t.seq != self.last_camera_seq:
                self.camera_telemetry = t
                self.last_camera_seq = t.seq
                self.last_camera_rx = time.monotonic()
        for t in self.nav_parser.feed(data):
            if t.seq != self.last_nav_seq:
                self.nav_telemetry = t
                self.last_nav_seq = t.seq
                self.last_nav_rx = time.monotonic()

    def live(self, now=None):
        now = time.monotonic() if now is None else now
        return self.telemetry is not None and 0 <= now - self.last_rx < 0.4

    def lidar_live(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.lidar_telemetry is not None and self.lidar_telemetry.fresh
                and 0 <= now - self.last_lidar_rx < .4)

    def predictive_live(self, now=None):
        now = time.monotonic() if now is None else now
        return self.predictive_telemetry is not None and 0 <= now - self.last_predictive_rx < .4

    def motion_live(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.motion_telemetry is not None and self.motion_telemetry.ready
                and 0 <= now - self.last_motion_rx < .4)

    def nav_live(self, now=None):
        now = time.monotonic() if now is None else now
        return self.nav_telemetry is not None and 0 <= now - self.last_nav_rx < .4

    def send(self, armed=False, command=0, speed=0, *, nav=False, follow=False):
        if not self.writable:
            raise PermissionError('receive-only link; enable driving explicitly before writing')
        packet = command_packet(self.seq, armed, command, speed, nav=nav, follow=follow)
        self.seq = (self.seq + 1) & 255
        if self.port.write(packet) != len(packet):
            raise OSError('short serial write')

    def configure_navigation(self, opcode, **fields):
        """Return the sequence to await in a fresh NavTelemetry ACK; never ARM."""
        if not self.writable:
            raise PermissionError('receive-only link; enable driving explicitly before writing')
        seq = self.seq
        packet = navigation_packet(seq, opcode, **fields)
        self.seq = (self.seq + 1) & 255
        if self.port.write(packet) != len(packet):
            raise OSError('short navigation configuration write')
        return seq

    def close(self):
        self._reset_lidar_diagnostics()
        if not self.port.is_open:
            return
        try:
            if self.writable:
                for _ in range(3):
                    self.send()
                    time.sleep(0.02)
        finally:
            self.port.close()


class PosixControlLink:
    """Explicit UART writer sharing the viewer's one exclusive serial reader."""
    def __init__(self, device):
        self.reader = PosixTelemetryLink(device)
        self.seq = 0

    def __getattr__(self, name):
        return getattr(self.reader, name)

    def send(self, armed=False, command=0, speed=0, *, nav=False, follow=False):
        import os
        if self.reader.fd is None:
            raise OSError('FPGA serial unavailable')
        packet=command_packet(self.seq,armed,command,speed,nav=nav,follow=follow)
        self.seq=(self.seq+1)&255
        self.reader._write_serial_packet(packet)

    def configure_navigation(self, opcode, **fields):
        import os
        if self.reader.fd is None:
            raise OSError('FPGA serial unavailable')
        seq = self.seq
        packet = navigation_packet(seq, opcode, **fields)
        self.seq = (self.seq + 1) & 255
        self.reader._write_serial_packet(packet)
        return seq

    def close(self):
        try:
            if self.reader.fd is not None:
                for _ in range(3):self.send()
        finally:
            self.reader.close()


class PosixTelemetryLink(_LidarDiagnosticsReceiver):
    """Receive-only serial access for the GTK/curses viewers (no pyserial needed).

    Reconnect resets the frame parser and freshness. Exclusive locking prevents
    our two viewers from consuming alternate halves of one telemetry stream.
    """
    def __init__(self, device):
        self._tx_lock=threading.RLock()
        self._control_pending=threading.Event()
        self.lidar_bridge_epoch=0
        self._lidar_seq=0
        self._lidar_reset_required=True
        self.last_lidar_frame_rx=float('-inf')
        self.last_lidar_frame_seq=None
        self.device = device
        self.fd = None
        self.old = None
        self.retry = 0.0
        self.parser = TelemetryParser()
        self.nav_parser = NavTelemetryParser()
        self.camera_parser = CameraDiagnosticParser()
        self.motion_parser = MotionTelemetryParser()
        self.predictive_parser = PredictiveTelemetryParser()
        self.lidar_parser = LidarTelemetryParser()
        self._reset_lidar_diagnostics()
        self.lidar_telemetry = None
        self.last_lidar_rx = float('-inf')
        self.last_lidar_scan = None
        self.predictive_telemetry = None
        self.last_predictive_rx = float('-inf')
        self.last_predictive_seq = None
        self.motion_telemetry = None
        self.last_motion_rx = float('-inf')
        self.last_motion_stamp = None
        self.camera_telemetry = None
        self.last_camera_rx = float('-inf')
        self.last_camera_seq = None
        self.telemetry = None
        self.last_rx = float('-inf')
        self.last_seq = None
        self.frames = 0
        self.bad_frames = 0
        self.error = ''
        self.nav_telemetry = None
        self.last_nav_rx = float('-inf')
        self.last_nav_seq = None

    def _disconnect(self):
        with self._tx_lock:
            self._disconnect_locked()

    def _disconnect_locked(self):
        import os
        import termios
        self.lidar_bridge_epoch+=1
        self._lidar_reset_required=True
        self._lidar_seq=0
        self.last_lidar_frame_rx=float('-inf')
        self.last_lidar_frame_seq=None
        fd, old = self.fd, self.old
        self.fd = self.old = None
        if fd is not None:
            try:
                if old is not None:
                    termios.tcsetattr(fd, termios.TCSANOW, old)
            except (OSError, termios.error):
                pass
            finally:
                os.close(fd)
        self.telemetry = None
        self.last_rx = float('-inf')
        self.last_seq = None
        self.bad_frames += self.parser.bad_frames
        self.parser = TelemetryParser()
        self.nav_parser = NavTelemetryParser()
        self.camera_parser = CameraDiagnosticParser()
        self.motion_parser = MotionTelemetryParser()
        self.predictive_parser = PredictiveTelemetryParser()
        self.lidar_parser = LidarTelemetryParser()
        self._reset_lidar_diagnostics()
        self.lidar_telemetry = None
        self.last_lidar_rx = float('-inf')
        self.last_lidar_scan = None
        self.predictive_telemetry = None
        self.last_predictive_rx = float('-inf')
        self.last_predictive_seq = None
        self.motion_telemetry = None
        self.last_motion_rx = float('-inf')
        self.last_motion_stamp = None
        self.camera_telemetry = None
        self.last_camera_rx = float('-inf')
        self.last_camera_seq = None
        self.nav_telemetry = None
        self.last_nav_rx = float('-inf')
        self.last_nav_seq = None

    def lidar_bridge_ready(self, now=None):
        now=time.monotonic() if now is None else now
        t=self.lidar_telemetry
        return (self.fd is not None and isinstance(t,LidarTelemetry)
                and t.usb_bridge_supported and t.usb_source
                and 0 <= now-self.last_lidar_frame_rx < .4)

    def _pending_output_bytes(self):
        import array,fcntl,termios
        queued=array.array('i',[0])
        fcntl.ioctl(self.fd,termios.TIOCOUTQ,queued,True)
        return max(0,queued[0])

    def _write_serial_packet(self, packet):
        """Only the explicit control owner calls this; raw sends yield to it."""
        import os
        self._control_pending.set()
        try:
            with self._tx_lock:
                if self.fd is None:raise OSError('FPGA serial unavailable')
                if os.write(self.fd,packet)!=len(packet):
                    raise OSError('short UART control write')
        finally:self._control_pending.clear()

    def send_lidar_bytes(self, payload, *, reset=False, now=None):
        """Nonblocking sensor path, never grants ARM or claims motion ownership."""
        import os
        now=time.monotonic() if now is None else now
        if self._control_pending.is_set() or not self._tx_lock.acquire(blocking=False):return False
        try:
            if self._control_pending.is_set() or not self.lidar_bridge_ready(now):return False
            if self._lidar_reset_required and not reset:return False
            packet=lidar_forward_packet(self._lidar_seq,payload,reset=reset)
            # Include the complete escaped frame, including the worst-case271B.
            # Bound enqueueing rather than filling a seconds-long kernel queue.
            if self._pending_output_bytes()+len(packet)>384:return False
            try:written=os.write(self.fd,packet)
            except BlockingIOError:return False
            except OSError:
                self._lidar_reset_required=True
                raise
            if written!=len(packet):
                self._lidar_reset_required=True
                raise OSError('short UART lidar write; RESET required')
            self._lidar_seq=(self._lidar_seq+1)&255
            self._lidar_reset_required=False
            return True
        finally:self._tx_lock.release()

    def poll(self, now=None):
        import fcntl
        import os
        import termios
        import tty
        now = time.monotonic() if now is None else now
        if self.fd is None and now >= self.retry:
            try:
                self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.old = termios.tcgetattr(self.fd)
                tty.setraw(self.fd, termios.TCSANOW)
                cfg = termios.tcgetattr(self.fd)
                cfg[4] = cfg[5] = termios.B115200
                cfg[2] = (cfg[2] | termios.CLOCAL | termios.CREAD) & ~termios.CRTSCTS
                # Nonblocking read distinguishes idle EAGAIN from unplug EOF.
                cfg[6][termios.VMIN] = 1
                cfg[6][termios.VTIME] = 0
                termios.tcsetattr(self.fd, termios.TCSANOW, cfg)
                self.lidar_bridge_epoch+=1
                self._lidar_reset_required=True
                self.error = ''
            except (OSError, termios.error) as exc:
                self._disconnect()
                self.error = str(exc)
                self.retry = now + 2
        if self.fd is not None:
            try:
                data = os.read(self.fd, 4096)
                if not data:
                    raise OSError('serial disconnected')
                data = self._receive_lidar_diagnostics(data, now)
                for frame in self.parser.feed(data):
                    self.frames += 1
                    if frame.seq != self.last_seq:
                        self.telemetry = frame
                        self.last_seq = frame.seq
                        self.last_rx = now
                for frame in self.lidar_parser.feed(data):
                    self.lidar_telemetry = frame
                    if frame.seq!=self.last_lidar_frame_seq:
                        self.last_lidar_frame_seq=frame.seq
                        self.last_lidar_frame_rx=now
                    if frame.fresh and frame.scan_id != self.last_lidar_scan:
                        self.last_lidar_scan = frame.scan_id
                        self.last_lidar_rx = now
                for frame in self.predictive_parser.feed(data):
                    if frame.seq != self.last_predictive_seq:
                        self.predictive_telemetry = frame
                        self.last_predictive_seq = frame.seq
                        self.last_predictive_rx = now
                for frame in self.motion_parser.feed(data):
                    self.motion_telemetry = frame
                    if frame.timestamp_ms != self.last_motion_stamp:
                        self.last_motion_stamp = frame.timestamp_ms
                        self.last_motion_rx = now
                for frame in self.camera_parser.feed(data):
                    if frame.seq != self.last_camera_seq:
                        self.camera_telemetry = frame
                        self.last_camera_seq = frame.seq
                        self.last_camera_rx = now
                for frame in self.nav_parser.feed(data):
                    if frame.seq != self.last_nav_seq:
                        self.nav_telemetry = frame
                        self.last_nav_seq = frame.seq
                        self.last_nav_rx = now
            except BlockingIOError:
                pass
            except OSError as exc:
                self._disconnect()
                self.error = str(exc)
                self.retry = now + 2

    def live(self, now=None):
        now = time.monotonic() if now is None else now
        return self.telemetry is not None and now - self.last_rx < 0.5

    def lidar_live(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.lidar_telemetry is not None and self.lidar_telemetry.fresh
                and 0 <= now - self.last_lidar_rx < .4)

    def predictive_live(self, now=None):
        now = time.monotonic() if now is None else now
        return self.predictive_telemetry is not None and 0 <= now - self.last_predictive_rx < .4

    def motion_live(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.motion_telemetry is not None and self.motion_telemetry.ready
                and 0 <= now - self.last_motion_rx < .4)

    def nav_live(self, now=None):
        now = time.monotonic() if now is None else now
        return self.nav_telemetry is not None and 0 <= now - self.last_nav_rx < .4

    @property
    def crc_errors(self):
        return self.bad_frames + self.parser.bad_frames

    def close(self):
        self._disconnect()
