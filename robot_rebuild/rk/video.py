"""Strict RBV1 RGB565 row receiver. No RAW interpretation or pair swapping."""
import socket
import struct
import threading
import time

HEADER = struct.Struct('<4sHHHHBBH')


class FrameAssembler:
    def __init__(self, width=640, height=480, max_age=0.5):
        if not 1 <= width <= 1024 or not 1 <= height <= 1080:
            raise ValueError('unsupported geometry')
        self.width, self.height, self.max_age = width, height, max_age
        self.active = None
        self.last_started = None
        self.next_row = 0
        self.started_at = 0.0
        self.image = bytearray(width * height * 2)
        self.bad_packets = 0
        self.complete_frames = 0

    def push(self, packet, now=None):
        now = time.monotonic() if now is None else now
        if len(packet) != HEADER.size + self.width * 2:
            self.bad_packets += 1
            self.active = None
            return None
        magic, seq, row, w, h, fmt, flags, nbytes = HEADER.unpack_from(packet)
        if (magic, w, h, fmt, flags, nbytes) != (b'RBV1', self.width, self.height, 1, 0, self.width * 2) or row >= h:
            self.bad_packets += 1
            self.active = None
            return None
        if row == 0:
            # Reject duplicate/older frame starts, but allow a sender reboot after idle.
            if self.last_started is not None and now-self.started_at < 2:
                difference = (seq-self.last_started) & 65535
                if not 0 < difference < 32768:
                    return None
            self.active = seq
            self.last_started = seq
            self.started_at = now
            self.next_row = 0
        if self.active != seq:
            return None
        if row != self.next_row or now-self.started_at > self.max_age:
            self.active = None
            self.bad_packets += 1
            return None
        offset = row * w * 2
        self.image[offset:offset+w*2] = packet[HEADER.size:]
        self.next_row += 1
        if self.next_row == h:
            self.active = None
            self.complete_frames += 1
            return bytes(self.image)
        return None


def rgb565_to_bgr(data, width, height):
    import numpy as np
    a = np.frombuffer(data, dtype='<u2').reshape(height, width)
    out = np.empty((height, width, 3), dtype=np.uint8)
    r, g, b = (a >> 11) & 31, (a >> 5) & 63, a & 31
    out[:, :, 0] = (b << 3) | (b >> 2)
    out[:, :, 1] = (g << 2) | (g >> 4)
    out[:, :, 2] = (r << 3) | (r >> 2)
    return out


class VideoReceiver:
    def __init__(self, width=640, height=480, port=6102):
        self.assembler = FrameAssembler(width, height)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
            self.sock.bind(('0.0.0.0', port))
            self.sock.settimeout(0.2)
        except BaseException:
            self.sock.close()
            raise
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = (None, 0.0)
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stop_event.is_set():
                try:
                    packet, addr = self.sock.recvfrom(65535)
                except socket.timeout:
                    continue
                if addr[0] != '192.168.0.2':
                    continue
                frame = self.assembler.push(packet)
                if frame is not None:
                    image = rgb565_to_bgr(frame, self.assembler.width, self.assembler.height)
                    with self.lock:
                        self.latest = (image, time.monotonic())
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = str(exc)

    def snapshot(self):
        with self.lock:
            return self.latest

    def close(self):
        self.stop_event.set()
        self.sock.close()
        self.thread.join(1)
