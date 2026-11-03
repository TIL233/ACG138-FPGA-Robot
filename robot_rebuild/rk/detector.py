"""YOLOv8 COCO ONNX detection. Only person(0) and bottle(39); no color logic.

A COCO bottle category recognizes the container, not its contents. Actual water
bottles must be included in the on-board acceptance set.
"""
from pathlib import Path
import os
import select
import subprocess
import struct
import sys
import threading
import time
import numpy as np

LABELS = {0: 'person', 39: 'bottle'}


def decode_predictions(output, scale, pad, original_shape, threshold=0.45, iou=0.45):
    p = np.asarray(output)
    if p.ndim != 3 or p.shape[0] != 1 or p.shape[1] != 84:
        raise ValueError(f'expected YOLOv8 raw [1,84,N], got {p.shape}; re-export using tools/export_model.py')
    rows = p[0].T
    class_ids = rows[:, 4:].argmax(axis=1)
    scores = rows[np.arange(len(rows)), 4+class_ids]
    keep = np.isin(class_ids, list(LABELS)) & (scores >= threshold) & np.isfinite(rows).all(axis=1)
    indices = np.flatnonzero(keep)
    h, w = original_shape[:2]
    detections = []
    for k in indices:
        x, y, bw, bh = rows[k, :4]
        box = np.array([(x-bw/2-pad[0])/scale, (y-bh/2-pad[1])/scale,
                        (x+bw/2-pad[0])/scale, (y+bh/2-pad[1])/scale])
        box[[0, 2]] = np.clip(box[[0, 2]], 0, w-1)
        box[[1, 3]] = np.clip(box[[1, 3]], 0, h-1)
        if box[2] > box[0] and box[3] > box[1]:
            detections.append((int(class_ids[k]), float(scores[k]), box))
    # Class-aware NMS; person must not suppress a held bottle.
    detections.sort(key=lambda d: d[1], reverse=True)
    out = []
    while detections:
        winner = detections.pop(0)
        out.append(winner)
        remaining = []
        a = winner[2]
        for other in detections:
            b = other[2]
            intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
            union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1])-intersection
            if other[0] != winner[0] or intersection/max(union, 1e-9) <= iou:
                remaining.append(other)
        detections = remaining
    return [(c, s, tuple(int(v) for v in b)) for c, s, b in out]


class Detector:
    def __init__(self, model, size=320):
        import cv2
        if not Path(model).is_file():
            raise FileNotFoundError(f'ONNX model missing: {model}; run tools/export_model.py first')
        self.cv = cv2
        self.size = size
        self.net = cv2.dnn.readNetFromONNX(str(model))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect(self, image):
        cv = self.cv
        h, w = image.shape[:2]
        scale = min(self.size/w, self.size/h)
        rw, rh = round(w*scale), round(h*scale)
        px, py = (self.size-rw)//2, (self.size-rh)//2
        resized = cv.resize(image, (rw, rh))
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[py:py+rh, px:px+rw] = resized
        self.net.setInput(cv.dnn.blobFromImage(canvas, 1/255., (self.size, self.size), swapRB=True))
        return decode_predictions(self.net.forward(), scale, (px, py), image.shape)


class DetectionWorker:
    def __init__(self, device, model):
        self.detector = Detector(model)
        self.device = device
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = (None, (), 0.0, 0.0)
        self.error = None
        # Isolate CSI/ISP capture in a child so shutdown never calls release() concurrently
        # with a native OpenCV read in another thread.
        self.process = subprocess.Popen(
            [sys.executable, str(Path(__file__).with_name('csi_capture.py')), str(device)],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        self.thread = threading.Thread(target=self._run, daemon=True)
        try:
            self.thread.start()
        except BaseException:
            self._close_capture()
            raise

    def _read_frame(self, timeout):
        data = bytearray()
        size = 8 + 640 * 480 * 3
        deadline = time.monotonic() + timeout
        fd = self.process.stdout.fileno()
        while len(data) < size and not self.stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('OV5647 camera capture timed out')
            ready, _, _ = select.select([fd], [], [], min(remaining, 0.2))
            if not ready:
                continue
            chunk = os.read(fd, size - len(data))
            if not chunk:
                raise OSError('OV5647 camera disconnected or capture failed')
            data.extend(chunk)
        if self.stop_event.is_set():
            return None
        captured = struct.unpack_from('<d', data)[0]
        return np.frombuffer(data, np.uint8, offset=8).reshape(480, 640, 3), captured

    def _close_capture(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1)
        self.process.stdout.close()

    def _run(self):
        try:
            first = True
            while not self.stop_event.is_set():
                frame = self._read_frame(5 if first else 2)
                if frame is None:
                    break
                first = False
                image, captured = frame
                start = time.monotonic()
                detections = self.detector.detect(image)
                duration = time.monotonic()-start
                with self.lock:
                    self.latest = (image, detections, captured, duration)
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = str(exc)
        finally:
            self._close_capture()

    def snapshot(self):
        with self.lock:
            return self.latest

    def close(self):
        self.stop_event.set()
        # A blocked capture read is bounded to 200 ms. Only its owning worker
        # closes pipes; shutdown therefore cannot race os.read().
        self.thread.join(3)
        if self.thread.is_alive():
            if self.process.poll() is None:
                self.process.terminate()
            raise TimeoutError('detector worker did not finish within 3 seconds')
