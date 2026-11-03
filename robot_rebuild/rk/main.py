"""Standalone OV5647 object detection and FPGA telemetry; manual driving only.

The installed GTK viewer also supports OV5647; see docs/OV5647接入.md.
"""
from pathlib import Path
import argparse
import logging
import math
import time
import numpy as np

from protocol import Link, RANGE_FAR, RANGE_INVALID
from detector import DetectionWorker, LABELS


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial', help='/dev/serial/by-id/... for FPGA CH340, NOT RK debug UART')
    parser.add_argument('--camera', default=str(Path(__file__).with_name('camera_config.json')),help='OV5647 camera configuration JSON (CSI0/FPC2 by default)')
    parser.add_argument('--model', default=str(Path(__file__).parent/'models/yolov8n.onnx'))
    parser.add_argument('--no-detection', action='store_true', help='task1/2/3 bring-up only')
    parser.add_argument('--enable-drive', action='store_true', help='enable manual ARM control')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float, default=0)
    args = parser.parse_args(argv)
    if args.enable_drive and (not args.serial or args.headless):
        parser.error('driving needs --serial and the interactive window')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    import cv2
    logging.basicConfig(level=logging.INFO)
    link = detection = None
    armed = False
    command = 0
    speed = 16
    deadline = 0.0
    last_send = 0.0
    start = time.monotonic()
    title = 'Robot: OV5647 detection + telemetry'
    result = 0
    window_open = False
    try:
        if args.serial:
            link = Link(args.serial, writable=args.enable_drive)
            if args.enable_drive:
                link.send()  # UART ownership is claimed only in drive mode.
        if not args.no_detection:
            detection = DetectionWorker(args.camera, args.model)
        if not args.headless:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            window_open = True
        while not args.duration or time.monotonic()-start < args.duration:
            now = time.monotonic()
            if link:
                link.poll()
                if not link.live() or (link.telemetry and link.telemetry.fault):
                    armed = False
                    command = 0
            if now >= deadline:
                command = 0
            # The UI thread sends heartbeats. If it freezes, FPGA watchdog stops.
            if args.enable_drive and link and now-last_send >= 0.1:
                link.send(armed, command if armed else 0, speed if armed else 0)
                last_send = now
            if detection and detection.error:
                raise OSError(f'OV5647 detection failed: {detection.error}')
            if args.headless:
                time.sleep(0.01)
                continue
            canvas = np.zeros((580, 1280, 3), np.uint8)
            if detection:
                image, objects, captured, cost = detection.snapshot()
                if image is not None and now-captured < 2:
                    view = image.copy()
                    for label, score, (x1, y1, x2, y2) in objects:
                        cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(view, f'{LABELS[label]} {score:.2f}', (x1, max(16, y1)), 0, 0.6, (0, 255, 0), 2)
                    canvas[:480, 320:960] = cv2.resize(view, (640, 480))
                    cv2.putText(canvas, f'OV5647 inference {cost*1000:.0f}ms', (10, 465), 0, 0.5, (255, 255, 255), 1)
                else:
                    message = detection.error or 'OV5647 DETECTION WAITING / STALE'
                    cv2.putText(canvas, message[:65], (10, 220), 0, 0.5, (0, 0, 255), 1)
            tele = link.telemetry if link and link.live() else None
            if tele:
                def ranging(v,bit):
                    if not tele.health&bit or v==RANGE_INVALID:return 'INVALID'
                    return 'FAR/no-echo' if v==RANGE_FAR else str(v)
                text = (f'R/P/Y {tele.roll/100:.2f}/{tele.pitch/100:.2f}/{tele.yaw/100:.2f}deg  '
                        f'D(mm) {ranging(tele.distance_l,4)}/{ranging(tele.distance_r,8)}  V(cnt/20ms) {tele.velocity_l}/{tele.velocity_r}  '
                        f'ST {tele.state} FAULT {tele.fault:04x}')
            else:
                text = 'TELEMETRY STALE / NOT CONNECTED - DRIVE DISABLED'
            cv2.putText(canvas, text, (10, 500), 0, 0.5, (255, 255, 255), 1)
            controls = (f'ARM={armed} CMD={command} speed={speed}  E arm | SPACE disarm | W/S/A/D move | Q/R spin | ESC exit'
                        if args.enable_drive else 'RECEIVE ONLY: remote control retained | ESC exit')
            cv2.putText(canvas, controls,
                        (10, 530), 0, 0.5, (0, 255, 255), 1)
            cv2.putText(canvas, 'Hold/repeat motion key: 400ms deadman. Detection NEVER commands motion.',
                        (10, 560), 0, 0.5, (255, 255, 255), 1)
            cv2.imshow(title, canvas)
            key = cv2.waitKey(10) & 255
            if key == 27 or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord(' ') and args.enable_drive:
                armed = False
                command = 0
                if link:
                    link.send()
            elif key == ord('e') and args.enable_drive and tele and tele.sensors_ready and not tele.fault:
                armed = True
                command = 0
            elif key in map(ord, 'wsadqr') and armed:
                command = dict(zip(map(ord, 'wsadqr'), (1, 2, 3, 4, 5, 6)))[key]
                deadline = time.monotonic()+0.4
            elif key in (ord('+'), ord('=')):
                speed = min(64, speed+2)
            elif key == ord('-'):
                speed = max(4, speed-2)
    except KeyboardInterrupt:
        pass
    except Exception:
        logging.exception('robot application failed')
        result = 1
    finally:
        # Stop motors BEFORE waiting for camera/network worker cleanup.
        for name, resource in [('serial', link), ('detection', detection)]:
            if resource is None:
                continue
            try:
                resource.close()
            except Exception:
                logging.exception('failed to close %s', name)
                result = 1
        if window_open:
            try:
                cv2.destroyAllWindows()
            except Exception:
                logging.exception('failed to close display window')
                result = 1
    return result


if __name__ == '__main__':
    raise SystemExit(main())
