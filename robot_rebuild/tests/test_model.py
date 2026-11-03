"""Real exported ONNX/OpenCV smoke test; separate from synthetic postprocessing tests."""
from pathlib import Path
import sys
import time
import json
import numpy as np

root=Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'rk'))
from detector import Detector, LABELS


def main():
    import cv2
    detector=Detector(root/'rk/models/yolov8n.onnx')
    blank=detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))
    assert not blank, blank
    # Test image bundled with the official Ultralytics wheel, not a private image.
    image_path=root/'tests/export_deps/ultralytics/assets/bus.jpg'
    image=cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(image_path)
    started=time.monotonic()
    result=detector.detect(image)
    elapsed=time.monotonic()-started
    assert any(c==0 for c,s,b in result), result
    assert all(c in (0,39) for c,s,b in result), result
    report={'model':'yolov8n.onnx','backend':'OpenCV CPU','host':'Windows development PC, NOT RK3588',
            'blank_frame_detections':len(blank),'official_bus_image_detections':[
                {'label':LABELS[c],'confidence':round(s,4),'box':b} for c,s,b in result],
            'single_inference_ms':round(elapsed*1000,2),
            'limits':'No real water-bottle acceptance set; no USB/CSI/ARM/NPU hardware test.'}
    (root/'docs/model_test.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('PASS real ONNX model:',len(result),'person/bottle detections;',round(elapsed*1000,1),'ms on development PC')


if __name__=='__main__':
    main()
