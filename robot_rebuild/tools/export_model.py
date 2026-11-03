"""Run in a separate Python export environment, not on the FPGA or motor loop.
pip install ultralytics==8.3.0 onnx==1.16.2
Uses Ultralytics official YOLOv8n COCO weights. No custom colors/classes training.
"""
from pathlib import Path
import hashlib
import json
import shutil


def main():
    from ultralytics import YOLO
    root = Path(__file__).resolve().parents[1]
    out = root/'rk/models'
    out.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(out/'yolov8n.pt'))
    exported = Path(model.export(format='onnx', imgsz=320, opset=12,
                                 simplify=False, dynamic=False, device='cpu'))
    target = out/'yolov8n.onnx'
    if exported.resolve() != target.resolve():
        shutil.copyfile(exported, target)
    (out/'manifest.json').write_text(json.dumps({
        'model': 'YOLOv8n COCO', 'input': [1, 3, 320, 320], 'output': '[1,84,2100]',
        'classes_used': {'0': 'person', '39': 'bottle'},
        'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
        'exporter': 'ultralytics==8.3.0',
        'documentation': 'https://docs.ultralytics.com/modes/export/',
        'license': 'See upstream Ultralytics AGPL-3.0 / enterprise terms',
    }, indent=2), encoding='utf-8')
    print(target)


if __name__ == '__main__':
    main()
