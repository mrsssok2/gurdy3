#!/usr/bin/env python3
"""Train a small YOLO detector on the synthetic dataset and export ONNX (GPU, vision image).

    docker compose run --rm vision python3 vision/train.py --data datasets/synth_v1 --name det_v1

Default model yolo11n (2.6 M params; batch 32 at 320 px needs ~2 GB VRAM, far below the 6 GB of the RTX 4050).
Ultralytics is AGPL-3.0 (see docs/vision_objects.md): it is used ONLY here, in the training image. The result is
exported to ONNX and the ROS node runs it through onnxruntime + gurdy_core.vision.detector (no Ultralytics import).
Exports to models/detector/<name>.onnx at 256x320 (the camera's 4:3 shape, multiples of 32) + <name>.json.
"""
import argparse
import json
import shutil
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--data", default=".", help="dataset dir with dataset.yaml (vision/generate_dataset.py finalize)")
ap.add_argument("--name", default="det_v1")
ap.add_argument("--model", default="yolo11n.pt", help="Ultralytics weights/config (downloaded on first use)")
ap.add_argument("--epochs", type=int, default=60)
ap.add_argument("--patience", type=int, default=20, help="early stopping: epochs without val improvement")
ap.add_argument("--imgsz", type=int, default=320)
ap.add_argument("--batch", type=int, default=32)
ap.add_argument("--workers", type=int, default=4)
ap.add_argument("--device", default="0", help="'0' = first GPU, 'cpu'")
ap.add_argument("--data-yaml", default="", help="use this dataset yaml instead of <data>/dataset.yaml (e.g. several datasets merged)")
ap.add_argument("--export-only", action="store_true")
ap.add_argument("--time-h", type=float, default=0.0, help="wall-time budget in hours (Ultralytics `time`, overrides --epochs; "
                "the LR schedule follows it). 0 = off (default): train for --epochs")
a = ap.parse_args()

from ultralytics import YOLO  # noqa: E402  (import late: --help works without it)

data = str(Path(a.data_yaml).resolve()) if a.data_yaml else str(Path(a.data).resolve() / "dataset.yaml")
runs = Path("vision/runs").resolve()
if not a.export_only:
    YOLO(a.model).train(data=data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, device=a.device, workers=a.workers,
                        project=str(runs), name=a.name, exist_ok=True, patience=a.patience, cos_lr=True, close_mosaic=10,
                        fliplr=0.5, flipud=0.0, hsv_h=0.02, hsv_s=0.5, hsv_v=0.4, degrees=0.0, amp=True, plots=True, seed=0,
                        **({"time": a.time_h} if a.time_h > 0 else {}))
best = runs / a.name / "weights" / "best.pt"
onnx = YOLO(str(best)).export(format="onnx", imgsz=[256, 320], opset=12, simplify=True, dynamic=False, half=False)
dst = Path("models/detector")
dst.mkdir(parents=True, exist_ok=True)
shutil.copy(onnx, dst / f"{a.name}.onnx")
(dst / f"{a.name}.json").write_text(json.dumps({"input_hw": [256, 320], "classes": YOLO(str(best)).names,
                                                "trainer": "ultralytics (AGPL-3.0)", "data": data, "epochs": a.epochs,
                                                **({"time_h": a.time_h} if a.time_h > 0 else {}),
                                                "base": a.model}, indent=1))
print("exported", dst / f"{a.name}.onnx")
