import json, sys
from collections import Counter
from pathlib import Path
import numpy as np
from PIL import Image
sys.path.insert(0, "gurdy_core")
from gurdy_core.vision.classes import CLASSES_V3
from gurdy_core.vision.detector import OnnxDetector
det = OnnxDetector("models/detector/det_v31_s.onnx", "cpu", conf=0.25, names=CLASSES_V3)
for v in ("white", "normals"):
    D = Path(f"datasets/cone_check_{v}"); c = Counter(); n = 0
    for mf in sorted((D / "meta/test").glob("*.json")):
        m = json.loads(mf.read_text())
        gts = [g for g in m["gt"] if g["cls"] == "mini_cone" and 0 <= g["uv"][0] < 320 and 0 <= g["uv"][1] < 240 and g["p_cam"][2] < 2.0]
        if not gts: continue
        dets = det(np.asarray(Image.open(D / "images/test" / (mf.stem + ".png")).convert("RGB")))
        for g in gts:
            n += 1
            u, w = g["uv"]
            hits = [d for d in dets if d.box[0] - 2 <= u <= d.box[2] + 2 and d.box[1] - 2 <= w <= d.box[3] + 2]
            c[max(hits, key=lambda d: d.score).name if hits else "(nothing)"] += 1
    print(v, n, "cones < 2 m ->", ", ".join(f"{k} {x}" for k, x in c.most_common()))
