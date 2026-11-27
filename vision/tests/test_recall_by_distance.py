"""vision/recall_by_distance.py with a fake detector (no ONNX)."""
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "gurdy_core"))
from gurdy_core.vision.classes import CLASSES_V3  # noqa: E402
from gurdy_core.vision.detector import Detection  # noqa: E402
from vision import recall_by_distance as R  # noqa: E402

CAM = {"width": 320, "height": 240, "hfov": 1.047}


def test_bins_tiny_and_hits(tmp_path):
    d = tmp_path / "ds"
    for sub in ("meta", "images", "labels"):
        (d / sub / "test").mkdir(parents=True)
    Image.fromarray(np.zeros((240, 320, 3), np.uint8)).save(d / "images" / "test" / "s0000_v00.png")
    gt = [{"cls": "can", "uv": [100, 100], "p_cam": [0, 0, 0.8]},             # < 1 m, labelled, detected
          {"cls": "small_box", "uv": [200, 50], "p_cam": [0, 0, 2.5]},        # 2-3 m, 5 cm cube ~5.5 px: tiny, unlabelled, missed
          {"cls": "mini_toolbox", "uv": [50, 200], "p_cam": [0, 0, 1.5]},     # 1-2 m, unlabelled but big enough: occluded
          {"cls": "chair", "uv": [10, 10], "p_cam": [0, 0, 1.0]}]             # not a small class: ignored
    (d / "meta" / "test" / "s0000_v00.json").write_text(json.dumps({"camera": CAM, "gt": gt}))
    can = CLASSES_V3.index("can")
    (d / "labels" / "test" / "s0000_v00.txt").write_text(f"{can} {100/320} {100/240} {30/320} {40/240}\n")
    det = lambda rgb: [Detection(can, "can", 0.9, (85, 80, 115, 120))]       # noqa: E731
    st = R.evaluate_split(det, d, "test")
    assert st["< 1 m"]["n"] == 1 and st["< 1 m"]["labelled"] == 1 and st["< 1 m"]["hit_labelled"] == 1
    assert st["2-3 m"]["tiny_unlabelled"] == 1 and st["2-3 m"]["hit"] == 0
    assert st["1-2 m"]["other_unlabelled"] == 1
    assert sum(s["n"] for s in st.values()) == 3
    rows = R.table("x", st)
    assert any(r.startswith("| all | 3 | 1 | 1 | 1 | 33.3 %") for r in rows)
