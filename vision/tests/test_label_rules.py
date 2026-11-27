"""Dataset v3.1 label rule (2026-11-27 audit): per-class size thresholds, unfiltered boxes in meta, relabel without Gazebo."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "gurdy_core"))
from gurdy_core.vision.classes import CLASSES_V3  # noqa: E402
from vision import scene as S  # noqa: E402
from vision import generate_dataset as GD  # noqa: E402

SMALL_BOX, CHAIR = CLASSES_V3.index("small_box"), CLASSES_V3.index("chair")
TINY = [(SMALL_BOX, 10.0, 10.0, 17.0, 17.0), (CHAIR, 50.0, 50.0, 57.0, 57.0)]     # 7 x 7 px each (49 px^2)


def test_v3_rule_is_the_old_default():
    assert S.label_thresholds("v3", CLASSES_V3) is None
    assert S.yolo_lines(TINY, 320, 240) == S.yolo_lines(TINY, 320, 240, thresholds=None) == []


def test_v31_keeps_tiny_small_objects_only():
    thr = S.label_thresholds("v3.1", CLASSES_V3)
    assert thr[SMALL_BOX] == (3.0, 12.0) and thr[CHAIR] == (6.0, 80.0)
    lines = S.yolo_lines(TINY, 320, 240, thresholds=thr)
    assert len(lines) == 1 and lines[0].split()[0] == str(SMALL_BOX)
    # below the small-class rule still dropped
    assert S.yolo_lines([(SMALL_BOX, 0, 0, 2.5, 10)], 320, 240, thresholds=thr) == []
    assert S.yolo_lines([(SMALL_BOX, 0, 0, 3.5, 3.4)], 320, 240, thresholds=thr) == []          # 11.9 px^2 < 12
    with pytest.raises(ValueError):
        S.label_thresholds("v9", CLASSES_V3)


def test_gt_3d_adds_size_and_yaw():
    sc = {"objects": [{"name": "obj0_chair", "cls": "chair", "x": 0, "y": 0, "yaw": 0.5, "scale": 1.0, "variant": {}}],
          "small": [{"name": "sm0_can", "cls": "can", "x": 1, "y": 1, "yaw": -0.2, "height": 0.12}]}
    gt = GD.gt_3d(sc, [{"cls": "chair", "name": "obj0_chair"}, {"cls": "can", "name": "sm0_can"}, {"cls": "box", "name": "x"}])
    assert gt[0]["kind"] == "object" and gt[0]["yaw"] == 0.5 and len(gt[0]["size"]) == 3
    assert gt[1]["kind"] == "small" and gt[1]["size"] == pytest.approx([0.066, 0.066, 0.12])
    assert "size" not in gt[2]                                                             # unknown object: unchanged


def test_relabel_from_raw_boxes(tmp_path):
    out = tmp_path / "ds"
    (out / "meta" / "train").mkdir(parents=True)
    (out / "labels" / "train").mkdir(parents=True)
    (out / "classes.json").write_text(json.dumps(list(CLASSES_V3)))
    (out / "meta" / "train" / "s0000_v00.json").write_text(json.dumps({"image_wh": [320, 240], "label_rule": "v3",
                                                                      "boxes_raw": [list(b) for b in TINY]}))
    (out / "meta" / "train" / "s0001_v00.json").write_text(json.dumps({"gt": []}))           # old frame: no boxes_raw
    (out / "labels" / "train" / "s0000_v00.txt").write_text("")
    class A:
        pass
    a = A(); a.out = str(out); a.labels = "v3.1"
    GD.cmd_relabel(a)
    lab = (out / "labels" / "train" / "s0000_v00.txt").read_text().split("\n")
    assert lab[0].split()[0] == str(SMALL_BOX) and len([x for x in lab if x]) == 1
    assert json.loads((out / "meta" / "train" / "s0000_v00.json").read_text())["label_rule"] == "v3.1"
    a.labels = "v3"
    GD.cmd_relabel(a)                                                                        # and back: no label
    assert (out / "labels" / "train" / "s0000_v00.txt").read_text() == ""
