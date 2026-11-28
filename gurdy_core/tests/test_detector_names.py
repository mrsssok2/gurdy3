"""Runtime detector class list follows the ONNX output width (det_v31_s default, det_v3_s / det_v2_s rollback)."""
import pytest

from gurdy_core.vision.classes import CLASSES, CLASSES_V3, COLORS_ALL, SIZE_PRIOR_ALL, names_for_outputs, radius_prior


def test_names_from_output_width():
    assert names_for_outputs(4 + 7) == CLASSES
    assert names_for_outputs(4 + 12) == CLASSES_V3
    with pytest.raises(ValueError):
        names_for_outputs(4 + 9)


def test_every_v3_class_has_priors():
    for c in CLASSES_V3:
        assert len(SIZE_PRIOR_ALL[c]) == 3 and len(COLORS_ALL[c]) == 3
        assert 0 < radius_prior(c) < 0.3
    assert radius_prior("can") == pytest.approx(0.033)


def test_runtime_default_is_det_v31_s_and_v3_stays_selectable():
    """2026-11-28: default det_v31_s (12 classes); det_v3_s kept for rollback (`detector_model:=det_v3_s.onnx`)."""
    import json
    import re
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    src = (root / "ros2_ws/src/gurdy_perception/gurdy_perception/detector_node.py").read_text()
    m = re.search(r'^DEFAULT_MODEL = "(det_v3[12]_s)\.onnx"', src, re.M)   # det_v32_s only if overnight_1128 switched it
    assert m, "default must be det_v31_s or det_v32_s"
    if m.group(1) == "det_v32_s":
        assert "rollback: model:=det_v31_s.onnx" in src
    for name in sorted({m.group(1), "det_v31_s", "det_v3_s"}):  # model cards are committed (the .onnx files are not)
        card = json.loads((root / f"models/detector/{name}.json").read_text())
        assert list(card["classes"].values()) == list(CLASSES_V3) and card["input_hw"] == [256, 320]
    for name in ("det_v31_s", "det_v3_s"):
        onnx = root / f"models/detector/{name}.onnx"
        if onnx.exists():                                      # local check only: both must load as 12-class detectors
            ort = pytest.importorskip("onnxruntime")
            out = ort.InferenceSession(str(onnx), providers=["CPUExecutionProvider"]).get_outputs()[0].shape
            assert names_for_outputs(out[1]) == CLASSES_V3
