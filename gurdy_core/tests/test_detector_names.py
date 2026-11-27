"""Runtime detector class list follows the ONNX output width (det_v3_s default, det_v2_s rollback)."""
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
