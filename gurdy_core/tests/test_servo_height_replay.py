"""Wrist-view height replayed on the REAL Gazebo frames saved by the fetch takes of 2026-11-28 (servo_frames/*.npz: depth,
K, T_base_opt). Before the fix, 4 of these frames read the 12 cm can as 3.0-6.3 cm tall -> 'implausible' -> re-look; the
object's own top above the fitted floor was 12.2-12.4 cm in every frame."""
import glob
import json
from pathlib import Path

import numpy as np
import pytest

from gurdy_core import visual_servo as VS

ROOT = Path(__file__).resolve().parents[2]


def frames():
    out = []
    for D in sorted(glob.glob(str(ROOT / "docs/results/data/demo_bring_1128_*"))):
        f = Path(D) / "raw/chat/attempts_align.jsonl"
        if not f.exists():
            continue
        for rec in (json.loads(x) for x in f.read_text().splitlines()):
            if "align" not in rec or not rec.get("frames"):
                continue
            views = [e for e in rec["log"] if "obs" in e]
            for i, e in enumerate(views):
                p = Path(D) / "raw/chat/servo_frames" / rec["frames"] / f"view_{i:02d}.npz"
                if p.exists():
                    out.append((f"{Path(D).name}/{rec['frames']}/{i}", e, p))
    if len(out) < 10:
        pytest.skip("saved fetch frames not in this checkout")
    return out


def test_no_saved_can_frame_is_implausible_any_more():
    was = now_ok = 0
    for name, logged, p in frames():
        z = np.load(p)
        o = VS.observe(z["depth"].astype(np.float32), tuple(z["K"]), z["T"], "can", None, VS.ServoParams())
        assert o.reason != "implausible", (name, o.height)
        if logged["obs"] == "implausible":
            was += 1
            now_ok += o.ok
    assert was >= 4 and now_ok == was


def test_full_top_views_measure_the_can_height():
    n = 0
    for name, logged, p in frames():
        z = np.load(p)
        o = VS.observe(z["depth"].astype(np.float32), tuple(z["K"]), z["T"], "can", None, VS.ServoParams())
        if o.ok and not o.partial:
            n += 1
            assert abs(o.height - 0.12) <= 0.01, (name, o.height)
    assert n >= 10
