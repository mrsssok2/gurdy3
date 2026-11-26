#!/usr/bin/env python3
"""Phase 8c: 3D ground truth for the semantic_lab world (centre x, y, z, size, yaw) from the SAME object lists the world
is generated from (vision/make_test_world.py PLACED + flat_lab boxes). Does not touch the world file.

    python3 vision/make_gt3d.py           # writes docs/results/data/phase8c/semantic_lab_gt3d.json
    python3 vision/make_gt3d.py --check   # exit 1 if the committed file is out of date

Size = axis-aligned bounding box of the model's parts in the OBJECT frame (sx along the object's x, before yaw); centre =
bounding-box centre rotated by yaw + model origin (objects stand on the floor, z = height / 2). The pose is the one written
into the SDF; static models do not move, so a Gazebo pose query should give the same (to be confirmed in the Gazebo run).
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from vision import objects as O  # noqa: E402
from vision.make_test_world import FLAT_LAB_BOXES, PLACED  # noqa: E402

OUT = ROOT / "docs/results/data/phase8c/semantic_lab_gt3d.json"


def bounds(part_list):
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for p in part_list:
        px, py, pz = p["pos"]
        if p["kind"] == "box":
            s = p["size"]
        elif p["kind"] == "cylinder":
            s = (2 * p["size"][0], 2 * p["size"][0], p["size"][1])
        elif p["kind"] == "sphere":
            s = (2 * p["size"][0],) * 3
        else:                                          # mesh_cone (r_base, r_top, h), origin at its base
            s = (2 * p["size"][0], 2 * p["size"][0], p["size"][2])
            pz += p["size"][2] / 2
        for i, (c, e) in enumerate(zip((px, py, pz), s)):
            lo[i], hi[i] = min(lo[i], c - e / 2), max(hi[i], c + e / 2)
    return lo, hi


def build():
    objs = []
    for name, cls, x, y, yaw, var in PLACED:
        lo, hi = bounds(O.parts(cls, 1.0, var))
        cx, cy, cz = ((a + b) / 2 for a, b in zip(lo, hi))
        c, s = math.cos(yaw), math.sin(yaw)
        objs.append({"name": name, "class": cls, "x": round(x + c * cx - s * cy, 4), "y": round(y + s * cx + c * cy, 4),
                     "z": round(cz, 4), "size": [round(b - a, 4) for a, b in zip(lo, hi)], "yaw": yaw,
                     "bottom_z": round(lo[2], 4)})
    from gen_world_flat_lab import OBJECTS                                   # flat_lab's own boxes (sizes there)
    sizes = {n: sz for n, kind, _xy, sz, _rgb in OBJECTS if kind == "box"}
    for name, x, y, color in FLAT_LAB_BOXES:
        sx, sy, sz = sizes[name]
        objs.append({"name": name, "class": "box", "x": x, "y": y, "z": sz / 2, "size": [sx, sy, sz], "yaw": 0.0,
                     "bottom_z": 0.0, "color": color})
    return json.dumps({"world": "semantic_lab", "frame": "world (= map / odom when the robot spawns at the origin)",
                       "objects": objs}, indent=1) + "\n"


if __name__ == "__main__":
    text = build()
    if "--check" in sys.argv:
        sys.exit(0 if OUT.exists() and OUT.read_text() == text else 1)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text)
    print(f"wrote {OUT}")
