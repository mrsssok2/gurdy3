#!/usr/bin/env python3
"""Phase 8c: offline accuracy of box + depth -> 3D (gurdy_core.vision.object3d) on synthetic depth images.

    python3 scripts/object3d_synth_eval.py [--n 300] [--seed 0]     # needs numpy only; no ROS / Gazebo

Camera = robot.yaml depth_camera (320x240, hfov 1.047, pitched down 0.17 rad, 0.13 m above ground when standing).
One object per image (box / barrel / extinguisher / toolbox shapes from SIZE_PRIOR), random robot pose 0.8-3.5 m away,
object roughly in view, random object yaw. Boxes: 'tight' = visible-pixel box (perfect detector), 'yolo' = tight box with
every edge moved by N(0, 4 px) (a loose / jittery detector). Depth noise sigma = k z^2 (k = 0 and 0.003).
xy method 'ray' = nearest surface + radius along the viewing ray; 'fit' = class footprint fitted (rectangle / circle).
These are idealised numbers (perfect geometry, flat floor, no rendering artefacts): an upper bound for the Gazebo run.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))

from gurdy_core.vision.object3d import estimate  # noqa: E402
from gurdy_core.vision.synth_depth import render  # noqa: E402
from gurdy_core.vision.transforms import T_base_optical, T_map_base, intrinsics_from_fov  # noqa: E402

CAM = {"pos": [0.15, 0.0, 0.02], "pitch_down": 0.17}
W, H, HFOV, BODY_Z = 320, 240, 1.047, 0.11
SHAPES = {   # class: solid template (centre z = half height), radius prior = half the smaller horizontal extent
    "box": (dict(kind="box", size=(0.40, 0.40, 0.35)), 0.20),
    "barrel": (dict(kind="cylinder", radius=0.275, height=0.90), 0.275),
    "fire_extinguisher": (dict(kind="cylinder", radius=0.08, height=0.50), 0.08),
    "toolbox": (dict(kind="box", size=(0.45, 0.20, 0.20)), 0.10),
}


def solid_at(cls, x, y, yaw):
    t, _ = SHAPES[cls]
    s = dict(t)
    h = s["size"][2] if s["kind"] == "box" else s["height"]
    s["centre"], s["yaw"] = (x, y, h / 2), yaw
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    K = intrinsics_from_fov(W, H, HFOV)
    rows = {}
    for i in range(a.n):
        cls = list(SHAPES)[i % len(SHAPES)]
        r, bearing, yaw_r = rng.uniform(0.8, 3.5), rng.uniform(-0.35, 0.35), rng.uniform(-math.pi, math.pi)
        rx, ry = rng.uniform(-1, 1, 2)
        ryaw = rng.uniform(-math.pi, math.pi)
        T = T_map_base(rx, ry, ryaw, z=BODY_Z) @ T_base_optical(CAM)
        cam = T[:3, 3]
        ox, oy = cam[0] + r * math.cos(ryaw + bearing), cam[1] + r * math.sin(ryaw + bearing)
        s = solid_at(cls, ox, oy, yaw_r)
        h = s["centre"][2] * 2
        for noise in (0.0, 0.003):
            d, _, boxes = render(K, W, H, T, [s], noise=noise, rng=rng)
            if boxes[0] is None:
                continue
            for kind in ("tight", "yolo"):
                b = boxes[0]
                if kind == "yolo":
                    b = tuple(float(np.clip(v + rng.normal(0, 4), 0, lim)) for v, lim in zip(b, (W, H, W, H)))
                    if b[2] <= b[0] + 2 or b[3] <= b[1] + 2:
                        continue
                t_, _ = SHAPES[cls]
                shape = {"kind": "box", "size": t_["size"]} if t_["kind"] == "box" else {"kind": "cylinder", "radius": t_["radius"]}
                for method in ("ray", "fit"):
                    o = estimate(d, b, K, T, radius_prior=SHAPES[cls][1], shape=shape if method == "fit" else None)
                    rr = rows.setdefault((cls, noise, kind, method), {"n": 0, "none": 0, "xy": [], "z": [], "top": [], "top_vis": 0})
                    rr["n"] += 1
                    if o is None:
                        rr["none"] += 1
                        continue
                    rr["xy"].append(math.hypot(o.centre[0] - ox, o.centre[1] - oy))
                    if o.top_known:
                        rr["top_vis"] += 1
                        rr["top"].append(abs(o.top_z - h))
                        rr["z"].append(abs(o.centre[2] - h / 2))
    q = lambda v, p: f"{np.percentile(v, p) * 100:.1f}" if v else "-"     # noqa: E731
    print("| class | depth noise k | box | xy method | sightings | no estimate | xy err median / p90 (cm) | top visible | "
          "top err median / p90 (cm) | centre z err median (cm) |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for (cls, noise, kind, method), rr in sorted(rows.items()):
        print(f"| {cls} | {noise} | {kind} | {method} | {rr['n']} | {rr['none']} | {q(rr['xy'], 50)} / {q(rr['xy'], 90)} | "
              f"{rr['top_vis']} | {q(rr['top'], 50)} / {q(rr['top'], 90)} | {q(rr['z'], 50)} |")


if __name__ == "__main__":
    main()
