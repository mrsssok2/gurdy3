#!/usr/bin/env python3
"""Target placements for the overnight fetch block (scripts/overnight_1127.sh): 5 small classes x 3 tries, 1-2.5 m from the
spawn point (0, 0) (detector range: det_v3_s recall > 90 % within 2 m, 75 % at 2-3 m on labelled objects).

    python3 scripts/overnight_1127_plan.py [--out docs/results/data/overnight_1127/plan.json] [--seed 1127]

Each placement passes the SAME free-space test as the fetch approach (brain/ros/fetch_robot.py `_approach_candidates`: the
grasp-zone pose and the stand-off pose 0.25 m further out, each with a 0.34 m ring, all free in the lab4 /map) for at least
3 of the 16 approach headings, the stand-off points are reachable through free space from the spawn point (BFS, robot radius
0.25 m), and it keeps >= 0.6 m from every other object of the world (the big ones and the other small ones) and the robot's
approach poses >= 0.45 m from them, and >= 0.8 m from every doorway (small objects are below the lidar plane). Try k of a class uses distance band k: [1.0, 1.5), [1.5, 2.0), [2.0, 2.5] m.
One model per class is moved there by gz set_pose after the stack is up: the command names it with its colour.
"""
import argparse
import json
import math
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
GT = json.loads((ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json").read_text())
TARGETS = [  # class, model, command (colour named: a second object of the class exists elsewhere for small_box / can / mini_cone)
    ("small_box", "f_red_cube", "pick up the red cube"),
    ("mini_toolbox", "f_yellow_mini_toolbox", "pick up the yellow mini toolbox"),
    ("can", "f_blue_can", "pick up the blue can"),
    ("mini_cone", "f_orange_cone", "pick up the orange mini cone"),
    ("mini_extinguisher", "f_red_mini_extinguisher", "pick up the red mini extinguisher"),
]
BANDS = [(1.0, 1.5), (1.5, 2.0), (2.0, 2.5)]
BACKUP_M, RING_R, ROBOT_R, DOOR_CLEAR = 0.25, 0.34, 0.25, 0.8


class Map:
    def __init__(self, yaml_path=ROOT / "maps/lab4_map.yaml"):
        m = yaml.safe_load(Path(yaml_path).read_text())
        self.res, self.ox, self.oy = m["resolution"], m["origin"][0], m["origin"][1]
        self.img = np.array(Image.open(Path(yaml_path).parent / m["image"]))
        self.h, self.w = self.img.shape

    def ij(self, x, y):
        return self.h - 1 - int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def cell(self, x, y):
        i, j = self.ij(x, y)
        if not (0 <= i < self.h and 0 <= j < self.w):
            return None
        v = self.img[i, j]
        return "occupied" if v < 100 else "free" if v > 250 else "unknown"

    def reachable_from(self, x0, y0, radius=ROBOT_R):
        """Boolean grid: cells whose centre is >= radius from any non-free cell and connected to (x0, y0)."""
        bad = self.img <= 250
        r = int(math.ceil(radius / self.res))
        infl = bad.copy()
        ys, xs = np.nonzero(bad)
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    infl[np.clip(ys + di, 0, self.h - 1), np.clip(xs + dj, 0, self.w - 1)] = True
        seen = np.zeros_like(bad)
        q = deque([self.ij(x0, y0)])
        if infl[q[0]]:
            return seen
        seen[q[0]] = True
        while q:
            i, j = q.popleft()
            for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= a < self.h and 0 <= b < self.w and not seen[a, b] and not infl[a, b]:
                    seen[a, b] = True
                    q.append((a, b))
        return seen


def zone_of(cls):
    import sys
    sys.path.insert(0, str(ROOT / "gurdy_core"))
    from gurdy_core.visual_servo import CLASS_INFO
    return CLASS_INFO[cls][3]


def approach_headings(mp, reach, x, y, zone, others):
    good = []
    for k in range(16):
        yaw = -math.pi + 2 * math.pi * k / 16
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = x - (c * zone[0] - s * zone[1]), y - (s * zone[0] + c * zone[1])
        sx, sy = bx + c * BACKUP_M, by + s * BACKUP_M
        ring = [(px + RING_R * math.cos(a), py + RING_R * math.sin(a)) for px, py in ((bx, by), (sx, sy))
                for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
        if any(mp.cell(px, py) != "free" for px, py in ring + [(bx, by), (sx, sy)]):
            continue
        if not reach[mp.ij(sx, sy)]:
            continue
        if any(math.hypot(px - ox, py - oy) < 0.45 for px, py in ((bx, by), (sx, sy)) for ox, oy in others):
            continue
        good.append(round(yaw, 3))
    return good


def plan(seed=1127, min_headings=3):
    mp = Map()
    reach = mp.reachable_from(0.0, 0.0)
    rng = np.random.default_rng(seed)
    used = []
    out = []
    for cls, model, cmd in TARGETS:
        zone = zone_of(cls)
        others = [(o["x"], o["y"]) for o in GT["objects"] if o.get("model") != model]
        for k, (lo, hi) in enumerate(BANDS, start=1):
            cands = []
            for x in np.arange(-3.0, 3.01, 0.1):
                for y in np.arange(-3.0, 3.01, 0.1):
                    d = math.hypot(x, y)
                    if not (lo <= d < hi or (k == len(BANDS) and d == hi)) or mp.cell(x, y) != "free":
                        continue
                    if any(math.hypot(x - ox, y - oy) < 0.6 for ox, oy in others + used):
                        continue
                    if any(math.hypot(x - dx, y - dy) < DOOR_CLEAR for dx, dy in GT["doors"].values()):
                        continue                         # below the lidar: an object in a doorway is in the route
                    hd = approach_headings(mp, reach, x, y, zone, others)
                    if len(hd) >= min_headings:
                        cands.append((round(float(x), 2), round(float(y), 2), hd))
            if not cands:
                raise SystemExit(f"no placement for {cls} in band {lo}-{hi} m")
            x, y, hd = cands[int(rng.integers(len(cands)))]
            used.append((x, y))
            room = next((n for n, r in (("room A", (-2, -2, 2, 2)), ("room B", (2, -2, 6, 2)), ("room C", (-2, 2, 2, 6)),
                                        ("room D", (2, 2, 6, 6))) if r[0] <= x <= r[2] and r[1] <= y <= r[3]), "?")
            out.append({"id": f"{cls}_{k}", "class": cls, "model": model, "command": cmd, "try": k, "x": x, "y": y,
                        "yaw": round(float(rng.uniform(-math.pi, math.pi)), 3), "dist_m": round(math.hypot(x, y), 2),
                        "band_m": [lo, hi], "room": room, "approach_headings": len(hd), "candidates_in_band": len(cands)})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs/results/data/overnight_1127/plan.json"))
    ap.add_argument("--seed", type=int, default=1127)
    a = ap.parse_args()
    p = plan(a.seed)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps({"seed": a.seed, "spawn": [0, 0, 0], "placements": p}, indent=1) + "\n")
    for t in p:
        print(f"{t['id']:22s} ({t['x']:5.2f}, {t['y']:5.2f}) {t['dist_m']:.2f} m {t['room']}  headings {t['approach_headings']:2d}  "
              f"(of {t['candidates_in_band']} candidates)  '{t['command']}'")


if __name__ == "__main__":
    main()
