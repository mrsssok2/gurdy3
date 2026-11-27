#!/usr/bin/env python3
"""Phase 8c accuracy check in Gazebo semantic_lab: stop plan + offline scoring (pure Python + numpy, no ROS).

The ROS side is scripts/object3d_gazebo_check.py (teleports the robot to each stop, stands still, writes stops.json);
detector_node (object3d:=true, detector_log) writes the sightings. Both run inside scripts/object3d_gazebo_check.sh.

    python3 vision/accuracy3d.py plan [--out stops_plan.json]                 # print / write the stop plan
    python3 vision/accuracy3d.py score RUN_DIR [--md report.md]               # RUN_DIR: sightings.jsonl + stops.json (+ semantic.json)
    python3 vision/accuracy3d.py synth [--seeds 3]                            # same stops, SYNTHETIC depth (upper bound, no Gazebo)

Stops: for every GT object, camera at range r in {0.6, 1.0, 1.5, 2.5} m (horizontal, camera lens -> object centre) and the
object at bearing b in {-20, 0, +20} deg of the camera view. The camera point stays fixed per (object, range); the robot
turns about it. The approach direction is the one with the most clearance (robot footprint circle 0.34 m + margin from
walls, flat_lab furniture and the other objects; camera -> object line of sight not through another object).

Scoring (frames: GT is the world, floor z 0; the detector works in odom = world shifted down to base_link height, so
heights are compared ABOVE THE FLOOR: est z - fitted floor z vs GT z; xy compared directly, odom = world when the robot
spawned at the origin with yaw 0, checked per stop by the runner):
  * per sighting, per class x range x bearing: xy error of object3d (fit / ray) and of the Phase 4 pixel estimate of the
    SAME box (`p_pixel` in the log); z (centre above floor; only sightings with the top known, the others are reported as
    z_topcut: their centre z is only a lower bound), top (only when top_known), height
  * per stop: Track3D fusion of that stop's sightings
  * honesty: sightings that claim a known top but are > 3 cm off (top cut off / box cut short)
  * floor: fitted floor z per sighting vs the -0.11 m prior (convention check)
  * whole run: the node's semantic map (semantic.json) vs GT with object3d.compare_3d
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gurdy_core"))
sys.path.insert(0, str(ROOT / "scripts"))
from gurdy_core.vision.object3d import Object3D, Track3D, compare_3d  # noqa: E402

GT3D = ROOT / "docs/results/data/phase8c/semantic_lab_gt3d.json"
RANGES = (0.6, 1.0, 1.5, 2.5)
BEARINGS_DEG = (-20.0, 0.0, 20.0)
CAM_X = 0.15                     # camera lens ahead of base_link (robot.yaml sensors.depth_camera.pos)
FOOT_R = 0.34                    # robot footprint circumradius (nav2 footprint corners 0.273 x 0.202)
MARGIN = 0.05
TOP_HONEST = 0.03                # m: a 'known' top further off than this counts as a false top claim


# ------------------------------------------------------------------ world geometry
def load_gt(path=GT3D) -> list:
    return json.loads(Path(path).read_text())["objects"]


def obstacles(gt: list) -> list:
    """[(name, kind, data)]: rect = (cx, cy, sx, sy, yaw); circle = (cx, cy, r). Walls + flat_lab furniture + GT objects."""
    import gen_world_flat_lab as fl
    out = [(n, "rect", (c[0], c[1], s[0], s[1], 0.0)) for n, c, s in fl.walls()]
    names = {g["name"] for g in gt}
    for name, kind, xy, size, _ in fl.OBJECTS:
        if name in names:
            continue                                   # the coloured boxes are GT objects too
        out.append((name, "rect", (xy[0], xy[1], size[0], size[1], 0.0)) if kind == "box" else (name, "circle", (xy[0], xy[1], size[0])))
    for g in gt:
        sx, sy = g["size"][:2]
        if g["class"] in ("barrel", "fire_extinguisher"):
            out.append((g["name"], "circle", (g["x"], g["y"], 0.5 * max(sx, sy))))
        else:
            out.append((g["name"], "rect", (g["x"], g["y"], sx, sy, g.get("yaw", 0.0))))
    return out


def _dist_point(ob, x, y) -> float:
    """Distance from (x, y) to the obstacle (0 inside)."""
    _, kind, d = ob
    if kind == "circle":
        return max(math.hypot(x - d[0], y - d[1]) - d[2], 0.0)
    cx, cy, sx, sy, yaw = d
    c, s = math.cos(-yaw), math.sin(-yaw)
    u, v = c * (x - cx) - s * (y - cy), s * (x - cx) + c * (y - cy)
    return math.hypot(max(abs(u) - sx / 2, 0.0), max(abs(v) - sy / 2, 0.0))


def _segment_clear(ob, a, b, step=0.02) -> float:
    n = max(int(math.hypot(b[0] - a[0], b[1] - a[1]) / step), 1)
    return min(_dist_point(ob, a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1))


def base_pose(obj_xy, rng: float, alpha: float, bearing: float):
    """Camera at obj + rng (cos a, sin a); robot yaw so the object is at `bearing` (left +) of the view; -> (x, y, yaw)."""
    cam = (obj_xy[0] + rng * math.cos(alpha), obj_xy[1] + rng * math.sin(alpha))
    yaw = alpha + math.pi - bearing
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return cam[0] - CAM_X * math.cos(yaw), cam[1] - CAM_X * math.sin(yaw), yaw


def plan_stops(gt=None, ranges=RANGES, bearings_deg=BEARINGS_DEG, n_dirs: int = 24) -> dict:
    """-> {"stops": [...], "skipped": [...]} in GT order, range, bearing."""
    gt = gt or load_gt()
    obs = obstacles(gt)
    stops, skipped = [], []
    for g in gt:
        others = [o for o in obs if o[0] != g["name"]]
        for r in ranges:
            best = None
            for k in range(n_dirs):
                a = 2 * math.pi * k / n_dirs
                cam = (g["x"] + r * math.cos(a), g["y"] + r * math.sin(a))
                los = min((_segment_clear(o, cam, (g["x"], g["y"])) for o in others), default=9.0)   # nothing in between
                if los <= 0.02:
                    continue
                foot = min(_dist_point(o, *base_pose((g["x"], g["y"]), r, a, math.radians(b))[:2])   # incl. the target itself
                           for o in obs for b in bearings_deg)
                foot_clear = foot - FOOT_R
                if foot_clear < MARGIN:
                    continue
                score = min(foot_clear, 0.5) + 0.2 * min(los, 0.5)
                if best is None or score > best[0]:
                    best = (score, a, foot_clear, los)
            if best is None:
                skipped.append({"name": g["name"], "class": g["class"], "range": r, "reason": "no free approach direction"})
                continue
            for b in bearings_deg:
                x, y, yaw = base_pose((g["x"], g["y"]), r, best[1], math.radians(b))
                stops.append({"id": f"{g['name']}_r{r:.1f}_b{b:+.0f}", "name": g["name"], "class": g["class"], "range": r,
                              "bearing_deg": b, "x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4),
                              "approach_deg": round(math.degrees(best[1]), 1), "foot_clearance": round(best[2], 3),
                              "los_clearance": round(best[3], 3)})
    return {"stops": stops, "skipped": skipped}


# ------------------------------------------------------------------ scoring
def _stats(v):
    v = sorted(x for x in v if x is not None and math.isfinite(x))
    if not v:
        return None
    return {"n": len(v), "median": float(np.median(v)), "p90": float(np.percentile(v, 90)), "max": float(v[-1])}


def range_bin(r: float) -> str:
    return min(RANGES, key=lambda x: abs(x - r)).__format__(".1f")


def assign(sightings: list, stops: list, gt: list) -> list:
    """Attach each sighting to the stop whose [t0, t1] (sim time) contains it and to its GT object (same class as the
    stop's object and nearest in xy; other-class sightings are kept as `other`). -> list of rows."""
    gtn = {g["name"]: g for g in gt}
    rows = []
    for s in sightings:
        st = next((p for p in stops if p.get("t0") is not None and p["t0"] <= s["t"] <= p["t1"]), None)
        if st is None or not st.get("valid", True):
            continue
        g = gtn[st["name"]]
        xy = (s["o3d"]["c"] if s.get("o3d") else s["p"])[:2]
        same = [h for h in gt if h["class"] == s["cls"]]
        nearest = min(same, key=lambda h: math.hypot(h["x"] - xy[0], h["y"] - xy[1])) if same else None
        rows.append({"stop": st, "gt": g, "s": s, "target": nearest is not None and nearest["name"] == g["name"]})
    return rows


def sighting_errors(row) -> dict:
    s, g = row["s"], row["gt"]
    o = s.get("o3d")
    out = {"cls": g["class"], "range": range_bin(row["stop"]["range"]), "bearing": row["stop"]["bearing_deg"],
           "pixel_xy": math.hypot(s["p_pixel"][0] - g["x"], s["p_pixel"][1] - g["y"]) if s.get("p_pixel") else None,
           "truncated": s.get("truncated", False)}
    if o is None:
        out["fallback"] = True
        return out
    fz = o["floor_z"]
    top_gt = g["z"] + g["size"][2] / 2
    out.update({"fallback": False, "method": o["method"], "xy": math.hypot(o["c"][0] - g["x"], o["c"][1] - g["y"]),
                "z": abs((o["c"][2] - fz) - g["z"]), "floor_z": fz, "top_known": o["top_known"]})
    if o["top_known"]:
        out["top"] = abs((o["top_z"] - fz) - top_gt)
        out["height"] = abs(o["height"] - g["size"][2])
        out["false_top"] = out["top"] > TOP_HONEST
    return out


def fuse_stop(rows) -> dict | None:
    t = Track3D()
    for r in rows:
        o = r["s"].get("o3d")
        if o is None:
            continue
        sig = np.array(o["sigma"], float)
        fz = o["floor_z"]
        c = np.array([o["c"][0], o["c"][1], o["c"][2] - fz])          # z above the floor
        obj = Object3D(c, o["top_z"] - fz, o["bottom_z"] - fz, o["width"], o["height"], o["range"], np.diag(sig ** 2),
                       o["n"], 1.0, o["on_floor"], {"top": not o["top_known"], "left": not o["width_known"]})
        t.update(obj)
    if not t.hits:
        return None
    sm = t.summary()
    g = rows[0]["gt"]
    out = {"xy": math.hypot(sm["x"] - g["x"], sm["y"] - g["y"]), "z": abs(sm["z"] - g["z"]), "hits": sm["hits"]}
    if sm["top_z"] is not None:
        out["top"] = abs(sm["top_z"] - (g["z"] + g["size"][2] / 2))
    return out


def score(run_dir, gt=None) -> dict:
    run = Path(run_dir)
    gt = gt or load_gt()
    stops = json.loads((run / "stops.json").read_text())["stops"]
    sightings = [json.loads(line) for line in (run / "sightings.jsonl").read_text().splitlines() if line.strip()]
    rows = assign(sightings, stops, gt)
    tgt = [r for r in rows if r["target"]]
    errs = [sighting_errors(r) for r in tgt]
    groups = {}
    for e in errs:
        groups.setdefault((e["cls"], e["range"]), []).append(e)
    table = []
    for (cls, rb), es in sorted(groups.items()):
        o3 = [e for e in es if not e["fallback"]]
        table.append({"class": cls, "range": rb, "n": len(es), "fallback": len(es) - len(o3),
                      "method": sorted({e["method"] for e in o3}),
                      "xy_o3d": _stats([e["xy"] for e in o3]), "xy_pixel": _stats([e["pixel_xy"] for e in es]),
                      "z": _stats([e["z"] for e in o3 if e["top_known"]]),        # centre z is a lower bound when the top is cut
                      "z_topcut": _stats([e["z"] for e in o3 if not e["top_known"]]), "top": _stats([e.get("top") for e in o3]),
                      "height": _stats([e.get("height") for e in o3]),
                      "top_known": sum(e["top_known"] for e in o3), "false_top": sum(e.get("false_top", False) for e in o3)})
    per_stop = []
    by_stop = {}
    for r in tgt:
        by_stop.setdefault(r["stop"]["id"], []).append(r)
    for st in stops:
        rs = by_stop.get(st["id"], [])
        per_stop.append({"id": st["id"], "valid": st.get("valid", True), "sightings": len(rs),
                         "other_class": sum(1 for r in rows if r["stop"]["id"] == st["id"] and not r["target"]),
                         "fused": fuse_stop(rs) if rs else None})
    floors = [e["floor_z"] for e in errs if e.get("floor_z") is not None]
    out = {"n_sightings": len(sightings), "n_assigned": len(rows), "n_target": len(tgt), "table": table, "stops": per_stop,
           "floor_z": _stats(floors) if floors else None,
           "floor_z_mean": float(np.mean(floors)) if floors else None,
           "false_top_total": sum(e.get("false_top", False) for e in errs),
           "invalid_stops": [s["id"] for s in stops if not s.get("valid", True)]}
    sem = run / "semantic.json"
    if sem.exists():
        found = json.loads(sem.read_text())["objects"]
        fz = out["floor_z_mean"] if out["floor_z_mean"] is not None else -0.11
        f2 = [{**f, "z": f["z"] - fz, "top_z": None if f.get("top_z") is None else f["top_z"] - fz} for f in found]
        out["semantic_map"] = compare_3d(f2, gt)
    return out


def _fmt(st, k="median"):
    return "-" if st is None else f"{100 * st[k]:.1f}"


def report_md(res: dict) -> str:
    L = ["| class | range m | n | fallback | method | xy o3d med / p90 cm | xy pixel med / p90 cm | z med cm (top known) | z med cm (top cut) | top med cm (known) | false top |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in res["table"]:
        L.append(f"| {r['class']} | {r['range']} | {r['n']} | {r['fallback']} | {','.join(r['method']) or '-'} | "
                 f"{_fmt(r['xy_o3d'])} / {_fmt(r['xy_o3d'], 'p90')} | {_fmt(r['xy_pixel'])} / {_fmt(r['xy_pixel'], 'p90')} | "
                 f"{_fmt(r['z'])} | {_fmt(r['z_topcut'])} | {_fmt(r['top'])} ({r['top_known']}) | {r['false_top']} |")
    fl = res["floor_z"]
    L += ["", f"sightings {res['n_sightings']}, at a stop {res['n_assigned']}, of the stop's object {res['n_target']}; "
              f"false top claims {res['false_top_total']}; invalid stops {len(res['invalid_stops'])}",
          "" if fl is None else f"fitted floor z: median {fl['median']:.4f} m (prior -0.11), max deviation {fl['max']:.4f}"]
    sm = res.get("semantic_map")
    if sm:
        L += ["", f"semantic map vs GT: recall {sm['recall']:.2f}, false {sm['false']}, xy median "
                  f"{_fmt(sm['xy_err'])} cm, z median {_fmt(sm['z_err'])} cm, top median {_fmt(sm['top_err'])} cm"]
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ synthetic dry run of the same stops
def _solid(g):
    if g["class"] in ("barrel", "fire_extinguisher"):
        return dict(kind="cylinder", centre=(g["x"], g["y"], g["z"]), radius=0.5 * max(g["size"][:2]), height=g["size"][2])
    return dict(kind="box", centre=(g["x"], g["y"], g["z"]), size=tuple(g["size"]), yaw=g.get("yaw", 0.0))


SYNTH_CLASSES = ("box", "toolbox", "barrel", "fire_extinguisher")    # primitive solids = the real shape; cone / chair / mannequin not


def synth(seeds: int = 3, noise: float = 0.003, gt=None) -> list:
    """Every planned stop of the SYNTH_CLASSES objects rendered with gurdy_core.vision.synth_depth (the object alone, flat
    floor, depth noise sigma = noise * z^2, tight visible-pixel box), floor at odom z -0.11 as in Gazebo. Methods: object3d
    with the class shape fit (`fit`), object3d without (`ray`), and the Phase 4 pixel method on the same box (`pixel`).
    -> rows per object: xy error stats per method, and top error (known tops) of `fit`."""
    from gurdy_core.vision import locate3d as L
    from gurdy_core.vision.classes import radius_prior
    from gurdy_core.vision.geometry import backproject, box_depth, surface_to_centre
    from gurdy_core.vision.synth_depth import render
    from gurdy_core.vision.transforms import T_base_optical, T_map_base, apply_T, intrinsics_from_fov, make_T
    gt = gt or load_gt()
    K = intrinsics_from_fov(320, 240, 1.047)
    cam = {"pos": [CAM_X, 0.0, 0.02], "pitch_down": 0.17}
    res = {}
    for st in plan_stops(gt)["stops"]:
        g = next(h for h in gt if h["name"] == st["name"])
        if g["class"] not in SYNTH_CLASSES:
            continue
        Tw = T_map_base(st["x"], st["y"], st["yaw"], z=0.11) @ T_base_optical(cam)
        T = make_T(t=(0, 0, -0.11)) @ Tw
        for k in range(seeds):
            d, _, boxes = render(K, 320, 240, Tw, [_solid(g)], noise=noise, rng=np.random.default_rng(k))
            if boxes[0] is None:
                continue
            r = res.setdefault(g["name"], {"class": g["class"], "fit": [], "ray": [], "pixel": [], "top": [], "n": 0})
            r["n"] += 1
            plane = L.FloorTracker().update(d, K, T, 0.0)
            for m in ("fit", "ray"):
                s3 = L.locate(d, boxes[0], K, T, g["class"], plane, use_shape=m == "fit")
                if s3 is None:
                    continue
                r[m].append(math.hypot(s3.centre[0] - g["x"], s3.centre[1] - g["y"]))
                if m == "fit" and s3.top_known:
                    r["top"].append(abs(s3.top_z - s3.floor_z - (g["z"] + g["size"][2] / 2)))
            z, _ = box_depth(d, boxes[0])
            if z is not None:
                u, v = 0.5 * (boxes[0][0] + boxes[0][2]), 0.5 * (boxes[0][1] + boxes[0][3])
                p = apply_T(T, surface_to_centre(backproject(u, v, z, K), radius_prior(g["class"]))[None, :])[0]
                r["pixel"].append(math.hypot(p[0] - g["x"], p[1] - g["y"]))
    return [{"name": n, **{k: (_stats(v) if isinstance(v, list) else v) for k, v in r.items()}} for n, r in res.items()]


def synth_md(rows) -> str:
    L = ["| object | class | sightings | xy fit med / p90 cm | xy ray med / p90 cm | xy pixel (Phase 4) med / p90 cm | top fit med cm (n) |",
         "|---|---|---|---|---|---|---|"]
    for r in rows:
        f = lambda k: f"{_fmt(r[k])} / {_fmt(r[k], 'p90')}"     # noqa: E731
        L.append(f"| {r['name']} | {r['class']} | {r['n']} | {f('fit')} | {f('ray')} | {f('pixel')} | "
                 f"{_fmt(r['top'])} ({0 if r['top'] is None else r['top']['n']}) |")
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan")
    p.add_argument("--out")
    s = sub.add_parser("score")
    s.add_argument("run_dir")
    s.add_argument("--md")
    y = sub.add_parser("synth")
    y.add_argument("--seeds", type=int, default=3)
    a = ap.parse_args(argv)
    if a.cmd == "synth":
        print(synth_md(synth(a.seeds)))
        return 0
    if a.cmd == "plan":
        plan = plan_stops()
        txt = json.dumps(plan, indent=1)
        if a.out:
            Path(a.out).write_text(txt + "\n")
        print(f"{len(plan['stops'])} stops, {len(plan['skipped'])} (object, range) skipped", file=sys.stderr)
        for k in plan["skipped"]:
            print(f"  skipped {k['name']} {k['range']} m: {k['reason']}", file=sys.stderr)
        if not a.out:
            print(txt)
        return 0
    res = score(a.run_dir)
    Path(a.run_dir, "accuracy3d.json").write_text(json.dumps(res, indent=1, default=str) + "\n")
    md = report_md(res)
    if a.md:
        Path(a.md).write_text(md)
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
