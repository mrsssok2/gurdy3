#!/usr/bin/env python3
"""Replay fetch-run sighting logs (FETCH=true -> chat/sightings.jsonl) through SemanticMap with different track-confirmation
settings and grade every confirmed track against lab4_fetch ground truth (+ the try's teleported target from tries.jsonl).
Same sightings for every setting, so they compare like for like (2026-11-28: the red cube was detected 11x but never
confirmed). CPU only.

    python3 vision/replay_fetch_sightings.py RUN_DIR [RUN_DIR ...]      RUN_DIR = .../<run>/ with fetch/tries.jsonl
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gurdy_core"))
from gurdy_core.vision.semantic_map import SemanticMap  # noqa: E402

GT = json.loads((ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json").read_text())["objects"]
NODE = dict(min_hits=8, min_span=2.0, min_mean_score=0.6, max_scatter=0.5, dominance_ratio=4.0)   # detector_node defaults
SETTINGS = [
    ("old node default (8 hits, span 2 s, ttl 10 s)", dict(NODE, ttl=10.0)),
    ("ttl 30 s", dict(NODE, ttl=30.0)),
    ("ttl 60 s", dict(NODE, ttl=60.0)),
    ("ttl 120 s", dict(NODE, ttl=120.0)),
    ("5 hits, ttl 10 s", dict(NODE, ttl=10.0, min_hits=5)),
]
TRUNC_W = 0.25                       # detector_node truncated_weight


def runs(paths):
    for p in paths:
        p = Path(p)
        for tr in (p / "fetch").glob("tries.jsonl"):
            for line in tr.read_text().splitlines():
                t = json.loads(line)
                log = p / "fetch" / t["id"] / "chat" / "sightings.jsonl"
                if log.exists():
                    yield p.name, t, [json.loads(x) for x in log.read_text().splitlines() if x.strip()]


def ground_truth(t):
    """lab4_fetch objects; the try's target model moved to its teleported pose."""
    tx, ty = map(float, t["can_before"].split()[:2])
    moved = t["model"][2:] if t["model"].startswith("f_") else t["model"]
    objs = [dict(o) for o in GT]
    for o in objs:
        if o["name"] == moved:
            o["x"], o["y"], o["target"] = tx, ty, True
    return objs


def grade(m, objs, gate=0.5):
    out = {"confirmed": 0, "false": 0, "target": False, "target_label": None}
    for tr in m.confirmed():
        out["confirmed"] += 1
        x, y = float(tr.xyz[0]), float(tr.xyz[1])
        match = [o for o in objs if o["class"] == tr.cls and math.hypot(o["x"] - x, o["y"] - y) <= gate]
        if not match:
            out["false"] += 1
        elif any(o.get("target") for o in match):
            out["target"], out["target_label"] = True, f"{tr.color} {tr.cls}"
    return out


def main():
    paths = sys.argv[1:]
    data = list(runs(paths))
    print(f"{len(data)} fetch tries with a sighting log: " + ", ".join(f"{r}/{t['id']}" for r, t, _ in data))
    print("\n| setting | target confirmed | confirmed tracks (sum) | false tracks (sum) | per try: target / false |")
    print("|---|---|---|---|---|")
    for name, kw in SETTINGS:
        tot = {"confirmed": 0, "false": 0, "target": 0}
        per = []
        for run, t, log in data:
            m = SemanticMap(**kw)
            for s in log:
                m.update(s["cls"], s["p"], s["score"], s["t"], range_m=s["range"], color=s.get("color", ""),
                         weight_scale=TRUNC_W if s.get("truncated") else 1.0)
            g = grade(m, ground_truth(t))
            tot["confirmed"] += g["confirmed"]
            tot["false"] += g["false"]
            tot["target"] += g["target"]
            per.append(f"{run}/{t['id']} {'Y' if g['target'] else 'n'}/{g['false']}")
        print(f"| {name} | {tot['target']}/{len(data)} | {tot['confirmed']} | {tot['false']} | {'; '.join(per)} |")


if __name__ == "__main__":
    main()
