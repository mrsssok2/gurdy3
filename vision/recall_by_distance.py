#!/usr/bin/env python3
"""Recall of the small pickable classes by camera distance, from the per-frame meta ground truth (2026-11-27 label audit).

    docker compose --profile vision run --rm -T vision python3 vision/recall_by_distance.py --model models/detector/det_v3_s.onnx \
        --data datasets/synth_v3_body datasets/synth_v3_wrist --out docs/results/phase9b_recall_by_distance.md

Every small-class object whose centre projects into the image (meta `gt`: class, uv, p_cam) is one ground-truth instance,
LABELLED OR NOT (v3 drops boxes < 6 px / 80 px^2, so far objects can be visible without a label). Detected = a detection of the
same class (score >= --conf) whose box contains the projected centre (+2 px). Distance = |p_cam| (camera to object centre).
Per distance bin: instances, labelled, unlabelled + expected tiny (class size at that distance below the v3 rule = visible
but unlabelled), unlabelled other (occluded / cut), recall over all instances and over the labelled ones only.
Recall over ALL instances includes occluded objects, so it is a lower bound; the labelled-only column is the comparable one.
"""
import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gurdy_core"))
from gurdy_core.vision.classes import CLASSES_V3, SIZE_PRIOR_SMALL, SMALL_CLASSES  # noqa: E402

BINS = [(0.0, 1.0, "< 1 m"), (1.0, 2.0, "1-2 m"), (2.0, 3.0, "2-3 m"), (3.0, 99.0, "> 3 m")]
PAD = 2.0


def bin_of(d):
    return next(lbl for lo, hi, lbl in BINS if lo <= d < hi)


def expected_tiny(cls, z, f, min_px=6.0, min_area=80.0):
    w, d, h = SIZE_PRIOR_SMALL[cls]
    wp, hp = f * max(w, d) / z, f * h / z
    return wp < min_px or hp < min_px or wp * hp < min_area


def label_boxes(path, W, H):
    out = []
    if path.exists():
        for ln in path.read_text().split("\n"):
            if ln.strip():
                c, cx, cy, bw, bh = ln.split()[:5]
                cx, cy, bw, bh = float(cx) * W, float(cy) * H, float(bw) * W, float(bh) * H
                out.append((int(c), cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2))
    return out


def inside(u, v, b):
    return b[0] - PAD <= u <= b[2] + PAD and b[1] - PAD <= v <= b[3] + PAD


def evaluate_split(det, data: Path, split: str, limit: int = 0, only=None) -> dict:
    stats = defaultdict(lambda: defaultdict(int))
    metas = sorted((data / "meta" / split).glob("*.json"))
    if limit:
        metas = metas[:limit]
    for mf in metas:
        m = json.loads(mf.read_text())
        cam = m["camera"]
        W, H = cam["width"], cam["height"]
        f = (W / 2) / math.tan(cam["hfov"] / 2)
        img = data / "images" / split / (mf.stem + ".png")
        if not img.exists():
            continue
        gts = [g for g in m["gt"] if g["cls"] in (only or SMALL_CLASSES) and 0 <= g["uv"][0] < W and 0 <= g["uv"][1] < H and g["p_cam"][2] > 0.07]
        if not gts:
            continue
        dets = det(np.array(Image.open(img).convert("RGB")))
        labs = label_boxes(data / "labels" / split / (mf.stem + ".txt"), W, H)
        for g in gts:
            cid = CLASSES_V3.index(g["cls"])
            u, v = g["uv"]
            dist = float(np.linalg.norm(g["p_cam"]))
            b = bin_of(dist)
            s = stats[b]
            s["n"] += 1
            labelled = any(L[0] == cid and inside(u, v, L[1:]) for L in labs)
            hit = any(d.cls == cid and inside(u, v, d.box) for d in dets)
            s["hit"] += hit
            if labelled:
                s["labelled"] += 1
                s["hit_labelled"] += hit
            elif expected_tiny(g["cls"], g["p_cam"][2], f):
                s["tiny_unlabelled"] += 1
                s["hit_tiny"] += hit
            else:
                s["other_unlabelled"] += 1
    return stats


def table(name, st) -> list:
    rows = [f"**{name}**", "", "| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |",
            "|---|---|---|---|---|---|---|---|"]
    tot = defaultdict(int)
    for _, _, b in BINS:
        s = st.get(b, {})
        n = s.get("n", 0)
        for k, v in s.items():
            tot[k] += v
        pct = lambda a, c: f"{100 * a / c:.1f} %" if c else "-"       # noqa: E731
        rows.append(f"| {b} | {n} | {s.get('labelled', 0)} | {s.get('tiny_unlabelled', 0)} | {s.get('other_unlabelled', 0)} | "
                    f"{pct(s.get('hit', 0), n)} | {pct(s.get('hit_labelled', 0), s.get('labelled', 0))} | "
                    f"{pct(s.get('hit_tiny', 0), s.get('tiny_unlabelled', 0))} |")
    pct = lambda a, c: f"{100 * a / c:.1f} %" if c else "-"           # noqa: E731
    rows.append(f"| all | {tot['n']} | {tot['labelled']} | {tot['tiny_unlabelled']} | {tot['other_unlabelled']} | {pct(tot['hit'], tot['n'])} | "
                f"{pct(tot['hit_labelled'], tot['labelled'])} | {pct(tot['hit_tiny'], tot['tiny_unlabelled'])} |")
    return rows + [""]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--data", nargs="+", required=True)
    ap.add_argument("--split", default="test")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--providers", default="auto")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--out", default="")
    ap.add_argument("--only", nargs="*", default=None, help="count only these small classes (default: all)")
    a = ap.parse_args()
    from gurdy_core.vision.detector import OnnxDetector
    det = OnnxDetector(a.model, a.providers, conf=a.conf, names=CLASSES_V3)
    what = ", ".join(a.only) if a.only else "small classes"
    lines = [f"Recall by distance, {what}, model `{a.model}`, split {a.split}, conf {a.conf} (vision/recall_by_distance.py)", ""]
    for d in a.data:
        lines += table(f"{d} ({a.split})", evaluate_split(det, Path(d), a.split, a.limit, a.only))
    txt = "\n".join(lines)
    print(txt)
    if a.out:
        Path(a.out).write_text(txt + "\n")


if __name__ == "__main__":
    main()
