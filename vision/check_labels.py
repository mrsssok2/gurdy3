#!/usr/bin/env python3
"""Label sanity check for a rendered dataset: draws the YOLO boxes on sample frames and compares every box with the
independent ground truth in meta/ (object centre projected with the planned camera pose; clean depth at the box centre).

    python3 vision/check_labels.py --data datasets/synth_v1 --out docs/results/phase4_trial [--samples 5]

A box "matches" if a ground-truth centre of the same class lies inside it. A wrong camera pose (set_pose not applied) or
frames from before the pose change would show up as unmatched boxes. GT centres without a box are objects that are
occluded, too small or cut by the image border (reported, not an error by itself).
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))
from gurdy_core.vision.classes import CLASSES  # noqa: E402

COL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6",
       "#d62a2a", "#7a5c00", "#00a3c4", "#c400a3", "#4d4d4d"]   # 7-11: Phase 9b small classes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--samples", type=int, default=5); ap.add_argument("--scale", type=int, default=3)
    a = ap.parse_args()
    data, out = Path(a.data), Path(a.out)
    global CLASSES
    if (data / "classes.json").exists():               # Phase 9b v3 datasets carry their 12 class names
        CLASSES = tuple(json.loads((data / "classes.json").read_text()))
    out.mkdir(parents=True, exist_ok=True)
    metas = sorted(data.glob("meta/*/*.json"))
    n_box = n_match = n_gt = n_gt_box = n_empty = 0
    derr, per_scene, cls_count = [], {}, np.zeros(len(CLASSES), int)
    frames = []
    for mp in metas:
        m = json.loads(mp.read_text())
        split, name = mp.parent.name, mp.stem
        W, H = m["camera"]["width"], m["camera"]["height"]
        depth = np.array(Image.open(data / "depth" / split / f"{name}.png")).astype(float) / 1000.0
        boxes = []
        for ln in (data / "labels" / split / f"{name}.txt").read_text().split("\n"):
            if ln.strip():
                c, cx, cy, bw, bh = ln.split()
                cx, cy, bw, bh = float(cx) * W, float(cy) * H, float(bw) * W, float(bh) * H
                boxes.append((int(c), cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2))
        n_empty += not boxes
        used = set()
        for c, x1, y1, x2, y2 in boxes:
            n_box += 1; cls_count[c] += 1
            hit = [i for i, g in enumerate(m["gt"]) if g["cls"] == CLASSES[c] and x1 <= g["uv"][0] <= x2 and y1 <= g["uv"][1] <= y2]
            if hit:
                n_match += 1; used.update(hit)
                g = min((m["gt"][i] for i in hit), key=lambda g: g["p_cam"][2])
                d = depth[int(np.clip((y1 + y2) / 2, 0, H - 1)), int(np.clip((x1 + x2) / 2, 0, W - 1))]
                if d > 0:
                    derr.append(d - g["p_cam"][2])
            ps = per_scene.setdefault(m["scene"], [0, 0]); ps[0] += 1; ps[1] += bool(hit)
        n_gt += len(m["gt"]); n_gt_box += len(used)
        frames.append((len(boxes), m["scene"], split, name, boxes, m["gt"]))
    # samples: the frame with the most boxes from each of the first `samples` scenes
    best = {}
    for f in frames:
        if f[1] not in best or f[0] > best[f[1]][0]:
            best[f[1]] = f
    for _, scene, split, name, boxes, gt in list(best.values())[:a.samples]:
        s = a.scale
        im = Image.open(data / "images" / split / f"{name}.png").convert("RGB")
        im = im.resize((im.width * s, im.height * s), Image.NEAREST)
        d = ImageDraw.Draw(im)
        for c, x1, y1, x2, y2 in boxes:
            d.rectangle([x1 * s, y1 * s, x2 * s, y2 * s], outline=COL[c], width=2)
            d.rectangle([x1 * s, y1 * s - 12, x1 * s + 7 * len(CLASSES[c]) + 4, y1 * s], fill=COL[c])
            d.text((x1 * s + 2, y1 * s - 12), CLASSES[c], fill="white")
        for g in gt:                                             # independent GT centre (white cross)
            u, v = g["uv"][0] * s, g["uv"][1] * s
            d.line([u - 5, v, u + 5, v], fill="white", width=1); d.line([u, v - 5, u, v + 5], fill="white", width=1)
        im.save(out / f"{name}_boxes.png")
    derr = np.array(derr)
    rep = {"frames": len(metas), "frames_without_boxes": int(n_empty), "boxes": n_box, "boxes_matching_gt_centre": n_match,
           "gt_centres_in_view": n_gt, "gt_centres_with_box": n_gt_box,
           "depth_minus_gt_z_m": {"median": float(np.median(derr)), "p90_abs": float(np.percentile(np.abs(derr), 90))} if len(derr) else None,
           "boxes_per_class": dict(zip(CLASSES, map(int, cls_count))),
           "per_scene_boxes_matched": {str(k): v for k, v in sorted(per_scene.items())}}
    (out / "label_check.json").write_text(json.dumps(rep, indent=1))
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
