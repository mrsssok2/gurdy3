#!/usr/bin/env python3
"""Dataset statistics + a grid of random labelled samples (complements vision/check_labels.py, which checks boxes vs GT).

    python3 vision/dataset_report.py --data datasets/synth_v1 --grid docs/results/phase4_dataset_grid.png \
        --json docs/results/phase4_dataset_stats.json [--must box mannequin]

Reports per split: images, empty-label images, instances per class; over all boxes: width/height/area distribution in
pixels, tiny boxes, boxes touching the image border, invalid lines (bad class, coords outside [0,1], zero size),
duplicate lines, images without a label file and label files without an image.
The grid is rows x cols random non-empty images; the first ones are chosen so that every --must class is shown.
"""
import argparse
import json
import random
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))
from gurdy_core.vision.classes import CLASSES  # noqa: E402

COL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6"]


def pct(x, qs=(5, 25, 50, 75, 95)):
    return {f"p{q}": round(float(np.percentile(x, q)), 1) for q in qs}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True); ap.add_argument("--grid"); ap.add_argument("--json")
    ap.add_argument("--rows", type=int, default=4); ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--must", nargs="*", default=["box", "mannequin"]); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--scale", type=int, default=2)
    a = ap.parse_args()
    data = Path(a.data)
    global CLASSES
    if (data / "classes.json").exists():               # Phase 9b v3 datasets carry their 12 class names
        CLASSES = tuple(json.loads((data / "classes.json").read_text()))
    rep, items, wh = {"splits": {}}, [], []                     # items: (split, name, [(c, x1, y1, x2, y2) px])
    invalid, dup, border, per_cls_h = [], 0, 0, {c: [] for c in CLASSES}
    for split in ("train", "val", "test"):
        imgs = sorted((data / "images" / split).glob("*.png"))
        labs = {p.stem for p in (data / "labels" / split).glob("*.txt")}
        inst, empty = np.zeros(len(CLASSES), int), 0
        W = H = None
        for ip in imgs:
            if W is None:
                W, H = Image.open(ip).size
            lp = data / "labels" / split / f"{ip.stem}.txt"
            lines = [ln for ln in lp.read_text().split("\n") if ln.strip()] if lp.exists() else []
            dup += len(lines) - len(set(lines))
            boxes = []
            for ln in lines:
                f = ln.split()
                try:
                    c = int(f[0]); cx, cy, bw, bh = map(float, f[1:])
                    ok = len(f) == 5 and 0 <= c < len(CLASSES) and bw > 0 and bh > 0 and \
                        cx - bw / 2 >= -1e-6 and cy - bh / 2 >= -1e-6 and cx + bw / 2 <= 1 + 1e-6 and cy + bh / 2 <= 1 + 1e-6
                except ValueError:
                    ok = False
                if not ok:
                    invalid.append(f"{split}/{ip.stem}: {ln}"); continue
                x1, y1, x2, y2 = (cx - bw / 2) * W, (cy - bh / 2) * H, (cx + bw / 2) * W, (cy + bh / 2) * H
                boxes.append((c, x1, y1, x2, y2)); inst[c] += 1
                wh.append((x2 - x1, y2 - y1)); per_cls_h[CLASSES[c]].append(y2 - y1)
                border += x1 < 1 or y1 < 1 or x2 > W - 1 or y2 > H - 1
            empty += not boxes
            items.append((split, ip.stem, boxes))
        rep["splits"][split] = {"images": len(imgs), "empty_label_images": int(empty),
                                "images_without_label_file": len([p for p in imgs if p.stem not in labs]),
                                "label_files_without_image": len(labs - {p.stem for p in imgs}),
                                "instances": dict(zip(CLASSES, map(int, inst)))}
    wh = np.array(wh)
    area = wh[:, 0] * wh[:, 1]
    rep.update({"image_wh": [W, H], "boxes": len(wh), "invalid_lines": invalid[:20], "n_invalid_lines": len(invalid),
                "duplicate_lines": int(dup), "boxes_touching_border": int(border),
                "width_px": pct(wh[:, 0]), "height_px": pct(wh[:, 1]), "sqrt_area_px": pct(np.sqrt(area)),
                "min_side_px": {"min": round(float(wh.min()), 1), "lt_4px": int((wh.min(1) < 4).sum()),
                                "lt_8px": int((wh.min(1) < 8).sum()), "lt_16px": int((wh.min(1) < 16).sum())},
                "area_fraction_gt_50pct": int((area > 0.5 * W * H).sum()),
                "boxes_per_image": pct([len(b) for _, _, b in items]) | {"max": max(len(b) for _, _, b in items)},
                "median_height_px_per_class": {c: round(float(np.median(v)), 1) for c, v in per_cls_h.items() if v}})
    print(json.dumps(rep, indent=1))
    if a.json:
        Path(a.json).write_text(json.dumps(rep, indent=1) + "\n")
    if a.grid:
        rng = random.Random(a.seed)
        pool = [it for it in items if it[2]]
        rng.shuffle(pool)
        chosen = []
        for cname in a.must:                                    # one image per required class first
            ci = CLASSES.index(cname)
            chosen.append(next(it for it in pool if it not in chosen and any(b[0] == ci for b in it[2])))
        chosen += [it for it in pool if it not in chosen][:a.rows * a.cols - len(chosen)]
        rng.shuffle(chosen)
        s, pad = a.scale, 14
        grid = Image.new("RGB", (a.cols * W * s, a.rows * (H * s + pad)), "white")
        for k, (split, name, boxes) in enumerate(chosen):
            im = Image.open(data / "images" / split / f"{name}.png").convert("RGB").resize((W * s, H * s), Image.NEAREST)
            d = ImageDraw.Draw(im)
            for c, x1, y1, x2, y2 in boxes:
                d.rectangle([x1 * s, y1 * s, x2 * s, y2 * s], outline=COL[c], width=2)
                ty = y1 * s - 11 if y1 * s >= 11 else y2 * s
                d.rectangle([x1 * s, ty, x1 * s + 6 * len(CLASSES[c]) + 4, ty + 11], fill=COL[c])
                d.text((x1 * s + 2, ty), CLASSES[c], fill="white")
            x0, y0 = (k % a.cols) * W * s, (k // a.cols) * (H * s + pad)
            grid.paste(im, (x0, y0 + pad))
            ImageDraw.Draw(grid).text((x0 + 3, y0 + 1), f"{split}/{name}", fill="black")
        Path(a.grid).parent.mkdir(parents=True, exist_ok=True)
        grid.save(a.grid)
        print("grid:", a.grid, "classes shown:", sorted({CLASSES[b[0]] for it in chosen for b in it[2]}))


if __name__ == "__main__":
    main()
