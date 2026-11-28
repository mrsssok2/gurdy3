#!/usr/bin/env python3
"""Colour-naming accuracy per class on labelled synthetic test images (CPU, host venv): the detector's crop (middle 50 % of
the box, detector_node.py) named by the old rule (colors.color_name_v1) and the current one (colors.color_name), against the
object's painted variant from the scene file (worlds/scene_NNNN.json "small" / object "variant").

    python3 vision/color_eval.py datasets/synth_v3_body datasets/synth_v3_wrist [--split test] [--out docs/results/x.md]

Expected names: the variant's English name; silver -> gray or white, lime -> green, dark_green -> green. Boxes smaller than
6 px on a side are skipped (the node's colour on them is noise either way). Large classes have no colour variant here.
"""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gurdy_core"))
from gurdy_core.vision.classes import CLASSES_V3  # noqa: E402
from gurdy_core.vision.colors import color_name, color_name_v1  # noqa: E402

SMALL = ("small_box", "mini_toolbox", "can", "mini_cone", "mini_extinguisher")
EXPECT = {"silver": {"gray", "white"}, "lime": {"green"}, "dark_green": {"green"}}


def load_png(p):
    try:
        from PIL import Image
        return np.asarray(Image.open(p).convert("RGB"))
    except ImportError:
        import cv2
        return cv2.imread(str(p))[:, :, ::-1]


def crop_of(img, box):
    x1, y1, x2, y2 = box                        # same crop as detector_node.py (middle 50 % in both directions)
    return img[int(y1 + 0.25 * (y2 - y1)):int(y2 - 0.25 * (y2 - y1)) + 1, int(x1 + 0.25 * (x2 - x1)):int(x2 - 0.25 * (x2 - x1)) + 1]


def instances(D: Path, split: str):
    """(class, expected colour, image, box) for every labelled small object whose GT entry can be matched."""
    scenes = {}
    for meta in sorted((D / "meta" / split).glob("*.json")):
        m = json.loads(meta.read_text())
        sc = scenes.get(m["scene"])
        if sc is None:
            sc = scenes[m["scene"]] = json.loads((D / "worlds" / f"scene_{m['scene']:04d}.json").read_text())
        colour = {o["name"]: o.get("colour") for o in sc.get("small", [])}
        lab = D / "labels" / split / (meta.stem + ".txt")
        if not lab.exists():
            continue
        img = None
        for ln in lab.read_text().splitlines():
            if not ln.strip():
                continue
            c, cx, cy, w, h = ln.split()[:5]
            cls = CLASSES_V3[int(c)]
            if cls not in SMALL:
                continue
            if img is None:
                img = load_png(D / "images" / split / (meta.stem + ".png"))
            H, W = img.shape[:2]
            box = ((float(cx) - float(w) / 2) * W, (float(cy) - float(h) / 2) * H, (float(cx) + float(w) / 2) * W,
                   (float(cy) + float(h) / 2) * H)
            if box[2] - box[0] < 6 or box[3] - box[1] < 6:
                continue
            cand = [g for g in m["gt"] if g["cls"] == cls and g.get("uv") and colour.get(g["name"])]
            inside = [g for g in cand if box[0] <= g["uv"][0] <= box[2] and box[1] <= g["uv"][1] <= box[3]]
            pool = inside or cand
            if not pool:
                continue
            ctr = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
            g = min(pool, key=lambda q: (q["uv"][0] - ctr[0]) ** 2 + (q["uv"][1] - ctr[1]) ** 2)
            if len(pool) > 1 and len({colour[q["name"]] for q in pool}) > 1 and not inside:
                continue                                      # ambiguous match with different colours: skip
            yield cls, colour[g["name"]], img, box


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("datasets", nargs="+")
    ap.add_argument("--split", default="test")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    L = ["Colour naming per class (vision/color_eval.py): detector crop (middle 50 % of the labelled box) vs the painted "
         f"variant from the scene file, split `{a.split}`. old = median RGB rule, new = saturated-pixel majority.", ""]
    for d in a.datasets:
        D = Path(d)
        n = defaultdict(int)
        ok = {"old": defaultdict(int), "new": defaultdict(int)}
        wrong = {"old": defaultdict(lambda: defaultdict(int)), "new": defaultdict(lambda: defaultdict(int))}
        for cls, col, img, box in instances(D, a.split):
            exp = EXPECT.get(col, {col})
            c = crop_of(img, box)
            n[cls] += 1
            for k, fn in (("old", color_name_v1), ("new", color_name)):
                got = fn(c)
                if got in exp:
                    ok[k][cls] += 1
                else:
                    wrong[k][cls][got] += 1
        L += [f"**{D} ({a.split})**", "", "| class | instances | old rule | new rule | new rule: most common wrong names |",
              "|---|---|---|---|---|"]
        for cls in SMALL:
            if n[cls]:
                w = ", ".join(f"{k} {v}" for k, v in sorted(wrong["new"][cls].items(), key=lambda x: -x[1])[:3]) or "-"
                L.append(f"| {cls} | {n[cls]} | {100 * ok['old'][cls] / n[cls]:.1f} % | {100 * ok['new'][cls] / n[cls]:.1f} % | {w} |")
        tot = sum(n.values())
        if tot:
            L.append(f"| **all** | {tot} | {100 * sum(ok['old'].values()) / tot:.1f} % | {100 * sum(ok['new'].values()) / tot:.1f} % | |")
        L.append("")
    txt = "\n".join(L) + "\n"
    print(txt)
    if a.out:
        Path(a.out).write_text(txt)


if __name__ == "__main__":
    main()
