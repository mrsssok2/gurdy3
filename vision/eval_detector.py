#!/usr/bin/env python3
"""Evaluate an exported detector on a split: per-class AP50 / precision / recall, depth-based 3D error, FPS.

    docker compose run --rm vision python3 vision/eval_detector.py --model models/detector/det_v1.onnx \
        --data datasets/synth_v1 --split test --providers cuda cpu --out docs/results/phase4_detector.md

3D error: for every true-positive detection the object centre is estimated exactly as the ROS node does
(box_depth -> backproject -> surface_to_centre) and compared with the ground-truth centre stored in meta/*.json.
FPS = pure inference (preprocess + ONNX + decode) over the split, per provider (CUDA needs onnxruntime-gpu).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))
from gurdy_core.vision.classes import CLASSES, CLASSES_V3, SIZE_PRIOR_SMALL, SMALL_CLASSES, radius_prior  # noqa: E402


def _radius(name):
    return 0.5 * min(SIZE_PRIOR_SMALL[name][:2]) if name in SIZE_PRIOR_SMALL else radius_prior(name)
from gurdy_core.vision.detector import OnnxDetector, iou_xyxy  # noqa: E402
from gurdy_core.vision.geometry import K_from_camera_info, backproject, box_depth, surface_to_centre  # noqa: E402
from gurdy_core.vision.metrics import evaluate  # noqa: E402


def yolo_gt(path: Path, w: int, h: int):
    out = []
    for ln in path.read_text().split("\n"):
        if ln.strip():
            c, cx, cy, bw, bh = ln.split()
            cx, cy, bw, bh = float(cx) * w, float(cy) * h, float(bw) * w, float(bh) * h
            out.append((int(c), (cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--split", default="test")
    ap.add_argument("--providers", nargs="+", default=["cuda", "cpu"])
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--out", default="")
    ap.add_argument("--classes", default="v1", choices=["v1", "v3"],
                    help="v1: the 7 deployed classes; v3: 12 classes (Phase 9b model, old ids 0-6 unchanged)")
    a = ap.parse_args()
    NAMES = CLASSES_V3 if a.classes == "v3" else CLASSES
    root = Path(a.data)
    files = sorted((root / "images" / a.split).glob("*.png"))[: a.limit or None]
    imgs = [np.array(Image.open(f).convert("RGB")) for f in files]
    rows, report = [], []
    for prov in a.providers:
        det = OnnxDetector(a.model, "CUDAExecutionProvider" if prov == "cuda" else "CPUExecutionProvider", conf=0.05, names=NAMES)
        if prov == "cuda" and "CUDA" not in det.provider:
            report.append(f"- {prov}: CUDA provider not available (got {det.provider}), skipped")
            continue
        det(imgs[0])
        t0 = time.perf_counter()
        preds = [det(im) for im in imgs]
        fps = len(imgs) / (time.perf_counter() - t0)
        rows.append((prov, det.provider, fps))
        if prov == a.providers[0] or "res" not in locals():
            per_image, errs = [], []
            for f, im, pr in zip(files, imgs, preds):
                h, w = im.shape[:2]
                gts = yolo_gt(root / "labels" / a.split / f"{f.stem}.txt", w, h)
                per_image.append({"pred": [(d.cls, d.score, d.box) for d in pr], "gt": gts})
                meta = json.loads((root / "meta" / a.split / f"{f.stem}.json").read_text())
                depth = np.array(Image.open(root / "depth" / a.split / f"{f.stem}.png")).astype(np.float32) / 1000.0
                K = K_from_camera_info([w / 2 / np.tan(meta["camera"]["hfov"] / 2), 0, w / 2, 0,
                                        w / 2 / np.tan(meta["camera"]["hfov"] / 2), h / 2, 0, 0, 1])
                for d in pr:
                    if d.score < a.conf:
                        continue
                    for g in meta["gt"]:
                        if g["cls"] != d.name:
                            continue
                        u, v = (d.box[0] + d.box[2]) / 2, (d.box[1] + d.box[3]) / 2
                        if np.hypot(u - g["uv"][0], v - g["uv"][1]) > 0.35 * max(d.box[2] - d.box[0], d.box[3] - d.box[1]):
                            continue
                        z, _ = box_depth(depth, d.box)
                        if z is None:
                            continue
                        est = surface_to_centre(backproject(u, v, z, K), _radius(d.name))
                        errs.append((float(np.linalg.norm(est - np.array(g["p_cam"]))), float(np.linalg.norm(g["p_cam"]))))
            res = evaluate(per_image, len(NAMES), 0.5, a.conf)
    lines = [f"# Phase 4 detector evaluation — {Path(a.model).name} on {a.data}/{a.split} ({len(files)} images)", "",
             f"| class | GT | AP50 | precision@{a.conf} | recall@{a.conf} |", "|---|---|---|---|---|"]
    for c, n in enumerate(NAMES):
        r = res[c]
        lines.append(f"| {n} | {r['n_gt']} | {r['ap']:.3f} | {r['precision']:.3f} | {r['recall']:.3f} |")
    lines += [f"| **mAP50** | | **{res['mAP']:.3f}** | | |", ""]
    if a.classes == "v3":       # regression guard: the old classes alone, and small-class false positives
        old = [res[c]["ap"] for c in range(len(CLASSES)) if res[c]["n_gt"] > 0]
        fp_small = sum(1 for im in per_image for cls, sc_, _ in im["pred"] if sc_ >= a.conf and NAMES[cls] in SMALL_CLASSES
                       and not any(g[0] == cls for g in im["gt"]))
        lines += [f"mAP50 over the {len(old)} old classes with GT: {np.mean(old):.3f}" if old else "no old-class GT in this split",
                  f"small-class detections (score >= {a.conf}) in images without that class: {fp_small} in {len(per_image)} images", ""]
    if errs:
        e = np.array(errs)
        lines += [f"3D centre error (camera frame, {len(e)} true positives): median {np.median(e[:, 0]):.3f} m, "
                  f"mean {e[:, 0].mean():.3f} m, p90 {np.percentile(e[:, 0], 90):.3f} m; median range {np.median(e[:, 1]):.2f} m", ""]
    lines += ["| provider | active | FPS (inference only) |", "|---|---|---|"] + [f"| {p} | {act} | {fps:.1f} |" for p, act, fps in rows] + [""] + report
    text = "\n".join(lines)
    print(text)
    if a.out:
        Path(a.out).write_text(text + "\n")


if __name__ == "__main__":
    main()
