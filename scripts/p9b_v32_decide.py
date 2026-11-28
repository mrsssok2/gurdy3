#!/usr/bin/env python3
"""Switch the runtime detector to det_v32_s (coloured mini cones) ONLY if (1) it passes the agreed acceptance
(scripts/p9b_report.py --json) AND (2) its recall on COLOURED mini cones (synth_v32 test sets) recovers to >= RECOVER x the
recall det_v31_s had on WHITE mini cones (synth_v31 test sets) -- both from vision/recall_by_distance.py --only mini_cone --json.
Otherwise det_v31_s stays. The rollback line is kept in detector_node.py. Idempotent: an existing decision file is reused.

    python3 scripts/p9b_v32_decide.py --verdict V.json --cone-new N.json --cone-ref R.json --out decision.json [--apply]
"""
import argparse
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "ros2_ws/src/gurdy_perception/gurdy_perception/detector_node.py"
RECOVER = 0.90
NEW, OLD = "det_v32_s", "det_v31_s"
LINE = (f'DEFAULT_MODEL = "{NEW}.onnx"  # 12 classes (Phase 9b v3.2: coloured mini cones, overnight 2026-11-28); '
        f'rollback: model:={OLD}.onnx (or det_v3_s.onnx / det_v2_s.onnx)')


def decide(verdict: dict, cone_new: dict, cone_ref: dict, onnx_exists: bool) -> dict:
    ref, new = cone_ref.get("recall_all"), cone_new.get("recall_all")
    need = None if ref is None else round(RECOVER * ref, 4)
    acc = bool(verdict.get(NEW))
    cone_ok = new is not None and need is not None and new >= need
    why = []
    if not onnx_exists:
        why.append(f"models/detector/{NEW}.onnx missing")
    if not acc:
        why.append("acceptance FAIL (old classes or v3 small-class targets)")
    if not cone_ok:
        why.append(f"coloured-cone recall {new} < {need} (= {RECOVER} x white-cone recall of {OLD} {ref})")
    switch = onnx_exists and acc and cone_ok
    return {"switch": switch, "model": NEW if switch else OLD, "acceptance": acc, "cone_recall_coloured": new,
            "cone_recall_white_ref": ref, "cone_needed": need, "why_not": why}


def apply(model: str) -> bool:
    src = NODE.read_text()
    new = re.sub(r'^DEFAULT_MODEL = "[^"]+".*$', LINE, src, count=1, flags=re.M)
    if model != NEW or new == src:
        return False
    NODE.write_text(new)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verdict", required=True)
    ap.add_argument("--cone-new", required=True)
    ap.add_argument("--cone-ref", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--apply", action="store_true")
    a = ap.parse_args()
    out = Path(a.out)
    if out.exists():
        d = json.loads(out.read_text())
    else:
        load = lambda p: json.loads(Path(p).read_text()) if Path(p).exists() else {}     # noqa: E731
        d = decide(load(a.verdict), load(a.cone_new), load(a.cone_ref), (ROOT / f"models/detector/{NEW}.onnx").exists())
        out.write_text(json.dumps(d))
    changed = apply(d["model"]) if a.apply else False
    print(f"runtime detector: {d['model']} ({'SWITCHED' if d['switch'] else 'kept ' + OLD}"
          f"{'; detector_node.py updated' if changed else ''}){'; ' + '; '.join(d['why_not']) if d['why_not'] else ''}")


if __name__ == "__main__":
    main()
