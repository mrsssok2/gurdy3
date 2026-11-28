"""GPU eval of VLM candidates for describe_view (docs/results/vlm_plan.md section 5). Run by scripts/vlm_eval.sh under the GPU
lock; NOT run yet. Frames: RGB rendered in Gazebo with per-image ground truth (vision/generate_dataset.py datasets); GT = a perfect
detector for the facts side.

Per model:
  vlm_alone   on "what do you see?": JSON validity, object precision (VLM detector-class objects that are in the frame; a
              sibling class counts), hallucination rate (detector-class objects NOT in the frame), recall of the GT objects,
              outside-the-detector nouns per frame (info), colour accuracy (VLM colour vs GT colour, objects with a GT colour)
  tool        the full tool with GT facts on see / colour / count questions: statuses (confirmed / not sure / ...), invariant
              violations (check_reply; must be 0), FALSE STATEMENTS vs GT (a colour or count stated without a hedge that the GT
              contradicts; must be 0)
  corrupted   the same with corrupted facts (one fact's colour changed, one fact dropped): how often the tool then states the
              wrong colour (= the VLM agreed with the wrong fact: both witnesses wrong) vs says "not sure"
  latency p50 / p95 per VLM call, VRAM peak (nvidia-smi polled every 0.5 s while the model runs)
  python3 -m brain.vlm.eval --model moondream --dataset DIR [--dataset DIR] --n 60 --out OUT   (--mock perfect|hallucinate: no GPU)
"""
from __future__ import annotations

import argparse
import json
import random
import re
import shutil
import statistics
import subprocess
import threading
import time
from collections import Counter
from pathlib import Path

from . import client as C
from . import ground as G
from .frames import frame_from_render
from .tool import DescribeView
from .vocab import SIBLINGS, colour, say, to_class


def dataset_frames(dirs, n: int, seed: int = 0) -> list:
    items = []
    for d in dirs:
        d = Path(d)
        for meta in sorted((d / "meta").glob("*/*.json")):
            png = d / "images" / meta.parent.name / (meta.stem + ".png")
            scene = d / "worlds" / f"scene_{int(meta.stem[1:5]):04d}.json"
            if png.exists():
                items.append((png, meta, scene if scene.exists() else None))
    random.Random(seed).shuffle(items)
    out = []
    for png, meta, scene in items:
        f = frame_from_render(png, meta, scene)
        if f.detections:
            out.append(f)
        if len(out) >= n:
            break
    return out


class _Backend:
    def __init__(self, t):
        self.t = t

    def now(self):
        return self.t

    def known_places(self):
        return []


def questions_for(frame, rng) -> list:
    qs = ["what do you see?"]
    c = Counter(d["cls"] for d in frame.detections)
    uniq = [d for d in frame.detections if c[d["cls"]] == 1 and d.get("color")]
    if uniq:
        d = rng.choice(uniq)
        qs.append(f"what colour is the {say(d['cls'])}?")
    cls = rng.choice(sorted(c))
    qs.append(f"how many {say(cls).split()[-1] + ('es' if say(cls).endswith('x') else 's')} can you see?")
    return qs


def _stated_colour(reply: str):
    m = re.match(r"^(?:The|My detector says the) [a-z ]+? is ([a-z]+)[ .(]", reply)
    return m.group(1) if m and "not sure" not in reply else None


def _false_statement(q, reply, gt_facts) -> bool:
    """A plain (unhedged) colour / count statement that the ground truth contradicts."""
    if q["kind"] == "colour":
        col = _stated_colour(reply)
        tgt = [d for d in gt_facts if d["cls"] == q["cls"]]
        return col is not None and bool(tgt) and colour(tgt[0].get("color")) not in ("", col)
    if q["kind"] == "count" and "not sure" not in reply:
        m = re.search(r"\b(?:see|sees) (no|one|two|three|four|five|six|\d+) ", reply)
        words = {"no": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6}
        if m:
            n = words.get(m.group(1), int(m.group(1)) if m.group(1).isdigit() else -1)
            return n != sum(1 for d in gt_facts if d["cls"] == q["cls"])
    return False


def corrupt(facts, rng):
    f = [dict(x) for x in facts]
    if not f:
        return f, None
    i = rng.randrange(len(f))
    if f[i].get("color") and rng.random() < 0.5:
        f[i]["color"] = rng.choice([c for c in ("red", "blue", "green", "yellow", "white", "black") if c != colour(f[i]["color"])])
        return f, "colour"
    del f[i]
    return f, "drop"


def run_eval(vlm, frames, seed: int = 0, log=None) -> dict:
    from .questions import classify
    rng = random.Random(seed)
    alone = {"calls": 0, "json_ok": 0, "vlm_objects": 0, "in_frame": 0, "hallucinated": 0, "outside": 0, "gt": 0, "recalled": 0,
             "colour_n": 0, "colour_ok": 0}
    tool = Counter()
    corr = Counter()
    lat = []
    for f in frames:
        gt = f.detections
        t0 = time.perf_counter()
        try:
            raw = vlm(f.image, "what do you see?")
        except Exception as e:                            # noqa: BLE001
            raw = None
            if log:
                log({"frame": f.source, "error": repr(e)})
        lat.append(time.perf_counter() - t0)
        alone["calls"] += 1
        p = C.parse(raw) if raw is not None else None
        if p is not None:
            alone["json_ok"] += 1
            used = set()
            for o in p["objects"]:
                cls, col = to_class(o["name"])
                col = colour(o.get("color")) or col
                if cls is None:
                    alone["outside"] += 1
                    continue
                alone["vlm_objects"] += 1
                i = next((i for i, d in enumerate(gt) if i not in used and d["cls"] in (cls, SIBLINGS.get(cls))), None)
                if i is None:
                    alone["hallucinated"] += 1
                    continue
                used.add(i)
                alone["in_frame"] += 1
                if gt[i].get("color") and col:
                    alone["colour_n"] += 1
                    alone["colour_ok"] += colour(gt[i]["color"]) == col
            alone["gt"] += len(gt)
            alone["recalled"] += len(used)
        if log:
            log({"frame": f.source, "q": "what do you see?", "raw": raw, "gt": [(d["cls"], d.get("color")) for d in gt]})
        for text in questions_for(f, rng):
            q = classify(text)
            for mode in ("gt", "corrupted"):
                facts, how = (gt, None) if mode == "gt" else corrupt(gt, rng)
                fr = type(f)(f.t, facts, f.cam, f.image, f.source)
                t = DescribeView(_Backend(f.t), vlm, frame_source=lambda fr=fr: fr)
                t1 = time.perf_counter()
                r = t(text, q)
                lat.append(time.perf_counter() - t1)
                bad = G.check_reply(r["reply"], r["facts"])
                false = _false_statement(q, r["reply"], gt)
                tgt = tool if mode == "gt" else corr
                tgt[f"{q['kind']}:{r['status']}"] += 1
                tgt["n"] += 1
                tgt["invariant_violations"] += bool(bad)
                tgt["false_statements"] += false
                if mode == "corrupted":
                    corr[f"corrupted_{how}"] += 1
                    corr[f"false_after_{how}"] += false
                if log:
                    log({"frame": f.source, "q": text, "mode": mode, "corrupted": how, "reply": r["reply"], "status": r["status"],
                         "false": false, "violations": bad, "raw": r.get("vlm_raw")})
    rate = lambda a, b: round(a / b, 4) if b else None      # noqa: E731
    lat.sort()
    return {"frames": len(frames),
            "vlm_alone": {**alone, "json_valid_rate": rate(alone["json_ok"], alone["calls"]), "precision": rate(alone["in_frame"], alone["vlm_objects"]),
                          "hallucination_rate": rate(alone["hallucinated"], alone["vlm_objects"]), "recall": rate(alone["recalled"], alone["gt"]),
                          "colour_accuracy": rate(alone["colour_ok"], alone["colour_n"]), "outside_nouns_per_frame": rate(alone["outside"], alone["calls"])},
            "tool_gt_facts": dict(tool), "tool_corrupted_facts": dict(corr),
            "latency_s": {"p50": round(statistics.median(lat), 3) if lat else None, "p95": round(lat[int(0.95 * (len(lat) - 1))], 3) if lat else None}}


class VramPoll:
    def __init__(self, dt=0.5):
        self.exe = shutil.which("nvidia-smi") or ("/usr/lib/wsl/lib/nvidia-smi" if Path("/usr/lib/wsl/lib/nvidia-smi").exists() else None)
        self.peak, self.dt, self.stop = None, dt, threading.Event()

    def _run(self):
        while not self.stop.wait(self.dt):
            try:
                out = subprocess.run([self.exe, "--query-gpu=memory.used", "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=5).stdout
                v = int(out.strip().splitlines()[0])
                self.peak = v if self.peak is None else max(self.peak, v)
            except Exception:                              # noqa: BLE001
                pass

    def __enter__(self):
        if self.exe:
            threading.Thread(target=self._run, daemon=True).start()
        return self

    def __exit__(self, *a):
        self.stop.set()


class MockVLM:
    """--mock perfect: names every GT object with its GT colour; --mock hallucinate: + a sofa and a barrel, colours wrong."""

    def __init__(self, kind, frames):
        self.kind, self.by_img = kind, {f.image: f for f in frames}

    def __call__(self, image, question):
        f = self.by_img.get(image)
        objs = [{"name": say(d["cls"]), "color": d.get("color") or None} for d in (f.detections if f else [])]
        if self.kind == "hallucinate":
            objs = [{"name": o["name"], "color": "purple"} for o in objs] + [{"name": "sofa"}, {"name": "barrel", "color": "red"}]
        return json.dumps({"answer": "", "objects": objs, "path_clear": None, "unsure": False})


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", action="append", default=[])
    ap.add_argument("--dataset", action="append", default=[])
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--no-schema", action="store_true", help="plain JSON mode instead of the schema `format`")
    ap.add_argument("--mock", choices=("perfect", "hallucinate"), help="no model (tests / dry run)")
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    frames = dataset_frames(a.dataset, a.n, a.seed)
    if not frames:
        print("no frames found in", a.dataset)
        return 1
    results = {}
    for m in (a.model or ["moondream"]) if not a.mock else [f"mock-{a.mock}"]:
        vlm = MockVLM(a.mock, frames) if a.mock else C.OllamaVLM(m, use_schema=not a.no_schema)
        logf = open(out / f"calls_{re.sub(r'[^A-Za-z0-9_.-]', '_', m)}.jsonl", "w")
        with VramPoll() as vp:
            r = run_eval(vlm, frames, a.seed, log=lambda d: logf.write(json.dumps(d, default=str) + "\n"))
        logf.close()
        r["vram_peak_mib"] = vp.peak
        results[m] = r
        print(m, json.dumps({k: r[k] for k in ("vlm_alone", "latency_s", "vram_peak_mib")}, default=str), flush=True)
    (out / "summary.json").write_text(json.dumps({"datasets": a.dataset, "n": len(frames), "seed": a.seed, "models": results}, indent=1) + "\n")
    L = ["# VLM eval", "", f"{len(frames)} frames from {', '.join(a.dataset)} (seed {a.seed})", "",
         "| model | JSON ok | precision | hallucination | recall | colour acc. | latency p50 / p95 s | VRAM peak MiB | tool false statements (GT / corrupted) | invariant violations |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for m, r in results.items():
        v = r["vlm_alone"]
        L.append(f"| {m} | {v['json_valid_rate']} | {v['precision']} | {v['hallucination_rate']} | {v['recall']} | {v['colour_accuracy']} | "
                 f"{r['latency_s']['p50']} / {r['latency_s']['p95']} | {r['vram_peak_mib']} | {r['tool_gt_facts'].get('false_statements', 0)} / "
                 f"{r['tool_corrupted_facts'].get('false_statements', 0)} | {r['tool_gt_facts'].get('invariant_violations', 0) + r['tool_corrupted_facts'].get('invariant_violations', 0)} |")
    (out / "report.md").write_text("\n".join(L) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
