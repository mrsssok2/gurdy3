"""Aggregate repeated held-out runs (brain/runs/final_eval/<ts>, one per seed) into one markdown table block + JSON.

  GURDY_FINAL_SUITE=final_suite_lab4 python3 -m brain.eval.heldout_report RUN1 RUN2 RUN3 [--baseline OLD_RUN --baseline-suite final_suite] [--json out.json]

Every record is re-scored with the CURRENT scorer (`final_checks.evaluate`, active suite) so runs are comparable. Infrastructure failures
(setup_failed / sim_hang / infra_failed / error) are counted separately and shown; success rate is reported over ALL missions of the suite
(infra failure = not a success) and over the valid missions only.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
from pathlib import Path

from .final_checks import evaluate
from .suites import suite_module


def load_run(run: Path, suite) -> dict:
    out = {}
    for m in suite.SUITE:
        f = run / "missions" / f"{m['id']}.json"
        if not f.exists():
            out[m["id"]] = None
            continue
        rec = json.load(open(f))
        try:
            rec["verdict"] = evaluate(rec)
        except Exception as e:                                     # noqa: BLE001
            rec["verdict"] = {"success": False, "infra_failure": True, "falls": 0, "wrong_claims": 0, "wrong_claim_details": [repr(e)], "checks": [], "timed_out": False, "grounding_violations": 0, "arrival_errors": []}
        out[m["id"]] = rec
    return out


def run_stats(recs: dict, suite) -> dict:
    n = len(suite.SUITE)
    vs = {k: (r or {}).get("verdict") for k, r in recs.items()}
    succ = sum(1 for v in vs.values() if v and v["success"])
    infra = [k for k, v in vs.items() if v is None or v.get("infra_failure")]
    valid = n - len(infra)
    lat = []
    for r in recs.values():
        for s in (r or {}).get("stops", []):
            if s.get("moving_before") and s.get("latency_s") is not None:
                lat.append(s["latency_s"])
    cats: dict[str, list] = {}
    for m in suite.SUITE:
        v = vs[m["id"]]
        c = cats.setdefault(m["category"], [0, 0])
        c[1] += 1
        c[0] += 1 if v and v["success"] else 0
    return {"n": n, "success": succ, "rate": succ / n, "valid": valid, "rate_valid": succ / valid if valid else 0.0, "infra": infra,
            "timeouts": [k for k, v in vs.items() if v and v.get("timed_out")],
            "falls": sum(int((v or {}).get("falls", 0)) for v in vs.values()),
            "wrong_claims": sum(int((v or {}).get("wrong_claims", 0)) for v in vs.values()),
            "wrong_claim_missions": [k for k, v in vs.items() if v and v.get("wrong_claims")],
            "grounding": sum(int((v or {}).get("grounding_violations", 0)) for v in vs.values()),
            "stop_latencies": lat, "by_category": {k: tuple(v) for k, v in cats.items()},
            "sim_s": round(sum((r or {}).get("sim_s", 0.0) for r in recs.values()), 0), "path_m": round(sum((r or {}).get("path_m", 0.0) for r in recs.values()), 1)}


def pct(x: float) -> str:
    return f"{100 * x:.0f} %"


def render(runs: list[tuple[str, dict]], suite, baseline: dict | None) -> str:
    stats = [run_stats(r, suite) for _, r in runs]
    rates = [s["rate"] for s in stats]
    L = []
    L.append("| | " + " | ".join(name for name, _ in runs) + " | **mean** | **min-max** |")
    L.append("|---|" + "---|" * (len(runs) + 2))

    def row(label, f, fmt=str, agg=True):
        vals = [f(s) for s in stats]
        cells = " | ".join(fmt(v) for v in vals)
        if agg and all(isinstance(v, (int, float)) for v in vals):
            L.append(f"| {label} | {cells} | {fmt(statistics.mean(vals))} | {fmt(min(vals))} - {fmt(max(vals))} |")
        else:
            L.append(f"| {label} | {cells} | | |")
    row(f"missions passed (of {stats[0]['n']})", lambda s: s["success"], lambda v: f"{v:.1f}".rstrip("0").rstrip("."))
    row("success rate (all missions)", lambda s: s["rate"], pct)
    row("success rate (valid missions only)", lambda s: s["rate_valid"], pct)
    row("infrastructure failures", lambda s: len(s["infra"]), lambda v: f"{v:g}")
    row("falls (tilt >= 60 deg)", lambda s: s["falls"], lambda v: f"{v:g}")
    row("wrong claims", lambda s: s["wrong_claims"], lambda v: f"{v:g}")
    row("grounding violations in final replies", lambda s: s["grounding"], lambda v: f"{v:g}")
    row("timeouts", lambda s: len(s["timeouts"]), lambda v: f"{v:g}")
    lat_all = [x for s in stats for x in s["stop_latencies"]]
    L.append("")
    if lat_all:
        L.append(f"Stop latency (wall s from the word 'stop' to measured speed < 0.02 m/s, only stops that began while moving): n = {len(lat_all)}, median {statistics.median(lat_all):.2f}, mean {statistics.mean(lat_all):.2f}, min {min(lat_all):.2f}, max {max(lat_all):.2f}.")
    else:
        L.append("Stop latency: no measurable stop.")
    L.append("")
    cats = list(stats[0]["by_category"])
    L.append("| category | " + " | ".join(name for name, _ in runs) + " | all runs |")
    L.append("|---|" + "---|" * (len(runs) + 1))
    for c in cats:
        tot = sum(s["by_category"][c][0] for s in stats); n = sum(s["by_category"][c][1] for s in stats)
        L.append(f"| {c} | " + " | ".join(f"{s['by_category'][c][0]}/{s['by_category'][c][1]}" for s in stats) + f" | {tot}/{n} ({pct(tot / n)}) |")
    L.append("")
    L.append("| mission | passes | " + " | ".join(name for name, _ in runs) + " | failing check (first run that failed) |")
    L.append("|---|---|" + "---|" * len(runs) + "---|")
    for m in suite.SUITE:
        cells, passes, why = [], 0, ""
        for _, recs in runs:
            r = recs[m["id"]]
            v = (r or {}).get("verdict")
            if v is None:
                cells.append("no result")
            elif v.get("infra_failure"):
                cells.append(f"INFRA ({r.get('status')})")
            elif v["success"]:
                cells.append("pass"); passes += 1
            else:
                cells.append("FAIL" + (" (timeout)" if v.get("timed_out") else ""))
                if not why:
                    bad = [c["why"] for c in v["checks"] if not c["ok"]] + v.get("arrival_errors", []) + (["fall"] if v.get("falls") else [])
                    why = "; ".join(bad)[:160] or ("timeout" if v.get("timed_out") else "?")
        L.append(f"| `{m['id']}` | {passes}/{len(runs)} | " + " | ".join(cells) + f" | {why} |")
    if baseline is not None:
        b = baseline
        L.append("")
        L.append(f"Baseline flat_lab (Gurdy 2.0 final eval, one run): {b['success']}/{b['n']} ({pct(b['rate'])}), falls {b['falls']}, wrong claims {b['wrong_claims']}.")
    return "\n".join(L)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--json")
    ap.add_argument("--baseline", help="run dir of the old flat_lab run (scored with its own suite)")
    a = ap.parse_args()
    suite = suite_module()
    runs = [(f"run {i + 1}", load_run(Path(r), suite)) for i, r in enumerate(a.runs)]
    base = None
    if a.baseline:
        prev = os.environ.get("GURDY_FINAL_SUITE")
        os.environ["GURDY_FINAL_SUITE"] = "final_suite"
        old = suite_module()
        base = run_stats(load_run(Path(a.baseline), old), old)
        if prev is None:
            os.environ.pop("GURDY_FINAL_SUITE")
        else:
            os.environ["GURDY_FINAL_SUITE"] = prev
    print(render(runs, suite, base))
    if a.json:
        json.dump({"runs": [run_stats(r, suite) for _, r in runs], "baseline": base}, open(a.json, "w"), indent=1, default=list)


if __name__ == "__main__":
    main()
