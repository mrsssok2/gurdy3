"""Measure the incident tool picker on the DEV set, offline (needs the project's Ollama):
     python3 -m brain.eval.picker_dev --model gurdy-sft-400 [--model qwen3:1.7b ...]
Reports tool accuracy (incl. "none"), room accuracy, validation rejections, failures and latency; lists the wrong ones."""
from __future__ import annotations

import argparse
import statistics

from ..incidents.picker import IncidentPicker, might_be_incident, validate
from .incident_picker_dev import DEV

ROOMS = ["room A", "room B", "room C", "room D"]


def run(model: str, verbose: bool = True) -> dict:
    pk = IncidentPicker(model=model)
    ok_tool = ok_room = n_room = prefilter_miss = 0
    wrong = []
    for text, tool, room in DEV:
        if tool is not None and not might_be_incident(text):
            prefilter_miss += 1
        plan = pk.pick(text, ROOMS)
        got = plan["tool"] if plan else None
        good = got == tool
        ok_tool += good
        if good and tool is not None and room:
            n_room += 1
            ok_room += (plan["args"].get("room") == room)
        if not good:
            wrong.append((text, tool, got))
    out = {"model": model, "n": len(DEV), "tool_ok": ok_tool, "room_ok": f"{ok_room}/{n_room}", "prefilter_miss": prefilter_miss, "failures": pk.failures,
           "invalid": pk.invalid, "latency_p50_s": round(statistics.median(pk.latencies), 3) if pk.latencies else None, "wrong": wrong}
    if verbose:
        print({k: v for k, v in out.items() if k != "wrong"})
        for w in wrong:
            print("  WRONG", w)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", action="append", default=[])
    for m in ap.parse_args().model or ["gurdy-sft-400"]:
        run(m)
