"""Run the Phase 9d fetch DEV suite offline against the mock backend.

    python3 -m brain.eval.fetch_dev [--suite brain/eval/fetch_dev_v1.yaml] [--failures]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from brain.fetch.mock import FetchMock
from brain.fetch.replies import ground
from brain.fetch.session import FetchSession

DEFAULT = Path(__file__).with_name("fetch_dev_v1.yaml")


def _faults(f):
    out = {}
    for k, v in (f or {}).items():
        out[k] = set(v) if isinstance(v, list) else v
    return out


def run_case(c) -> tuple[bool, list, list]:
    s = FetchSession(FetchMock.lab4_fetch(_faults(c.get("faults"))))
    replies, problems = [], []
    for t in c["turns"]:
        r = s.handle(t)
        replies.append(r)
        if r is not None:
            held = s.r.carrying is not None
            conf = (s.last.confirmed if s.last else set()) | ({"picked"} if held else set())
            problems += [f"grounding: {v}" for v in ground(r, conf, held)]
    last = replies[-1]
    if c.get("none"):
        if last is not None:
            problems.append(f"expected no fetch route, got {last!r}")
        return not problems, replies, problems
    if last is None:
        return False, replies, ["not routed (None)"]
    problems += [f"missing {h!r}" for h in c.get("has", []) if h.lower() not in last.lower()]
    problems += [f"must not say {h!r}" for h in c.get("not", []) if h.lower() in last.lower()]
    if "tools" in c and [x["tool"] for x in s.calls] != c["tools"]:
        problems.append(f"tools {[x['tool'] for x in s.calls]} != {c['tools']}")
    if "holding" in c and s.r.carrying != c["holding"]:
        problems.append(f"holding {s.r.carrying!r} != {c['holding']!r}")
    return not problems, replies, problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default=str(DEFAULT))
    ap.add_argument("--failures", action="store_true")
    a = ap.parse_args()
    cases = yaml.safe_load(Path(a.suite).read_text())["cases"]
    by_group, n_ok = {}, 0
    for c in cases:
        ok, replies, probs = run_case(c)
        g = {"f": "fetch", "s": "safety", "e": "failures", "n": "not fetch"}[c["id"][0]]
        by_group.setdefault(g, [0, 0])
        by_group[g][0] += ok
        by_group[g][1] += 1
        n_ok += ok
        if a.failures and not ok or not a.failures:
            print(f"{'PASS' if ok else 'FAIL'} {c['id']}: {replies[-1]!r}" + ("" if ok else f"  <- {probs}"))
    print(f"\n{n_ok}/{len(cases)} pass; " + ", ".join(f"{g} {v[0]}/{v[1]}" for g, v in by_group.items()))


if __name__ == "__main__":
    main()
