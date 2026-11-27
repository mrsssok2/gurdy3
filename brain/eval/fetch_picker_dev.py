"""DEV set + offline eval for the fetch LLM picker (brain/fetch/picker.py), like brain/eval/picker_dev.py for Phase 6d.

    scripts/fetch_picker_dev.sh [--model gurdy-sft-400] [--schema] [--out FILE.json]      # one command (starts Ollama if needed)
    python3 -m brain.eval.fetch_picker_dev [--model M ...] [--schema] [--out FILE.json] [--verbose]

DEV set, NOT held-out: written by the assistant on 2026-11-27 while building the Ollama client. While writing it, 4
phrasings showed that the pre-filter was too narrow ("can I have", "could really use", "ferrying", "should end up in"),
and the pre-filter was widened for them. So the pre-filter numbers on this set are optimistic; the user's blind suite is
the measurement. Disjoint from the prompt's examples, from brain/tests/test_fetch_picker.py and from fetch_dev_v1.yaml
(checked by brain/tests/test_fetch_picker_dev.py), and none of the texts is handled by the deterministic router (also checked),
so every case really reaches the picker.

Each case: (text, expected intent or None). Intent = what `picker.validate` returns (router.parse format); `alts` lists
other acceptable intents. Scoring is exact on the intent: intent type, object class / colour / room, mode, destination.
Rooms of the mock world (brain/fetch/mock.py): room A-D. Objects (lab4_fetch): red / blue cube (small_box), red / blue can,
orange / green mini cone, red mini extinguisher, yellow mini toolbox, + big ones and the mannequin.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from ..fetch.mock import ROOMS
from ..fetch.picker import FetchPicker, might_be_fetch
from ..fetch.router import parse


def F(cls, color=None, room=None, mode="user", dest=None) -> dict:
    o = {"class": cls, **({"color": color} if color else {}), **({"room": room} if room else {})}
    return {"intent": "fetch", "object": o, "mode": mode, **({"room": dest} if mode == "room" else {})}


def PICK(cls, color=None, room=None) -> dict:
    return {"intent": "pick", "object": {"class": cls, **({"color": color} if color else {}), **({"room": room} if room else {})}}


def DELIVER(mode, dest=None) -> dict:
    return {"intent": "deliver", "mode": mode, **({"room": dest} if mode == "room" else {})}


HAND = {"intent": "hand_over"}

DEV = [
    # ---- bring to the user (fetch)
    ("I'd really like the blue soda tin over here", F("can", "blue")),
    ("could you pass me that little orange cone", F("mini_cone", "orange")),
    ("I need the red cube, please", F("small_box", "red")),
    ("the green mini cone, I want it next to me", F("mini_cone", "green")),
    ("can I have the yellow mini toolbox", F("mini_toolbox", "yellow")),
    ("my hands are full of paperwork, I could use the red tin", F("can", "red")),
    ("snag the red cube for me", F("small_box", "red")),
    ("the can that's in room C, I need it", F("can", room="room C")),
    ("I'd love the small red extinguisher over by my desk", F("mini_extinguisher", "red")),
    ("I could really use a mini extinguisher here", F("mini_extinguisher")),
    ("pass me a can, any colour", F("can")),
    ("I want the dummy brought over here", F("mannequin")),                 # the session refuses it afterwards (never people)
    # ---- bring to a room / home
    ("would you mind ferrying the blue cube to room A", F("small_box", "blue", mode="room", dest="room A")),
    ("I'd like the red soda can delivered to room B", F("can", "red", mode="room", dest="room B")),
    ("the orange mini cone should end up in room B", F("mini_cone", "orange", mode="room", dest="room B")),
    ("the blue cube in room D, I want it in room A", F("small_box", "blue", room="room D", mode="room", dest="room A")),
    ("the orange mini cone, I'd love it in room C", F("mini_cone", "orange", mode="room", dest="room C")),
    ("I want the small toolbox left in room B", F("mini_toolbox", mode="room", dest="room B")),
    ("I'd like the red cube back at home base", F("small_box", "red", mode="home")),
    # ---- pick up and hold
    ("nab the yellow mini toolbox and keep hold of it, I'll want it later", PICK("mini_toolbox", "yellow")),
    # ---- what the robot already holds
    ("pass it over", HAND),
    ("give me whatever you're holding", HAND),
    ("I'd like whatever you're carrying handed to me", HAND),
    ("I need that thing you're holding", HAND),
    ("drop what you're carrying in room B", DELIVER("room", "room B")),
    ("leave whatever you have at home", DELIVER("home")),
    ("whatever you're holding, room D is where it should end up, put it there", DELIVER("room", "room D")),
    # ---- not fetch requests (the model must say "none"; unknown objects must not become a known class)
    ("fetch me the stapler", None),
    ("I'd love a cup of coffee", None),
    ("take a photo of room C", None),
    ("get going to room B", None),
    ("move over to the doorway a bit", None),
    ("I want to know how much battery is left", None),
    ("leave room A and come back", None),
    ("pick a room and explore it", None),
    ("give me a status report", None),
    ("I need you to look for the red box", None),
    ("I'd like you to patrol room D", None),
    ("where did you put the red can?", None),
    ("keep the cube you have and come to me", None),
]
ALTS = {   # other acceptable intents (genuinely two readings)
    "keep the cube you have and come to me": [HAND],          # coming to the user holding it ~ a hand-over
}


def group(expected) -> str:
    if expected is None:
        return "none"
    if expected["intent"] in ("hand_over", "deliver"):
        return "held"
    if expected["intent"] == "pick":
        return "pick"
    return "fetch_user" if expected["mode"] == "user" else "fetch_place"


def run(picker: FetchPicker, verbose: bool = True) -> dict:
    rooms = sorted(ROOMS)
    rows, by = [], {}
    for text, exp in DEV:
        got = picker.pick(text, rooms)
        ok = got == exp or got in ALTS.get(text, [])
        g = group(exp)
        by.setdefault(g, [0, 0])
        by[g][0] += ok
        by[g][1] += 1
        rows.append({"text": text, "expected": exp, "got": got, "ok": ok, "prefilter": might_be_fetch(text),
                     "raw": picker.last_raw if isinstance(picker.last_raw, (str, type(None))) else str(picker.last_raw)})
    n_ok = sum(r["ok"] for r in rows)
    fetch_rows = [r for r in rows if r["expected"] is not None]
    out = {"n": len(rows), "ok": n_ok, "by_group": {k: f"{v[0]}/{v[1]}" for k, v in by.items()},
           "false_fetch": sum(1 for r in rows if r["expected"] is None and r["got"] is not None),   # a non-fetch text turned into an action
           "wrong_fetch": sum(1 for r in fetch_rows if r["got"] is not None and not r["ok"]),       # acted, but the wrong intent
           "missed": sum(1 for r in fetch_rows if r["got"] is None),                                # fell through (safe: the planner handles it)
           "prefilter_miss": sum(1 for r in fetch_rows if not r["prefilter"]), **picker.stats(), "rows": rows}
    if verbose:
        print({k: v for k, v in out.items() if k != "rows"})
        for r in rows:
            if not r["ok"]:
                print(f"  WRONG {r['text']!r}\n        expected {r['expected']}\n        got      {r['got']}   raw {r['raw']!r}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", action="append", default=[])
    ap.add_argument("--schema", action="store_true", help="send the full JSON schema as Ollama format (default: JSON mode)")
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--out", help="write all results (per model) as JSON")
    ap.add_argument("--verbose", action="store_true", help="also list the correct ones")
    a = ap.parse_args(argv)
    res = []
    for m in a.model or ["gurdy-sft-400"]:
        pk = FetchPicker(model=m, schema=a.schema, timeout=a.timeout)
        print(f"== {m} (format {'schema' if a.schema else 'json'}, {pk.llm.host})")
        r = run(pk)
        r.update({"model": m, "schema": a.schema})
        if a.verbose:
            for row in r["rows"]:
                if row["ok"]:
                    print(f"  ok    {row['text']!r} -> {row['got']}")
        if pk.failures == pk.calls and pk.calls:
            print(f"!! every call failed: is Ollama running at {pk.llm.host} with model {m!r}?", file=sys.stderr)
        res.append(r)
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=1) + "\n")
    return 0 if all(r["calls"] > r["failures"] for r in res) else 1


if __name__ == "__main__":
    sys.exit(main())
