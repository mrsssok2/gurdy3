"""FOURTH incident question set (Phase 6d) for OFFLINE REPLAY only (no Gazebo, `incident_replay`): 20 NEW own questions on the same scenario as v1-v3
(red box A -> C, red barrel D -> B, toolbox removed, everything else untouched), written AFTER the picker, the router and the templates were frozen (commit before this
file) and replayed ONCE against the recorded graph of the 6c run. Written by the assistant that wrote the router and the picker: not independent. No external questions.
Scorer: class with optional colour (`use_suite("incident_suite_v4")`)."""
from __future__ import annotations

import glob
import os

from . import incident_suite as V1
from . import incident_suite_v2 as V2
from . import incident_suite_v3 as V3
from .incident_suite import (CHANGED, FINAL_WAIT_S, MAP_ID, MAP_NAME, OPS, PLACES_FILE, ROUND1, ROUND2, SCENARIO, SWEEP_LIMIT_S, UNCHANGED_WORDS, WORLD,  # noqa: F401
                             Q, X, _norm)

NAME = "incident_v4"
ASSUMED = r"(don't|do not) know a room|no such room|not a room|there is no room"

QUESTIONS = [
    # ---- what changed (6)
    Q("p01", "changes", "Were any of the items swapped around while I was gone?", "what_changed",
      all=[r"red box", r"barrel"], none=[r"nothing (has )?changed"]),
    Q("p02", "changes", "Has the contents of room C been altered?", "what_changed",
      all=[r"red box"], none=[r"toolbox", r"barrel", r"nothing (has )?changed"]),
    Q("p03", "changes", "Something feels off in room D, is it?", "what_changed",
      all=[r"toolbox|barrel"], none=[r"red box", r"nothing (has )?changed"]),
    Q("p04", "changes", "Has room A lost anything since the start?", "what_changed",
      all=[r"red box"], none=[r"toolbox", r"barrel", r"nothing (has )?changed"]),
    Q("p05", "changes", "Did anything go astray?", "what_changed",
      all=[r"toolbox"], none=[r"blue box|yellow box|mannequin|extinguisher|blue barrel"]),
    Q("p06", "changes", "Give me the differences between now and when you began.", "what_changed",
      all=[r"red box", r"barrel", r"toolbox"], none=[r"blue box|yellow box|mannequin|extinguisher|blue barrel"]),
    # ---- when / where last seen (4)
    Q("p07", "last_seen", "At what time did you last have the toolbox in sight?", "last_seen",
      all=[r"toolbox", r"not there|missing|gone"], time={"round": 1, "room": "room D", "tol_s": 300}),
    Q("p08", "last_seen", "Where did the red box show up most recently?", "last_seen",
      all=[r"room C"], none=[r"room A"]),
    Q("p09", "last_seen", "How long has the mannequin been out of your view?", "last_seen",
      all=[r"mannequin", r"room D"]),
    Q("p10", "last_seen", "When was a ladder last on your camera?", "last_seen",
      all=[r"no record|never|haven't seen|have not seen|don't know what"], none=[r"room [A-D]"]),
    # ---- has it moved (4)
    Q("p11", "history", "Has the red barrel stayed in room D?", "object_history",
      all=[r"^\W*no", r"room B"], none=[r"^\W*yes"]),
    Q("p12", "history", "Was the blue barrel ever shoved somewhere else?", "object_history",
      all=[r"haven't recorded|no record|not moved|hasn't moved|did not move"], none=[r"^\W*yes"]),
    Q("p13", "history", "Is the red box in the spot where it began?", "object_history",
      all=[r"^\W*no", r"room C"], none=[r"^\W*yes"]),
    Q("p14", "history", "Did the toolbox vanish?", "object_history",
      all=[r"missing|not there|gone"], none=[r"^\W*yes"]),
    # ---- timeline / why (3)
    Q("p15", "timeline", "Bring me up to date on what's been going on in room C.", "timeline",
      all=[r"room C"], none=[r"toolbox"]),
    Q("p16", "timeline", "What have you been busy with since I gave you that first order?", "timeline",
      all=[r"entered room"]),
    Q("p17", "timeline", "What led to you coming to a halt after my command?", "incident_why",
      all=[r"told me to|you said|stop"]),
    # ---- which rooms / when last checked (3)
    Q("p18", "rooms", "Which rooms have you left alone for over ten minutes?", "unchecked_rooms", unchecked={"threshold_s": 600}),
    Q("p19", "rooms", "When did you last look in on room B?", "unchecked_rooms",
      all=[r"room B"]),
    Q("p20", "rooms", "Does a room H exist on your map?", "unchecked_rooms",
      all=[r"there is no room|no such room|don't know a room"]),
]

EXTERNAL: list = []
ALL_QUESTIONS = QUESTIONS + EXTERNAL
BY_ID = {q["id"]: q for q in ALL_QUESTIONS}


def say_texts() -> list[str]:
    return [s[1] for s in SCENARIO if s[0] in ("say", "stop")] + [q["text"] for q in ALL_QUESTIONS]


def check_overlap(root: str = ".") -> dict:
    """Questions that also occur (normalised) in the v1 incident questions, any older suite, test set, SFT data, voice prompts, my_commands or a unit test: must be {}."""
    from . import final_suite_lab4 as base, final_suite_lab4_v2, final_suite_lab4_v3
    mine = {_norm(q["text"]): q["text"] for q in ALL_QUESTIONS}
    hits: dict[str, list[str]] = {}
    for q in V1.ALL_QUESTIONS + V2.ALL_QUESTIONS + V3.ALL_QUESTIONS:
        if _norm(q["text"]) in mine:
            hits.setdefault(mine[_norm(q["text"])], []).append("incident_suite")
    saved = base.say_texts
    base.say_texts = lambda: list(mine.values())
    try:
        for k, v in base.check_overlap(root).items():
            hits.setdefault(k, []).extend(v)
    finally:
        base.say_texts = saved
    for mod in (base, final_suite_lab4_v2, final_suite_lab4_v3):
        for t in [s[1] for m in mod.SUITE for s in m["steps"] if s[0] in ("say", "stop")]:
            if _norm(t) in mine:
                hits.setdefault(mine[_norm(t)], []).append(mod.__name__.rsplit(".", 1)[-1])
    for f in glob.glob(os.path.join(root, "brain/tests/*.py")) + glob.glob(os.path.join(root, "web/tests/*.py")):
        if os.path.basename(f) in ("test_incident_suite.py", "test_incident_suite_v2.py", "test_incident_suite_v3.py", "test_incident_suite_v4.py"):
            continue
        txt = _norm(open(f).read())
        for n, t in mine.items():
            if n in txt:
                hits.setdefault(t, []).append(os.path.basename(f))
    return hits
