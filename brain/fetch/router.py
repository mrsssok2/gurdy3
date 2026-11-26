"""Deterministic routing of fetch utterances -> fetch intents (no LLM). None = not a fetch command (the planner handles it).

Intents: {"intent": "fetch", "object": {...}, "mode": "user" | "home" | "room", "room": ...}   bring me X / bring X to room B
         {"intent": "pick", "object": {...}}                                                     pick up / grab X (and hold it)
         {"intent": "deliver", "mode": ..., "room": ...}                                         put it in room B / bring it here
         {"intent": "hand_over"}                                                                hand it to me / give it to me
         {"intent": "refuse", "code": ..., "reply": ...}                                        safety (throw, people)
"""
from __future__ import annotations

import re

from .catalog import COLOURS, PICKABLE, TOO_BIG
from .safety import refuse_reason

VERB = r"(bring|fetch|get|grab|pick|lift|carry|put|place|drop off|leave|hand|give|deliver|take|move|throw|toss|hurl|fling|chuck|lob)"
FETCH_VERB_RE = re.compile(rf"\b{VERB}\b", re.I)
ROOM_RE = re.compile(r"\broom\s+([a-d])\b", re.I)
PRONOUN_RE = re.compile(r"\b(it|that|this|them)\b", re.I)
TO_USER_RE = re.compile(r"\b(to me|me the|me a|me an|me that|me it|me this|here|over here|back to me|for me)\b|\b(bring|fetch|get|grab)\s+me\b", re.I)
HOME_RE = re.compile(r"\b(home|back home|to the start|where we started)\b", re.I)
HAND_RE = re.compile(r"\b(hand (it|that|this|the \w+(?: \w+)?)\s*(to me|over)|hand over|give (it|that|this) (to me|back|here)|give me (it|that|this)|"
                     r"hand me (it|that|this))\b", re.I)
PLACE_VERB_RE = re.compile(r"\b(put|place|drop off|leave|set|deliver|take|move|bring|carry)\b", re.I)
PICK_ONLY_RE = re.compile(r"\b(pick (up )?|grab|lift)\b", re.I)

ALIASES = sorted([(w, c) for c, (_, _, ws) in PICKABLE.items() for w in ws] + [(w, c) for c, (_, _, ws) in TOO_BIG.items() for w in ws]
                 + [("mannequin", "mannequin"), ("dummy", "mannequin")], key=lambda t: -len(t[0]))


def find_object(text: str) -> dict | None:
    t = text.lower()
    for w, c in ALIASES:                                   # longest first: "mini toolbox" before "toolbox"
        m = re.search(rf"\b{re.escape(w)}s?\b", t)
        if m:
            obj = {"class": c}
            if re.search(r"\b(big|large|huge|tall|traffic|fire|full[- ]size)\s+(\w+\s+)?" + re.escape(w.split()[-1]), t):
                obj["explicit_size"] = True                # "the big cone", "the fire extinguisher": not the mini one
            before = t[:m.start()].split()[-2:]
            col = next((x for x in before if x in COLOURS), None)
            if col:
                obj["color"] = col
            return obj
    return None


def room_in(text: str, after: str | None = None) -> str | None:
    s = text if after is None else text[text.lower().find(after):] if after in text.lower() else ""
    m = ROOM_RE.search(s)
    return f"room {m.group(1).upper()}" if m else None


def parse(text: str) -> dict | None:
    if not FETCH_VERB_RE.search(text):
        return None
    ref = refuse_reason(text)
    if ref:
        return {"intent": "refuse", "code": ref[0], "reply": ref[1]}
    if HAND_RE.search(text):
        return {"intent": "hand_over"}
    obj = find_object(text)
    t = text.lower()
    # "X in room C" = where the object is; "to/in/into room B" after a place verb = destination
    dest_room = None
    m = re.search(r"\b(to|into|in|inside)\s+room\s+([a-d])\b", t)
    src_room = None
    if obj:
        # first room mention directly after the object noun = its location ("the can in room C to room B")
        rooms = [(mm.start(), f"room {mm.group(1).upper()}") for mm in ROOM_RE.finditer(t)]
        verbs_to = [mm.start() for mm in re.finditer(r"\b(to|into)\s+room\b", t)]
        for pos, r in rooms:
            if any(abs(pos - v) <= 5 for v in verbs_to):
                dest_room = r
            elif src_room is None:
                src_room = r
        if dest_room is None and PLACE_VERB_RE.search(t) and re.search(r"\b(put|place|leave|set|drop off)\b", t) and src_room:
            dest_room, src_room = src_room, None             # "put the cube in room B": room = destination
        if src_room:
            obj["room"] = src_room
        if TO_USER_RE.search(t):
            return {"intent": "fetch", "object": obj, "mode": "user"}
        if dest_room:
            return {"intent": "fetch", "object": obj, "mode": "room", "room": dest_room}
        if HOME_RE.search(t):
            return {"intent": "fetch", "object": obj, "mode": "home"}
        if PICK_ONLY_RE.search(t) or re.search(r"\b(get|fetch|take)\b", t):
            return {"intent": "pick", "object": obj}
        return None
    if PRONOUN_RE.search(t) or re.search(r"\b(the object|what you have|what you're holding)\b", t):
        if m:
            return {"intent": "deliver", "mode": "room", "room": f"room {m.group(2).upper()}"}
        if TO_USER_RE.search(t):
            return {"intent": "deliver", "mode": "user"}
        if HOME_RE.search(t):
            return {"intent": "deliver", "mode": "home"}
        if re.search(r"\b(put|set|place) (it|that|this) down\b", t):
            return {"intent": "deliver", "mode": "here"}
    return None
