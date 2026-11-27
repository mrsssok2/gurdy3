"""Deterministic routing of fetch utterances -> fetch intents (no LLM). None = not a fetch command (the planner handles it).

Intents: {"intent": "fetch", "object": {...}, "mode": "user" | "home" | "room", "room": ...}   bring me X / bring X to room B
         {"intent": "multi", "items": [fetch intents]}       "take the red can and the blue cube to me": ONE AFTER ANOTHER
         {"intent": "multi_pick", "objects": [...]}           "pick up the can and the cube": one gripper -> the session asks
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
# 'it' / 'them' are always pronouns; 'that' / 'this' only when no noun follows ("bring that to me", not "bring that tall figure")
PRONOUN_RE = re.compile(r"\b(it|them)\b|\b(that|this)\b(?=\s*(?:$|[,.!?]|to\b|in\b|into\b|over\b|here\b|back\b|home\b|down\b|"
                        r"please\b|for\b|on\b|at\b|now\b))", re.I)
TO_USER_RE = re.compile(r"\b(to me|me the|me a|me an|me that|me it|me this|here|over here|back to me|for me)\b|\b(bring|fetch|get|grab)\s+me\b", re.I)
HOME_RE = re.compile(r"\b(home|back home|to the start|where we started)\b", re.I)
HAND_RE = re.compile(r"\b(hand (it|that|this|the \w+(?: \w+)?)\s*(to me|over)|hand over|give (it|that|this) (to me|back|here)|give me (it|that|this)|"
                     r"hand me (it|that|this))\b", re.I)
PLACE_VERB_RE = re.compile(r"\b(put|place|drop off|leave|set|deliver|take|move|bring|carry)\b", re.I)
PICK_ONLY_RE = re.compile(r"\b(pick (up )?|grab|lift)\b", re.I)

ALIASES = sorted([(w, c) for c, (_, _, ws) in PICKABLE.items() for w in ws] + [(w, c) for c, (_, _, ws) in TOO_BIG.items() for w in ws]
                 + [("mannequin", "mannequin"), ("dummy", "mannequin")], key=lambda t: -len(t[0]))


# "can" is also the modal verb: "can you ...", "you can leave it" (a subject pronoun right after or right before it)
_SUBJ = r"(?:you|i|we|u|he|she|they|someone|somebody|anyone|anybody|one)"
MODAL_CAN_RE = re.compile(rf"\bcan\s+{_SUBJ}\b|\b{_SUBJ}\s+can\b", re.I)
CONJ_RE = re.compile(r"(?:,|\band\b|\bthen\b|\bplus\b|\bas well as\b)", re.I)


def _is_modal(t: str, m: re.Match) -> bool:
    if m.group(0).split()[0] not in ("can", "cans"):
        return False
    return any(mm.start() <= m.start() < mm.end() for mm in MODAL_CAN_RE.finditer(t))


def find_objects(text: str) -> list[dict]:
    """Every object mention in the text, in order of appearance: [{class, color?, explicit_size?, _span: (start, end)}].
    Longest alias first claims its span ("mini toolbox" before "toolbox"); the modal verb "can" is not an object."""
    t = text.lower()
    found, taken = [], []
    for w, c in ALIASES:
        for m in re.finditer(rf"\b{re.escape(w)}s?\b", t):
            if any(a < m.end() and m.start() < b for a, b in taken) or _is_modal(t, m):
                continue
            taken.append((m.start(), m.end()))
            obj = {"class": c, "_span": (m.start(), m.end())}
            if re.search(r"\b(big|large|huge|tall|traffic|fire|full[- ]size)\s+(\w+\s+)?" + re.escape(w.split()[-1]), t):
                obj["explicit_size"] = True                # "the big cone", "the fire extinguisher": not the mini one
            before = t[:m.start()].split()[-2:]
            col = next((x for x in before if x in COLOURS), None)
            if col:
                obj["color"] = col
            found.append(obj)
    return sorted(found, key=lambda o: o["_span"][0])


def _public(o: dict) -> dict:
    return {k: v for k, v in o.items() if not k.startswith("_")}


def find_object(text: str) -> dict | None:
    """The first object mentioned (see find_objects)."""
    objs = find_objects(text)
    return _public(objs[0]) if objs else None


def joined_objects(text: str) -> list[dict]:
    """Objects listed with a conjunction ("the red can and the blue cube", "a can, then the cube"). A second object NOT
    joined by a conjunction ("the cube next to the barrel") is a description, not another request: only the first."""
    objs = find_objects(text)
    t = text.lower()
    out = objs[:1]
    for a, b in zip(objs, objs[1:]):
        gap = t[a["_span"][1]:b["_span"][0]]
        if not CONJ_RE.search(gap) or ROOM_RE.search(gap) and not re.search(r"\band\b|,", gap):
            break
        out.append(b)
    return out


def room_in(text: str, after: str | None = None) -> str | None:
    s = text if after is None else text[text.lower().find(after):] if after in text.lower() else ""
    m = ROOM_RE.search(s)
    return f"room {m.group(1).upper()}" if m else None


# verbs that mean "bring it to me" when no other destination is given (fetch me X / bring the X / hand the X over)
TO_USER_VERB_RE = re.compile(r"\b(bring|fetch|hand|give|pass|deliver)\b", re.I)


def _src_rooms(t: str, objs: list[dict], dest_room: str | None) -> None:
    """A room mentioned after an object and before the next one (not the destination) is where that object is."""
    for i, o in enumerate(objs):
        end = objs[i + 1]["_span"][0] if i + 1 < len(objs) else len(t)
        for mm in ROOM_RE.finditer(t, o["_span"][1], end):
            r = f"room {mm.group(1).upper()}"
            if r != dest_room:
                o["room"] = r
                break


def parse(text: str) -> dict | None:
    if not FETCH_VERB_RE.search(text):
        return None
    ref = refuse_reason(text)
    if ref:
        return {"intent": "refuse", "code": ref[0], "reply": ref[1]}
    objs = joined_objects(text)
    if HAND_RE.search(text) and not objs:
        return {"intent": "hand_over"}                       # "hand it to me": what the robot holds
    t = text.lower()
    # "X in room C" = where the object is; "to/into room B" = destination
    m = re.search(r"\b(to|into|in|inside)\s+room\s+([a-d])\b", t)
    if objs:
        rooms = [(mm.start(), f"room {mm.group(1).upper()}") for mm in ROOM_RE.finditer(t)]
        verbs_to = [mm.start() for mm in re.finditer(r"\b(to|into)\s+room\b", t)]
        dest_room = next((r for pos, r in rooms if any(abs(pos - v) <= 5 for v in verbs_to)), None)
        _src_rooms(t, objs, dest_room)
        first_src = objs[0].get("room")
        if dest_room is None and len(objs) == 1 and re.search(r"\b(put|place|leave|set|drop off)\b", t) and first_src:
            dest_room = first_src                            # "put the cube in room B": room = destination
            objs[0].pop("room")
        # destination: an explicit room beats "for me" (benefactive), then the user words, then home
        if dest_room:
            dest = {"mode": "room", "room": dest_room}
        elif TO_USER_RE.search(t):
            dest = {"mode": "user"}
        elif HOME_RE.search(t):
            dest = {"mode": "home"}
        elif TO_USER_VERB_RE.search(t):
            dest = {"mode": "user"}                          # "fetch that blue tin", "hand over the cone"
        elif PICK_ONLY_RE.search(t) or re.search(r"\b(get|take)\b", t):
            dest = None                                      # pick up and hold
        else:
            return None
        items = [({"intent": "fetch", "object": _public(o), **dest} if dest else {"intent": "pick", "object": _public(o)})
                 for o in objs]
        if len(items) == 1:
            return items[0]
        if dest is None:
            return {"intent": "multi_pick", "objects": [_public(o) for o in objs]}
        return {"intent": "multi", "items": items}
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
