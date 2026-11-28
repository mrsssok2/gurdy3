"""VLN plan: the constrained JSON an LLM may return for a navigation instruction, and its validation + grounding against
the map (Phase 10 prep, docs/results/vln_plan.md). Same principle as the Phase 6d incident picker: the model only
CHOOSES a structure; every name, colour and room must come from the user's words, every landmark from the semantic map,
every coordinate from code.

Model output:
  {"kind": "go" | "none",
   "steps": [{"action": "pass" | "enter" | "stop", "target": TARGET}, ...],        1..6 steps; the last is enter / stop
   "avoid": ["room B", ...]}
  TARGET = {"room": "room D"}
         | {"object": REF}                          a landmark
         | {"room_with": REF}                       "the room with the red barrel"
         | {"relation": REL, "of": [REF] or [REF, REF]}    REL: left_of right_of next_to behind in_front_of between
  REF    = {"class": <class>, "color": <colour>?, "in_room": "room C"?}

validate(raw, text, world) -> Plan (ok) | Plan with an error:
  rejected (the model is not trusted; `None` reply -> the keyword fallback, then the normal planner):
    BAD_JSON, BAD_SCHEMA, NOT_NAV (kind none), UNSAID (a class / colour / room the user did not say), DROPPED (an object /
    room the user named is not in the plan), AVOID_DROPPED (the user said "avoid room B" and the plan does not), AVOID_ADDED
  answered (a reply for the user, nothing moves):
    LANDMARK_NOT_FOUND, AMBIGUOUS (with the options), UNKNOWN_ROOM, IMPOSSIBLE (goal / landmark in an avoided room, no
    route around), NO_FREE_SPOT (a relation point inside a wall / object)
A valid plan: waypoints [{x, y, kind: pass | door | enter | stop, room, label, landmark?}], the goal region, the room
sequence, the avoided rooms, a length estimate and progress checks (landmarks to pass within 1.2 m, rooms in order).
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field

from .world import VLNWorld

ACTIONS = ("pass", "enter", "stop")
RELATIONS = ("left_of", "right_of", "next_to", "behind", "in_front_of", "between")
CLASSES = ("box", "barrel", "fire_extinguisher", "traffic_cone", "toolbox", "mannequin", "chair")
COLOURS = ("red", "blue", "green", "yellow", "orange", "purple", "pink", "white", "black", "grey", "gray", "brown", "silver")
WORDS = {   # word -> class (the detector's classes; English words people use)
    "box": "box", "boxes": "box", "crate": "box", "crates": "box", "carton": "box",
    "barrel": "barrel", "barrels": "barrel", "drum": "barrel", "drums": "barrel",
    "extinguisher": "fire_extinguisher", "extinguishers": "fire_extinguisher",
    "cone": "traffic_cone", "cones": "traffic_cone",
    "toolbox": "toolbox", "toolboxes": "toolbox",
    "mannequin": "mannequin", "dummy": "mannequin",
    "chair": "chair", "chairs": "chair",
}
MAX_STEPS = 6
OFFSET = 0.7                 # m: relation points beside / behind / in front of a landmark
PASS_RING = (0.6, 0.8, 1.0)  # m: "go past X" = a free point this close round X (the whole-route length picks the side)
STOP_STANDOFF = 0.6          # m: "go to X" stops this far in front of it
PASS_CHECK = 1.2             # m: progress check: the path must come this close to a landmark it passes
GOAL_RADIUS = 0.5            # m: success radius of a point goal (Nav2 tolerance 0.12 + margin)
_AVOID_VERB = (r"\b(?:avoid(?:ing)?|stay(?:ing)? out of|keep(?:ing)? out of|never (?:go|enter|walk)(?: into| through| in)?|"
               r"without (?:going|passing|walking) (?:through|into|in)|not (?:through|via|into)|"
               r"(?:don't|do not|dont) (?:go|walk|pass|enter|cross)(?: through| into| in| via)?|skip(?:ping)?|bypass(?:ing)?)\s+")


def _room_word_re(names=()) -> str:
    """'room X' or one of the map's spoken room names ('the kitchen'), as a regex alternative."""
    alts = [r"room\s+[a-z]\b"] + [r"(?:the\s+)?" + re.escape(n) + r"\b" for n in sorted(names, key=len, reverse=True)]
    return "(?:" + "|".join(alts) + ")"


def avoid_re(names=()):
    r = _room_word_re(names)
    return re.compile(_AVOID_VERB + rf"((?:{r}(?:\s*(?:,|and|or|nor)\s*)?)+)", re.I)


AVOID_RE = avoid_re()


# ------------------------------------------------------------------ the user's words
def _tokens(text):
    return re.findall(r"[a-z]+", text.lower())


def mentions(text: str) -> list[dict]:
    """Every object the user named, in order: [{class, color?}] (colour = one of the 2 words before)."""
    toks = _tokens(text)
    out = []
    for i, t in enumerate(toks):
        c = WORDS.get(t)
        if c is None:
            continue
        ref = {"class": c}
        for back in (1, 2, 3):
            if i - back >= 0 and toks[i - back] in COLOURS:
                ref["color"] = "grey" if toks[i - back] == "gray" else toks[i - back]
                break
            if i - back >= 0 and WORDS.get(toks[i - back]):
                break
        out.append(ref)
    return out


def _room_words(s: str, names: dict) -> list[str]:
    """'room X' (any letter, even one the map does not have) and the spoken room names, in order of appearance."""
    pat = r"\broom\s+([a-z])\b" + "".join(rf"|\b({re.escape(n)})\b" for n in sorted(names, key=len, reverse=True))
    out = []
    for m in re.finditer(pat, s.lower()):
        out.append(f"room {m.group(1).upper()}" if m.group(1) else names[next(g for g in m.groups()[1:] if g)])
    return out


def rooms_said(text: str, world=None) -> list[str]:
    return _room_words(text, getattr(world, "names", None) or {})


def avoid_said(text: str, world=None) -> set[str]:
    names = getattr(world, "names", None) or {}
    out = set()
    for m in avoid_re(names).finditer(text):
        out |= set(_room_words(m.group(1), names))
    return out


# ------------------------------------------------------------------ result types
@dataclass
class Plan:
    ok: bool
    error: str | None = None
    reply: str | None = None               # for answered errors (None for rejected ones: the planner handles the text)
    options: list = field(default_factory=list)
    waypoints: list = field(default_factory=list)
    goal: dict | None = None               # {"kind": "room", "room"} | {"kind": "point", "x", "y", "radius", "room"}
    rooms: list = field(default_factory=list)
    avoid: list = field(default_factory=list)
    checks: list = field(default_factory=list)
    length_m: float = 0.0
    summary: str = ""
    notes: list = field(default_factory=list)
    source: str = ""                       # "llm" | "keyword": which parser produced the JSON (set by VLNPlanner)
    goal_landmark: dict | None = None      # "the room with the red barrel": the object that defines the goal room (re-observed)

    @property
    def answered(self) -> bool:
        return self.reply is not None

    def to_dict(self):
        return {k: v for k, v in self.__dict__.items()}


def _reject(code, detail=""):
    return Plan(False, code, None, notes=[detail] if detail else [])


def _answer(code, reply, options=()):
    return Plan(False, code, reply, list(options))


def _json(raw):
    if isinstance(raw, dict):
        return raw
    if not isinstance(raw, str):
        return None
    s = re.sub(r"^```(?:json)?\s*|\s*```$", "", raw.strip())
    try:
        v = json.loads(s)
    except ValueError:
        return None
    return v if isinstance(v, dict) else None


# ------------------------------------------------------------------ schema
def _ref_ok(r) -> bool:
    return (isinstance(r, dict) and set(r) <= {"class", "color", "in_room"} and r.get("class") in CLASSES
            and (r.get("color") is None or r.get("color") in COLOURS) and (r.get("in_room") is None or isinstance(r["in_room"], str)))


def _target_ok(t) -> bool:
    if not isinstance(t, dict) or len(t) not in (1, 2):
        return False
    if set(t) == {"room"}:
        return isinstance(t["room"], str)
    if set(t) == {"object"}:
        return _ref_ok(t["object"])
    if set(t) == {"room_with"}:
        return _ref_ok(t["room_with"])
    if set(t) == {"relation", "of"}:
        of = t["of"]
        n = 2 if t["relation"] == "between" else 1
        return t["relation"] in RELATIONS and isinstance(of, list) and len(of) == n and all(_ref_ok(r) for r in of)
    return False


def schema_ok(d) -> bool:
    if not isinstance(d, dict) or not set(d) <= {"kind", "steps", "avoid"} or d.get("kind") not in ("go", "none"):
        return False
    if d["kind"] == "none":
        return True
    st, av = d.get("steps"), d.get("avoid", [])
    if not isinstance(st, list) or not 1 <= len(st) <= MAX_STEPS or not isinstance(av, list) or not all(isinstance(a, str) for a in av):
        return False
    for s in st:
        if not isinstance(s, dict) or set(s) != {"action", "target"} or s["action"] not in ACTIONS or not _target_ok(s["target"]):
            return False
    if st[-1]["action"] == "pass":
        return False                                     # the walk must end somewhere
    return all(s["action"] != "stop" for s in st[:-1])   # "stop" only at the end


# ------------------------------------------------------------------ the validator
def _canon_room(name, world):
    return world.canon_room(name)


def _refs(d):
    for s in d["steps"]:
        t = s["target"]
        if "object" in t:
            yield t["object"]
        elif "room_with" in t:
            yield t["room_with"]
        elif "of" in t:
            yield from t["of"]


def _rooms_in(d):
    for s in d["steps"]:
        t = s["target"]
        if "room" in t:
            yield t["room"]
    for r in _refs(d):
        if r.get("in_room"):
            yield r["in_room"]


def describe_ref(r) -> str:
    c = r["class"].replace("_", " ")
    return f"the {r['color'] + ' ' if r.get('color') else ''}{c}" + (f" in {r['in_room']}" if r.get("in_room") else "")


def _obj_label(o, world):
    return f"the {o['color'] + ' ' if o.get('color') else ''}{o['class'].replace('_', ' ')} in {world.room_label(world.room_of(o['x'], o['y']))}"


def ground(ref, world):
    """-> list of matching semantic-map objects."""
    c = [o for o in world.objects if o["class"] == ref["class"]]
    if ref.get("color"):
        c = [o for o in c if (o.get("color") or "") == ref["color"]]
    if ref.get("in_room"):
        c = [o for o in c if world.room_of(o["x"], o["y"]) == ref["in_room"]]
    return c


def _unit(dx, dy):
    n = math.hypot(dx, dy) or 1.0
    return dx / n, dy / n


def _relation_point(rel, objs, prev, world):
    """Point for a relation, seen from the walking direction prev -> landmark (left = the walker's left)."""
    if rel == "between":
        a, b = objs
        cands = [((a["x"] + b["x"]) / 2, (a["y"] + b["y"]) / 2)]
    else:
        L = objs[0]
        ux, uy = _unit(L["x"] - prev[0], L["y"] - prev[1])
        lx, ly = -uy, ux
        dirs = {"left_of": (lx, ly), "right_of": (-lx, -ly), "behind": (ux, uy), "in_front_of": (-ux, -uy), "next_to": (-ux, -uy)}
        dx, dy = dirs[rel]
        cands = [(L["x"] + dx * k, L["y"] + dy * k) for k in (OFFSET, 0.55, 0.85)]
        if rel == "next_to":                             # any free side, nearest to the walker first
            cands += [(L["x"] + sx * OFFSET, L["y"] + sy * OFFSET) for sx, sy in ((lx, ly), (-lx, -ly), (ux, uy))]
    ignore = {o["id"] for o in objs}
    sig = [o.get("sigma", 0.0) for o in objs]
    ref = math.hypot(*sig) / 2 if rel == "between" else sig[0]          # the midpoint of two noisy points
    for x, y in cands:
        if world.free(x, y, ignore=ignore, ref_sigma=ref) and all(math.hypot(x - o["x"], y - o["y"]) >= landmark_clearance(o, world) for o in objs):
            return x, y
    return None


def room_point(room, world) -> list:
    """'Into room D': the free point closest to the room centre (the centre itself may be occupied: a chair next to it), as a
    one-element list; [] if the room has no free spot."""
    cx, cy = world.centre(room)
    pts = [(cx, cy)] + [(cx + r * math.cos(t), cy + r * math.sin(t)) for r in (0.3, 0.5, 0.75, 1.0, 1.25)
                        for t in [2 * math.pi * i / 12 for i in range(12)]]
    for x, y in pts:
        if world.room_of(x, y) == room and world.free(x, y):
            return [(x, y)]
    return []


def landmark_clearance(o, world) -> float:
    """How far a stop / pass / relation point must be from the landmark it is computed from: the object margin plus the
    world's margin plus sigma_k x the landmark's own position sigma (it may really be that much closer)."""
    return 0.4 + world.margin + world.sigma_k * o.get("sigma", 0.0)


def validate(raw, text: str, world: VLNWorld) -> Plan:
    d = _json(raw)
    if d is None:
        return _reject("BAD_JSON")
    if not schema_ok(d):
        return _reject("BAD_SCHEMA")
    d = json.loads(json.dumps(d))                         # never modify the caller's object
    if d["kind"] == "none":
        return _reject("NOT_NAV")
    # ---- every word must be the user's
    said = mentions(text)
    for r in _refs(d):
        m = [s for s in said if s["class"] == r["class"]]
        if not m:
            return _reject("UNSAID", f"class {r['class']}")
        if r.get("color") and not any(s.get("color") == r["color"] for s in m):
            return _reject("UNSAID", f"colour {r['color']}")
        if not r.get("color"):                           # the user's own colour word wins (the model may drop it)
            cols = {s.get("color") for s in m if s.get("color")}
            if len(cols) == 1 and len(m) == 1:
                r["color"] = cols.pop()
    typed_rooms = set(rooms_said(text, world))
    for name in _rooms_in(d):
        c = _canon_room(name, world)
        if c is None:                                     # 'room E' the user said -> answered below; any other word -> not ours
            if not re.fullmatch(r"\s*room\s+[a-z]\s*", name.lower()) or f"room {name.strip().split()[-1].upper()}" not in typed_rooms:
                return _reject("UNSAID", f"room {name}")
        elif c not in typed_rooms:
            return _reject("UNSAID", f"room {name}")
    # ---- and every thing the user named must be in the plan (a dropped "past the chair" is a silently different walk)
    outside = avoid_re(world.names).sub(" ", text)
    if len(list(_refs(d))) < len(mentions(outside)):
        return _reject("DROPPED", "object")
    if len(list(_rooms_in(d))) < len(rooms_said(outside, world)):
        return _reject("DROPPED", "room")
    for name in typed_rooms | set(_rooms_in(d)):
        if _canon_room(name, world) is None:
            return _answer("UNKNOWN_ROOM", f"There is no {name} on this map. The rooms are {world.rooms_text()}.")
    for s in d["steps"]:
        t = s["target"]
        if "room" in t:
            t["room"] = _canon_room(t["room"], world)
    for r in _refs(d):
        if r.get("in_room"):
            r["in_room"] = _canon_room(r["in_room"], world)
    avoid = {_canon_room(a, world) for a in d.get("avoid", [])}
    if None in avoid:
        return _answer("UNKNOWN_ROOM", f"There is no such room on this map. The rooms are {world.rooms_text()}.")
    want = {_canon_room(a, world) or a for a in avoid_said(text, world)}
    if want - avoid:
        return _reject("AVOID_DROPPED", ", ".join(sorted(want - avoid)))
    if avoid - want:
        return _reject("AVOID_ADDED", ", ".join(sorted(avoid - want)))
    # ---- ground every step first (anchors), then place the points with a look-ahead and route through the doors
    rx, ry, _ = world.robot
    start_room = world.room_of(rx, ry) or min(world.rooms, key=lambda r: math.dist(world.centre(r), (rx, ry)))   # in a doorway: nearest
    steps = []                                            # (action, kind, room, label, objs, relation)
    for s in d["steps"]:
        t, a = s["target"], s["action"]
        if "room" in t:
            steps.append((a, "room", t["room"], world.room_label(t["room"]), [], None))
            continue
        if "room_with" in t:
            m = ground(t["room_with"], world)
            if not m:
                return _not_found(t["room_with"])
            rooms = sorted({world.room_of(o["x"], o["y"]) for o in m})
            if len(rooms) > 1:
                return _answer("AMBIGUOUS", f"I know {describe_ref(t['room_with'])} in more than one room: {' and '.join(map(world.room_label, rooms))}. Which room?", rooms)
            steps.append((a, "room", rooms[0], f"{world.room_label(rooms[0])} (with {describe_ref(t['room_with'])})", [m[0]], None))
            continue
        objs = []
        for ref in ([t["object"]] if "object" in t else t["of"]):
            m = ground(ref, world)
            if not m:
                return _not_found(ref)
            if len(m) > 1:
                opts = [_obj_label(o, world) for o in m]
                return _answer("AMBIGUOUS", f"I know more than one {ref['class'].replace('_', ' ')}: {' or '.join(opts)}. Which one?", opts)
            objs.append(m[0])
        rel = t.get("relation")
        label = _obj_label(objs[0], world) if rel is None else f"{rel.replace('_', ' ')} {' and '.join(_obj_label(o, world) for o in objs)}"
        steps.append((a, "point", world.room_of(objs[0]["x"], objs[0]["y"]), label, objs, rel))
    for a, kind, room, label, objs, rel in steps:
        if room in avoid:
            return _answer("IMPOSSIBLE", f"You asked me to avoid {world.room_label(room)}, but {label} is in {world.room_label(room)}.")
    def _anchor(st):
        if st[1] == "room":
            return world.centre(st[2])
        objs = st[4]
        return (sum(o["x"] for o in objs) / len(objs), sum(o["y"] for o in objs) / len(objs))   # "between": the midpoint
    anchors = [_anchor(st) for st in steps]
    wps, rooms_seq, notes = [], [start_room], []
    if start_room in avoid:
        notes.append(f"I'm in {world.room_label(start_room)} now; I'll leave it the shortest way.")
    cur, pos, length = start_room, (rx, ry), 0.0
    for k, (a, kind, room, label, objs, rel) in enumerate(steps):
        nxt = anchors[k + 1] if k + 1 < len(steps) else None
        best = None
        routes = world.room_routes(cur, room, avoid)
        anchor = anchors[k]
        routes.sort(key=lambda rd: sum(math.dist(p, q) for p, q in zip([pos, *[world.doors[dn] for dn in rd[1]], anchor],
                                                                         [*[world.doors[dn] for dn in rd[1]], anchor])))
        if rel is not None:
            routes = routes[:1]           # left / right / behind are seen from the SHORTEST way in; no detour to make them fit
        for rooms, doors in routes:
            pts = [world.doors[dn] for dn in doors]
            entry = pts[-1] if pts else pos               # the robot comes into the room from here
            if kind == "room":
                cand = room_point(room, world)
            elif rel is not None:
                rp = _relation_point(rel, objs, entry, world)
                cand = [rp] if rp else []
            else:
                L = objs[0]
                ux, uy = _unit(L["x"] - entry[0], L["y"] - entry[1])
                if a == "pass":                           # any free point close round it; the whole-route length picks the side
                    raw = [(L["x"] + kk * math.cos(t_), L["y"] + kk * math.sin(t_)) for kk in (0.6, 0.8, 1.0)
                           for t_ in [2 * math.pi * i / 16 for i in range(16)]]
                else:
                    raw = [(L["x"] - ux * kk, L["y"] - uy * kk) for kk in (STOP_STANDOFF, 0.5, 0.8, 1.0)]
                ok_ = lambda x, y: (world.free(x, y, ignore={L["id"]}, ref_sigma=L.get("sigma", 0.0)) and world.room_of(x, y) == room   # noqa: E731
                                    and math.hypot(x - L["x"], y - L["y"]) >= landmark_clearance(L, world))
                cand = [(x, y) for x, y in raw if ok_(x, y)]
                if a != "pass" and not cand:          # "stop near X" needs no particular side: the free side nearest the approach
                    ring = [(L["x"] + kk * math.cos(t_), L["y"] + kk * math.sin(t_)) for kk in (STOP_STANDOFF, 0.8, 1.0)
                            for t_ in [2 * math.pi * i / 16 for i in range(16)]]
                    ring = sorted((q for q in ring if ok_(*q)), key=lambda q: (round(math.dist(q, (L["x"], L["y"])), 2), math.dist(q, entry)))
                    cand = ring[:1]
            for c in cand:
                seq = [pos, *pts, c]
                ln = sum(math.dist(p, q) for p, q in zip(seq, seq[1:])) + (math.dist(c, nxt) if nxt else 0.0)
                if best is None or ln < best[0] - 1e-9:
                    best = (ln, rooms, doors, pts, c)
        if best is None:
            if not world.room_routes(cur, room, avoid):
                if avoid:
                    return _answer("IMPOSSIBLE", f"I can't reach {world.room_label(room)} without going through {' or '.join(map(world.room_label, sorted(avoid)))}.")
                return _answer("IMPOSSIBLE", f"I don't know a way from {world.room_label(cur)} into {world.room_label(room)} on this map.")
            where = label if rel else f"{'beside' if a == 'pass' else 'next to'} {label}"
            return _answer("NO_FREE_SPOT", f"There is no free spot {where} where I fit.")
        _, rooms, doors, pts, c = best
        for dn, (dx, dy) in zip(doors, pts):
            wps.append({"x": dx, "y": dy, "kind": "door", "room": None, "label": f"door {dn}"})
            length += math.dist(pos, (dx, dy))
            pos = (dx, dy)
        L = objs[0] if objs else None
        wps.append({"x": round(c[0], 3), "y": round(c[1], 3), "kind": a, "room": room, "label": label,
                    **({"landmark": _lm(L, world)} if L and kind == "point" else {})})
        length += math.dist(pos, c)
        pos, cur = c, room
        rooms_seq += rooms[1:]
    targets = [(st[0], wps[-1]["x"], wps[-1]["y"], st[2], st[3], None, st[1]) for st in steps[-1:]]
    last = targets[-1]
    goal = {"kind": "room", "room": last[3]} if last[6] == "room" else \
        {"kind": "point", "x": round(last[1], 3), "y": round(last[2], 3), "radius": GOAL_RADIUS, "room": last[3]}
    checks = [{"type": "pass_near", "landmark": w["landmark"], "within_m": PASS_CHECK} for w in wps if w["kind"] == "pass" and "landmark" in w]
    checks += [{"type": "room_order", "rooms": rooms_seq}, {"type": "never_in", "rooms": sorted(avoid)}]
    summary = "; ".join(("go past " if w["kind"] == "pass" else "through " if w["kind"] == "door" else
                         "into " if w["kind"] == "enter" else
                         "stop " if w["label"].split(" ", 1)[0] in ("left", "right", "next", "behind", "in", "between") else "stop at ") + w["label"] for w in wps)
    gl = steps[-1][4][0] if steps[-1][1] == "room" and steps[-1][4] else None
    return Plan(True, waypoints=wps, goal=goal, rooms=rooms_seq, avoid=sorted(avoid), checks=checks, length_m=round(length, 2),
                summary=summary, notes=notes, goal_landmark=_lm(gl, world) if gl else None)


def _lm(o, world):
    return {"id": o["id"], "class": o["class"], "color": o.get("color") or "", "x": o["x"], "y": o["y"], "label": _obj_label(o, world)}


def _not_found(ref):
    return _answer("LANDMARK_NOT_FOUND", f"I haven't seen {describe_ref(ref)} yet, so I can't use it as a landmark. "
                                         "Shall I explore first?")
