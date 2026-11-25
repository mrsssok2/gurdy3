"""Deterministic guard between the LLM's plan and the safety checker (OOD fixes, docs/results/phase5_eval_ood.md).

The planner is untrusted text. This module runs on every plan, for every LLM (fine-tuned, base, scripted):

1. tool aliases      "continue" / "carry_on" / ... -> mission_control(resume); other unknown tool names -> "I can't do that".
2. state changes     save_place and no_go_zone add/remove only when the user ASKED for it (explicit cue words, typo tolerant).
                     Otherwise the call is dropped; if it was the whole plan, the robot asks first (confirmation gate).
3. unknown objects   a class the robot does not have (table, cylinder, ...) is never replaced by a similar known one.
4. substitution      an object class in a call that the user never named (no word like it, no "it/that one", not the focus
                     object) is a guess: ask instead.

`unknown_noun(text)` is the pre-LLM check (graph.n_route): everyday nouns / rooms the robot has no concept of -> say so.
Everything here is regex + word lists, no LLM, no state outside the tool layer.
"""
from __future__ import annotations

import difflib
from difflib import SequenceMatcher
import re
from dataclasses import dataclass, field
from typing import Any

from ..tools import TOOLS
from ..tools.specs import _CLASSES

# ---------------------------------------------------------------- 1. tool aliases
_RESUME_ALIASES = {"continue", "resume", "carry_on", "carryon", "keep_going", "go_on", "proceed", "unpause", "continue_mission",
                   "resume_mission", "resume_task", "continue_task"}
_PAUSE_ALIASES = {"pause_mission", "hold", "wait", "pause_task"}
_CANCEL_ALIASES = {"cancel", "abort", "cancel_mission", "abort_mission", "cancel_task"}


def normalize_call(call: dict) -> dict:
    """Map an alias tool name to the real tool. Unknown names are returned unchanged (the guard refuses them)."""
    tool = call.get("tool")
    if not isinstance(tool, str) or tool in TOOLS:
        return call
    key = re.sub(r"[^a-z_]", "", tool.lower().replace("-", "_").replace(" ", "_"))
    if key in _RESUME_ALIASES:
        return {"tool": "mission_control", "args": {"action": "resume"}}
    if key in _PAUSE_ALIASES:
        return {"tool": "mission_control", "args": {"action": "pause"}}
    if key in _CANCEL_ALIASES:
        return {"tool": "mission_control", "args": {"action": "cancel"}}
    return call


# ---------------------------------------------------------------- word lists
# nouns the robot has no concept of, as objects (not in the 7 classes) -- everyday things people mention
UNKNOWN_OBJECT_NOUNS = {
    "table", "desk", "cylinder", "sphere", "ball", "cube", "wall", "walls", "window", "door", "couch", "sofa", "bed", "shelf", "bookshelf",
    "cabinet", "cupboard", "lamp", "light", "tv", "television", "computer", "laptop", "monitor", "keyboard", "phone", "cup", "mug", "bottle",
    "plate", "bowl", "book", "bag", "backpack", "plant", "tree", "flower", "carpet", "rug", "stairs", "staircase", "ladder", "fridge",
    "refrigerator", "stove", "sink", "toilet", "car", "bike", "bicycle", "dog", "cat", "person", "people", "human", "man", "woman",
    "child", "kid", "stool", "bench", "pillar", "column", "pole", "rock", "stone", "wheel", "tire", "tyre", "key", "keys", "pen", "pencil",
    "paper", "screwdriver", "hammer", "drill", "wrench", "battery pack", "charger", "cable", "wire", "pipe", "bucket", "mop", "broom",
    "ceiling", "floor", "curtain", "mirror", "clock", "painting", "picture", "sign", "pallet", "crate pile", "trolley", "cart", "robot",
    "drone", "ramp", "step", "steps", "puddle", "hole", "snake", "spider", "mouse", "bird",
}
# rooms / places the robot does not know (anything not in KNOWN PLACES)
UNKNOWN_PLACE_NOUNS = {"kitchen", "bathroom", "bedroom", "toilet", "garage", "office", "corridor", "hallway", "basement", "balcony", "roof",
                       "garden", "yard", "attic", "lobby", "classroom", "library", "cafeteria", "canteen", "lounge", "porch", "terrace",
                       "staircase", "entrance", "exit", "outside", "upstairs", "downstairs", "workshop", "warehouse", "store room", "storeroom"}
# 'person' / 'people' / 'man' etc. are handled by the mannequin stand-in rules, 'door' by the doorway place, 'robot' is the robot itself:
_NOT_UNKNOWN = {"person", "people", "human", "man", "woman", "child", "kid", "door", "robot", "light", "step", "steps", "key", "keys", "sign",
                "mouse", "hole", "floor", "ceiling", "ramp", "wall", "walls"}
# these are exempt from the PRE-LLM noun check (they carry safety meaning or are common verbs / idioms and are better judged by the planner)
PRE_CHECK_NOUNS = (UNKNOWN_OBJECT_NOUNS - _NOT_UNKNOWN) | UNKNOWN_PLACE_NOUNS

_CLASS_WORDS = ["box", "crate", "barrel", "drum", "extinguisher", "fire", "cone", "traffic", "toolbox", "tool", "mannequin", "dummy", "chair"]
_ANAPHORA = {"it", "that", "this", "those", "these", "them", "one", "ones", "other", "another", "same", "again", "there", "previous",
             "last", "first", "second", "former", "latter", "him", "her"}

_SAVE_VERBS = ("save", "remember", "memorize", "memorise", "store", "bookmark", "mark", "label", "name", "call", "note", "register")
_SAVE_NOUNS = ("place", "spot", "location", "position", "here", "pose", "point", "this", "waypoint", "landmark")
_ZONE_CUES = re.compile(
    r"\b(avoid|keep (?:out|away|off|clear)|stay (?:out|away|off|clear)|steer clear|no[- ]?go|off[- ]?limits?|forbid\w*|prohibit\w*|restrict\w*|"
    r"block(?:ed)?(?: off)?|ban(?:ned)?|danger\w*|not allowed|no entry|never|"
    r"(?:do not|dont|don't|do nt|dnt|cannot|can't|must not|mustn't|should not|shouldn't) (?:go|enter|walk|drive|come|step|cross|move|approach|get)|"
    r"allow|unblock|un-block|remove|delete|clear|lift|undo|re-?enable|reopen|you may|you can (?:go|enter)|is fine again|safe again|zone)\b", re.I)
_TOKEN = re.compile(r"[a-z']+")


def _tokens(text: str) -> list[str]:
    return _TOKEN.findall(text.lower())


def _close(tok: str, words, cut: float = 0.78) -> bool:
    if tok in words:
        return True
    if len(tok) < 3:
        return False
    return any(difflib.SequenceMatcher(None, tok, w).ratio() >= cut for w in words)


def has_save_cue(text: str) -> bool:
    toks = _tokens(text)
    verb_at = [i for i, t in enumerate(toks) if _close(t, _SAVE_VERBS, 0.8)]
    noun_at = [i for i, t in enumerate(toks) if _close(t, _SAVE_NOUNS, 0.8)]
    return any(abs(i - j) <= 6 for i in verb_at for j in noun_at if i != j)


def has_zone_cue(text: str) -> bool:
    if _ZONE_CUES.search(text):
        return True
    toks = _tokens(text)
    return any(_close(t, ("avoid", "forbidden", "restricted", "blocked"), 0.82) for t in toks)


def known_class_words() -> list[str]:
    return _CLASS_WORDS


def mentions_class(text: str, cls: str) -> bool:
    """Does the user's text name this class (or a typo / speech-recognition variant of one of its words)?"""
    from ..data_gen.lexicon import CONFUSIONS, OBJECT_WORDS
    words = {w for phrase in OBJECT_WORDS.get(cls, []) for w in phrase.split()} | set(cls.split("_"))
    toks = _tokens(text)
    conf = {k for k, v in CONFUSIONS.items() if v in words or v == cls}
    return any(_close(t, words, 0.7) or t in conf or (len(t) <= 6 and t[:1] == next((w[:1] for w in words if SequenceMatcher(None, t, w).ratio() >= 0.5), "?"))
               for t in toks)


_ROOM_WORDS = {"room", "rooms", "area", "areas", "alpha", "bravo", "charlie", "delta"}


def _is_room_word(w: str) -> bool:
    """'room', 'room B', 'B', 'Alpha' ... : names of rooms are places, never unknown object nouns."""
    t = w.strip().lower().replace("_", " ")
    return t in _ROOM_WORDS or bool(re.fullmatch(r"(?:room\s+)?[a-h]|room\s+(?:alpha|bravo|charlie|delta)", t))


def unknown_noun(text: str, known_places: list[str] | None = None) -> str | None:
    """The first noun in `text` that is neither an object class nor a known place, or None. Pre-LLM check."""
    low = text.lower()
    known = {p.lower() for p in (known_places or [])}
    toks = _tokens(low)
    # naming a new place legitimately uses such words ("call this place garage")
    if has_save_cue(text):
        return None
    for i, t in enumerate(toks):
        if _is_room_word(t):
            continue
        if t in PRE_CHECK_NOUNS and t not in known and not any(t in k.split() for k in known):
            return t
    for noun in PRE_CHECK_NOUNS:
        if " " in noun and noun in low:
            return noun
    return None


# ---------------------------------------------------------------- guard
@dataclass
class GuardResult:
    calls: list[dict]
    reply: str | None = None                       # set: nothing is planned, say this (refuse / decline)
    notes: list[str] = field(default_factory=list)   # appended to the reply after the remaining calls ran
    confirm: tuple[str, dict, str] | None = None   # (tool, {"calls": [...]}, question): hold in the confirmation gate
    clarify: str | None = None                     # ask this question instead of acting
    rule: str = ""
    changed: bool = False


def _object_refs(call: dict) -> list[dict]:
    a = call.get("args") if isinstance(call.get("args"), dict) else {}
    out = []
    for k in ("object", "a", "b"):
        if isinstance(a.get(k), dict):
            out.append(a[k])
    q = a.get("query")
    if isinstance(q, dict) and "class" in q:
        out.append(q)
    return out


def _is_state_change(call: dict) -> str | None:
    t, a = call.get("tool"), call.get("args") if isinstance(call.get("args"), dict) else {}
    if t == "schedule" and isinstance(a.get("call"), dict):
        return _is_state_change({"tool": a["call"].get("tool"), "args": a["call"].get("args")})
    if t == "save_place":
        return "save"
    if t == "no_go_zone" and a.get("action") in ("add", "remove"):
        return "zone"
    return None


def guard_plan(utterance: str, calls: list[dict], focus: dict | None = None, recall=None,
               allow_state_changes: bool = False) -> GuardResult:
    """Check and repair a plan. `recall(color) -> set of classes` of seen objects with that colour (may be None)."""
    calls = [normalize_call(c) if isinstance(c, dict) else c for c in calls]
    if any(not isinstance(c, dict) or c.get("tool") not in TOOLS for c in calls):
        return GuardResult([], reply="I can't do that. I don't have a way to do it.", rule="unknown_tool", changed=True)

    # 3. unknown object classes: never substitute
    for c in calls:
        for ref in _object_refs(c):
            cls = ref.get("class")
            if cls is not None and cls not in _CLASSES and _is_room_word(str(cls)):
                named = re.search(r"\broom\s+([A-Za-z])\b", utterance)
                if named:                                                     # the user DID name a room: say what can be asked about it, never ask which one
                    reply = (f"I can tell you what changed in room {named.group(1).upper()}, when I was last there, or what happened there. "
                             "Which of those do you want?")
                else:
                    reply = "That sounds like a question about a room. Which room do you mean? Please name one, for example room A."
                return GuardResult([], reply=reply, rule="room_not_object", changed=True)      # a room is never an unknown object
            if cls is not None and cls not in _CLASSES:
                noun = str(cls).replace("_", " ")
                say = noun if noun.lower() in utterance.lower() else "that"
                reply = (f"I can't do that. I don't know what {('a ' + noun) if say == noun else 'that'} is, and I have not seen one."
                         if say != "that" else "I can't do that. I don't know that kind of object, and I have not seen one.")
                return GuardResult([], reply=reply, rule="unknown_object", changed=True)

    # 2. state changes need an explicit request
    kept, dropped = [], []
    if not allow_state_changes:
        for c in calls:
            kind = _is_state_change(c)
            ok = kind is None or (kind == "save" and has_save_cue(utterance)) or (kind == "zone" and has_zone_cue(utterance))
            (kept if ok else dropped).append(c)
    else:
        kept = list(calls)
    if dropped:
        if not kept:
            d = dropped[0]
            if _is_state_change(d) == "save":
                a = d.get("args", {})
                q = f"I did not save anything. Do you want me to save this place as {a['name']}?" if isinstance(a.get("name"), str) and a["name"] in utterance.lower() else \
                    "I did not save anything. If you want a place saved, say for example: save this place as workbench."
                if "Do you want" in q:
                    return GuardResult([], confirm=("save_place", {"calls": [d]}, q), rule="state_change_needs_request", changed=True)
                return GuardResult([], reply=q, rule="state_change_needs_request", changed=True)
            return GuardResult([], reply="I did not change any no-go zone. If you want one, say for example: keep out of room B.",
                               rule="state_change_needs_request", changed=True)
        note = ("I did not save a place, because you did not ask for that." if _is_state_change(dropped[0]) == "save"
                else "I did not change any no-go zone, because you did not ask for that.")
        return _substitution(utterance, GuardResult(kept, notes=[note], rule="state_change_needs_request", changed=True), focus, recall)
    return _substitution(utterance, GuardResult(calls, changed=any(a is not b for a, b in zip(calls, calls))), focus, recall)


def _substitution(utterance: str, res: GuardResult, focus: dict | None, recall) -> GuardResult:
    toks = set(_tokens(utterance))
    if toks & _ANAPHORA:
        return res
    colours = [t for t in toks if t in {"red", "blue", "green", "yellow", "orange", "white", "black", "brown", "grey", "gray"}]
    for c in res.calls:
        for ref in _object_refs(c):
            cls = ref.get("class")
            if cls is None or mentions_class(utterance, cls):
                continue
            if focus and focus.get("object") and focus["object"].get("class") == cls:
                continue
            if colours and recall is not None:
                classes = recall(colours[0])
                if classes == {cls}:
                    continue
            return GuardResult([], clarify="Which kind of object do you mean?", rule="object_not_named", changed=True)
    return res
