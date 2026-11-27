"""FetchSession: utterance -> (router) intent -> object resolution + safety -> validated tool calls -> state machine ->
grounded template reply. Keeps a pending clarification ('which can?') for the next turn.

    s = FetchSession(FetchMock.lab4_fetch()); s.handle("bring me the red can")  # -> "Here you are: the red can."
Wiring into the agent graph (like Agent._incident_route): call `handle` first when the backend offers a fetch session;
None = not a fetch command, the planner handles the text as before.
"""
from __future__ import annotations

import re

from .catalog import COLOURS, label
from .machine import FetchAborted, FetchMachine, Outcome
from .mock import ROOMS, room_of
from .replies import describe, outcome_reply, safe
from .router import find_object, parse
from .safety import object_allowed, refuse_reason
from .specs import check

# generic words that also name a small pickable class ("cone" = traffic cone or mini cone)
FAMILY = {"traffic_cone": "mini_cone", "fire_extinguisher": "mini_extinguisher", "toolbox": "mini_toolbox", "box": "small_box"}
SIZE_WORDS = re.compile(r"\b(big|large|huge|tall|traffic|fire|full[- ]size)\b", re.I)


class FetchSession:
    def __init__(self, robot, picker=None):
        self.r = robot
        self.picker = picker           # Phase 9f: brain.fetch.picker.FetchPicker for free phrasings (None = router only)
        self.m = FetchMachine(robot)
        self.pending = None            # (intent, candidates) waiting for 'the red one' / 'the one in room C'
        self.queue: list = []          # remaining items of a multi-object request ("the red can and the blue cube"), in order
        self.calls: list = []          # validated tool calls of the last turn
        self.last: Outcome | None = None

    # ---- resolution
    def _candidates(self, obj: dict, classes=None):
        classes = classes or {obj["class"]}
        c = [o for o in self.r.objects.values() if o.cls in classes and o.name in self.r.known and not o.with_user]
        if obj.get("color"):
            c = [o for o in c if o.color == obj["color"]]
        if obj.get("room"):
            c = [o for o in c if o.room == obj["room"]]
        return c

    def _clarify(self, intent, cands):
        self.pending = (intent, cands)
        opts = " or ".join(f"{describe(o)} in {o.room}" for o in cands)
        noun = label(cands[0].cls) if len({o.cls for o in cands}) == 1 else "of those"
        return f"I know more than one {noun}: {opts}. Which one?"

    def _resolve_pending(self, text):
        intent, cands = self.pending
        t = text.lower()
        o = find_object(text)
        narrowed = False
        if o and any(c.cls == o["class"] for c in cands):           # "the mini cone" / "the traffic cone"
            narrowed = len([c for c in cands if c.cls == o["class"]]) < len(cands)
            cands = [c for c in cands if c.cls == o["class"]]
            if len(cands) == 1 and not re.search(r"\b(red|blue|green|yellow|orange|room)\b", t):
                self.pending = None
                return intent, cands[0]
        col = next((c for c in COLOURS if re.search(rf"\b{c}\b", t)), None)
        m = re.search(r"\broom\s+([a-d])\b", t)
        room = f"room {m.group(1).upper()}" if m else None
        pick = [o for o in cands if (col is None or o.color == col) and (room is None or o.room == room)]
        if (col or room) and len(pick) == 1:
            self.pending = None
            return intent, pick[0]
        if (narrowed or col or room) and len(pick) > 1:
            return "ask", pick                                       # still ambiguous: ask again with fewer options
        return None

    # ---- tool calls (strict specs)
    def _call(self, tool, args):
        probs = check(tool, args)
        if probs:
            raise ValueError("; ".join(probs))
        self.calls.append({"tool": tool, "args": args})

    def handle(self, text: str) -> str | None:
        self.calls = []
        self._requester_noted = False
        chosen = None
        if self.pending:
            res = self._resolve_pending(text)
            if res and res[0] == "ask":
                return self._clarify(self.pending[0], res[1])
            if res:
                intent, chosen = res
            else:
                self.pending = None
                self.queue = []                        # a new request replaces the rest of an interrupted list
                intent = parse(text)
        else:
            self.queue = []
            intent = parse(text)
        if intent is None and self.picker is not None:
            ref = refuse_reason(text)                  # text-level safety BEFORE any model sees the request
            if ref:
                return ref[1]
            intent = self.picker.pick(text, sorted(ROOMS))
        if intent is None:
            return None
        if intent["intent"] == "multi_pick":           # one gripper: holding two things is impossible -> ask, never guess
            names = " or ".join(f"the {(o.get('color') + ' ') if o.get('color') else ''}{label(o['class'])}" for o in intent["objects"])
            return f"I can only hold one thing at a time. Which one should I pick up: {names}?"
        if intent["intent"] == "multi":                # several objects: one after another, in the order they were said
            self.queue = list(intent["items"][1:])
            intent = intent["items"][0]
        try:
            return self._run_queue(self._run(intent, chosen))
        except FetchAborted as e:                        # e-stop during a real-robot primitive: stop everything, say what is true
            self.queue, self.pending = [], None
            held = self.r.carrying
            what = f"I'm still holding {describe(self.r.objects[held])}." if held and held in self.r.objects else \
                "I'm not holding anything."
            return f"I stopped ({e}). {what}"

    # ---- multi-object requests: one after another; stop at the first item that does not end as asked
    def _run_queue(self, reply: str | None) -> str | None:
        parts = [reply]
        while self.queue and self.pending is None and self.last is not None and self.last.status in ("placed", "handed_over"):
            nxt = self.queue.pop(0)
            self.last = None
            parts.append(self._run(nxt, None))
        if self.queue and self.pending is None:
            rest = " and ".join(self._name(i["object"]) for i in self.queue)
            self.queue = []
            parts.append(f"I stopped there, so I haven't done {rest} yet.")
        return " ".join(p for p in parts if p)

    @staticmethod
    def _name(o: dict) -> str:
        return f"the {(o['color'] + ' ') if o.get('color') else ''}{label(o['class'])}"

    def _run(self, intent: dict, chosen) -> str | None:
        self.last = None
        if intent["intent"] == "refuse":
            return intent["reply"]
        if intent["intent"] == "hand_over":
            return self._run_deliver("user")
        if intent["intent"] == "deliver":
            mode = intent["mode"]
            if mode == "here":
                return self._run_deliver("room", room_of(*self.r.robot))
            return self._run_deliver(mode, intent.get("room"))
        obj = intent["object"]
        fam = FAMILY.get(obj["class"])
        if fam and not obj.get("explicit_size") and chosen is None:
            # "the orange cone": could be the small mini cone or a big traffic cone -> decide from what exists
            both = self._candidates(obj, {obj["class"], fam})
            small = [o for o in both if o.cls == fam]
            if small and len(small) == len(both):
                obj = {**obj, "class": fam}
            elif small:
                return self._clarify(intent, both)
        bad = object_allowed(obj["class"]) if chosen is None else None     # a chosen object is checked by its own class below
        if bad:
            return bad[1]
        if chosen is None:
            if intent.get("room") and intent["room"] not in ROOMS:
                return f"I don't know {intent['room']}."
            cands = self._candidates(obj)
            if not cands:
                # not in the map: the machine's FIND step searches (and may fail with NOT_FOUND)
                unseen = [o for o in self.r.objects.values() if o.cls == obj["class"] and o.name not in self.r.known
                          and (not obj.get("color") or o.color == obj["color"]) and (not obj.get("room") or o.room == obj["room"])]
                if not unseen and callable(getattr(self.r, "search", None)):
                    # real robot (brain/ros/fetch_robot.py): nobody has seen one yet -> search the rooms, then look again
                    ref = {"class": obj["class"], **({"color": obj["color"]} if obj.get("color") else {})}
                    if self.r.search(ref):
                        unseen = self._candidates(obj)
                if not unseen:
                    c = f"{obj['color']} " if obj.get("color") else ""
                    where = f" in {obj['room']}" if obj.get("room") else ""
                    return f"I don't know any {c}{label(obj['class'])}{where}, and I won't guess another object."
                cands = unseen[:1] if len(unseen) == 1 else unseen
            if len(cands) > 1:
                return self._clarify(intent, cands)
            chosen = cands[0]
        bad = object_allowed(chosen.cls, chosen.mass)
        if bad:
            return bad[1]
        args = {"object": {"class": chosen.cls, **({"color": chosen.color} if chosen.color in COLOURS else {})}}
        if chosen.room:
            args["room"] = chosen.room
        self._call("pick_up", args)
        if intent["intent"] == "pick":
            out = self.m.pick(chosen.name)
        else:
            mode = intent["mode"]
            if mode == "user":
                self._call("bring_to", {"target": "user"})
                self._call("hand_over", {})
                if callable(getattr(self.r, "note_requester", None)) and not getattr(self, "_requester_noted", False):
                    self._requester_noted = True      # once per request: a second item of "X and Y to me" goes to the same spot
                    self.r.note_requester()          # real robot, bring_me flag: come back to where the request came from
            elif mode == "home":
                self._call("bring_to", {"target": "home"})
                self._call("place", {"room": "home"})
            else:
                self._call("bring_to", {"target": "room", "room": intent["room"]})
                self._call("place", {"room": intent["room"]})
            out = self.m.fetch(chosen.name, mode, intent.get("room"))
        return self._reply(out)

    def _run_deliver(self, mode, room=None):
        if not self.r.carrying:
            return "I'm not holding anything."
        if mode == "room":
            if room not in ROOMS:
                return f"I don't know {room}."
            self._call("bring_to", {"target": "room", "room": room})
            self._call("place", {"room": room})
        elif mode == "home":
            self._call("bring_to", {"target": "home"})
            self._call("place", {"room": "home"})
        else:
            self._call("bring_to", {"target": "user"})
            self._call("hand_over", {})
        return self._reply(self.m.deliver(mode, room))

    def _reply(self, out: Outcome) -> str:
        self.last = out
        held = self.r.carrying is not None                 # verified by an earlier VERIFY_GRASP of this session
        confirmed = out.confirmed | ({"picked"} if held else set())
        text = getattr(self.r, "handover_text", "Here you are: {what}.")
        return safe(outcome_reply(out, self.r.objects, text), confirmed, held)
