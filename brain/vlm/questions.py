"""Scene questions -> a kind + a target, deterministically (no model). None = not a scene question (the normal routing goes on).
  see     "what do you see?", "what's in front of you?", "describe what you see"
  on      "what's on the table?", "what is next to the chair?"          target surface noun (+ class if it is one)
  colour  "what colour is the box near the barrel?"                     target class (+ near-reference class)
  path    "is the path to the kitchen clear?", "is the way to room B blocked?"   target place words
  count   "how many cones can you see?"
"""
from __future__ import annotations

import re

from .vocab import to_class

SEE = re.compile(r"\b(what (?:do|can) you see|what(?:'s| is) (?:in front of you|ahead|around you|there)|describe (?:what you see|the (?:view|scene))|"
                 r"what are you looking at|look around and tell me)\b", re.I)
ON = re.compile(r"\bwhat(?:'s| is| are)? (on|under|next to|beside|behind|near|in front of) (?:the |that |this )?([a-z ]+?)\s*\??$", re.I)
COLOUR = re.compile(r"\bwhat colou?r is (?:the |that |this )?([a-z ]+?)(?: (?:near|next to|beside|by) (?:the |that )?([a-z ]+?))?\s*\??$", re.I)
PATH = re.compile(r"\b(?:is|are) (?:the )?(?:path|way|route|corridor|doorway)(?: to(?:wards)? (?:the )?([a-z ]+?))? (clear|free|blocked|open|obstructed)\b", re.I)
COUNT = re.compile(r"\bhow many ([a-z ]+?) (?:can you see|do you see|are (?:there|in front of you|in view))\b", re.I)


def classify(text: str) -> dict | None:
    t = text.strip()
    if m := PATH.search(t):
        return {"kind": "path", "place": (m.group(1) or "").strip() or None, "asked": m.group(2).lower()}
    if m := COLOUR.search(t):
        cls, col = to_class(m.group(1))
        near = to_class(m.group(2))[0] if m.group(2) else None
        return {"kind": "colour", "cls": cls, "noun": m.group(1).strip(), "near": near, "near_noun": (m.group(2) or "").strip() or None}
    if m := COUNT.search(t):
        cls, col = to_class(m.group(1))
        return {"kind": "count", "cls": cls, "color": col, "noun": m.group(1).strip()}
    if m := ON.search(t):
        cls, _ = to_class(m.group(2))
        return {"kind": "on", "relation": m.group(1).lower(), "surface": m.group(2).strip(), "surface_cls": cls}
    if SEE.search(t):
        return {"kind": "see"}
    return None
