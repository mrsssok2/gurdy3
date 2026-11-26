"""Fetch safety rules (checked BEFORE anything moves; the mock / robot never sees a refused request).

1. only the five small pickable classes; everything else is refused with the reason (too big / too heavy)
2. mass <= 0.08 kg (catalog masses; an unknown heavier object is refused)
3. never grasp, lift, carry or hand over a person (any people word as the object, 'pick me up', 'carry her' ...)
4. no throwing / tossing / dropping things on purpose; the offer is to bring it instead
5. carry speed limit 0.08 m/s for every motion while an object is held
"""
from __future__ import annotations

import re

from .catalog import CARRY_SPEED_MPS, MAX_GRIP_WIDTH_M, MAX_PAYLOAD_KG, PICKABLE, TOO_BIG, label

THROW_RE = re.compile(r"\b(throw|throws|throwing|toss|tossing|hurl|fling|chuck|lob|yeet)\b", re.I)
DROP_ON_PURPOSE_RE = re.compile(r"\bdrop (it|that|the \w+(?: \w+)?) (on|onto|at) (me|him|her|them|someone|somebody|the (?:person|mannequin))\b", re.I)
PERSON_OBJECT_RE = re.compile(
    r"\b(pick|lift|carry|grab|grasp|hold|take|bring|fetch|get|move)\b(?:\s+\w+){0,3}?\s+"
    r"(?:the |a |an |that |this |my )?(person|people|human|man|woman|child|kid|baby|boy|girl|mannequin|dummy|somebody|someone|user|operator)\b"
    r"|\b(pick|lift|carry|grab|grasp|hold)\s+(me|him|her|them|you|yourself)\b", re.I)


def refuse_reason(text: str) -> tuple[str, str] | None:
    """(code, reply) if the utterance itself is never allowed, else None."""
    if THROW_RE.search(text) or DROP_ON_PURPOSE_RE.search(text):
        return "THROW", "I don't throw or drop things at anyone. I can bring it to you and hand it over instead."
    if PERSON_OBJECT_RE.search(text):
        return "PERSON", "I never pick up, carry or grab people."
    return None


def object_allowed(cls: str, mass: float | None = None) -> tuple[str, str] | None:
    """(code, reply) if this object class / mass may not be picked up, else None."""
    if cls == "mannequin":
        return "PERSON", "That is a mannequin. I don't pick up people or person-shaped things."
    if cls in TOO_BIG:
        w, m, _ = TOO_BIG[cls]
        why = []
        if w > MAX_GRIP_WIDTH_M:
            why.append(f"its narrowest side is about {w * 100:.0f} cm and my gripper opens {MAX_GRIP_WIDTH_M * 100:.0f} cm")
        if m > MAX_PAYLOAD_KG:
            why.append(f"it weighs about {m:.0f} kg and I can carry {MAX_PAYLOAD_KG * 1000:.0f} g")
        return "TOO_BIG", f"The {label(cls)} is too big for me to pick up: " + " and ".join(why) + "."
    if cls not in PICKABLE:
        return "TOO_BIG", f"I can't pick up a {label(cls)}."
    m = PICKABLE[cls][0] if mass is None else mass
    if m > MAX_PAYLOAD_KG + 1e-9:
        return "TOO_HEAVY", f"The {label(cls)} weighs {m * 1000:.0f} g; I can carry at most {MAX_PAYLOAD_KG * 1000:.0f} g."
    return None


def carry_speed(requested: float, carrying: bool) -> float:
    """Speed limit for any motion: while carrying, never above CARRY_SPEED_MPS."""
    return min(requested, CARRY_SPEED_MPS) if carrying else requested
