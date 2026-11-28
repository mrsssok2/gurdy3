"""Template replies built ONLY from the machine's Outcome, plus the grounding check that every reply must pass:
a success claim ('picked up', 'got it', 'I'm holding', 'delivered', 'here you are', 'put it in', 'placed', 'handed')
is allowed only if the matching event is in Outcome.confirmed (and, for holding, the robot still holds it)."""
from __future__ import annotations

import re

from .catalog import label

CLAIMS = {
    "picked": re.compile(r"\b(picked (it |\w+ )?up|i('ve| have) got (it|the)|got it|i'm holding|i am holding|holding the)\b", re.I),
    "placed": re.compile(r"\b(put (it|the [\w ]+?) (down )?(in|at)|placed|left (it|the [\w ]+?) in|delivered)\b", re.I),
    "handed_over": re.compile(r"\b(here you are|here it is|here's your|here is your|handed|you have it|you've got it)\b", re.I),
}
NEG = re.compile(r"\b(not|n't|couldn't|could not|can't|cannot|can not|don't|won't|isn't|wasn't|failed|never|didn't|unable)\b", re.I)


def ground(reply: str, confirmed: set, holding: bool) -> list[str]:
    """Violations of the claim rule (empty = ok). Sentences with a negation are not claims."""
    v = []
    for sent in re.split(r"(?<=[.!?])\s+", reply):
        if NEG.search(sent):
            continue
        for ev, rx in CLAIMS.items():
            if rx.search(sent) and ev not in confirmed:
                v.append(f"claims {ev} without confirmation: {sent!r}")
            if ev == "picked" and rx.search(sent) and re.search(r"\b(i'm holding|i am holding|holding the)\b", sent, re.I) and not holding:
                v.append(f"claims holding while empty: {sent!r}")
    return v


def describe(obj) -> str:
    c = f"{obj.color} " if obj.color else ""
    return f"the {c}{label(obj.cls)}"


def outcome_reply(out, objects, handover_text: str = "Here you are: {what}.") -> str:
    o = objects.get(out.obj) if out.obj else None
    what = describe(o) if o else "it"
    if out.status == "handed_over":
        if "handed_over" not in out.confirmed:
            return "I could not confirm the hand-over."
        if "{what}" in handover_text and "your" in handover_text and o:       # "Here's your red can." (no 'the')
            return handover_text.format(what=what.removeprefix("the "))
        return handover_text.format(what=what)
    if out.status == "placed":
        return f"I put {what} down in {out.where}." if out.where not in ("home",) else f"I put {what} down at home."
    if out.status == "holding" and out.code is None:
        return f"I picked up {what} in {out.where} and I'm holding it with the arm folded."
    code = out.code
    if code == "NOT_FOUND":
        return f"I couldn't find {what}: I looked in every room."
    if code == "NO_PATH":
        tail = " I'm still holding it." if out.status == "holding" else ""
        return f"I can't get there: there is no path ({out.detail}).{tail}"
    if code == "NOT_REACHABLE":
        return f"I found {what} but I couldn't line my gripper up with it ({out.detail}). Can you move it into the open?"
    if code == "LOST_AFTER_MISS":
        return (f"I tried to grasp {what} and missed, and now my wrist camera can't see it any more ({out.detail}). "
                "I may have pushed it away.")
    if code == "GRASP_FAILED":
        # no claim about where it is now: a missed grasp can push / knock the object over (2026-11-28 GUI take: knocked
        # 20 cm and lying on its side while the reply said "It is still where it was")
        return f"I couldn't get a grip on {what} after {out.detail}. I may have pushed it, so it might not be where it was."
    if code == "DROPPED":
        return f"I dropped {what} on the way. It's on the floor in {out.where}. Shall I try to pick it up again?"
    if code == "NOT_TAKEN":
        return f"You didn't take {what}, so I'm still holding it. Tell me where it should go, or say 'hand it to me' again."
    if code == "GRASP_UNCONFIRMED":
        why = f" ({out.detail})" if out.detail else ""
        return (f"I closed my gripper on {what} and lifted it, but I can't confirm I'm holding it{why}. "
                "I'm keeping my gripper closed: say 'put it down' or 'try again'.")
    if code == "HOLD_UNCONFIRMED":
        return f"I can't confirm I'm holding {what} (I couldn't read my gripper), so I didn't let go."
    if code == "RELEASE_UNCONFIRMED":
        return f"I couldn't check that my gripper opened, so I closed it again. I can't confirm I'm holding {what}."
    if code == "NOT_STATIONARY":
        return f"I couldn't stand still to hand over {what}, so I didn't let go: I'm still holding it."
    if code == "NOT_RELEASED":
        return f"My gripper did not open, so I'm still holding {what}."
    if code == "DELIVERY_UNCONFIRMED":
        return f"I opened my gripper, but I can't confirm that {what} reached you. Please check the floor next to me."
    if code == "NOT_HOLDING":
        return "I'm not holding anything."
    if code == "BUSY":
        b = objects.get(out.detail)
        return f"I'm already holding {describe(b) if b else 'something'}. Tell me where to put it first."
    return "I could not finish that."


def safe(reply: str, confirmed: set, holding: bool) -> str:
    """The reply if it is grounded, else a neutral fallback (never an unconfirmed success claim)."""
    return reply if not ground(reply, confirmed, holding) else "I could not confirm that it worked."
