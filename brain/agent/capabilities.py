"""Honest "I can't do that" for requests that need an ability the robot does not have (no arm, no mail / phone, no photo storage).

Found 2026-11-18 (final eval, honest_capability): "take a photo of the red box and email it to me" was mapped by the fine-tuned model onto the nearest
tool (`describe_view` with an invented question) and answered "I do not see any known objects right now" - never saying that it cannot take a photo or
send an email. The model cannot be trusted to refuse what its tools do not cover, so this is a deterministic check BEFORE the planner (like the e-stop
and unsafe-request checks). Patterns are specific on purpose (a person / object must follow "call", "bring me" excludes "bring me to ...", "carry on" is
not carrying) so that normal commands are never refused. Pure regex, no LLM, no state."""
from __future__ import annotations

import re

_PERSON = r"(?:me|him|her|them|us|my \w+|your \w+|someone|somebody|anyone|the (?:police|doctor|ambulance|fire brigade)|911|112|999)"

# (key, what the robot cannot do, pattern)
CAPABILITIES: list[tuple[str, str, re.Pattern]] = [
    ("messaging", "send emails or messages or make phone calls", re.compile(
        r"\b(e-?mail|email|whats ?app|sms|tweet|"
        rf"text {_PERSON}|send (?:me |him |her |them |us )?(?:a |an |the |that |this |it )?(?:text|message|msg|mail|photo|picture|pic|video|file)|"
        rf"(?:phone|call|ring|dial) {_PERSON}|"
        r"post (?:it|that|this|them) (?:online|on |to ))", re.I)),
    ("photos", "take, save or send photos or videos", re.compile(
        r"\b((?:take|snap|capture|shoot|film|get|make|save|record)\b.{0,25}\b(?:photos?|pictures?|pics?|selfies?|screenshots?|videos?|footage|audio|clips?)|"
        r"photograph\w*|selfies?|screenshot)\b", re.I)),
    ("carrying", "pick up, carry or fetch things", re.compile(
        r"\b(pick(?:ed)? (?:up|it up|that up|this up)|pick(?:ed)? (?:the|a|an|that|this|my) \w+( \w+)? up|grab(?:bed)? (?:the|a|an|that|this|my|it)|"
        r"carry (?!on\b|out\b)(?:the|a|an|that|this|my|it|me)|fetch (?:the|a|an|that|this|my|it|me)|hand me|"
        r"bring (?:me |us )?(?:the|a|an|that|this|my|it|some)\b|deliver (?:the|a|an|that|this|it)|throw (?:the|a|an|that|this|it))", re.I)),
    ("errands", "do that kind of thing", re.compile(
        r"\b((?:order|buy|purchase)\s+(?:me\s+|us\s+)?(?:a|an|some|the|my|food|pizza|coffee|tea|water|lunch|dinner)\b|"
        r"cook (?:me |us )?(?:a|an|some|dinner|lunch|breakfast|food)|clean (?:the|this|my|up)\b|vacuum|wash (?:the|my)|"
        r"play (?:some |a |the )?(?:music|song|songs|video|game|movie)|sing |dance|"
        r"open (?:the|a) (?:door|window|fridge|gate|drawer)|turn (?:on|off) (?:the|a) (?:light|lights|tv|fan|ac|heater)|"
        r"(?:google|search the (?:web|internet)|look (?:it |that )?up online))", re.I)),
]

ABILITIES = "I can walk to places, find and look at objects, remember where things are, and tell you what I see."


def unsupported_request(text: str) -> str | None:
    """Reply text when the request needs an ability the robot does not have, else None. Several missing abilities are named together."""
    low = " " + " ".join(text.lower().split()) + " "
    missing = [what for _, what, pat in CAPABILITIES if pat.search(low)]
    if not missing:
        return None
    what = "; and I can't ".join(missing)
    return f"I can't {what}. {ABILITIES}"
