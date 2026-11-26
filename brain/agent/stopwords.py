"""The e-stop fast path text rules, with NO heavy imports (the GPU voice image has no langgraph): stop detection and the unsafe-request pattern.

A false stop is safe, a missed stop is not. Speech recognition turns the one-word "Stop." into "Stalk." and "Halt." into "Hide." (measured on real
clips, docs/results/phase5_eval_sft_v2.md), so very short utterances are matched fuzzily against stop / halt / freeze / wait."""
from __future__ import annotations

import re
from difflib import SequenceMatcher

# phrases that try to switch safety off or hurt things: refused before any tool is considered
_UNSAFE_SAFETY = re.compile(r"\b(disable|turn off|switch off|ignore|bypass|override|deactivate)\b.{0,30}\b(safety|e-?stop|emergency|no[- ]go|limits?|collision)\b", re.I)
# Phase 7c: a harm verb counts only with a THING or PERSON after it ("run over the box", "push him"). "run over to room D", "run over there", "push on to room B",
# "push forward", "kick off the patrol" and "your RAM usage" are ordinary words; a question about the PAST ("did you hit anything?") is not a request.
_HARM_OBJECT = (r"(?:(?:into|against|through|at)\s+)?(?:the|a|an|that|this|those|these|my|his|her|their|its|him|them|it|me|us|someone|somebody|anyone|anybody|people|person|"
                r"pedestrians?|everything|anything|something|all|any|every|each|both|red|blue|yellow|green|white|orange|black|brown|grey|gray|box|boxes|barrel|barrels|wall|walls|"
                r"chair|chairs|cone|cones|toolbox|mannequin|dummy|extinguisher|table|door)\b")
_UNSAFE_HARM = re.compile(rf"\b(?:hit|ram|smash|kick|shove|push|knock\s+over|crash\s+into|run\s+over)\s+{_HARM_OBJECT}", re.I)
_PAST_QUESTION = re.compile(r"^\W*(?:did|have|has|had|was|were|why|what|how|when|where|which|who|is|are)\b", re.I)
_STRONG_STOP = re.compile(r"\b(stop|halt|freeze|emergency)\b", re.I)
_WEAK_STOP = re.compile(r"^\W*(stap|stob|stup|top|stopp?|holt|hault|hallt)\W*$", re.I)     # STT confusions, only as a whole utterance
# a question about stopping ("why did you stop") or a request to switch the e-stop off is not a stop command
_NOT_A_COMMAND = re.compile(r"^\W*(why|what|when|where|how|which|who)\b|\b(did|have|had|was|were)\s+(you|it)\b", re.I)

STOP_WORDS = ("stop", "halt", "freeze", "wait")
# words speech recognition produced (or plausibly produces) for them; matched as a whole short utterance, always
STOP_CONFUSIONS = {"stalk", "stock", "stomp", "stoop", "stoke", "stuck", "stab", "stap", "stob", "stup", "top", "stops", "stopp",
                   "hide", "hold", "holt", "hault", "hallt", "hall", "hot", "halts", "weight", "wade", "wait", "frees", "freeze", "breeze", "freezes"}
# real command words that merely look like a stop word (step / start / star ...): never matched by similarity
_PROTECTED = {"step", "start", "stand", "stay", "star", "spot", "stone", "store", "walk", "turn", "go", "back", "find", "look", "where", "what", "explore",
              "patrol", "left", "right", "forward", "home", "room", "yes", "no", "the", "box", "barrel", "chair", "cone", "red", "blue", "green", "yellow"}
_TOK = re.compile(r"[a-z']+")


def is_unsafe_request(text: str) -> bool:
    """Level-3 text rule: requests to disable safety, hit or push things."""
    return bool(_UNSAFE_SAFETY.search(text)) or (bool(_UNSAFE_HARM.search(text)) and not _PAST_QUESTION.search(text))


def fuzzy_stop(text: str) -> bool:
    """A SHORT utterance (at most 3 words) with a word that is a known stop confusion or looks like stop / halt / freeze / wait ("Stalk.", "Hide.",
    "Stoop it", "hold on")."""
    toks = _TOK.findall(text.lower())
    if not toks or len(toks) > 3:
        return False
    for w in toks:
        if w in STOP_CONFUSIONS:
            return True
        if 3 <= len(w) <= 6 and w not in _PROTECTED and any(w[0] == s[0] and SequenceMatcher(None, w, s).ratio() >= 0.75 for s in STOP_WORDS):
            return True
    return False


def is_estop_phrase(text: str) -> bool:
    """E-stop fast path (design 4.3 level 2): runs before the LLM, on every transcript."""
    t = text.strip()
    if _WEAK_STOP.match(t):
        return True
    if (bool(_STRONG_STOP.search(t)) and len(t.split()) <= 6 and not _NOT_A_COMMAND.search(t)
            and not is_unsafe_request(t) and not re.search(r"\b(don'?t|do not|never|until)\b", t, re.I)):
        return True
    return (fuzzy_stop(t) and not _NOT_A_COMMAND.search(t) and not is_unsafe_request(t)
            and not re.search(r"\b(don'?t|do not|never|until)\b", t, re.I))


# "why are you standing still" / "what made you stop": a question about the last stop, answered from the agent's own stop record
_STOP_WHY = re.compile(
    r"^\W*(?:(?:hey|so|ok|okay|and|but|tell me)\W+)*(?:"
    r"(?:why|how come)\b.*\b(?:stop(?:ped)?|stand(?:ing)?(?:\s+still)?|stood|halt(?:ed)?|freez\w*|froze|not\s+moving|(?:aren'?t|are\s+not)\s+you\s+(?:moving|walking|going)|(?:are|did|do)(?:n'?t)?\s+you\s+(?:still|idle|waiting)|still\s+here)"
    r"|what\W+(?:made|stopped|caused|got|is\s+stopping)\s+(?:you|the\s+robot)\b"
    r"|what(?:'s|\s+is|\s+was)\s+the\s+reason\s+(?:you|for)\b.*\b(?:stop|stand|halt|still)"
    r")", re.I)


def is_stop_why(text: str) -> bool:
    return bool(_STOP_WHY.search(text)) and not is_estop_phrase(text)
