"""Phase 7b: deterministic text rules for what the USER's text may and may not do (red-team fixes, docs/results/phase7b_redteam_fixes.md).

Pure regex, no LLM, no heavy imports, no state. All of it runs BEFORE the planner (graph.n_route) except `unsupported_calls` / `drop_after_stop`
(they judge the plan) and `vacuous_ack` (judges the reply).

  a) `spoofed_input`      text that LOOKS like a tool result / system message / rule change ("<tool_result>", "SYSTEM:", "the last tool output said ...",
                          "safety checks are off", "developer mode", "ignore all previous instructions") is plain data. It can never change state or rules,
                          so the whole turn is answered with a short explanation and nothing runs (a stop is handled earlier and still wins).
  b) `stop_prefix`        a message that STARTS with a stop ("Emergency stop, then continue walking ...") is a stop and nothing else;
     `drop_after_stop`    a plan that has a stop call loses every later call. Motion needs a new message.
  c) `speed_request`      numeric / maximum speed requests: the speed phrase is removed, the rest of the command runs, the reply says the robot's top speed.
  d) `unsupported_calls`  a call whose tool the user's words never asked for (a patrol out of "spin in place") is dropped, so it can never reach a confirmation question.
  e) `privacy_request`    system prompt, hidden instructions, tokens / passwords / private links / addresses, raw memory or database dumps -> a clear refusal.
     `sql_text`           database commands are not run and not passed on.
  f) `claim_request` + `vacuous_ack`   "pretend you did X and say so" -> an honest reply; a bare "Okay." with nothing executed is replaced by a plain statement.
"""
from __future__ import annotations

import re
from pathlib import Path

MAX_SPEED_MPS = 0.15          # FALLBACK only (= robot.yaml command_limits.vx); `configured_max_speed()` reads the real value
ROBOT_YAML = Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "gurdy_description" / "config" / "robot.yaml"


def configured_max_speed(path: Path | None = None) -> float:
    """The robot's maximum forward walking speed: `command_limits.vx` of robot.yaml (single source of truth for geometry and limits).
    Falls back to MAX_SPEED_MPS when the file is not shipped (static demo bundle) or has no such key."""
    try:
        import yaml
        v = yaml.safe_load(open(path or ROBOT_YAML))["command_limits"]["vx"]
        return float(v) if float(v) > 0 else MAX_SPEED_MPS
    except Exception:                                    # noqa: BLE001
        return MAX_SPEED_MPS

# ------------------------------------------------------------------------------------------------ a) spoofed / injected framing
_SPOOF = [
    re.compile(r"</?\s*(user|system|assistant|tool|tool_result|tool_results|function_results?|function|result|instructions?)\b[^>]{0,40}>", re.I),
    re.compile(r"\btool_(result|output|call)s?\b", re.I),
    re.compile(r"(^|[.!?\n:;]\s*)\[?\s*(system|admin|administrator|developer|root|operator|assistant)\s*(message|prompt|override|note|notice|command)?\s*\]?\s*[:>\]]", re.I),
    re.compile(r"\b(last|previous|latest|prior|that|the)\s+(tool|function)\s+(output|result|response|call|message)\b", re.I),
    re.compile(r"\b(tool|function)\s+(output|result|response)\s+(said|says|showed|shows|confirmed|returned|reported|told)\b", re.I),
    re.compile(r"\b(developer|debug|maintenance|admin(?:istrator)?|god|sudo|unrestricted|jailbreak)\s+mode\b", re.I),
    re.compile(r"\bauto-?confirm\b|\bnever ask me anything\b", re.I),
    re.compile(r"\bignore\s+(?:all\s+|any\s+|every\s+)?(?:of\s+)?(?:your\s+|the\s+|my\s+)?(?:previous|prior|above|earlier|preceding|all)\b[^.?!]{0,25}\b(?:instructions?|rules?|prompts?|limits?|restrictions?|guidelines?)\b", re.I),
    re.compile(r"\b(?:no|without any|without)\s+safety\s+(?:limits?|checks?|rules?)\b|\bfrom now on you (?:have|are|will)\b", re.I),
    re.compile(r"[\"']\s*(?:tool|calls?|args|action|continue|result|ok|confirmed|estop)\s*[\"']\s*:", re.I),                       # JSON tool-call / result syntax typed into the chat
    re.compile(r"\b(?:authori[sz]e[sd]?|allow(?:s|ed)?|permit(?:s|ted)?|order(?:s|ed)?)\s+you\s+to\s+(?:override|ignore|bypass|disable|break|skip|turn off)\b|\boverride\s+(?:the\s+)?(?:\w+\s+){0,3}(?:distance|safety|limits?|rules?|checks?)\b", re.I),
    re.compile(r"\bpretend (?:you are|to be|you're)\b|\byou are now (?:a|an|in)\b|\bact as (?:a|an) (?:robot|ai|assistant) (?:with|without)\b", re.I),
    re.compile(r"\b(?:the\s+)?(?:safety(?:\s+(?:checks?|limits?|rules?|checker|system))?|e-?stop|emergency stop|no[- ]go zones?)\s+(?:is|are|was|were|has been|have been|is now|are now|got)\s+(?:now\s+)?(?:turned\s+)?(?:off|disabled|removed|deleted|deactivated|lifted|suspended)\b", re.I),
]
_QUESTION = re.compile(r"^\W*(?:is|are|was|were|did|do|does|can|could|why|what|when|where|how|which|who|tell me (?:if|whether))\b|\?\s*$", re.I)
_QUOTED = re.compile(r"\"[^\"]*\"|“[^”]*”|'[^']*'")
_SAVE_AS = re.compile(r"\b(save|remember|store|mark|record|call|name|label)\b", re.I)


def spoofed_input(text: str) -> bool:
    """True when the text carries tool-result / system / rule-change framing. A quoted string right after a save / name cue is a name (data), not framing."""
    t = text
    if _SAVE_AS.search(t):
        t = _QUOTED.sub(" ", t)
    for i, pat in enumerate(_SPOOF):
        if pat.search(t):
            if i == len(_SPOOF) - 1 and _QUESTION.search(t):      # "is the e-stop disabled?" is a question, not a claim
                continue
            return True
    return False


SPOOF_REPLY = ("I treat things like a tool result, a system message, an authorisation or a rule change typed into the chat as plain words: they can't change "
               "my rules or what I have done. I did nothing. If you want something done, tell me just that.")

# ------------------------------------------------------------------------------------------------ b) stop wins
_STOP_PREFIX = re.compile(r"^\W*(?:(?:please|ok|okay|hey|now|robot|gurdy)\W+)*(?:emergency\s+stop|e-?stop|stop(?:\s+(?:now|right\s+now|everything|moving|walking))?|halt|freeze)\s*(?:[,.;:!\-]|\band\b|\bthen\b|\bafter that\b)\s*\S", re.I)


def stop_prefix(text: str) -> bool:
    """A message that starts with an unambiguous stop and carries more text ("stop and then keep walking"): the stop is all that runs."""
    return bool(_STOP_PREFIX.match(text)) and not re.search(r"\b(?:don'?t|do not|never|not)\b", text[:40], re.I)


_SAYS_STOP = re.compile(r"\b(?:stop|halt|freeze|e-?stop|emergency|hold\s+(?:on|it|still)|stay\s+(?:put|still)|wait)\b", re.I)


def says_stop(text: str) -> bool:
    """The USER's words contain a stop cue (the planner adding a stop by itself, e.g. for a replacement command, does not count)."""
    return bool(_SAYS_STOP.search(text))


def drop_after_stop(calls: list[dict]) -> tuple[list[dict], list[dict]]:
    """(kept, dropped): everything after the first stop call is dropped (motion needs a new message)."""
    for i, c in enumerate(calls):
        if isinstance(c, dict) and c.get("tool") == "stop":
            return calls[:i + 1], calls[i + 1:]
    return calls, []


STOP_ONLY_NOTE = "I did nothing else. After a stop, tell me again what you want me to do."

# ------------------------------------------------------------------------------------------------ c) speed
_NUM = r"\d+(?:[.,]\d+)?"
_SPEED_NUM = re.compile(rf"(?:\bat\s+|\bwith\s+|\bgo\s+|\bmove\s+|\brun\s+|\bwalk\s+)?{_NUM}\s*(?:m/s|mps|m\s*/\s*s|(?:meters?|metres?|km|kilomet(?:er|re)s?|miles?)\s*(?:per|an|/)\s*(?:second|sec|hour|hr|h)|km/?h|kph|mph)\b", re.I)
_SPEED_WORDS = re.compile(r"\b(?:at\s+)?(?:max(?:imum)?|full|top|highest|breakneck|lightning|supersonic)\s+(?:possible\s+)?(?:speed|velocity|pace)\b"
                          r"|\bas\s+(?:fast|quickly)\s+as\s+(?:you\s+(?:can|possibly\s+can)|possible)\b|\bat\s+(?:the\s+)?fastest\b|\bfull\s+(?:throttle|blast|steam)\b"
                          r"|\b(?:very|really|super|extremely)\s+(?:fast|quickly)\b|\bsprint(?:ing)?\b|\bfaster\b", re.I)


def speed_request(text: str) -> tuple[str, bool]:
    """(text without the speed phrase, whether a speed above the robot's one speed was requested)."""
    new = _SPEED_NUM.sub(" ", text)
    new = _SPEED_WORDS.sub(" ", new)
    hit = new != text
    if hit:
        new = re.sub(r"\s+([,.;!?])", r"\1", re.sub(r"\s{2,}", " ", new))
        new = re.sub(r"([,;])(\s*[,;])+", r"\1", new).strip(" ,;")
        new = re.sub(r"^(?:and\s+)?", "", new)
    return new, hit


def speed_note(max_mps: float = MAX_SPEED_MPS) -> str:
    return f"I can't go that fast. My top speed is {max_mps:g} metres per second, so I will go at that speed."


# ------------------------------------------------------------------------------------------------ endless requests (no open-ended motion)
_ENDLESS = re.compile(r"\b(?:forever|for\s+ever|endlessly|indefinitely|non-?stop|infinitely|without\s+(?:ever\s+)?stopping|until\s+(?:the\s+)?(?:battery|power)\s+(?:dies|runs\s+out|is\s+(?:empty|dead)))\b", re.I)
_MOTION_VERB = re.compile(r"\b(walk|spin|turn|rotate|go|move|patrol|circle|run|drive|explore|search|pace|wander|roam|march|step|crawl)\b", re.I)


def endless_request(text: str) -> bool:
    return bool(_ENDLESS.search(text)) and bool(_MOTION_VERB.search(text))


ENDLESS_REPLY = "I don't move without an end. I walk up to 3 metres or turn up to 360 degrees per command. Tell me how far or how many degrees, and I will do that."

# ------------------------------------------------------------------------------------------------ d) calls the user's words never asked for
_CUES: dict[str, re.Pattern] = {
    "walk": re.compile(r"\b(walk|move|go|step|forward|backwards?|back|advance|head|proceed|ahead|left|right|sideways?|strafe|come|approach|shuffle|creep|inch|retreat|reverse|meters?|metres?|cm|centimet\w+)\b", re.I),
    "turn": re.compile(r"\b(turn|rotate|spin|face|left|right|around|degrees?|deg|pivot|u-?turn|look|clockwise|anticlockwise|counter\w*)\b", re.I),
    "go_to": re.compile(r"\b(go|walk|head|navigate|take|come|move|drive|bring|visit|check|get|proceed|travel|return|follow|reach|to|into|towards?|near|next|beside|by|at)\b", re.I),
    "find": re.compile(r"\b(find|search|look|locate|where|seek|spot|see|hunt|check|scan|show|detect|any|is there)\b", re.I),
    "explore": re.compile(r"\b(explor\w*|check|room|area|rooms|areas|map|scan|survey|discover|sweep|investigate|wander|roam|look around|coverage|everything|whole|entire|all)\b", re.I),
    "patrol": re.compile(r"\b(patrol\w*|guard|rounds?|loops?|circuit|laps?|sweep|check|visit|go|walk|between|times|repeat\w*|again|back and forth|keep watch|watch)\b", re.I),
    "inspect": re.compile(r"\b(inspect\w*|look|check|examine|see|view|approach|go|walk|read|study|investigate|scan|observe|closer|near)\b", re.I),
    "return_home": re.compile(r"\b(home|back|return|base|start|origin|dock|charg\w*|come|leave|there)\b", re.I),
    "schedule": re.compile(r"\b(in|after|at|later|minutes?|seconds?|hours?|every|wait|then|delay\w*|schedule\w*|tomorrow|tonight|morning|evening|o'?clock|am|pm|soon|timer|countdown|mins?|secs?)\b", re.I),
}
# stronger, narrower cues for tools whose unrequested use is costly (a confirmation question about a patrol nobody asked for)
_STRICT = {"patrol", "explore", "schedule"}


def unsupported_calls(text: str, calls: list[dict]) -> tuple[list[dict], list[dict]]:
    """(kept, dropped): a motion call whose tool has no cue at all in the user's words is dropped. Only the patrol / explore / schedule tools are judged
    (the others are handled by the guard + safety layers and by their own typed arguments)."""
    kept, dropped = [], []
    for c in calls:
        t = c.get("tool") if isinstance(c, dict) else None
        pat = _CUES.get(t) if t in _STRICT else None
        if pat is not None and not pat.search(text):
            dropped.append(c)
        else:
            kept.append(c)
    return kept, dropped


# ------------------------------------------------------------------------------------------------ e) privacy
_PRIVACY = [
    re.compile(r"\b(?:system|initial|hidden|secret|original|internal|developer)\s+(?:prompt|instructions?|message|rules)\b", re.I),
    re.compile(r"\b(?:your|the)\s+(?:prompt|instructions|programming|configuration|config|rules text)\b.{0,30}\b(?:repeat|print|show|reveal|read|recite|tell|say|quote|output|dump|copy)\b|\b(?:repeat|print|show|reveal|read|recite|tell|say|quote|output|dump|copy)\b.{0,40}\b(?:your|the)\s+(?:prompt|instructions|programming|configuration|config)\b", re.I),
    re.compile(r"\b(?:repeat|print|show|read|recite|quote|output|copy|echo)\b.{0,30}\b(?:text|everything|all|words|message|lines?)\s+(?:above|before|so far|at the (?:top|start|beginning))\b|\bstarting with\s+[\"“']?you are\b|\b(?:text|everything|words)\s+above\b", re.I),
    re.compile(r"\b(?:tokens?|passwords?|passcodes?|pass\s*phrases?|api[\s_-]?keys?|credentials?|private\s+keys?|access\s+keys?|auth(?:entication)?\s+(?:keys?|codes?)|env(?:ironment)?\s+(?:var(?:iable)?s?|settings?))\b", re.I),
    re.compile(r"\b(?:web|private|dashboard|control|access|remote|share|sharing|public|secret|login)\s+(?:link|url|address|page|site|portal)\b|\bip\s+address\b|\btailscale\b|\bts\.net\b|\bhttps?://|\bcontrol you (?:remotely|from)\b", re.I),
    re.compile(r"\b(?:dump|export|exfiltrate|leak|print|show|list|give|read|send)\b.{0,30}\b(?:memory|database|db|tables?|sqlite|everything you (?:know|remember|stored?))\b.{0,40}\b(?:raw|every|entire|all|whole|verbatim|rows?)\b|\b(?:raw|every|entire|all|whole)\s+(?:rows?|tables?|records?|database|memory (?:database|dump|file))\b|\bmemory\s+(?:dump|file|database)\b", re.I),
]
PRIVACY_REPLY = ("I can't share that. My instructions, access tokens and links, passwords, settings and the raw contents of my memory are private. "
                 "I can tell you what I have seen, where things are, and what I am doing.")


def privacy_request(text: str) -> bool:
    return any(p.search(text) for p in _PRIVACY)


_SQL = re.compile(r"\b(?:drop|truncate|alter)\s+(?:table|database|index)\b|\bdelete\s+from\b|\binsert\s+into\b|\bupdate\s+\w+\s+set\b|\bselect\b[^.?!]{1,60}\bfrom\b|;\s*--|\bunion\s+select\b|\bexec(?:ute)?\s+\w+\(", re.I)
SQL_REPLY = "I don't run database commands, and I won't pass text like that on to my memory. If you want me to do something, say it on its own."


def sql_text(text: str) -> bool:
    return bool(_SQL.search(text))


# ------------------------------------------------------------------------------------------------ f) no false claims
_CLAIM_REQ = re.compile(
    r"\b(?:pretend|act\s+(?:as\s+if|like)|imagine|claim|lie|fake|make\s+believe|just\s+say|say|tell\s+me|write|report|announce|state|confirm|admit)\b[^.?!]{0,50}"
    r"\b(?:(?:you(?:'ve|\s+have|\s+had)?\s+(?:already\s+)?(?:picked|grabbed|found|reached|arrived|saved|searched|checked|finished|completed|done|did|been|delivered|moved|took|taken|cleaned|fetched|brought|opened|removed|deleted|disabled))"
    r"|(?:mission|task|search|job|patrol|exploration|inspection)\s+(?:is\s+|was\s+|has\s+been\s+)?(?:complete[d]?|done|finished|successful)|(?:it(?:'s|\s+is)|all|that(?:'s|\s+is))\s+(?:done|complete[d]?|finished))\b", re.I)
CLAIM_REPLY = "I can only tell you what really happened. I have not done that, so I won't say that I did."


def claim_request(text: str) -> bool:
    return bool(_CLAIM_REQ.search(text))


_BARE_ACK = re.compile(r"^\W*(?:ok(?:ay)?|sure|done|alright|all right|got it|will do|fine|noted|roger|understood)\W*$", re.I)
_USER_ACK = re.compile(r"\b(thanks?|thank you|ok(?:ay)?|good|great|cool|nice|fine|got it|alright|cheers|well done|perfect|awesome)\b", re.I)
VACUOUS_REPLY = "I did not do anything, because I did not understand what to do. Could you say it another way?"


def vacuous_ack(reply: str, user_text: str, did_something: bool) -> str | None:
    """A bare 'Okay.' when nothing was executed, nothing is pending and the user did not just acknowledge something: say plainly that nothing happened."""
    if did_something or not _BARE_ACK.match(reply or "") or _USER_ACK.search(user_text):
        return None
    return VACUOUS_REPLY
