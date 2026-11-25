"""Room names by voice. Whisper cannot reliably hear a single letter ("D" -> "B", or dropped: "Go to room."), so rooms can be
said phonetically ("room Delta", "room D as in dog") and the robot reads the letter back ("Going to room B, Bravo.").
Pure strings: used by Agent.say before anything else (planner, guard, fuzzy matcher all see "room D")."""
from __future__ import annotations

import re
from dataclasses import dataclass, field

NATO = {"A": "Alpha", "B": "Bravo", "C": "Charlie", "D": "Delta", "E": "Echo", "F": "Foxtrot", "G": "Golf", "H": "Hotel"}
_WORD2LETTER = {w.lower(): k for k, w in NATO.items()} | {"alfa": "A", "charly": "C"}
_WORDS = "|".join(sorted(_WORD2LETTER, key=len, reverse=True))
_AS_IN = r"(?:\s*,?\s*(?:as in|for|like)\s+[a-z]+)?"
_ROOM_PHON = re.compile(rf"\broom\s*,?\s+(?:({_WORDS})|([a-h])){_AS_IN}\b", re.I)
_ANSWER = re.compile(rf"^\W*(?:the\s+)?(?:room\s*,?\s+)?(?:({_WORDS})|([a-h])){_AS_IN}\W*$", re.I)
_CORRECTION = re.compile(r"^\W*(?:no|nope|wrong|not that|not that one|i said|i meant|no i said|no i meant)\W+(.+)$", re.I)
_MISSING = re.compile(r"\b(?:to|into|toward|towards)\s+(?:the\s+)?room\b(?!\s*,?\s*(?:[a-z0-9]+\b))", re.I)


def phonetic(letter: str) -> str | None:
    return NATO.get(letter.upper())


def normalize(text: str) -> str:
    """"room delta" / "room D as in dog" / "room d" -> "room D" (a room word followed by a single letter or a NATO word)."""
    def sub(m):
        return "room " + (_WORD2LETTER[m.group(1).lower()] if m.group(1) else m.group(2).upper())
    return _ROOM_PHON.sub(sub, text)


_BARE_WORD = re.compile(rf"(?<![\w-])(?<!room )(?<!room, )({_WORDS})(?![\w-])(?:\s+room\b)?", re.I)
_BARE_LETTER = re.compile(r"\b((?:go|head|walk|move|navigate|drive|come|take me|bring me|run|proceed|travel)(?:\s+\w+){0,2}?\s+(?:to|into|towards?)|\bto|\binto)\s+(?:the\s+)?([b-dB-D])(?=\s*(?:$|[.,!?;]|\b(?:and|then|please|now|first|next|after)\b))", re.I)


def bare_rooms(text: str) -> str:
    """Room names win over everything else (the object fuzzy matcher turned "go to Charlie" into "Did you mean chair?"):
    a NATO word without the word "room" ("go to Charlie", "Charlie", "then Delta") becomes "room C"; a lone letter B / C / D right after
    to / into / towards ("go to c") too. The letter "A" alone is an article, so it is only read as a room inside "room A" / "Alpha".
    The whole utterance being just a room name ("charlie", "room c", "C") becomes a go-to command."""
    t = text.strip()
    m = _ANSWER.match(t)
    if m and (m.group(1) or t.lower().startswith("room") or (m.group(2) and m.group(2).lower() in "bcd" and len(t.strip(" .!?")) == 1)):
        letter = _WORD2LETTER[m.group(1).lower()] if m.group(1) else m.group(2).upper()
        return f"go to room {letter}"
    out = _BARE_WORD.sub(lambda mm: "room " + _WORD2LETTER[mm.group(1).lower()], text)
    return _BARE_LETTER.sub(lambda mm: f"{mm.group(1)} room {mm.group(2).upper()}", out)


# "that room" / "the other room" / "another room" with no letter: which room is only known from the conversation
_REF = re.compile(r"\b(?:(?P<that>that|this|the same)\s+room|(?P<other>(?:that|the|an)\s+other\s+room|another\s+room|other\s+room))\b(?!\s*,?\s*(?:[a-h]\b|" + _WORDS + r"\b))", re.I)


@dataclass
class RoomRef:
    text: str | None = None            # resolved: the utterance with "room X" in its place
    question: str | None = None        # ask which room
    template: str | None = None        # the utterance with a bare "room" in its place (the answer's letter completes it)
    letters: list[str] = field(default_factory=list)
    rule: str = ""


def room_reference(text: str, places: list[str], last_room: str | None) -> RoomRef | None:
    """None = no such reference. `last_room`: the letter of the room the conversation talked about last (None = no earlier room).
    "that room"        -> the last room; none yet -> ask (listing the rooms).
    "the other room"   -> the single known room that is not the last one; several (or no earlier room) -> ask, listing the candidates."""
    m = _REF.search(text)
    letters = known_letters(places)
    if not m or not letters:
        return None
    other = bool(m.group("other"))
    cand = [l for l in letters if not (other and last_room and l == last_room)]
    if not other and last_room in letters:
        return RoomRef(text=text[:m.start()] + f"room {last_room}" + text[m.end():], rule="room_ref_that")
    if other and last_room and len(cand) == 1:
        return RoomRef(text=text[:m.start()] + f"room {cand[0]}" + text[m.end():], rule="room_ref_other")
    template = text[:m.start()] + "room" + text[m.end():]
    return RoomRef(question=f"Which room: {listing(cand)}?", template=template, letters=cand, rule="room_ref_ask")


def answer_letter(text: str) -> str | None:
    """The whole utterance is a room answer: "B", "bravo", "room Delta", "D as in dog" -> "B" / "D"; anything else -> None."""
    m = _ANSWER.match(text.strip())
    if not m:
        return None
    return _WORD2LETTER[m.group(1).lower()] if m.group(1) else m.group(2).upper()


def correction_letter(text: str) -> str | None:
    """"no, Delta" / "no I said room D" -> "D"."""
    m = _CORRECTION.match(text.strip())
    return answer_letter(m.group(1)) if m else None


def mentioned_letters(text: str) -> list[str]:
    return re.findall(r"\broom ([A-Z])\b", normalize(text))


def known_letters(places: list[str]) -> list[str]:
    return sorted(m.group(1) for p in places if (m := re.fullmatch(r"room ([A-Z])", p)))


def listing(letters: list[str]) -> str:
    return letters[0] if len(letters) == 1 else ", ".join(letters[:-1]) + " or " + letters[-1]


def missing_letter_question(text: str, places: list[str]) -> str | None:
    """"Go to room." (the letter was not heard) -> "Which room: A, B or D?"."""
    letters = known_letters(places)
    if letters and _MISSING.search(text):
        return f"Which room: {listing(letters)}?"
    return None


# Letters speech recognition really swaps for a room letter ("be" / "dee" / "see" / "pee" / "vee" / "tee" / "gee"). Every letter is "edit distance 1" from
# every other one, so for letters the near-miss rule is acoustic: only these go on to the "Did you mean ...?" repair; any other letter (Z, Q, X ...) gets the honest answer.
_SOUND_ALIKE = set("BCDEGPTV")


def unknown_room_message(text: str, places: list[str]) -> str | None:
    """An explicit "room X" whose letter is not on the map: say so honestly ("There is no room Z. I know rooms A, B, C and D.") BEFORE any
    spelling repair. Only misspelt words ("roon B") reach the "Did you mean ...?" repair, which names what it assumed."""
    letters = known_letters(places)
    if not letters:
        return None
    top = max(ord(letters[-1]) + 2, ord("D"))
    typed = mentioned_letters(text) + [m.upper() for m in re.findall(r"\broom\s+([a-hj-z])\b", text)]     # "room z" (lowercase) too; a lone "i" stays a word
    for ch in dict.fromkeys(typed):
        if ch not in letters and (ord(ch) <= top or ch not in _SOUND_ALIKE):
            plural = "rooms" if len(letters) > 1 else "room"
            known = letters[0] if len(letters) == 1 else ", ".join(letters[:-1]) + " and " + letters[-1]
            return f"There is no room {ch}. I know {plural} {known}."
    return None


# spoken names of the letters (Whisper writes "bee" / "be" / "dee" / "sea" for a lone letter)
_LETTER_NAMES = {"a": "A", "ay": "A", "eh": "A", "b": "B", "bee": "B", "be": "B", "c": "C", "see": "C", "sea": "C", "cee": "C",
                 "d": "D", "dee": "D", "e": "E", "f": "F", "eff": "F", "g": "G", "gee": "G", "h": "H"}


def fuzzy_answer(text: str, letters: list[str]) -> str | None:
    """An answer to "Which room: A or B?" that Whisper garbled ("Alfa", "alpa", "bee", "Bravvo"): the OPTION that is clearly closest by
    spelling (difflib ratio >= 0.6, runner-up at least 0.1 worse), looking at the whole short answer and at each word. Only short answers
    (<= 3 words). The caller always reads the choice back, so a wrong match cannot move the robot."""
    from difflib import SequenceMatcher
    t = re.sub(r"[^a-z ]", " ", text.lower()).split()
    if not t or len(t) > 3 or not letters:
        return None
    t = [w for w in t if w not in ("room", "the", "to", "go")] or t
    cands = t + ["".join(t)]
    scores: dict[str, float] = {}
    for ch in letters:
        names = [NATO[ch].lower(), ch.lower()] + [k for k, v in _LETTER_NAMES.items() if v == ch and len(k) > 1]
        scores[ch] = max((1.0 if c == n else SequenceMatcher(None, c, n).ratio()) for c in cands for n in names
                         if len(n) > 1 or c == n)
    ranked = sorted(scores.items(), key=lambda kv: -kv[1])
    best = ranked[0]
    if best[1] < 0.6 or (len(ranked) > 1 and ranked[1][1] > best[1] - 0.1):
        return None
    return best[0]


def answer_hint(letters: list[str]) -> str:
    """Whisper initial_prompt while "Which room?" is pending: the expected answers."""
    return " ".join([f"{NATO[l]}." for l in letters if l in NATO] + [f"Room {l}." for l in letters] + [f"{l}." for l in letters]
                    + ["Yes. No. Stop."])


def canon_place(name: str) -> str:
    """Display form of a place name: a room letter keeps its capital ("room b" -> "room B"). Matching stays case-insensitive elsewhere."""
    return re.sub(r"\b(room) ([a-z])\b", lambda m: f"{m.group(1)} {m.group(2).upper()}", name)
