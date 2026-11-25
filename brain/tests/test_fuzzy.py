"""Speech-recognition repair: Whisper vocabulary bias + "Did you mean room B?" (never executed without a yes)."""
import pytest

from brain.agent.fuzzy import suggest_object, suggest_place
from brain.agent.graph import Agent
from brain.agent.llm import KeywordLLM
from brain.mock import MockBackend
from brain.tools import ToolLayer
from brain.voice.vocab import hotwords, initial_prompt, vocabulary

PLACES = ["home", "room A", "room B", "doorway"]


@pytest.mark.parametrize("text,want", [("Go to room V and then find the red box.", "room B"), ("go to room e", "room B"),
                                       ("go to room D", "room B"), ("go to room P please", "room B"), ("go to dorway", "doorway")])
def test_place_suggestions(text, want):
    s = suggest_place(text, PLACES)
    assert s and s.options == [want]
    assert want.lower() in s.corrected[want].lower()


@pytest.mark.parametrize("text", ["go to room B", "go to room A", "Go to room b.", "go to room 8", "go to the kitchen", "go to home",
                                  "save this place as room V", "find the red box", "what is the battery"])
def test_no_suggestion_for_known_or_unrelated(text):
    assert suggest_place(text, PLACES) is None


def test_correction_keeps_rest_of_sentence():
    s = suggest_place("Go to room V and then find the red box.", PLACES)
    assert s.corrected["room B"] == "Go to room B and then find the red box."


def test_object_suggestion():
    s = suggest_object("find the extinguiser", PLACES)
    assert s and s.options == ["extinguisher"]
    assert suggest_object("find the red box", PLACES) is None
    assert suggest_object("find the table", PLACES) is None


def _agent():
    b = MockBackend()
    return b, Agent(ToolLayer(b), KeywordLLM(b.known_places))


def test_agent_asks_and_does_not_move_without_yes():
    b, a = _agent()
    r = a.say("go to dorway")
    assert "Did you mean doorway" in r.reply and r.calls == [] and r.llm_calls == 0
    assert a.executor.state == "idle"


def test_agent_runs_corrected_request_after_yes():
    b, a = _agent()
    a.say("go to dorway")
    r = a.say("yes")
    assert any(c["tool"] == "go_to" for c in r.calls) and "doorway" in str(r.calls)


def test_agent_drops_fix_on_other_answer():
    b, a = _agent()
    a.say("go to dorway")
    r = a.say("no, go home")
    assert a.pending_fix is None and "room b" not in str(r.calls).lower()
    r = a.say("yes")                        # a stale yes must not resurrect the old request
    assert not any(c["tool"] == "go_to" for c in r.calls)


def test_stop_beats_pending_fix():
    b, a = _agent()
    a.say("go to dorway")
    r = a.say("stop")
    assert r.reply == "Stopped." and a.pending_fix is None


def test_fix_expires():
    b, a = _agent()
    a.say("go to dorway")
    a.clock.wall = lambda t0=a.clock.wall(): t0 + 100.0       # 100 wall-clock seconds later
    r = a.say("yes")
    assert not any(c["tool"] == "go_to" for c in r.calls)


def test_vocabulary_contains_places_objects_colours():
    v = vocabulary(PLACES)
    for w in ("room A", "room B", "doorway", "home", "barrel", "fire extinguisher", "red", "yellow"):
        assert w in v
    assert "room B" in initial_prompt(PLACES) and "room B" in hotwords(PLACES)


@pytest.mark.parametrize("text", ["go to room B and then come back", "come back", "walk home and halt", "go to the doorway then come home"])
def test_ordinary_words_are_not_places(text):
    assert suggest_place(text, PLACES) is None


from brain.agent.fuzzy import suggest_verb
from brain.voice.vocab import COMMANDS


@pytest.mark.parametrize("text", ["Open room B and then find the red box.", "go do room B", "goto room B", "open the doorway"])
def test_verb_confusion_suggests_go_to(text):
    s = suggest_verb(text, PLACES)
    assert s and s.options[0].startswith("go to ") and s.question.startswith("Did you mean: go to ")


def test_verb_correction_text():
    s = suggest_verb("Open room B and then find the red box.", PLACES)
    assert list(s.corrected.values()) == ["go to room B and then find the red box."]


@pytest.mark.parametrize("text", ["go to room B", "open the log", "please open room B", "what is near you", "open"])
def test_verb_no_suggestion(text):
    assert suggest_verb(text, PLACES) is None


def test_agent_open_room_asks_then_yes_goes():
    b, a = _agent()
    r = a.say("Open room B and then find the red box.")
    assert "Did you mean: go to room B" in r.reply and r.calls == [] and a.executor.state == "idle"
    r = a.say("yes")
    assert any(c["tool"] == "go_to" for c in r.calls)


def test_prompt_has_command_phrases():
    p, h = initial_prompt(PLACES), hotwords(PLACES)
    for w in ("go to", "find", "look for", "where is", "stop", "patrol", "go home", "what is near you"):
        assert w in COMMANDS and w in h.lower() and w in p.lower()
