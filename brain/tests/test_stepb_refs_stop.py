"""Step B fixes 1 + 2: "that other room" without an earlier room asks; the stop explanation comes from the last stop reason, not a tool name."""
import pytest

from brain.agent import Agent, ScriptedLLM, rooms
from brain.agent.stopwords import is_stop_why

from test_safety import make

LAB4 = ["home", "room A", "room B", "room C", "room D"]


def call(tool, **args):
    return {"tool": tool, "args": args}


def agent4(plans=None):
    b, l, s = make()
    for r in ("room C", "room D"):
        b.places[r] = b.places["room B"]
    llm = ScriptedLLM(plans=plans or [{"calls": [call("go_to", place="room B")]}] * 6)
    return b, Agent(l, llm, s), llm


# ---------------------------------------------------------------- 1: room references (pure)
@pytest.mark.parametrize("text", ["go into that other room", "go to the other room", "head to another room", "go to that room", "move to the same room"])
def test_reference_without_earlier_room_asks_and_lists_rooms(text):
    r = rooms.room_reference(text, LAB4, None)
    assert r.question == "Which room: A, B, C or D?" and r.text is None and r.template.endswith("room")


def test_that_room_resolves_to_the_last_room():
    r = rooms.room_reference("go back into that room", LAB4, "C")
    assert r.text == "go back into room C"


def test_other_room_with_two_rooms_resolves_to_the_remaining_one():
    r = rooms.room_reference("go to the other room", ["home", "room A", "room B"], "A")
    assert r.text == "go to room B"


def test_other_room_with_several_candidates_asks_among_them():
    r = rooms.room_reference("go to the other room", LAB4, "B")
    assert r.question == "Which room: A, C or D?" and r.letters == ["A", "C", "D"]


def test_no_reference_when_a_letter_or_name_follows():
    for t in ("go to room C", "the other room B", "that room Charlie", "go to the chair", "what is in the room"):
        assert rooms.room_reference(t, LAB4, "B") is None, t


# ---------------------------------------------------------------- 1: in the agent
def test_agent_asks_instead_of_guessing_and_never_calls_the_planner():
    b, a, llm = agent4()
    r = a.say("go into that other room")
    assert r.reply == "Which room: A, B, C or D?" and not llm.plan_calls and b.task is None
    r2 = a.say("Charlie")                                   # the answer completes the held command
    assert llm.plan_calls and llm.plan_calls[0]["utterance"] == "go into room C"


def test_agent_resolves_that_room_after_an_earlier_room():
    b, a, llm = agent4()
    a.say("go to room D")
    llm.plan_calls.clear()
    a.say("go into that room")
    assert llm.plan_calls[0]["utterance"] == "go into room D"


def test_agent_other_room_after_room_b_lists_the_remaining_rooms():
    b, a, llm = agent4()
    a.say("go to room B")
    r = a.say("now the other room")
    assert r.reply == "Which room: A, C or D?"


# ---------------------------------------------------------------- 2: stop explanation
@pytest.mark.parametrize("text", ["why are you standing still", "what made you stop", "what made you stand still", "why did you stop",
                                  "why aren't you moving", "what stopped you", "why did you stand still", "how come you stopped"])
def test_stop_why_questions(text):
    assert is_stop_why(text), text


@pytest.mark.parametrize("text", ["stop", "please stop", "go to room B", "why is the box red", "what is in room A", "where are you"])
def test_not_stop_why(text):
    assert not is_stop_why(text), text


def test_after_a_user_stop_the_answer_names_the_user_and_the_command():
    b, a, llm = agent4()
    a.say("go to room B")
    r = a.say("please stop")
    assert r.reply == "Stopped."
    why = a.say("what made you stand still")
    assert why.reply.startswith('You asked me to stop ("please stop")') and "go_to" not in why.reply and not llm.plan_calls[1:]
    assert a.last_stop["reason"] == "user_stop"


def test_stop_reasons_from_events():
    b, a, llm = agent4()
    a.last_stop = None
    a._note_stops([{"type": "task_finished", "data": {"tool": "go_to", "stopped_reason": "reached"}}])
    assert a.last_stop["reason"] == "goal_reached" and "reached my goal" in a._stop_explanation()
    a._note_stops([{"type": "task_failed", "data": {"tool": "go_to", "stopped_reason": "no_go_zone"}}])
    assert "no-go zone" in a._stop_explanation()
    a._note_stops([{"type": "task_failed", "data": {"tool": "go_to", "stopped_reason": "blocked"}}])
    assert "blocked" in a._stop_explanation()
    a._note_stops([{"type": "task_failed", "data": {"tool": "go_to", "stopped_reason": "timeout"}}])
    assert "ran out of time" in a._stop_explanation()
    a._note_stops([{"type": "estop", "text": "Emergency stop engaged.", "data": {}}])
    assert "emergency stop" in a._stop_explanation()


def test_cancel_bookkeeping_does_not_overwrite_a_user_stop():
    b, a, llm = agent4()
    a.say("go to room B"); a.say("stop")
    a._note_stops([{"type": "task_failed", "data": {"tool": "go_to", "stopped_reason": "cancelled"}}])
    assert a.last_stop["reason"] == "user_stop"


def test_no_stop_on_record_falls_through_to_the_planner():
    b, a, llm = agent4(plans=[{"calls": [call("why", about="stop")]}])
    a.say("why did you stop")
    assert llm.plan_calls
