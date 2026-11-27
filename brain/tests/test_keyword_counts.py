"""rt24 / rt29 (2026-11-27): the keyword planner must never drop a repeat count or a schedule delay and act once, now.
It carries it into the call (patrol loops, schedule delay_s) when the tool can hold it, else it asks / refuses."""
import pytest

from brain.eval.keyword_parser import parse

PLACES = ["room A", "room B", "room C", "room D", "home"]


@pytest.mark.parametrize("text", ["patrol room A and room B 100000 times", "patrol room A 11 times", "patrol room A 2.5 times"])
def test_patrol_count_beyond_the_tool_limit_asks(text):
    r = parse(text, PLACES)
    assert "calls" not in r and "at most 10 loops" in r["reply"]


@pytest.mark.parametrize("text,loops", [("patrol room A and room B 3 times", 3), ("patrol between home and room B twice", 2),
                                        ("patrol room A 10 loops", 10)])
def test_patrol_count_in_range_is_kept(text, loops):
    assert parse(text, PLACES)["calls"][0]["args"]["loops"] == loops


def test_patrol_once_is_one_loop():
    assert "loops" not in parse("patrol room A once", PLACES)["calls"][0]["args"]


def test_count_on_a_tool_without_repeat_is_refused_not_done_once():
    r = parse("walk forward 2 meters 4 times", PLACES)
    assert "calls" not in r and "4 times" in r["reply"]


@pytest.mark.parametrize("text", ["schedule a walk forward 1 meter in 99999999999 seconds", "go to room B in 2 days",
                                  "in 100000 minutes go home"])
def test_delay_beyond_24_h_is_refused(text):
    r = parse(text, PLACES)
    assert "calls" not in r and "24 hours" in r["reply"]


def test_schedule_without_a_delay_asks_when():
    r = parse("schedule a walk forward 1 meter", PLACES)
    assert "calls" not in r and "When" in r["reply"]


@pytest.mark.parametrize("text,delay,tool", [("schedule a walk forward 1 meter in 30 seconds", 30, "walk"),
                                             ("in 5 minutes go to room B", 300, "go_to"), ("after 2 hours return home", 7200, "return_home")])
def test_delay_in_range_becomes_a_schedule_call(text, delay, tool):
    c = parse(text, PLACES)["calls"]
    assert len(c) == 1 and c[0]["tool"] == "schedule" and c[0]["args"]["delay_s"] == delay and c[0]["args"]["call"]["tool"] == tool


def test_two_commands_cannot_be_scheduled_together():
    assert "one command" in parse("in 5 minutes go to room B and then turn left", PLACES)["reply"]


def test_metres_are_not_minutes():
    c = parse("walk forward 2 m", PLACES)["calls"]
    assert c[0]["tool"] == "walk" and c[0]["args"]["distance_m"] == 2.0


@pytest.mark.parametrize("text", ["patrol room A and room B 100000 times", "schedule a walk forward 1 meter in 99999999999 seconds"])
def test_agent_end_to_end_executes_nothing(text):
    from brain.agent.graph import Agent
    from brain.agent.llm import KeywordLLM
    from brain.mock.backend import MockBackend
    from brain.tools.layer import ToolLayer
    b = MockBackend()
    r = Agent(ToolLayer(b), KeywordLLM(b.known_places)).say(text)
    assert r.calls == [] and r.reply
