"""Held-out lab4 v1 failures: object-search tool choice, invented place arguments, room names before the object fuzzy matcher."""
import pytest

from brain.agent import Agent, ScriptedLLM, rooms
from brain.agent.routing import named_objects, route_plan, wants_search

from test_safety import make

PLACES = ["home", "room A", "room B", "room D"]
RED = {"class": "box", "color": "red"}


def call(tool, **args):
    return {"tool": tool, "args": args}


# ---------------------------------------------------------------- routing (pure)
def test_where_is_becomes_find_when_the_user_asks_to_go_and_check():
    r = route_plan("go and check where the yellow box is", [call("where_is", object={"class": "box", "color": "yellow"})], PLACES)
    assert [c["tool"] for c in r.calls] == ["find"] and r.calls[0]["args"]["object"] == {"class": "box", "color": "yellow"}
    assert r.calls[0]["args"]["approach"] is True and r.rule == "where_is->find"


def test_pure_memory_question_keeps_where_is():
    for u in ("where is the red box?", "where can I find the toolbox", "do you know where the barrel is"):
        r = route_plan(u, [call("where_is", object=RED)], PLACES)
        assert [c["tool"] for c in r.calls] == ["where_is"] and not r.changed, u


def test_explore_without_search_becomes_find():
    r = route_plan("look around for the mannequin", [call("explore", area="room A")], PLACES)
    assert r.calls == [{"tool": "find", "args": {"object": {"class": "mannequin"}}}]


def test_plain_explore_stays_explore():
    r = route_plan("explore room B", [call("explore", area="room B")], PLACES)
    assert [c["tool"] for c in r.calls] == ["explore"] and not r.changed
    r = route_plan("have a look around", [call("explore")], PLACES)
    assert r.calls[0]["tool"] == "explore"


def test_invented_place_argument_known_place_becomes_go_to_then_find():
    r = route_plan("find the traffic cone that is in room D", [call("find", object={"class": "traffic_cone"}, place="room D")], PLACES)
    assert r.calls == [call("find", object={"class": "traffic_cone"}, in_place="room D")]


def test_invented_place_argument_unknown_place_is_refused_with_the_known_places():
    r = route_plan("find the cone in the kitchen", [call("find", object={"class": "traffic_cone"}, place="kitchen")], PLACES)
    assert r.calls == [] and "kitchen" in r.reply and "room A" in r.reply


def test_named_objects_exact_words_only():
    assert named_objects("go to charlie") == []
    assert named_objects("find the red barrel") == [{"class": "barrel", "color": "red"}]
    assert named_objects("the fire extinguisher") == [{"class": "fire_extinguisher"}]
    assert wants_search("track down the toolbox") and not wants_search("where can I find the toolbox")


# ---------------------------------------------------------------- room names
@pytest.mark.parametrize("text,want", [("go to charlie", "go to room C"), ("charlie", "go to room C"), ("room c", "go to room C"),
                                       ("Room Charlie", "go to room C"), ("go to c", "go to room C"), ("then Delta", "then room D"),
                                       ("the chair", "the chair"), ("go to the chair", "go to the chair"), ("go to a box", "go to a box"),
                                       ("a", "a"), ("find the chair in room A", "find the chair in room A")])
def test_bare_room_names(text, want):
    assert rooms.bare_rooms(rooms.normalize(text)) == want


def test_go_to_charlie_never_asks_about_a_chair():
    b, l, s = make()
    llm = ScriptedLLM(plans=[{"calls": [call("go_to", place="room B")]}] * 4)
    a = Agent(l, llm, s)
    r = a.say("go to charlie")
    assert "chair" not in r.reply.lower()
    assert "no room C" in r.reply or llm.plan_calls[0]["utterance"] == "go to room C"


def test_charlie_reaches_the_planner_as_room_c_when_the_room_exists():
    b, l, s = make()
    b.places["room C"] = b.places["room B"]
    llm = ScriptedLLM(plans=[{"calls": [call("go_to", place="room C")]}])
    a = Agent(l, llm, s)
    a.say("charlie")
    assert llm.plan_calls[0]["utterance"] == "go to room C"
