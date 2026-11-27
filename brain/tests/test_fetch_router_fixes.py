"""Fetch router fixes (2026-11-27): the 5 mis-parses found while writing the picker DEV set, fixed by general rules,
plus the multi-object policy (one after another; holding two things -> ask)."""
import pytest

from brain.fetch.mock import FetchMock
from brain.fetch.router import find_object, find_objects, joined_objects, parse
from brain.fetch.session import FetchSession


def F(cls, color=None, room=None, mode="user", dest=None):
    o = {"class": cls, **({"color": color} if color else {}), **({"room": room} if room else {})}
    return {"intent": "fetch", "object": o, "mode": mode, **({"room": dest} if mode == "room" else {})}


def run(*turns, faults=None):
    s = FetchSession(FetchMock.lab4_fetch(faults or {}))
    return [s.handle(t) for t in turns], s


# ------------------------------------------------------------------ rule 1: a named object turns "hand X ..." into a fetch
@pytest.mark.parametrize("text,exp", [
    ("could you hand the green cone to me", F("traffic_cone", "green")),     # the session maps it to the green MINI cone
    ("hand over the red can", F("can", "red")),
    ("give the blue cube to me", F("small_box", "blue")),
])
def test_hand_with_a_named_object_is_a_fetch(text, exp):
    assert parse(text) == exp


@pytest.mark.parametrize("text", ["hand it to me", "hand it over", "give it to me", "hand me that"])
def test_hand_without_an_object_is_still_the_hand_over(text):
    assert parse(text) == {"intent": "hand_over"}


def test_hand_the_held_object_hands_it_over():
    (r1, r2), s = run("pick up the green mini cone", "could you hand the green cone to me")
    assert r2 == "Here you are: the green mini cone." and s.r.carrying is None


# ------------------------------------------------------------------ rule 2: bring / fetch / hand / give / pass default to the user
@pytest.mark.parametrize("text,exp", [
    ("please fetch that blue tin you saw in room A", F("can", "blue", room="room A")),
    ("bring the blue cube", F("small_box", "blue")),
    ("fetch the orange mini cone", F("mini_cone", "orange")),
])
def test_delivery_verbs_default_to_the_user(text, exp):
    assert parse(text) == exp


@pytest.mark.parametrize("text", ["get the red cube", "take the blue can", "grab the orange cone", "pick up the red cube"])
def test_get_take_grab_without_destination_still_pick(text):
    assert parse(text)["intent"] == "pick"


# ------------------------------------------------------------------ rule 3: an explicit destination room beats "for me"
@pytest.mark.parametrize("text,dest", [
    ("the yellow mini toolbox, carry it to room A for me", "room A"),
    ("could you take the red can to room B for me", "room B"),
    ("bring the blue cube into room D for me please", "room D"),
])
def test_destination_room_beats_for_me(text, dest):
    p = parse(text)
    assert p["mode"] == "room" and p["room"] == dest


def test_source_room_and_user_words_still_work():
    assert parse("bring me the can in room C") == F("can", room="room C")
    assert parse("take the red can in room C to room B") == F("can", "red", room="room C", mode="room", dest="room B")
    assert parse("put the cube in room B") == F("small_box", mode="room", dest="room B")


# ------------------------------------------------------------------ rule 4: the modal verb "can" is not an object
@pytest.mark.parametrize("text,exp", [
    ("can you lift the blue can and wait", {"class": "can", "color": "blue"}),
    ("can I have the red can", {"class": "can", "color": "red"}),
    ("you can grab the red cube", {"class": "small_box", "color": "red"}),
])
def test_modal_can_is_skipped(text, exp):
    assert find_object(text) == exp


def test_can_as_an_object_still_found():
    assert find_object("bring me the can") == {"class": "can"}
    assert find_object("the cans") == {"class": "can"}


def test_objects_are_found_in_order_and_longest_alias_wins_its_span():
    objs = find_objects("put the mini toolbox next to the toolbox")
    assert [o["class"] for o in objs] == ["mini_toolbox", "toolbox"]
    assert find_object("pick up the cube next to the barrel") == {"class": "small_box"}   # first mention, was 'barrel'


# ------------------------------------------------------------------ rule 5: several objects
def test_two_objects_joined_by_and_are_one_after_another():
    p = parse("take the red can and the blue cube to me")
    assert p == {"intent": "multi", "items": [F("can", "red"), F("small_box", "blue")]}
    (r,), s = run("take the red can and the blue cube to me")
    assert r == "Here you are: the red can. Here you are: the blue cube." and s.r.carrying is None


def test_per_object_source_rooms_and_shared_destination():
    p = parse("bring the can in room C and the cube in room D to room B")
    assert p["items"] == [F("can", room="room C", mode="room", dest="room B"), F("small_box", room="room D", mode="room", dest="room B")]


def test_a_description_is_not_a_second_request():
    assert [o["class"] for o in joined_objects("pick up the cube next to the barrel")] == ["small_box"]
    assert parse("bring me the red can from beside the toolbox")["intent"] == "fetch"


def test_multi_stops_at_the_first_failure_and_says_what_is_left():
    (r,), s = run("take the red can and the blue cube to me", faults={"drop": {"red_can"}})
    assert "dropped the red can" in r and "haven't done the blue cube" in r and "Here you are" not in r
    assert s.queue == []


def test_multi_with_a_clarification_continues_after_the_answer():
    (r1, r2), s = run("bring me the can and the red cube", "the blue one")
    assert "Which one?" in r1
    assert r2 == "Here you are: the blue can. Here you are: the red cube."


def test_multi_refused_item_stops_the_list_honestly():
    (r,), _ = run("bring me the barrel and the red can")
    assert "too big" in r and "haven't done the red can" in r and "Here you are" not in r


def test_a_new_request_drops_an_interrupted_list():
    (r1, r2), s = run("bring me the can and the red cube", "go to room B")
    assert "Which one?" in r1 and r2 is None and s.queue == []


def test_holding_two_things_is_asked_not_guessed():
    (r,), s = run("pick up the red can and the blue cube")
    assert r == "I can only hold one thing at a time. Which one should I pick up: the red can or the blue cube?"
    assert s.calls == [] and s.r.carrying is None


def test_safety_still_runs_first_on_multi():
    assert parse("throw me the red can and the blue cube")["intent"] == "refuse"
    assert parse("carry the child and the red can to room B")["intent"] == "refuse"
