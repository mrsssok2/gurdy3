"""Phase 9d fetch: specs, safety, state machine, grounding, router, session (mock backend only; stdlib + PyYAML)."""
import pytest

from brain.fetch import catalog as C
from brain.fetch.machine import FetchMachine
from brain.fetch.mock import FetchMock
from brain.fetch.replies import ground, safe
from brain.fetch.router import parse
from brain.fetch.safety import carry_speed, object_allowed, refuse_reason
from brain.fetch.session import FetchSession
from brain.fetch.specs import check


def sess(faults=None):
    return FetchSession(FetchMock.lab4_fetch(faults or {}))


# ------------------------------------------------------------------ specs
def test_specs_strict():
    assert check("pick_up", {"object": {"class": "can", "color": "red"}, "room": "room C"}) == []
    assert check("pick_up", {"object": {"class": "rock"}})                       # unknown class
    assert check("pick_up", {"object": {"class": "can"}, "speed": 1})            # extra key
    assert check("pick_up", {}) and check("place", {})
    assert check("bring_to", {"target": "room"})                                 # room missing
    assert check("bring_to", {"target": "user", "room": "room B"})               # room not allowed
    assert check("bring_to", {"target": "kitchen"})
    assert check("hand_over", {}) == [] and check("hand_over", {"to": "me"})
    assert check("teleport", {})


# ------------------------------------------------------------------ safety
@pytest.mark.parametrize("text", ["throw me the can", "toss the red cube to me", "chuck it over", "fling the cone",
                                  "drop it on him", "lob the mini toolbox into room B"])
def test_no_throwing(text):
    assert refuse_reason(text)[0] == "THROW"


@pytest.mark.parametrize("text", ["pick up the person", "carry the child to room B", "grab the mannequin", "lift me",
                                  "pick her up", "bring the woman here", "carry him home"])
def test_never_people(text):
    r = refuse_reason(text) or (object_allowed("mannequin") if "mannequin" in text else None)
    assert r and r[0] == "PERSON"


@pytest.mark.parametrize("text", ["bring me the red can", "pick up the cube", "hand it to me", "put it in room B"])
def test_normal_requests_not_refused(text):
    assert refuse_reason(text) is None


def test_only_small_classes_and_payload():
    for cls in C.PICKABLE:
        assert object_allowed(cls) is None
    for cls in C.TOO_BIG:
        assert object_allowed(cls)[0] == "TOO_BIG"
    assert object_allowed("can", mass=0.2)[0] == "TOO_HEAVY"
    assert object_allowed("mannequin")[0] == "PERSON"
    assert all(m <= C.MAX_PAYLOAD_KG for m, _, _ in C.PICKABLE.values())


def test_carry_speed_limit():
    assert carry_speed(0.15, True) == C.CARRY_SPEED_MPS and carry_speed(0.15, False) == 0.15
    s = sess()
    s.handle("bring me the red can")
    assert all(v <= C.CARRY_SPEED_MPS for v, carrying in s.r.speeds if carrying)
    assert any(carrying for _, carrying in s.r.speeds)


def test_catalog_matches_arm_and_models():
    np = pytest.importorskip("numpy")  # noqa: F841
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "gurdy_core"))
    sys.path.insert(0, str(root))
    from gurdy_core import arm as A
    a = A.load_arm()
    assert C.MAX_PAYLOAD_KG == pytest.approx(a.payload_max) and C.MAX_GRIP_WIDTH_M == pytest.approx(a.width_max)
    for cls, (m, w, _) in C.PICKABLE.items():
        assert m == pytest.approx(A.PICKABLE[cls]["mass"]) and w == pytest.approx(A.PICKABLE[cls]["grip_w"])


# ------------------------------------------------------------------ state machine + grounding
def test_machine_full_fetch_log_and_confirmations():
    r = FetchMock.lab4_fetch()
    out = FetchMachine(r).fetch("red_can", "user")
    assert out.status == "handed_over" and out.confirmed == {"picked", "handed_over"}
    assert [e["state"] for e in out.log] == ["FIND", "APPROACH", "FINE_ALIGN", "GRASP", "VERIFY_GRASP", "FOLD_ARM", "CARRY", "HAND_OVER"]
    assert r.carrying is None and r.objects["red_can"].with_user


@pytest.mark.parametrize("faults,code,status", [
    ({"not_found": {"red_can"}}, "NOT_FOUND", "failed"), ({"unreachable": {"red_can"}}, "NOT_REACHABLE", "failed"),
    ({"grasp_fail": {"red_can": 3}}, "GRASP_FAILED", "failed"), ({"slip": {"red_can"}}, "GRASP_FAILED", "failed"),
    ({"drop": {"red_can"}}, "DROPPED", "failed"), ({"user_takes": False}, "NOT_TAKEN", "holding"),
    ({"no_path": {"room C"}}, "NO_PATH", "failed")])
def test_machine_failures(faults, code, status):
    out = FetchMachine(FetchMock.lab4_fetch(faults)).fetch("red_can", "user")
    assert out.code == code and out.status == status
    assert "handed_over" not in out.confirmed


def test_regrasp_succeeds_after_one_failure():
    out = FetchMachine(FetchMock.lab4_fetch({"grasp_fail": {"red_cube": 2}})).pick("red_cube")
    assert out.status == "holding" and out.regrasps == 2


def test_dropped_object_moves_and_robot_is_empty():
    r = FetchMock.lab4_fetch({"drop": {"red_can"}})
    out = FetchMachine(r).fetch("red_can", "room", "room B")
    assert out.code == "DROPPED" and r.carrying is None and "placed" not in out.confirmed
    assert (r.objects["red_can"].x, r.objects["red_can"].y) != (1.3, 2.6)


@pytest.mark.parametrize("reply,confirmed,holding,bad", [
    ("Here you are: the red can.", set(), False, True), ("Here you are: the red can.", {"picked", "handed_over"}, False, False),
    ("I picked up the cube.", set(), False, True), ("I couldn't pick up the cube.", set(), False, False),
    ("I put the can down in room B.", {"picked"}, True, True), ("I put the can down in room B.", {"picked", "placed"}, False, False),
    ("Delivered!", {"picked"}, True, True), ("I'm holding the cube.", {"picked"}, False, True)])
def test_grounding_rule(reply, confirmed, holding, bad):
    assert bool(ground(reply, confirmed, holding)) == bad
    assert (safe(reply, confirmed, holding) != reply) == bad


# ------------------------------------------------------------------ router + session
@pytest.mark.parametrize("text,intent", [
    ("bring me the red can", ("fetch", "can", "user")), ("could you fetch me the cube", ("fetch", "small_box", "user")),
    ("put the mini toolbox in room B", ("fetch", "mini_toolbox", "room")), ("take the can in room C to room B", ("fetch", "can", "room")),
    ("bring the blue cube home", ("fetch", "small_box", "home")), ("pick up the orange cone", ("pick", "traffic_cone", None)),
    ("hand it to me", ("hand_over", None, None)), ("put it in room D", ("deliver", None, "room")),
    ("go to room B", None), ("find the red box", None), ("what is in room C", None)])
def test_router(text, intent):
    p = parse(text)
    if intent is None:
        assert p is None
        return
    assert p["intent"] == intent[0]
    if intent[1]:
        assert p["object"]["class"] == intent[1]
    if intent[2]:
        assert p["mode"] == intent[2]


def test_router_object_room_vs_destination():
    p = parse("take the can in room C to room B")
    assert p["object"]["room"] == "room C" and p["room"] == "room B"


def test_session_clarifies_and_resolves():
    s = sess()
    r = s.handle("bring me the can")
    assert "Which one" in r and "blue can in room A" in r and "red can in room C" in r and s.calls == []
    assert s.handle("the red one") == "Here you are: the red can."


def test_session_cone_family_and_size_words():
    s = sess()
    assert "picked up the orange mini cone" in s.handle("pick up the orange cone")
    assert "too big" in sess().handle("bring me the big cone")
    assert "too big" in sess().handle("bring me the fire extinguisher")


def test_session_pronouns_need_something_held():
    s = sess()
    assert s.handle("hand it to me") == "I'm not holding anything."
    assert s.handle("put it in room B") == "I'm not holding anything."
    s.handle("pick up the blue cube")
    assert s.handle("put it in room B") == "I put the blue cube down in room B."
    assert s.r.carrying is None and s.r.objects["blue_cube"].room == "room B"


def test_session_unknown_object_never_substituted():
    assert "won't guess" in sess().handle("bring me the purple cube")


def test_session_busy_when_holding_another():
    s = sess()
    s.handle("pick up the red cube")
    assert "already holding the red cube" in s.handle("pick up the blue can")


def test_every_tool_call_is_valid():
    s = sess()
    for t in ["bring me the red can", "put the mini toolbox in room B", "pick up the blue cube", "bring it home"]:
        s.handle(t)
        assert all(check(c["tool"], c["args"]) == [] for c in s.calls)


# ------------------------------------------------------------------ agent integration (brain.agent.graph._fetch_route)
def test_agent_routes_fetch_only_with_a_fetch_backend():
    pytest.importorskip("langgraph")
    from brain.agent import Agent, KeywordLLM
    from brain.mock import MockBackend
    from brain.tools import ToolLayer
    b = MockBackend()
    a = Agent(ToolLayer(b), KeywordLLM(b.known_places))
    plain = a.say("bring me the red can")
    assert "Here you are" not in plain.reply                       # no fetch backend: unchanged behaviour
    b2 = MockBackend()
    b2.fetch_session = sess()
    a2 = Agent(ToolLayer(b2), KeywordLLM(b2.known_places))
    r = a2.say("bring me the red can")
    assert r.reply == "Here you are: the red can." and [c["tool"] for c in r.calls] == ["pick_up", "bring_to", "hand_over"]
    assert "don't throw" in a2.say("throw the blue can at me").reply
    assert b2.fetch_session.r.objects["blue_can"].with_user is False      # nothing was executed for the refusal
    r3 = a2.say("go to room B")                                     # not a fetch command: the planner path as before
    assert "Here you are" not in r3.reply and r3.calls and r3.calls[0]["tool"] == "go_to"
