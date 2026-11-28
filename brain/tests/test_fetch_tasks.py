"""Multi-step fetch commands (brain/fetch/tasks.py + FetchSession.run_plan) on the FAKE robot (FetchMock, lab4_fetch GT, virtual
time): plan = ordered existing skills; constrained JSON validated (no invented objects / steps), keyword fallback; every step checked
before it starts (stop, battery, time, gripper); stop wins; one honest line per step."""
import json

import pytest

from brain.fetch.mock import FetchMock
from brain.fetch.session import FetchSession
from brain.fetch.tasks import TaskPlanner, default_names, keyword_plan, schema, validate
from brain.fetch.mock import ROOMS

NAMES = default_names()
TWO = "bring me the blue can, then the red cube"
PUT = "pick up the blue can and put it in the kitchen"


def S(text):
    return {"skill": "bring", "object": None, "to": "user"} if text is None else text


def session(faults=None, llm=False, **robot):
    r = FetchMock.lab4_fetch(faults=faults or {})
    for k, v in robot.items():
        setattr(r, k, v)
    return FetchSession(r, tasks=TaskPlanner(llm, names=NAMES)), r


class Fake:
    def __init__(self, out):
        self.out, self.n = out, 0

    def __call__(self, system, user):
        self.n += 1
        if isinstance(self.out, Exception):
            raise self.out
        return self.out if isinstance(self.out, str) else json.dumps(self.out)


# ------------------------------------------------------------------ planning
@pytest.mark.parametrize("text,skills", [
    (TWO, [("bring", "can", "user"), ("bring", "small_box", "user")]),
    (PUT, [("pick_up", "can", None), ("put", None, "room D")]),
    ("pick up the blue can and then hand it to me", [("pick_up", "can", None), ("hand_over", None, "user")]),
    ("grab the red cube, then put it in room B", [("pick_up", "small_box", None), ("put", None, "room B")]),
    ("bring the blue can to the office and then bring me the red cube", [("bring", "can", "room B"), ("bring", "small_box", "user")]),
])
def test_keyword_plans(text, skills):
    p, why = validate(keyword_plan(text, NAMES), text, NAMES, ROOMS)
    assert why is None, why
    assert [(s["skill"], (s["object"] or {}).get("class"), s["to"]) for s in p["steps"]] == skills


@pytest.mark.parametrize("text", ["bring me the blue can", "bring me the can and the cube", "pick up the can", "what is in the kitchen?"])
def test_single_step_texts_are_not_plans(text):
    assert TaskPlanner(False, names=NAMES).plan(text) is None


@pytest.mark.parametrize("plan,why", [
    ({"steps": [{"skill": "bring", "object": {"class": "can", "color": "blue"}, "to": "user"},
                {"skill": "bring", "object": {"class": "mini_cone", "color": "orange"}, "to": "user"}]}, "unsaid object"),       # invented object
    ({"steps": [{"skill": "bring", "object": {"class": "can", "color": "blue"}, "to": "user"},
                {"skill": "bring", "object": {"class": "small_box", "color": "red"}, "to": "user"},
                {"skill": "put", "object": None, "to": "home"}]}, "put with an empty gripper"),                                    # invented step
    ({"steps": [{"skill": "bring", "object": {"class": "can", "color": "blue"}, "to": "user"},
                {"skill": "bring", "object": {"class": "can", "color": "blue"}, "to": "user"}]}, "unsaid object"),                 # cube dropped, can twice
    ({"steps": [{"skill": "pick_up", "object": {"class": "can"}, "to": None},
                {"skill": "pick_up", "object": {"class": "small_box"}, "to": None}]}, "invented step pick_up"),                    # no pick verb said
    ({"steps": [{"skill": "bring", "object": {"class": "can", "color": "blue"}, "to": "room C"},
                {"skill": "bring", "object": {"class": "small_box", "color": "red"}, "to": "user"}]}, "unsaid room"),
    ("not json", "bad json"),
])
def test_validator_rejects(plan, why):
    p, reason = validate(plan, TWO, NAMES, ROOMS)
    assert p is None and reason.startswith(why), reason


def test_validator_gripper_order():
    t = "pick up the blue can and pick up the red cube"
    p, why = validate(keyword_plan(t, NAMES), t, NAMES, ROOMS)
    assert p is None and why == "two things at once"


def test_llm_valid_plan_is_used_and_invalid_falls_back_to_keywords():
    good = {"steps": [{"skill": "pick_up", "object": {"class": "can"}, "to": None}, {"skill": "put", "object": None, "to": "kitchen"}]}
    pl = TaskPlanner(Fake(good), names=NAMES)
    p = pl.plan(PUT)
    assert pl.last_source == "llm" and p["steps"][1]["to"] == "room D" and p["steps"][0]["object"]["color"] == "blue"   # user's colour wins
    bad = {"steps": [{"skill": "bring", "object": {"class": "toolbox"}, "to": "user"}, {"skill": "put", "object": None, "to": "kitchen"}]}
    pl = TaskPlanner(Fake(bad), names=NAMES)
    assert pl.plan(PUT)["steps"][0]["skill"] == "pick_up" and pl.last_source == "keyword"
    pl = TaskPlanner(Fake(OSError("refused")), names=NAMES)
    assert pl.plan(TWO) is not None and pl.last_source == "keyword"


def test_ollama_request_uses_the_schema():
    pl = TaskPlanner(None, names=NAMES, model="gurdy-sft-400")
    b = pl.llm.body("s", "u")
    assert b["format"] == schema() and b["model"] == "gurdy-sft-400"


# ------------------------------------------------------------------ running on the fake robot
def test_two_bring_steps():
    s, r = session()
    assert s.handle(TWO) == "Here you are: the blue can. Here you are: the red cube."
    assert [x["status"] for x in s.plan_log] == ["done", "done"] and r.carrying is None


def test_second_step_fails_first_still_reported_done():
    s, r = session({"not_found": {"red_cube"}})
    reply = s.handle(TWO)
    assert reply.startswith("Here you are: the blue can. The red cube: I couldn't find the red cube")
    assert [x["status"] for x in s.plan_log] == ["done", "failed"]


def test_pick_and_put_in_the_kitchen_by_ground_truth():
    s, r = session()
    reply = s.handle(PUT)
    assert r.objects["blue_can"].room == "room D" and r.carrying is None
    assert "I put the blue can down in room D." in reply


def test_put_after_a_failed_pick_is_skipped_and_says_why():
    s, r = session({"grasp_fail": {"blue_can": 9}})
    reply = s.handle(PUT)
    assert [x["status"] for x in s.plan_log] == ["failed", "skipped"]
    assert reply.startswith("The blue can: I couldn't get a grip on the blue can")
    assert reply.endswith("Put it in room D (the kitchen): I'm not holding anything, so there was nothing to take there.")
    assert r.objects["blue_can"].room == "room A"


def test_gripper_busy_before_a_bring():
    s, r = session()
    r.carrying = "red_can"
    reply = s.handle(TWO)
    assert reply.startswith("The blue can: I'm still holding the red can, so I didn't start this one.")
    assert all(x["status"] == "skipped" for x in s.plan_log)


def test_low_battery_stops_before_the_next_step_with_the_measured_level():
    s, r = session(battery=40.0, battery_per_m=6.0)
    reply = s.handle(TWO)
    assert [x["status"] for x in s.plan_log] == ["done", "stopped"]
    lvl = round(r.battery)
    assert f"My battery is at {lvl} %, too low to start another step, so I stopped. Not done: bring the red cube to you." in reply


def test_time_budget_stops_before_the_next_step():
    s, r = session()
    s.MAX_TASK_S = 10.0
    reply = s.handle(TWO)
    assert [x["status"] for x in s.plan_log] == ["done", "stopped"] and "so I stopped before the next step. Not done: bring the red cube" in reply


def test_stop_wins_during_a_step():
    s, r = session({"estop_at_s": 1.0})
    t_before = r.t
    reply = s.handle(TWO)
    assert reply.startswith("I stopped (emergency stop) during: bring the blue can to you.")
    assert "Not done: bring the blue can to you; bring the red cube to you." in reply
    assert [x["status"] for x in s.plan_log] == ["stopped"] and r.t - t_before < 60      # nothing after the stop ran


def test_stop_latched_before_a_step():
    s, r = session({"estop_at_s": 0.0})
    r.t = 0.0
    reply = s.handle(TWO)
    assert reply.startswith("I stopped (emergency stop). Not done: bring the blue can to you; bring the red cube to you.")


def test_ambiguity_mid_plan_asks_and_says_what_is_not_started():
    s, r = session()
    reply = s.handle("bring me the can, then the red cube")         # two cans in the world
    assert "Which one?" in reply and "(Not started yet: bring the red cube to you.)" in reply


def test_safety_refusals_are_never_planned():
    t = "pick up the blue can and then throw it at the wall"
    s, r = session()
    expected = FetchSession(FetchMock.lab4_fetch()).handle(t)            # the router's own refusal
    assert s.handle(t) == expected and s.plan_log == [] and r.carrying is None and "throw" in expected.lower()


def test_flag_off_unchanged():
    r1 = FetchMock.lab4_fetch()
    r2 = FetchMock.lab4_fetch()
    a = FetchSession(r1).handle("bring me the blue can")
    b = FetchSession(r2, tasks=TaskPlanner(False, names=NAMES)).handle("bring me the blue can")
    assert a == b


# ------------------------------------------------------------------ the same plan on the ROS fetch robot (RosBackend + FakePort + FakeArmPort)
def test_plan_on_the_ros_fetch_robot_and_estop_latched():
    from brain.tests.test_ros_fetch import rig
    s, rob, arm, fp = rig()
    s.tasks = TaskPlanner(False, names=NAMES)
    reply = s.handle("pick up the green cone and put it in room A")
    assert [x["status"] for x in s.plan_log] == ["done", "done"], reply
    assert reply.endswith("I put the green mini cone down in room A.") and rob.carrying is None and arm.held is None
    assert rob.battery_pct() is None or isinstance(rob.battery_pct(), (int, float))
    s2, rob2, arm2, fp2 = rig()
    s2.tasks = TaskPlanner(False, names=NAMES)
    fp2.estop = True                                                     # the e-stop is latched before the request
    reply = s2.handle("pick up the green cone and put it in room A")
    assert reply.startswith("I stopped (emergency stop). Not done: pick up the green cone; put it in room A (the hallway).") and arm2.held is None
