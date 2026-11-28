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


def test_ambiguity_mid_plan_asks_and_says_what_comes_after():
    # changed 2026-11-28 (resumable questions): the plan now WAITS for the answer instead of stopping
    s, r = session()
    reply = s.handle("bring me the can, then the red cube")         # two cans in the world
    assert "Which one?" in reply and "(After that: bring the red cube to you.)" in reply and s.plan_pending is not None


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



# ------------------------------------------------------------------ resumable "which one?" mid-plan (2026-11-28)
class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def asked(faults=None, text="bring me the can, then the red cube"):
    s, r = session(faults)
    s.clock = Clock()
    q = s.handle(text)
    assert q.startswith("I know more than one can: the blue can in room A or the red can in room C. Which one?"), q
    return s, r


def test_answer_resumes_with_the_remaining_steps_without_repeating():
    s, r = asked()
    reply = s.handle("the red one")
    assert reply == "Here you are: the red can. Here you are: the red cube."
    assert [x["status"] for x in s.plan_log] == ["asked", "done", "done"] and s.plan_pending is None and s.pending is None
    assert r.objects["red_can"].with_user or r.carrying is None


def test_question_in_the_second_step_does_not_repeat_the_first():
    s, r = session()
    s.clock = Clock()
    first = s.handle("bring me the red cube, then the can")
    assert first.startswith("Here you are: the red cube. I know more than one can")
    reply = s.handle("the blue one")
    assert reply == "Here you are: the blue can." and "red cube" not in reply


def test_answer_still_ambiguous_asks_again_with_a_new_timer():
    s, r = session()
    s.clock = Clock()
    q = s.handle("bring me the cone, then the red cube")             # two traffic cones + two mini cones
    assert "Which one?" in q and s.plan_pending is not None
    s.clock.t += 50
    again = s.handle("the mini cone")                                # narrows to the two mini cones: asked again
    assert again.startswith("I know more than one mini cone") and again.endswith("Which one?") and s.plan_pending is not None
    s.clock.t += 50                                                  # 100 s after the first question, 50 s after the second
    assert s.poll() is None
    assert s.handle("the green one") == "Here you are: the green mini cone. Here you are: the red cube."


def test_question_times_out_after_60_s_and_the_plan_stops_honestly():
    s, r = asked()
    s.clock.t += 59
    assert s.poll() is None
    s.clock.t += 2
    msg = s.poll()
    assert msg == ("Nobody answered my question for 60 seconds, so I stopped the plan. "
                   "Not done: bring the can to you; bring the red cube to you.")
    assert s.poll() is None and s.plan_pending is None and s.pending is None and r.carrying is None
    assert s.handle("the red one") is None                           # a late answer is not a command: nothing moves


def test_timeout_noticed_by_the_next_message_then_the_message_is_handled():
    s, r = asked()
    s.clock.t += 120
    reply = s.handle("bring me the blue can")
    assert reply.startswith("Nobody answered my question for 60 seconds, so I stopped the plan. Not done: ")
    assert reply.endswith("Here you are: the blue can.")


def test_stop_still_wins_while_waiting_for_the_answer():
    s, r = asked()
    r.faults["estop_at_s"] = r.t                                     # the e-stop latched while the robot waited
    reply = s.handle("the red one")
    assert reply == "I stopped (emergency stop). Not done: bring the can to you; bring the red cube to you."
    assert r.carrying is None and s.plan_pending is None


def test_a_new_command_instead_of_an_answer_drops_the_rest_of_the_plan():
    s, r = asked()
    reply = s.handle("bring me the yellow mini toolbox")
    assert reply.startswith("I dropped the rest of that plan. Not done: bring the can to you; bring the red cube to you.")
    assert "yellow mini toolbox" in reply and s.plan_pending is None


def test_the_answer_cannot_add_an_object():
    s, r = asked()
    reply = s.handle("the green one")                                # no green can exists: not one of the candidates
    assert reply == "I dropped the rest of that plan. Not done: bring the can to you; bring the red cube to you."
    assert r.carrying is None and not any(o.with_user for o in r.objects.values())


def test_agent_pump_announces_the_timeout_once():
    from brain.agent.graph import Agent
    from brain.agent.llm import KeywordLLM
    from brain.mock import MockBackend, gazebo_lab4
    from brain.tools import ToolLayer
    be = MockBackend(gazebo_lab4())
    be.fetch_session, r = session()
    be.fetch_session.clock = Clock()
    ag = Agent(ToolLayer(be), KeywordLLM(be.known_places))
    assert "Which one?" in ag.say("bring me the can, then the red cube").reply
    assert not any("Nobody answered" in m for m in ag.pump())
    be.fetch_session.clock.t += 61
    msgs = ag.pump()
    assert sum("Nobody answered my question for 60 seconds" in m for m in msgs) == 1
    assert not any("Nobody answered" in m for m in ag.pump())
