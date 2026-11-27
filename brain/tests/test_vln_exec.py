"""VLN execution through the AGENT on the lab4 MOCK backend (brain/vln/executor.py, Agent(vln=...)); no ROS, no LLM
(the DEV oracle or a scripted plan stands in for the model)."""
import json
import math

import pytest

from brain.agent.graph import Agent
from brain.agent.llm import KeywordLLM
from brain.eval import vln_dev as D
from brain.mock import MockBackend, gazebo_lab4
from brain.tools import ToolLayer
from brain.vln.executor import VLNExecutor
from brain.vln.plan import PASS_CHECK, Plan, validate
from brain.vln.planner import VLNPlanner
from brain.vln.world import VLNWorld

R, S, GO = D.R, D.S, D.GO


class Scripted:
    def __init__(self, table):
        self.table = {t: json.dumps(p) for t, p in table.items()}

    def __call__(self, system, user):
        return self.table.get(json.loads(user.split(": ", 1)[1]), '{"kind": "none"}')


def make(table=None, backend_cls=MockBackend, vln=True):
    be = backend_cls(gazebo_lab4())
    be.x, be.y, be.yaw = 0.0, 0.0, 0.0
    for o in be.world.objects:
        be._sight(o)
    llm = D.OracleLLM() if table is None else Scripted(table)
    ag = Agent(ToolLayer(be), KeywordLLM(be.known_places), vln=VLNPlanner(llm) if vln else None)
    return ag, be


def gid(be, cls, color=""):
    return next(o.gid for o in be.world.objects if o.cls == cls and o.color == color)


# ------------------------------------------------------------------ flag
def test_flag_off_means_the_planner_handles_navigation_as_before():
    ag, be = make(vln=False)
    r = ag.say("go to room B")
    assert ag.vln_last is None and ag.vln is None and r.calls and r.calls[0]["tool"] == "go_to"


def test_vln_object_goal_end_to_end():
    ag, be = make()
    r = ag.say("go to the red barrel")
    assert r.reply == "I'm at the red barrel in room D."
    v = ag.vln_last
    assert v.ok and v.rooms_visited == ["room A", "room B", "room D"] and be.zones == []
    assert [c["type"] for c in v.claims] == ["arrived", "in_room"]


def test_no_per_leg_arrival_announcements_afterwards():
    ag, be = make()
    ag.say("go to room B, then room D")
    assert all("arriv" not in m.lower() and "reached" not in m.lower() for m in ag.pump())


# ------------------------------------------------------------------ avoided rooms = temporary zones
def test_avoid_room_becomes_a_temporary_zone_and_is_removed():
    seen = []

    class Spy(MockBackend):
        def add_temp_zone(self, rect, tag, name):
            seen.append((tuple(rect), name))
            return super().add_temp_zone(rect, tag, name)
    ag, be = make(backend_cls=Spy)
    r = ag.say("go to room D but avoid room B")
    assert r.reply == "I'm in room D."
    assert seen == [((2.0, -2.0, 6.0, 2.0), "room B (avoid, this walk only)")]
    assert "room B" not in ag.vln_last.rooms_visited and be.zones == []          # removed at the end
    assert not any(2.0 < x < 6.0 and -2.0 < y < 2.0 for _, x, y in ag.vln_last.trajectory)


def test_avoided_start_room_gets_its_zone_after_leaving():
    seen = []

    class Spy(MockBackend):
        def add_temp_zone(self, rect, tag, name):
            seen.append((self.x, self.y, name))
            return super().add_temp_zone(rect, tag, name)
    ag, be = make(backend_cls=Spy)
    r = ag.say("go to room C without going through room A")
    assert r.reply.startswith("I'm in room A now; I'll leave it the shortest way.") and r.reply.endswith("I'm in room C.")
    assert len(seen) == 1 and seen[0][2] == "room A (avoid, this walk only)"
    assert VLNWorld.from_backend(be).room_of(seen[0][0], seen[0][1]) == "room C" and seen[0][1] > 2.0   # added only after leaving A
    assert be.zones == []


def test_impossible_avoid_is_answered_without_moving():
    ag, be = make()
    r = ag.say("go to room D avoiding room B and room C")
    assert "can't reach room D" in r.reply and (be.x, be.y) == (0.0, 0.0) and be.zones == []


# ------------------------------------------------------------------ room order / pass / re-observation / pose
def test_pass_landmark_comes_within_the_check_distance():
    ag, be = make()
    r = ag.say("go past the blue box into room D")
    assert r.reply == "I'm in room D. On the way I went past the blue box in room B."
    L = next(o for o in be.world.objects if o.cls == "box" and o.color == "blue")
    assert min(math.hypot(x - L.x, y - L.y) for _, x, y in ag.vln_last.trajectory) <= PASS_CHECK


def test_landmark_moved_is_reported_not_claimed():
    ag, be = make()
    be.move_object(gid(be, "barrel", "red"), 4.0, 5.3)             # moved after it was mapped; the map still has the old spot
    r = ag.say("go to the red barrel")
    assert ag.vln_last.code == "LANDMARK_MOVED" and "isn't where I remembered" in r.reply and "I'm at" not in r.reply


def test_landmark_gone_is_not_claimed():
    ag, be = make()
    be.remove_object(gid(be, "mannequin"))
    r = ag.say("take me to the mannequin")
    assert ag.vln_last.code == "LANDMARK_NOT_CONFIRMED" and "couldn't see the mannequin" in r.reply and "I'm at" not in r.reply


def test_room_with_goal_re_observes_its_landmark():
    ag, be = make()
    be.remove_object(gid(be, "barrel", "red"))
    r = ag.say("go to the room with the red barrel")
    assert ag.vln_last.code == "LANDMARK_NOT_CONFIRMED" and "I'm in room D." not in r.reply


def test_arrival_only_after_checking_the_real_pose():
    be = MockBackend(gazebo_lab4())
    be.x, be.y, be.yaw = 0.0, 0.0, 0.0
    w = VLNWorld.from_backend(be)
    plan = validate(json.dumps(GO(S("enter", room="room B"))), "go to room B", w)
    plan.goal = {"kind": "point", "x": 5.5, "y": -1.5, "radius": 0.5, "room": "room B"}   # goal region != where the legs end
    res = VLNExecutor(be, lambda: VLNWorld.from_backend(be)).run(plan, None, "go to room B")
    assert not res.ok and res.code == "NOT_AT_GOAL" and "not where you asked" in res.reply and not res.claims


def test_blocked_leg_is_an_honest_failure():
    ag, be = make()
    from brain.mock.world import Rect
    be.add_obstacle(Rect(1.95, -1.3, 2.05, -0.1))                   # close the A-B doorway
    r = ag.say("go to room B, don't go through room C")
    assert not ag.vln_last.ok and ag.vln_last.code in ("LEG_FAILED", "LEG_REFUSED") and "I'm in room B" not in r.reply
    assert be.zones == []


def test_wrong_room_stops_and_replans_once():
    be = MockBackend(gazebo_lab4())
    be.x, be.y, be.yaw = 0.0, 0.0, 0.0
    for o in be.world.objects:
        be._sight(o)
    w = VLNWorld.from_backend(be)
    raw = json.dumps(GO(S("enter", room="room D")))
    plan = validate(raw, "go to room D", w)
    plan.rooms = ["room A", "room C", "room D"]                     # the legs go through B: the room check must catch it
    res = VLNExecutor(be, lambda: VLNWorld.from_backend(be)).run(plan, raw, "go to room D")
    assert res.replans == 1 and res.ok and res.reply == "I'm in room D."


def test_never_in_is_the_safety_net_when_zones_fail():
    class NoZones(MockBackend):
        def add_temp_zone(self, rect, tag, name):
            return 0                                                # pretend: the zone is not enforced
    be = NoZones(gazebo_lab4())
    be.x, be.y, be.yaw = 0.0, 0.0, 0.0
    w = VLNWorld.from_backend(be)
    plan = validate(json.dumps(GO(S("enter", room="room B"))), "go to room B", w)
    plan.avoid = ["room B"]
    plan.rooms = ["room A", "room B"]
    plan.goal = {"kind": "room", "room": "room B"}
    res = VLNExecutor(be, lambda: VLNWorld.from_backend(be)).run(plan, None, "x")
    assert res.code == "NEVER_IN" and "asked me to avoid" in res.reply and be.task is None


def test_pass_missed_stops_the_walk():
    be = MockBackend(gazebo_lab4())
    be.x, be.y, be.yaw = 0.0, 0.0, 0.0
    plan = Plan(True, waypoints=[{"x": 1.0, "y": 0.0, "kind": "pass", "room": "room A", "label": "the red box in room A",
                                   "landmark": {"id": 99, "class": "box", "x": -1.2, "y": -1.3}},
                                  {"x": 1.0, "y": 1.0, "kind": "enter", "room": "room A", "label": "room A"}],
                goal={"kind": "room", "room": "room A"}, rooms=["room A"])
    res = VLNExecutor(be, lambda: VLNWorld.from_backend(be)).run(plan, None, "x")
    assert res.code == "PASS_MISSED" and "couldn't get within 1.2 m" in res.reply


# ------------------------------------------------------------------ e-stop at any time
@pytest.mark.parametrize("how", ["latch", "stop_tool"])
def test_estop_mid_walk(how):
    class Stopper(MockBackend):
        def advance(self, dt):
            super().advance(dt)
            if self.t > 25.0 and not getattr(self, "_done", False):
                self._done = True
                if how == "latch":
                    self.estop = True
                    self.stop()
                else:
                    self.stop()                                     # AgentHost.estop -> the stop tool, without the agent lock
    ag, be = make(backend_cls=Stopper)
    r = ag.say("go to room D but avoid room B")
    v = ag.vln_last
    assert v.code == "ESTOP" and r.reply.startswith("I stopped (") and "I'm in room" in r.reply
    assert be.task is None and be.zones == [] and v.claims == []
    x0, y0 = be.x, be.y
    be.estop = False
    be.advance(5.0)
    assert (be.x, be.y) == (x0, y0)                                 # nothing keeps walking afterwards


# ------------------------------------------------------------------ answered / not routed
def test_answers_without_moving():
    for text, part in [("go to the cone", "Which one?"), ("go to the chair", "haven't seen the chair"),
                       ("go to the blue barrel but avoid room B", "avoid room B")]:
        ag, be = make()
        r = ag.say(text)
        assert part in r.reply and (be.x, be.y) == (0.0, 0.0), text


def test_busy_backend_is_not_taken_over():
    ag, be = make()
    be.go_to(place="room C")
    assert ag._vln_route("go to room B") is None


def test_dev_set_end_to_end_on_the_mock():
    out = D.run_mock(lambda: VLNPlanner(D.OracleLLM()), verbose=False)
    assert out["ok"] == out["n"], [r for r in out["rows"] if not r["ok"]]
    assert out["success_rate"] == 1 and out["false_claims"] == 0 and out["avoid_violation_rate"] == 0
    assert out["answer_accuracy"] == 1 and out["spl"] > 0.95
