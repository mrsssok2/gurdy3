"""VLN on the ROS backend (RosBackend + FakePort, no ROS): temporary rectangular no-go zones, the world model from the
places file + /map + semantic map, and walks through the agent; plus the sigma-aware clearance (vln_plan.md section 12)."""
import json
import math
import random
import shutil
from pathlib import Path

import yaml

from brain.agent.graph import Agent
from brain.agent.llm import KeywordLLM
from brain.eval import vln_dev as D
from brain.mock import gazebo_lab4
from brain.ros.backend import RosBackend
from brain.ros.fake_port import FakePort
from brain.tools import ToolLayer
from brain.vln.executor import VLNExecutor
from brain.vln.plan import validate
from brain.vln.planner import VLNPlanner
from brain.vln.world import VLNWorld, doors_from_map

ROOT = Path(__file__).resolve().parents[2]
LAB = gazebo_lab4()
GT = json.loads((ROOT / "docs/results/data/lab4/lab4_gt.json").read_text())
R, S, GO = D.R, D.S, D.GO
BLUE_LEFT = json.dumps(GO(S("stop", relation="left_of", of=[R("barrel", "blue")])))


class LabPort(FakePort):
    """FakePort with the lab4 walls on /map and a fake detector: objects in the camera cone (3.5 m, +-45 deg, line of
    sight) get last_seen = now."""

    def map_cell(self, x, y):
        if not LAB.bounds.contains(x, y):
            return "unknown"
        return "occupied" if LAB._cell(x, y) in LAB.sight_blockers else "free"

    def semantic_objects(self):
        for o in self.objects:
            d = math.hypot(o["x"] - self.x, o["y"] - self.y)
            b = math.atan2(o["y"] - self.y, o["x"] - self.x) - self.yaw
            b = math.atan2(math.sin(b), math.cos(b))
            if d <= 3.5 and abs(b) <= math.radians(45) and LAB.line_of_sight((self.x, self.y), (o["x"], o["y"])):
                o["last_seen"] = self.t
        return super().semantic_objects()


def port(objects=None, **kw):
    fp = LabPort(pose=(0.0, 0.0, 0.0), **kw)
    objs = objects if objects is not None else [
        {"id": i + 1, "class": o["class"], "color": o.get("color", ""), "x": o["x"], "y": o["y"], "last_seen": -1.0, "hits": 20,
         "sigma_m": 0.05} for i, o in enumerate(GT["objects"])]
    fp.set_objects(objs)
    return fp


def backend(tmp_path=None, fp=None):
    places = ROOT / "maps/lab4.places.yaml"
    if tmp_path is not None:
        shutil.copy(places, tmp_path / "places.yaml")
        places = tmp_path / "places.yaml"
    fp = fp or port()
    return RosBackend(fp, places), fp


# ------------------------------------------------------------------ 1. clearance with position uncertainty
def noisy_world(seed, sigma=0.04, k=None, margin=None):
    rng = random.Random(seed)
    w = VLNWorld.lab4()
    if k is not None:
        w.sigma_k = k
    if margin is not None:
        w.margin = margin
    for o in w.objects:
        o["x"] += rng.gauss(0, sigma)
        o["y"] += rng.gauss(0, sigma)
        o["sigma"] = sigma
    return w


def test_blue_barrel_gap_is_refused_consistently_with_sigma():
    """0.47 m gap between the blue barrel and the wall: without the sigma term the answer flips with 4 cm map noise; with the
    default sigma_k (3) it is NO_FREE_SPOT for every one of 300 noise draws."""
    flips = sum(validate(BLUE_LEFT, "stop to the left of the blue barrel", noisy_world(s, k=0.0)).error != "NO_FREE_SPOT" for s in range(300))
    refused = sum(validate(BLUE_LEFT, "stop to the left of the blue barrel", noisy_world(s)).error == "NO_FREE_SPOT" for s in range(300))
    assert flips > 30 and refused == 300


def test_exact_map_and_roomy_spots_are_unaffected():
    assert validate(BLUE_LEFT, "stop to the left of the blue barrel", VLNWorld.lab4()).error == "NO_FREE_SPOT"
    for rel, word in (("right_of", "right"), ("behind", "behind")):         # roomy spots (DEV r02, r09): 200 / 200 accepted
        raw = json.dumps(GO(S("stop", relation=rel, of=[R("mannequin")])))
        assert all(validate(raw, f"stop {word} of the mannequin", noisy_world(s)).ok for s in range(100)), rel


def test_a_spot_the_robot_does_not_fit_is_never_accepted():
    """The safe direction: with sigma, acceptance of the no-room blue-barrel spot needs noise beyond 3 sigma; over 1000 draws:
    none. (A spot whose spare clearance is ABOUT sigma_k * sigma, like DEV r04, still flips: on the refusing side.)"""
    assert not any(validate(BLUE_LEFT, "stop to the left of the blue barrel", noisy_world(s)).ok for s in range(1000))


def test_margin_parameter_and_sigma_terms():
    w = VLNWorld.lab4()
    assert w.free(1.6, 0.0) and not w.free(1.6, 0.0, ref_sigma=0.05)         # 0.40 from the wall: 0.35 + 3 x 0.05 fails
    w.margin = 0.1
    assert not w.free(1.6, 0.0)
    w2 = VLNWorld.lab4()
    o = next(o for o in w2.objects if o["class"] == "mannequin")
    assert w2.free(o["x"] + 0.45, o["y"]) and (o.update(sigma=0.05) or True) and not w2.free(o["x"] + 0.45, o["y"])
    mann = json.dumps(GO(S("stop", relation="left_of", of=[R("mannequin")])))
    big = VLNWorld.lab4()
    big.margin = 1.0
    assert validate(mann, "stop to the left of the mannequin", big).error == "NO_FREE_SPOT"


# ------------------------------------------------------------------ 2a. temporary zones on RosBackend
def test_temp_zone_enforced_like_user_zones_and_never_saved(tmp_path):
    be, fp = backend(tmp_path)
    v0 = be.zones_version
    be.add_temp_zone((2.0, -2.0, 6.0, 2.0), "vln1", "room B (avoid, this walk only)")
    assert be.zones_version == v0 + 1 and be._in_zone(4.0, 0.0) and not be._in_zone(0.0, 0.0)
    r = be.go_to(point={"x": 4.0, "y": 0.0})
    assert not r.ok                                                    # a goal inside the temporary zone is refused
    be.no_go_zone("add", place="room C")                               # a USER zone: saved; the temporary one is not
    saved = yaml.safe_load((tmp_path / "places.yaml").read_text())
    assert [z["name"] for z in saved["no_go_zones"]] == ["room C"]
    assert be.remove_temp_zones("vln1") == 1 and [z["name"] for z in be.zones] == ["room C"]


def test_temp_zones_removed_on_estop_and_by_ttl(tmp_path):
    be, fp = backend(tmp_path)
    be.add_temp_zone((2.0, -2.0, 6.0, 2.0), "vln1", "room B")
    be.estop(True)
    assert not any(z.get("temporary") for z in be.zones)
    be.estop(False)
    be.add_temp_zone((2.0, -2.0, 6.0, 2.0), "vln2", "room B", ttl_s=30.0)
    fp.advance(31.0)
    be.tick()
    assert not any(z.get("temporary") for z in be.zones)
    assert any(e["type"] == "zone_expired" for e in be.events)


def test_temp_zones_from_a_file_are_ignored_on_load(tmp_path):
    p = tmp_path / "places.yaml"
    d = yaml.safe_load((ROOT / "maps/lab4.places.yaml").read_text())
    d["no_go_zones"] = [{"id": 1, "name": "x", "x": 0, "y": 0, "r": 1, "temporary": True}]
    p.write_text(yaml.safe_dump(d))
    assert RosBackend(port(), p).zones == []


# ------------------------------------------------------------------ 2b. world model from the real map
def test_world_from_ros_rooms_doors_objects_pose():
    be, fp = backend()
    fp.objects[0]["sigma_xyz"] = [0.02, 0.03, 0.05]
    fp.x, fp.y, fp.yaw = 0.5, -0.5, 1.0
    w = VLNWorld.from_backend(be)
    assert set(w.rooms) == {"room A", "room B", "room C", "room D"} and w.robot == (0.5, -0.5, 1.0)
    gt = {"A-B": (2.0, -0.7), "B-D": (4.0, 2.0), "C-D": (2.0, 4.2), "A-C": (-0.7, 2.0)}       # lab4_gt.json doors (sorted names)
    assert set(w.doors) == set(gt) and all(math.dist(w.doors[k], gt[k]) < 0.06 for k in gt)
    assert len(w.objects) == 10 and w.objects[0]["sigma"] == 0.03 and w.objects[1]["sigma"] == 0.05


def test_doors_from_the_places_file_win_and_default_sigma():
    be, fp = backend(fp=port(objects=[{"id": 1, "class": "barrel", "color": "red", "x": 5.1, "y": 5.0}]))
    be.doors = {"A-B": (2.0, -0.7)}
    w = VLNWorld.from_backend(be)
    assert w.doors == {"A-B": (2.0, -0.7)} and w.objects[0]["sigma"] == 0.10


def test_doors_from_map_synthetic():
    rooms = {"room A": (0, 0, 2, 2), "room B": (2, 0, 4, 2), "room C": (0, 2, 2, 4)}
    wall_gap = lambda x, y: "occupied" if (abs(x - 2) < 0.06 and not 0.6 <= y <= 1.6) or abs(y - 2) < 0.06 and x < 2 else "free"   # noqa: E731
    d = doors_from_map(rooms, wall_gap)
    assert set(d) == {"A-B"} and d["A-B"][0] == 2 and abs(d["A-B"][1] - 1.1) < 0.06          # A-C wall has no gap
    narrow = lambda x, y: "occupied" if abs(x - 2) < 0.06 and not 0.9 <= y <= 1.2 else "free"   # noqa: E731
    assert "A-B" not in doors_from_map(rooms, narrow)                                          # 0.3 m: too narrow


# ------------------------------------------------------------------ 2c. walks on RosBackend
def test_executor_walk_with_avoid_on_ros_backend():
    seen = []

    class Spy(RosBackend):
        def add_temp_zone(self, rect, tag, name, ttl_s=1800.0):
            seen.append(tuple(rect))
            return super().add_temp_zone(rect, tag, name, ttl_s)
    fp = port()
    be = Spy(fp, ROOT / "maps/lab4.places.yaml")
    w = VLNWorld.from_backend(be)
    raw = json.dumps(GO(S("enter", room="room D"), avoid=["room B"]))
    plan = validate(raw, "go to room D but avoid room B", w)
    res = VLNExecutor(be, lambda: VLNWorld.from_backend(be)).run(plan, raw, "go to room D but avoid room B")
    assert res.ok and res.reply == "I'm in room D." and res.rooms_visited == ["room A", "room C", "room D"]
    assert seen == [(2.0, -2.0, 6.0, 2.0)] and not any(z.get("temporary") for z in be.zones)
    assert not any(2.0 < x < 6.0 and -2.0 < y < 2.0 for _, x, y in res.trajectory)


def agent_on_ros(vln=True):
    be, fp = backend()
    ag = Agent(ToolLayer(be), KeywordLLM(be.known_places), vln=VLNPlanner(D.OracleLLM()) if vln else None)
    return ag, be, fp


def test_agent_walk_with_reobservation_on_ros_backend():
    ag, be, fp = agent_on_ros()
    r = ag.say("go to the red barrel")
    assert r.reply == "I'm at the red barrel in room D." and ag.vln_last.ok
    assert ag.vln_last.rooms_visited[-1] == "room D"


def test_agent_landmark_not_seen_is_not_claimed_on_ros():
    ag, be, fp = agent_on_ros()

    def blind():                                                       # the detector never sees anything (stale map only)
        return json.loads(json.dumps(fp.objects))
    fp.semantic_objects = blind
    r = ag.say("go to the red barrel")
    assert ag.vln_last.code == "LANDMARK_NOT_CONFIRMED" and "I'm at" not in r.reply


def test_estop_mid_walk_on_ros_removes_zones():
    ag, be, fp = agent_on_ros()
    orig = fp.advance

    def adv(dt):
        orig(dt)
        if fp.t > 20.0 and not fp.estop:
            be.estop(True)                                             # the e-stop button / AgentHost fast path
    fp.advance = adv
    r = ag.say("go to room D but avoid room B")
    assert ag.vln_last.code == "ESTOP" and r.reply.startswith("I stopped (emergency stop)")
    assert not any(z.get("temporary") for z in be.zones) and be.task is None


def test_crash_mid_walk_removes_zones_and_stops():
    ag, be, fp = agent_on_ros()
    calls = {"n": 0}
    orig = VLNExecutor._sample

    def boom(self, world):
        calls["n"] += 1
        if calls["n"] > 20:
            raise RuntimeError("simulated crash")
        return orig(self, world)
    VLNExecutor._sample = boom
    try:
        r = ag.say("go to room D but avoid room B")
    finally:
        VLNExecutor._sample = orig
    assert "Something went wrong" in r.reply and not any(z.get("temporary") for z in be.zones) and be.task is None
    assert any("vln route failed" in e for e in ag.errors)


def test_flag_off_on_ros_backend():
    ag, be, fp = agent_on_ros(vln=False)
    ag.say("go to room B")
    assert ag.vln_last is None
