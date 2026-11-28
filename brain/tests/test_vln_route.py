"""VLN route (2026-11-28): "go to the kitchen, past the chair, stop near the red barrel" end to end through the agent on the
lab4 MOCK (and the ROS backend on a FakePort): named rooms, the keyword fallback (no LLM / bad LLM output), the schema-constrained
Ollama request, the uncertainty-aware clearance to the landmark itself, the pre-move reachability check, and the honest
replies (unknown landmark, ambiguous 'the chair', unreachable goal, a walk blocked half-way). No ROS, no LLM, no Gazebo."""
import json
import math
from pathlib import Path

import pytest
import yaml

from brain.agent.graph import Agent
from brain.agent.llm import KeywordLLM
from brain.eval import vln_dev as D
from brain.mock import MockBackend, gazebo_lab4
from brain.mock.world import GtObject, Rect
from brain.tools import ToolLayer
from brain.vln import keyword as K
from brain.vln.plan import landmark_clearance, validate
from brain.vln.planner import VLNPlanner, _ollama_vln, vln_json_schema
from brain.vln.world import VLNWorld

ROOT = Path(__file__).resolve().parents[2]
R, S, GO = D.R, D.S, D.GO
CMD = "go to the kitchen, past the chair, stop near the red barrel"
CHAIR_D = (4.3, 3.9)               # in room D (the kitchen), off the B-D door -> red barrel line
CHAIR_B = (3.0, -1.0)              # a second chair, in room B
BARREL_RED = (5.1, 5.0)


def down(system, user):
    raise OSError("connection refused")                       # Ollama not running


class Scripted:
    def __init__(self, table):
        self.table = {t: json.dumps(p) for t, p in table.items()}
        self.n = 0

    def __call__(self, system, user):
        self.n += 1
        return self.table.get(json.loads(user.split(": ", 1)[1]), '{"kind": "none"}')


def lab(chairs=(CHAIR_D,), llm=down, backend_cls=MockBackend, names=None):
    w = gazebo_lab4()
    for i, (x, y) in enumerate(chairs):
        w.objects.append(GtObject(100 + i, "chair", "", x, y, 0.25))
    w._build()
    w.room_names = {"kitchen": "room D"} if names is None else names
    be = backend_cls(w)
    be.x, be.y, be.yaw = 0.0, 0.0, 0.0
    for o in be.world.objects:
        be._sight(o)
    ag = Agent(ToolLayer(be), KeywordLLM(be.known_places), vln=VLNPlanner(llm))
    return ag, be


def gt(be, cls, color=""):
    return next(o for o in be.world.objects if o.cls == cls and o.color == color)


def not_moved(be):
    return math.hypot(be.x, be.y) < 1e-6 and be.task is None


# ------------------------------------------------------------------ the example, keyword fallback (no LLM)
def test_kitchen_chair_red_barrel_keyword_fallback_end_to_end():
    ag, be = lab()
    r = ag.say(CMD)
    v, p = ag.vln_last, ag.vln.last_plan
    assert p.source == "keyword" and ag.vln.failures == 1 and ag.vln.keyword_used == 1
    assert r.reply == ("I'm at the red barrel in room D (the kitchen). "
                       "On the way I went past the chair in room D (the kitchen).")
    assert v.ok and v.rooms_visited[0] == "room A" and v.rooms_visited[-1] == "room D" and len(v.rooms_visited) == 3
    assert v.rooms_visited == p.rooms and be.zones == []                  # the shortest way round the ring, as planned
    # by ground truth, not by the robot's own map
    chair, barrel = gt(be, "chair"), gt(be, "barrel", "red")
    assert min(math.hypot(x - chair.x, y - chair.y) for _, x, y in v.trajectory) <= 1.2
    assert math.hypot(be.x - barrel.x, be.y - barrel.y) <= 1.2 and be.world.rooms["room D"].contains(be.x, be.y)
    assert [w["kind"] for w in p.waypoints] == ["door", "door", "enter", "pass", "stop"]


def test_same_instruction_without_commas_keeps_the_chair():
    """'go to the kitchen past the chair and stop ...': the chair must not be dropped from the route."""
    w = VLNWorld.lab4(names={"kitchen": "room D"}, objects=None)
    w.objects.append({"id": 99, "class": "chair", "color": "", "x": CHAIR_D[0], "y": CHAIR_D[1]})
    d = K.parse("go to the kitchen past the chair and stop near the red barrel", w)
    assert [s["action"] for s in d["steps"]] == ["enter", "pass", "stop"]
    assert d["steps"][1]["target"] == {"object": {"class": "chair"}}


def test_llm_plan_with_room_name_is_used_when_valid():
    plan = GO(S("enter", room="kitchen"), S("pass", object=R("chair")), S("stop", object=R("barrel", "red")))
    llm = Scripted({CMD: plan})
    ag, be = lab(llm=llm)
    r = ag.say(CMD)
    assert ag.vln.last_plan.source == "llm" and llm.n == 1 and ag.vln.keyword_used == 0
    assert ag.vln_last.ok and r.reply.startswith("I'm at the red barrel in room D (the kitchen).")


def test_llm_inventing_a_landmark_falls_back_to_the_keyword_plan():
    bad = GO(S("pass", object=R("box", "blue")), S("pass", object=R("chair")), S("stop", object=R("barrel", "red")))
    ag, be = lab(llm=Scripted({CMD: bad}))
    ag.say(CMD)
    p = ag.vln.last_plan
    assert ag.vln.rejected == 1 and p.source == "keyword" and ag.vln_last.ok
    assert not any("blue box" in w["label"] for w in p.waypoints)


def test_llm_dropping_the_avoid_falls_back_and_room_b_is_never_entered():
    text = CMD + ", avoid room B"
    dropped = GO(S("enter", room="kitchen"), S("pass", object=R("chair")), S("stop", object=R("barrel", "red")))
    ag, be = lab(llm=Scripted({text: dropped}))
    r = ag.say(text)
    v = ag.vln_last
    assert ag.vln.last_plan.source == "keyword" and ag.vln.last_plan.avoid == ["room B"]
    assert v.ok and "room B" not in v.rooms_visited and "red barrel" in r.reply
    assert not any(2.0 < x < 6.0 and -2.0 < y < 2.0 for _, x, y in v.trajectory)


def test_avoid_by_room_name():
    w = VLNWorld.lab4(names={"kitchen": "room D"})
    d = K.parse("go to room C but stay out of the kitchen", w)
    p = validate(d, "go to room C but stay out of the kitchen", w)
    assert d["avoid"] == ["room D"] and p.ok and p.avoid == ["room D"]


# ------------------------------------------------------------------ honest answers, nothing moves
def test_unknown_landmark_chair_never_seen():
    ag, be = lab(chairs=())
    r = ag.say(CMD)
    assert r.reply == "I haven't seen the chair yet, so I can't use it as a landmark. Shall I explore first?"
    assert ag.vln.last_plan.error == "LANDMARK_NOT_FOUND" and not_moved(be) and be.zones == []


def test_unknown_word_as_landmark_is_not_dropped_and_not_walked():
    llm = Scripted({})
    ag, be = lab(llm=llm)
    r = ag.say("go to the kitchen, past the sofa, stop near the red barrel")
    assert ag.vln.last_plan.error == "UNKNOWN_LANDMARK" and ag.vln_last is None and llm.n == 0
    assert r.reply.startswith("I don't know a place or landmark called the sofa, so I can't plan that walk.") and not_moved(be)


def test_saved_place_is_not_an_unknown_landmark():
    w = VLNWorld.lab4(names={"kitchen": "room D"})
    assert K.unknown_landmark("go past the red box to the charging dock", w, ("home", "charging dock")) is None
    assert K.unknown_landmark("go past the sofa to room B", w, ("home",)) == "sofa"


def test_llm_that_drops_a_landmark_is_rejected():
    w = _world(0.0)
    dropped = json.dumps(GO(S("enter", room="kitchen"), S("stop", object=R("barrel", "red"))))
    assert validate(dropped, CMD, w).error == "DROPPED"
    no_room = json.dumps(GO(S("stop", object=R("barrel", "red"))))
    assert validate(no_room, "go through room C to the red barrel", w).error == "DROPPED"


def test_ambiguous_chair_asks_which_one_and_does_not_move():
    ag, be = lab(chairs=(CHAIR_D, CHAIR_B))
    r = ag.say(CMD)
    assert ag.vln.last_plan.error == "AMBIGUOUS" and not_moved(be)
    assert r.reply == "I know more than one chair: the chair in room B or the chair in room D (the kitchen). Which one?"


def test_ambiguous_chair_resolved_by_room():
    ag, be = lab(chairs=(CHAIR_D, CHAIR_B))
    r = ag.say("go to the kitchen, past the chair in the kitchen, stop near the red barrel")
    assert ag.vln_last.ok and "past the chair in room D (the kitchen)" in r.reply


def test_unknown_room_name():
    ag, be = lab()
    r = ag.say("go to room E, then stop near the red barrel")
    assert "There is no room E" in r.reply and not_moved(be)               # the agent's room guard answers first
    w = _world(0.0)
    p = validate(K.parse("go to room E", w), "go to room E", w)
    assert p.error == "UNKNOWN_ROOM" and "room D (the kitchen)" in p.reply


def test_unreachable_goal_is_refused_before_moving():
    """The user's own no-go zones close both doorways of the kitchen: the validator's door graph still has a route, the
    backend's path check does not -> refused before the first step, user zones kept, the walk's zones removed."""
    ag, be = lab()
    be.add_temp_zone((3.3, 1.5, 4.7, 2.5), "user", "no-go B-D")            # rectangles like the user's zones
    be.add_temp_zone((1.5, 3.5, 2.5, 4.9), "user", "no-go D-C")
    r = ag.say(CMD)
    assert ag.vln_last.code == "UNREACHABLE" and not_moved(be)
    assert r.reply.startswith("I can't reach ") and "no-go zone" in r.reply and r.reply.endswith("I haven't moved.")
    assert "I'm at" not in r.reply and [z["tag"] for z in be.zones] == ["user", "user"]


def test_room_without_a_door_is_impossible_without_mentioning_avoid():
    w = VLNWorld.lab4(names={"kitchen": "room D"})
    w.doors = {k: v for k, v in w.doors.items() if "D" not in k}
    p = validate(K.parse("go to the kitchen", w), "go to the kitchen", w)
    assert p.error == "IMPOSSIBLE" and p.reply == "I don't know a way from room A (the hallway) into room D (the kitchen) on this map."


def test_walk_blocked_half_way_is_an_honest_failure():
    class Blocks(MockBackend):                                  # the 2nd doorway closes once the robot is on its way to it
        legs = 0

        def go_to(self, **kw):
            Blocks.legs += 1
            if Blocks.legs == 2:
                x, y = kw["point"]["x"], kw["point"]["y"]
                self.add_obstacle(Rect(x - 0.6, y - 0.6, x + 0.6, y + 0.6))
            return super().go_to(**kw)
    ag, be = lab(backend_cls=Blocks)
    r = ag.say(CMD)
    v = ag.vln_last
    assert not v.ok and v.code in ("LEG_FAILED", "LEG_REFUSED") and v.claims == []
    second = ag.vln.last_plan.waypoints[1]["label"]
    assert "I'm at" not in r.reply and second in r.reply and be.zones == []
    assert not be.world.rooms["room D"].contains(be.x, be.y)


# ------------------------------------------------------------------ uncertainty-aware clearance (margin + 3 sigma)
def _world(sigma, margin=0.0):
    w = VLNWorld.lab4(names={"kitchen": "room D"})
    w.objects.append({"id": 99, "class": "chair", "color": "", "x": CHAIR_D[0], "y": CHAIR_D[1]})
    for o in w.objects:
        o["sigma"] = sigma
    w.margin = margin
    return w


@pytest.mark.parametrize("sigma", [0.0, 0.04, 0.10, 0.15])
def test_points_keep_margin_plus_3_sigma_from_their_landmark_and_the_walls(sigma):
    w = _world(sigma)
    p = validate(K.parse(CMD, w), CMD, w)
    if sigma >= 0.15:                     # every landmark that unsure: the chair's ring has no spot left -> honest refusal (safe side)
        assert p.error == "NO_FREE_SPOT" and "beside the chair" in p.reply
        return
    assert p.ok
    for wp in p.waypoints:
        if "landmark" not in wp:
            continue
        L = next(o for o in w.objects if o["id"] == wp["landmark"]["id"])
        assert landmark_clearance(L, w) == pytest.approx(0.4 + 3 * sigma)
        assert math.hypot(wp["x"] - L["x"], wp["y"] - L["y"]) >= landmark_clearance(L, w) - 1e-9
        assert w.room_of(wp["x"], wp["y"], 0.35 + 3 * sigma - 1e-9) is not None            # wall clearance grows with sigma
        for o in w.objects:
            if o["id"] != L["id"]:
                assert math.hypot(wp["x"] - o["x"], wp["y"] - o["y"]) >= 0.4 + 3 * math.hypot(sigma, sigma) - 1e-9


def test_clearance_numbers():
    assert landmark_clearance({"sigma": 0.10}, _world(0.1)) == pytest.approx(0.7)
    assert landmark_clearance({"sigma": 0.10}, _world(0.1, margin=0.05)) == pytest.approx(0.75)


def test_too_uncertain_landmark_has_no_free_spot():
    """sigma 0.3 m: the stop point would have to be >= 1.3 m from the barrel; the stand-offs go to 1.0 m -> honest no."""
    w = _world(0.3)
    text = "stop near the red barrel"
    p = validate(K.parse(text, w), text, w)
    assert p.error == "NO_FREE_SPOT" and p.reply == "There is no free spot next to the red barrel in room D (the kitchen) where I fit."


# ------------------------------------------------------------------ keyword parser rules
def test_keyword_refuses_order_reversals_and_open_ends():
    w = _world(0.0)
    assert K.parse("go to the kitchen after passing the chair", w) is None
    assert K.parse("before room D go past the chair", w) is None
    assert K.parse("go past the chair", w) is None                       # does not end anywhere


def test_keyword_stop_by_is_a_stop_and_first_then_keeps_order():
    w = _world(0.0)
    d = K.parse("go into the kitchen and stop by the toolbox", w)
    assert [s["action"] for s in d["steps"]] == ["enter", "stop"]
    d = K.parse("first walk into room C and then go to the toolbox", w)
    assert [s["action"] for s in d["steps"]] == ["enter", "stop"] and d["steps"][0]["target"] == {"room": "room C"}


def test_keyword_only_planner_needs_no_model():
    pl = VLNPlanner(False)
    p = pl.plan(CMD, _world(0.0))
    assert p.ok and p.source == "keyword" and pl.calls == 0


# ------------------------------------------------------------------ constrained JSON for the model
def test_ollama_request_carries_the_vln_schema():
    b = _ollama_vln("gurdy-sft-400").body("sys", "usr")
    assert b["format"] == vln_json_schema() and b["model"] == "gurdy-sft-400" and b["options"]["temperature"] == 0
    assert _ollama_vln("m", schema=False).body("s", "u")["format"] == "json"


def _conforms(v, sch):
    """Tiny JSON-schema check (the subset vln_json_schema uses; no jsonschema in the host venv)."""
    if "anyOf" in sch:
        return any(_conforms(v, s) for s in sch["anyOf"])
    t = sch.get("type")
    if t == "object":
        if not isinstance(v, dict) or any(k not in v for k in sch.get("required", [])):
            return False
        if sch.get("additionalProperties") is False and set(v) - set(sch["properties"]):
            return False
        return all(_conforms(v[k], sch["properties"][k]) for k in v)
    if t == "array":
        return isinstance(v, list) and sch.get("minItems", 0) <= len(v) <= sch.get("maxItems", 10 ** 9) and all(_conforms(x, sch["items"]) for x in v)
    if t == "string":
        return isinstance(v, str) and ("enum" not in sch or v in sch["enum"])
    return True


def test_every_dev_reference_plan_fits_the_schema_and_bad_ones_do_not():
    sch = vln_json_schema()
    for cid, text, ref, exp in D.DEV:
        if ref is not None:                                                 # not-nav cases have no reference plan
            assert _conforms(ref, sch), cid
    assert not _conforms({"kind": "go", "steps": [{"action": "fly", "target": {"room": "room B"}}]}, sch)
    assert not _conforms({"kind": "go", "steps": [{"action": "stop", "target": {"relation": "between", "of": [R("box")]}}]}, sch)
    assert not _conforms({"kind": "go", "steps": [{"action": "stop", "target": {"object": {"class": "sofa"}}}]}, sch)


# ------------------------------------------------------------------ ROS backend (FakePort): names from the places file, unreachable
def _ros(tmp_path):
    from brain.ros.backend import RosBackend
    from brain.tests.test_vln_ros import port
    d = yaml.safe_load((ROOT / "maps/lab4.places.yaml").read_text())
    d["places"]["room D"]["names"] = ["kitchen"]
    (tmp_path / "places.yaml").write_text(yaml.safe_dump(d, sort_keys=False))
    fp = port()
    fp.objects.append({"id": 50, "class": "chair", "color": "", "x": CHAIR_D[0], "y": CHAIR_D[1], "last_seen": -1.0, "hits": 20, "sigma_m": 0.05})
    be = RosBackend(fp, tmp_path / "places.yaml")
    return be, fp


def test_room_names_from_the_places_file_survive_load_and_save(tmp_path):
    be, fp = _ros(tmp_path)
    w = VLNWorld.from_backend(be)
    assert w.names == {"hallway": "room A", "office": "room B", "living room": "room C", "kitchen": "room D"} and w.canon_room("the kitchen") == "room D"
    be._save_places()
    again = yaml.safe_load((tmp_path / "places.yaml").read_text())
    assert again["places"]["room D"]["names"] == ["kitchen"]


def test_ros_walk_kitchen_chair_red_barrel(tmp_path):
    be, fp = _ros(tmp_path)
    ag = Agent(ToolLayer(be), KeywordLLM(be.known_places), vln=VLNPlanner(down))
    r = ag.say(CMD)
    assert ag.vln_last.ok and ag.vln.last_plan.source == "keyword", r.reply
    assert r.reply.startswith("I'm at the red barrel in room D (the kitchen).") and ag.vln_last.rooms_visited[-1] == "room D"
    lm = {w["landmark"]["id"]: w for w in ag.vln.last_plan.waypoints if "landmark" in w}
    chair = lm[50]
    assert math.hypot(chair["x"] - CHAIR_D[0], chair["y"] - CHAIR_D[1]) >= 0.4 + 3 * 0.05 - 1e-9


def test_ros_unreachable_goal_refused_before_moving(tmp_path):
    be, fp = _ros(tmp_path)
    w = VLNWorld.from_backend(be)
    stop = validate(K.parse(CMD, w), CMD, w).waypoints[-1]
    fp.unreachable = [(stop["x"], stop["y"], 0.05)]
    ag = Agent(ToolLayer(be), KeywordLLM(be.known_places), vln=VLNPlanner(down))
    r = ag.say(CMD)
    assert ag.vln_last.code == "UNREACHABLE" and r.reply.endswith("I haven't moved.") and "I'm at" not in r.reply
    assert math.hypot(fp.x, fp.y) < 1e-6 and not any(z.get("temporary") for z in be.zones)


# ------------------------------------------------------------------ the demo names in maps/lab4.places.yaml (2026-11-28)
@pytest.mark.parametrize("text,rooms", [
    ("go to the kitchen", ["room A", "room B", "room D"]),
    ("go to the office", ["room A", "room B"]),
    ("go to the living room", ["room A", "room C"]),                   # a two-word name with "room" in it
    ("go to the living room, then the kitchen", ["room A", "room C", "room D"]),
    ("go to the kitchen but stay out of the office", ["room A", "room C", "room D"]),
    ("go through the living room to the red barrel", ["room A", "room C", "room D"]),
    ("go to room D but avoid room B", ["room A", "room C", "room D"]),   # letters (the blind suite) work as before
])
def test_lab4_demo_names(text, rooms):
    w = VLNWorld.lab4()
    assert w.names == {"hallway": "room A", "office": "room B", "living room": "room C", "kitchen": "room D"}
    p = VLNPlanner(False).plan(text, w)
    assert p is not None and p.ok and p.rooms == rooms, (text, p and (p.error, p.reply))


def test_names_never_count_as_extra_room_letters():
    w = VLNWorld.lab4()
    from brain.vln.plan import rooms_said
    assert rooms_said("go to the living room and then room D", w) == ["room C", "room D"]
    assert rooms_said("the room with the red barrel", w) == []
    assert K.unknown_landmark("go past the red box into the living room", w, w.places) is None


def test_mock_lab4_has_the_names_and_says_them():
    from brain.mock import gazebo_lab4
    assert gazebo_lab4().room_names["living room"] == "room C"
    ag, be = lab(names=gazebo_lab4().room_names)
    r = ag.say("go to the office")
    assert r.reply == "I'm in room B (the office)." and ag.vln_last.ok
