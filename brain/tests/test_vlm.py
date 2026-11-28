"""describe_view (brain/vlm): scene questions answered by a VLM but stated only when the robot's own facts agree.
Frames from TODAY's Gazebo runs (2026-11-28): the body-camera detector output per frame (sightings.jsonl of the fetch smoke runs)
and RGB frames rendered in Gazebo for the cone check (copied to brain/tests/data/vlm). The VLM is a scripted mock; no GPU."""
import json
import math
import random
from pathlib import Path

import pytest

from brain.vlm import client as C
from brain.vlm import ground as G
from brain.vlm.frames import Frame, frame_from_render, frames_from_sightings
from brain.vlm.questions import classify
from brain.vlm.tool import DescribeView, ros_frame_source

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "brain/tests/data/vlm"
CUBE_LOG = ROOT / "docs/results/data/smoke_1128b/cube/fetch/small_box_1/chat/sightings.jsonl"   # 586 frames, lab4_fetch, today


def render(name, scene):
    return frame_from_render(DATA / f"{name}.png", DATA / f"{name}.meta.json", DATA / f"scene_{scene}.json")


@pytest.fixture(scope="module")
def log_frames():
    return frames_from_sightings(CUBE_LOG)


def frame_at(frames, t):
    """A frame of today's sighting logs. Those runs saved no RGB, so for the MOCK VLM the frame gets a stand-in image (the mock
    ignores pixels; the real VLM is evaluated on rendered RGB frames by scripts/vlm_eval.sh)."""
    f = next(f for f in frames if abs(f.t - t) < 1e-6)
    return Frame(f.t, [dict(d) for d in f.detections], f.cam, b"stand-in: no RGB was saved by this run", f.source)


def vlm(*objects, path_clear=None, unsure=False, answer="..."):
    return C.ScriptedVLM({"answer": answer, "objects": [o if isinstance(o, dict) else {"name": o} for o in objects],
                          "path_clear": path_clear, "unsure": unsure})


class Backend:
    """Just enough backend for the tool: clock, pose, no map tracks, path_check from a table."""

    def __init__(self, t=100.0, pose=(0.0, 0.0, 0.0), paths=None, places=("room A", "room B", "room C", "room D"), names=None):
        self.t, self.pose, self.paths, self.places = t, pose, paths or {}, list(places)
        self.room_names = names or {"kitchen": "room D", "office": "room B"}

    def now(self):
        return self.t

    def _pose(self):
        return self.pose

    def known_places(self):
        return self.places

    def path_check(self, place=None, **kw):
        from brain.tools.types import ok
        return ok(**self.paths[place])


def tool_for(frame, model, backend=None):
    be = backend or Backend(t=frame.t)
    return DescribeView(be, model, frame_source=lambda: frame)


# ------------------------------------------------------------------ frames from today's runs
def test_sightings_become_frames_and_the_camera_looks_at_its_detections(log_frames):
    assert len(log_frames) == 586
    f = frame_at(log_frames, 79.993)
    assert sorted((d["cls"], d["color"]) for d in f.detections) == [("barrel", "blue"), ("box", "blue"), ("mini_extinguisher", "red")]
    # the cam quaternion -> viewing direction: every logged detection lies inside the camera's field of view (hfov 60 deg)
    off = []
    for fr in log_frames:
        cx, cy, yaw = fr.cam_xy_yaw()
        for d in fr.detections:
            b = math.atan2(d["y"] - cy, d["x"] - cx) - yaw
            off.append((abs(math.degrees(math.atan2(math.sin(b), math.cos(b)))), d["truncated"]))
    assert len(off) == 855 and sum(o <= 35 for o, _ in off) >= 830
    assert all(tr for o, tr in off if o > 35)      # the 18 outside: boxes cut by the image edge (a near object's centre is out of view)


def test_rendered_frames_carry_rgb_and_gt_colours():
    f = render("s0099_v01", "0099")
    assert f.image[:8] == b"\x89PNG\r\n\x1a\n"
    assert sorted((d["cls"], d["color"]) for d in f.detections) == [
        ("box", "red"), ("can", "green"), ("chair", "white"), ("fire_extinguisher", ""), ("small_box", "green"), ("traffic_cone", "")]


# ------------------------------------------------------------------ questions
@pytest.mark.parametrize("text,kind,extra", [
    ("what's on the table?", "on", {"surface": "table", "surface_cls": None}),
    ("What is next to the chair?", "on", {"relation": "next to", "surface_cls": "chair"}),
    ("is the path to the kitchen clear?", "path", {"place": "kitchen"}),
    ("Is the way to room B blocked", "path", {"place": "room B", "asked": "blocked"}),
    ("what colour is the box near the barrel?", "colour", {"cls": "box", "near": "barrel"}),
    ("what color is the cube", "colour", {"cls": "small_box", "near": None}),
    ("how many cones can you see?", "count", {"cls": "traffic_cone"}),
    ("what do you see?", "see", {}),
    ("go to room B", None, None),
    ("what changed in room B?", None, None),
])
def test_classify(text, kind, extra):
    q = classify(text)
    if kind is None:
        assert q is None
    else:
        assert q["kind"] == kind and all(q[k] == v for k, v in extra.items()), q


# ------------------------------------------------------------------ "what do you see?"
def test_see_agreeing_vlm_states_the_facts():
    f = render("s0099_v01", "0099")
    r = tool_for(f, vlm({"name": "traffic cone", "color": "white"}, "chair", {"name": "fire extinguisher", "color": "red"},
                        {"name": "box", "color": "red"}, {"name": "cube", "color": "green"}, {"name": "can", "color": "green"}))("what do you see?")
    assert r["status"] == "confirmed"
    assert r["reply"] == "I can see a fire extinguisher, a red box, a white chair, a traffic cone, a green can and a green cube."


def test_see_never_states_what_only_the_vlm_saw():
    f = render("s0099_v01", "0099")
    r = tool_for(f, vlm("chair", "sofa", {"name": "barrel", "color": "blue"}))("what do you see?")
    assert r["status"] == "not_sure"
    assert "The camera model also mentions a blue barrel, but my detector doesn't see that, so I'm not sure about it." in r["reply"]
    assert "It also mentions a sofa, which my detector can't confirm." in r["reply"]
    assert G.check_reply(r["reply"], r["facts"]) == []


def test_see_dark_frame_with_a_ball_that_is_not_a_detector_class():
    f = render("s0239_v05", "0239")                                  # real render: a big green ball (a distractor) next to chairs
    r = tool_for(f, vlm({"name": "green ball", "color": "green"}, "chair", {"name": "cone", "color": "blue"}))("what do you see?")
    assert "It also mentions a green ball, which my detector can't confirm." in r["reply"]
    assert "a black chair" in r["reply"] and "a white chair" in r["reply"] and "ball" not in r["reply"].split(".")[0]


def test_see_colour_conflict_is_not_sure():
    f = render("s0099_v01", "0099")
    r = tool_for(f, vlm({"name": "box", "color": "orange"}))("what do you see?")
    assert "I'm not sure about the colour of the box: my detector says red, the camera model says orange." in r["reply"]


# ------------------------------------------------------------------ "what colour is the box near the barrel?" (today's real frame)
def test_colour_near_reference_agree(log_frames):
    r = tool_for(frame_at(log_frames, 79.993), vlm({"name": "blue box"}, {"name": "barrel", "color": "blue"}))("what colour is the box near the barrel?")
    assert r["status"] == "confirmed" and r["reply"] == "The box near the barrel is blue."


def test_colour_near_reference_disagree(log_frames):
    r = tool_for(frame_at(log_frames, 79.993), vlm({"name": "box", "color": "green"}))("what colour is the box near the barrel?")
    assert r["status"] == "not_sure"
    assert r["reply"] == "I'm not sure: my detector says the box near the barrel is blue, but the camera model says green."


def test_colour_without_vlm_says_so(log_frames):
    r = tool_for(frame_at(log_frames, 79.993), C.ScriptedVLM(OSError("refused")))("what colour is the box near the barrel?")
    assert r["status"] == "detector_only" and r["reply"] == "My detector says the box near the barrel is blue (my camera model isn't available)."


def test_colour_missing_reference_and_ambiguity(log_frames):
    no_barrel = next(f for f in log_frames if {d["cls"] for d in f.detections} >= {"box"} and "barrel" not in {d["cls"] for d in f.detections})
    r = tool_for(no_barrel, vlm({"name": "box", "color": "blue"}, {"name": "barrel"}))("what colour is the box near the barrel?")
    assert r["reply"] == "I don't see a barrel here."                    # the VLM's barrel does not count
    f = frame_at(log_frames, 79.993)
    twin = Frame(f.t, f.detections + [{**next(d for d in f.detections if d["cls"] == "box"), "color": "red", "y": 3.55}], f.cam, None)
    r = tool_for(twin, vlm())("what colour is the box near the barrel?")
    assert r["status"] == "ask" and r["reply"].endswith("Which one?")


def test_colour_of_unknown_thing():
    f = render("s0099_v01", "0099")
    r = tool_for(f, vlm({"name": "ball", "color": "green"}))("what colour is the ball?")
    assert r["reply"] == "I don't know what a ball is, so I can't check its colour."


# ------------------------------------------------------------------ "what's on the table?"
def test_table_is_never_confirmed():
    f = render("s0099_v01", "0099")
    r = tool_for(f, vlm({"name": "table"}, {"name": "red box", "where": "on the table"}, {"name": "vase", "where": "on the table"}))("what's on the table?")
    assert r["reply"].startswith("I can't confirm a table: my detector doesn't know tables, though the camera model mentions one.")
    assert "The camera model says a red box is on it, which I can't check." in r["reply"]
    assert "It also mentions a vase, which my detector can't confirm." in r["reply"]
    assert G.check_reply(r["reply"], r["facts"]) == []


# ------------------------------------------------------------------ counts
def test_count_disagreement():
    f = render("s0099_v01", "0099")
    assert tool_for(f, vlm("cone"))("how many cones can you see?")["reply"] == "I can see one traffic cone."
    r = tool_for(f, vlm("cone", "cone"))("how many cones can you see?")
    assert r["status"] == "not_sure" and r["reply"] == "I'm not sure how many: my detector sees one traffic cone, the camera model counts two."


# ------------------------------------------------------------------ "is the path to the kitchen clear?"
def _path_tool(reachable, model, pose=(4.0, 0.5, math.pi / 2), blocked_by=None):
    be = Backend(t=10.0, pose=pose, paths={"room D": {"reachable": reachable, "blocked_by": blocked_by, "target": {"x": 4.0, "y": 4.0}}})
    fr = Frame(10.0, [], pose, (DATA / "s0099_v01.png").read_bytes())
    return DescribeView(be, model, frame_source=lambda: fr)


def test_path_agree_disagree_and_not_in_view():
    q = "is the path to the kitchen clear?"
    assert _path_tool(True, vlm(path_clear=True))(q)["reply"] == \
        "Yes, the way to room D (the kitchen) looks clear: my path check finds a way to room D (the kitchen), and the camera agrees."
    r = _path_tool(True, vlm(path_clear=False))(q)
    assert r["status"] == "not_sure" and "but the camera model thinks it is blocked" in r["reply"]
    r = _path_tool(False, vlm(path_clear=True), blocked_by="no-go zone")(q)
    assert r["status"] == "not_sure" and "finds no way through (no-go zone)" in r["reply"]
    assert _path_tool(False, vlm(path_clear=None), blocked_by="no-go zone")(q)["reply"].startswith("No, the way to room D (the kitchen) is blocked")
    r = _path_tool(True, vlm(path_clear=False), pose=(4.0, 0.5, -math.pi / 2))(q)               # facing away from the kitchen
    assert r["status"] == "detector_only" and r["reply"].endswith("It isn't in front of me, so I checked it on the map only.")
    assert _path_tool(True, vlm())("is the path to the garage clear?")["reply"] == "I don't know a place called the garage."


# ------------------------------------------------------------------ no view / VLM failures
def test_stale_frame_bad_json_and_unsure(log_frames):
    f = frame_at(log_frames, 79.993)
    late = DescribeView(Backend(t=f.t + 10), vlm("box"), frame_source=lambda: f)("what do you see?")        # 10 s old
    assert late["status"] == "no_view" and late["reply"] == "I don't have a current camera view, so I can't answer that right now."
    r = tool_for(f, C.ScriptedVLM("I see a box"))("what do you see?")
    assert r["status"] == "detector_only" and "(My camera model gave no usable answer, so this is from my detector only.)" in r["reply"]
    r = tool_for(f, vlm("sofa", unsure=True))("what do you see?")
    assert "sofa" not in r["reply"] and "The camera model wasn't sure about this view." in r["reply"]


# ------------------------------------------------------------------ the invariant, fuzzed on real frames
NOUNS = ["box", "crate", "barrel", "drum", "cone", "chair", "mannequin", "toolbox", "extinguisher", "can", "cube", "table", "sofa",
         "ball", "person", "dog", "lamp", "shelf", "plant", "bottle", "floor", "wall", "door"]
QUESTIONS = ["what do you see?", "what's on the table?", "what is next to the chair?", "what colour is the box near the barrel?",
             "what colour is the barrel?", "how many cones can you see?", "how many boxes can you see?"]


def test_replies_only_state_facts_for_random_vlm_outputs(log_frames):
    rng = random.Random(7)
    frames = [frame_at(log_frames, 79.993), frame_at(log_frames, log_frames[0].t), frame_at(log_frames, log_frames[300].t),
              render("s0099_v01", "0099"), render("s0239_v05", "0239")]
    n = 0
    for _ in range(500):
        f = rng.choice(frames)
        objs = [{"name": rng.choice(NOUNS), "color": rng.choice([None, "red", "blue", "green", "white"]),
                 "where": rng.choice([None, "on the table", "left"])} for _ in range(rng.randint(0, 6))]
        r = tool_for(f, vlm(*objs, unsure=rng.random() < 0.1))(rng.choice(QUESTIONS))
        assert G.check_reply(r["reply"], r["facts"]) == [], (r["reply"], r["facts"])
        n += 1
    assert n == 500


def test_runtime_guard_falls_back_to_facts(monkeypatch, log_frames):
    real = G.answer

    def leaky(q, facts, vlm_, path=None, why=""):
        v = real(q, facts, vlm_, path, why)
        if vlm_ is not None:
            v.reply += " There is a sofa and a red traffic cone."            # a template bug that would state VLM-only objects
        return v
    monkeypatch.setattr(G, "answer", leaky)
    r = tool_for(frame_at(log_frames, 79.993), vlm("cone"))("what do you see?")
    assert "guard" in r and "traffic cone" not in r["reply"] and "gave an answer I couldn't check" in r["reply"]


# ------------------------------------------------------------------ the agent route (flag)
def test_agent_routes_scene_questions_only_with_the_flag(log_frames):
    from brain.agent.graph import Agent
    from brain.agent.llm import KeywordLLM
    from brain.mock import MockBackend, gazebo_lab4
    from brain.tools import ToolLayer
    be = MockBackend(gazebo_lab4())
    f = frame_at(log_frames, 79.993)
    f = Frame(be.now(), f.detections, f.cam, None)
    ag = Agent(ToolLayer(be), KeywordLLM(be.known_places), vlm=DescribeView(be, vlm({"name": "box", "color": "blue"}), frame_source=lambda: f))
    r = ag.say("what colour is the box near the barrel?")
    assert ag.vlm_last["kind"] == "colour" and r.reply.startswith("My detector says the box near the barrel is blue")   # no image: facts only
    ag.say("go to room B")
    assert ag.vlm_last["kind"] == "colour"                                # navigation is not a scene question
    off = Agent(ToolLayer(MockBackend(gazebo_lab4())), KeywordLLM(be.known_places))
    off.say("what colour is the box near the barrel?")
    assert off.vlm is None and off.vlm_last is None


# ------------------------------------------------------------------ transport + ROS adapter
def test_ollama_request_and_parse():
    png = (DATA / "s0099_v01.png").read_bytes()
    import base64
    b = C.OllamaVLM("moondream").body(png, "what do you see?")
    assert b["format"] == C.schema() and b["messages"][1]["images"] == [base64.b64encode(png).decode()]
    assert b["options"]["temperature"] == 0 and "Question (data)" in b["messages"][1]["content"]
    assert C.OllamaVLM("m", use_schema=False).body(png, "q")["format"] == "json"
    assert C.parse('```json\n{"answer": "a", "objects": ["box", {"name": "cone", "color": "red"}], "unsure": false}\n```')["objects"][1] == \
        {"name": "cone", "color": "red", "where": None}
    assert C.parse("not json") is None and C.parse({"answer": "x", "objects": [{"name": 3}]}) is None


def test_ros_frame_source_takes_boxes_and_positions_from_the_map():
    class Extras:
        def camera(self):
            return {"jpeg": b"\xff\xd8jpeg", "boxes": [{"name": "barrel", "score": 0.9, "box": [1, 2, 3, 4]}]}

    class Be(Backend):
        def _tracks(self):
            return [{"id": 3, "class": "barrel", "color": "blue", "x": 2.0, "y": 0.2}, {"id": 4, "class": "box", "color": "red", "x": -3.0, "y": 0.0}]
    be = Be(t=5.0, pose=(0.0, 0.0, 0.0))
    t = DescribeView(be, vlm({"name": "barrel", "color": "blue"}), frame_source=ros_frame_source(Extras(), be))
    r = t("what colour is the barrel?")
    assert [(f["cls"], f["color"], f["x"]) for f in r["facts"]] == [("barrel", "blue", 2.0)]          # the box behind the robot is out of view
    assert r["reply"] == "The barrel is blue."


# ------------------------------------------------------------------ the GPU eval, checked with mocks (no model, no GPU)
def _mini_dataset(tmp_path):
    import shutil
    for sub in ("meta/test", "images/test", "worlds"):
        (tmp_path / sub).mkdir(parents=True)
    for name, scene in (("s0099_v01", "0099"), ("s0239_v05", "0239"), ("s0077_v03", "0077")):
        shutil.copy(DATA / f"{name}.png", tmp_path / "images/test" / f"{name}.png")
        shutil.copy(DATA / f"{name}.meta.json", tmp_path / "meta/test" / f"{name}.json")
        shutil.copy(DATA / f"scene_{scene}.json", tmp_path / "worlds" / f"scene_{scene}.json")
    return tmp_path


def test_eval_metrics_with_a_perfect_and_a_hallucinating_mock(tmp_path):
    from brain.vlm import eval as E
    frames = E.dataset_frames([_mini_dataset(tmp_path)], n=10)
    assert len(frames) == 3
    good = E.run_eval(E.MockVLM("perfect", frames), frames)
    v = good["vlm_alone"]
    assert (v["json_valid_rate"], v["precision"], v["hallucination_rate"], v["recall"], v["colour_accuracy"]) == (1, 1, 0, 1, 1)
    assert good["tool_gt_facts"]["false_statements"] == 0 and good["tool_gt_facts"]["invariant_violations"] == 0
    assert good["tool_corrupted_facts"]["invariant_violations"] == 0
    bad = E.run_eval(E.MockVLM("hallucinate", frames), frames)
    b = bad["vlm_alone"]
    assert b["hallucination_rate"] > 0 and b["colour_accuracy"] == 0 and b["outside_nouns_per_frame"] == 1
    assert bad["tool_gt_facts"]["false_statements"] == 0 and bad["tool_gt_facts"]["invariant_violations"] == 0   # the tool held


def _run_script(tmp_path, ps_out="", extra=None):
    import os
    import stat
    import subprocess
    import sys
    bindir = tmp_path / "bin"
    bindir.mkdir()
    d = bindir / "docker"
    d.write_text(f'#!/bin/bash\ncase "$1" in ps) printf "{ps_out}" ;; esac\n')
    d.chmod(d.stat().st_mode | stat.S_IEXEC)
    env = {k: v for k, v in os.environ.items() if not k.startswith("GPU_LOCK")}
    env.update({"PATH": f"{bindir}:{env['PATH']}", "GPU_LOCK": str(tmp_path / "lock"), "MOCK": "perfect", "N": "3",
                "DATASETS": str(_mini_dataset(tmp_path / "ds")), "OUT": str(tmp_path / "out"), "PY": sys.executable, **(extra or {})})
    return subprocess.run(["bash", str(ROOT / "scripts/vlm_eval.sh")], cwd=ROOT, env=env, capture_output=True, text=True, timeout=120)


def test_eval_script_mock_run_uses_its_own_lock_file(tmp_path):
    r = _run_script(tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    s = json.loads((tmp_path / "out/summary.json").read_text())
    assert s["n"] == 3 and s["models"]["mock-perfect"]["vlm_alone"]["precision"] == 1
    assert "| mock-perfect |" in (tmp_path / "out/report.md").read_text()


def test_eval_script_refuses_while_containers_run(tmp_path):
    r = _run_script(tmp_path, ps_out="abc\\n")
    assert r.returncode == 1 and "docker ps" in r.stdout + r.stderr


# ------------------------------------------------------------------ frames saved by a Gazebo run (detector_frames:=DIR), replayed
def test_saved_frames_replay_through_the_tool(tmp_path):
    import io
    import numpy as np
    from PIL import Image
    from gurdy_core.vision.frame_saver import FrameSaver
    from brain.vlm.frames import frames_from_saved
    src = frames_from_sightings(CUBE_LOG)
    f = next(x for x in src if abs(x.t - 79.993) < 1e-6)
    rgb = np.asarray(Image.open(io.BytesIO((DATA / "s0099_v01.png").read_bytes())).convert("RGB"))   # pixels: a stand-in render
    dets = [{"cls": d["cls"], "score": d["score"], "box": d["box"], "color": d["color"], "p": [d["x"], d["y"], d["z"]],
             "truncated": d["truncated"]} for d in f.detections]
    FrameSaver(tmp_path, free_gb=lambda: 100.0).maybe_save(f.t, rgb, f.cam, dets)      # tmp_path: small tmpfs
    back = frames_from_saved(tmp_path)
    assert len(back) == 1 and back[0].image[:2] == b"\xff\xd8" and back[0].cam_xy_yaw() == pytest.approx(f.cam_xy_yaw())
    assert [(d["cls"], d["color"], d["x"]) for d in back[0].detections] == [(d["cls"], d["color"], d["x"]) for d in f.detections]
    r = tool_for(back[0], vlm({"name": "box", "color": "blue"}))("what colour is the box near the barrel?")
    assert r["reply"] == "The box near the barrel is blue."
