"""Gazebo VLN runner (2026-11-28), everything that can be checked without Gazebo:
  * the lab4_vln world variant (lab4 + one chair; lab4 / lab4_fetch untouched)
  * the episode file + GT-graded scoring (brain/vln/gazebo_eval.py): success, path length / SPL, wrong room, avoid violations,
    false claims, refusal accuracy, unexpected refusals, infrastructure failures kept out of the rates
  * the in-container episode runner (brain/ros/vln_eval_runner.EpisodeRunner) on RosBackend + FakePort: GT trajectory, landmark
    seeding from GT, sim-time limit, hang
  * the host script scripts/vln_eval.sh with a STUB docker and a TEMPORARY lock file (never the real ~/projects/.gpu_lock)."""
import json
import os
import stat
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from brain.vln import gazebo_eval as G

ROOT = Path(__file__).resolve().parents[2]
DEV = ROOT / "brain/eval/vln_episodes_dev.yaml"
PY = sys.executable


# ------------------------------------------------------------------ the world variant
def test_lab4_vln_world_is_generated_and_lab4_worlds_are_untouched():
    for script in ("gen_world_lab4_vln.py", "gen_world_lab4.py", "gen_world_lab4_fetch.py"):
        r = subprocess.run([PY, str(ROOT / "scripts" / script), "--check"], capture_output=True, text=True)
        assert r.returncode == 0, (script, r.stdout, r.stderr)
    gt = json.loads((ROOT / "docs/results/data/lab4_vln/lab4_vln_gt.json").read_text())
    base = json.loads((ROOT / "docs/results/data/lab4/lab4_gt.json").read_text())
    chairs = [o for o in gt["objects"] if o["class"] == "chair"]
    assert gt["world"] == "lab4_vln" and len(chairs) == 1 and chairs[0]["room"] == "room D"
    assert [o for o in gt["objects"] if o["class"] != "chair"] == base["objects"] and gt["doors"] == base["doors"]
    assert not any(o["class"] == "chair" for o in base["objects"])
    sdf = (ROOT / "ros2_ws/src/gurdy_gazebo/worlds/lab4_vln.sdf").read_text()
    assert '<world name="lab4_vln">' in sdf and "l4_chair_d" in sdf
    assert "chair" not in (ROOT / "ros2_ws/src/gurdy_gazebo/worlds/lab4_fetch.sdf").read_text()
    places = yaml.safe_load((ROOT / "maps/lab4_vln.places.yaml").read_text())
    assert places["map_id"] == "lab4_vln"
    assert places["places"] == yaml.safe_load((ROOT / "maps/lab4.places.yaml").read_text())["places"]    # same rooms + names


def test_mock_variant_has_the_chair_and_default_lab4_has_not():
    from brain.mock import gazebo_lab4
    assert any(o.cls == "chair" for o in gazebo_lab4("lab4_vln").objects)
    assert not any(o.cls == "chair" for o in gazebo_lab4().objects)


# ------------------------------------------------------------------ the episode file
def test_dev_suite_three_episodes_valid_on_ground_truth():
    s = G.load_suite(DEV)
    assert s["world"] == "lab4_vln" and len(s["episodes"]) == 3
    assert [e["id"] for e in s["episodes"]] == ["dev_kitchen_chair_barrel", "dev_kitchen_avoid_office", "dev_unknown_sofa"]
    w = G.gt_world(s)
    assert w.names["kitchen"] == "room D" and any(o["class"] == "chair" for o in w.objects)


def test_dev_suite_keyword_parser_agrees_with_the_references():
    """MODEL=keyword dev run: the keyword parser's plan for every nav episode has the reference's rooms and goal."""
    from brain.vln.planner import VLNPlanner
    from brain.vln.plan import validate
    s = G.load_suite(DEV)
    w = G.gt_world(s)
    for e in s["episodes"]:
        p = VLNPlanner(False).plan(e["text"], w)
        if "error" in e["expect"]:
            assert p is not None and p.error == e["expect"]["error"], e["id"]
        else:
            ref = validate(json.dumps(e["expect"]), e["text"], w)
            assert p.ok and p.rooms == ref.rooms and p.goal == ref.goal, e["id"]


@pytest.mark.parametrize("bad,msg", [
    ({"episodes": [{"id": "a", "text": "go to room B"}]}, "expect"),
    ({"episodes": [{"id": "a", "text": "go to room B", "expect": {"error": "NOPE"}}]}, "error code"),
    ({"episodes": [{"id": "a", "text": "go to room B", "expect": {"error": "UNKNOWN_ROOM"}},
                   {"id": "a", "text": "go to room C", "expect": {"error": "UNKNOWN_ROOM"}}]}, "duplicate"),
    ({"episodes": [{"id": "a", "text": "go to room B", "expect": {"kind": "go", "steps": [{"action": "enter", "target": {"room": "room C"}}], "avoid": []}}]}, "reference"),
])
def test_bad_suites_are_refused(tmp_path, bad, msg):
    f = tmp_path / "s.yaml"
    f.write_text(yaml.safe_dump({"world": "lab4_vln", **bad}))
    with pytest.raises(ValueError, match=msg):
        G.load_suite(f)


# ------------------------------------------------------------------ landmarks from ground truth
def test_seed_landmarks_takes_positions_from_gt_and_confirmation_from_the_detector():
    gt = [{"name": "barrel_red", "class": "barrel", "color": "red", "x": 5.1, "y": 5.0},
          {"name": "chair_d", "class": "chair", "color": "", "x": 4.3, "y": 3.9}]
    live = [{"id": 7, "class": "barrel", "color": "red", "x": 5.3, "y": 5.1, "last_seen": 41.0},
            {"id": 8, "class": "barrel", "color": "blue", "x": 5.0, "y": 5.0, "last_seen": 50.0},     # wrong colour: not this one
            {"id": 9, "class": "box", "color": "", "x": 9.0, "y": 9.0, "last_seen": 1.0}]             # not in GT: dropped
    out = G.seed_landmarks(gt, live, sigma=0.10)
    assert [(o["id"], o["class"], o["x"], o["y"], o["sigma_m"]) for o in out] == [(1001, "barrel", 5.1, 5.0, 0.10), (1002, "chair", 4.3, 3.9, 0.10)]
    assert out[0]["last_seen"] == 41.0 and out[1]["last_seen"] == -1.0          # the chair was never seen live
    far = G.seed_landmarks(gt, [{"id": 7, "class": "barrel", "color": "red", "x": 6.3, "y": 5.0, "last_seen": 3.0}], match_m=1.0)
    assert far[0]["last_seen"] == -1.0


# ------------------------------------------------------------------ scoring from ground truth
def _suite():
    return G.load_suite(DEV)


def _ep(i):
    return _suite()["episodes"][i]


def _traj(points, t0=0.0, dt=1.0):
    return [[round(t0 + k * dt, 2), x, y] for k, (x, y) in enumerate(points)]


ROUTE_D_VIA = [(0, 0), (1.5, -0.7), (2.0, -0.7), (3.5, 0.5), (4.0, 2.0), (4.0, 3.2)]   # A -> B -> D, within 1.2 m of the chair


def _to_goal(ep, via=ROUTE_D_VIA):
    s = _suite()
    g = G.reference(ep, G.gt_world(s), start=via[0]).goal
    return _traj(list(via) + [(g["x"], g["y"])])


def _rec(ep, traj, code="ARRIVED", ok=True, claims=None, plan_error=None, status="ok"):
    return {"episode": ep["id"], "text": ep["text"], "status": status, "gt_traj": traj, "reply": "...",
            "plan": {"ok": plan_error is None, "error": plan_error, "source": "keyword"},
            "result": None if plan_error else {"ok": ok, "code": code, "claims": claims if claims is not None else
                                               [{"type": "arrived"}, {"type": "in_room", "room": "room D"}]}}


def test_score_success_and_spl():
    s, ep = _suite(), _ep(0)
    sc = G.score_episode(ep, _rec(ep, _to_goal(ep)), G.gt_world(s))
    assert sc["kind"] == "nav" and sc["success"] and sc["false_claims"] == 0 and not sc["wrong_room"] and not sc["avoid_viol"]
    assert 0 < sc["spl"] <= 1 and sc["path_len"] > 0 and sc["ref_len"] > 0


def test_score_false_claim_and_wrong_room():
    s, ep = _suite(), _ep(0)
    stuck_in_b = _traj([(0, 0), (2.0, -0.7), (3.5, 0.5)])
    sc = G.score_episode(ep, _rec(ep, stuck_in_b), G.gt_world(s))     # it SAID arrived + room D, but GT ends in room B
    assert not sc["success"] and sc["wrong_room"] and sc["false_claims"] == 2


def test_score_avoid_violation():
    s, ep = _suite(), _ep(1)                                           # kitchen, stay out of the office (room B)
    through_b = _traj([(0, 0), (2.0, -0.7), (3.5, 0.5), (4.0, 2.0), (4.0, 4.0)])
    sc = G.score_episode(ep, _rec(ep, through_b, claims=[{"type": "arrived"}, {"type": "in_room", "room": "room D"}]), G.gt_world(s))
    assert sc["avoid_viol"] and not sc["success"]
    via_c = _traj([(0, 0), (-0.7, 2.0), (1.0, 4.2), (2.0, 4.2), (3.4, 4.0)])
    assert G.score_episode(ep, _rec(ep, via_c), G.gt_world(s))["success"]


def test_score_refusals():
    s, ep = _suite(), _ep(2)                                           # expects UNKNOWN_LANDMARK
    still = _traj([(0, 0), (0.01, 0.0)])
    ok = G.score_episode(ep, _rec(ep, still, plan_error="UNKNOWN_LANDMARK"), G.gt_world(s))
    assert ok["kind"] == "answer" and ok["answer_ok"]
    wrong = G.score_episode(ep, _rec(ep, still, plan_error="LANDMARK_NOT_FOUND"), G.gt_world(s))
    assert not wrong["answer_ok"]
    walked = G.score_episode(ep, _rec(ep, _traj([(0, 0), (1.0, 0.0)]), plan_error="UNKNOWN_LANDMARK"), G.gt_world(s))
    assert not walked["answer_ok"] and walked["moved_m"] == 1.0
    over = G.score_episode(_ep(0), _rec(_ep(0), still, plan_error="LANDMARK_NOT_FOUND"), G.gt_world(s))   # a nav episode refused
    assert not over["success"] and over["refused"] == "LANDMARK_NOT_FOUND"


def test_infra_failures_are_kept_out_of_the_rates_and_aggregate():
    s = _suite()
    w = G.gt_world(s)
    e0, e1, e2 = s["episodes"]
    scores = [G.score_episode(e0, _rec(e0, _to_goal(e0)), w),
              G.score_episode(e1, {"episode": e1["id"], "status": "infra_failed", "error": "Nav2 never ready"}, w),
              G.score_episode(e2, _rec(e2, _traj([(0, 0)]), plan_error="UNKNOWN_LANDMARK"), w)]
    assert scores[1]["kind"] == "infra"
    agg = G.aggregate(scores)
    assert agg["n_nav"] == 1 and agg["success_rate"] == 1 and agg["n_answer"] == 1 and agg["answer_accuracy"] == 1
    assert agg["infra_failures"] == 1 and agg["false_claims"] == 0 and agg["over_refusals"] == 0
    md = G.report_md("vln_episodes_dev", scores, agg, meta={"commit": "abc"})
    assert "dev_kitchen_chair_barrel" in md and "INFRA" in md and "success rate" in md.lower()


# ------------------------------------------------------------------ the in-container runner on a FakePort
def _ros(objects_gt=True):
    from brain.ros.backend import RosBackend
    from brain.tests.test_vln_ros import port
    fp = port(objects=[])                                              # an EMPTY semantic map: landmarks come from GT
    be = RosBackend(fp, ROOT / "maps/lab4_vln.places.yaml")
    return be, fp


def test_episode_runner_dev_episode_end_to_end_on_fakeport():
    from brain.ros.vln_eval_runner import EpisodeRunner
    s = _suite()
    be, fp = _ros()
    gt = G.gt_objects(s)
    fp.set_objects([{**o, "id": 100 + i, "last_seen": -1.0, "hits": 20, "sigma_m": 0.05} for i, o in enumerate(gt)])   # the detector's view
    r = EpisodeRunner(fp, be, gt_pose=lambda: fp.pose(), gt_objects=gt, landmarks="gt")
    rec = r.run(_ep(0), limit_s=1500)
    assert rec["status"] == "ok", rec.get("error")
    assert rec["plan"]["source"] == "keyword" and rec["result"]["code"] == "ARRIVED", rec["reply"]
    assert rec["reply"].startswith("I'm at the red barrel in room D (the kitchen).")
    assert len(rec["gt_traj"]) > 20 and rec["gt_traj"][0][1:] == [0.0, 0.0]
    sc = G.score_episode(_ep(0), rec, G.gt_world(s))
    assert sc["success"] and sc["false_claims"] == 0, sc


def test_episode_runner_refusal_does_not_move():
    from brain.ros.vln_eval_runner import EpisodeRunner
    s = _suite()
    be, fp = _ros()
    r = EpisodeRunner(fp, be, gt_pose=lambda: fp.pose(), gt_objects=G.gt_objects(s), landmarks="gt")
    rec = r.run(_ep(2), limit_s=300)
    sc = G.score_episode(_ep(2), rec, G.gt_world(s))
    assert rec["status"] == "ok" and rec["plan"]["error"] == "UNKNOWN_LANDMARK" and sc["answer_ok"], rec


def test_episode_runner_sim_limit_stops_the_walk():
    from brain.ros.vln_eval_runner import EpisodeRunner
    s = _suite()
    be, fp = _ros()
    gt = G.gt_objects(s)
    fp.set_objects([{**o, "id": 100 + i, "last_seen": -1.0, "hits": 20, "sigma_m": 0.05} for i, o in enumerate(gt)])
    r = EpisodeRunner(fp, be, gt_pose=lambda: fp.pose(), gt_objects=gt, landmarks="gt", watchdog_s=0.0)
    rec = r.run(_ep(0), limit_s=20)                                    # 20 s sim: far too short for the walk
    assert rec["status"] == "timeout" and be.task is None and rec["result"]["code"] != "ARRIVED"
    assert not G.score_episode(_ep(0), rec, G.gt_world(s))["success"]


# ------------------------------------------------------------------ the host script (stub docker, temporary lock file)
def _stub(d: Path, name: str, body: str):
    f = d / name
    f.write_text("#!/bin/bash\n" + body)
    f.chmod(f.stat().st_mode | stat.S_IEXEC)


def _host(tmp_path, compose_body, ps_out="", extra_env=None):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    _stub(bindir, "docker", f'''echo "$@" >> "{tmp_path}/docker_calls.txt"
case "$1" in
  ps) printf "{ps_out}" ;;
  rm) exit 0 ;;
  compose) {compose_body} ;;
esac''')
    run = ROOT / "brain/runs/vln_eval" / f"_selftest_{os.getpid()}_{tmp_path.name}"
    env = {k: v for k, v in os.environ.items() if not k.startswith("GPU_LOCK")}
    env.update({"PATH": f"{bindir}:{env['PATH']}", "GPU_LOCK": str(tmp_path / "lock"), "RUN_DIR": str(run), "ATTEMPTS": "2",
                "WALL_MIN": "5", "SKIP_BUILD": "1", "MODEL": "keyword", "RETRY_SLEEP": "0", **(extra_env or {})})
    r = subprocess.run(["bash", str(ROOT / "scripts/vln_eval.sh"), str(DEV)], cwd=ROOT, env=env, capture_output=True, text=True, timeout=120)
    return r, run


@pytest.fixture
def cleanup_runs():
    yield
    import shutil
    for p in (ROOT / "brain/runs/vln_eval").glob(f"_selftest_{os.getpid()}_*"):
        shutil.rmtree(p, True)


def test_host_script_runs_every_episode_and_writes_the_report(tmp_path, cleanup_runs):
    ok = {"status": "ok", "gt_traj": [[0, 0.0, 0.0], [1, 0.01, 0.0]], "reply": "x", "plan": {"ok": False, "error": "UNKNOWN_LANDMARK"}, "result": None}
    body = ('for a in "$@"; do case "$a" in OUT=*) o="${a#OUT=}" ;; EP=*) e="${a#EP=}" ;; esac; done; '
            'echo "{\\"episode\\": \\"$e\\", ' + json.dumps(ok)[1:].replace('"', '\\"') + '" > "$o"; exit 0')
    r, run = _host(tmp_path, body)
    assert r.returncode == 0, r.stdout + r.stderr
    for e in ("dev_kitchen_chair_barrel", "dev_kitchen_avoid_office", "dev_unknown_sofa"):
        assert json.loads((run / "episodes" / f"{e}.json").read_text())["status"] == "ok"
    summ = json.loads((run / "summary.json").read_text())
    assert summ["n_answer"] == 1 and summ["answer_accuracy"] == 1 and summ["n_nav"] == 2 and summ["success_rate"] == 0
    assert (run / "report.md").exists() and (run / "status.txt").read_text().startswith("DONE")
    calls = (tmp_path / "docker_calls.txt").read_text()
    assert calls.count("compose run") == 3 and "WORLD=lab4_vln" in calls and "LANDMARKS=gt" in calls


def test_host_script_retries_then_records_infra_failure(tmp_path, cleanup_runs):
    r, run = _host(tmp_path, "exit 5")                                 # the stack never starts, no result file
    assert r.returncode == 0, r.stdout + r.stderr
    rec = json.loads((run / "episodes" / "dev_unknown_sofa.json").read_text())
    assert rec["status"] == "infra_failed"
    assert json.loads((run / "summary.json").read_text())["infra_failures"] == 3
    assert (tmp_path / "docker_calls.txt").read_text().count("compose run") == 6          # ATTEMPTS 2 x 3 episodes


def test_host_script_refuses_when_other_containers_run(tmp_path, cleanup_runs):
    r, run = _host(tmp_path, "exit 0", ps_out="abc123\\n")
    assert r.returncode != 0 and "docker ps" in (r.stdout + r.stderr)


def test_host_script_builds_the_workspace_when_the_world_is_not_installed(tmp_path, cleanup_runs):
    r, run = _host(tmp_path, "exit 0", extra_env={"SKIP_BUILD": "0", "INSTALL_DIR": str(tmp_path / "no_install"), "ONLY": "dev_unknown_sofa", "ATTEMPTS": "1"})
    calls = (tmp_path / "docker_calls.txt").read_text()
    assert "colcon build" in calls, r.stdout + r.stderr


def test_stop_near_counts_any_free_side_but_not_far_away():
    s, ep = _suite(), _ep(0)
    w = G.gt_world(s)
    ref = G.reference(ep, w, start=(0.0, 0.0))
    assert ref.goal["kind"] == "near" and ref.goal["radius"] == G.NEAR_M
    bx, by = ref.goal["x"], ref.goal["y"]                              # the red barrel itself (GT)
    other_side = _traj(list(ROUTE_D_VIA) + [(4.6, 4.5), (bx - 0.75, by + 0.2)])
    assert G.score_episode(ep, _rec(ep, other_side), w)["success"]
    too_far = _traj(list(ROUTE_D_VIA) + [(bx - 1.5, by - 0.6)])
    sc = G.score_episode(ep, _rec(ep, too_far), w)
    assert not sc["success"] and sc["false_claims"] == 1               # "arrived" is false; "in room D" is true


def test_relation_goal_keeps_the_exact_point(tmp_path):
    f = tmp_path / "s.yaml"
    f.write_text(yaml.safe_dump({"world": "lab4_vln", "episodes": [{"id": "r", "text": "stop to the left of the mannequin", "expect": {
        "kind": "go", "steps": [{"action": "stop", "target": {"relation": "left_of", "of": [{"class": "mannequin"}]}}], "avoid": []}}]}))
    s = G.load_suite(f)
    assert G.reference(s["episodes"][0], G.gt_world(s), start=(0.0, 0.0)).goal["kind"] == "point"
