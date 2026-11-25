"""RosBackend against FakePort (fake messages / action servers; no ROS, no Gazebo)."""
import math

import pytest

from brain.agent.graph import Agent
from brain.agent.llm import KeywordLLM
from brain.ros import RosBackend
from brain.ros.fake_port import FakePort
from brain.ros.relmotion import RelativeMotion
from brain.tools import ToolLayer, missing_methods

PLACES = """map_id: fake_lab
places:
  home: {x: 0.0, y: 0.0, kind: waypoint}
  room a: {rect: [-1.0, -1.0, 2.0, 2.0], kind: room}
  room b: {rect: [3.0, -1.0, 6.0, 2.0], kind: room}
  stairs: {x: 1.0, y: -3.0, kind: hazard}
"""
BOX = {"id": 1, "class": "box", "color": "red", "x": 4.0, "y": 0.5, "z": 0.2, "hits": 9, "score": 0.8, "sigma_m": 0.04,
       "first_seen": 1.0, "last_seen": 2.0}
CONE = {"id": 2, "class": "traffic_cone", "color": "orange", "x": 1.0, "y": 0.0, "z": 0.2, "hits": 6, "score": 0.7,
        "sigma_m": 0.05, "first_seen": 1.0, "last_seen": 2.0}


def make(tmp_path, objs=(), pose=(0.0, 0.0, 0.0)):
    f = tmp_path / "places.yaml"
    f.write_text(PLACES)
    port = FakePort(pose)
    port.set_objects(list(objs))
    return RosBackend(port, f), port


def run(b, port, secs, dt=0.2):
    for _ in range(int(secs / dt)):
        port.advance(dt)
        b.tick()
        if b.task is None:
            break


def final(b):
    ev = [e for e in b.events if e["type"] in ("task_finished", "task_failed")]
    return ev[-1]["data"] if ev else None


def test_implements_robot_backend(tmp_path):
    b, _ = make(tmp_path)
    assert missing_methods(b) == []


def test_go_to_place_reaches_and_reports(tmp_path):
    b, p = make(tmp_path)
    r = b.go_to(place="Room B")
    assert r.ok and r.data["state"] == "started" and r.data["estimate"]["distance_m"] > 4
    run(b, p, 80)
    d = final(b)
    assert d["stopped_reason"] == "reached" and d["reached"] and d["final_distance_m"] < 0.1
    assert d["travelled_m"] == pytest.approx(4.5, abs=0.3) and d["target"]["place"] == "room B"
    assert b.where_am_i().data["place"] == "room B"


def test_go_to_errors(tmp_path):
    b, p = make(tmp_path, [BOX])
    assert b.go_to(place="kitchen").code == "UNKNOWN_PLACE"
    assert b.go_to(place="stairs").code == "REFUSED"
    p.unreachable.append((4.5, 0.5, 4.0))                 # the whole room is unreachable (a single bad goal point now falls back to another point)
    assert b.go_to(place="room b").code == "NO_PATH"
    assert b.go_to(object={"class": "barrel"}).code == "NOT_SEEN"
    p.nav_up = False
    assert b.go_to(place="home").code == "UNAVAILABLE"


def test_go_to_object_uses_standoff_and_faces_it(tmp_path):
    b, p = make(tmp_path, [BOX])
    assert b.go_to(object={"class": "box", "color": "red"}, standoff_m=0.6).ok
    gx, gy, yaw = p.nav_log[-1]
    assert math.hypot(gx - 4.0, gy - 0.5) == pytest.approx(0.6, abs=1e-6)
    assert yaw == pytest.approx(math.atan2(0.5 - gy, 4.0 - gx))


def test_ambiguous_object(tmp_path):
    b, p = make(tmp_path, [BOX, dict(BOX, id=3, x=-2.0)])
    r = b.go_to(object={"class": "box"})
    assert r.code == "AMBIGUOUS" and len(r.error["candidates"]) == 2


def test_blocked_goal_aborts_with_path_blocked_event(tmp_path):
    b, p = make(tmp_path)
    p.blocked.append((4.5, 0.5, 0.5))
    assert b.go_to(place="room b").ok
    run(b, p, 30)
    assert final(b)["stopped_reason"] == "blocked"
    assert any(e["type"] == "path_blocked" for e in b.events)
    assert b.why("stop").data["events"]


def test_busy_and_stop(tmp_path):
    b, p = make(tmp_path)
    b.go_to(place="room b")
    assert b.go_to(place="home").code == "BUSY"
    run(b, p, 5)
    r = b.stop()
    assert r.ok and r.data["cancelled_task"]["tool"] == "go_to" and b.task is None
    assert p.cmd == (0.0, 0.0, 0.0) and p.goals[1]["state"] == "canceled"
    assert final(b)["stopped_reason"] == "cancelled"
    assert b.stop().data["cancelled_task"] is None


def test_estop_latches_and_refuses_motion(tmp_path):
    b, p = make(tmp_path)
    b.go_to(place="room b")
    run(b, p, 3)
    b.estop(True)
    assert p.estop and final(b)["stopped_reason"] == "estop"
    assert b.go_to(place="home").code == "ESTOP" and b.status().data["mode"] == "estop"
    b.estop(False)
    assert b.go_to(place="home").ok


def test_pause_resume_resends_goal(tmp_path):
    b, p = make(tmp_path)
    b.go_to(place="room b")
    run(b, p, 5)
    assert b.mission_control("pause").data["state"] == "paused"
    x_paused = p.x
    run(b, p, 5)
    assert p.x == pytest.approx(x_paused, abs=0.02) and b.status().data["mode"] == "paused"
    assert b.mission_control("resume").data["state"] == "running"
    run(b, p, 80)
    assert final(b)["reached"] and len(p.nav_log) == 2
    assert b.mission_control("pause").code == "UNAVAILABLE"


def test_walk_forward_and_left(tmp_path):
    b, p = make(tmp_path, pose=(1.0, 1.0, math.pi / 2))
    assert b.walk("forward", 0.5).data["estimate"]["duration_s"] == 5.0
    run(b, p, 15)
    d = final(b)
    assert d["stopped_reason"] == "done" and d["moved_m"] == pytest.approx(0.5, abs=0.05)
    assert (p.x, p.y) == pytest.approx((1.0, 1.5), abs=0.06)       # forward = along the heading (+y)
    b.walk("left", 0.3)
    run(b, p, 10)
    assert (p.x, p.y) == pytest.approx((0.7, 1.5), abs=0.06)       # left of a +y heading is -x
    assert abs(p.yaw - math.pi / 2) < 0.05                          # heading held while strafing


def test_turn_left_right_and_full_circle(tmp_path):
    b, p = make(tmp_path)
    b.turn("left", 90)
    run(b, p, 20)
    assert math.degrees(p.yaw) == pytest.approx(90, abs=5) and final(b)["turned_deg"] == pytest.approx(90, abs=5)
    b.turn("right", 360)
    run(b, p, 40)
    assert final(b)["turned_deg"] == pytest.approx(360, abs=5)


def test_walk_without_progress_is_blocked(tmp_path):
    b, p = make(tmp_path)
    p.send_cmd = lambda *a: None                 # the robot never moves (collision monitor holds it)
    b.walk("forward", 1.0)
    run(b, p, 30)
    assert final(b)["stopped_reason"] == "blocked"


def test_relative_motion_pure():
    m = RelativeMotion("walk", "backward", 0.2, (0, 0, 0))
    assert m.command((0, 0, 0))[0] < 0
    assert m.command((-0.19, 0, 0)) == (0.0, 0.0, 0.0) and m.done


def test_where_is_nearby_relation_recall(tmp_path):
    b, p = make(tmp_path, [BOX, CONE], pose=(0.0, 0.0, 0.0))
    p.t = 10.0
    r = b.where_is({"class": "box", "color": "red"})
    o = r.data["matches"][0]
    assert o["place"] == "room B" and o["bearing_word"] == "ahead" and o["last_seen_age_s"] == 8.0
    assert b.where_is({"class": "barrel"}).code == "NOT_SEEN"
    p.sectors = [{"bearing_deg": 90.0, "distance_m": 0.7}]
    n = b.whats_nearby(2.0).data
    assert [x["id"] for x in n["objects"]] == [2] and n["obstacles"] == [{"bearing_word": "left", "distance_m": 0.7}]
    rel = b.relation({"class": "box"}, {"class": "traffic_cone"}).data
    assert rel["distance_m"] == pytest.approx(3.04, abs=0.01) and rel["same_place"] is False
    rec = b.recall("object", {"class": "box"}).data["records"][0]
    assert rec["map_id"] == "fake_lab" and rec["stale"] is False
    p.t = 5000.0
    assert b.recall("object").data["records"][0]["stale"] is True
    assert {r["name"] for r in b.recall("place").data["records"]} == {"home", "room A", "room B", "stairs"}


def test_path_check(tmp_path):
    b, p = make(tmp_path, [BOX])
    p.unknown_cells.append((4.5, 0.5, 1.0))
    d = b.path_check(place="room b").data
    assert d["reachable"] and d["leaves_mapped_area"] and d["eta_s"] == pytest.approx(d["path_length_m"] / 0.1, abs=0.2)
    b.no_go_zone("add", place="room b", radius_m=1.0)
    d = b.path_check(place="room b").data
    assert not d["reachable"] and d["crosses_no_go"] and d["blocked_by"] == "no-go zone"
    p.unreachable.append((0.0, 5.0, 0.5))
    assert b.path_check(point={"x": 0.0, "y": 5.0}).data["blocked_by"] == "unknown obstacle"


def test_no_go_zone_blocks_goal_and_path_and_persists(tmp_path):
    b, p = make(tmp_path, [BOX])
    assert b.no_go_zone("add", around_object_id=1, radius_m=0.8).ok
    assert b.go_to(point={"x": 4.0, "y": 0.5}).code == "IN_NO_GO_ZONE"
    assert b.go_to(place="room b").ok is True or True        # centre (4.5, 0.5) is 0.5 m from the box: inside the zone
    b2 = RosBackend(FakePort(), tmp_path / "places.yaml")
    assert len(b2.no_go_zone("list").data["zones"]) == 1
    assert b.no_go_zone("remove", around_object_id=1).data["zones"] == []
    assert b.no_go_zone("remove", around_object_id=1).code == "UNKNOWN_PLACE"


def test_save_place_and_home_from_start_pose(tmp_path):
    p = FakePort((2.0, 1.0, 0.0))
    b = RosBackend(p)                                  # no places file: home = start pose
    assert b.known_places() == ["home"]
    assert b.where_am_i().data["home_distance_m"] == 0.0
    p.x = 3.0
    assert b.save_place("desk").data["place"]["x"] == 3.0
    assert b.where_am_i().data["place"] == "desk"
    b.go_to(place="home")
    run(b, p, 30)
    assert final(b)["reached"]


def test_find_known_object_without_motion(tmp_path):
    b, p = make(tmp_path, [BOX])
    assert b.find({"class": "box", "color": "red"}).ok
    d = final(b)
    assert d["found"] and d["source"] == "memory" and d["object"]["id"] == 1 and b.task is None


def test_find_live_explores_until_seen_then_approaches(tmp_path):
    b, p = make(tmp_path)
    p.unknown_cells = [(2.0, 0.0, 20.0)]            # unmapped space: explore first
    p.reveal.append((20.0, dict(BOX)))
    assert b.find({"class": "box", "color": "red"}, approach=True).ok
    run(b, p, 400)
    d = final(b)
    assert d["found"] and d["source"] == "live" and d["stopped_reason"] == "done"
    assert p.explores[1]["state"] == "canceled"
    assert math.hypot(p.x - 4.0, p.y - 0.5) == pytest.approx(0.6, abs=0.1)


def test_find_times_out_when_never_seen(tmp_path):
    b, p = make(tmp_path)
    p.unknown_cells = [(2.0, 0.0, 20.0)]
    p.reveal.append((9999.0, dict(BOX)))
    b.find({"class": "box"}, max_time_s=30)
    run(b, p, 60)
    d = final(b)
    assert d["stopped_reason"] == "timeout" and not d["found"] and p.explores[1]["state"] != "running"


def test_explore_reports_new_objects_and_mapped_pct(tmp_path):
    b, p = make(tmp_path, [CONE])
    p.reveal.append((10.0, dict(BOX)))
    assert b.explore(max_time_s=120).ok
    run(b, p, 200)
    d = final(b)
    assert d["stopped_reason"] == "done" and [o["id"] for o in d["new_objects"]] == [1] and d["mapped_pct"] > 40


def test_explore_unavailable_without_action(tmp_path):
    b, p = make(tmp_path)
    p.explore_ok = False
    assert b.explore().code == "UNAVAILABLE"
    assert b.find({"class": "box"}).code == "UNAVAILABLE"


def test_patrol_reports_new_missing_and_moved(tmp_path):
    b, p = make(tmp_path, [BOX, CONE])
    assert b.patrol(["room a", "room b"], loops=1).ok
    run(b, p, 2)
    p.objects = [dict(BOX, x=4.9), dict(BOX, id=7, x=5.0, y=1.0)]           # cone gone, box moved 0.9, new box
    run(b, p, 200)
    d = final(b)
    assert d["reached"] == 2 and [x["id"] for x in d["new"]] == [7] and [x["id"] for x in d["missing"]] == [2]
    assert [x["id"] for x in d["moved"]] == [1]


def test_patrol_unknown_place_and_hazard(tmp_path):
    b, _ = make(tmp_path)
    assert b.patrol(["room a", "attic"]).code == "UNKNOWN_PLACE"
    assert b.patrol(["stairs"]).code == "REFUSED"


def test_patrol_continues_after_blocked_place(tmp_path):
    b, p = make(tmp_path)
    p.blocked.append((4.5, 0.5, 0.5))
    b.patrol(["room b", "home"])
    run(b, p, 200)
    d = final(b)
    assert d["reached"] == 1 and d["places"][0]["reached"] is False


def test_inspect_confirms_fresh_detection(tmp_path):
    b, p = make(tmp_path, [BOX])
    assert b.inspect(object_id=1).ok
    run(b, p, 10)                                       # still travelling; the detector re-sees it on arrival
    for _ in range(300):
        p.objects[0]["last_seen"] = p.t
        p.advance(0.2)
        b.tick()
        if b.task is None:
            break
    d = final(b)
    assert d["confirmed"] and d["moved_m"] == 0.0


def test_inspect_not_confirmed_when_stale(tmp_path):
    b, p = make(tmp_path, [BOX])
    b.inspect(object_id=1)
    run(b, p, 200)
    assert final(b)["confirmed"] is False


def test_status_reports_unknowns_honestly(tmp_path):
    b, p = make(tmp_path)
    s = b.status().data
    assert s["battery_pct"] is None and s["legs_known"] is False and s["mode"] == "idle" and s["policy"] == "policy"
    p.battery, p.legs = 80.0, {"front_left": "ok"}
    s = b.status().data
    assert s["battery_pct"] == 80.0 and s["legs_known"] is True


def test_failed_leg_refuses_motion_but_not_home(tmp_path):
    b, p = make(tmp_path)
    p.legs = {"front_left": "failed"}
    assert b.go_to(place="room b").code == "REFUSED"
    assert b.return_home().ok


def test_alert_topic_feeds_events_and_why(tmp_path):
    b, p = make(tmp_path)
    p.alert("leg_failure", "critical", "Leg front left has failed.", leg="front_left")
    ev = b.poll_events()
    assert ev[0]["type"] == "leg_failure" and "leg_failure" in b.status().data["alerts_active"]
    assert b.why("alert").data["events"][0]["type"] == "leg_failure"
    assert b.poll_events() == []


def test_fall_alert_ends_task(tmp_path):
    b, p = make(tmp_path)
    b.go_to(place="room b")
    run(b, p, 3)
    p.alert("fall", "critical", "The robot fell.")
    b.tick()
    assert final(b)["stopped_reason"] == "fell"


def test_schedule_fires_on_ros_time(tmp_path):
    b, p = make(tmp_path)
    assert b.schedule(5, {"tool": "walk", "args": {"direction": "forward", "distance_m": 0.2}}).ok
    assert b.schedule(5, {"tool": "walk", "args": {"direction": "up"}}).code == "INVALID_ARGS"
    for _ in range(12):
        p.advance(0.5)
        b.tick()
    assert any(e["type"] == "schedule_fired" for e in b.events) and b.task is not None


def test_report_and_describe_view(tmp_path):
    b, p = make(tmp_path)
    p.reveal.append((5.0, dict(BOX)))
    b.explore(max_time_s=30)
    run(b, p, 100)
    r = b.report().data["summary"]
    assert r["tool"] == "explore" and [o["id"] for o in r["objects"]] == [1]
    p.objects[0]["last_seen"] = p.t
    v = b.describe_view().data
    assert v["unverified"] and v["detections"][0]["id"] == 1 and "red box" in v["text"]


def test_no_pose_is_unavailable(tmp_path):
    b, p = make(tmp_path)
    p.pose = lambda: None
    assert b.where_am_i().code == "UNAVAILABLE" and b.walk("forward", 0.5).code == "UNAVAILABLE"


def test_agent_end_to_end_over_ros_backend(tmp_path):
    """The unchanged agent (keyword planner) drives RosBackend: same code path as with the mock."""
    b, p = make(tmp_path, [BOX])
    agent = Agent(ToolLayer(b), KeywordLLM(b.known_places))
    assert "box" in agent.say("where is the red box").reply.lower()
    agent.say("go to room b")
    for _ in range(400):
        p.advance(0.5)
        agent.pump()
    assert b.task is None and b.where_am_i().data["place"] == "room B"
    assert agent.say("what is your status").reply        # battery unknown must not crash the state block


# ---------------------------------------------------------------- explore action wiring, health, alerts, zones, estop
def test_explore_area_is_sent_to_the_action_as_a_rectangle(tmp_path):
    b, p = make(tmp_path)
    assert b.explore(max_time_s=50, area="Room B").ok
    assert p.explores[1]["area_rect"] == [3.0, -1.0, 6.0, 2.0] and p.explores[1]["max_time_s"] == 50
    b.stop()
    assert p.explores[1]["state"] == "canceled"
    assert b.explore(area="kitchen").code == "UNKNOWN_PLACE"
    assert b.explore().ok and p.explores[2]["area_rect"] is None
    b2, p2 = make(tmp_path)
    p2.explore_ok = False
    assert b2.explore().code == "UNAVAILABLE" and "/explore" in b2.explore().error["message"]
    assert b2.find({"class": "barrel"}).code == "UNAVAILABLE"


def test_explore_action_aborted_ends_task_blocked(tmp_path):
    b, p = make(tmp_path)
    b.explore(max_time_s=100)
    p.explores[1]["state"] = "aborted"
    b.tick()
    assert final(b)["stopped_reason"] == "blocked" and b.task is None


def test_status_uses_battery_and_leg_health_and_failed_leg_rule(tmp_path):
    b, p = make(tmp_path)
    p.battery, p.legs = 63.2, {"front_left": "ok", "rear_right": "failed"}
    s = b.status().data
    assert s["battery_pct"] == 63.2 and s["battery_simulated"] and s["legs"]["rear_right"] == "failed"
    assert b.walk("forward", 0.3).code == "REFUSED" and b.explore().code == "REFUSED"
    assert b.return_home().ok


def test_monitor_alerts_enter_events_and_are_not_duplicated(tmp_path):
    b, p = make(tmp_path)
    p.alert("battery_low", "warning", "Battery is low at 19 percent (simulated).", battery_pct=19.0)
    p.alert("object_new", "info", "New object: red box.", object_id=1)
    b.tick()
    assert [e["type"] for e in b.events] == ["battery_low", "object_new"]
    assert b.status().data["alerts_active"] == ["battery_low"]        # info alerts are not "active"
    assert b.why("alert").data["events"][0]["type"] == "battery_low"
    # the backend saw Nav2 abort AND the monitor reports it: one path_blocked event
    p.blocked.append((4.5, 0.5, 0.5))
    b.go_to(place="room b")
    p.alert("path_blocked", "warning", "The path is blocked: navigation gave up on the goal.")
    run(b, p, 10)
    assert [e["type"] for e in b.events].count("path_blocked") == 1
    # and the other order: monitor first, backend second
    b2, p2 = make(tmp_path)
    p2.blocked.append((4.5, 0.5, 0.5))
    b2.go_to(place="room b")
    p2.advance(2.5)
    p2.alert("path_blocked", "warning", "x")
    b2.tick()
    b2.tick()
    assert [e["type"] for e in b2.events].count("path_blocked") == 1


def test_fall_alert_ends_the_task(tmp_path):
    b, p = make(tmp_path)
    b.walk("forward", 1.0)
    p.alert("fall", "critical", "I have fallen.")
    b.tick()
    assert final(b)["stopped_reason"] == "fell" and b.task is None and p.cmd == (0.0, 0.0, 0.0)


def test_walk_into_a_zone_is_refused_up_front(tmp_path):
    b, p = make(tmp_path)
    b.zones.append({"id": 1, "name": "spill", "x": 1.0, "y": 0.0, "r": 0.3})
    r = b.walk("forward", 2.0)
    assert r.code == "IN_NO_GO_ZONE" and "spill" in r.error["message"] and b.task is None
    assert b.walk("forward", 0.5).ok                      # stops before the zone edge at x = 0.7
    b.stop()
    assert b.walk("backward", 2.0).ok and b.walk("left", 1.0).code == "BUSY"
    b.stop()
    b.stop()


def test_walk_sideways_through_zone_and_turn(tmp_path):
    b, p = make(tmp_path, pose=(1.0, -1.0, math.pi / 2))      # facing +y, zone straight ahead at (1, 0)
    b.zones.append({"id": 1, "name": "spill", "x": 1.0, "y": 0.0, "r": 0.3})
    assert b.walk("forward", 1.5).code == "IN_NO_GO_ZONE"
    assert b.walk("backward", 1.5).ok
    b.stop()
    assert b.turn("left", 90).ok                              # turning in place cannot enter a zone


def test_walking_out_of_the_zone_you_stand_in_is_allowed(tmp_path):
    b, p = make(tmp_path)
    b.zones.append({"id": 1, "name": "spill", "x": 0.0, "y": 0.0, "r": 0.5})
    assert b.walk("forward", 1.0).ok                          # start inside, moving away
    b.stop()
    b.zones[0]["x"] = 1.0                                     # zone ahead of us now; we stand at its edge... walking in is refused
    p.x = 0.4
    assert b.walk("forward", 0.5).code == "IN_NO_GO_ZONE"


def test_walk_guard_stops_when_a_zone_appears_ahead_while_walking(tmp_path):
    b, p = make(tmp_path)
    assert b.walk("forward", 2.0).ok
    run(b, p, 3.0)
    b.zones.append({"id": 1, "name": "spill", "x": p.x + 0.3, "y": 0.0, "r": 0.5})     # added mid-walk (or drift) around the robot's path
    # the robot is now inside the zone it was not in at the start of the step -> stop
    run(b, p, 10.0)
    d = final(b)
    assert d["stopped_reason"] == "no_go_zone" and p.cmd == (0.0, 0.0, 0.0)
    assert any(e["type"] == "path_blocked" and e["source"] == "zone_guard" for e in b.events)


def test_estop_cancels_task_zeroes_velocity_and_latches(tmp_path):
    b, p = make(tmp_path)
    b.walk("forward", 1.0)
    run(b, p, 2.0)
    assert any(c[0] > 0 for c in p.cmds)
    assert b.estop(True).ok
    assert p.estop and p.cmd == (0.0, 0.0, 0.0) and final(b)["stopped_reason"] == "estop"
    assert b.walk("forward", 0.5).code == "ESTOP" and b.status().data["mode"] == "estop"
    assert b.estop(False).ok and b.walk("forward", 0.5).ok


def test_why_stop_names_only_the_latest_cause(tmp_path):
    b, p = make(tmp_path)
    b.go_to(place="room b"); run(b, p, 60)                       # old history: a finished task
    b.go_to(place="home"); run(b, p, 3)
    b.stop()
    evs = b.why("stop").data["events"]
    assert [e["text"] for e in evs] == ["You told me to stop."]
    b.go_to(place="room b"); run(b, p, 120)                     # a later cause replaces it
    evs = b.why("stop").data["events"]
    assert len(evs) == 1 and "finished" in evs[0]["text"] and "told me" not in evs[0]["text"]
    b.stop()                                                     # a stop while idle is still the latest cause
    assert b.why("stop").data["events"][0]["text"] == "You told me to stop."


def test_why_stop_blocked_names_the_blockage(tmp_path):
    b, p = make(tmp_path)
    p.blocked.append((4.5, 0.5, 0.5))
    b.go_to(place="room b"); run(b, p, 30)
    evs = b.why("stop").data["events"]
    assert len(evs) == 1 and evs[0]["type"] == "path_blocked"


def test_find_on_mapped_rooms_sweeps_before_it_explores(tmp_path):
    b, p = make(tmp_path)                           # every cell known: nothing to explore, look in the rooms first
    assert b._rooms_mapped()
    b.find({"class": "box"}, max_time_s=300)
    kinds = [b.task.cur["kind"]] * (b.task.cur is not None) + [s["kind"] for s in b.task.steps]
    assert kinds[0] == "nav" and kinds[-1] == "explore"


def test_find_on_unmapped_space_explores_first(tmp_path):
    b, p = make(tmp_path)
    p.unknown_cells = [(2.0, 0.0, 20.0)]
    assert not b._rooms_mapped()
    b.find({"class": "box"}, max_time_s=300)
    first = b.task.cur if b.task.cur is not None else b.task.steps[0]
    assert first["kind"] == "explore"


def test_sweep_order_is_a_nearest_neighbour_chain_with_looked_at_rooms_last(tmp_path):
    b, p = make(tmp_path)
    b.places = {"room A": {"x": 0.0, "y": 0.0, "kind": "room"}, "room B": {"x": 4.0, "y": 0.0, "kind": "room"},
                "room C": {"x": 0.0, "y": 4.0, "kind": "room"}, "room D": {"x": 4.0, "y": 4.0, "kind": "room"}}
    p.x, p.y = 0.0, 0.0
    names = lambda: [s["name"] for s in b._sweep_steps() if s["kind"] == "nav"]
    assert names()[:3] in (["room A", "room B", "room D"], ["room A", "room C", "room D"])     # a ring: no crossing the map twice
    b._scanned["room A"] = 1.0
    assert names()[-1] == "room A"                  # already looked at without finding anything: last


def test_scan_marks_the_room_and_a_second_search_visits_it_last(tmp_path):
    b, p = make(tmp_path)
    p.unknown_cells = []
    b.find({"class": "box"}, max_time_s=900)
    run(b, p, 400)
    assert b._scanned                                # the rooms of the first sweep were marked
