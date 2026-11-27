"""Looking around: find / explore must turn a full circle in each room (the camera sees only a cone), and explore must not report "done" for an
already mapped area without going there. Different objects / areas than the Gazebo eval missions. Run in the brain image: pytest brain/tests/test_search_sweep.py"""
import math

import pytest

from brain.ros import RosBackend
from brain.ros.fake_port import FakePort

PLACES = """map_id: fake_lab
places:
  home: {x: 0.0, y: 0.0, kind: waypoint}
  room a: {rect: [-2.0, -3.0, 2.5, 3.0], kind: room}
  room b: {rect: [2.5, -3.0, 6.5, 3.0], kind: room}
  stairs: {x: 1.0, y: -5.0, kind: hazard}
"""
CONE = {"id": 7, "class": "traffic_cone", "color": "orange", "x": -1.5, "y": -2.2, "z": 0.2, "hits": 9, "score": 0.8, "sigma_m": 0.04}
BARREL = {"id": 8, "class": "barrel", "color": "blue", "x": 6.0, "y": 2.4, "z": 0.3, "hits": 9, "score": 0.8, "sigma_m": 0.04}


def make(tmp_path, pose=(0.0, 0.0, 0.0)):
    f = tmp_path / "places.yaml"
    f.write_text(PLACES)
    port = FakePort(pose)
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


def total_turn(port_cmds_yaw: list[float]) -> float:
    return sum(abs(((b - a) + math.pi) % (2 * math.pi) - math.pi) for a, b in zip(port_cmds_yaw, port_cmds_yaw[1:]))


def test_find_turns_to_look_behind_and_finds_a_cone_in_the_corner(tmp_path):
    b, p = make(tmp_path)
    p.hide_objects([CONE])                                    # far corner of room A, behind the robot's travel direction
    assert b.find({"class": "traffic_cone", "color": "orange"}).ok
    run(b, p, 400)
    d = final(b)
    assert d["found"] and d["source"] == "live" and d["object"]["id"] == 7


def test_without_the_scan_the_same_search_misses_it(tmp_path):
    """The behaviour before the fix, kept as a regression guard that the scenario really needs the turn."""
    b, p = make(tmp_path)
    b.SCAN_QUARTERS = 0
    p.hide_objects([CONE])
    b.find({"class": "traffic_cone"}, max_time_s=200)
    run(b, p, 260)
    d = final(b)
    assert d["stopped_reason"] == "timeout" and not d["found"]


def test_find_searches_the_second_room_too(tmp_path):
    b, p = make(tmp_path)
    p.hide_objects([BARREL])                                  # corner of room B
    assert b.find({"class": "barrel"}).ok
    run(b, p, 500)
    d = final(b)
    assert d["found"] and d["object"]["id"] == 8


def test_the_scan_stops_as_soon_as_the_object_is_seen_and_covers_a_full_circle_otherwise(tmp_path):
    b, p = make(tmp_path)
    yaws: list[float] = []
    b.find({"class": "chair"}, max_time_s=500)                # not in this world: the whole sweep runs
    for _ in range(int(500 / 0.2)):
        p.advance(0.2)
        b.tick()
        yaws.append(p.yaw)
        if b.task is None:
            break
    assert final(b)["stopped_reason"] == "timeout" and not final(b)["found"]
    assert total_turn(yaws) > 2 * math.radians(330)           # a (nearly) full circle in each of the two rooms (+ driving turns)


def test_explore_of_an_already_mapped_room_still_goes_there_and_looks_around(tmp_path):
    b, p = make(tmp_path)
    p.explore_finish_after = 0.0                              # the explorer has nothing left to explore and ends at once
    p.hide_objects([BARREL])
    assert b.explore(area="room B").ok
    run(b, p, 400)
    d = final(b)
    assert d["stopped_reason"] == "done"
    assert b.where_am_i().data["place"] == "room b" or b.where_am_i().data["place"] == "room B"
    assert [o["id"] for o in d["new_objects"]] == [8]         # and what it saw there is reported


def test_explore_that_really_explores_does_not_add_a_sweep(tmp_path):
    b, p = make(tmp_path)
    p.explore_finish_after = 40.0
    b.explore(area="room A")
    run(b, p, 200)
    assert final(b)["stopped_reason"] == "done"
    assert p.nav_log == []                                    # no extra navigation goals after a real exploration


def test_sweep_never_enters_a_hazard_or_a_no_go_zone(tmp_path):
    b, p = make(tmp_path)
    assert b.no_go_zone("add", place="room B").ok
    steps = b._sweep_steps()
    assert [s["name"].lower() for s in steps if s["kind"] == "nav"] == ["room a"]
    assert b._sweep_steps("stairs") == []


def test_wait_step_stands_still_for_its_time(tmp_path):
    b, p = make(tmp_path)
    p.hide_objects([])
    b.find({"class": "chair"}, max_time_s=30)
    run(b, p, 8)
    assert b.task is not None                                 # still scanning after 8 s: waits and turns take time


# ---------------------------------------------------------------- 2026-11-27: multi-viewpoint room sweep (default off)
def _nav_points(steps):
    return [(s["x"], s["y"]) for s in steps if s["kind"] == "nav"]


def test_default_sweep_is_one_viewpoint_per_room(tmp_path):
    b, _ = make(tmp_path)
    assert b.search_view_radius_m is None
    pts = _nav_points(b._sweep_steps())
    assert len(pts) == 2                                                     # room a, room b


def test_view_radius_covers_every_point_of_the_room(tmp_path):
    b, port = make(tmp_path)
    b.search_view_radius_m = 1.5
    for name in [n for n, v in b.places.items() if v.get("kind") == "room"]:
        room = b.places[name]
        vps = b._room_viewpoints(room)
        assert vps[0] == (room["x"], room["y"]) and len(vps) > 1
        x0, y0, x1, y1 = room["rect"]
        worst = max(min(math.hypot(x - vx, y - vy) for vx, vy in vps)
                    for x in [x0 + (x1 - x0) * i / 20 for i in range(21)] for y in [y0 + (y1 - y0) * j / 20 for j in range(21)])
        assert worst <= 1.5 + b.VIEWPOINT_CLEARANCE_M                        # grid radius + the wall-clearance shift
    steps = b._sweep_steps()
    assert sum(1 for s in steps if s.get("mark_room")) == 2                  # still one 'room looked at' mark per room
    assert len(_nav_points(steps)) > 2 and all(s.get("optional") for s in steps if s["kind"] == "nav")


def test_viewpoints_skip_occupied_cells(tmp_path):
    b, port = make(tmp_path)
    b.search_view_radius_m = 1.5
    room = next(v for v in b.places.values() if v.get("kind") == "room")
    free = b._room_viewpoints(room)
    port.occupied = [(x, y, 0.1) for x, y in free[1:]]                       # block every extra viewpoint
    assert b._room_viewpoints(room) == [free[0]]


def test_fetch_search_sets_the_radius_only_for_its_search(tmp_path):
    from brain.ros.fetch_robot import FetchConfig, RosFetchRobot
    b, port = make(tmp_path)
    seen = []
    b.find = lambda **kw: (seen.append(b.search_view_radius_m), type("R", (), {"ok": False})())[1]
    r = RosFetchRobot.__new__(RosFetchRobot)
    r.b, r.cfg = b, FetchConfig(search_view_radius_m=1.5)
    assert r.search({"class": "can"}) is False
    assert seen == [1.5] and b.search_view_radius_m is None
