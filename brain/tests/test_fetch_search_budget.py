"""Phase 9k: the fetch search time limit is config-driven and sized from the map (RosBackend.search_budget_s), not a fixed
300 s (2026-11-27 dry run: a whole-house sweep with search_view_radius_m 1.5 needs > 900 s). FakePort / FakeArmPort only."""
import pytest

from brain.ros.fetch_robot import FetchConfig
from brain.tests.test_ros_fetch import rig


def search(lim, radius, **cfg):
    s, rob, arm, fp = rig(unseen={"blue_cube"})
    fp.reveal = []                                       # never found: the search runs until its limit / the sweep ends
    rob.cfg = FetchConfig(search_timeout_s=lim, search_view_radius_m=radius, **cfg)
    found = rob.search({"class": "small_box", "color": "blue"})
    return found, rob


def test_budget_grows_with_the_viewpoints_of_the_map():
    s, rob, arm, fp = rig()
    one = rob.engine.search_budget_s()
    rob.engine.search_view_radius_m = 1.5
    many = rob.engine.search_budget_s()
    assert 300 < one < many and many > 900               # lab4, 4 rooms: one viewpoint each vs a 1.5 m grid


def test_budget_shrinks_when_a_room_is_a_no_go_zone(tmp_path, monkeypatch):
    s, rob, arm, fp = rig()
    full = rob.engine.search_budget_s()
    monkeypatch.setattr(rob.engine, "_save_places", lambda: None)     # never write the zone into maps/*.places.yaml
    rob.engine.no_go_zone("add", place="room D")
    assert rob.engine.search_budget_s() < full


def test_fixed_300_s_cuts_the_whole_house_sweep_short():
    found, rob = search(300.0, 1.5)
    assert not found and sorted(rob.engine._scanned) == ["room A"]


def test_map_sized_limit_finishes_the_whole_house_sweep():
    found, rob = search(None, 1.5)
    lim = next(x["search_limit_s"] for x in rob.log if "search_limit_s" in x)
    assert lim > 900 and sorted(rob.engine._scanned) == ["room A", "room B", "room C", "room D"]
    assert rob.engine.missions[-1]["args"]["max_time_s"] == lim


@pytest.mark.parametrize("lo,hi,expect", [(300.0, 2400.0, None), (2000.0, 2400.0, 2000.0), (300.0, 600.0, 600.0)])
def test_limit_is_clamped(lo, hi, expect):
    s, rob, arm, fp = rig()
    rob.engine.search_view_radius_m = 1.5
    rob.cfg = FetchConfig(search_min_s=lo, search_max_s=hi)
    got = rob.search_limit_s()
    assert got == (expect if expect is not None else rob.engine.search_budget_s())


def test_fixed_number_is_used_as_is():
    s, rob, arm, fp = rig()
    rob.cfg = FetchConfig(search_timeout_s=123.0)
    assert rob.search_limit_s() == 123.0
