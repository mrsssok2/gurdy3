"""Phase 9c: the low-obstacle Nav2 overlay (pure Python; no ROS)."""
from pathlib import Path

import yaml

from gurdy_navigation.param_overlay import apply_overlay, deep_merge

CFG = Path(__file__).resolve().parents[1] / "config"


def _base():
    return yaml.safe_load((CFG / "nav2_params.yaml").read_text())


def test_default_params_have_no_low_obstacle_layer():
    p = _base()
    assert "low_obstacle_layer" not in p["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]
    assert p["collision_monitor"]["ros__parameters"]["observation_sources"] == ["scan"]


def test_overlay_adds_layer_before_inflation_and_keeps_the_rest():
    base = _base()
    p = apply_overlay(base, "low_obstacles")
    lc = p["local_costmap"]["local_costmap"]["ros__parameters"]
    assert lc["plugins"] == ["obstacle_layer", "low_obstacle_layer", "inflation_layer"]
    assert lc["obstacle_layer"] == base["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]
    assert lc["low_obstacle_layer"]["low_mark"]["topic"] == "/low_obstacles/marking"
    assert lc["low_obstacle_layer"]["low_clear"]["clearing"] is True and lc["low_obstacle_layer"]["low_clear"]["marking"] is False
    assert p["global_costmap"] == base["global_costmap"]                       # global costmap untouched
    cm = p["collision_monitor"]["ros__parameters"]
    assert cm["observation_sources"] == ["scan", "low_obstacles"] and cm["scan"] == base["collision_monitor"]["ros__parameters"]["scan"]
    assert base == _base()                                                     # the input is not modified


def test_deep_merge_replaces_lists_and_merges_dicts():
    assert deep_merge({"a": {"b": 1, "c": [1]}, "d": 2}, {"a": {"c": [2]}}) == {"a": {"b": 1, "c": [2]}, "d": 2}


def test_overlay_then_patch_nav2_params():
    """The overlay composes with the robot.yaml patch used by nav.launch.py."""
    import pytest
    pytest.importorskip("numpy")
    from gurdy_core.robot import load_robot
    from gurdy_navigation.footprint import patch_nav2_params
    p = patch_nav2_params(apply_overlay(_base(), "low_obstacles"), load_robot())
    assert "low_obstacle_layer" in p["local_costmap"]["local_costmap"]["ros__parameters"]
