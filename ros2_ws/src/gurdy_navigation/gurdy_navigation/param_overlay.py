"""Optional Nav2 parameter overlays (no ROS imports): deep-merge an overlay YAML into the parsed nav2_params.

Dicts merge recursively, every other value (lists included) is replaced by the overlay's. The default parameter file
is never edited; a launch flag (Phase 9c: low_obstacles:=true, NOT added yet) would call apply_overlay before
patch_nav2_params.
"""
from __future__ import annotations

import copy
from pathlib import Path

import yaml

OVERLAYS = {"low_obstacles": Path(__file__).resolve().parents[1] / "config" / "nav2_low_obstacles_overlay.yaml"}


def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def apply_overlay(params: dict, overlay) -> dict:
    """overlay: a dict, a path, or a name in OVERLAYS."""
    if isinstance(overlay, str) and overlay in OVERLAYS:
        overlay = OVERLAYS[overlay]
    if not isinstance(overlay, dict):
        overlay = yaml.safe_load(Path(overlay).read_text())
    return deep_merge(params, overlay)
