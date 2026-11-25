#!/usr/bin/env python3
"""Eval copy of the lab4 world for the incident-graph evaluation: the three objects the scenario moves / removes (red box, red barrel, toolbox) are
DYNAMIC instead of static, so Gazebo set_pose / remove really move them. The world used by every other suite (lab4.sdf) is not touched.
  python3 scripts/make_incident_world.py        # writes ros2_ws/src/gurdy_gazebo/worlds/lab4_incident.sdf (world name stays `lab4`)"""
import re
from pathlib import Path

W = Path(__file__).resolve().parents[1] / "ros2_ws/src/gurdy_gazebo/worlds"
MODELS = ("l4_red_box", "l4_barrel_red", "l4_toolbox")


def main() -> None:
    s = (W / "lab4.sdf").read_text()
    for m in MODELS:
        s, n = re.subn(rf'(<model name="{m}">)<static>true</static>', r"\1<static>false</static>", s)
        assert n == 1, m
    (W / "lab4_incident.sdf").write_text(s)
    print("wrote", W / "lab4_incident.sdf")


if __name__ == "__main__":
    main()
