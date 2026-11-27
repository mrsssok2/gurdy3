#!/usr/bin/env python3
"""For the RViz / MoveIt viewing session: (1) add the floor to MoveIt's planning scene, (2) trigger the wrist camera at
TRIGGER_HZ so the RViz image panel updates (the camera renders only when triggered).

    python3 scripts/moveit_view_extras.py [trigger_hz]      (default 2 Hz)
Floor: scripts/moveit_floor.py (fitted body-camera floor plane, top face 5 mm above it, floor-vs-leg contacts allowed).
Runs until killed.
"""
import sys
import time
from pathlib import Path

import rclpy
from std_msgs.msg import Bool


def main():
    hz = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    rclpy.init()
    n = rclpy.create_node("moveit_view_extras")
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from moveit_floor import add_fitted_floor
    r = add_fitted_floor(n, margin=0.005)
    print(f"floor added to the planning scene: {r['added']} (fitted {r['fitted']}, z {r['z0']:.4f} m in base_link, "
          f"top face 5 mm above it; floor-vs-leg contacts allowed)", flush=True)
    pub = n.create_publisher(Bool, "/wrist_camera/trigger", 2)
    try:
        while rclpy.ok():
            pub.publish(Bool(data=True))
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
