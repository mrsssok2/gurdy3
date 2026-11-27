#!/usr/bin/env python3
"""Add the floor to MoveIt's planning scene FROM THE FITTED FLOOR PLANE (body depth camera, RANSAC, gurdy_core.low_obstacles).

    python3 scripts/moveit_floor.py [--margin 0.005] [--out floor.json]      (stack + move_group running)
    from moveit_floor import add_fitted_floor                                (grasp test / viewing session)

1. One /camera/depth_image + camera_info -> base_link points (robot.yaml depth_camera pose, same chain as the URDF) ->
   `fit_floor` (prior z -0.11, band 6 cm, inlier 12 mm). Not fitted -> the prior plane, reported as such.
2. Collision object `floor`: a 4 x 4 x 0.02 m box in base_link, aligned with the fitted normal, its TOP FACE `margin` ABOVE the
   fitted plane (safety margin: the arm keeps that much extra distance from the real floor).
3. The legs stand on the floor: floor-vs-leg links (coxa / femur / tibia / foot + their proxies) are ALLOWED in the ACM (else
   the start state is always in collision). The ACM of a scene diff replaces the whole matrix, so the current one is read
   (/get_planning_scene) and extended. Arm-vs-floor stays checked.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AllowedCollisionEntry, CollisionObject, PlanningScene, PlanningSceneComponents,
                             PlanningSceneWorld)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))
from gurdy_core import low_obstacles as LO  # noqa: E402
from gurdy_core.robot import load_robot  # noqa: E402

LEG_WORDS = ("coxa", "femur", "tibia", "foot")


def _spin_until(node, cond, timeout):
    t_end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while not cond() and node.get_clock().now().nanoseconds < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return cond()


def fit_floor_once(node, timeout=15.0) -> dict:
    """-> {'n': [3], 'd': float, 'fitted': bool, 'z0': floor z under base_link, 'tilt_deg', 'inliers'}"""
    got = {}
    s1 = node.create_subscription(Image, "/camera/depth_image", lambda m: got.setdefault("depth", m), 2)
    s2 = node.create_subscription(CameraInfo, "/camera/camera_info", lambda m: got.setdefault("info", m), 2)
    ok = _spin_until(node, lambda: "depth" in got and "info" in got, timeout)
    node.destroy_subscription(s1)
    node.destroy_subscription(s2)
    prm = LO.LowObstacleParams()
    if not ok:
        return {"n": [0.0, 0.0, 1.0], "d": -prm.floor_prior_z, "fitted": False, "z0": prm.floor_prior_z, "tilt_deg": 0.0,
                "inliers": 0, "why": "no depth image"}
    m, k = got["depth"], got["info"].k
    depth = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
    pts, _ = LO.depth_to_points(depth, (k[0], k[4], k[2], k[5]), prm)
    pts = LO.apply_T(LO.T_base_optical(load_robot().sensors["depth_camera"]), pts)
    n, d, fitted, inl = LO.fit_floor(pts, prm)
    n = np.asarray(n, float)
    return {"n": n.tolist(), "d": float(d), "fitted": bool(fitted), "z0": float(-d / n[2]),
            "tilt_deg": math.degrees(math.acos(min(1.0, n[2]))), "inliers": int(np.sum(inl)), "points": int(len(pts))}


def _quat_z_to(n):
    """Quaternion rotating +z onto unit vector n."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, n)
    c = float(z @ n)
    if np.linalg.norm(v) < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    q = np.array([v[0], v[1], v[2], 1.0 + c])
    q = q / np.linalg.norm(q)
    return tuple(float(x) for x in q)


def add_fitted_floor(node, margin=0.005, size=4.0, thick=0.02, timeout=15.0) -> dict:
    fl = fit_floor_once(node, timeout)
    n = np.asarray(fl["n"])
    gp = node.create_client(GetPlanningScene, "/get_planning_scene")
    ap = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not (gp.wait_for_service(timeout_sec=30.0) and ap.wait_for_service(timeout_sec=30.0)):
        return {**fl, "added": False, "why": "move_group services not available"}
    f = gp.call_async(GetPlanningScene.Request(components=PlanningSceneComponents(
        components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)))
    rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
    acm = f.result().scene.allowed_collision_matrix
    names = list(acm.entry_names)
    rows = [list(e.enabled) for e in acm.entry_values]
    if "floor" not in names:
        allow = [any(w in nm for w in LEG_WORDS) for nm in names]
        for r, a in zip(rows, allow):
            r.append(a)
        names.append("floor")
        rows.append(allow + [False])
    acm.entry_names = names
    acm.entry_values = [AllowedCollisionEntry(enabled=r) for r in rows]
    # box centre: on the plane normal through the point under base_link, top face `margin` above the plane
    p0 = np.array([0.0, 0.0, fl["z0"]]) + n * (margin - thick / 2)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in p0)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = _quat_z_to(n)
    co = CollisionObject(id="floor", operation=CollisionObject.ADD)
    co.header.frame_id = "base_link"
    co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size, size, thick])]
    co.primitive_poses = [pose]
    scene = PlanningScene(is_diff=True, world=PlanningSceneWorld(collision_objects=[co]), allowed_collision_matrix=acm)
    f = ap.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
    ok = f.result() is not None and f.result().success
    return {**fl, "added": bool(ok), "margin": margin, "top_z_under_base": fl["z0"] + margin,
            "allowed_leg_links": sorted(nm for nm in names[:-1] if any(w in nm for w in LEG_WORDS))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--margin", type=float, default=0.005)
    ap.add_argument("--out")
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("moveit_floor")
    r = add_fitted_floor(node, a.margin)
    print(f"floor: fitted={r['fitted']} z under base_link {r['z0']:.4f} m, tilt {r['tilt_deg']:.2f} deg, "
          f"{r['inliers']} inliers; added={r['added']} (top face {r.get('top_z_under_base', float('nan')):.4f} m, margin {a.margin} m), "
          f"{len(r.get('allowed_leg_links', []))} leg links allowed", flush=True)
    if a.out:
        Path(a.out).write_text(json.dumps(r, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if r["added"] else 1)


if __name__ == "__main__":
    main()
