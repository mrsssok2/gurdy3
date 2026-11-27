#!/usr/bin/env python3
"""Execute MoveIt plans for real (plan_only = False) through arm_controller: folded -> servo_view -> pre_grasp -> folded,
with the fitted floor in the planning scene (scripts/moveit_floor.py). Per move: MoveIt error code, planning time,
execution wall time, final joint error vs the target from /joint_states 1 s after the result, arm joint efforts.
Plus a floor sanity check: an arm pose whose offline model puts the gripper INTO the floor must be invalid with a contact
against `floor`.
    python3 scripts/moveit_exec_check.py OUT.json            (stack with arm:=true + move_group running)
"""
import json
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gurdy_core"))
sys.path.insert(0, str(ROOT / "scripts"))
from gurdy_core import arm as A  # noqa: E402
from moveit_floor import add_fitted_floor  # noqa: E402


def main():
    out = sys.argv[1]
    rclpy.init()
    n = rclpy.create_node("moveit_exec_check")
    js, eff = {}, {}

    def on_js(m):
        js.update(dict(zip(m.name, m.position)))
        if len(m.effort) == len(m.name):
            eff.update(dict(zip(m.name, m.effort)))
    n.create_subscription(JointState, "/joint_states", on_js, 10)
    ac = ActionClient(n, MoveGroup, "/move_action")
    if not ac.wait_for_server(timeout_sec=60.0):
        print("FAIL no /move_action")
        sys.exit(1)
    t1 = time.monotonic()                   # move_group's state monitor needs a few joint states after start-up (a move sent
    while time.monotonic() - t1 < 5.0:      # at once failed with "couldn't receive full current joint state", code -4)
        rclpy.spin_once(n, timeout_sec=0.1)
    floor = add_fitted_floor(n, margin=0.005)
    print(f"floor: fitted {floor['fitted']}, z {floor['z0']:.4f} m, tilt {floor['tilt_deg']:.2f} deg, added {floor['added']}", flush=True)
    srdf = ET.parse(ROOT / "ros2_ws/src/gurdy_arm_moveit/config/gurdy.srdf").getroot()
    poses = {g.get("name"): [float(g.find(f"joint[@name='{j}']").get("value")) for j in A.ARM_JOINTS]
             for g in srdf.findall("group_state[@group='arm']")}
    res = {"floor": floor, "moves": []}
    for name in ("servo_view", "pre_grasp", "folded"):
        g = MoveGroup.Goal()
        g.request.group_name = "arm"
        g.request.allowed_planning_time = 5.0
        g.request.num_planning_attempts = 3
        g.request.max_velocity_scaling_factor = 0.5
        g.request.max_acceleration_scaling_factor = 0.5
        c = Constraints()
        for j, v in zip(A.ARM_JOINTS, poses[name]):
            c.joint_constraints.append(JointConstraint(joint_name=j, position=v, tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        g.request.goal_constraints.append(c)
        g.planning_options.plan_only = False
        for attempt in (1, 2):              # one retry on CONTROL_FAILED (logged)
            t0 = time.monotonic()
            f = ac.send_goal_async(g)
            rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
            r = f.result().get_result_async()
            rclpy.spin_until_future_complete(n, r, timeout_sec=90.0)
            wall = time.monotonic() - t0
            rr = r.result().result
            if rr.error_code.val != -4:
                break
            print(f"{name}: CONTROL_FAILED on attempt {attempt}, retrying", flush=True)
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.0:
            rclpy.spin_once(n, timeout_sec=0.05)
        err = [js.get(j, float("nan")) - v for j, v in zip(A.ARM_JOINTS, poses[name])]
        m = {"target": name, "attempts": attempt, "error_code": rr.error_code.val, "planning_time_s": rr.planning_time, "wall_s": wall,
             "waypoints": len(rr.planned_trajectory.joint_trajectory.points), "final_err_rad": err,
             "max_abs_err_rad": float(np.nanmax(np.abs(err))), "effort_nm": [eff.get(j) for j in A.ARM_JOINTS]}
        res["moves"].append(m)
        print(f"{name}: code {m['error_code']}, plan {m['planning_time_s']:.2f} s, executed in {wall:.1f} s wall, "
              f"max joint error {m['max_abs_err_rad']:.4f} rad", flush=True)
    # floor sanity: find a pose (offline model) with the gripper 2 cm into the floor, ask MoveIt
    a, rb = A.load_arm(), A.load_robot()
    gz = A.ground_z(rb)
    rng = np.random.default_rng(1)
    lo = np.array([a.limits[j][0] for j in A.ARM_JOINTS])
    hi = np.array([a.limits[j][1] for j in A.ARM_JOINTS])
    bad = None
    for _ in range(20000):
        q = rng.uniform(lo, hi)
        c = A.clearances(a, q, rb, gz)
        if c["floor"] < -0.02 and min(c["legs"], c["body"], c["lidar"]) > 0.02:
            bad = q
            break
    sv = n.create_client(GetStateValidity, "/check_state_validity")
    sv.wait_for_service(timeout_sec=10.0)
    st = JointState(name=list(js), position=[float(v) for v in js.values()])
    for j, v in zip(A.ARM_JOINTS, bad):
        st.position[st.name.index(j)] = float(v)
    fv = sv.call_async(GetStateValidity.Request(robot_state=RobotState(joint_state=st), group_name="arm"))
    rclpy.spin_until_future_complete(n, fv, timeout_sec=5.0)
    v = fv.result()
    floor_hits = sorted({(c.contact_body_1, c.contact_body_2) for c in v.contacts if "floor" in (c.contact_body_1 + c.contact_body_2)})
    res["floor_sanity"] = {"q": bad.tolist(), "valid": v.valid, "floor_contacts": floor_hits}
    print(f"floor sanity: arm pose 2+ cm into the floor -> valid={v.valid}, floor contacts {floor_hits[:3]}", flush=True)
    ok = all(m["error_code"] == 1 for m in res["moves"]) and not v.valid and floor_hits and floor["added"]
    Path(out).write_text(json.dumps(res, indent=1, default=float))
    print(("PASS" if ok else "FAIL") + " moveit execute folded -> servo_view -> pre_grasp -> folded", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
