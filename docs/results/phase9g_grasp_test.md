# Phase 9g: first grasp test in Gazebo (written 2026-11-27, NOT run)

Branch `phase8c-wire`. Host-venv tests only; nothing has run in Gazebo, MoveIt or ROS.

## What it does

`scripts/grasp_test.sh [RUN] [-- runner args]` → `scripts/grasp_test.py` (rclpy) → `gurdy_core/gurdy_core/grasp_test.py`
(pure, tested). 5 small classes × 3 tries, each on the lab4_fetch instance of the class:

| class | object | grasp |
|---|---|---|
| small_box | red_cube | top |
| mini_toolbox | yellow_mini_toolbox | top, across the handle |
| can | blue_can | side |
| mini_cone | orange_cone | side |
| mini_extinguisher | red_mini_extinguisher | side |

**Stack** (headless): `world:=lab4_fetch arm:=true controller:=tripod`; `move_group` unless `MOVEIT=off`.

**Each try:**
1. **Reset.** Gripper open, arm `folded`, object `set_pose` back to its ground-truth pose. The robot is teleported so the
   object lies at the class grasp zone (`visual_servo.CLASS_INFO`, behind the robot) plus a known per-try offset
   (3/2, −2/−3, 4/0 cm). The yaw is the one of 24 with the most footprint clearance from walls and objects (all 8
   objects: 0.44–0.63 m).
2. **View and servo.** Arm to `servo_view` / `servo_view_far`, wrist camera triggered, `VisualServo`. Body steps are
   **teleports by default** (exact, isolates the grasp from walking); `--step walk` uses closed-loop `/cmd_vel` at
   0.05 m/s on GT odom.
3. **Plan.** `grasp_plan` at the servo's estimate: `arm.plan_grasp` (limits + collision model), pre-grasp −5 cm along the
   approach, 5 vertical IK waypoints for the lift. Top grasps pick the wrist roll numerically so the fingers close across
   the short side. The object yaw comes from the **wrist view** (`--yaw wrist`, default; section "Wrist-view yaw"). Below
   `--yaw-min-conf` (0.5) the roll falls back to 0. `--yaw gt` uses ground truth (comparison only); `--yaw none` = roll 0.
   Each try records the wrist estimate, its confidence and its error vs ground truth in every mode.
4. **Approach.** Pre-grasp with **MoveIt** if `/move_action` answers (a failure falls back to a direct goal; logged per
   try), else a direct `FollowJointTrajectory` goal. The final straight approach and the lift always use direct goals.
5. **Close.** Contact physics only: `gripper_controller` (JointGroupEffortController) −3 N per finger (`--grip-effort`;
   joint limit 10 N, needed ≈ 0.4 N for 80 g at μ 1). No DetachableJoint, no attach.
6. **Lift and hold.** Lift 5 cm, hold 5 s sim. The object z is sampled at ~20 Hz from `/world/lab4_fetch/dynamic_pose/info`.
7. **Classify, then clean up:** put it back down, open, fold.

**Outcomes** (`grasp_test.classify`):

| outcome | rule |
|---|---|
| missed | never rose ≥ 60 % of the lift (3 cm); also servo / plan failures |
| dropped | back within 1 cm of rest during / at the end of the hold |
| slipped | held but sank > 1 cm during the hold (or ends < 3 cm up) |
| success | none of the above |

`knocked` is a flag: the object moved > 3 cm in xy before the close. Time = sim seconds from the servo start to the end of
the hold.

**PASS per class:** ≥ 2/3 success and 0 drops (proposed). Lines `PASS|FAIL grasp_<class> ...` go into
`docs/results/data/grasp_test/summary.txt`. The run dir holds `tries.jsonl` (servo log, plan, fingers, z samples, MoveIt vs
direct per move), `summary.json`, the logs and `rtf.txt`.

## Wrist-view yaw (`gurdy_core/gurdy_core/wrist_yaw.py`, added 2026-11-27)

* **Points.** The wrist depth goes to base_link points, then the RANSAC floor plane (the same as the servo), then the
  floor frame. The object's points are 1 cm .. class height + 3 cm above the floor, within the footprint radius + 3 cm
  of the servo estimate.
* **small_box** (yaw mod 90°): minimum-area rectangle of the top-face points (top 1.2 cm), 0.5° grid. Confidence =
  distinctness of the minimum (area ratio 1.1 → 1.35; a full face gives ~1.4) × point count × 0.5 if the face touches
  the image border.
* **mini_toolbox** (mod 180°): principal axis of the handle bar (points above 8 cm). Its elongation (std ratio 2 → 3.5)
  gives the confidence. Without handle points, the body top (4.5–6.8 cm) is used instead (method `body`).
* **can, mini_cone, mini_extinguisher:** no yaw (symmetric, side grasp with roll 0).
* **Report.** `tries.jsonl` has `yaw` = {method, yaw, confidence, partial, gt_base, err_deg, used, used_src}. The console
  shows the per-try error, and the PASS/FAIL line has the class median / max yaw error and the low-confidence count.

Synthetic wrist frames (`tests/servo_sim.py`: view pose, flat floor, depth noise σ = k·z², object at the zone centre,
5° steps):

| class | noise k | frames | error median / max (deg) | min confidence |
|---|---|---|---|---|
| small_box | 0 | 19 | 1.0 / 2.5 | 1.00 |
| small_box | 0.02 | 19 | 1.5 / 2.5 | 1.00 |
| small_box | 0.05 | 19 | 1.0 / 3.0 | 1.00 |
| mini_toolbox | 0 | 36 | 0.0 / 0.1 | 1.00 |
| mini_toolbox | 0.02 | 36 | 0.0 / 0.1 | 1.00 |
| mini_toolbox | 0.05 | 36 | 0.0 / 0.2 | 1.00 |

These are upper bounds: ideal geometry and no edge artefacts. Tests (`gurdy_core/tests/test_wrist_yaw.py`, 88) also
cover:
* body tilt 0.03 rad + heavy noise: ≤ 4° / ≤ 3°
* a partial view: flagged, confidence halved
* a toolbox with no visible handle: body top ≤ 3°
* symmetric classes, an empty floor, no hint, a second cube 12 cm away

An end-to-end test (`test_grasp_test.py`) checks wrist frame → yaw → grasp plan → fingers across the toolbox handle at
0 / 30 / 75 / 140°.

## Run (after day_0927 01_build, 04_arm_spawn, 09_wrist_camera PASS)

```bash
docker ps                                                   # empty
scripts/grasp_test.sh first -- --classes small_box --tries 1   # smoke test first
scripts/grasp_test.sh v1                                    # 5 x 3
MOVEIT=off scripts/grasp_test.sh v1_direct                  # without move_group
```

## To verify on the first run (unverified assumptions)

1. **Finger effort sign.** + = open: both prismatic axes point outward (`axis 0 ±1 0`), limits 0–0.04 = opening per
   finger. Check `fingers_closed` in tries.jsonl: about 0 with nothing in between, about grip width / 2 on an object.
2. **Pose stream format.** The text of `gz topic -e -t /world/lab4_fetch/dynamic_pose/info`; zero fields are omitted,
   which the parser handles (unit test with a hand-written sample, not real output).
3. **Teleport.** `set_pose` on the standing robot (z 0.115, level) keeps it standing. Not tried yet; the object3d
   check has the same risk.
4. **Contact physics.** DART at 1 ms with 5 cm objects, finger friction 1.0, object μ 1.0: jitter or squeeze-out is
   possible. Tune `--grip-effort` only after looking at the first z traces.
5. **Wrist camera trigger.** Covered by day_0927 step 09; the runner waits up to 3 s for a new frame.
6. **Two control blocks.** `gz_ros2_control` with two blocks (day_0927 step 04): `arm_controller` and
   `gripper_controller` must both be active, otherwise `FAIL grasp_setup`.
7. **MoveIt.** Execution through the controllers (`plan_only` false) with the generated SRDF. The planning scene has
   no floor or objects. That is why the approach and the lift are direct goals.

## Honest limits

* The body steps are teleports, so locomotion error is excluded (`--step walk` tests it).
* The wrist-view yaw is tested on synthetic frames only (ideal geometry); the Gazebo error is in the report per try.
* One world, one instance per class, 3 tries: too few for rates; this is a first functional test.
* The time is sim time, which includes waiting phases (settle, the 5 s hold).
