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
   the short side. The object yaw comes from **ground truth** (`--yaw gt`, an oracle input: the wrist-view yaw is not
   wired yet); `--yaw none` = roll 0.
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
* The object yaw for the top-grasp roll is an oracle (ground truth).
* One world, one instance per class, 3 tries: too few for rates; this is a first functional test.
* The time is sim time, which includes waiting phases (settle, the 5 s hold).
