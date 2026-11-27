# Phase 9i: effort-controlled arm, MoveIt with the fitted floor, first grasps in Gazebo (2026-11-27)

All runs: lab4_fetch, `arm:=true`, headless, one container at a time. Raw data: `docs/results/data/day_0927/{04_arm_spawn,06_walk_arm_folded,moveit_exec}/`,
`docs/results/data/grasp_test/{first_gt,servo_box1,all5x3}/`. Single runs (no repeats unless stated).

## 1. Arm on effort control (PD like the legs)
* `arm.yaml control: effort` (default now; `position` kept for comparison): `joint_trajectory_controller` on the **effort** interface with PID per joint
  (`pd_gains`), `gurdy_arm.xacro` takes the command interface from arm.yaml, the gazebo launch passes the gains and sends `folded` once the arm
  controllers are active (with effort control the arm hangs on gravity until then).
* Gains = about half the explicit-PD stability bounds at the 500 Hz controller rate (ω·dt ≤ 0.53, kd·dt/I ≤ 0.35) for the smallest effective
  inertia per joint over 3000 random poses: yaw kp 1.2, shoulder 12 / kd 0.03, elbow 20 / 0.08, wrist pitch 6 / 0.015, roll 1.0; ki = kp (1 s),
  i_clamp = effort limit; the URDF joint damping 0.05 (implicit) damps the light joints.
* **Evidence it is real torque control:** holding `folded`, the measured shoulder / elbow / wrist-pitch efforts are 0.199 / 0.098 / 0.009 N m, the
  offline gravity model gives 0.201 / 0.098 / 0.009. Step 04: folded within 1e-6 rad after the integral settles.
* **Walking with the arm folded (1 walk, 30 s + 30 s):** no fall, forward error 10 %, turn 4 %, arm max deviation from folded **0.043 rad**.
* Gripper: unchanged force mode (`JointGroupEffortController`, a constant closing force: −3 N per finger commanded, 2.7 N measured on the joints
  while holding). A position PD would squeeze with kp × error up to the limit instead of a set force, so the force mode was kept on purpose.
  The finger joints now also publish an `effort` state.

## 2. Floor in MoveIt from the fitted floor plane (`scripts/moveit_floor.py`)
Body depth camera → base_link points → RANSAC `fit_floor`: **z −0.1083 m under base_link, tilt 0.11°** (wrist fit in day_0927 step 09: −0.107).
Box aligned with the fitted normal, **top face 5 mm above the plane** (safety margin). Floor-vs-leg contacts are allowed in the ACM (the feet stand
on it; otherwise every start state is in collision; the ACM of a scene diff replaces the whole matrix, so the current one is read and extended).
Check: an arm pose the offline model puts 2+ cm into the floor is **invalid** in MoveIt with finger / palm vs `floor` contacts. Used by the grasp
test (`--floor fitted`, default) and the RViz session.

## 3. MoveIt plans executed for real (`scripts/moveit_exec_check.py`)
| move | MoveIt | executed (wall) | final joint error 1 s later |
|---|---|---|---|
| folded → servo_view | SUCCESS | 6.1 s | 0.0097 rad |
| servo_view → pre_grasp | SUCCESS | 4.6 s | 0.0083 rad |
| pre_grasp → folded | SUCCESS | 5.3 s | 0.0098 rad |

First attempt failed with CONTROL_FAILED: the move was sent right after move_group started ("couldn't receive full current joint state").
The check now waits 5 s and retries once; the run above needed no retry.

## 4. First grasp: f_red_cube at its KNOWN ground-truth position (`grasp_test.py --locate gt --yaw gt`)
**Success.** Lifted 4.8 cm, sank 0.2 mm in the 5 s hold, not knocked, fingers 25 / 25 mm (cube 50 mm), 13.3 s sim.
**The weight is in the gripper:** shoulder / elbow efforts while holding 0.208 / 0.0705 N m vs the offline model at the hold pose with the 50 g cube
0.2085 / 0.0709 (empty arm 0.139 / 0.044).
Then the user-requested `--classes small_box --tries 1` (visual servo + wrist yaw): success, servo position error 0.9 mm, wrist yaw error 1.3°,
all 3 big moves through MoveIt, finger force 2.7 N each.

## 5. All 5 classes × 3 tries, `--yaw wrist` (servo, MoveIt, fitted floor; RTF ~0.7)
| class | grasp | result (scorer) | lift (m) | notes |
|---|---|---|---|---|
| small_box | top | **3/3** PASS | 0.048 | wrist yaw err 0.3-2.8° |
| mini_toolbox | top across the handle | **3/3** PASS | 0.050 | yaw err 0.07-0.15° |
| can | side | 2/3 PASS (1 missed) | 0.017 / 0.033 / 0.066 | try 1: held at first (one finger 17 mm), slipped out during the lift; **try 2 counted success but both fingers fully CLOSED (0 mm) and knocked**: the can rose without being between the fingers (wedged on a finger / the palm) -> doubtful |
| mini_cone | side | 2/3 PASS (1 missed) | 0 / 0.052 / 0.052 | try 1: fingers closed on air; one MoveIt move fell back to a direct goal |
| mini_extinguisher | side | **3/3** PASS | 0.084 / 0.053 / 0.052 | one finger at the open limit (40 mm) in all three: object off-centre, tilted in the grip (lift 8.4 cm) |

**Scorer: 13/15, all 5 classes PASS (≥ 2/3, 0 drops). Counting the doubtful can try as a miss: 12/15.**
Top grasps are clean (6/6, symmetric fingers, servo error ~1 mm). Side grasps are the weak part: servo position error 3-5 mm and the fingers close
asymmetrically (one at its open limit), i.e. the object is ~1-1.5 cm off the gripper centre along the finger axis.

## 6. Bugs found and fixed
* `grasp_test.py` `Rig.move`: `(log or []).append(...)` threw away the log when it was an empty list (MoveIt vs direct was never recorded).
* `grasp_test.sh`: ran on `gurdy2/ros:jazzy` (no MoveIt, so every move silently went direct); now `sim_moveit` unless `MOVEIT=off`.
* Missing `effort` state on the finger joints (finger force unreadable).

## 7. Next
1. Side grasps: centre the object between the fingers. Use the finger opening at contact to measure the offset, and/or close one finger first,
   then re-check the servo's side-class point (3-5 mm error vs 1 mm for top).
2. Scorer: require the fingers to be apart (≥ ~50 % of the object width) for a success, so a wedged object is not counted (can try 2).
3. More tries per class (3 is too few to tell 2/3 from 3/3), then the walking `--step walk` variant.

## 8. Grasp fixes (2026-11-27 afternoon) and 5 classes x 5 tries
All general, no per-class tuning:
1. **Centring** (`--centre on`, default): settle at the pre-grasp (integral term), measure the finger-midpoint error by FK of the MEASURED
   joints vs the command (`grasp_test.centre_correction`), shift the grasp target by it (clip 2 cm) and re-plan, so the midpoint lands
   on the wrist-camera 3D estimate. The wrist camera cannot see the object at the pre-grasp (closer than its 7 cm near clip), hence FK.
   Logged per try: ground-truth object position in the gripper frame (`obj_in_gripper_gt`: approach / finger axis / normal).
2. **Scorer:** a lifted object counts only if the fingers are apart (total opening ≥ 50 % of the class grip width,
   `grasp_test.fingers_apart`). Re-scored morning run: 13/15 -> 12/15 (can try 2: fingers 0 mm).
3. **Target in MoveIt's scene** (`--target-obstacle on`, default): the estimated object (class size + 1 cm) is a collision object for the
   big MoveIt moves and is removed before the final straight approach. Found by run `fixed5x5`: can tries 2 and 5 (same placement offset)
   had the can 14-17 cm away along the approach at the close (knocked by the arm swing; MoveIt did not know the target), then lifted
   wedged with the fingers closed -> counted as missed by rule 2.

| run | small_box | mini_toolbox | can | mini_cone | mini_extinguisher | total |
|---|---|---|---|---|---|---|
| morning `all5x3` (no fixes, old scorer) | 3/3 | 3/3 | 2/3 | 2/3 | 3/3 | 13/15 (12/15 strict) |
| `fixed5x5` (1 + 2) | 5/5 | 5/5 | 3/5 (2 knocked) | 5/5 | 5/5 | 23/25 |
| `can5_target` (1 + 2 + 3, can only) | | | 5/5, 0 knocked | | | 5/5 |
| **`final5x5` (1 + 2 + 3)** | **5/5** | **5/5** | **5/5** | **5/5** | **5/5** | **25/25**, 0 knocked / slipped / dropped |

Ground-truth centring after fix 1 (fixed5x5, successful side grasps): object within 0-6 mm of the finger midpoint along the closing axis
(before: 1-1.5 cm). **Remaining model artefact:** the two finger joints are independent with equal force, so a held object can slide
with BOTH fingers until one reaches its open limit (seen even with the object centred to 3 mm). The real gripper has one servo + rack
(coupled fingers); a mimic / coupling would model it. Not a failure cause in these runs.
Wrist yaw error (final5x5): small_box median 1.7° (max 2.8°), mini_toolbox 0.09° (max 0.2°). Mean try time 19-24 s sim. Single runs.
