# Phase 9a plan: arm + gripper for Gurdy (pick an object and bring it back)

Status 2026-11-26: **design plus offline checks only**, on branch `phase9a-arm`. Nothing has run in Gazebo or MoveIt.
The default robot model (`gurdy.urdf.xacro`, `robot.yaml`) and every launch file are unchanged. The arm exists only in
`gurdy_with_arm.urdf.xacro arm:=true` (default `false`, which expands to the same URDF as today).

Numbers below come from `scripts/arm_reach_report.py` (full output: `docs/results/data/phase9a/arm_reach.md`) and
`gurdy_core/tests/test_arm.py` (14 tests). Servo masses and torques are datasheet-class values for hobby servos, **not
measured**.

## 1. Constraints read from the code

| item | value | source |
|---|---|---|
| body | 0.30 × 0.18 × 0.06 m box, 1.40 kg; base_link 0.11 m above the floor standing | robot.yaml |
| legs | 6 × 0.28 kg, hip mounts at x = ±0.13 / 0, rear legs at 135°, nominal feet at reach 0.14 m | robot.yaml |
| total mass | 3.08 kg | gurdy_core.robot |
| Nav2 footprint | hexagon, x ±0.273, y ±0.285 | gurdy_navigation.footprint |
| front depth camera | x +0.15, z +0.02 (lens ≈ 0.13 m above floor) | robot.yaml |
| LiDAR | centre, z +0.05 (plane ≈ 0.16 m above floor), puck r 0.025 | robot.yaml / URDF |
| **mass the walking policy has seen** | payload 0–0.30 kg on the body, ≤ 2 cm xy offset, **at body height**; body mass ×0.85–1.15 (±0.21 kg); link masses ×0.9–1.1 | `rl/envs/randomization.py` (`terrain_dr2` uses these defaults) |
| Nav2 stop accuracy | xy goal tolerance 0.12 m, yaw 0.2 rad | nav2_params.yaml |
| objects in the labs | box 0.25–0.40 m, toolbox 0.45 × 0.2 × 0.24, extinguisher 0.19 × 0.16 × 0.49, cone 0.36 × 0.65, barrel 0.56 × 0.88, chair, mannequin | Phase 8c `semantic_lab_gt3d.json` |
| wrist camera idea | RGB-D 0.07–1.0 m, only on while picking | Phase 8c plan §2 |

**Consequence: every existing lab object is too big for this robot.** The narrowest side is 0.16–0.56 m and real
masses are 1–10 kg, against a 3 kg hexapod with a 0.08 kg payload budget (table in `arm_reach.md`). Picking needs **new
small objects**. The plan adds five pickable classes (§4). The detector does not know them yet: a YOLO retrain is in the
Extras list.

## 2. Arm design (`ros2_ws/src/gurdy_description/config/arm.yaml`)

**Type:** 5-DOF serial arm + parallel 2-finger gripper:
* yaw
* shoulder pitch, elbow pitch, wrist pitch
* wrist roll
Roll lets the jaws line up with an object without turning the whole robot. Four DOF would also work, using the body's
yaw for alignment, but roll costs only 9 g.

**Mount: body top, rear (x = −0.10 m), turned 180°.** The arm works **behind** the robot: Gurdy turns its back to the
object and backs up to it. It folds **forward** over the body for walking. Why the rear:
* The front depth camera and the forward LiDAR view stay free.
* The space directly behind the body, between the rear legs, is open.
* Only the 2.5 cm yaw-servo base crosses the LiDAR plane: it hides about **±11° straight behind**, inside the
  footprint, so `scan_self_filter` turns it into "no return". A front mount would put that blind sector straight ahead.

**Links** (arm frame; pitch angles from horizontal, + = up):

| link | length (m) | mass (kg) | contents |
|---|---|---|---|
| base (fixed to body) | 0.025 high | 0.014 | yaw servo MG90S-class |
| turret | shoulder axis 0.025 above the yaw joint (z = 0.08 in base_link) | 0.065 | shoulder servo MG996R-class + bracket |
| upper arm | 0.12 | 0.015 + 0.014 | tube + elbow servo MG92B-class at its end |
| forearm | 0.12 | 0.010 + 0.009 | tube + wrist-pitch servo SG90-class |
| wrist | 0.03 | 0.015 | wrist-roll servo + bracket |
| palm | 0.035 | 0.045 | gripper servo + rack + wrist camera |
| fingers | 0.04 (each) | 2 × 0.005 | friction pads μ 1.0; opening 0–0.08 m |
| **total** | grasp centre 0.085 from the wrist axis | **0.197** | |

**Joint limits** (each a 180° hobby servo; tests check the span ≤ π):

| joint | range (rad) | rated torque (N·m) |
|---|---|---|
| yaw | ±1.57 (0 = straight back) | 0.22 |
| shoulder | −0.55 .. 2.59 | 0.92 |
| elbow | −2.80 .. 0.34 | 0.34 |
| wrist pitch | −1.25 .. 1.89 (shifted so that side grasps fit) | 0.18 |
| wrist roll | ±1.57 | 0.18 |
| fingers | 0–0.04 m each | 10 N |

**Payload limit 0.08 kg.** At 0.10 kg the elbow would need 0.274 N·m in some side grasps, above 80 % of its rating. A
bigger elbow servo (+41 g) would push arm + object above the 0.30 kg the policy was trained with.

**Mass and CoM vs what the policy has seen:**

| configuration | added mass | CoM xy shift | CoM rise |
|---|---|---|---|
| arm folded | 0.197 kg | 0.2 mm | 7.6 mm |
| arm folded + 0.08 kg object | **0.277 kg** (≤ 0.30 seen) | **3.2 mm** (≤ 20 mm seen) | 10.4 mm (**height never randomised**) |
| arm at the most rearward grasp (standing still) | 0.277 kg | 22.7 mm | 1.1 mm |

* **Mass and xy offset:** inside the trained range.
* **CoM height:** +1 cm while carrying, outside it: the randomisation only ever placed the payload at body height.
  Static stability margin is 112 mm on the worst tripod, so tipping is not the risk. Tracking quality while walking is,
  and needs the Gazebo test (§7). If it fails, fine-tune `terrain_dr2` in MuJoCo with the arm mass model; a heavy job,
  for later.
* **Most rearward grasp:** 22.7 mm is beyond the trained 2 cm, but the robot does not walk then. It stands on 6 feet,
  margin 206 mm.

**Static torques** (worst feasible grasp with 0.08 kg):

| joint | worst torque (N·m) | of rating |
|---|---|---|
| shoulder | 0.48 | 52 % |
| elbow | 0.26 | 76 % |
| wrist pitch | 0.10 | 55 % |

These are gravity only: dynamic torques and the squeeze force are not included.

**Folded pose** `[0, 2.475, 0.225, 1.725, 0]`:
* chosen by `gurdy_core.arm.search_folded`: the lowest CoM among poses with CoM xy ≤ 1 cm, ≥ 0.1 rad from every limit,
  ≥ 1 cm clear of body, LiDAR puck and legs, and every arm part above the LiDAR plane
* the arm leans forward over the body, and the carried object sits above the front at z ≈ 0.12 m
* clearances: body 2.9 cm, LiDAR puck 4.1 cm, legs 5.8 cm; lowest arm part 4.1 cm above the LiDAR plane
* the object (≤ 7 cm, centre at x 0.12) sits about 0.1 m above the front camera lens, far outside its view, whose top
  edge is 13.7° above horizontal

**Ready pose** `[0, 1.2, −1.6, −1.0, 0]`:
* the wrist camera looks 80° down from 0.22 m above the floor
* it sees 0.30 × 0.23 m of floor centred 0.30 m behind the body centre: every grasp zone below

## 3. Reach analysis (Nav2 stops with ~12 cm error)

Object centre on a 1 cm grid behind the robot. Feasible = object clear of legs/body, IK within limits for one of the
class's approach angles, and grasp + pre-grasp (5 cm back along the approach) collision-free (capsule model, 5 mm margin).

| object | grasp | feasible area | x range at y = 0 | largest disc inside | coverage of a 2 / 5 / 12 cm disc |
|---|---|---|---|---|---|
| small_box 5 cm cube, 50 g | top | 204 cm² | −0.32 .. −0.21 | r 6.0 cm at (−0.27, −0.02) | 100 / 100 / 45 % |
| mini_toolbox 12 × 5 × 7.5 cm, 80 g | top (handle) | 67 cm² | −0.32 .. −0.26 | r 4.0 cm | 100 / 75 / 15 % |
| can r 3.3 cm, h 12 cm, 60 g | side | 257 cm² | −0.39 .. −0.31 | r 5.0 cm | 100 / 99 / 45 % |
| mini_cone r 4 cm, h 10 cm, 30 g | side | 253 cm² | −0.39 .. −0.31 | r 5.0 cm | 100 / 99 / 45 % |
| mini_extinguisher r 2.5 cm, h 15 cm, 80 g | side | 298 cm² | −0.40 .. −0.31 | r 5.1 cm | 100 / 100 / 50 % |

**So the arm cannot absorb Nav2's 12 cm error: only 15–50 % of that disc is reachable.** Every pick needs a **final
approach**:
1. Nav2 goal 0.5 m from the object, facing away.
2. Arm to `ready`, wrist camera on.
3. Phase 8c `object3d.estimate` with the wrist camera's K and TF.
4. Closed-loop `relmotion` (back up / strafe; the robot is holonomic) until the object centre is within 2 cm of the
   class's zone centre. A 2 cm disc is 100 % covered for every class.
Maps per class: `docs/results/data/phase9a/arm_reach.md`.

## 4. Grasp strategy per class

| class | strategy | approach | grip width |
|---|---|---|---|
| small_box | top-down, jaws on two opposite faces (roll = object yaw mod 90°) | −90° | 5 cm |
| mini_toolbox | top-down on the handle (roll = handle direction) | −90° | 2 cm |
| can, mini_extinguisher | side grasp at half height | 0° / −17° / −34° (first feasible) | 6.6 / 5 cm |
| mini_cone | side grasp at half height (cone is 4.5 cm wide there) | as above | 4.5 cm |
| box, toolbox, extinguisher, cone, barrel, chair, mannequin (lab classes) | **too big / heavy**; the agent says so honestly | | |

**Grasp sequence:**
1. Open to grip width + 1.5 cm.
2. Move to the pre-grasp 5 cm back along the approach.
3. Straight-line approach.
4. Close with effort control (finger force limit, not position).
5. Check the grip:
   * the fingers stopped before closing fully (an object is between them), and
   * the wrist camera still sees the object between the fingers.
6. Lift 3 cm, hold 1 s, check for slip, then fold.
7. Release at the destination in the reverse order.

## 5. Gazebo grasping

1. **Real contact physics first** (default).
   * Finger collision boxes with μ 1.0 (`<mu1>/<mu2>`, DART reads them like the feet).
   * Effort-controlled fingers (10 N limit).
   * Object models with real mass and inertia (30–80 g).
   * Known risks: small light objects jitter or are pushed away; penetration at 1 ms steps; DART friction on thin
     contacts. Mitigations to try in this order: larger finger pads, higher object friction, `<max_contacts>`, a finer
     physics step only during the grasp.
2. **Fallback, clearly labelled:** gz `DetachableJoint`.
   * `gurdy_with_arm.urdf.xacro arm_fallback_model:=<model>` adds the plugin (attach/detach topics
     `/arm/fallback/attach|detach`, one object model per robot spawn).
   * It is off by default; a test checks that it is absent without the argument.
   * Any result produced with it is reported as **"fallback attach, not a physical grasp"** in a separate column. It
     never counts as pick success.

## 6. MoveIt 2

* **SRDF:** `config/gurdy_arm.srdf` (DRAFT, tested against the expanded URDF).
  * Groups `arm` (chain `arm_mount_link` → `arm_grasp_link`) and `gripper` (2 finger joints).
  * Named states `folded` / `ready` / `open` / `closed`, end effector `gripper`.
  * Fixed virtual joint: the hexapod does not move while the arm plans.
  * Disable-collision list for adjacent pairs only. **Arm-vs-leg pairs stay enabled.** The full list must be generated
    with `moveit_setup_assistant` (sampling) on the expanded URDF.
* **IK plugin: `pick_ik`.** A 5-DOF arm cannot reach an arbitrary 6-D pose, and KDL fails or hangs on such requests.
  pick_ik supports position + partial orientation costs and is in the Jazzy apt repository (to confirm in the image).
  The analytic IK in `gurdy_core.arm.ik` (top-down / side, both elbow branches, over-the-top) seeds and checks it, and
  serves as a planner-free fallback for the fixed grasp motions.
* **Collision.**
  * The leg links come from the URDF and move with `/joint_states`, so a leg moved by the policy is seen. The robot must
    stand still, nominal stance, while the arm moves.
  * The body box and LiDAR puck are in the URDF.
  * The object comes from the wrist camera, as a box/cylinder from `object3d` plus a class size, and is added to the
    planning scene. An octomap from the wrist depth is optional, off at first (CPU).
* **Controllers.**
  * `joint_trajectory_controller` for the 5 arm joints, position interface in the second `<ros2_control>` block
    (`GurdyArmSystem`).
  * Effort command for the fingers.
  * TO VERIFY: gz_ros2_control loading two `<ros2_control>` hardware blocks; else merge them into the main block, which
    is a robot-model change for later.
* **Budget:** move_group typically uses 0.3–0.5 GB RAM (estimate). It runs only in the picking profile, never together
  with RL training.

## 7. Safety: arm folded while walking

* **Gate.**
  * A `arm_state` check: `/joint_states` within 0.05 rad of `folded`.
  * The agent's motion tools and Nav2 goals are refused unless folded (`not folded: I'm still holding the arm out`).
  * The safety shield / estop chain stays in front of the leg controller.
* **During picking.**
  * The robot stands (zero /cmd_vel). Only the final-approach `relmotion` moves it, and only with the arm in `ready`
    (camera up, fingers above 0.10 m).
  * E-STOP also stops the arm: the arm trajectory is cancelled and the joints hold position.
* **Carrying.** Folded pose and speed limit 0.08 m/s while carrying (to be tuned). Drop detection: the finger gap closes
  to 0, or the wrist camera loses the object.
* **Camera off while walking.** The wrist `rgbd_camera` is a gz **triggered** sensor (`/wrist_camera/trigger`), so it
  renders only while picking (RTF). TO VERIFY that triggered mode works for `rgbd_camera` in Harmonic.

## 8. Test plan and metrics (Gazebo, one heavy job at a time)

1. **Model checks** (ROS image): xacro expands; `check_urdf`; robot_state_publisher; arm moves to `folded` / `ready`;
   gz_ros2_control loads both systems; wrist camera publishes only when triggered.
2. **Standing with the arm:** 2 min nominal stance folded, then `ready`, then a grasp pose. Body tilt, joint torques,
   no leg contact.
3. **Walking with the arm folded, ± 0.08 kg object.** The same tracking protocol as Phase 2 (`tracking_experiments.sh`):
   tracking error vs no arm, falls, CoT. Acceptance: tracking within +5 points of the no-arm robot, 0 falls in 2 min.
4. **Pick success.** 5 classes × 10 trials, object placed 0.5 m ± 0.1 m behind a Nav2 goal with random yaw.
   * pick success (object lifted 3 cm and held 2 s)
   * time Nav2-arrive → folded with object
   * final-approach residual error
   * number of re-grasps
   * collisions arm–leg / arm–body (contact sensor), and how many tries used the fallback attach (separate column)
5. **Fetch** ("bring me the can"): go there, pick, return to the start. Success, total time, **drops while carrying**,
   **falls while carrying**, distance carried.
6. **Report:** `docs/results/phase9b_pick_eval.md`, real measured numbers only; RAM / RTF / VRAM per run.

## 9. Files on this branch

| file | what |
|---|---|
| `ros2_ws/src/gurdy_description/config/arm.yaml` | arm design (single source for the arm) |
| `ros2_ws/src/gurdy_description/urdf/gurdy_arm.xacro` | arm + gripper + wrist camera macro; Gazebo block, triggered RGB-D sensor, optional DetachableJoint fallback |
| `ros2_ws/src/gurdy_description/urdf/gurdy_with_arm.urdf.xacro` | wrapper: unchanged robot + arm only with `arm:=true` |
| `ros2_ws/src/gurdy_description/config/gurdy_arm.srdf` | MoveIt SRDF draft |
| `gurdy_core/gurdy_core/arm.py` | FK, analytic IK, capsule collision model, folded-pose search, mass / CoM / torque, grasp planning, object classes |
| `gurdy_core/tests/test_arm.py` | 14 tests (below) |
| `scripts/arm_reach_report.py` → `docs/results/data/phase9a/arm_reach.md` | report |

The tests cover:
* limits sane; named poses within limits; IK ↔ FK round trip (both elbow branches + over-the-top)
* folded pose safe; mass and CoM inside the trained range; torques ≤ 80 % of rating
* every pickable class has a grasp; every lab class is too big
* default URDF unchanged without the arm; URDF ↔ Python FK parity at 50 random poses; URDF masses / limits / inertia
  validity; Gazebo variant contents; SRDF vs URDF

## 10. Honest limits

* **Design only.** Nothing has run in Gazebo, MoveIt or the ROS image. Servo masses and torques are datasheet-class
  values. The collision model is capsules, not the URDF boxes. Link inertias use a box about the combined CoM
  (approximation).
* **The CoM rise (+1 cm carrying) is outside the trained randomisation.** Walking with the arm is the main risk, and
  only Gazebo can tell.
* **Narrow reach window** (6–10 cm deep behind the robot), so picking depends on the final approach working: Phase 8c
  3D estimate from the wrist camera plus `relmotion` precision (~2 cm needed; not measured yet).
* **New small object classes are needed.** The detector does not know them (retrain), and the semantic map has none yet.
* **Unverified in Gazebo:** two `<ros2_control>` blocks; triggered `rgbd_camera`; depth `<noise>`; DART grasp stability
  for 30–80 g objects; `pick_ik` availability.
* **LiDAR:** the arm base hides ±11° straight behind. Reversing during the final approach relies on the wrist camera,
  and the safety shield sees no obstacle in that sector.
