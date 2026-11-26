# Phase 9e prep: MoveIt 2 config for the arm + visual servoing for the final approach

Status 2026-11-26: offline code, configs and tests on branch `phase9a-arm`, run in the host venv only (no Docker tonight).
**Nothing has run in ROS, MoveIt or Gazebo.** Items marked **[ROS image]** can only be verified there.

## A. MoveIt 2 configuration (`ros2_ws/src/gurdy_arm_moveit/`)

| file | content |
|---|---|
| `config/gurdy.srdf` | **GENERATED** by `scripts/gen_arm_srdf.py`. Groups `arm` (chain `arm_mount_link` → `arm_grasp_link`, 5 joints) and `gripper` (2 finger joints). Named states `folded`, `ready`, `servo_view`, `servo_view_far` (= `arm.yaml`), `pre_grasp` (placeholder: the real pre-grasp is computed per object by `gurdy_core.arm.plan_grasp`), `open` / `closed`. End effector `gripper`, fixed virtual joint odom → base_link (the hexapod stands still while the arm moves; leg joints are passive, from `/joint_states`). |
| `config/kinematics.yaml` | `pick_ik/PickIkPlugin`, `mode: global`, position_scale 1.0, rotation_scale 0.3 (a 5-DOF arm cannot reach every 6-D pose), timeout 0.05 s, 3 attempts. **Seeded / fallback by the analytic IK:** the fetch backend computes `gurdy_core.arm.ik` / `plan_grasp` (top-down or side approach, both elbow branches, limits + collisions checked) and plans a joint-space goal to it. pick_ik is only used for small Cartesian corrections. **[ROS image: is `ros-jazzy-pick-ik` installable?]** |
| `config/joint_limits.yaml` | GENERATED from `arm.yaml` (positions and velocities as in the URDF; acceleration 2 rad/s² arm, 0.1 m/s² fingers, a conservative choice; scaling 0.3) |
| `config/ompl_planning.yaml` | RRTConnect, adapters (start-state bounds / collision check, time parameterisation) |
| `config/moveit_controllers.yaml` | `moveit_simple_controller_manager`, `arm_controller` FollowJointTrajectory. **The gripper is not a MoveIt controller:** the fetch backend closes it with effort control until the fingers stall (object between them). |
| `config/ros2_controllers.yaml` | `arm_controller` (JointTrajectoryController, position command interface of the `GurdyArmSystem` block) and `gripper_controller` (JointGroupEffortController). To be merged into the sim's controller-manager file when `arm:=true`. **[ROS image: gz_ros2_control with two `<ros2_control>` blocks]** |
| `launch/move_group.launch.py` | `arm:=false` (default) starts nothing. `arm:=true` starts `move_group` with its own robot_description (see below), SRDF, kinematics, limits, OMPL and controllers. **[ROS image: launch + MoveIt start]** |

**Disabled collision pairs computed offline** (`scripts/gen_arm_srdf.py`, report `docs/results/data/phase9e/srdf_collision_report.md`):
* Method: the expanded URDF, every collision geometry as an oriented bounding box (cylinders and spheres by their
  bounding box: conservative), 3000 random configurations (all arm **and all 18 leg** joints uniform in their limits),
  separating-axis test per link pair.
* Disabled: 45 Adjacent, 0 Always, 259 Never.
* **Every arm-vs-leg and arm-vs-body pair stays enabled** (192 pairs; 54 of them were in contact in some random sample,
  so the checks matter). The only exception is the adjacent mount-on-body pair. Tested.

**Found while doing this: the leg links have no collision geometry** (only visuals, plus the foot sphere on the tibia).
MoveIt could therefore not have seen a femur. Fix without touching Gazebo:
* `gurdy_arm.xacro` gets `leg_collision_proxies`: collision-only links on every coxa, femur and tibia, sized from
  robot.yaml plus 5 mm, with no mass.
* `gurdy_with_arm.urdf.xacro moveit_collision:=true` adds them **only for MoveIt's robot_description**. Gazebo and
  robot_state_publisher keep the description without them (same joints, so `/joint_states` fits both); Gazebo's contact
  physics is unchanged.

Tests: `ros2_ws/src/gurdy_arm_moveit/test/test_moveit_config.py`, **8 pass**:
* generated files up to date; SRDF links / joints exist; named states = `arm.yaml`
* arm-vs-robot never disabled; 18 proxies with no mass
* **folded / ready / servo_view / servo_view_far collision-free** at the nominal stance with every enabled pair
* joint limits, controller joint lists and solver consistent; launch parses
The wrist-camera and palm frames of `gurdy_core.arm` match the URDF chain at 30 random poses (`test_arm.py`).
`gurdy_description/config/gurdy_arm.srdf` (the hand-written 9a draft) is replaced by the generated SRDF.

## B. Visual servoing for the final approach (`gurdy_core/gurdy_core/visual_servo.py`)

**Observation** (wrist RGB-D):
1. Depth → points in base_link: wrist camera pose from the arm model (`arm.T_base_wrist_optical`).
2. **RANSAC floor plane** (9c `low_obstacles.fit_floor`). A floor frame is built from the plane: **no fixed floor
   height.** The floor was measured tonight at map z = −0.11 m, but the body tilts and the stance height varies; tested
   with 2 cm higher stance and ±3° tilt.
3. Object box: a detector box, or (until the small classes are trained, Phase 9b) the connected depth blob above the
   floor nearest the image centre.
4. Phase 8c `object3d.estimate` (copied unchanged from `phase8c-3d`, identical files) with the class shape prior.
5. **Looking down** (camera elevation > 50°): the 8c side-view fits do not apply, because the whole top face is seen.
   The centre is the xy centroid of the object's top band (its own points within 1.2 cm of its top).
6. `partial` = the top face is cut by the image border. Partial views are never accepted for alignment.
7. `implausible` = height ≠ class height ± 3 cm, or a cut view already taller than the class. That is probably an
   occluder (tested with a 25 cm post).

**Servo loop:**
* error = object xy − the class's grasp-zone centre. Zone centres come from the Phase 9a reach maps; all 9 points of a
  ±2 cm grid around each are graspable (tested).
* **Done** when |error| ≤ 2 cm on a complete view (`ALIGNED`), or ≤ 4 cm when the arm's grasp planner reaches the
  estimate and its ±1 cm neighbours (`ALIGNED_ARM`, the arm adjustment instead of another step).
* Else one holonomic body step of at most **6 cm** toward it, **max 3 steps**, timeout 60 s.
* **Target lost:** step toward the prior (the Nav2 / map position), re-observe; fail `TARGET_LOST`.
* **Occluded:** one 3 cm sideways step, then `OCCLUDED`.
* **Too close** (object under the rear legs): the step direction already points away (tested: first step forward).
* Still off after 3 steps: `NOT_ALIGNED`.

**View poses** (new in `arm.yaml`, found by search, ≥ 0.1 rad from every limit, ≥ 1.5 cm clear of body, LiDAR and
legs, collision-checked in the SRDF test):
* The 9a `ready` view (camera 0.22 m above the floor) was **too close**: a 12 cm can filled most of the image.
* `servo_view` [0, 1.95, −1.95, −1.15, 0]: camera 0.27 m up, 66° down, view centre 0.335 m behind base_link. For the
  top-grasp classes.
* `servo_view_far` [0, 1.80, −1.625, −1.15, 0]: 0.31 m, 56°, 0.44 m. For the side-grasp classes (zone 0.35 m behind,
  up to 15 cm tall), whose top face is cut from `servo_view`.
* The wrist-pitch lower limit caps the camera height at ~0.31 m.

**Results** (synthetic wrist depth from the Phase 8c renderer, kinematic robot `gurdy_core/tests/servo_sim.py`: holonomic
steps with gain 0.85–1.15 and 8 mm noise, depth noise σ = 0.003 z², random object yaw, start error uniform in the Nav2
12 cm disc; `scripts/servo_mc.py`, 60 trials per class):

| class | success | ALIGNED / ALIGNED_ARM | true final error median / max | graspable at the end (ground truth) | steps | time |
|---|---|---|---|---|---|---|
| small_box | 100 % | 41 / 19 | 1.5 / 4.1 cm | 60/60 | 1.2 | 5 s |
| mini_toolbox | 100 % | 41 / 19 | 1.6 / 3.8 cm | 60/60 | 1.4 | 6 s |
| can | 100 % | 37 / 23 | 1.6 / 4.0 cm | 60/60 | 1.2 | 5 s |
| mini_cone | 100 % | 43 / 17 | 1.2 / 4.2 cm | 60/60 | 1.2 | 5 s |
| mini_extinguisher | 100 % | 38 / 22 | 1.7 / 4.2 cm | 60/60 | 1.2 | 5 s |

* The max errors above 2 cm are `ALIGNED_ARM` ends, where the arm reaches the object from there (checked at the ground
  truth position).
* Single-observation accuracy (complete views, no noise): ≤ 0.7 cm for all classes. Partial views up to 4.7 cm, which
  is why they are not accepted.

**How the numbers got there (honest):**
* First run: mini_extinguisher 30 %, toolbox 97 %, with 3 ends not graspable. Two fixes:
  1. top-face partiality instead of box partiality, and the second view pose
  2. an arm-reach margin of ±1 cm on the estimate
* Two estimator problems were found and fixed on the way:
  1. the 8c rectangle fit is wrong for top-down views
  2. the 8c "top continues above the box" test misfires when looking down, because that is the floor
* The test robot first lacked the toolbox handle (my sim's bug, fixed).

Tests: `gurdy_core/tests/test_visual_servo.py`, **38 pass**:
* zone targets graspable; view poses safe and looking at the zones
* observe accuracy (5 classes × 3 offsets × 2 yaws); fitted floor vs a different stance height and tilt; depth noise;
  the depth blob box without a detector
* servo from 12 cm in 8 directions (box, can; ground-truth check); success never claimed with a wrong position under
  bad steps
* target lost (with and without prior), too close, occluder, not aligned (slipping steps), timeout, step limit

## C. Floor height

Floor at **map z = −0.11 m** (measured in Gazebo tonight, base_link level = map 0). Every new piece uses the **fitted
plane**: the servo's floor frame, and the 9c low-obstacle node. A fixed −0.11 is only the RANSAC prior. The 9c Nav2
overlay comment now records the measurement (its −0.3..0.6 window contains it).

## D. Needs the ROS image / Gazebo (one heavy job at a time)

1. **[ROS image]** `colcon build` of `gurdy_arm_moveit`; `pick_ik` installed; `move_group.launch.py arm:=true` starts.
   The planning scene shows the leg proxies; plan folded → servo_view → a grasp pose without leg contact.
2. **[ROS image]** gz_ros2_control with the second hardware block, `arm_controller` executing a MoveIt trajectory, the
   fingers' effort control.
3. **[Gazebo]** wrist camera: the triggered `rgbd_camera`, its real K vs `arm.yaml`, depth noise; the floor fit on real
   wrist frames.
4. **[Gazebo]** servo with the policy walking small steps. The real step accuracy is the open question: the policy has a
   start dead zone (~3 cm/s) and the Nav2 min-speed lift, so 1–6 cm steps need closed-loop `relmotion` with leg
   odometry. Measure success, final true error (GT), steps, time, falls.

## E. Limits

* **Kinematic robot model only:** perfect holonomic steps with gain and noise, no body sway while standing, no leg
  occlusion of the wrist view.
* **Synthetic depth:** the cone is rendered as a cylinder (the renderer has no cones). Primitive shapes only.
* **OBB collision model** for the SRDF statistics (conservative). MoveIt itself uses the exact URDF shapes.
* **One object per scene** in the servo tests. A second object next to the target can win the "nearest blob to the image
  centre" rule when no detector box is given.
* The arm stays in its view pose during the servo. Arm-only corrections (without body steps) are used only as the
  `ALIGNED_ARM` acceptance, not as active arm motions.

## F. Files

| file | what |
|---|---|
| `ros2_ws/src/gurdy_arm_moveit/` | package.xml, setup.py, config/*, launch/move_group.launch.py, test/test_moveit_config.py |
| `scripts/gen_arm_srdf.py` | SRDF + joint_limits generator → `docs/results/data/phase9e/srdf_collision_report.md` |
| `ros2_ws/src/gurdy_description/urdf/gurdy_arm.xacro`, `gurdy_with_arm.urdf.xacro` | `leg_collision_proxies`, `moveit_collision:=false` default |
| `ros2_ws/src/gurdy_description/config/arm.yaml` | poses `servo_view`, `servo_view_far` |
| `gurdy_core/gurdy_core/arm.py` | `T_base_palm`, `T_base_wrist_optical` (+ URDF parity test) |
| `gurdy_core/gurdy_core/visual_servo.py` | servo core |
| `gurdy_core/tests/test_visual_servo.py`, `servo_sim.py` | tests, kinematic robot |
| `scripts/servo_mc.py` | Monte-Carlo table |
| `gurdy_core/gurdy_core/vision/{object3d,transforms,synth_depth}.py`, `tests/test_object3d.py` | identical copies from `phase8c-3d` |
