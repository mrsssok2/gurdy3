# Day 2026-11-27 (0927): merge + test plan

Branch **`integration-0927`** (local, from `main` at `30e1f91`), prepared 2026-11-26 night. Host venv only, no Docker.
**Not pushed, not merged into main.** Main is untouched.

## 1. Merge summary

| step | result |
|---|---|
| `git switch -c integration-0927 main` | from `30e1f91` (main: Phase 8a, 8b merged + live check, 8c convention checks) |
| merge `phase8c-3d` | **1 conflict, CLAUDE.md only**: both sides added status lines → kept all of them (main's 8a / 8b, then 8c). No code conflicts. |
| merge `phase9a-arm` (9a–9f) | **1 conflict, CLAUDE.md only** → all lines kept (8a / 8b / 8c, then 9a–9f). No code conflicts: the 8c files present on both branches (`vision/object3d.py`, `transforms.py`, `synth_depth.py`, `tests/test_object3d.py`) are byte-identical to `phase8c-3d` (checked). |
| apply 9c shield depth patch | **applied cleanly** (`use_depth_obstacles` default **false**) |
| apply 9f scan-timeout patch | **Regenerated 2026-11-27** on top of 9c (node only, default 0.5 s; branch `phase8c-wire`), NOT applied to the node yet: apply after step 10 picks the timeout. Old note: **PENDING.** It applies cleanly to main's node alone, but not on top of the 9c patch (same lines; it would need fuzz). Its new file `scan_timing.py` is already here through the 9a-arm merge. To do: regenerate the node part on top of 9c after step 10 picks the timeout value. |
| new on this branch | `sim.launch.py` (bringup + gazebo) flags **`arm:=false`** and **`arm_yaml:=""`**: with the arm, `gurdy_with_arm.urdf.xacro`, the arm controllers merged into the controller file, an arm + gripper spawner after the leg PD, the wrist-camera bridge (+ trigger). Needed for tomorrow's arm checks. With the default the xacro file, mappings and controllers are exactly as before. |

**Default behaviour = main.** Every new flag defaults OFF:

| flag | default |
|---|---|
| `sim.launch.py` `arm`, `arm_yaml` | false / "" |
| `shield` | false (main) |
| `use_depth_obstacles` | false |
| `gurdy_with_arm.urdf.xacro` `arm` | false (expands to the default URDF; tested) |
| `moveit_collision` | false |
| `low_obstacle_node` `wrist` | false (the node itself is in no launch file) |
| Nav2 low-obstacle overlay | not applied by default (tested) |
| `move_group.launch.py` `arm` | false |
| `Agent._fetch_route` | only with a backend `fetch_session` (none exists) |
| `FetchSession` picker | None |
| dataset profiles v3, `eval_detector --classes` | v3 / default v1 |

**Files that existed on main and changed** (`git diff --stat main --diff-filter=M`; every change guarded as above):
* `CLAUDE.md`
* `brain/agent/graph.py`
* `gurdy_core/gurdy_core/vision/classes.py` (constants only)
* `gurdy_navigation` package.xml / setup.py
* `safety_shield_node.py` (9c patch)
* `vision/check_labels.py`, `dataset_report.py`, `eval_detector.py`, `generate_dataset.py`
* the two `sim.launch.py` files (committed with this plan)

## 2. Host-venv test results on integration-0927 (2026-11-26, Python 3.14, numpy 2.5, langgraph from pip)

| suite | integration-0927 | the same suite on an untouched copy of main (`git archive main`) | verdict |
|---|---|---|---|
| `gurdy_core/tests` | **241 passed**, 2 skipped | – | OK |
| `brain/tests` | **1029 passed**, 3 skipped | – | OK |
| `vision/tests` | **46 passed** | – | OK |
| `gurdy_arm_moveit/test` | **8 passed** | – (new) | OK |
| `web/tests` (unittest) | **OK** | – | OK |
| fetch DEV suite (`brain.eval.fetch_dev`) | **42/42** | – (new) | OK (not held-out) |
| `scripts/shield_sim_report.py` | runs (0 collisions as before) | – | OK |
| `gurdy_navigation/test` | 12 passed, **5 failed**, 7 skipped | 9 passed, **5 failed**, 6 skipped | **environment**: numpy 2 removed 2-D `np.cross` (`footprint.py`); the image has numpy 1.x. Same 5 on main. |
| `gurdy_description/test` | 7 passed, **1 failed** | 7 passed, **1 failed** | **environment**: no `check_urdf` binary on the host |
| `rl/tests` | **7 errors** | **7 errors** | **environment**: no `mujoco` on the host |
| `gurdy_safety/test` | 1 skipped | – | rclpy only in the image |

**No failure is caused by the merge.** Every failing suite fails identically on main in this environment. Step 02
tomorrow runs them in the image.

## 3. Tomorrow's checks: `scripts/day_0927_checks.sh`

**How to run:** `scripts/day_0927_checks.sh list | all | <step> ... | stop`. Every step:
* headless, in ONE container `gurdy3_day0927`, refuses to start if `docker ps` is not empty
* a hard timeout, logs in `docs/results/data/day_0927/<step>/`
* exactly one `PASS` / `FAIL` / `SKIP` line in `docs/results/data/day_0927/summary.txt`
* Ctrl+C or `stop` removes the container
* A FAIL of a **gate** step stops `all`.
* Probes: `scripts/day0927_probe.py` (rclpy; syntax-checked only).
* The script was dry-checked with a fake `docker` (it builds the right container script, `arm:=true` reaches the launch,
  FAIL is reported); it never started a container.

| # | step | what | expected outcome / pass rule |
|---|---|---|---|
| 01 | `01_build` [gate] | colcon build; xacro + `check_urdf` of `gurdy_with_arm arm:=true sim_gazebo:=true`; `arm:=false` == default URDF; is pick_ik installed? | PASS. pick_ik may be missing from the image (noted in the line; then step 08 fails) |
| 02 | `02_unit_in_image` | `colcon test` gurdy_safety / navigation / description / arm_moveit; gurdy_core pytest (numpy < 2, so the 4 footprint-dependent skips run there) | PASS, no failures |
| 03 | `03_8c_noise_and_wiring` | 8c conventions are DONE (2026-11-26: K "half", z-depth, floor map z −0.11). Left: `<noise>` on the depth image, `detector_node` → `object3d` wiring, semantic_lab accuracy vs `semantic_lab_gt3d.json` | **SKIP** until the wiring is coded (the step detects that). Then targets: static xy median ≤ 2 cm, p90 ≤ 4 cm |
| 04 | `04_arm_spawn` [gate] | `arm:=true`: both `<ros2_control>` blocks load, `arm_controller` active, arm at `folded` (< 0.05 rad), base z > 0.07 m | PASS if gz_ros2_control accepts two hardware blocks. The likeliest failure: then merge them into the main block |
| 05 | `05_walk_no_arm` | baseline: 30 s at 0.10 m/s + 30 s at 0.3 rad/s, GT-odom tracking, falls | PASS (reference numbers for 06 / 07) |
| 06 | `06_walk_arm_folded` | same with the arm folded (0.197 kg, CoM +7.6 mm) | no fall; tracking within +5 points of 05 |
| 07 | `07_walk_arm_object` | same with +0.08 kg in the palm (arm.yaml variant): CoM +10 mm, **outside the trained randomisation** | no fall; tracking within +5 points of 05. The riskiest check |
| 08 | `08_moveit_plan` | `move_group arm:=true` (generated SRDF, leg proxies); plan only: folded → servo_view, → analytic pre-grasp of a cube at (−0.27, 0) | both plans SUCCESS. Fails if pick_ik is missing or the SRDF / URDF strings mismatch |
| 09 | `09_wrist_camera` | arm → servo_view (arm_controller), trigger on, frames + `camera_info`; fx vs arm.yaml (± 1 px); fitted floor z in base_link ≈ −0.11 (± 2 cm) | PASS if the triggered `rgbd_camera` + bridge work; else the trigger may need `always_on` |
| 10 | `10_scan_gaps` | lab4, Nav2 + detector (perception:=true) load, 3 goal rounds, `ros2 bag record /scan` → `scan_gaps.py` | PASS = statistics + recommended timeout. Expected 0.6–0.8 s (live test: gaps just over 0.5 s); capped at 1.0 s |
| 11 | `11_small_object_render` | 5 + 5 trial scenes of v3-body / v3-wrist, render, `check_labels` | > 20 small-class labels (ids 7–11); then LOOK at the sample images before the full render (`scripts/p9b_small_dataset.sh`) |

**Run order rationale:**
* build and unit tests first (01, 02)
* arm spawn (04) gates everything with the arm
* walking with the arm (05–07) before MoveIt (08), because if walking fails the arm design changes
* the wrist camera (09) before the render
* the scan recording (10) whenever the machine is otherwise idle
* the render (11) last: it is a GPU / CPU job of its own

## 4. Risks

1. **Two `<ros2_control>` blocks in gz_ros2_control** (step 04). If only one is loaded, merge the arm joints into the main
   block. That is a robot-model change: `gurdy.gazebo.xacro` would need an arm branch.
2. **Walking with the arm** (06 / 07): the CoM height change (+7.6 / +10 mm) was never in the training randomisation.
   If tracking degrades by more than 5 points or the robot falls, the fix is a MuJoCo fine-tune with the arm mass model
   (a heavy job: ~hours, alone on the machine).
3. **pick_ik not in the image** (01 / 08): installing it means an image rebuild. The fallback is planning to the analytic
   joint goals (no IK needed) with the default KDL plugin.
4. **Triggered RGB-D camera** (09): gz-sensors may not support `<triggered>` for `rgbd_camera`. Fallback: `always_on`
   with a low `update_rate` and the bridge lazily subscribed, at an RTF cost.
5. **Patch stacking:** 9f is pending (it needs regenerating on top of 9c), and the shield node now carries the 9c change.
   The 8b live numbers were measured without it (flag off, so the behaviour should be identical, but it is a code change
   in the node).
6. **Host vs image environment:** tonight's tests ran on Python 3.14 / numpy 2.5 in a venv. Some tests (footprint, ROS
   nodes) only run in the image (step 02).
7. **The 8c wiring is not written** (step 03 SKIP). The accuracy numbers wait for it; the floor must come from the fitted
   plane (map z −0.11), as in the servo.
8. **CPU:** steps 03, 10 and 11 load the CPU / GPU heavily. Never run two at a time (the script refuses when
   `docker ps` is not empty).

## 5. Results 2026-11-27 (main at 3b92bab + 3b7d402, headless, one container at a time)
Raw: `docs/results/data/day_0927/summary.txt` and the per-step folders. Every number below was measured from one run (no repeats).

| step | result | key numbers |
|---|---|---|
| 01_build [gate] | **PASS** | 11 packages built, `gurdy_with_arm arm:=true` passes `check_urdf`, `arm:=false` == default URDF. `pick_ik` **not installed** |
| 02_unit_in_image | **FAIL (1 test, known cause)** | pytest in the image: gurdy_safety 3, gurdy_navigation 24, gurdy_description 9, gurdy_arm_moveit 8 passed; gurdy_core 242 passed, **1 failed: `test_scan_timing::test_patch_applies_to_main`**. Main now contains the 9c shield patch, so the 9f patch no longer applies: this is the pending "regenerate the 9f patch on top of 9c" item, not broken code. Script bug fixed: `colcon test` ran these ament_python packages with unittest discovery ("Ran 0 tests", exit 5), the `&&` chain stopped and the step logged a false PASS (marked WRONG in summary.txt); the step now runs pytest per package |
| 03_8c_noise_and_wiring | **SKIP** | not run on request (the 8c wiring is being coded in another session) |
| 04_arm_spawn [gate] | **PASS** | both `<ros2_control>` blocks load (GazeboSimSystem + GurdyArmSystem); joint_pd / joint_state_broadcaster / arm_controller / gripper_controller active; arm error to `folded` 2.5e-11 rad, base z 0.108 m. The error is ~0 because the arm joints use a **position** command interface (gz_ros2_control sets the joint position directly). It shows the folded pose is held, not that the servos can hold it |
| 05_walk_no_arm | PASS | 30 s at 0.10 m/s: 0.0873 m/s (**12.7 %** err); 30 s at 0.3 rad/s: 0.294 (**2.1 %**); min z 0.080 / 0.103 m, no fall |
| 06_walk_arm_folded | PASS | forward **16.2 %** (+3.5 pts), turn **3.5 %** (+1.4 pts), min z 0.079 m, no fall -> within +5 pts |
| 07_walk_arm_object (+0.08 kg palm) | PASS | forward **1.4 %** (−11 pts), turn **1.6 %** (−0.4 pts), min z 0.081 m, no fall -> within +5 pts. Payload checked: palm 0.045 -> 0.125 kg in the generated URDF. Being 11 pts *better* than no-arm means a single 30 s window is noisier than the ±5-point rule, so 05-07 do not establish an effect of the arm either way. Repeats would be needed |
| 08_moveit_plan | **FAIL** | **MoveIt 2 is not in the image** (`package 'moveit_ros_move_group' not found`, `No module named 'moveit_msgs'`), not just pick_ik. Needs an image change (the project rule says images are not rebuilt: user decision) |
| 09_wrist_camera | **PASS** | 45 depth frames in 5 s, fx = fy 233.9 (arm.yaml 233.9), cx/cy 160/120, fitted floor z **−0.107 m** in base_link (expected −0.11 ± 0.02), NaN share 0. **But `<triggered>` is ignored**: SDF warns that `triggered` / `trigger_topic` are unknown elements, and with no trigger the wrist depth topic still publishes at **7.1 Hz**. So the camera always renders (RTF cost), and plan risk 4 is real |
| 10_scan_gaps | **FAIL (step timeout), data OK** | step wall limit 1500 s was too short (each 3-goal round ~12 min wall; round 3 cut). The 617 s `/scan` bag was complete and analysed afterwards with `scan_gaps.py`: 3434 scans, gap mean 0.180 s, p50 0.171, p95 0.254, p99 0.327, max 0.512 s; 59 gaps > 0.3 s, **2 > 0.5 s**, 0 > 1.0 s. **Recommended `scan_timeout` 0.5 s** (p99 x 1.2 = 0.39 -> floor 0.5), i.e. the current value. With 0.5 s the 2 gaps over 0.5 s would still cause 2 short `stale` stops per 10 min (fits the 9 blips seen in 8b). 0.6 s would have removed both in this sample. Nav2 under this load: 6/6 goals SUCCEEDED, 1 recovery. Step limit raised to 2700 s |
| 11_small_object_render | not run | on request |

Gates: 01 PASS, 04 PASS, so nothing stopped the sequence.

## 6. Risks found today and next steps
1. **MoveIt 2 missing from `gurdy2/ros:jazzy`** (08). It blocks 9e. Options: (a) a separate derived image `gurdy3/ros-moveit` on top of the existing one (`ros-jazzy-moveit` + `ros-jazzy-pick-ik`), which leaves `gurdy2/*` untouched but needs a rule change from the user; (b) plan without MoveIt: send the analytic joint goals straight to `arm_controller` (works today, no collision checking). **User decision.**
2. **Wrist camera always on** (09): gz-sensors ignores `<triggered>` for this sensor, 7.1 Hz without a trigger. Measure the RTF cost (arm:=true vs false, idle) before 9b/9e use it; fallback `update_rate` 1-2 Hz, or the camera only in the fetch launch.
3. **Arm held by a position interface** (04): Gazebo sets the arm joints kinematically, so servo torque limits and arm compliance are not simulated, and walking with the arm (06/07) sees a rigid arm. The offline torque check (9a) is the only torque evidence. Consider an effort + PD interface like the legs before trusting the walk results.
4. **Walking effect of the arm is unresolved** (05-07): 0 falls in all three, but the single 30 s windows vary more (07 better than no-arm by 11 pts) than the ±5-point rule. Run 3+ repeats per configuration (about 90 s each) before calling the acceptance met.
5. **9f scan-timeout patch** must be regenerated on top of 9c; `test_patch_applies_to_main` fails until then. Today's data says keep 0.5 s (tool) or take 0.6 s (removes the 2 gaps > 0.5 s seen in 10 min); one 10-min sample only.
6. **Phase 8a leftovers** (`phase8a_nav2_doorways.md` §6): `long_path` 8.0 m confirmation on the return leg (3 overnight failures), route switch before door B-D, and cause-2 costmap marks (record the costmap at planner failures).
7. Not started on request: step 11 (small-object render), step 03 (8c wiring).

## 7. Follow-up 2026-11-27 (user decisions: MoveIt option A, shield scan_timeout 0.6 s, keep the 8 m `long_path` confirmation)

### 7.1 MoveIt 2 image + plans (step 08 now PASS)
* New image **`gurdy3/ros:jazzy-moveit`** (`docker/ros-moveit/Dockerfile`, `FROM gurdy2/ros:jazzy`, +80 MB: MoveIt 2.12.4 move_group / OMPL /
  kinematics / simple controller manager / configs utils / msgs + **pick_ik 1.1.2**, which has a Jazzy package, so no KDL fallback is needed).
  `gurdy2/ros:jazzy` is unchanged (same image ID 1a21daf9d753). Compose service `sim_moveit` (profile `moveit`). Only step 08 (move_group) uses it
  (`SERVICE=sim_moveit` in `run_in`); every other arm:=true run stays on `gurdy2/ros:jazzy`.
* `move_group arm:=true`, MoveIt's description with the leg collision proxies (`*_coxa/femur/tibia_proxy`; no moving-arm-link vs proxy pair is
  disabled in the SRDF), plan only, chained (the second plan starts at the end of the first):

| plan | result | planning time | waypoints | invalid waypoints (`/check_state_validity`, legs included) | leg contacts | min clearance (offline `arm.clearances`): legs / body / lidar / floor |
|---|---|---|---|---|---|---|
| folded -> servo_view | SUCCESS | 0.03 s | 25 | 0 | 0 | 5.8 / 2.9 / 4.1 / 17.3 cm |
| servo_view -> pre-grasp (cube at (-0.27, 0)) | SUCCESS | 0.01 s | 20 | 0 | 0 | 7.1 / 4.2 / 5.8 / **0.5** cm |

  Sanity check of the checker: 200 random arm poses through the same service gave 36 invalid states, 22 of them touching a leg proxy, so leg
  collisions are detected. **No floor in MoveIt's planning scene yet** (the launch file says it is added from the fitted plane at run time;
  nothing does that), so the 5 mm floor clearance of the pre-grasp is not protected by the planner. Octomap: "No 3D sensor plugin" (fine, not used).

### 7.2 Walking with the arm, 3 runs each (`docs/results/data/day_0927/repeats/`, 30 s at 0.10 m/s + 30 s at 0.3 rad/s, GT odom)

| config | forward error mean (range) | turn error mean (range) | falls | min body z |
|---|---|---|---|---|
| no arm | 8.3 % (0.9-19.7) | 1.9 % (0.6-2.7) | 0/3 | 0.080 m |
| arm folded | 13.7 % (6.4-19.9) | 2.5 % (1.9-3.1) | 0/3 | 0.079 m |
| arm folded + 80 g | 14.5 % (9.7-19.7) | 3.1 % (2.1-4.1) | 0/3 | 0.078 m |

0 falls in 9 runs. By the mean, forward error is +5.4 / +6.2 points over no-arm, just above the +5-point rule, while turning is within it. But
the run-to-run spread of the SAME configuration (0.9-19.7 %) is 3x the difference, so 3 runs cannot show whether the arm changes forward tracking.
Not met as stated, and not shown to be worse either. A longer window (e.g. 120 s) or ~10 runs would be needed to resolve 5 points.

### 7.3 Wrist camera cost (`docs/results/data/day_0927/wrist_rtf/`, `scripts/wrist_rtf_check.sh`)
* **Root cause of "always on"**: `<triggered>` / `<trigger_topic>` were children of `<sensor>`; SDFormat wants them under `<camera>`. Moved
  (`gurdy_arm.xacro`, test `test_wrist_camera_trigger_is_a_camera_child_and_can_be_disabled`). Now: **0 frames without a trigger, one frame per
  trigger message** (step 09 re-run: 25 frames for 25 triggers in 5 s, fx 233.9, floor -0.107 m, PASS). New `wrist_camera.enabled` (default true)
  removes the sensor for measurements.
* RTF, robot standing, 3 rounds x 3 x 15 s: no arm 0.89 (0.83-0.92); arm + camera off 0.73 (0.69-0.75); always on 10 Hz 0.75 (0.69-0.82);
  always on 2 Hz 0.82 (0.63-0.95); triggered (idle) 0.75 (0.65-0.90). **The camera modes cannot be told apart**: another session ran host
  `pytest` (~190 % CPU) during these rounds. The ARM itself (camera off) looks like a real cost vs no arm (0.69-0.75 vs 0.83-0.92, no overlap);
  cause not measured (7 extra joints, two controllers, position interface).
* **Decision: keep `triggered: true`** (now effective). The camera renders only while the pick code triggers it, which costs nothing between picks
  by construction. The visual servo must publish a trigger per frame it needs.

### 7.4 Shield scan_timeout 0.6 s + 9f patch + costmap recording
* 9f patch (regenerated on `phase8c-wire`, c234906) applied to the node; the default is now **0.6 s** in `safety_shield_node` and `ShieldConfig`. The
  live 8b numbers were measured at 0.5 s; a stale scan now stops after ~0.65 s instead of 0.55 s. Tests: GNU `patch --batch` silently
  auto-reverses an applied patch (exit 0), so the patch tests used `--forward` instead; new test pins 0.6 s in node + config; the stale-scan test reads the default.
* **Costmap recorder** (`gurdy_navigation/costmap_recorder.py` + `costmap_recorder_node`, launch `costmap_record_dir:=<dir>`, default empty = off;
  `p5_final_eval.sh RECORD_COSTMAP=true` -> `<stack dir>/costmaps`): on every planner_server failure line in /rosout it saves the last two global
  costmaps (received BEFORE the clear), the parsed start / goal, the GT pose and the lethal-cell count within 0.3 m of the start (.npz/.json/.pgm).
  Live check in lab4 (goal off the map): recorded 20 ms after the failure; the snapshot shows doorway A-B at cost 41 at the gap centre (doc 1.3: ~40),
  99-100 at the jambs. Bursts are debounced 0.5 s (the second off-map goal 38 ms later was not saved). Unit tests `test_costmap_recorder.py`.
* Tests in the images: gurdy_core 244 passed, 2 skipped (patch already applied); gurdy_navigation 29, gurdy_safety 3, gurdy_description 9,
  gurdy_arm_moveit 8 passed; brain (brain image) 1029 passed, 3 skipped.

### 7.5 lab4_fetch smoke test
`world:=lab4_fetch arm:=true` loads; all 8 small objects exist (inline models, no `model://`) and are still at their spawn poses after 20 s.

### 7.6 Next blocker for the first grasp test
**The arm joints use a position command interface**: Gazebo sets them kinematically, so a grasp exercises neither the arm's torque limits nor any
compliance when the fingers or palm touch the object or the floor, and the pre-grasp sits 5 mm above a floor that MoveIt does not know about. Before a
first grasp: (1) add the floor (fitted plane) as a collision object in MoveIt's scene; (2) switch the arm to effort + PD like the legs (or at least
check contact forces); (3) execute the plan (not plan_only) through `arm_controller` and close the effort-controlled gripper on `f_red_cube` with its
GT pose (no detector needed for the first test; the v3 small-object detector is not rendered / trained yet, step 11).
