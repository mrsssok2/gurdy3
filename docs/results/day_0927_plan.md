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
| apply 9f scan-timeout patch | **PENDING.** It applies cleanly to main's node alone, but not on top of the 9c patch (same lines; it would need fuzz). Its new file `scan_timing.py` is already here through the 9a-arm merge. To do: regenerate the node part on top of 9c after step 10 picks the timeout value. |
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
