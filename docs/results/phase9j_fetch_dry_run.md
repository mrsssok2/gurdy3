# Phase 9j: first fetch dry run in Gazebo with det_v3_s (2026-11-27)

Stack: `CHAT_LOG_DIR=... FETCH=true MODEL=keyword MEMORY=false scripts/start_web_robot.sh` (lab4_fetch, arm:=true, object3d:=true,
move_group on `sim_moveit`, runtime detector det_v3_s on the CPU provider, no LLM: the fetch router is deterministic). Commands
through the dashboard REST API (`/api/say`), one stack for all tries, robot NOT reset between tries. Ground truth from gz
`/world/lab4_fetch/pose/info`. Single tries, no repeats. RTF ~0.3 (sim 1.5 s per 5 s wall) with Gazebo + arm + wrist camera
+ CPU detector + move_group. Raw logs: `docs/results/data/fetch_dry_run_1127/`.

| try | command | target (GT) | start | result | wall | failure reason |
|---|---|---|---|---|---|---|
| 1 | "pick up the blue can" | blue_can (-1.30, 0.30), 1.33 m, behind the robot, not yet in the map | (0, 0) facing +x | **stalled** in the search, aborted by e-stop after 11 min: "I stopped (emergency stop). I'm not holding anything." | 681 s | search = RosBackend.find: Nav2 goal to the room A viewpoint (0, 0) succeeded, the robot turned to ~178° and the detector mapped the can (map (-1.32, 0.34): **3-4 cm from GT**; agent said "Found the blue can in Room A, 1.3 metres ahead"), then nothing: no /cmd_vel (policy "stand: cmd_vel timeout"), task `find` still active, the fetch session never went on to the approach |
| 2 | "pick up the blue can" (now in the map) | blue_can (-1.30, 0.30) | (0.02, 0.08) | **GRASP_FAILED**: "I couldn't get a grip on the blue can after 3 attempts. It is still where it was." | 246 s | approach worked (Nav2 to the stand-off (-0.72, 0.34) facing the can, turn, back-up; MoveIt moves executed, status SUCCEEDED); 3 grasp attempts failed; the can was **pushed 6 cm** (-> (-1.34, 0.25)); robot ended at (-0.96, 0.35), 0.39 m from the can. Per-attempt cause not logged (the ROS fetch robot does not log verify details) |
| 3 | "pick up the red can" (~3 m search test) | red_can (1.30, 2.60), room C, 3.2 m, not in the map | (-0.96, 0.35) | **stalled** in the search exactly like try 1, aborted by e-stop | 438 s | Nav2 goal to the room A viewpoint (0, 0) succeeded, then no motion: the room sweep never turned / moved on to room C, so the multi-room search (and the multi-viewpoint option) was never reached |

**Summary: 0 / 3 grasped. The detection side works (det_v3_s mapped the can 3-4 cm from GT from ~1.3 m); the replies were
honest in all three (no false success, e-stop replies correct).**

## Findings (not fixed: no time before the 16:45 IST hard stop)
1. **Search stalls after its first Nav2 step when called from the fetch session (reproduced 2 / 2).** `RosFetchRobot.search`
   starts `RosBackend.find` and waits while `backend.task` is set, ticking the backend every 0.2 s (`_wait`). After the first
   `nav` step the next steps (2 s wait + 90° `rel` turns, `_scan_steps`) never send a command (`_step_rel` sends /cmd_vel every
   tick and has a no-progress stop, yet the policy saw no /cmd_vel for minutes), and the task never ends although `until`
   (target in the map) should have fired in try 1. So the task state machine is not advancing in this path. Next: log
   `_run_task` (step kind, pose) during a fetch search; check `_pose()` with `--odom-topic /odom` and the fetch session's
   tick vs the host's own pump loop (both may tick; or neither while the agent lock is held). The FakePort tests
   (`test_ros_fetch.py`) do not cover this; the normal agent `find` worked in the Phase 5 live tests.
2. **Grasp in the fetch path fails where grasp_test succeeded 25/25.** Differences from grasp_test: real Nav2 approach (no
   teleport), the object 0.39 m from the robot after the back-up (grasp_test starts in the zone after servo steps), detector
   box (not GT) for the servo, can knocked 6 cm. Needs the per-attempt log (servo result, residual, finger widths, verify
   reason) in `RosFetchRobot`, then a fetch-path grasp test with the object placed at a known stand-off.
3. Visual servo `box_from_depth` prints NaN RuntimeWarnings (inf depth x 0); harmless (masked afterwards), but noisy.
4. Infra: `start_web_robot.sh` had lost its exec bit, and with FETCH=true it ran the plain `sim` image, where move_group
   cannot start (fixed: `sim_moveit`). One start in sim_moveit reported "Nav2 never became ready" (logs lost in the
   container; the next start was ready on attempt 1; cause not established; `CHAT_LOG_DIR` now keeps the logs on the host).
5. Not tested: the ~2 m vs ~3 m detection-range question in the live search (blocked by finding 1), place / hand-over,
   `search_view_radius_m`.

## Follow-up (2026-11-27 evening, CPU only: fake-robot tests, no Gazebo)
**Finding 1, root cause FOUND (not an rclpy executor deadlock).** `run_text` wraps RosBackend (`with_incidents`, plus
MemoryBackend with MEMORY=true) and gives the WRAPPER to `RosFetchRobot`. `IncidentBackend.tick()` only samples pose / objects
for its recorder; it does not tick the wrapped RosBackend. Normally the agent's pump thread ticks RosBackend through
`poll_events()`, but that needs the agent lock, which `Agent.say` holds for the whole fetch. So during a fetch search nobody
ran RosBackend's task machine: `_start` sends the first Nav2 goal directly, Nav2 finishes it on its own, and then no step, no
`until` (target in the map) and no timeout is ever evaluated. The same wrapper also swallowed `search_view_radius_m`
(set on the wrapper, never seen by RosBackend). Correction to the table above: "Found the blue can ..." in try 1 was the
e-stop finishing the task (its final report says found because the can was in the map by then), not the search ending.
Executor check: RclpyPort = MultiThreadedExecutor(3), RclpyArmPort = MultiThreadedExecutor(2), each on its own thread, default
callback groups; every blocking wait (`_wait` on a threading.Event, `_done` polling) runs in the CALLER's thread, never inside a
callback; Nav2 result / feedback only set state in done-callbacks. No deadlock path.
**Fix:** `RosFetchRobot.engine` = the innermost backend (follows `_inner`); `_wait` ticks the engine (and still the wrapper,
for the incident recorder); the search waits on / reads missions from the engine and sets `search_view_radius_m` on it. Events
are not consumed, so the agent gets them after the fetch. Tests (`brain/tests/test_ros_fetch.py`): fetch -> search -> nav
THROUGH the run_text wrappers (failed before the fix: "I don't know any blue cube", passes now), radius reaches the engine.
Side finding: the fetch search limit `search_timeout_s` 300 s is tight: the default room sweep needed 270 s (fake, sim time)
to reach room D; with `search_view_radius_m` 1.5 a full sweep needs > 900 s. Not changed.

**Finding 2: reach is NOT the cause; the fetch grasp path lacked the three grasp_test fixes.** From the measured poses: at the
end of try 2 the can's ORIGINAL spot was at base (-0.341, -0.039), 0.34 m: 1 cm from the can's servo target (-0.35, 0) and
inside the side-grasp reach (x -0.30 .. -0.395 m on the centre line, `grasp_plan` ok). The 0.39 m in the table is to the can
AFTER it was pushed (base (-0.381, -0.084)). So the stand-off / back-up was right and is unchanged. But `RosFetchRobot.grasp`
had none of today's grasp_test fixes (13/15 -> 25/25): no fitted floor in MoveIt's scene, no target collision object for the
big moves (the arm swing pushed the can in grasp_test fixed5x5 exactly like here), no settle + measured-FK centring.
**Fix (no new flag; the fetch path itself stays behind FETCH=false):** `grasp()` now = the grasp_test pipeline: floor added
once (`RclpyArmPort.add_floor`, scripts/moveit_floor.py split into `fit_floor_from` + `floor_scene`, no spin_once on the
executor-owned node), target in the scene for the big moves and removed before the straight approach (also on errors),
settle + `centre_correction` + re-plan. **Per-attempt record** (`RosFetchRobot.attempts`, JSONL with `--fetch-log`): robot
pose, map (detector) pose, servo result, estimate + source, robot->object distance, GT pose + estimate error (`--fetch-gt
log`), yaw, plan ok / reason, target in scene, settle errors, centre shift, re-plan, fingers closed / lifted, standing
points, outcome + reason. Tests: floor once, target on/off per attempt, one record per attempt with reason; centring with a
-0.1 rad shoulder sag: success with it, 3 misses without; teleport approach (variant A) and GT grasp (variant B) with a fake
ground truth. Suites: brain 1189 passed / 3 skipped, gurdy_core 409, vision 56, CI yaml / lint / safety PASS.

**Retest (NOT run; needs Docker + GPU):** `nohup scripts/fetch_retest.sh > /tmp/fetch_retest.out 2>&1 &`
(A = teleport to the grasp_test pose + detector pose, B = Nav2 approach + GT pose, C = full fetch with search; blue can 1.33 m
from the spawn point; TRIES 3 for A / B, C_TRIES 2 fresh stacks; report `docs/results/phase9j_fetch_retest.md`). Expected
~65-80 min wall (estimate from today's start-up and try times at RTF ~0.3, not measured for this script).
