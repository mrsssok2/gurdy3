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
