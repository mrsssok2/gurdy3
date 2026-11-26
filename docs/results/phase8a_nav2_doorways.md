# Phase 8a: Nav2 "path blocked" near doorways (lab4), 2026-11-26

Question: why does the robot sometimes report "my path is blocked" / "I cannot find a path there; it is blocked or in unknown space" in lab4,
often near a doorway? Evidence first (no code changes), then a doorway suite committed before any change, then general fixes, then a short
Gazebo check. **Bottom line: the doorways were not too narrow. 5 of the 6 failures were Nav2 behaviour-tree goal-acknowledgement timeouts
(20 ms) under CPU load, which abort the goal without running any recovery. The sixth was a planner failure from a free pose just inside room C.**

## 1. Evidence

### 1.1 Every blocked-path failure (lab4 v3 runs 1-3, postfix_7d, web robot)
Sources: `brain/runs/heldout_lab4_v3/run{1,2,3}`, `brain/runs/postfix_7d/run1` (mission json + per-mission stack `launch.log`), `~/.gurdy/memory.db`.
88 Nav2 `navigate_to_pose` goals in these runs, 57 succeeded, **5 "Goal failed"**, plus 1 planner refusal through the safety path check.

| run / mission | robot at (map) | nearest doorway | goal | what Nav2 reported | mission |
|---|---|---|---|---|---|
| v3 run2 `v3_step_into_charlie` | (-0.82, 1.54), room A | C-A, 0.47 m before the gap | room C | `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose` -> BT abort | passed (re-sent goal reached C) |
| v3 run2 `v3_bravo_then_alpha` | (3.74, -1.26), room B | A-B, 1.8 m | room A | same, `compute_path_to_pose` | passed |
| v3 run2 `v3_bravo_delta_home` | (4.02, 1.84) | **in** door B-D (0.16 m from the gap centre) | room D | same, `follow_path` | **failed** (never reached D) |
| postfix `v3_cone_in_d` | (1.21, 3.78), room C | D-C, 0.9 m | search goal in D | same, `follow_path` | passed (cone already mapped) |
| postfix `v3_toolbox_search` | (2.51, -0.65), room B | A-B, 0.5 m past the gap | search goal | same, `follow_path` | **failed** (search stopped) |
| v3 run3 `v3_leg_walking` | (-0.70, 2.62), room C | C-A, 0.62 m past the gap, on its axis | home (0, 0) | planner_server: `Failed to create plan with tolerance of: 0.300000` (direct ComputePathToPose of the safety path check, not the BT) | **failed** (refused "blocked or unknown space") |
| web robot (memory.db, flat_lab, 2026-11-19) | ? | flat_lab doorway? | room B | mission state `blocked`; no Nav2 log kept | unknown |

The same signature is in the Gurdy 2.0 final-eval runs too: 6 Nav2 aborts there, **all 6 goal-acknowledgement timeouts** (4 compute_path_to_pose,
2 follow_path). In all v3 / postfix / final-eval logs: **0** `Failed to make progress`, **0** spin / backup / wait recoveries. `Optimizer fail
to compute path` (MPPI) appears in 13 stacks but always recovers within the 1 s `failure_tolerance` and never caused an abort.

Why "near doorways": 4 of the 5 BT aborts happened within 1 m of a doorway. The mechanism is not tied to geometry, though: the BT re-plans at 1 Hz
everywhere, and doorway crossings make up a large share of every lab4 mission's walking time. 17 `Control loop missed its desired rate of
10 Hz` warnings in the same stacks show the CPU contention (Gazebo + policy + detector + MPPI 1000 x 40 + agent on 12 threads).

### 1.2 Mechanism of the BT abort (verified in the BT XML of the image)
`bt_navigator.default_server_timeout: 20` (ms, the Nav2 default) is how long a BT action node waits for an action server to ACCEPT a goal.
The default `navigate_to_pose_w_replanning_and_recovery.xml` sends a new ComputePathToPose every second and updates FollowPath. When the
planner or controller server's executor is busy for more than 20 ms, the BT node fails **without an error code**. Both recovery gates
(`WouldAPlannerRecoveryHelp`, `WouldAControllerRecoveryHelp`) only let recoveries run for specific error codes. With no code, the
`NavigateRecovery` node fails at once: the goal is aborted with no costmap clearing and no retry. The backend reports that as
`path_blocked` ("The path is blocked: navigation gave up on the goal."). The logs fit this: the abort comes 10-30 ms after a normal
"Passing new path to controller" and nothing runs in between.

### 1.3 Doorway width vs footprint and inflation (`scripts/gen_world_lab4.py`, `nav2_params.yaml`, both costmaps identical)
- Doorway gap 1.00 m (walls 0.10 m). Footprint hexagon 0.546 x 0.570 m: inscribed radius **0.273 m**, circumscribed **0.340 m**.
- Inflation radius 0.55 m, cost_scaling_factor 4.0 (local = global). Cost at distance d from a jamb = 252 exp(-4 (d - 0.273)):
  gap centre (d = 0.50) **101**, d = 0.40 -> 151, 0.30 -> 225, lethal-inscribed at d <= 0.273.
- The footprint centre can move ±0.227 m from the gap centre before any footprint cell touches inscribed (lethal) cost. With the circumscribed circle the margin is ±0.16 m.
  Because the inflation radius (0.55) is larger than the half gap (0.50), no doorway cell has zero cost. Doorways are expensive but passable.
- Measured, not just computed: `scripts/lab4_door_check.sh` (2026-11-22) passed all 4 doorways both ways, 16/16 goals, 0 recoveries, max stall 1 s.
  Of the 88 goals above, **none** failed with a geometric cause (no planner failure, no controller "no valid trajectories", no progress-checker abort) except the one row in 1.1.
  -> Geometry is not the main cause. Not changed.

### 1.4 LiDAR self-hits
The lidar plane is 0.05 m above base_link and the femurs reach it. In Phase 3 (`docs/results/phase3_selfhit_and_minspeed.md`), 91 % of walking
scans had returns at 0.15-0.29 m, all inside the footprint. `scan_self_filter_node` already turns every return inside the footprint (+2 cm)
into +inf. The costmaps also ignore returns below `obstacle_min_range: 0.2`, and `clearing: true` with raytracing to 6 m is on in both
costmaps. A self-hit would show up as lethal cost inside the robot or a planner "start occupied" error. Neither appears in any log.
**Not re-measured tonight** (limit). Not changed.

### 1.5 Progress checker (0.15 m in 20 s)
Nominal speed 0.10 m/s means 0.15 m in about 1.5 s, so 20 s is 13x margin. 0 `Failed to make progress` in 88 goals. Not changed.

### 1.6 The planner failure from (-0.70, 2.62)
The pose is free: 0.6 m from the divider on the C-A axis, nothing static nearby. NavFn could not connect it to home, so the global costmap
must have held a lethal mark from the obstacle layer around the robot or across the gap. The marks were not recorded, so this is a
hypothesis. The safety path check calls ComputePathToPose directly. Unlike the BT, it never cleared the costmap or re-planned.

### Ranked causes
1. **BT goal-ack timeout of 20 ms under CPU load**: 5/5 BT aborts in lab4, 6/6 in the final eval; the abort skips every recovery. Strong evidence.
2. **Planner failures from free poses because of obstacle-layer marks that a costmap clear removes**: 1 case before (no clear-and-retry in the agent's own path check, so it was refused); 2 more in tonight's quick check, recovered by the BT's clear. Source of the marks unknown.
3. Doorway inflation (no zero-cost cell in a 1 m gap): makes doorways costly, not impassable. No failure traced to it.
4. LiDAR self-hits, obstacle clearing, progress checker: already handled or margin is large. No failure traced to them.

## 2. Doorway suite (committed before any change)
`brain/eval/final_suite_doorway_v1.py` (commit 2e61c44, before the fix commit 480bf7f), test `brain/tests/test_final_suite_doorway_v1.py`.
14 missions: each doorway at least twice (both directions) via single hops, back-and-forth, the full ring both ways, two-hop goals, and 2 tight turns:
`dw_tight_corner` (B -> C through D, door-to-door corner around the mannequin) and `dw_tight_reverse` (reversal ordered 28 s after "walk to room C", at or near door C-A).
Checks: the robot passes within 0.45 m of each doorway's gap centre, ends in the target room, and never says `blocked|unknown space|gave up`. **Not to be tuned on.**

## 3. Changes (general, not per mission)
1. `nav2_params.yaml` `bt_navigator.default_server_timeout: 20 -> 1000` ms. A busy server now has 1 s to accept a goal before the BT gives up.
   The BT tick may block that long in the worst case. MPPI keeps running in the controller server, so the robot keeps moving along the last path.
   Fixes cause 1 at its root and is independent of the world.
2. `brain/ros/rclpy_port.py` `compute_path`: when the plan fails, clear the global costmap once (`/global_costmap/clear_entirely_global_costmap`;
   the static map stays, obstacles re-mark from the next scans) and plan again. This is the same planner recovery Nav2's BT does
   (`ClearGlobalCostmap-Context`). A real obstacle is re-marked and the retry still fails, so honest refusals stay. Test in `brain/tests/test_rclpy_port_smoke.py` (ROS image).
3. Not changed, with reasons in 1.3-1.5: footprint (matches robot.yaml via `patch_nav2_params`), inflation / cost scaling, self-filter,
   obstacle clearing, progress checker, recovery behaviours. No failure was traced to them, and tuning them without evidence could break the 16/16 door check.

## 4. Quick check tonight (doorway_v1, 4 missions, once, headless, Gazebo)
Run `brain/runs/doorway_v1/quick_check` (commit 480bf7f, `ONLY=dw_a_to_b,dw_c_to_a,dw_b_to_d,dw_tight_reverse`, seed 1, one attempt each):
**4 / 4 PASS**, 0 falls, 0 wrong claims, 0 blocked replies. Totals: 525 s sim, 1211 s wall, 49.1 m walked, LLM p50 0.91 s.
Nav2: 6 goals, 6 succeeded, **0 aborts, 0 goal-ack timeouts**; 5 `Control loop missed` warnings, so the CPU load was still there.

| mission | doorways | sim s (limit) | path m | result |
|---|---|---|---|---|
| `dw_a_to_b` | A-B forward | 87.5 (240) | 8.3 | PASS |
| `dw_c_to_a` | C-A both ways | 195.7 (460) | 18.5 | PASS |
| `dw_b_to_d` | A-B, B-D from B | 168.9 (460) | 15.8 | PASS |
| `dw_tight_reverse` | reversal at C-A | 73.2 (340) | 6.4 | PASS |

**Cause 2 happened again** in `dw_b_to_d`. On the B -> D leg the planner failed twice, from free poses in room B, (4.12, -0.04) and
(4.20, 0.15), 2 m before door B-D: `Failed to create plan with tolerance of: 0.300000`. Both times the BT cleared the global costmap (its
planner recovery runs here because a planner failure carries an error code) and the goal then SUCCEEDED. So something in the global
costmap's obstacle layer that a full clear removes blocks the B-D doorway for a while. This is the mark hypothesised in 1.6. What marks
it is not known (the costmap was not recorded). Nav2's own BT already recovers this way, and change 2 now does the same for the agent's
path check. Finding the source of those marks is the next step (record the global costmap at each planner failure).
This is a smoke test (4 missions, one run), not a measurement of the failure rate.

## 5. Overnight run (written, NOT started)
`scripts/doorway_v1_runs.sh`: doorway_v1 x 3 seeds -> `brain/runs/doorway_v1/run{1,2,3}`, then a lab4 v3 regression subset (10 navigation-heavy
missions, including every one with a blocked-path failure above) x 3 -> `brain/runs/doorway_v1/v3_regr{1,2,3}`. Ollama is restarted before each run
and stopped at the end. Estimated time 5-6 h, based on the v3 run time (33 missions in about 2 h). The v3 missions are not held-out for this question any more.

## Honest limits
- Cause 1 is proven from the logs. Cause 2 is a hypothesis: the costmap at the time was not recorded.
- The fix only shows its effect over many goals: a 5-in-88 failure rate cannot be confirmed by 4 missions. The overnight 3 x 14 + 3 x 10 run is the real test.
- The ack delay itself was not measured. The CPU contention that causes it is still there; 1000 ms is a margin, not a guarantee.
- Self-hits were not re-measured on lab4 tonight; the evidence is from Phase 3 (open world, same robot and filter).
- The one web-robot blocked mission (flat_lab, 2026-11-19) has no Nav2 log, so its cause is unknown.
- Single world (lab4, 1 m doors). Narrower doors would hit the inflation and lane-width limits in 1.3.
