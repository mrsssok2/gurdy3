# Phase 8a: Nav2 "path blocked" near doorways (lab4), 2026-11-26

Question: why does the robot sometimes report "my path is blocked" / "I cannot find a path there; it is blocked or in unknown space" in lab4,
often near a doorway? Evidence first (no code changes), then a doorway suite committed before any change, then general fixes, then a short
Gazebo check. **Bottom line: the doorways were not too narrow. 5 of the 6 failures were Nav2 behaviour-tree goal-acknowledgement timeouts
(20 ms) under CPU load, which abort the goal without running any recovery. The sixth was a planner failure from a free pose just inside room C.**
**Overnight after the fix (section 6): doorway_v1 39/42 (93 %, 13/14 each run), v3 subset 28/30 vs 18/30 before; 132 Nav2 goals, 0 aborts, 0 blocked replies; 0 falls, 0 wrong claims. Remaining: `long_path` confirmation (3), route switch at door B-D (2); planner failures from free poses (cause 2) still happen but are recovered.**

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

## 5. Overnight run (ran 2026-11-26 17:43 -> 2026-11-27 01:05 UTC, results in section 6)
`scripts/doorway_v1_runs.sh`: doorway_v1 x 3 seeds -> `brain/runs/doorway_v1/run{1,2,3}`, then a lab4 v3 regression subset (10 navigation-heavy
missions, including every one with a blocked-path failure above) x 3 -> `brain/runs/doorway_v1/v3_regr{1,2,3}`. Ollama is restarted before each run
and stopped at the end. Estimated time 5-6 h, based on the v3 run time (33 missions in about 2 h). The v3 missions are not held-out for this question any more.

## 6. Overnight results (commit 30e1f91, clean tree, gurdy-sft-400, typed, fresh stack per mission)
Aggregated with `brain.eval.heldout_report` (current scorer); Nav2 counts from every attempt's stack `launch.log`
(grep for `[bt_navigator]: Goal failed`, `Timed out while waiting for action server to acknowledge`, `Failed to create plan`, `clear entirely the global_costmap`).
Before = `brain/runs/heldout_lab4_v3/run{1,2,3}` (commit 28d4987, before both 8a changes **and** before the Phase 7c language fixes).
The frozen v3 results (`phase5_heldout_lab4_v3.md`) are not changed; the "before" numbers below are the same records re-scored by today's scorer for a like-for-like comparison.

### 6.1 doorway_v1 (14 missions x 3 seeds)
**13 / 14 in every run: mean 93 % (range 93-93 %), 39 / 42.** 0 falls, 0 wrong claims, 0 grounding violations, 0 timeouts, 0 infra failures
(1 stack restart: run 2 `dw_a_to_d` attempt 1 `sim_hang`, attempt 2 ok). **0 blocked / unknown-space / gave-up replies.** 873 m walked, 9547 s sim.
By category: single 12/12, both 12/12, ring 6/6, two-hop 5/6, tight 4/6.

| failure | run | what happened | cause |
|---|---|---|---|
| `dw_d_to_a` | 1 | Robot reached D (0.1 m from goal). "walk back to room A" -> "That is 8.3 metres away. Shall I go?"; the mission script never answers, robot stays in D. | Agent safety rule `long_path` (`confirm_path_longer_m: 8.0`). D->A is 7.8-8.4 m depending on where in D the robot stands, so this flips between runs. Not navigation. |
| `dw_tight_corner` | 2, 3 | B -> C: robot walked toward door B-D, stopped 0.2-0.5 m before the gap (y 1.3-1.8, gap at y 2.0), dithered there ~40 s, then turned round and reached C through A (A-B, C-A). Arrived "0.1 m from the goal", true claim; scored FAIL only because door D-C was never passed. On the A route the planner also failed 1-2x from free poses in/after door A-B; each time the BT cleared the global costmap and continued. | Not proven (costmap not recorded). Same signature as cause 2: something in the global costmap makes the B-D gap expensive or closed as the robot approaches, so the 1 Hz re-plan switches to the A route. Run 1 shows the same hesitation at door D-C (~80 s around (2.5, 4.1)) but got through. |

### 6.2 lab4 v3 regression subset (10 missions x 3 seeds), same missions before vs after

| | before (v3 runs 1-3, 28d4987) | after (v3_regr 1-3, 30e1f91) |
|---|---|---|
| passed | **18 / 30 (60 %)** | **28 / 30 (93 %)**, per run 10 / 9 / 9 |
| falls / wrong claims | 0 / 0 | 0 / 0 |
| missions with a blocked / unknown-space reply | 4 | **0** |
| failures caused by navigation | 3 (`v3_bravo_delta_home` run 2 BT abort + run 3 timeout, `v3_leg_walking` run 3 planner refusal) | **0** |
| failures caused by language / agent | 9 (`v3_d_back_to_a` x3 `run over` regex, `v3_cone_in_d` x3 place-in-object, `v3_toolbox_search` x3 `where_is`; fixed in 7c) | 2 (`long_path` confirmation, below) |

Per mission (before -> after): delta_phonetic 3->3, room_b 3->3, step_into_charlie 3->3, bravo_then_alpha 3->3, c_then_b 3->3,
bravo_delta_home 1->2, d_back_to_a 0->2, cone_in_d 0->3, toolbox_search 0->3, leg_walking 2->3.
**The before/after gain mixes two changes**: d_back_to_a, cone_in_d and toolbox_search went from 0/3 by the Phase 7c language fixes, not by the Nav2 change.
The navigation part alone: 3 nav failures -> 0.

After-fix failures, both the same agent rule as `dw_d_to_a`: `v3_d_back_to_a` run 2 (reached D, then "That is 8.2 metres away. Shall I go?" for the
return home, mission timeout) and `v3_bravo_delta_home` run 3 (patrol B, D done, then "8.4 metres away. Shall I go?" for home, timeout).

### 6.3 Nav2 before vs after

| | before: v3 all 33 x 3 | before: v3 subset | after: v3 subset | after: doorway_v1 |
|---|---|---|---|---|
| mission records | 99 | 30 | 30 | 42 |
| BT `Goal succeeded` | 52 | 31 | 43 | 89 |
| BT `Goal failed` (abort) | **3** | **3** | **0** | **0** |
| goal-ack timeouts (cause 1) | 3 | 3 | 0 | 0 |
| planner `Failed to create plan` (cause 2) | 12 | 8 | 6 | 11 |
| ... recovered by a global-costmap clear | 11 | 7 | 6 | 11 |
| `Control loop missed its desired rate` | 7 | 6 | 0 | 42 |

**After the fix: 132 Nav2 goals, 0 aborts, 0 goal-ack timeouts, 0 blocked replies** (before: 3 aborts in 55 goals of the v3 runs, 5 in 88 incl. postfix).
Cause 1 is fixed as far as these 72 missions can show. **Cause 2 is not fixed, only recovered:** 17 planner failures from free poses after the fix, every one cleared
and recovered by the BT, and the 2 `dw_tight_corner` route switches look like the same mark. The CPU load is still there (42 `Control loop missed` warnings in the doorway runs).

### 6.4 Next
1. Cause 2: record the global costmap (or the obstacle layer around the robot) at each planner failure and when the robot stalls > 10 s within 1 m of a doorway; find what marks the gap.
2. `long_path` at 8.0 m confirms "go back home" after a mission the user just ordered (3 of the 3 non-route failures tonight). Options: do not confirm a `return_home` / the return leg of an ordered plan, or raise the threshold above the lab4 diagonal. Agent decision, not done here.
3. `dw_tight_corner` checks the route, not just the arrival; keep it as is (suite frozen).

## Honest limits
- Cause 1 is proven from the logs. Cause 2 is a hypothesis: the costmap at the time was not recorded.
- The fix only shows its effect over many goals: a 5-in-88 failure rate cannot be confirmed by 4 missions. The overnight 3 x 14 + 3 x 10 run is the real test.
- The ack delay itself was not measured. The CPU contention that causes it is still there; 1000 ms is a margin, not a guarantee.
- Self-hits were not re-measured on lab4 tonight; the evidence is from Phase 3 (open world, same robot and filter).
- The one web-robot blocked mission (flat_lab, 2026-11-19) has no Nav2 log, so its cause is unknown.
- Single world (lab4, 1 m doors). Narrower doors would hit the inflation and lane-width limits in 1.3.
