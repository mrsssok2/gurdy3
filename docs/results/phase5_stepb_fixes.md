# Step B: fixes for the remaining lab4 v2 failures

Source: `docs/results/phase5_heldout_lab4_v2.md` (88 % mean, 84-91 %). v2 is no longer an unseen suite; nothing from it (or v1, testset_v1, ood_v1) was re-scored. All numbers below are measured here; the probe runs are development measurements (`brain/eval/final_suite_probe_b.py`, `brain/runs/stepb_probe/`), not a held-out score.

## 1. "that room" / "the other room" without an earlier room asks
`rooms.room_reference` + `Agent._room_turn`: "that / the other / another room" with no room named earlier -> `Which room: A, B, C or D?` (the answer completes the held command, the planner is never called). With an earlier room: "that room" -> that room; "the other room" -> the one remaining room (2-room world) or a question listing the remaining rooms. The last room comes from the conversation (`Agent.last_room`). Tests: `brain/tests/test_stepb_refs_stop.py` (15).

## 2. Stop explanation
`Agent.last_stop` = {reason (user_stop, estop, goal_reached, blocked, no_go, timeout, cancelled, failed), time, command}, set by the stop fast path and by `task_finished` / `task_failed` / `estop` events. "why are you standing still", "what made you stop", "why have you halted", "how come you stopped" (`stopwords.is_stop_why`) are answered from it before the planner: `You asked me to stop ("please stop"), 4 seconds ago.` The backend's own "cancelled" bookkeeping does not overwrite a user stop. No stop on record, or the robot is working -> the planner / `why` tool as before. Tests: same file.

## 3. No-go detour
Root cause (debug trace, `GURDY_DEBUG_NAV=1`): the checked plan A->B->D (7.8 m) was fine, but Nav2 re-plans a goal by itself and knows nothing about our zones: after a deviation or a recovery spin the shortest way from where the robot stands can lead through room C, and the live pose check then (correctly) stopped the mission. Fix: while a zone exists the checked route is walked in legs of about 2 m (`RosBackend._waypoints`, only points with no occupied cell within the inflation radius, so no doorway waypoints); each leg's shortest path stays on the checked route. The estimate counts the route once (first try said 17.6 m: legs counted from the start).
Measured, same mission as `v2_nogo_c_route` (three runs each, fresh stack):
| | pass | sim s (path m) |
|---|---|---|
| v2 held-out runs (before) | 1/3 | 149 (13.4) stopped, 135-180 |
| probe before | 1/3 | |
| probe before, with debug prints only | 3/3 | ~170 |
| **after (legs)** | **3/3** | 249 (23.4), 179 (16.8), 336 (31.2) |
The robot is nondeterministic (before: 1/3, 1/3, 3/3 on the same code), so 3/3 is evidence, not proof. The legs make the walk slower and wigglier (path 17-31 m for a 8 m route). A first version with waypoints on the doorway was aborted by Nav2 within a second and led to the "no doorway waypoints" rule. Stops honestly ("the way there leads through a no-go zone") when no route exists. Tests: `test_nogo_route.py`.

## 4. `find` in a named room
`find` takes an agent-internal argument `in_place` (not in the planner prompt, not accepted from the planner: `specs.INTERNAL_ARGS`, `route_plan`): only an object INSIDE that room counts; if none is known there the robot goes to that room and looks (no global explore). Set by routing from the planner's invented `place` argument or from the user's words ("... in room D"). The replies say it: `I did not find it in room D after N seconds.` Mock and ROS backends both. Tests: `test_stepb_refs_stop.py` (cone remembered in room A does not satisfy room B, remembered in room A is accepted for room A, unknown room fails cleanly).

## 5. Search speed
Findings: in lab4 `find` ran the frontier explorer FIRST on a saved map and used the whole 300 s budget walking frontier cells (v2: 214-392 s searches that never reached the room sweep). Changes: if the rooms are mapped (>= 85 % known cells) sweep first, explore after; unmapped -> explore first as before. Sweep order = nearest-neighbour chain, rooms already looked at in this session last (a room is marked after a complete circle); scan dwell 2.5 -> 2.0 s per heading.
Five lab4 searches, sim seconds until the robot stopped (before = three v2 runs per mission, 15 searches; after = one probe run):
| search | before (3 runs, mean) | after |
|---|---|---|
| yellow box (room C) | 82.6 | 260.3 |
| fire extinguisher (room C) | 28.7 | 15.1 |
| mannequin (room D) | 254.4 (1 of 3 timed out) | 163.6 |
| toolbox in room D | 296.0 (1 of 3 timed out) | 167.1 |
| red barrel (room D) | 193.1 | 257.8 |
| **mean** | **171.0** (12/15 found) | **172.8** (5/5 found) |
Honest reading: no gain in the mean; the 300 s explore-budget timeouts are gone (5/5 found, longest 260 s vs 392 s) and the room-restricted search is faster, but the order of rooms is luck without a prior (yellow box was found early by the old explorer and is now in the last room visited). The time is dominated by walking at ~0.09 m/s (4 m = 44 s) plus ~35 s per room scan; a real gain needs a prior (the persistent memory of earlier sessions) or a faster scan. n = 1 per search after, so the per-search differences are not significant.

## Regression (offline sets, current code, gurdy-sft-400; files in `docs/results/stepb_regression/`)
| set | after step A | now |
|---|---|---|
| testset_v1 | 94 % | 94 % |
| testset_v1_1 | 95 % | 95 % |
| ood_v1 | 75 % | 75 % |
Same failing cases as after step A. Brain unit tests: 714 passed, 3 skipped. The flat_lab 27-mission suite was NOT re-run in this step (the find changes touch its search missions: run it once before relying on them).

## New held-out suite
`brain/eval/final_suite_lab4_v3.py`: 33 missions (rooms 4, multi-room 4, objects 5, ambiguity 3, honesty 5, safety 3, unreachable 3, stop 3, alerts 3), new phrasings, no text in common with the 27-mission suite, lab4 v1 / v2, testsets, ood_v1, SFT data, voice prompts or any unit test (`check_overlap`, tested). Committed before any run; NOT run. It targets the step-B fixes (other room with nothing said before, "that room", stop + "why have you halted", keep out of room B and still reach D through C, find restricted to a room, a cone that does not exist in room B).
Start the three runs (about 5 h):
`nohup scripts/heldout_lab4_v3_runs.sh > /tmp/heldout_lab4_v3.out 2>&1 &`
then aggregate: `R=brain/runs/heldout_lab4_v3; docker compose run --rm -T -e GURDY_FINAL_SUITE=final_suite_lab4_v3 brain python3 -m brain.eval.heldout_report $R/run1 $R/run2 $R/run3`.
