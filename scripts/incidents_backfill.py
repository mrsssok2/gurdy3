#!/usr/bin/env python3
"""Backfill docs/incidents/incidents.jsonl (2026-11-28): every past incident with a CONFIRMED root cause, from docs/results,
CLAUDE.md and commit messages. Evidence is never typed by hand: each spec names (file glob or commit, regex) and this script copies
the first matching line verbatim (file:line or commit hash). A pattern that matches nothing aborts the build.

  evidence_status "full"    at least one evidence line comes from a run's own log / trace (docs/results/data/**, *.log, *.jsonl)
  evidence_status "partial" only docs / commit messages / code state it (the run's logs are not in the repo)
Incidents whose cause the sources call unknown / not identified / not established are NOT included.
Re-run: python3 scripts/incidents_backfill.py [--check]    (--check: exit 1 if the committed file differs)
New incidents are appended by hand (same fields) -- see docs/incidents/README.md; this script only owns the backfilled ids.
"""
from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/incidents/incidents.jsonl"
GAZ, MUJ, MOCK, HOST, EVAL = "gazebo", "mujoco", "mock backend", "host / tooling", "eval harness"

# (id, date, env, symptom, logs [globs], root_cause, evidence [(glob | commit:HASH, regex)], fix_commit [hashes], tests [paths])
SPECS = [
 ("INC-20261114-01", "2026-11-14", MUJ, "Reward sanity: walking at half the commanded speed scored about the same as full speed.",
  [], "Tripod strides surge 0.013-0.137 m/s around 0.096 m/s; the tracking reward used the instantaneous body velocity, so the surge noise dominated the error. Fixed with an EMA-filtered velocity (tau 0.2 s).",
  [("CLAUDE.md", r"EMA-filtered \(τ 0\.2 s\)\*\*: tripod strides surge")], ["89f52cd"], ["rl/tests/test_reward.py"]),
 ("INC-20261115-01", "2026-11-15", MUJ, "Policy comparison numbers wrong: the tracking error of dr_energy_ft was reported 10.9 % (target < 10 % 'missed').",
  [], "rl/eval.py used each run's TRAINING command mix (axis-aligned / rest-start probabilities) instead of the standard distribution, so runs were evaluated on different inputs.",
  [("CLAUDE.md", r"eval bug — `rl/eval.py` used each run's training command mix"), ("commit:0e40487", r"standard command distribution")], ["0e40487"], []),
 ("INC-20261116-01", "2026-11-16", GAZ, "Nav2 + RL policy: the robot never started moving towards a goal (stall, recoveries).",
  ["docs/results/data/phase3_nav_policy.log"], "Start deadlock: MPPI plans from the measured velocity (0 at standstill) with ax_max*dt = 0.04 m/s, below the velocity_smoother deadband 0.05 m/s and the policy's dead zone, so the command never grew.",
  [("docs/results/phase3_gazebo_results.md", r"Cause: start deadlock between MPPI and the minimum speeds"), ("docs/results/data/phase3_nav_policy.log", r"'max_stall_s': 84\.0")], ["748b480"], ["ros2_ws/src/gurdy_navigation/test"]),
 ("INC-20261116-02", "2026-11-16", GAZ, "collision_monitor throttled /cmd_vel to ~5 % (FootprintApproach); Nav2 only worked with it disabled.",
  ["docs/results/data/phase3_nav_dr2_nocm.log", "docs/results/data/phase3_nav_dr2_cm_filtered.log"], "LiDAR self-hits: the femurs reach the lidar plane in 91 % of walking scans (0.15-0.29 m), so the monitor saw an obstacle inside the footprint. Fixed by scan_self_filter_node (/scan_raw -> /scan).",
  [("CLAUDE.md", r"lidar self-hits confirmed \(femurs reach the lidar plane, 91 % of walking scans")], ["7e06642"], ["ros2_ws/src/gurdy_navigation/test"]),
 ("INC-20261116-03", "2026-11-16", GAZ, "Nav2 with EKF odometry: MPPI stalled in every EKF run.",
  ["docs/results/data/phase3_explore_ekf.log"], "EKF mode never set controller_server.odom_topic, so MPPI read a non-existent /odom.",
  [("CLAUDE.md", r"EKF mode never set `controller_server\.odom_topic`")], ["3982ea1"], ["ros2_ws/src/gurdy_navigation/test"]),
 ("INC-20261116-04", "2026-11-16", GAZ, "Nav2 start-up hung forever in some runs (lifecycle_manager never finished bring-up).",
  [], "A lost DDS service response from controller_server/change_state; the lifecycle manager then waits forever and re-sending RESET/STARTUP hangs too. Fixed with a readiness wait + whole-stack restart (<= 3x).",
  [("scripts/lib_nav_start.sh", r"failed to send response \.\.\. timeout")], ["3982ea1"], []),
 ("INC-20261117-01", "2026-11-17", MOCK, "Mock world: the return from room B was reported blocked although the doorway was free.",
  [], "World.line_free sampled every 5 cm and clipped path corners against the walls.",
  [("CLAUDE.md", r"Mock bug fixed: `World\.line_free` sampled 5 cm, path corners clipped")], ["8889937"], ["brain/tests"]),
 ("INC-20261117-02", "2026-11-17", EVAL, "'why did you stop' and 'disable the emergency stop' triggered the e-stop.",
  [], "The e-stop fast-path regex matched the word 'stop' inside questions and disable requests.",
  [("CLAUDE.md", r"Bugs found \+ fixed: \"why did you stop\" and \"disable the emergency stop\" hit the e-stop fast path")], ["17af1e5"], ["brain/tests"]),
 ("INC-20261117-03", "2026-11-17", EVAL, "Agent looped: the planner kept calling tools after the results were in.",
  [], "The plan JSON schema forced at least one tool call, even after the tool results (no way to say 'done').",
  [("CLAUDE.md", r"plan schema forced a call after results")], ["ca20907"], ["brain/tests"]),
 ("INC-20261117-04", "2026-11-17", EVAL, "Safety checker crashed on some model plans.", [],
  "Malformed tool arguments from the model reached the safety checker unvalidated.",
  [("CLAUDE.md", r"safety checker crashed on malformed args")], ["ca20907"], ["brain/tests"]),
 ("INC-20261117-05", "2026-11-17", EVAL, "Every `schedule` tool call was rejected.", [],
  "The stdlib validator rejected the nested call argument of `schedule`.",
  [("CLAUDE.md", r"validator rejected all\s+`schedule` calls")], ["ca20907"], ["brain/tests"]),
 ("INC-20261117-06", "2026-11-17", HOST, "Host RAM grew ~0.3 GiB per distinct 2.5k-token prompt in Ollama.", [],
  "llama.cpp keeps a host-RAM prompt cache per unique prompt prefix; non-identical system prompts filled it. Rule: the tool block stays byte-identical.",
  [("CLAUDE.md", r"Trap: unique prompt prefixes fill the llama\.cpp host-RAM prompt cache")], [], []),
 ("INC-20261118-01", "2026-11-18", GAZ, "Live test: `find` on an already mapped area gave up at once.", [],
  "find only explored frontiers; on a mapped area there were none, so it ended without looking. Fixed: find sweeps the rooms.",
  [("CLAUDE.md", r"Fixed: `find` on a mapped area gave up \(now sweeps rooms\)")], ["1dda44b"], ["brain/tests"]),
 ("INC-20261118-02", "2026-11-18", GAZ, "Live agent crashed with a SQLite error (objects created in a thread can only be used in that same thread).", [],
  "MemoryStore's sqlite3 connection was shared between the agent thread and the pump / web threads (check_same_thread default).",
  [("CLAUDE.md", r"\(live bug: SQLite cross-thread\): `MemoryStore` is thread-safe")], ["35b94ab"], ["brain/tests/test_memory.py"]),
 ("INC-20261118-03", "2026-11-18", GAZ, "RclpyPort constructor crashed in the ROS container.", [],
  "A local `Clock` import shadowed rosgraph_msgs Clock.",
  [("commit:72c6ca1", r"local 'Clock' import shadowed the rosgraph_msgs Clock")], ["72c6ca1"], []),
 ("INC-20261118-04", "2026-11-18", HOST, "WSL crashed with a Bus error during the GGUF build.", [],
  "The C: drive (WSL's backing disk) ran full during the merge / convert. build_gguf.sh now refuses to start with < 20 GB free.",
  [("CLAUDE.md", r"a full C: crashed WSL with a Bus error")], ["08e4089"], []),
 ("INC-20261118-05", "2026-11-18", HOST, "Whisper transcribed silence as 'Thank you.'", [],
  "Whisper hallucinates on silent audio without voice-activity filtering; fixed with vad_filter=True.",
  [("CLAUDE.md", r"Whisper on SILENCE says \"Thank you\.\" → `vad_filter=True`")], ["aa6641b"], []),
 ("INC-20261118-06", "2026-11-18", HOST, "A one-word 'stop' was heard as 'Stalk' / 'Hide' and not handled as a stop.", [],
  "Whisper mis-recognises single short words; the e-stop text rule matched only exact stop words. Fixed by a fuzzy 1-3-word stop rule (stopwords.py).",
  [("CLAUDE.md", r"one-word stop heard as 'Stalk' / 'Hide'")], ["208cf13"], ["brain/tests"]),
 ("INC-20261118-07", "2026-11-18", GAZ, "Final eval: 2 missions failed at t=0 with 'not running / unknown space'.", [],
  "Nav2 was not ready yet from the runner's point of view when the first command was sent. Fixed: readiness gate (brain/ros/readiness.py) + infra retry.",
  [("CLAUDE.md", r"2 missions failed at t=0 because Nav2 was not ready from the runner's point of view")], ["6e0da2a"], ["brain/tests/test_final_eval.py"]),
 ("INC-20261118-08", "2026-11-18", EVAL, "Final eval scored nav_there_and_back FAIL although the robot arrived correctly.", [],
  "Scorer defect: one `goal` per mission judged the correct room-B arrival claim against the final goal.",
  [("CLAUDE.md", r"one `goal` per mission judges the correct room-B arrival claim as false")], ["80b7359"], ["brain/tests/test_final_eval.py"]),
 ("INC-20261118-09", "2026-11-18", GAZ, "Final eval: 'Sorry, I did not understand that.' for a correct 3-step plan (go_b_report_home).", [],
  "The model's plan contained one stray `}`; the strict JSON parser rejected the whole plan.",
  [("docs/results/phase5_final_eval_fixes.md", r"contained ONE stray `\}`; the strict JSON parser rejected it")], ["76a2c11"], ["brain/tests"]),
 ("INC-20261118-10", "2026-11-18", GAZ, "Final eval find_blue_box: ran out of time, never saw the box.", [],
  "The room sweep went to each room centre facing the travel direction and never turned; the camera cone missed objects in corners / behind.",
  [("docs/results/phase5_final_eval_fixes.md", r"The room sweep went to each room's centre facing the direction of travel and never turned")], ["76a2c11"], ["brain/tests/test_search_sweep.py"]),
 ("INC-20261118-11", "2026-11-18", GAZ, "Final eval explore_b: 'Exploring.' then 'Done.' after 11 s without moving.", [],
  "The explorer ends at once on an already mapped room (no frontiers) and the agent reported done.",
  [("docs/results/phase5_final_eval_fixes.md", r"The explorer ends at once on an already mapped room \(no frontiers\)")], ["76a2c11"], ["brain/tests"]),
 ("INC-20261118-12", "2026-11-18", GAZ, "Final eval honest_capability: 'take a photo and email it' answered with an unrelated scene description.", [],
  "The model mapped an impossible request onto the nearest tool (describe_view) and never said it cannot do it.",
  [("docs/results/phase5_final_eval_fixes.md", r"The model mapped an impossible request onto the nearest tool")], ["76a2c11"], ["brain/tests"]),
 ("INC-20261118-13", "2026-11-18", GAZ, "'go to the other box' went to the box just discussed.", [],
  "No reference resolution: the planner picked the known box in focus.",
  [("docs/results/phase5_final_eval_fixes.md", r"No reference resolution: the planner saw \"the other box\"")], ["76a2c11"], ["brain/tests"]),
 ("INC-20261118-14", "2026-11-18", MOCK, "'why did you stop' answered with an old 'I have arrived' line.", [],
  "The mock returned the last 3-5 events of any task and the model quoted the wrong one.",
  [("docs/results/phase5_final_eval_fixes.md", r"The mock returned the last 3 / 5 events of any task")], ["76a2c11"], []),
 ("INC-20261118-15", "2026-11-18", GAZ, "Final-eval stack teardown killed the eval script itself.", [],
  "`pkill -f` patterns in stop_nav_stack matched the calling shell's own command line.",
  [("CLAUDE.md", r"`stop_nav_stack` self-kill fixed \(7639828\)")], ["7639828"], []),
 ("INC-20261119-01", "2026-11-19", GAZ, "From the phone: the agent answered that the LLM was unreachable (Errno 111).", [],
  "The WSL address from `hostname -I` refused connections from containers while host.docker.internal worked; OLLAMA_URL was built from the wrong address.",
  [("CLAUDE.md", r"the WSL address from `hostname -I` \(172\.30\.x\) gave Errno 111 from containers")], ["774921a"], ["brain/tests/test_llm_unavailable.py"]),
 ("INC-20261119-02", "2026-11-19", HOST, "`python3 -m web.api` crashed at start-up.", [],
  "web/api/__main__.py imported ..voice.cli (a relative import outside the package).",
  [("CLAUDE.md", r"Fixed a startup crash \(`web/api/__main__\.py` imported `\.\.voice\.cli`\)")], ["02c79ee"], ["web/tests"]),
 ("INC-20261119-03", "2026-11-19", HOST, "The staged public demo failed to start.", [],
  "KeywordLLM imports brain.eval.keyword_parser, which was missing from the staged files.",
  [("CLAUDE.md", r"`KeywordLLM` imports `brain\.eval\.keyword_parser` \(was missing from the staged files\)")], [], ["web/tests/test_demo_stage.py"]),
 ("INC-20261122-01", "2026-11-22", GAZ, "Held-out lab4: 'go to Charlie' -> 'Did you mean chair?', no movement (3/3).", [],
  "The phonetic room-name handling needed the word 'room'; a bare 'Charlie' went to the vocabulary fuzzy matcher, which maps it to the class chair.",
  [("docs/results/phase5_heldout_lab4.md", r"needs the word \"room\" \(\"room Charlie\"\); a bare \"Charlie\" goes to the vocabulary fuzzy matcher")], ["3f67bbc"], ["brain/tests"]),
 ("INC-20261122-02", "2026-11-22", GAZ, "Held-out lab4 l4_nogo_c_route: the robot crossed the no-go room C on the way to D (2/3).", [],
  "No-go zones were discs checked when a walk starts; Nav2 has no keepout filter and re-plans by itself after deviations / recoveries, through the forbidden room.",
  [("docs/results/phase5_heldout_lab4.md", r"a no-go zone is a disc \(radius 1 m around the room centre\) checked when a walk starts"),
   ("docs/results/phase5_stepb_fixes.md", r"Nav2 re-plans a goal by itself and knows nothing about our zones")], ["3f67bbc", "9578e64"], ["brain/tests"]),
 ("INC-20261125-01", "2026-11-25", GAZ, "Incident graph: 16 false 'missing' / ghost toolbox reports.", ["docs/results/data/phase6b/ghost/*"],
  "The recorder mixed sim and wall clocks at RTF < 1 (sighting ages converted wrongly), so stale sightings looked new.",
  [("CLAUDE.md", r"ghost toolbox = recorder sim/wall clock mix at RTF<1 \(fixed\)"), ("docs/results/data/phase6b/ghost/sightings.jsonl", r"toolbox")], ["72269e2"], ["brain/tests"]),
 ("INC-20261125-02", "2026-11-25", EVAL, "Offline OOD set item o15 failed on the old code too (72.5 % instead of 75 %).", [],
  "A stale Ollama server process; a restart fixes it. Not a code bug.",
  [("CLAUDE.md", r"o15 flake = stale Ollama server \(restart fixes\)")], [], []),
 ("INC-20261126-01", "2026-11-26", GAZ, "lab4 v3: 'Charlie first, afterwards Delta' refused (3/3).", [],
  "Phase 7b rule call_not_asked_for applied to every tool, not only to sensitive ones (no_go_zone / save_place).",
  [("docs/results/phase5_heldout_lab4_v3.md", r"Refused when it should have acted \(Phase 7b rule\)")], ["7f9f6e8"], ["brain/tests"]),
 ("INC-20261126-02", "2026-11-26", GAZ, "lab4 v3: 'run over to room D' refused as unsafe (3/3).", [],
  "The unsafe-request regex 'run over' did not require an object after the harm verb.",
  [("CLAUDE.md", r"old `run over` regex refuses \"run over to room D\"")], ["7f9f6e8"], ["brain/tests"]),
 ("INC-20261126-03", "2026-11-26", GAZ, "lab4 v3: 'is there a cone in room D' -> invented place argument inside the object (3/3).", [],
  "The planner put `place` inside the object reference; no split of 'X in room Y'.",
  [("CLAUDE.md", r"planner puts `place` inside the object for \"is there a cone in room D/B\"")], ["7f9f6e8"], ["brain/tests"]),
 ("INC-20261126-04", "2026-11-26", GAZ, "lab4 v3: 'I need the toolbox located' answered 'I have not seen a toolbox' without searching.", [],
  "The planner chose where_is (memory lookup) instead of find for an unseen object.",
  [("CLAUDE.md", r"`where_is` for \"located\"")], ["7f9f6e8"], ["brain/tests"]),
 ("INC-20261126-05", "2026-11-26", EVAL, "Post-fix live check flagged a false claim of 5 m/s in a correct speed reply.", [],
  "Scorer regex `5 (meters|metres) per second` matched the tail of '0.15 metres per second'.",
  [("docs/results/phase7d_postfix_live.md", r"it matches the tail of \"0\.1\*\*5 metres per second\*\*\"")], [], []),
 ("INC-20261126-06", "2026-11-26", GAZ, "Nav2 'path blocked' aborts in lab4 (5/5 lab4 + 6/6 final-eval aborts), no recovery tried.", [],
  "BT goal-ack timeout: default_server_timeout 20 ms; every abort was a goal-ack timeout that skips all recoveries. Raised to 1000 ms.",
  [("CLAUDE.md", r"Nav2 \"path blocked\" in lab4 = BT goal-ack timeout \(`default_server_timeout` 20 ms")], ["480bf7f"], []),
 ("INC-20261127-01", "2026-11-27", HOST, "gurdy_navigation footprint tests failed on the host.", [],
  "numpy 2 removed 2-D np.cross, used by the footprint hull.",
  [("commit:4a7375d", r"2-D np\.cross was removed")], ["4a7375d"], ["ros2_ws/src/gurdy_navigation/test"]),
 ("INC-20261127-02", "2026-11-27", GAZ, "The wrist camera streamed at 7.1 Hz always, `<triggered>` ignored.", ["docs/results/data/day_0927/*"],
  "`<triggered>` / `<trigger_topic>` were children of `<sensor>`; SDFormat wants them under `<camera>`.",
  [("docs/results/day_0927_plan.md", r"Root cause of \"always on\"")], ["05d8cf3"], []),
 ("INC-20261127-03", "2026-11-27", HOST, "day_0927 step 02 failed test_patch_applies_to_main.", [],
  "The 9f shield scan-timeout patch was made on main and did not stack on the 9c depth patch already applied in the integration tree.",
  [("CLAUDE.md", r"02 FAIL only `test_patch_applies_to_main` \(9f patch must be regenerated on top of 9c\)")], ["c234906"], []),
 ("INC-20261127-04", "2026-11-27", GAZ, "Fetch in Gazebo: the object search stalled after its first Nav2 step (2/2).", ["docs/results/data/fetch_dry_run_1127/*"],
  "RosFetchRobot ticked the incident-recorder WRAPPER, whose tick() never ticks RosBackend; the agent pump that normally ticks it was blocked by the agent lock during the fetch.",
  [("docs/results/phase9j_fetch_dry_run.md", r"Finding 1, root cause FOUND \(not an rclpy executor deadlock\)"), ("docs/results/data/fetch_dry_run_1127/agent.log", r"Found the blue can")],
  ["618ada9"], ["brain/tests/test_ros_fetch.py"]),
 ("INC-20261127-05", "2026-11-27", GAZ, "Fetch retest C try 2 never started (nav start gate).", ["docs/results/data/fetch_retest/*"],
  "lib_nav_start.sh waited for exactly 2 active controllers; with arm:=true there are 4 (race).",
  [("docs/results/data/fetch_retest/**/nav_ready.txt", r"controllers active=0 after 120 s wall"),
   ("CLAUDE.md", r"`lib_nav_start\.sh` waited for exactly 2 active controllers, arm:=true has 4 \(race\)")], ["015a749"], []),
 ("INC-20261127-06", "2026-11-27", GAZ, "object3d: the 0.30 m red box was located 6.5 cm off.", [],
  "The fixed 0.40 m rectangle prior for the generic `box` class; replaced by a size-adaptive fit over the training range 0.25-0.50 m.",
  [("CLAUDE.md", r"the fixed 0\.40 prior put the 0\.30 red box 6\.5 cm off")], ["ec94602"], ["gurdy_core/tests"]),
 ("INC-20261127-07", "2026-11-27", GAZ, "Grasp test: the can was knocked over by the arm swing (2/5).", [],
  "The target object was not in MoveIt's planning scene during the big arm moves.",
  [("commit:f15f562", r"target object in MoveIt's scene for the big moves \(can knocked by the arm swing in 2/5\)")], ["f15f562"], []),
 ("INC-20261128-01", "2026-11-28", GAZ, "Fetch: every mini cone was named 'white mini cone' (target orange / green): not found 3/3.", ["docs/results/data/overnight_1127/fetch/*/agent.log"],
  "Colour rule = median of the middle 50 % of the box; for the narrow cone that crop is its white band + background (white 85 % offline on 135 boxes).",
  [("docs/results/overnight_1127.md", r"\*mini_cone 3/3 = colour naming \(confirmed\)\.\*"), ("docs/results/data/overnight_1127/fetch/*/agent.log", r"white mini cone")], ["e648a32"], ["vision/tests"]),
 ("INC-20261128-02", "2026-11-28", GAZ, "Mini cones rendered WHITE in Gazebo whatever their SDF colour (the detector was trained on white cones).", ["docs/results/data/cone_check/*"],
  "The lathe-cone OBJ had no vertex normals; Gazebo drew it white regardless of the material.",
  [("docs/results/cone_check.md", r"\*\*Cause of the white cones:\*\* the lathe-cone OBJ had no vertex normals"),
   ("docs/results/data/cone_check/recall_det_v31_s.md", r"\| all \| 164 \| 149 \| 11 \| 4 \| 10\.4 %")], ["b511e7d"], ["vision/tests/test_small_objects.py"]),
 ("INC-20261128-03", "2026-11-28", GAZ, "Fetch reply printed 'residual 99 cm' although nothing was measured.", ["docs/results/data/overnight_1127/run.log"],
  "fine_align returned a 0.99 m placeholder when the wrist camera never measured the object, and the reply template printed it.",
  [("docs/results/data/overnight_1127/run.log", r"residual 99 cm after 3 tries"), ("commit:ed67031", r"no made-up alignment residual")], ["ed67031"], ["brain/tests/test_reply_numbers.py"]),
 ("INC-20261128-04", "2026-11-28", GAZ, "Grasp closed on the object, then it slipped during the lift (4/13 fetch failures).", [],
  "Independent fingers without centring: both stalled at the object width, then 40/10 mm during the lift. Fixed with a coupled gripper (one-servo rack).",
  [("CLAUDE.md", r"grasp slip 4/13 \(fingers centred at the object width, then 40/10 mm during the lift: independent fingers, no centring\)")], ["75bd1ea"], []),
 ("INC-20261128-05", "2026-11-28", GAZ, "Fetch: 'I can't confirm I'm holding it' -- the verify step reported no wrist frame.", ["docs/results/data/smoke_1128*/**/attempts.jsonl"],
  "Not a missing frame: every wrist view returned in 0.02-0.05 s; the lift check returned None because the HELD can fills the wrist view (no floor / standing points).",
  [("docs/results/data/smoke_1128c/**/attempts.jsonl", r"no wrist frame for the lift check\".*\"wrist_views\": \[\[true"),
   ("commit:ef767d5", r"the 'no wrist frame' was wrong")], ["ef767d5"], []),
 ("INC-20261128-06", "2026-11-28", GAZ, "Cube fetch: the red cube was seen 11 times but never became a confirmed track.", ["docs/results/data/smoke_1128b/cube_sightings.txt"],
  "Unconfirmed-track memory (ttl) 10 s: the cube was seen in two short scan passes 29 s apart (6 + 5 sightings), below min_hits 8 within the ttl.",
  [("docs/results/data/smoke_1128b/cube_sightings.txt", r"within 0\.4 m of the cube: 11"), ("commit:6ee44fc", r"unconfirmed tracks are kept 30 s \(was 10\)")], ["6ee44fc"], []),
 ("INC-20261128-07", "2026-11-28", GAZ, "GUI demo take 1: 'pre-grasp move failed' x3, the arm never reached the pre-grasp.", ["docs/results/data/demo_bring_1128/*"],
  "MoveIt could not plan the pre-grasp goal and the direct-trajectory fallback waited in WALL time (seconds + 10) = ~2.6 s sim at RTF 0.2, shorter than the 3 s trajectory.",
  [("docs/results/data/demo_bring_1128/run.log", r"couldn't get a grip on the blue can after 3 attempts"),
   ("docs/results/demo_bring_1128.md", r"the direct fallback waited in WALL time")], ["931cc05"], ["brain/tests"]),
 ("INC-20261128-08", "2026-11-28", GAZ, "Bring-me headless take 1: 'I dropped the blue can on the way' while GT had it in the gripper.", ["docs/results/data/demo_bring_1128_headless/*"],
  "The carry drop check compared the object's GT xy with the localisation pose while walking. Fixed: dropped = the carried model is on the floor.",
  [("docs/results/data/demo_bring_1128_headless/run.log", r"I dropped the blue can on the way"), ("docs/results/demo_bring_1128.md", r"the carry drop check compared GT xy with the localisation pose")], ["720a741"], ["brain/tests"]),
 ("INC-20261128-09", "2026-11-28", GAZ, "Bring-me: the wrist alignment followed 'objects' 9-11 cm off the can (3 failed traces).", [],
  "The wrist blob chosen was the one nearest the image centre, not the one nearest the body camera's map prior.",
  [("commit:a9ad32c", r"the 3 failed traces followed 'objects' 9-11 cm off the can")], ["a9ad32c"], ["brain/tests"]),
 ("INC-20261128-10", "2026-11-28", GAZ, "GUI take 3: 'couldn't line my gripper up (residual 18 cm)' with correct wrist estimates.", ["docs/results/data/demo_bring_1128_gui3/*"],
  "The can was 0.59 m behind after the approach: Nav2 stops up to 12 cm short of the stand-off goal and the 180-degree turn walks the body 10-17 cm away. Fixed by a re-approach.",
  [("docs/results/data/demo_bring_1128_gui3/result.json", r"residual 18 cm after 3 tries"), ("docs/results/demo_bring_1128.md", r"Nav2 stops up to its\s*$")], ["119d46e", "077688a"], ["brain/tests"]),
 ("INC-20261128-11", "2026-11-28", GAZ, "Wrist views judged 'implausible' (measured can height 3-6 cm instead of 12 cm) -> extra re-looks.", [],
  "Looking down, the height came from the side-view estimate; the measured top above the fitted floor is the right height.",
  [("commit:91475ac", r"est\.height read 0\.8-6\.3 cm in some")], ["91475ac"], ["brain/tests"]),
 ("INC-20261128-12", "2026-11-28", HOST, "Two sessions' Gazebo batches interleaved on the shared machine.", [],
  "`docker ps` empty was used as the lock; per-try batches of two sessions started in each other's gaps. Fixed with a shared flock (~/projects/.gpu_lock) held for the whole batch.",
  [("CLAUDE.md", r"`docker ps` empty is NOT a lock: per-try batches of two sessions interleaved in the gaps between stacks")], ["980b85c"], ["brain/tests/test_gpu_lock.py"]),
 ("INC-20261128-13", "2026-11-28", HOST, "test_yolo_block_runs_all_stages broke when the runtime detector default changed.", [],
  "The overnight block-d check compared against a hard-coded det_v3_s instead of the default before vs after.",
  [("commit:0516326", r"instead of a hard-coded det_v3_s")], ["0516326"], ["brain/tests"]),
 ("INC-20261128-14", "2026-11-28", MOCK, "VLN: a walk into the kitchen was refused as unreachable (chair next to the room centre).", [],
  "An 'enter room' waypoint was the room centre, used without a free-space check.",
  [("docs/results/vln_plan.md", r"Room entry point = the free point closest to the centre \(the centre itself was used unchecked")], ["eec9c77"], ["brain/tests/test_vln_route.py"]),
 ("INC-20261128-15", "2026-11-28", MOCK, "VLN: with landmark sigma 0.10 m a 0.6 m stand-off could really be 0.3 m from the object.", [],
  "The sigma term of the clearance was applied to walls and other objects but not to the distance from the landmark itself.",
  [("docs/results/vln_plan.md", r"Points now also keep `0\.4 \+ margin \+ 3σ` from \*\*their own landmark\*\*")], ["eec9c77"], ["brain/tests/test_vln_route.py"]),
 ("INC-20261128-16", "2026-11-28", HOST, "test_final_eval_wall_timeout timed out (120 s) and left 5 orphaned flock waiters.", [],
  "p5_final_eval.sh takes the shared GPU lock; it was held by another project, so the stub test waited until its subprocess timeout; the killed scripts' flock holders stayed queued.",
  [("commit:2341ca1", r"test_final_eval_wall_timeout left out: it takes the real GPU lock|it takes the\s*$")], [], []),
]

LOGISH = re.compile(r"(^docs/results/data/|\.log$|\.jsonl$)")


def _files(glob):
    if any(c in glob for c in "*?["):
        return sorted(p for p in ROOT.glob(glob) if p.is_file())
    p = ROOT / glob
    return [p] if p.is_file() else sorted(q for q in p.rglob("*") if q.is_file()) if p.is_dir() else []


def _window(line: str, m, width: int = 360) -> str:
    """The matching part of a (possibly very long) line, verbatim, with up to `width` characters of context; '...' marks cuts."""
    line = line.strip()
    if len(line) <= width:
        return line
    a = max(0, m.start() - (width - (m.end() - m.start())) // 3)
    b = min(len(line), a + max(width, m.end() - m.start() + 40))
    return ("..." if a > 0 else "") + line[a:b] + ("..." if b < len(line) else "")


def _evidence(src, rx):
    if src.startswith("commit:"):
        h = src.split(":", 1)[1]
        msg = subprocess.run(["git", "-C", str(ROOT), "log", "-1", "--format=%B", h], capture_output=True, text=True).stdout
        for line in msg.splitlines():
            m = re.search(rx, line)
            if m:
                return {"commit": h, "text": _window(line, m)}
        return None
    for f in _files(src):
        try:
            for i, line in enumerate(f.read_text(errors="replace").splitlines(), 1):
                m = re.search(rx, line)
                if m:
                    return {"file": str(f.relative_to(ROOT)), "line": i, "text": _window(line, m)}
        except OSError:
            continue
    return None


def build() -> list:
    out, errs = [], []
    for (iid, date, env, symptom, logs, cause, ev, fixes, tests) in SPECS:
        items = []
        for src, rx in ev:
            e = _evidence(src, rx)
            if e is None:
                errs.append(f"{iid}: no line matches {rx!r} in {src}")
            else:
                items.append(e)
        log_paths = sorted({str(f.relative_to(ROOT)) for g in logs for f in _files(g)})
        for h in fixes:
            if subprocess.run(["git", "-C", str(ROOT), "cat-file", "-e", f"{h}^{{commit}}"], capture_output=True).returncode:
                errs.append(f"{iid}: fix commit {h} not found")
        for t in tests:
            if not (ROOT / t).exists():
                errs.append(f"{iid}: test path {t} missing")
        full = any("file" in e and LOGISH.search(e["file"]) for e in items) and bool(log_paths)
        out.append({"id": iid, "date": date, "robot": "gurdy hexapod (simulation)" if env in (GAZ, MUJ) else "gurdy agent / tooling",
                    "env": env, "symptom": symptom, "logs": log_paths[:20] + ([f"... {len(log_paths) - 20} more"] if len(log_paths) > 20 else []),
                    "root_cause": cause, "evidence": items, "evidence_status": "full" if full else "partial",
                    "fix_commit": fixes or None, "test_added": tests or None, "source": "backfill 2026-11-28"})
    if errs:
        raise SystemExit("\n".join(errs))
    ids = [r["id"] for r in out]
    assert len(ids) == len(set(ids)), "duplicate ids"
    return out


def main():
    rows = build()
    text = "".join(json.dumps(r, ensure_ascii=False) + "\n" for r in rows)
    if "--check" in sys.argv:
        cur = OUT.read_text() if OUT.exists() else ""
        mine = "".join(line + "\n" for line in cur.splitlines() if json.loads(line).get("source") == "backfill 2026-11-28")
        sys.exit(0 if mine == text else 1)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    keep = [line for line in (OUT.read_text().splitlines() if OUT.exists() else []) if json.loads(line).get("source") != "backfill 2026-11-28"]
    OUT.write_text(text + "".join(k + "\n" for k in keep))
    print(f"{len(rows)} backfilled incidents ({sum(r['evidence_status'] == 'full' for r in rows)} full, "
          f"{sum(r['evidence_status'] == 'partial' for r in rows)} partial) + {len(keep)} appended -> {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
