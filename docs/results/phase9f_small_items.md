# Phase 9f: three small items (shield scan timeout, wrist camera for low obstacles, fetch LLM picker scaffolding)

Status 2026-11-26 (night): offline only, host venv tests, branch `phase9a-arm`. No Docker (an overnight Gazebo run was
going); main is not touched. No fetch held-out suite was created or edited.

## 1. Shield stale-scan timeout

**Problem (live test, main):** 9 short stops (< 0.3 s each). `/scan` sometimes arrived more than 0.5 s apart under
detector load, so the shield's fail-safe (`scan_timeout` 0.5 s) zeroed the command until the next scan.

**Finding:** on main `scan_timeout` is **already** a ROS parameter of `safety_shield_node` (declared, default 0.5 s).
Nothing had to be made configurable. The change adds what was missing; the default stays 0.5 s.

* `gurdy_core/gurdy_core/scan_timing.py`:
  * gap statistics, plus the **decision rule** `timeout = clamp(1.2 × p99(gap), 0.5 s, 1.0 s)`
  * **Cap 1.0 s = the Nav2 collision monitor's `source_timeout`** (nav2_params.yaml): the last safety layer must not be
    laxer than the one before it.
  * If 1.2 × p99 exceeds the cap, the timeout stays 1.0 s and the result says **fix the late scans** (detector CPU load)
    instead of raising it further.
  * Fewer than 100 gaps → keep 0.5 s and record longer.
  * `validate_timeout` for the node: invalid → 0.5 s, above the cap → 1.0 s, with a warning.
* `scripts/scan_gaps.py` (stdlib, to run tomorrow) reads **receive** times from a rosbag2 sqlite `.db3` (bag record time
  = what the shield's monotonic arrival clock sees), or a text file of times. It prints mean / p50 / p95 / p99 / max,
  the counts of gaps > 0.3 / 0.5 / 1.0 s, and the recommended timeout. MCAP is not supported: convert, or export times.
  * **Recording plan:** 10 min of `ros2 bag record /scan` with the detector + Nav2 running (the load that caused the
    stops).
* **Patch for main** (`docs/results/data/phase9f/shield_scan_timeout.patch`, applies cleanly to main's node; a test
  checks it with `git show main:`):
  * `validate_timeout` on the parameter
  * every scan gap recorded
  * each stale period's **duration logged** ("scan back after 0.27 s stale (stale periods so far: 9)"), which directly
    diagnoses the short stops
  * the status JSON gets `scan_timeout`, `scan_gap_last`, `scan_gap_max`, `stale_events`
  * The patch also adds `gurdy_core/gurdy_core/scan_timing.py` to main (the same file as here).
* Tests: `gurdy_core/tests/test_scan_timing.py`, **9 pass**:
  * rule: normal load → floor; 2 % late gaps → 1.2 × p99; heavy → cap + "FIX"; too few samples
  * parameter validation; stats
  * the tool on a synthetic rosbag2 sqlite file (two topics) and on a text log
  * the patch applies to main

## 2. Wrist RGB-D as a second low-obstacle source (Phase 9c pipeline)

* `gurdy_core/low_obstacles.py`:
  * `extract_wrist` (wrist camera pose from the arm joint states; clip 0.07–1.0 m; the RANSAC floor fit as for the body
    camera, no fixed floor height)
  * `wrist_self_capsules`: arm + gripper capsules with the finger opening from `/joint_states`, plus the **held object**
    as a capsule from the grasp centre down by its height
  * `merge` for the shield
* `low_obstacle_node` (`wrist:=false` default, needs the arm):
  * subscribes to `/wrist_camera/depth_image` + `camera_info` (frames arrive only while the triggered camera is on) and
    `/fetch/held_object` (JSON radius / height, "" = nothing)
  * publishes `/low_obstacles/wrist_marking` / `wrist_clearing`
  * merges the wrist points into `/low_obstacles/shield` while younger than `wrist_timeout` (0.5 s)
* The overlay has the two wrist sources with `sensor_frame: wrist_camera_optical_frame`, so the costmap ray-traces them
  from the right origin.
* Tests: `gurdy_core/tests/test_low_obstacles_wrist.py`, **7 pass** (synthetic wrist frames, servo_view pose):
  * floor only → nothing
  * **a 5 cm cube behind the robot (where backing up was blind) → found within 3 cm, height 5 cm ± 1 cm**
  * own rear foot in view → filtered
  * held object, tilt + noise → ≤ 3 false cells
  * merge
* **Finding: a held object is never seen as an obstacle; it blinds the camera.** The grasp centre is 4.5 cm in front of
  the lens, inside the 0.07 m near clip, so every ray inside the object's silhouette is clipped. **A held 12 cm can
  blinds ~50 % of the wrist image** (measured on the synthetic frame). While carrying, backing up is half-blind again.
  The held-object capsule is still there (tested geometrically) for objects that reach beyond the clip.

## 3. Fetch LLM picker scaffolding (like Phase 6d)

* `brain/fetch/picker.py` + `picker_prompt.txt`. The model returns constrained JSON
  `{tool: fetch|pick_up|place|hand_over|none, object, colour, room, target: user|home|room, dest_room}`. It only
  chooses; validation:
  * unknown keys or values → rejected
  * **colour, room and dest_room must literally appear in the user's words**, and rooms must exist on the map
  * if the user's words name a class, the model must agree (small/big family allowed)
  * target "user" needs a user word; "home" needs "home"
  * a place / fetch without the fields it needs → rejected
  * The user text is JSON-quoted as data.
* **Safety stays outside the model:** `FetchSession(picker=…)` calls the picker only when the router returns None and
  `might_be_fetch` (verb / wish words, not a question) is true. The **text-level rules (throw, people) run before the
  model is called**, and the object rules (classes, mass, mannequin) after it. The state machine and grounding are
  unchanged.
* Any failure (server down, bad JSON, invalid value) → None → the planner path as before.
* Tests: `brain/tests/test_fetch_picker.py`, **22 pass, FAKE scripted LLM only**:
  * a free phrasing → delivered; router phrasings never reach the model; place what is held
  * 10 invalid outputs rejected (invented colour / room, room not on the map, unknown class / tool, extra key, not JSON,
    missing fields, wrong target)
  * class must agree with the words; fenced JSON; none / questions without an LLM call
  * "yeet that red can at me" refused before the model; mannequin / barrel from the picker refused; server error →
    fallback; invented colour → "won't guess"; drop fault → no success claim; prompt content and quoting
* **Router bug found and fixed:** "bring that tall figure … to me" was read as a pronoun ("bring that to me"). `that` and
  `this` now count as pronouns only when no noun follows. The DEV suite is still 42/42; fetch unit tests pass.
* **Not done:** a real model. Tomorrow: an Ollama client for the picker (the 6d client pattern, JSON format), a dev set of
  free phrasings scored like `picker_dev`, then the user's blind suite.

## Files

| item | files |
|---|---|
| 1 | `gurdy_core/gurdy_core/scan_timing.py`, `scripts/scan_gaps.py`, `gurdy_core/tests/test_scan_timing.py`, `docs/results/data/phase9f/shield_scan_timeout.patch` |
| 2 | `gurdy_core/gurdy_core/low_obstacles.py` (wrist part), `ros2_ws/src/gurdy_navigation/gurdy_navigation/low_obstacle_node.py`, `ros2_ws/src/gurdy_navigation/config/nav2_low_obstacles_overlay.yaml`, `gurdy_core/tests/test_low_obstacles_wrist.py` |
| 3 | `brain/fetch/picker.py`, `brain/fetch/picker_prompt.txt`, `brain/fetch/session.py` (picker hook), `brain/fetch/router.py` (pronoun fix), `brain/tests/test_fetch_picker.py` |
