# VLN plan: vision-and-language navigation (design + plan validator; written 2026-11-27)

Branch `phase10-vln-prep` (from `phase8c-wire`). The ROADMAP calls this "Phase 5b VLN"; the user's brief calls it Phase 9
VLN. Nothing here moves the robot yet: this is the design, the plan parser / validator with a fake LLM, the metrics
code, and a DEV set. Example: "go past the cone into the room with the red barrel".

## 1. What exists today (read first)

| piece | how it works now | what VLN uses |
|---|---|---|
| Rooms / places | `maps/<world>.places.yaml`: rooms as rectangles, waypoints (`home`); RosBackend `_place_goals` picks a free point near the room centre. lab4 = a ring of 4 rooms A-B-D-C-A, 1 m doorways at A-B (2, −0.7), B-D (4, 2), D-C (2, 4.2), C-A (−0.7, 2) (`lab4_gt.json`) | room rectangles + the door graph (`brain/vln/world.py`) |
| Navigation | RosBackend tasks with steps (nav / rotate / walk / explore), Nav2 `navigate_to_pose` + `compute_path_to_pose`, path check against no-go zones (`_path_crosses_zone`, `_detour` via another room centre), goal tolerance 0.12 m, "close enough" rules, readiness gate | waypoint legs = `go_to(point)`; avoided rooms = temporary no-go zones |
| Semantic map | `detector_node` → `SemanticMap` tracks (class, colour, x, y, hits, confirmation, dominance rule). With `object3d:=true` (Phase 8c): fused 3D centre, top / height / size / yaw / sigma. JSON on `/semantic_map/objects` | landmarks = confirmed tracks; 3D size for clearance; sigma for "is it where I remember" |
| Incident graph (Phase 6) | SQLite temporal graph of sightings / changes, 6 query tools (what_changed, last_seen, object_history, timeline, why, unchecked_rooms) behind deterministic routing + a validated LLM picker (6d) | landmark history: a landmark that moved since the instruction was written; "the room where the toolbox was this morning" (later) |
| Agent tools | 24 tools (go_to, find, explore, patrol, relation, path_check, where_is, ...), LangGraph agent, safety rules, grounding check (every stated fact must come from a tool result), template replies | VLN becomes ONE new route in front of the planner, like `_incident_route` / `_fetch_route` |
| Pattern to copy | the 6d incident picker: cheap pre-filter → one LLM call → strict JSON → code validates every value against the user's words and the map → `None` = the normal planner handles the text | the VLN planner (`brain/vln/planner.py`) |

## 2. Instruction types (all in the DEV set)

| type | example | plan |
|---|---|---|
| object goal | "go to the red barrel" | `stop` at an object (stand-off 0.6 m on the approach side) |
| room by contents | "go to the room with the red barrel" | `enter` `room_with` → the room of the (unique) match |
| sequential | "go to room B, then room D" | several steps in order |
| landmark route | "go past the blue box into room D", "go through room B to the red barrel" | `pass` steps (an object, or a room) before the goal |
| relational | "stop to the left of the mannequin", "between the mannequin and the toolbox", "behind / in front of / next to X" | `stop` at a `relation` point |
| negation / constraint | "go to room D but avoid room B", "without going through room C", "stay out of room B" | `avoid` rooms |
| failures | "go to the chair" (none seen), "go to the cone" (two), "room E", "avoid B and C to reach D" | answered, nothing moves |

## 3. Instruction → plan (`brain/vln/plan.py`, `planner.py`, `prompt.txt`)

1. **Pre-filter.** A navigation verb or preposition (go, walk, head, past, through, into, towards, left of, between,
   avoid, ...) and not a question; otherwise no LLM call.
2. **One LLM call** (project Ollama, gurdy-sft-400 by default, JSON mode): the constrained JSON
   ```json
   {"kind": "go", "steps": [{"action": "pass", "target": {"object": {"class": "traffic_cone", "in_room": "room A"}}},
                            {"action": "enter", "target": {"room_with": {"class": "barrel", "color": "red"}}}],
    "avoid": []}
   ```
   * `action`: pass / enter / stop; 1–6 steps; the last is enter or stop; stop only at the end.
   * `target`: one of `{room}`, `{object: REF}`, `{room_with: REF}`, `{relation, of: [REF] | [REF, REF]}`.
   * `REF`: `{class, color?, in_room?}`, over the detector's 7 classes.
3. **Validation**, the model is not trusted. A rejection returns `None` and the normal planner handles the text:
   * **Schema:** strict keys and enums; unknown relations, classes or keys are rejected.
   * **Words:** every class must be named by the user, every colour must be said right before it, and every room must be
     said. If the model drops a colour the user said, the user's colour is used (the code trusts the words). An
     invented colour, room or object → `UNSAID`.
   * **Avoid** must equal the rooms the user's words exclude (regex over avoid / don't go through / stay out of / without
     going through / ...). A dropped constraint (`AVOID_DROPPED`) or an added one (`AVOID_ADDED`) rejects the plan. A
     lost negation is the most dangerous VLN error, so code enforces it, not the model.
4. **Grounding and geometry** (code; see 4). Answered errors carry a template reply; nothing moves.
5. **Output:**
   * waypoints `[{x, y, kind: pass | door | enter | stop, room, label, landmark}]`, with the door points included
   * the goal region (a room rectangle or a point + 0.5 m), the room sequence, avoid, a length estimate
   * progress checks, and a one-line summary for the confirmation / reply: "through door A-B; go past the blue box in
     room B; through door B-D; into room D"

## 4. Landmark grounding

* **Source.** A landmark is a confirmed semantic-map track: class, colour, and an optional room (`in_room`) as a filter.
  0 matches → `LANDMARK_NOT_FOUND` ("I haven't seen the chair yet, so I can't use it as a landmark. Shall I explore
  first?"). More than one → `AMBIGUOUS` with the options ("the traffic cone in room A or the traffic cone in room D").
  The model never picks between candidates.
* **Room by contents.** The room of the unique match. Matches in several rooms → `AMBIGUOUS` (which room).
* **Points are computed by code.**
  * `stop` at an object: 0.6 m in front of it, seen from the entry.
  * `pass`: a free point 0.6–1.0 m round the landmark; the whole-route length picks the side.
  * Relations are seen from where the robot ENTERS the landmark's room (the last door of the shortest route): left /
    right = the walker's left / right, behind / in front = beyond / before along that line, between = the midpoint,
    next to = the near side (then the other sides).
  * Relations are judged on the shortest way in only. The planner does not detour to make "left of" fit.
  * Every point must be free: ≥ 0.35 m inside the room and ≥ 0.4 m from objects. Otherwise `NO_FREE_SPOT`, e.g. the left
    of the blue barrel in lab4 is a 0.47 m gap to the wall.
* **Routing.** All simple room sequences through the door graph that avoid the avoided rooms; the geometrically
  shortest wins (start → doors → point → next anchor).
  * A goal or landmark inside an avoided room → `IMPOSSIBLE` ("You asked me to avoid room B, but the blue barrel is in
    room B").
  * No route → `IMPOSSIBLE`.
  * The start room is exempt ("I'm in room A now; I'll leave it the shortest way").
* **3D (object3d mode).** For later: footprint from `size`, plus a stand-off scaled to the measured width (now a fixed
  0.4 m object margin), and sigma: a landmark with sigma > 0.3 m is "not reliable", so ask or look first.
* **Incident graph.** For later: if `object_history` says the landmark moved after the user last saw it, say so ("The red
  barrel was moved to room B at 10:40; I'll use where it is now").

## 5. Execution and progress checking while walking (design; built on the MOCK 2026-11-27, see section 11)

* **Legs.** Each waypoint is a RosBackend `go_to(point=…)` leg (the existing task machinery: Nav2, close-enough, e-stop,
  failure replies). The avoided rooms are added as TEMPORARY no-go zones for the episode (`no_go_zone add` with an
  episode tag, removed at the end), so Nav2's path check and `_detour` keep the robot out. A leg whose planned path
  crosses an avoided room is never started.
* **Checks** (`Plan.checks`), evaluated on the robot's pose (TF) at 2 Hz:
  * `room_order`: the rooms visited match the plan's sequence. An unexpected room → stop, re-plan once from the current
    pose, else report.
  * `never_in`: entering an avoided room → immediate stop + a report. This should never happen with the zones; it is the
    safety net.
  * `pass_near`: the path came within 1.2 m of each passed landmark; the leg is not done until then.
  * **Landmark re-observation.** On approach (≤ 3 m, in the camera cone), the detector must see the landmark within
    max(0.5 m, 2σ) of the map position. Not seen → one look-around (rotate), then "The red barrel isn't where I
    remembered it" and stop. Seen elsewhere → re-ground and re-compute the points once.
* **Arrival claim.** Only from the final leg's Nav2 success plus the goal check on the pose (room rectangle / 0.5 m).
  The reply is a template ("I'm in room D, next to the red barrel") built from the plan labels and the measured pose
  (grounding check as for every reply).
* **Timeouts.** Per leg: the Nav2 timeout plus a progress watchdog (no 0.5 m progress in 30 s → stop and report).

## 6. Failure replies (templates in `plan.py`; reached without moving)

| code | reply (example) |
|---|---|
| LANDMARK_NOT_FOUND | "I haven't seen the green barrel yet, so I can't use it as a landmark. Shall I explore first?" |
| AMBIGUOUS | "I know more than one box: the box in room A or the blue box in room B or .... Which one?" / "... in more than one room: room A and room D. Which room?" |
| UNKNOWN_ROOM | "There is no room E on this map. The rooms are room A, room B, room C, room D." |
| IMPOSSIBLE | "You asked me to avoid room B, but the blue barrel is in room B." / "I can't reach room D without going through room B or room C." |
| NO_FREE_SPOT | "There is no free spot left of the blue barrel in room B where I fit." |
| rejected (UNSAID / AVOID_* / BAD_*) | none: the normal planner handles the text (as with the 6d picker) |

While walking (section 5): "The red barrel isn't where I remembered it"; "I ended up in room C instead of room B; going
back"; "I stopped: I was about to enter room B, which you asked me to avoid".

## 7. Metrics and how to compute them from Gazebo ground truth (`brain/vln/metrics.py`, tested)

* **Inputs per episode:**
  * the GT trajectory (`/odom/ground_truth`, 10 Hz)
  * the GOAL from the world's ground truth (`lab4_gt.json` objects + the places rectangles), recomputed with the same
    geometry code. Not from the semantic map: otherwise a wrongly grounded landmark scores as a success.
  * the reference length: Nav2 `compute_path_to_pose` on the GT map through the GT waypoints, around the avoided rooms;
    for now the validator's door-point estimate
  * the robot's replies (claims)
* **Success rate (SR).** The final pose is in the goal region (room rectangle, or ≤ 0.5 m of the point), no avoided room
  was entered (start room exempt until left), every passed landmark came within 1.2 m, and the robot SAID it arrived.
* **SPL** = (1/N) Σ Sᵢ · ℓᵢ / max(pᵢ, ℓᵢ): ℓ = the reference length, p = the travelled GT path length.
* **Oracle success.** Some pose of the trajectory was in the goal region (the robot passed it but stopped elsewhere).
* **Wrong-room rate.** The final pose is in a room other than the goal's room.
* **Avoid-violation rate.** Any pose inside an avoided room (reported separately: a safety metric).
* **False claims** (count): "arrived" / "I'm in room X" / "passed the X" that the GT contradicts. Target 0, like every
  other suite.
* **Answer accuracy** (expected-failure instructions): the right kind of answer (not found / ambiguous / impossible /
  unknown room / no free spot) and the robot moved ≤ 0.3 m.
* **Protocol** (to build with the user's blind suite):
  * fresh stack per instruction, as `p5_final_eval.sh` does
  * GT odometry, then a SLAM / AMCL repeat
  * 3 runs for variance
  * episodes from the user's held-out file only
  * the DEV set below is NOT used for reporting.

## 8. DEV set and offline results

`brain/eval/vln_dev.py`: 45 instructions, NOT held-out (written by the assistant while building this; the user writes
the blind suite). lab4 with all 10 big GT objects known, robot at home (0, 0) in room A.

| type | cases |
|---|---|
| object goal | 5 |
| room with X | 6 (1 ambiguous, 1 with avoid) |
| sequential | 4 |
| past / through | 6 (1 ambiguous) |
| relational | 8 (1 ambiguous, 1 honest no-free-spot) |
| negation | 7 (2 impossible, 1 start-room avoid) |
| failure | 5 |
| not nav | 4 |

* **Expected room sequences are the geometric shortest routes.** I checked them by computing both ways round the ring:
  A-B-D beats A-C-D by 0.13–1.9 m for most room-D goals. My first guesses were wrong.
* `python3 -m brain.eval.vln_dev --oracle` (each case's reference plan as the model): **45 / 45**. This checks the
  validator and the expectations, not a model.
* Not run yet: `python3 -m brain.eval.vln_dev [--model gurdy-sft-400 --model qwen3:1.7b] [--out FILE]` (needs Ollama).
* Tests: `brain/tests/test_vln.py`, 42:
  * schema rejections; unsaid class / colour / room
  * avoid dropped / added; the user's colour wins
  * not found / ambiguous / in-room filter
  * left vs right = the walker's side; between = the midpoint; behind vs in front; no free spot; pass within 1.2 m
  * door order; avoid reroutes; impossible; start room exempt; robot in a doorway
  * planner pre-filter / failure / reject / answered; prompt; DEV disjoint from the prompt examples; oracle 45/45
  * metrics: SR / SPL / oracle / wrong room / avoid violation (start exempt) / missed landmark / false claims / answers

## 9. Next steps (order)

1. Run the DEV set against gurdy-sft-400 and base qwen3:1.7b (offline, Ollama). If the SFT model fails the format,
   consider few-shot or schema mode, or train VLN examples into SFT v3 (data generator family), never using the blind
   suite.
2. `Agent._vln_route` in front of the planner, behind a flag. Execution = RosBackend legs + temporary no-go zones +
   the progress monitor (section 5), mock backend first, then FakePort.
3. A Gazebo VLN runner on the user's blind suite (fresh stack per episode, GT trajectory recorder, metrics.py) and
   ablations: GT odom vs AMCL, landmark from GT vs semantic map.
4. Later: 3D sizes for stand-offs, the incident-graph history for moved landmarks, a VLM for unnamed landmarks ("the
   shiny thing"), multi-sentence instructions.

## 10. Honest limits

* The world model is lab4-like: rectangular rooms joined by doors, routes enumerated (fine for 4 rooms, not for big
  maps).
* Relations are 2D and egocentric to the entry direction; "left of the barrel" from another viewpoint differs, and the
  plan says which one it used.
* "Past X" is approximated as "come within 1.2 m"; there is no check that the robot really went BEYOND it.
* The validator trusts the semantic map: a false track (a table seen as a box, Phase 4b) can become a landmark. The
  re-observation check in section 5 is the guard.
* Only the 7 detector classes can be landmarks: no walls, doors-as-landmarks ("through the door on the left"), or
  colours of walls.

## 11. Execution on the mock backend (built 2026-11-27, `brain/vln/executor.py`)

**Flag, default off.** `Agent(..., vln=VLNPlanner(...))` (`vln=None` = exactly the old behaviour: tested);
`python3 -m brain.agent.cli --vln [--vln-model M]` = the lab4 mock world with VLN (needs Ollama; not run). The route
`Agent._vln_route` comes AFTER the stop fast path, the untrusted-text rules and the unsafe-request check, and only
when no confirmation is pending and the backend is idle. Not navigation / rejected → `None` → the planner as before;
an answered plan error → its reply, nothing moves.

**How a walk runs.** Blocking, like the ROS fetch: the executor advances the mock clock in 0.5 s steps and checks on
the robot's REAL pose (the backend's own x / y), not on the plan.

| requirement | implementation | test (`brain/tests/test_vln_exec.py`) |
|---|---|---|
| avoided rooms = temporary no-go zones | `MockBackend.add_temp_zone(rect, tag)` / `remove_temp_zones(tag)`: RECTANGLE zones (new; the mock planner and the walk check now accept rectangles besides circles); one tag per walk, removed in `finally` whatever happens. An avoided START room gets its zone only once the robot is out of it (otherwise the robot could not move) | zone = the room B rectangle, trajectory never inside B, zones empty afterwards; start-room zone added only in room C |
| room order | the rooms entered (pose → rectangle) must follow `plan.rooms`. Another room → stop, ONE re-plan from the current pose (same model output, re-validated), else "I ended up in room C instead of room B." An avoided room → immediate stop "I stopped: I was in room B, which you asked me to avoid." (safety net behind the zones) | a plan with a wrong room list re-plans once and arrives; zones disabled → NEVER_IN |
| "pass" landmarks within 1.2 m | min distance robot → landmark (map position) during the pass leg; over 1.2 m → the walk stops: "I couldn't get within 1.2 m of ..." | blue box passed within 1.2 m (GT positions); a far pass waypoint → PASS_MISSED |
| re-observation before claiming arrival | the goal landmark (stop at / relation to an object, and the defining object of "the room with X") must be SEEN during the last leg, within 0.5 m of the map position used for the plan; not seen → one turn towards it, second look | moved barrel → "The red barrel ... isn't where I remembered it: I see it 1.1 m from there", gone mannequin / gone red barrel (room-with goal) → "I couldn't see ... so I can't say I'm there" |
| arrival only after checking the real pose | `metrics.in_goal(pose, plan.goal)` (room rectangle / 0.5 m) before any "I'm in / at"; claims are recorded (arrived, in_room, passed) for the metrics | goal region not where the legs end → NOT_AT_GOAL, no claim |
| honest failure replies | LEG_REFUSED / LEG_FAILED (blocked doorway: "I couldn't get to door A-B ..."), TIMEOUT, PASS_MISSED, WRONG_ROOM, NEVER_IN, NOT_AT_GOAL, LANDMARK_MOVED, LANDMARK_NOT_CONFIRMED, NO_ZONES (backend cannot keep out of rooms → the walk is refused); alerts during the walk (battery, legs) are appended ("Also: ...") | blocked doorway, answers without moving (cone, chair, impossible avoid) |
| e-stop at any time | every step checks the backend's e-stop flag (mock `estop`, RosBackend `p.estop_active()`) AND whether the running leg was cancelled by someone else (`AgentHost.estop` calls the stop tool without the agent lock) → "I stopped (emergency stop / you stopped me). I'm in room X.", zones removed, no claims, nothing moves afterwards | both ways, mid-walk |
| no fake announcements | the legs' task events are consumed by the executor, so the agent's pump does not announce "arrived" per leg afterwards | `pump()` after a walk |

**DEV set end to end on the mock** (`python3 -m brain.eval.vln_dev --mock --oracle`; fresh lab4 mock per case, robot at
home, all 10 objects already in the map, the oracle plan as the model; metrics against GROUND TRUTH: goal recomputed
from the true object positions): **45 / 45**. 30 navigation episodes: success rate 1.0, SPL 0.996, oracle success 1.0,
wrong-room 0, avoid violations 0, false claims 0; 11 answer cases: accuracy 1.0; 4 not-nav: not routed. 1.4 s wall for
all 45 (virtual clock).

**Finding (r01, knife edge) — addressed in section 12 (sigma in the clearance).** "Stop to the left of the blue barrel" is NO_FREE_SPOT with the true positions (0.47 m
gap), but the mock's map has 4 cm observation noise, and with it the gap comes out just wide enough: the robot walks
there and arrives (the spot IS free in the mock's true occupancy, which uses a smaller robot inflation than the VLN
margins). Scored as correct only because of that check. Lesson for Gazebo: free-spot decisions within the map's sigma
of the margin flip; add sigma (object3d `sigma_xyz`) to the clearance before the real run, or ask the user.

**Suites:** brain 1162 passed / 3 skipped (was 1142 + 20 new), fetch DEV 42/42, web 118 OK, keyword baseline v1 57.0 %
(unchanged).

**Not done / limits of this step:**
* The mock is kinematic: no Nav2 behaviour, no detector misses, exact goals. Gazebo will show leg failures, missed
  re-observations and doorway congestion the mock cannot.
* RosBackend: no `add_temp_zone` yet (the executor refuses avoid-walks there, NO_ZONES), no `advance` (it would sleep
  and tick), `VLNWorld.from_backend` only for the mock (needs the places rectangles + doors + semantic map).
* Blocking: while walking, the agent answers nothing but the e-stop (same as the ROS fetch).
* One re-plan only; the re-plan reuses the model's first output (no second LLM call).

## 12. Clearance with position uncertainty + VLN on the ROS backend (2026-11-27; host tests only, nothing run in ROS)

### 12.1 Sigma in the free-spot check (`VLNWorld.free`, `plan._relation_point`)

Every landmark carries its xy sigma:
* object3d tracks: `sigma_xyz` (max of x, y);
* Phase 4 tracks: `sigma_m`;
* unknown: 0.10 m;
* mock: 0.04 m (its sighting noise);
* ground truth: 0.

A point computed from a landmark (stop / pass / relation) is only as sure as that landmark, so:
* wall clearance = 0.35 + `margin` + `sigma_k` · σ_landmark;
* object clearance = 0.4 + `margin` + `sigma_k` · √(σ_obj² + σ_landmark²);
* "between" uses the midpoint's σ = √(σa² + σb²) / 2.

Parameters on the world: `sigma_k` (default **3.0**) and `margin` (default 0.0, a fixed extra clearance).

**Measured: the blue-barrel case (DEV r01).** The 0.47 m gap to the wall, with 4 cm map noise:

| sigma_k | refused (of 500 draws) |
|---|---|
| 0 | 305 (61 %, a coin flip) |
| 2.0 | 496 |
| 2.5 | 499 |
| 3.0 | **500** |

Over 1000 more draws at k = 3 it was never accepted (test).

**What it does NOT do.** It moves the decision boundary; it cannot remove it. Under the same noise:

| case | accepted (of 200) |
|---|---|
| r02, r03, r05, r06, r09 | 200 |
| r08 | 199 |
| r04 "behind the yellow box" (about 0.10 m spare ≈ 2.5 σ) | **96 (still a coin flip)** |

The flip band has moved from "spare ≈ 0" (the robot does not fit, but the spot is accepted: the dangerous side) to "spare
≈ 3σ" (the robot would fit, but the spot is refused: the safe side). The DEV mock e2e accepts a refusal for r04 (marked
`sigma_refusal_ok`) and keeps a positive "behind" case (r09). On an exact map (σ = 0) nothing changes: the oracle DEV
run is 46/46, as before.

### 12.2 RosBackend (behind `run_text --vln` / `VLN=true scripts/start_web_robot.sh`, default off)

**Temporary zones** (`RosBackend.add_temp_zone(rect, tag, name, ttl_s=1800)` / `remove_temp_zones(tag | None)`):
* They go into the SAME `zones` list as the user's no-go zones, as rectangles, so they get the same enforcement:
  * goals are refused inside them;
  * the Nav2 plan check and `_detour` apply;
  * running legs re-check their path (`zones_version` is bumped);
  * straight walks check them.
* They are removed:
  * by tag in the executor's `finally` (normal end, failure, exception);
  * all of them on `estop(True)`;
  * by their TTL in `tick()` (event `zone_expired`), if the executor never comes back.
* They are never written to the places file (`_save_places` filters them; `_load_places` drops any found there), so a
  process crash loses them too.

**World model** (`VLNWorld.from_ros`):
* rooms: the places-file rectangles;
* doors: the places file's optional `doors:` section, else detected on `/map` (`doors_from_map`) and cached per map
  id + rooms;
  * `doors_from_map`: along each shared room edge, where the map is free on the edge and 0.25 m into both rooms, the
    longest free run ≥ 0.5 m is the doorway centre;
  * on the lab4 walls: within 2.5 cm of the ground-truth doorways;
* objects: the confirmed semantic-map tracks with their σ;
* pose: TF.

**Executor on ROS:**
* It ticks RosBackend every 0.5 s and sleeps (FakePort.advance in the tests).
* E-stop: the port's `estop_active()`, a cancelled leg, or a leg that RosBackend ended with state `estop`.
* The re-observation uses the detector's `last_seen` (sim time, the same clock as `now()`).

**Tests** (`brain/tests/test_vln_ros.py`, 16; RosBackend + a FakePort with the lab4 walls on `/map` and a fake detector
that marks objects in the camera cone as seen):
* sigma: consistency of the blue barrel; roomy spots unaffected; never accepting a no-room spot; margin and σ terms
* temporary zones: enforced, not saved, removed by tag, on e-stop and by TTL, ignored when found in a file
* world: rooms / doors / σ / pose from RosBackend; places-file doors win; synthetic door detection (gap, no gap, too narrow)
* walks: an avoid walk on RosBackend (zone = room B, never entered, removed after); the agent's red-barrel walk with
  re-observation; a blind detector → no arrival claim; e-stop mid walk → zones removed; a crash mid walk → zones removed,
  the robot stopped, "Something went wrong"; flag off

**Suites:** brain 1178 passed / 3 skipped; VLN DEV oracle 46/46; mock end to end 46/46 (SR 1.0, SPL 0.995, 0 false claims,
0 avoid violations, answers 11/11); fetch DEV 42/42; keyword v1 57.0 % (unchanged); web OK.

**Not verified:**
* Nothing ran in ROS or Gazebo.
* The real `/map` has noise and unknown cells: door detection may need a looser free test.
* Nav2 does not know the temporary zones (backend-side checks only, as for the user's zones; the keepout filter is
  still "NEW").
* The walk is blocking: the agent answers only the e-stop while walking.

**Stop here (user, 2026-11-27):** no more new features until the Gazebo tests catch up.

## 13. Named rooms, keyword fallback, constrained JSON, reachability (2026-11-28, branch `phase10-vln`; host tests only)

Target instruction: **"go to the kitchen, past the chair, stop near the red barrel"** → rooms + semantic-map landmarks →
waypoints with the σ-aware clearance → honest replies. Nothing ran in ROS / Gazebo or against Ollama (CPU only).

### 13.1 What changed

| piece | change |
|---|---|
| Room names | places file: `room D: {rect: [...], kind: room, names: [kitchen]}`. `VLNWorld.names` (`room_names()`); RosBackend keeps `names` on load and save; mock: `world.room_names`. `canon_room("the kitchen")` → room D. Replies say "room D (the kitchen)"; "avoid the kitchen" / "stay out of the kitchen" work. **No world has names yet**: `maps/lab4.places.yaml` is unchanged (decide which room is which before a run). |
| "near X" | = `stop` at X (prompt rule + keyword "stop by / near / at"). "Stop near X" falls back to the free side of a ring round X (nearest to the approach) when the straight stand-off is blocked. |
| Keyword fallback | `brain/vln/keyword.py`: text → the SAME JSON → the SAME validator. Used when the model is down, returns bad JSON, or its plan is rejected; alone with `VLNPlanner(False)` (`run_text --vln` without `--ollama`, `brain.agent.cli --vln --vln-keyword`). Returns None (normal planner) when unsure: an unknown noun, "before" / "after X" (order reversal), a route that does not end anywhere, or any object / room word it could not place (never drops one). `Plan.source` = llm / keyword. |
| Constrained JSON | `vln_json_schema()` = the plan grammar as JSON schema, sent as Ollama `format` (grammar-constrained decoding; `schema=False` → plain `"json"`). Every DEV reference plan fits it (test). The validator still does all the checks a schema cannot. |
| Validator | NEW reject `DROPPED`: an object / room the user named (outside the avoid phrases) is missing from the plan — the model silently leaving out "past the chair" is a different walk (→ keyword fallback). NEW answer `UNKNOWN_LANDMARK` before any model call: a word in a landmark position that is no class, room name or known place ("past the sofa"). |
| Clearance | Points now also keep `0.4 + margin + 3σ` from **their own landmark** (before: only from walls and other objects; with the ROS default σ 0.10 m a 0.6 m stand-off could really be 0.3 m). Room entry point = the free point closest to the centre (the centre itself was used unchecked: a chair next to room D's centre made the leg unreachable). |
| Reachability | Executor pre-flight: `path_check(point)` for every waypoint, with the avoid zones and the user's no-go zones in place, BEFORE moving → `UNREACHABLE`: "I can't reach X from here: a no-go zone is in the way. I haven't moved." A block appearing later stays an honest LEG_FAILED. No door into a room: "I don't know a way from room A into room D on this map" (was "... without going through ." with an empty avoid list). |

### 13.2 Tests and numbers (DEV, NOT held-out; `docs/heldout/` not opened)

`brain/tests/test_vln_route.py` (30): the target instruction end to end on the mock (keyword, LLM valid, LLM inventing a
landmark, LLM dropping the avoid; GT: chair passed within 1.2 m, stop in room D ≤ 1.2 m from the barrel), commas or not,
avoid by name, never-seen chair, "sofa", two chairs → "Which one?" (+ resolved by "in the kitchen"), room E, user zones
closing both kitchen doors (refused before moving, user zones kept), a doorway closing mid-walk, room without a door,
clearance for σ 0 / 0.04 / 0.10 (σ 0.15 everywhere → honest NO_FREE_SPOT), keyword rules, schema, RosBackend + FakePort
(names load / save, the walk, unreachable).

| run | result |
|---|---|
| DEV plan level, oracle (no fallback) | 46 / 46 |
| DEV plan level, keyword only | 46 / 46 — **tuned on this set** (I fixed "first ... then" / "afterwards" after seeing q02 / q03 fail), so not a clean measure |
| DEV mock end to end, oracle / keyword | 46 / 46 each: SR 1.0, SPL 0.993, 0 false claims, 0 avoid violations, answers 11 / 11 |
| brain suite | see the commit; keyword baseline v1 58.0 % (unchanged); safety gate PASS; web OK |

### 13.3 Left before a Gazebo run

1. **Room names in a real places file** (which lab4 room is "the kitchen"?) — a user decision; plus a chair in the world
   (lab4 has none: "past the chair" is LANDMARK_NOT_FOUND there, correctly).
2. **Ollama, never run**: `python3 -m brain.eval.vln_dev --model gurdy-sft-400 [--no-fallback]` (GPU, under the lock).
   Unknown: whether gurdy-sft-400 follows the prompt at all, and whether Ollama's schema `format` with nested `anyOf`
   works on this version (fallback: `schema=False`).
3. **Blind suite** written by the user (the DEV set is mine and now tuned against).
4. **ROS-only behaviour the FakePort cannot show**: `path_check` per waypoint = one Nav2 `compute_path` each (+ detour
   tries): measure the pre-flight time; door waypoints exactly on 1 m doorway centres inside the 0.55 m inflation; door
   detection on a noisy `/map`; landmark re-observation with the real detector (`last_seen`, sim time); temporary zones
   are backend-side only (Nav2 keepout filter still not used).
5. A Gazebo VLN runner (fresh stack per episode, GT trajectory, `metrics.py`), as in section 9 step 3.
