# Phase 6 (3.0): Incident Graph = temporal memory

Status 2026-11-25: **built and unit-tested, NOT evaluated in Gazebo.** The step 6 evaluation (scenario + 20 held-out questions) is committed and not run.
(The older ops idea in `docs/phase6_design.md` — Redpanda / HDBSCAN fault clustering over telemetry — is a separate track and stays paused.)

## What it does
Gurdy records what happened over time and answers: "What changed in room C since I left?", "When did you last see the toolbox, and where?", "Has the
fire extinguisher moved?", "What happened while I was away?", "Why did you stop earlier?", "Which rooms haven't you checked for an hour?".

## Pieces (`brain/incidents/`)
| file | job |
|---|---|
| `store.py` | thread-safe SQLite temporal graph (`IncidentStore`). Nodes: object, room, robot, user, event. Edges: `seen_in`, `about`, `in`, `moved_from`, `moved_to`, `caused_by`, `before` (`after` = reverse), `issued_by`, `by`. Every row has `map_id`; event nodes have sim time AND wall time. `":memory:"` = nothing survives a restart. `to_networkx()` on demand. |
| `recorder.py` | events from what Gazebo / the backend already produce: `object_seen` (class, colour, position, room, confidence; re-sightings only every 60 s), `object_moved` (same object > 0.5 m from its last position: old -> new, rooms), `object_missing`, `room_visited` / `room_left`, `path_blocked`, `no_go_set` / `no_go_cleared`, `stop` / `estop` (reason), `alert`, `task_*`, `user_command`, `session_start`; cause links (`caused_by`). |
| `queries.py` | `what_changed(room?, since?)`, `last_seen(object)`, `object_history(object)`, `timeline(since?, room?, object?)`, `incident_why`, `unchecked_rooms`. Plain dicts; every time is given as `wall_t`, `clock` "HH:MM", `ago_text`, `ago_s`. No data = `no_data` with a reason, never an empty success. |
| `phrasing.py` | template answers built only from the result (also the fallback of the grounding check); `must_include` = facts an LLM-phrased reply must contain. |
| `nlu.py` | deterministic recognition of clear question phrasings (before the planner). Commands never match. Unknown room -> "I don't know a room called ...". |
| `backend.py` | `IncidentBackend` wraps any backend (mock, ROS, `MemoryBackend`): records in `poll_events`, `no_go_zone`, `note_user_command` (after the answer, so "while I was away" = since the PREVIOUS command). |
| `tools.py` | the 6 incident tools. They are NOT in `brain.tools.TOOLS` (that dict is the planner prompt; the fine-tuned model was trained on exactly those 24 tools), `ToolLayer.call` validates and dispatches them separately. |

## Identity, move and missing rules (all deterministic, conservative)
- Same object = same backend track id, or same class + colour within 0.6 m.
- **Moved**: the same object sighted > 0.5 m from its last position (detector noise ~0.04 m). A same-class same-colour object that appears while exactly ONE such object is *missing* is that object (one `object_moved`; the "new object" event is marked superseded). Two candidates = nothing merged (ambiguous, never guessed).
- **Missing**: expected in room R (last seen there), and the robot stayed >= 20 s in R with the object's last position within 3 m and +-40 deg of the heading of some pose of the visit (camera cone, conservative), without seeing it. Also when the backend says so after an `inspect`. Passing through a doorway never flags anything.
- The evaluation of all this is in two parts (graph vs ground truth, answers vs ground truth): see below.

## Agent
`Agent._incident_route` (in `n_route`, before the stop explanation and the planner): parse -> tool call through the `ToolLayer` -> template reply. If the LLM has `phrases_incidents` (OllamaLLM) it may rephrase: the reply must pass the grounding check against the query result AND mention the facts that carry the answer, else the template is used. Without a timeline-capable backend the text goes to the planner as before. Wiring: `brain.ros.run_text` always wraps with the graph (`incidents.db` next to `memory.db` when `MEMORY=true`; RAM only otherwise); the web demo / mock voice host use a RAM graph on a virtual wall clock.

## Web (`web/`)
Timeline tab / panel (latest events; filter by time window, room, object), **What changed?** button (same query + template answer), dashed orange arrows (old -> new position) with a ghost marker for moved objects on the map. API: `GET /api/timeline`, `GET /api/changes` (`web/api/incidents_api.py`, also used by the in-browser demo; its Pyodide worker tries to load `sqlite3`, and without it the demo has no timeline). Browser (Playwright) tests were NOT re-run for the new panel.

## Tests
`brain/tests/test_incidents.py` (store, recorder, each query, grounding of every template, routing, agent, LLM phrasing + fallback), `web/tests/test_incidents_api.py`, `brain/tests/test_incident_suite.py` (suite / scorer, no question is ever put to an agent). Mock world = the Gazebo flat_lab; mock tracks now follow a real move (> 0.5 m) of an object.

## Regression (2026-11-25, gurdy-sft-400, offline sets, files in `docs/results/incident_regression/`)
| set | step B | now |
|---|---|---|
| testset_v1 | 94 % | 94 % |
| testset_v1_1 | 95 % | 95 % |
| ood_v1 | 75 % | 75 % |
Identical failing case ids in all three. Brain + web unit tests: all pass (see commit). No Gazebo run, no 27 / 33-mission suite run (not needed: the only change on the agent path is a new branch that fires on incident questions).

## Evaluation (prepared, NOT run)
`brain/eval/incident_suite.py` (scenario + 20 questions with fixed expectations), `brain/ros/incident_eval_runner.py`, `brain/eval/incident_score.py`, `scripts/incident_eval.sh`.
One session in one fresh lab4 stack: look around every room (A B D C) -> typed "go to room B" / "stop" / "hello" (the user leaves) -> Gazebo `set_pose`: red box A -> C, red barrel D -> B; `remove`: toolbox (read back, `verified` in the log) -> look around again (B D C A) -> 300 s wait -> the 20 questions. Scored in two parts: **graph** (both moves found with the right rooms, toolbox reported missing, zero false moved / missing reports for untouched objects) and **answers** (routing, content, clock times against the ground-truth windows, rooms for "not checked for 5 minutes", wrong claims, grounding).
Open risks to check first tomorrow: (1) `set_pose` on STATIC models (all lab4 models are `<static>`): look at `gt.ops[*].verified` in `result.json` before trusting anything else; (2) the operator-level look-around uses `explore(area=room)`; (3) one run, no variance.
The question phrasings were written by the same assistant that wrote the router: not independent of it (some are deliberately less regular). Failures to understand them are findings.

Run: `nohup scripts/incident_eval.sh > /tmp/incident_eval.out 2>&1 &` (about 1-2 h; refuses to start while any container runs; report -> `docs/results/phase6_incident_eval.md`, run dir `brain/runs/incident_eval/<ts>/`).
