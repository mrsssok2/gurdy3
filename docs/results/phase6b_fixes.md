# Phase 6b: incident-graph fixes after `phase6_incident_eval.md` (2026-11-25)

The as-run numbers in `phase6_incident_eval.md` are unchanged and nothing was re-scored. These fixes are NOT yet evaluated: the held-out sets were not re-run
(a new held-out set is needed, the old questions now sit in no test; the unit tests use different wording).

1. **"while I was away / since I left / since earlier"**: anchored to the last user command that STARTED a mission (`user_command.data.starts_task`, set by the agent
   when the turn replaced `executor.steps`), else to the session start. Questions never move it. `since earlier` is now the same window.
2. **Unknown rooms**: `rooms.unknown_room_message` answers "There is no room Y. I know rooms A and B." for any typed `room X` (lowercase too) whose letter is
   not on the map, BEFORE the spelling repair. Single letters are all "edit distance 1" from each other, so the near-miss rule for letters is acoustic: only
   B C D E G P T V (STT swaps) and letters within two of the last known room still go to "Did you mean ...?". A misspelt word ("roon B") is read as room B in
   incident questions and the reply starts with `I took "roon B" as room B.`
3. **"Why did you stop?"**: the user's stop / e-stop wins over a later task failure or timeout; failures are used only when there was no user stop, or when asked about a failure/mission.
4. **Ghost toolbox: cause found, not YOLO and not rendering.** Check (`scripts/ghost_toolbox_check.sh`, data in `docs/results/data/phase6b/ghost/`, one run):
   robot 1.4 m from the toolbox -> 57 toolbox detections (score 0.97); toolbox removed with the same `gz` call as the eval -> camera frame empty
   (`frames/3_after_removal_30s.png`), model gone from the pose stream, **0 toolbox detections** while looking at the empty spot (other objects behind it were
   detected: fire extinguisher, yellow box). In the eval DB the toolbox track's sim `last_seen` stops at 516 s and never changes, yet `object_seen` events keep
   coming every ~60 s wall: the recorder computed `wall_seen = wall - age` with `age` in SIM seconds. At RTF ~0.41 (eval: sim 12 -> 1217 s over 2909 s wall) a
   stale track looks 0.59 s newer per second. Fix: `_wall_at` interpolates a recorded (sim, wall) clock; a sighting must also be newer in sim time.
   Extra guards as asked: in the camera cone (5 m, +-55 deg) of the pose nearest in time, and >= 3 new detector hits since the last accepted sighting
   (`hits` now in `RosBackend` object records); rejected ones are counted in `recorder.rejected_sightings`. Unit tests (`TestGhostSightings`) reproduce the bug at RTF 0.4 (fail on the old code).
   Open: the real effect on missing flapping is unmeasured until the next Gazebo run. The scorer's colour key for the toolbox ("toolbox" vs "red toolbox") is still not fixed.
5. **Robust routing** (`brain/incidents/nlu.py`): exact patterns first, then `classify()`, a cue-word classifier (change / different / moved / relocated / missing / taken / last seen / laid eyes /
   summary / events / not checked / neglected ...) that picks the TOOL and its ARGUMENTS (room, object, window, threshold; validated by the tool layer's schema). Motion verbs never match;
   "is there a room X" is answered from the map's room list; "still where it was / stay put" questions get a Yes / No start (a moved object = "No."). No LLM picker (the fine-tuned planner is not a
   classifier; nothing in the 20 + 10 questions needed it), so the only free-text model use left is the planner for non-incident text. Causes found for the old failures: the **LLM rephrasing of
   incident answers** (q18: dropped "room A checked recently"; q19: no clock time; q14 and q01: one sentence repeated ~100 times; q12: "has been seen ever since") -> incident answers are now the template
   over the query result only (`GURDY_INCIDENT_LLM_PHRASE=1` re-enables the grounded LLM phrasing). `templates.squash` removes repeated segments and caps any list at 8 ("I recall: ..." dumps).
   Router vocabulary caveat: the user's 10 new questions were visible while writing the cue lists (generic words only); the router was not run on them before the evaluation.
6. **Scorer, future runs only**: `incident_score.use_suite("incident_suite_v2")` matches graph events by class with the colour optional (v1 scoring stays exact, committed run not re-scored).
   x08: the expectation (`none ^yes`) was right, the defect was the history template, which now answers "No." to a "still where ..." question about a moved object; the v1 suite is unchanged.
7. **Unknown-room list**: the reply lists the rooms of the CURRENT map (`There is no room Y. I know rooms A, B, C and D.`, unit test with the lab4 room names; the earlier "A and B" was the mock world).

Regression after 1-7 (gurdy-sft-400, `docs/results/incident_regression_6b/`): testset_v1 94 %, v1.1 95 %, ood_v1 75 % = unchanged, same failing ids. Brain suite 798 passed, web 117 OK.
New held-out sets: `brain/eval/incident_suite_v2.py` (20 own + 10 user-written), committed before the run.
