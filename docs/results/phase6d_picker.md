# Phase 6d: LLM tool picker for incident questions (2026-11-25)

Earlier reports are unchanged. Everything here is offline (no Gazebo); the one Gazebo-free "evaluation" is the replay below.

## What was built
- `brain/incidents/picker.py`: a cheap high-recall pre-filter (`might_be_incident`) + a few-shot prompt (generic examples, none equal to any test / dev / held-out phrasing: `test_incident_picker.TestHeldOut`)
  asking the local model for ONE JSON `{tool, room, about, still}`. Strict validation: tool in the six incident tools (else `none` -> existing routing); the room comes from the USER's words and must be on the
  current map (a room the model adds is dropped, one that contradicts the text or is not on the map rejects the pick); the object comes from the user's words (known classes / colours), never from the model; time windows and
  thresholds are parsed from the user's words (`nlu.parse_since` / `parse_threshold`, now also "for more than / longer than / at least N minutes", "lately", "for a bit"; vague = 10 min), never from the model; an object tool
  with only a room is remapped to `unchecked_rooms` / `what_changed`. Server down, bad JSON, invalid value: `None` (keyword routing continues). Answers stay templates over the query result: the model never words an answer and cannot print memory.
- Order in `nlu.parse`: exact patterns -> picker -> cue-word classifier -> planner. `OllamaLLM.incident_picker` (off with `GURDY_INCIDENT_PICKER=0`, model `GURDY_PICKER_MODEL`, default `gurdy-sft-400`).
- Dev set `brain/eval/incident_picker_dev.py` (66 phrasings incl. 16 non-incident negatives; looked at while building) and `python3 -m brain.eval.picker_dev --model M`.
  **Model choice:** gurdy-sft-400 **58 / 66** (after prompt + validation tuning on this dev set; first untuned prompt 50 / 66, rooms 9 / 9, 0 failures, p50 0.31 s) vs base qwen3:1.7b 53 / 66 (p50 0.30 s) at the same prompt state before the last two changes. Chosen: gurdy-sft-400 (also the agent's model: no second model to load). 0 of the 16 negatives was routed to an incident tool.

## Other fixes
- m19 ("How recently were you in room C?"): when the planner uses an object tool with class "room" and the user NAMED a room, the guard now says what can be asked about that room instead of "Which room do you mean?".
- "Does room G exist?": any typed unknown room letter now gets "There is no room G. I know rooms A, B, C and D." The letter-to-room "repair" is gone (the sound-alike set made G a repair of B/C/D); misspelt WORDS ("dorway") still get a "Did you mean ...?" (tests updated).
- "What was the reason for you halting after I gave that order": `_WHY_STOP` covers reason / cause / how come / what made + halt / pause / quit / freeze; "after I said / that order" counts as an earlier stop; `incident_why` prefers the last user stop or e-stop (6b).

## o15 regression (ood_v1 75 % -> 72.5 %): NOT our bug
`how many boxes have you found so far` failed in 3 of 3 runs after the Gazebo evaluations, also on the 6b code (commit 5b75e73, which had scored 75 %) checked out today, so the code did not change it. After **restarting the
project's Ollama server** ood_v1 is 75.0 % again in 2 of 2 runs, with the same model files. Conclusion: server state (a long-lived llama.cpp process that served the long incident evaluations) changed the model's output for this
borderline case at temperature 0; cause inside Ollama not investigated. Practice: restart Ollama before offline evaluations. Offline sets after the 6d code (fresh Ollama): testset_v1 94 %, v1.1 95 %, ood_v1 75 % (`docs/results/incident_regression_6d/`). Brain suite 837 passed.

## Offline replay harness (item 4 of the task text was cut off after "build a harness"; this is my reading of it)
`python3 -m brain.eval.incident_replay RUN_DIR --suite incident_suite_v4 --picker on|off --llm ollama|keyword`: copies a finished run's `stack/incidents.db`, replays its graph with the clock of the live questions
(`ReplayBackend`: graph + places + object memory, everything robot-bound answers UNAVAILABLE) and asks a suite's questions through the REAL agent, scored with the same checks against the run's ground truth. Minutes instead of an hour.
Limits: the graph is what that run's recorder produced (the 6c graph still has 4 false missing reports); suites other than that run's scenario are not valid; planner-path answers have no live robot.

| replay on the 6c graph (run 111853), gurdy-sft-400 | picker off | picker on |
|---|---|---|
| **incident_v4, 20 NEW questions, replayed once after freezing the code** | **4 / 20** (routed 5, wrong claims 1, LLM questions 13) | **12 / 20** (routed 13, wrong claims 2, LLM questions 1) |
| incident_v3 (seen; used for the 6c fixes, so dev-like) own / external | 8 / 20, 7 / 10 | 10 / 20, 7 / 10 |
| incident_v2 on the 6b graph (seen, used for 6b / 6c fixes) own / external | 16 / 20, 10 / 10 | 16 / 20, 10 / 10 |
Latency p50 0.35 s with the picker (one extra model call per candidate question), 0.5 s without (the planner is the slow path). Live 6c numbers for comparison: own 7 / 20, external 7 / 10 (live Gazebo, different questions).
v4 is not independent (same author as the router and picker) and the graph under it is noisy; the 8-question gain is the measured effect of the picker on unseen phrasings. v2 / v3 numbers must not be read as held-out any more.

## Still failing on v4 with the picker (replies read, nothing changed after seeing them)
p03 ("Something feels off in room D, is it?" -> `incident_why`, a wrong tool pick), p05 / p06 (whole-map `what_changed` lists unchanged objects: false `missing` / `back` events of the noisy 6c graph; p06 routed to `timeline`),
p09 ("out of your view" -> `unchecked_rooms`, wrong tool), p10 (a ladder: honest "I don't know what a ladder is" but not an incident route), p14 (`what_changed` instead of `object_history`), p15 (not routed), p20 (honest, not routed).
Next steps that would help: a real held-out Gazebo run with the picker on (not done: the replay was chosen to save the Gazebo slot), more false-report suppression in the graph (class / colour flips), few-shot examples for "feels off" / "out of view".
