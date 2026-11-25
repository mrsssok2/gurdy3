# Phase 6e: blind incident suite + graph-noise debounce (2026-11-25, offline, no Gazebo)

Replays use the recorded graph of the 6c live run (`brain/runs/incident_eval/20261125_111853`), model `gurdy-sft-400`, the real agent (router, picker, planner, templates). Ollama was restarted before each block.
Commands: `scripts/p6e_replay.sh TAG "SUITES" "PICKERS" 3`, `scripts/p6e_summarize.py TAG`; raw per-run JSON / reports in `docs/results/phase6e/{before,after,regression}/`.

## Blind suite (`brain/eval/incident_suite_blind_v1.py`, commit ed5444f, committed before any run)
The 20 questions are the user's, wording verbatim; only `{room}` / `{object}` were filled from the 6c ground truth (room D / room A / room C; toolbox / red box / red barrel) and the expectations were fixed from the same ground truth
(red box A -> C, red barrel D -> B, toolbox removed from D, everything else untouched). Q16-Q20 are honesty / negative questions: Q17-Q20 must NOT call an incident tool, Q17 must call `go_to`, Q18 `describe_view`. The scorer got two small additions for this
(`expect_tool` `"none"` / `"any"`, `check["calls"]`); older suites score exactly as before. Nothing in the router, picker or prompt was changed on this suite. Q-expectations were written by me, so scoring defects are possible (see b09).

## Results (mean and range of 3 runs; temperature 0, so the 3 runs are identical: range = 0)
| suite (20 q) | picker off | picker on |
|---|---|---|
| **blind_v1 (before fix)** | **14.0** (14-14), routed 16, wrong claims 3 | **15.0** (15-15), routed 17, wrong claims 2 |
| **blind_v1 (after fix)** | not re-run (fix cannot change the replay, see below) | **15.0** (15-15), routed 17, wrong claims 2 |
| v4 (before fix) | 4.0 (4-4), routed 5, wrong claims 1 | 12.0 (12-12), routed 13, wrong claims 2 |
| v4 (after fix) | not re-run | 12.0 (12-12), routed 13, wrong claims 2 |

v4 reproduces the 6d numbers exactly (4 / 12), so the harness is stable. On blind_v1 the picker adds only +1 (14 -> 15); on v4 it added +8. Q16-Q20 (honesty + 4 negatives) pass 5 / 5 in every run: no incident tool was called for "go to room C", "what do you see", "how many legs", "stop".

### The "after" columns are identical by construction: the graph fix is NOT measured by replay
The replay copies the finished run's `incidents.db`, i.e. the events the OLD recorder produced; it feeds no raw sightings or poses through the recorder, so a recorder change cannot alter it. The fix is verified by unit tests only
(`brain/tests/test_incident_debounce.py`, 7 tests, 3 of them fail on the old recorder). Its effect on the false reports (p05 / p06 / b01 / b14) needs a new Gazebo run; none was done (offline-only task).

## Per-question failures (blind_v1, picker on; picker off adds b08, which is a wrong route)
| q | question | tool called | cause | detail |
|---|---|---|---|---|
| b01 | anything look different in room D since you last went in? | what_changed (right) | **graph noise** | reply lists "the gray mannequin is back in room D" (false missing + back of the 6c graph; unchanged object) next to the 3 true changes. "since you last went in" is also ambiguous: the last visit saw the changes. |
| b09 | where'd the red box go? | object_history (right) | **scoring** | reply "The red box moved from room A to room C" is correct; my `none: room A` check wrongly rejects the origin. Counts as 1 of the 2 "wrong claims". |
| b11 | any rooms you skipped? | recall (wrong) | **wrong tool** | not routed to `unchecked_rooms`: planner picks `recall place`; reply "I have no record of any places." (no picker hit, no cue-word match). |
| b14 | has anything been missing for more than five minutes? | unchecked_rooms, `older_than_s=300` (wrong) | **wrong tool** (+ bad args) | picker / classifier read "more than five minutes" as the room-age threshold; should be `what_changed` (missing since). Reply "Not checked for 5 minutes: ..." is a true statement to the wrong question. With picker off: `what_changed` is called, but the whole-map list contains false reports of the noisy graph -> content fails. |
| b15 | tell me the story of the red box | recall (wrong) | **wrong tool** | "the story of X" is not a routing phrase; the planner answers "The red box in room C." (no history). |

Picker off, additionally: b08 (no tool, reply misses red box), b09 routed to `where_is` ("The red box in room C, 0.0 metres ahead": the 0.0 m is a replay artefact, stale memory), b14 as above.
Picker on, v4 (unchanged): p03 / p09 wrong tool, p05 / p06 graph noise, p10 / p15 / p20 unrouted, p14 wrong tool (`what_changed` instead of object_history).
Wrong-claim count 2 on blind_v1 = b01 (graph noise) + b09 (scoring defect): no wrong claim was invented by the router or a template.

## Graph-noise fix (`brain/incidents/recorder.py`, only brain/incidents touched)
Cause found in the 6c graph: 8 false `object_missing` events, all logged at 20-21 s into a room visit (the minimum), i.e. the moment `MIN_VISIT_S` ended, using a rule "any ONE pose sample within 3 m and +-40 degrees of the spot": one glimpse from the doorway, before the detector could confirm a track (8 hits / 2 s), marked the object missing; the next confirmed sighting then produced "is back" (5 of the 6c `returned` events).
- **missing**: the spot must be in view in >= 6 CONSECUTIVE pose observations spanning >= 4 s (a gap > 3 s or one non-covering sample restarts the run) - `MISS_MIN_OBS`, `MISS_MIN_SPAN_S`, `MISS_GAP_S`, `_clearly_in_view`. Still "not detected during the visit".
- **back**: a missing object needs 2 accepted re-sightings >= 1 s apart, each within 8 s of the previous one (on top of the existing 3-new-hits rule per sighting); a one-frame flicker leaves it missing - `_return_confirmed`.
- Tests: doorway glimpse is not missing; clear 25 s look is; interrupted looks restart the run; seen during the visit is never missing; flicker is not back; two separated sightings are back; sightings > 8 s apart start over.
- Limit / risk: a real removal now needs ~4 s of unbroken view (the robot's sweeps pause at each heading, so the 6c toolbox case would still be caught, but a fast pass-through no longer reports it); the thresholds are my choice, not tuned on data; a real "back" is delayed by about a second.

## Regression after the fix (fresh Ollama)
| set | before (6d) | after |
|---|---|---|
| testset_v1 | 94 % | **94 %** |
| testset_v1.1 | 95 % | **95 %** |
| ood_v1 | 75 % | **75 %** |
Brain suite (`pytest brain/tests`): **845 passed, 3 skipped** (836 + 7 debounce + 2 blind-suite tests).

## Honest limits
- The blind numbers are one replay on one noisy graph; with temperature 0 the 3 runs are identical, so "range" says nothing about variance across Ollama states or phrasings. 20 questions: one question = 5 points.
- Expectations are mine. b09's check is too strict (counted against the system, left as run); b01's "since you last went in" is ambiguous. Neither was changed after seeing results (frozen).
- The replay has no robot: Q17 / Q18 / Q20 replies say "not available in a replay"; only the routing (no incident tool, right tool called) is meaningful for them.
- The graph fix is unit-tested but unmeasured on real data until the next Gazebo run (the incident_v3/blind evaluation can show it).
- Open: "any rooms you skipped" / "story of ..." routing, "missing for more than N minutes" picked as a room-age threshold. Not fixed (the blind suite must not be tuned on).
