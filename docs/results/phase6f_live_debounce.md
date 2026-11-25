# Phase 6f: live validation of the recorder debounce (2026-11-25)

One live incident eval in Gazebo (lab4_incident.sdf, `incident_suite_v3`, same scenario and model `gurdy-sft-400` as 6c) with the debounced recorder of 6e (commit 6bfff18, Ollama restarted first).
Run dir `brain/runs/incident_eval/20261125_132350` (13:24-14:51 UTC, 87 min wall vs 65 min for 6c; run dir is gitignored), raw report `docs/results/phase6f_live_eval_raw.md`.
Replays (picker on, 1 run each, temperature 0) on the NEW graph: `scripts/p6e_replay.sh p6f "incident_suite_blind_v1 incident_suite_v4" on 1 <new run dir>`, outputs in `docs/results/phase6e/p6f/`.
Suites, scorer, router, picker untouched; no frozen result re-scored. 0 falls; the 3 world changes were verified in Gazebo.

## 1. False events in the graph (6c db `20261125_111853` vs new db `20261125_132350`)
Counted by me directly from `nodes` (`etype='object_missing'`, and `object_seen` rows whose label is "is back"); a missing event is "false" if the object was not removed or moved at that time in the scenario (ground truth: red box A -> C, red barrel D -> B, toolbox removed from D; the old spots of the box and barrel legitimately read "not where I last saw it").

| | 6c (before) | 6f (after) |
|---|---|---|
| `object_missing` events total | 7 | 5 |
| explained by the scenario (barrel D, toolbox D, red box A) | 3 | 3 |
| **false missing** | **4** (gray cone B, blue barrel B, gray box B, gray mannequin D) | **2** (gray cone B at 13:30, red barrel in room B at 14:02) |
| **false "is back"** | **4** (blue barrel, mannequin, gray box, cone) | **1** (red barrel in B 14:13; it follows the false missing at 14:02) |
| scorer "false moved / missing reports for untouched objects" | 4 | 1 (the cone; the barrel false missing is attributed to a moved object, so the scorer does not count it) |
| moved objects found with the right rooms | 2 / 2 | 2 / 2 |
| removed object (toolbox) reported missing | 1 / 1 | 1 / 1 |

Note: 6e and CLAUDE.md say "8 false missing" for 6c. This 6c DB contains 7 missing events in total, 4 false; I cannot reproduce 8 and do not know how it was counted, so the table uses my own count.
A false "new gray box in room A" (confidence 0.66, flagged `appeared`) was also recorded at 14:33 (6c had the same kind in room B). That is a detector-level false appearance, outside the debounce.

The debounce works on the failure it was written for (doorway glimpses at the minimum visit time: 4 -> 0 of that kind, mannequin / blue barrel / gray box gone) but **not completely**: the gray cone in room B is still reported missing at 13:30 (~2 min into the run, a visit of room B), and a new false missing appeared for the red barrel in room B at 14:02, 1 min after it was moved there (the old record in room B vs the new one; probably the track identity of the moved barrel, not examined further). That one produced the only false "is back".

## 2. Real changes missed because of the debounce?
None missed: all three scenario changes were reported (2/2, 1/1). Report delays (first report minus change, from the scorer) were red box 2263 s (6c 1694), red barrel 202 s (912), toolbox 1238 s (1099). Delays differ in both directions because the robot's exploration path differs between runs; this single pair of runs cannot say whether the debounce adds delay (expected: ~4 s of unbroken view for a missing, ~1 s for a back). A fast pass-through that never gives 6 consecutive covering observations would be missed, but no such case occurred here.

## 3. Live questions on the new graph (the eval's own 30)
Own 14 / 20 (6c 7 / 20), external 7 / 10 (6c 7 / 10); wrong claims 2 + 2, grounding violations 0, motion started by a question 0. This run and 6c are both live and differ in graph AND in nothing else of the code except the 6d picker, 6e debounce and earlier fixes (6c predates the picker), so the own-question gain is not attributable to the debounce alone.

## 4. Replays on the new graph (picker on, 1 run; temperature 0)
| suite | 6e (old graph) | 6f (new graph) |
|---|---|---|
| blind_v1 | 15 / 20 (routed 17, wrong claims 2) | **16 / 20** (routed 17, wrong claims 1) |
| v4 | 12 / 20 (routed 13, wrong claims 2) | **13 / 20** (routed 13, wrong claims 0) |

Specific questions:
- **b01** (anything different in room D): now passes; reply lists only the barrel and toolbox, the false "gray mannequin is back" is gone.
- **p05** (anything go astray): passes, but the reply still lists the false "gray traffic cone is missing from room B" and "a new gray box appeared in room A": the suite's checks do not forbid those, so a pass hides the residual noise.
- **p06**: still fails; routed to `timeline` (latest 10 events), content lacks barrel / toolbox. A routing failure, as in 6e (6e attributed it to graph noise; with the noise lower it still fails, so that attribution was incomplete).
- Still failing in blind_v1: b09 (scorer defect, `none: room A` rejects the correct origin, unchanged), b11 / b15 (wrong tool, as in 6e), b14 (now `unchecked_rooms` for "missing for more than five minutes": wrong tool, no longer crashes into noise). v4: p03, p09, p10, p14, p15, p20 fail (p14 / p20 now fail; 6e's p14 was also wrong).
- No new failure caused by the debounce was found; the +1 on each suite comes from b01 and p05 (noise removed).

## Honest limits
- One live run on one scenario, one replay each; temperature 0 makes replays deterministic but says nothing about run-to-run variance of the live recorder (6c and 6f differ also in exploration path, timing and RTF).
- Counts are small (4 -> 2 false missing, 4 -> 1 false back); with one run per arm this is a direction, not a significance test.
- Residual false events remain (cone B, red barrel B, gray box A); the thresholds were never tuned on data.
- Replay scores are +1 / +1 on 20 questions = 5 points each; the suites' checks do not penalise all false events (p05).
- The 87 min wall time (vs 65) is not explained; both runs used the same command and 9000 s limit.
