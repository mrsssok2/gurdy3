# Incident graph evaluation (incident_v3)

Run: 2026-11-25T11:19:30Z commit e92848e, model gurdy-sft-400, falls 0, ground-truth changes verified in Gazebo: True.

## Graph (events vs ground truth)

- moved objects found with the right rooms: 2 / 2; removed object reported missing: 1 / 1; false moved / missing reports for untouched objects: 4

  - red box: found=True delay 1693.8 s
  - red barrel: found=True delay 911.6 s
  - toolbox (removed): found=True in room D, delay 1098.9 s
  - FALSE: object_missing gray traffic_cone room B
  - FALSE: object_missing blue barrel room B
  - FALSE: object_missing gray box room B
  - FALSE: object_missing gray mannequin room D

## Own questions (20, written by the router's author)

**7 / 20** correct; routed 11 / 20, content 8 / 20, wrong claims 2, grounding violations 0, questions that used the LLM 5, motion started by a question 0; latency p50 0.16 s, max 23.66 s.

| category | ok | n |
|---|---|---|
| changes | 3 | 6 |
| last_seen | 0 | 4 |
| history | 3 | 4 |
| timeline | 1 | 3 |
| rooms | 0 | 3 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| m01 | Did I miss anything while I was out? | timeline | NO | missing /barrel/; missing /toolbox/; must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| m02 | Has room B been rearranged? | what_changed | yes |  |
| m03 | Is anything gone from room D? | what_changed | yes |  |
| m04 | Is everything in room A where it should be? | where_is | NO | missing /red box/ |
| m05 | What differs in room C compared with before? | what_changed | yes |  |
| m06 | Any objects that have vanished altogether? | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| m07 | When did the toolbox last show up on camera? | recall | NO | missing /not there|missing|gone/; no clock time in the reply |
| m08 | What's the most recent place you saw the red box? | last_seen | NO | clock 12:03 outside the ground-truth window |
| m09 | Do you remember when you spotted the fire extinguisher? | last_seen | NO | clock 12:07 outside the ground-truth window |
| m10 | When did you last notice a wheelbarrow? | recall | NO |  |
| m11 | Is the red barrel in the same spot as at the start? | object_history | yes |  |
| m12 | Has the blue barrel been shifted at all? | object_history | yes |  |
| m13 | Did the toolbox get taken? | object_history | yes |  |
| m14 | Is the red box staying in room A? | where_is | NO | missing /^\W*no/ |
| m15 | Fill me in on the last stretch while I was gone. | what_changed | NO | missing /entered room/ |
| m16 | What went on in room D today? | timeline | yes |  |
| m17 | What was the reason for you halting after I gave that order? |  | NO | no clock time in the reply |
| m18 | Which rooms have gone unvisited for more than five minutes? | unchecked_rooms | NO | room B should be listed (not visited for 300 s); room D should be listed (not visited for 300 s); room C should be listed (not visited for 300 s) |
| m19 | How recently were you in room C? |  | NO | missing /room C/; no clock time in the reply |
| m20 | Does room G exist? |  | NO | missing /(don't|do not) know a room|no such room|not a room|there is no room/ |

Replies:

- **m01** Did I miss anything while I was out? -> Here is what happened while you were away (the latest 10 of 26): 11:55 I had to stop exploring: my path is blocked. 11:55 Done. 11:56 The gray box is back in room B. 12:01 Entered room C. 12:02 Saw a new yellow box in room C. 12:03 The red box moved from room A to room C. 12:04 Entered room A. 12:05 The red box was not where I last saw it. 12:06 The gray traffic cone is back in room B. 12:08 Done.
- **m02** Has room B been rearranged? -> Today, in room B: 5 changes. The red barrel moved from room D to room B at 11:50 (31 minutes ago); a new gray box appeared in room B at 11:36 (45 minutes ago); the blue barrel is back in room B at 11:39 (43 minutes ago); the gray box is back in room B at 11:56 (25 minutes ago); the gray traffic cone is back in room B at 12:06 (16 minutes ago).
- **m03** Is anything gone from room D? -> Today, in room D: 3 changes. The red barrel moved from room D to room B at 11:50 (31 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (28 minutes ago) and it was not there; the gray mannequin is back in room D at 11:53 (28 minutes ago).
- **m04** Is everything in room A where it should be? -> I can't answer that; it is not possible.
- **m05** What differs in room C compared with before? -> Today, in room C: 1 change. The red box moved from room A to room C at 12:03 (19 minutes ago).
- **m06** Any objects that have vanished altogether? -> Today: 8 changes. The red barrel moved from room D to room B at 11:50 (32 minutes ago); the red box moved from room A to room C at 12:03 (19 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (29 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (43 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **m07** When did the toolbox last show up on camera? -> red toolbox in room D.
- **m08** What's the most recent place you saw the red box? -> I last saw the red box in room C at 12:03 (19 minutes ago).
- **m09** Do you remember when you spotted the fire extinguisher? -> I last saw the red fire extinguisher in room C at 12:07 (15 minutes ago).
- **m10** When did you last notice a wheelbarrow? -> I have no record of that.
- **m11** Is the red barrel in the same spot as at the start? -> No. The red barrel moved from room D to room B at 11:50 (32 minutes ago).
- **m12** Has the blue barrel been shifted at all? -> I haven't recorded the blue barrel moving. I have seen it 20 times so far. I last recorded it in room B.
- **m13** Did the toolbox get taken? -> I haven't recorded the red toolbox moving. I have seen it 2 times so far. I last recorded it in room D. It is currently marked missing: it was not there when I checked.
- **m14** Is the red box staying in room A? -> The red box in room A, 1.8 metres ahead; The red box in room C, 4.8 metres behind-right.
- **m15** Fill me in on the last stretch while I was gone. -> While you were away: 8 changes. The red barrel moved from room D to room B at 11:50 (32 minutes ago); the red box moved from room A to room C at 12:03 (19 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (29 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (44 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **m16** What went on in room D today? -> Here is what happened today in room D (the latest 10 of 14): 11:33 I am not making progress. 11:34 Saw a new white traffic cone in room D. 11:50 The red barrel moved from room D to room B. 11:52 Entered room D. 11:53 The path is blocked. 11:53 The gray mannequin was not where I last saw it. 11:53 The red barrel was not where I last saw it. 11:53 The red toolbox was not where I last saw it. 11:53 The gray mannequin is back in room D. 11:55 I am not making progress.
- **m17** What was the reason for you halting after I gave that order? -> I stopped because I reached my goal, 15 minutes ago.
- **m18** Which rooms have gone unvisited for more than five minutes? -> I have checked every room less than an hour ago.
- **m19** How recently were you in room C? -> That sounds like a question about a room. Which room do you mean? Please name one, for example room A.
- **m20** Does room G exist? -> Did you mean room B, room C or room D? I heard "room g".

## External questions (10, written by the user)

**7 / 10** correct; routed 7 / 10, content 7 / 10, wrong claims 1, grounding violations 0, questions that used the LLM 2, motion started by a question 0; latency p50 0.16 s, max 1.23 s.

| category | ok | n |
|---|---|---|
| external | 7 | 10 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| z01 | what's been shuffled around since this morning? | recall | NO | missing /red box/; missing /barrel/ |
| z02 | tell me only the things that actually changed | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| z03 | is the barrel still in room D? | object_history | yes |  |
| z04 | I think someone took something, did they? | where_is | NO | missing /toolbox/ |
| z05 | what did room C look like before versus now? | what_changed | yes |  |
| z06 | when did you last pass through room A? | unchecked_rooms | yes |  |
| z07 | which rooms are overdue for a check? | unchecked_rooms | yes |  |
| z08 | has the red box gone anywhere? | object_history | yes |  |
| z09 | is there anything in a room F? |  | yes |  |
| z10 | quick recap, nothing extra | timeline | yes |  |

Replies:

- **z01** what's been shuffled around since this morning? -> explore; explore; explore; explore; cancelled; timeout; explore; blocked; explore; done.
- **z02** tell me only the things that actually changed -> Today: 8 changes. The red barrel moved from room D to room B at 11:50 (33 minutes ago); the red box moved from room A to room C at 12:03 (20 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (30 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (44 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **z03** is the barrel still in room D? -> No. The red barrel moved from room D to room B at 11:50 (33 minutes ago).
- **z04** I think someone took something, did they? -> The red box in room A, 1.8 metres ahead; The red box in room C, 4.8 metres behind-right.
- **z05** what did room C look like before versus now? -> Since you started me, in room C: 1 change. The red box moved from room A to room C at 12:03 (20 minutes ago).
- **z06** when did you last pass through room A? -> I was last in room A at 12:23 (just now).
- **z07** which rooms are overdue for a check? -> I have checked every room less than an hour ago.
- **z08** has the red box gone anywhere? -> Yes. The red box moved from room A to room C at 12:03 (20 minutes ago).
- **z09** is there anything in a room F? -> There is no room F. I know rooms A, B, C and D.
- **z10** quick recap, nothing extra -> Here is what happened today (the latest 10 of 46): 11:55 I had to stop exploring: my path is blocked. 11:55 Done. 11:56 The gray box is back in room B. 12:01 Entered room C. 12:02 Saw a new yellow box in room C. 12:03 The red box moved from room A to room C. 12:04 Entered room A. 12:05 The red box was not where I last saw it. 12:06 The gray traffic cone is back in room B. 12:08 Done.

---
# Analysis (written after the run, 2026-11-25; ONE successful run, no variance; no code was changed after seeing results)

Run dir `brain/runs/incident_eval/20261125_111853`, commit e92848e. The first attempt (`phase6c_incident_eval_run1_FAILED.md`, commit ece8552) died with `sim_hang` after the world changes (Gazebo
stopped publishing) before any question: an infrastructure failure, the same code was run again. Earlier reports are unchanged. The question sets differ per run (v1 / v2 / v3), so question counts are NOT like-for-like; graph numbers are.

| | v1 (first) | 6b (v2) | 6c (v3) |
|---|---|---|---|
| moves found, right rooms | 2 / 2 | 2 / 2 | 2 / 2 |
| removed object reported missing | 0 / 1 (1 / 1 colour-free) | 1 / 1 | 1 / 1 |
| false moved / missing reports | 16 (4 colour-free) | 1 | 4 (traffic cone, blue barrel, gray box in B; mannequin in D: class / colour flips and a run with many blocked paths and explore time-outs) |
| own questions | 3 / 20 | 10 / 20 | 7 / 20 (routed 11 / 20, wrong claims 2) |
| external (user-written) | 3 / 10 | 7 / 10 | 7 / 10 (routed 7 / 10, wrong claims 1) |
| latency p50 / max | 0.96 / 30.3 s | 0.19 / 46.9 s | 0.16 / 23.7 s (planner-path answers are the slow ones) |
| falls | 0 | 0 | 0 |

Offline regression after the 6c code (gurdy-sft-400, `docs/results/incident_regression_6c/`): testset_v1 94 %, v1.1 95 %, ood_v1 **72.5 %** (was 75 %): case o15 ("how many boxes have you found so far", a list instead of "3") now fails.
It fails identically when commit 5b75e73 (the 6b code, which scored 75 %) is checked out and run today, so it is not caused by the 6c changes; cause unknown (the Ollama model files are unchanged), reported as a drop of unexplained origin. Brain suite 810 passed.

**What the fixes did (observed):**
- Fix 1 works where measured: "what did room C look like before versus now" and "what differs in room C" give exactly the one real change (red box moved in), no first sightings. The whole-map questions still list 8 changes for 3 real ones in this run, because
  of the 4 false `missing` reports and "is back" events from a noisy run (class / colour flips, blocked paths), not first sightings. A moved object is no longer also listed as missing.
- Fix 4 works on the routed path (z03 "is the barrel still in room D?" -> "No. The red barrel moved from room D to room B ..."; m11, m13). It does not fire when the router misses the phrasing: m14 "Is the red box staying in room A?" and z04 ("someone took something") went to `where_is` and printed stale positions again.
- Fix 5: z07 and z06 fine; m18 "gone unvisited for more than five minutes" has an explicit time the parser does not read ("more than" between "for" and the number) -> default one hour ("checked every room less than an hour ago"), a wrong answer.
- Fix 3: "I don't know what a room is" is gone, but the replacement guard reply is poor: m19 "How recently were you in room C?" got "That sounds like a question about a room. Which room do you mean?" although room C was named (the planner called an object tool with class "room"); m04 got "I can't answer that; it is not possible".

**Remaining failures:** m01 (timeline instead of what_changed for "did I miss anything"), m04 / m14 / z04 / m07 / z01 (not routed: "where it should be", "staying", "someone took something", "show up on camera", "shuffled around since this morning" -> planner `where_is` / `recall`; z01 printed "explore; explore; ... cancelled; timeout" junk),
m06 / z02 (whole-map list includes the false reports), m08 / m09 (last-seen clock later than the scorer's round-2 window: the object was seen again from elsewhere, a scorer-window limitation), m10 (honest "no record", not routed so counted wrong), m17 (stop reason: "because I reached my goal", the old explanation path; `incident_why` is not reached without the word "why"),
m18 (threshold), m19, m20 ("Does room G exist?" -> repair question "Did you mean room B ...": G is in the sound-alike set, so the honest room-existence answer is skipped).
The routing coverage grows with each set but every new set still finds unrouted phrasings: the cue-word approach does not generalise to free phrasing; an LLM tool picker (validated) was not built.
Disclosure: the user's external question list was part of the task message and therefore visible to the assistant while coding; it was not used for cue words or tests, and the router was not run on it before the run.
