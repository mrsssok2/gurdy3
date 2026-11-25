# Incident graph evaluation (incident_v2)

Run: 2026-11-25T09:11:40Z commit 5b75e73, model gurdy-sft-400, falls 0, ground-truth changes verified in Gazebo: True.

## Graph (events vs ground truth)

- moved objects found with the right rooms: 2 / 2; removed object reported missing: 1 / 1; false moved / missing reports for untouched objects: 1

  - red box: found=True delay 1769.4 s
  - red barrel: found=True delay 244.7 s
  - toolbox (removed): found=True in room D, delay 193.6 s
  - FALSE: object_missing gray box room A

## Own questions (20, written by the router's author)

**10 / 20** correct; routed 12 / 20, content 12 / 20, wrong claims 2, grounding violations 0, questions that used the LLM 5, motion started by a question 0; latency p50 0.19 s, max 46.87 s.

| category | ok | n |
|---|---|---|
| changes | 2 | 6 |
| last_seen | 2 | 4 |
| history | 3 | 4 |
| timeline | 2 | 3 |
| rooms | 1 | 3 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| n01 | Has anything been shuffled around since I stepped out? | recall | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| n02 | Is room A the way I left it? |  | NO | missing /red box/ |
| n03 | Any surprises in room B since I went out? |  | NO | missing /barrel/ |
| n04 | What's different in room C compared to before I left? | what_changed | yes |  |
| n05 | Did anything disappear? | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| n06 | Tell me what is new in room D. | what_changed | yes |  |
| n07 | Where did you see the red barrel most recently, and when? | last_seen | NO | clock 09:54 outside the ground-truth window |
| n08 | How long ago was the mannequin spotted? | last_seen | yes |  |
| n09 | Have you seen a toolbox lately? | last_seen | yes |  |
| n10 | When did you last spot a bicycle? |  | NO |  |
| n11 | Did the red box end up somewhere new? | object_history | yes |  |
| n12 | Has the yellow box been touched? | object_history | yes |  |
| n13 | Was the red barrel relocated? | object_history | yes |  |
| n14 | Is the red toolbox where I left it? | where_is | NO | missing /missing|not there|gone/ |
| n15 | What went on while I was out of the room? | timeline | yes |  |
| n16 | Can you describe the latest events in room C? | timeline | yes |  |
| n17 | What made you stop earlier on? |  | NO | no clock time in the reply |
| n18 | Which rooms haven't been visited in the past 4 minutes? | unchecked_rooms | yes |  |
| n19 | At what time were you last in room B? |  | NO | missing /room B/; no clock time in the reply |
| n20 | Is there anything in room Q? |  | NO |  |

Replies:

- **n01** Has anything been shuffled around since I stepped out? -> red fire extinguisher in room C; red box in room A; white traffic cone in room A; blue barrel in room B; blue box in room B; red barrel in room D; gray mannequin in room D; red toolbox in room D; white traffic cone in room D; yellow box in room C; red barrel in room B; red box in room C; gray box in room A; red barrel in room B; red box in room C; gray box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in room C; box in room A; red traffic cone in room D; box in
- **n02** Is room A the way I left it? -> I can't do that. I don't know that kind of object, and I have not seen one.
- **n03** Any surprises in room B since I went out? -> I can't do that. I don't know what a room is, and I have not seen one.
- **n04** What's different in room C compared to before I left? -> Today, in room C: 3 changes. The red box moved from room A to room C at 10:00 (22 minutes ago); a new red fire extinguisher appeared in room C at 09:12 (1 hour ago); a new yellow box appeared in room C at 09:26 (56 minutes ago).
- **n05** Did anything disappear? -> Today: 15 changes. The red barrel moved from room D to room B at 09:34 (47 minutes ago); the red box moved from room A to room C at 10:00 (22 minutes ago); the red toolbox is missing from room D; I looked there at 09:34 (48 minutes ago) and it was not there; the red box is missing from room A; I looked there at 10:04 (18 minutes ago) and it was not there; the gray box is missing from room A; a new red fire extinguisher appeared in room C at 09:12 (1 hour ago); and 1 more.
- **n06** Tell me what is new in room D. -> Today, in room D: 6 changes. The red barrel moved from room D to room B at 09:34 (47 minutes ago); the red toolbox is missing from room D; I looked there at 09:34 (48 minutes ago) and it was not there; the red barrel is missing from room D; I looked there at 09:35 (47 minutes ago) and it was not there; a new gray mannequin appeared in room D at 09:21 (1 hour ago); a new red toolbox appeared in room D at 09:21 (1 hour ago); a new white traffic cone appeared in room D at 09:21 (1 hour ago).
- **n07** Where did you see the red barrel most recently, and when? -> I last saw the red barrel in room B at 09:54 (27 minutes ago).
- **n08** How long ago was the mannequin spotted? -> I last saw the gray mannequin in room D at 09:57 (25 minutes ago).
- **n09** Have you seen a toolbox lately? -> I last saw the red toolbox in room D at 09:24 (58 minutes ago), but it was not there when I checked later.
- **n10** When did you last spot a bicycle? -> I can't do that. I don't know what a bicycle is, and I have not seen one.
- **n11** Did the red box end up somewhere new? -> Yes. The red box moved from room A to room C at 10:00 (22 minutes ago).
- **n12** Has the yellow box been touched? -> I haven't recorded the yellow box moving. I have seen it 11 times so far. I last recorded it in room C.
- **n13** Was the red barrel relocated? -> Yes. The red barrel moved from room D to room B at 09:34 (48 minutes ago).
- **n14** Is the red toolbox where I left it? -> The red toolbox in room D, 6.0 metres behind-left.
- **n15** What went on while I was out of the room? -> Here is what happened while you were away (the latest 10 of 37): 09:48 Entered room D. 09:53 Done. 09:56 Entered room C. 10:00 The red box moved from room A to room C. 10:00 Done. 10:02 Saw a new gray box in room A. 10:03 Entered room A. 10:04 The red box was not where I last saw it. 10:04 The gray box was not where I last saw it. 10:06 Done.
- **n16** Can you describe the latest events in room C? -> Here is what happened today in room C: 09:12 Saw a new red fire extinguisher in room C. 09:25 Entered room C. 09:26 Saw a new yellow box in room C. 09:56 Entered room C. 10:00 The red box moved from room A to room C.
- **n17** What made you stop earlier on? -> I stopped because I reached my goal, 16 minutes ago.
- **n18** Which rooms haven't been visited in the past 4 minutes? -> Not checked for 4 minutes: room B (last at 09:48, 34 minutes ago); room C (last at 10:03, 20 minutes ago); room D (last at 09:56, 26 minutes ago). Checked recently: room A.
- **n19** At what time were you last in room B? -> I can't do that. I don't know what a room is, and I have not seen one.
- **n20** Is there anything in room Q? -> There is no room Q. I know rooms A, B, C and D.

## External questions (10, written by the user)

**7 / 10** correct; routed 9 / 10, content 7 / 10, wrong claims 2, grounding violations 0, questions that used the LLM 0, motion started by a question 0; latency p50 0.15 s, max 0.28 s.

| category | ok | n |
|---|---|---|
| external | 7 | 10 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| y01 | has anything been taken away since you started? | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| y02 | the barrel, where did it end up? | object_history | yes |  |
| y03 | compare room A now with how it looked at the start | what_changed | yes |  |
| y04 | list what moved and where to | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| y05 | when was the last time you laid eyes on the toolbox? | last_seen | yes |  |
| y06 | anything different in room D? | what_changed | yes |  |
| y07 | give me a quick summary of today's events | timeline | yes |  |
| y08 | did the red box stay put? | object_history | yes |  |
| y09 | which room have you not checked in a while? | unchecked_rooms | NO | room B should be listed (not visited for 600 s); room D should be listed (not visited for 600 s); room C should be listed (not visited for 600 s) |
| y10 | is there a room E? |  | yes |  |

Replies:

- **y01** has anything been taken away since you started? -> Since you started me: 15 changes. The red barrel moved from room D to room B at 09:34 (48 minutes ago); the red box moved from room A to room C at 10:00 (23 minutes ago); the red toolbox is missing from room D; I looked there at 09:34 (49 minutes ago) and it was not there; the red box is missing from room A; I looked there at 10:04 (19 minutes ago) and it was not there; the gray box is missing from room A; a new red fire extinguisher appeared in room C at 09:12 (1 hour ago); and 1 more.
- **y02** the barrel, where did it end up? -> Yes. The red barrel moved from room D to room B at 09:34 (48 minutes ago).
- **y03** compare room A now with how it looked at the start -> Since you started me, in room A: 5 changes. The red box moved from room A to room C at 10:00 (23 minutes ago); the red box is missing from room A; I looked there at 10:04 (19 minutes ago) and it was not there; the gray box is missing from room A; a new white traffic cone appeared in room A at 09:15 (1 hour ago); a new gray box appeared in room A at 10:02 (20 minutes ago).
- **y04** list what moved and where to -> Today: 15 changes. The red barrel moved from room D to room B at 09:34 (49 minutes ago); the red box moved from room A to room C at 10:00 (23 minutes ago); the red toolbox is missing from room D; I looked there at 09:34 (49 minutes ago) and it was not there; the red box is missing from room A; I looked there at 10:04 (19 minutes ago) and it was not there; the gray box is missing from room A; a new red fire extinguisher appeared in room C at 09:12 (1 hour ago); and 1 more.
- **y05** when was the last time you laid eyes on the toolbox? -> I last saw the red toolbox in room D at 09:24 (59 minutes ago), but it was not there when I checked later.
- **y06** anything different in room D? -> Today, in room D: 6 changes. The red barrel moved from room D to room B at 09:34 (49 minutes ago); the red toolbox is missing from room D; I looked there at 09:34 (49 minutes ago) and it was not there; the red barrel is missing from room D; I looked there at 09:35 (48 minutes ago) and it was not there; a new gray mannequin appeared in room D at 09:21 (1 hour ago); a new red toolbox appeared in room D at 09:21 (1 hour ago); a new white traffic cone appeared in room D at 09:21 (1 hour ago).
- **y07** give me a quick summary of today's events -> Here is what happened today (the latest 10 of 37): 09:48 Entered room D. 09:53 Done. 09:56 Entered room C. 10:00 The red box moved from room A to room C. 10:00 Done. 10:02 Saw a new gray box in room A. 10:03 Entered room A. 10:04 The red box was not where I last saw it. 10:04 The gray box was not where I last saw it. 10:06 Done.
- **y08** did the red box stay put? -> No. The red box moved from room A to room C at 10:00 (23 minutes ago).
- **y09** which room have you not checked in a while? -> I have checked every room less than an hour ago.
- **y10** is there a room E? -> There is no room E. I know rooms A, B, C and D.

---
# Analysis (written after the run, 2026-11-25; ONE run, no variance; nothing was changed after seeing these results)

Run dir `brain/runs/incident_eval/20261125_091044`, commit 5b75e73, same scenario / world / model as the first run (`phase6_incident_eval.md`, which is unchanged). v2 questions are NEW, so the
question numbers are not like-for-like; the graph numbers are. v2 scoring matches objects by class with the colour optional (the first run's committed score is not re-scored).

| | first run (v1) | this run (v2) |
|---|---|---|
| moves found, right rooms | 2 / 2 | 2 / 2 |
| removed object reported missing | 0 / 1 (scorer key; 1 / 1 colour-free) | 1 / 1 (193.6 s after the removal) |
| false moved / missing reports | 16 (4 colour-free) | 1 (gray box, room A) |
| `object_seen` events for the removed toolbox | ~52 over 20 min (ghost) | 3 |
| own questions | 3 / 20 | 10 / 20 (routed 12 / 20, wrong claims 2) |
| external (user-written) | 3 / 10 | 7 / 10 (routed 9 / 10, wrong claims 2) |
| falls / LLM-phrased incident answers | 0 / yes | 0 / none (templates); p50 latency 0.19 s (was 0.96 s) |

**Ghost toolbox: confirmed fixed in the graph.** Sim-age to wall conversion + in-view + new-hits guards: the removed toolbox has 3 sightings (all before the removal), is reported missing once, no flapping
("is back" spam gone), and "last seen" is the real last sighting (n09 / y05 correct, clock inside the window). The graph's false reports fell 16 -> 1. Cause (earlier doc): clock mixing at RTF ~0.4, not YOLO, not rendering.

**Remaining failures (observations; causes read from the replies, not all verified):**
1. **n02 / n03 / n19 (unrouted phrasings)**: "Is room A the way I left it?", "Any surprises in room B since I went out?", "At what time were you last in room B?" had no cue word, went to the planner, and the
   unknown-noun guard answered "I can't do that. I don't know what a room is" (a wrong, confusing reply for a room question: separate bug). n14 "Is the red toolbox where I left it?" went to `where_is` and printed the stale remembered
   location ("6.0 metres behind-left"). n17 "What made you stop earlier on?" got the old stop explanation ("because I reached my goal"). n01 "shuffled around" fell to `recall`: the memory dump is now cut by `squash` but still lists unchanged objects (wrong claim).
2. **"What changed" counts first sightings as "new"** (n05, y01, y04, y06, n04): with a window that includes round 1 ("today", "since you started") every object first seen is listed as a change ("a new gray mannequin appeared in room D"):
   15 changes for 3 real ones. The graph is right, the query semantics are not: first sightings during the first look at a room are a baseline, not a change. Also a moved object is listed again as "missing from" its old room.
3. **y09 "which room have you not checked in a while?"**: the exact pattern matched first and its threshold stayed 3600 s ("checked every room less than an hour ago"); the "in a while" -> 600 s rule exists only in the classifier. n18 (explicit 4 minutes) passes.
4. n07: the barrel's last sighting (09:54) is later than the scorer's room-B round-2 window (it was seen again from another room): arguably correct, scored as wrong by the window.
5. y02 "where did it end up?" answers "Yes. The red barrel moved ..." (history template for a where-question), content right. n10 (bicycle) and n20 (room Q) give the right honest replies but count as not routed (the own set requires an incident tool).
Passes include all three `what moved` history questions (n11, n12, n13), both timeline questions (n15, n16), the unchecked-rooms threshold question (n18), room-existence (y10), "stay put" -> "No." (y08), last seen (n08, n09, y05), and the unknown room list "A, B, C and D" (n20, y10).

Not run: LLM tool picker (not needed for any question here), a second run for variance, the lab4 v3 suite (scheduled after 22:30). Fixes 1-3 above are not done; they need the next held-out set.
