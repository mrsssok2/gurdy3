# OFFLINE REPLAY (ollama, picker on, run brain/runs/incident_eval/20261125_111853)

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

**10 / 20** correct; routed 16 / 20, content 12 / 20, wrong claims 4, grounding violations 0, questions that used the LLM 1, motion started by a question 0; latency p50 0.36 s, max 7.69 s.

| category | ok | n |
|---|---|---|
| changes | 4 | 6 |
| last_seen | 1 | 4 |
| history | 3 | 4 |
| timeline | 1 | 3 |
| rooms | 1 | 3 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| m01 | Did I miss anything while I was out? | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| m02 | Has room B been rearranged? | what_changed | yes |  |
| m03 | Is anything gone from room D? | what_changed | yes |  |
| m04 | Is everything in room A where it should be? | what_changed | yes |  |
| m05 | What differs in room C compared with before? | what_changed | yes |  |
| m06 | Any objects that have vanished altogether? | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| m07 | When did the toolbox last show up on camera? | last_seen | yes |  |
| m08 | What's the most recent place you saw the red box? | last_seen | NO | clock 12:03 outside the ground-truth window |
| m09 | Do you remember when you spotted the fire extinguisher? | last_seen | NO | clock 12:07 outside the ground-truth window |
| m10 | When did you last notice a wheelbarrow? | recall | NO | missing /no record|never|haven't seen|have not seen|don't know what/; must not say /room [A-D]/ |
| m11 | Is the red barrel in the same spot as at the start? | object_history | yes |  |
| m12 | Has the blue barrel been shifted at all? | object_history | yes |  |
| m13 | Did the toolbox get taken? | what_changed | NO |  |
| m14 | Is the red box staying in room A? | object_history | yes |  |
| m15 | Fill me in on the last stretch while I was gone. | what_changed | NO | missing /entered room/ |
| m16 | What went on in room D today? | timeline | yes |  |
| m17 | What was the reason for you halting after I gave that order? | incident_why | NO | must not say /no record/ |
| m18 | Which rooms have gone unvisited for more than five minutes? | unchecked_rooms | yes |  |
| m19 | How recently were you in room C? | unchecked_rooms | NO | clock 12:04 outside the ground-truth window |
| m20 | Does room G exist? |  | NO |  |

Replies:

- **m01** Did I miss anything while I was out? -> While you were away: 8 changes. The red barrel moved from room D to room B at 11:50 (31 minutes ago); the red box moved from room A to room C at 12:03 (18 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (28 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (45 minutes ago); the blue barrel is back in room B at 11:39 (43 minutes ago); the gray mannequin is back in room D at 11:53 (28 minutes ago); and 2 more.
- **m02** Has room B been rearranged? -> Today, in room B: 5 changes. The red barrel moved from room D to room B at 11:50 (31 minutes ago); a new gray box appeared in room B at 11:36 (45 minutes ago); the blue barrel is back in room B at 11:39 (43 minutes ago); the gray box is back in room B at 11:56 (25 minutes ago); the gray traffic cone is back in room B at 12:06 (16 minutes ago).
- **m03** Is anything gone from room D? -> Today, in room D: 3 changes. The red barrel moved from room D to room B at 11:50 (31 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (28 minutes ago) and it was not there; the gray mannequin is back in room D at 11:53 (28 minutes ago).
- **m04** Is everything in room A where it should be? -> Today, in room A: 1 change. The red box moved from room A to room C at 12:03 (18 minutes ago).
- **m05** What differs in room C compared with before? -> Today, in room C: 1 change. The red box moved from room A to room C at 12:03 (19 minutes ago).
- **m06** Any objects that have vanished altogether? -> Today: 8 changes. The red barrel moved from room D to room B at 11:50 (32 minutes ago); the red box moved from room A to room C at 12:03 (19 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (29 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (43 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **m07** When did the toolbox last show up on camera? -> I last saw the red toolbox in room D at 11:34 (48 minutes ago), but it was not there when I checked later.
- **m08** What's the most recent place you saw the red box? -> I last saw the red box in room C at 12:03 (19 minutes ago).
- **m09** Do you remember when you spotted the fire extinguisher? -> I last saw the red fire extinguisher in room C at 12:07 (15 minutes ago).
- **m10** When did you last notice a wheelbarrow? -> blue barrel in room B; red barrel in room B.
- **m11** Is the red barrel in the same spot as at the start? -> No. The red barrel moved from room D to room B at 11:50 (32 minutes ago).
- **m12** Has the blue barrel been shifted at all? -> I haven't recorded the blue barrel moving. I have seen it 20 times so far. I last recorded it in room B.
- **m13** Did the toolbox get taken? -> Today: 8 changes. The red barrel moved from room D to room B at 11:50 (32 minutes ago); the red box moved from room A to room C at 12:03 (19 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (29 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (44 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **m14** Is the red box staying in room A? -> No. The red box moved from room A to room C at 12:03 (19 minutes ago).
- **m15** Fill me in on the last stretch while I was gone. -> While you were away: 8 changes. The red barrel moved from room D to room B at 11:50 (32 minutes ago); the red box moved from room A to room C at 12:03 (19 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (29 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (44 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **m16** What went on in room D today? -> Here is what happened today in room D (the latest 10 of 14): 11:33 I am not making progress. 11:34 Saw a new white traffic cone in room D. 11:50 The red barrel moved from room D to room B. 11:52 Entered room D. 11:53 The path is blocked. 11:53 The gray mannequin was not where I last saw it. 11:53 The red barrel was not where I last saw it. 11:53 The red toolbox was not where I last saw it. 11:53 The gray mannequin is back in room D. 11:55 I am not making progress.
- **m17** What was the reason for you halting after I gave that order? -> At 11:35 (48 minutes ago): I stopped because you told me to. I have no recorded cause for it.
- **m18** Which rooms have gone unvisited for more than five minutes? -> Not checked for 5 minutes: room B (last at 11:52, 30 minutes ago); room C (last at 12:04, 18 minutes ago); room D (last at 12:01, 21 minutes ago). Checked recently: room A.
- **m19** How recently were you in room C? -> I was last in room C at 12:04 (18 minutes ago).
- **m20** Does room G exist? -> There is no room G. I know rooms A, B, C and D.

## External questions (10, written by the user)

**7 / 10** correct; routed 8 / 10, content 7 / 10, wrong claims 2, grounding violations 0, questions that used the LLM 1, motion started by a question 0; latency p50 0.32 s, max 7.14 s.

| category | ok | n |
|---|---|---|
| external | 7 | 10 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| z01 | what's been shuffled around since this morning? | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| z02 | tell me only the things that actually changed | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| z03 | is the barrel still in room D? | object_history | yes |  |
| z04 | I think someone took something, did they? |  | NO | missing /toolbox/ |
| z05 | what did room C look like before versus now? | what_changed | yes |  |
| z06 | when did you last pass through room A? | unchecked_rooms | yes |  |
| z07 | which rooms are overdue for a check? | unchecked_rooms | yes |  |
| z08 | has the red box gone anywhere? | object_history | yes |  |
| z09 | is there anything in a room F? |  | yes |  |
| z10 | quick recap, nothing extra | timeline | yes |  |

Replies:

- **z01** what's been shuffled around since this morning? -> Today: 8 changes. The red barrel moved from room D to room B at 11:50 (33 minutes ago); the red box moved from room A to room C at 12:03 (20 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (30 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (44 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **z02** tell me only the things that actually changed -> Today: 8 changes. The red barrel moved from room D to room B at 11:50 (33 minutes ago); the red box moved from room A to room C at 12:03 (20 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (30 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (44 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **z03** is the barrel still in room D? -> No. The red barrel moved from room D to room B at 11:50 (33 minutes ago).
- **z04** I think someone took something, did they? -> Which kind of object do you mean?
- **z05** what did room C look like before versus now? -> Since you started me, in room C: 1 change. The red box moved from room A to room C at 12:03 (20 minutes ago).
- **z06** when did you last pass through room A? -> I was last in room A at 12:23 (just now).
- **z07** which rooms are overdue for a check? -> I have checked every room less than an hour ago.
- **z08** has the red box gone anywhere? -> Yes. The red box moved from room A to room C at 12:03 (20 minutes ago).
- **z09** is there anything in a room F? -> There is no room F. I know rooms A, B, C and D.
- **z10** quick recap, nothing extra -> Here is what happened today (the latest 10 of 46): 11:55 I had to stop exploring: my path is blocked. 11:55 Done. 11:56 The gray box is back in room B. 12:01 Entered room C. 12:02 Saw a new yellow box in room C. 12:03 The red box moved from room A to room C. 12:04 Entered room A. 12:05 The red box was not where I last saw it. 12:06 The gray traffic cone is back in room B. 12:08 Done.
