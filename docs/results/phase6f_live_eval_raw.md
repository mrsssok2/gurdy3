# Incident graph evaluation (incident_v3)

Run: 2026-11-25T13:25:08Z commit 6bfff18, model gurdy-sft-400, falls 0, ground-truth changes verified in Gazebo: True.

## Graph (events vs ground truth)

- moved objects found with the right rooms: 2 / 2; removed object reported missing: 1 / 1; false moved / missing reports for untouched objects: 1

  - red box: found=True delay 2262.9 s
  - red barrel: found=True delay 202.0 s
  - toolbox (removed): found=True in room D, delay 1238.3 s
  - FALSE: object_missing gray traffic_cone room B

## Own questions (20, written by the router's author)

**14 / 20** correct; routed 16 / 20, content 17 / 20, wrong claims 2, grounding violations 0, questions that used the LLM 1, motion started by a question 0; latency p50 0.57 s, max 20.2 s.

| category | ok | n |
|---|---|---|
| changes | 5 | 6 |
| last_seen | 3 | 4 |
| history | 3 | 4 |
| timeline | 1 | 3 |
| rooms | 2 | 3 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| m01 | Did I miss anything while I was out? | what_changed | yes |  |
| m02 | Has room B been rearranged? | what_changed | yes |  |
| m03 | Is anything gone from room D? | what_changed | yes |  |
| m04 | Is everything in room A where it should be? | what_changed | yes |  |
| m05 | What differs in room C compared with before? | what_changed | yes |  |
| m06 | Any objects that have vanished altogether? | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| m07 | When did the toolbox last show up on camera? | last_seen | yes |  |
| m08 | What's the most recent place you saw the red box? | last_seen | yes |  |
| m09 | Do you remember when you spotted the fire extinguisher? | last_seen | yes |  |
| m10 | When did you last notice a wheelbarrow? | recall | NO |  |
| m11 | Is the red barrel in the same spot as at the start? | object_history | yes |  |
| m12 | Has the blue barrel been shifted at all? | object_history | yes |  |
| m13 | Did the toolbox get taken? | what_changed | NO |  |
| m14 | Is the red box staying in room A? | object_history | yes |  |
| m15 | Fill me in on the last stretch while I was gone. | what_changed | NO | missing /entered room/ |
| m16 | What went on in room D today? | timeline | yes |  |
| m17 | What was the reason for you halting after I gave that order? | incident_why | NO | must not say /no record/ |
| m18 | Which rooms have gone unvisited for more than five minutes? | unchecked_rooms | yes |  |
| m19 | How recently were you in room C? | unchecked_rooms | yes |  |
| m20 | Does room G exist? |  | NO |  |

Replies:

- **m01** Did I miss anything while I was out? -> While you were away: 5 changes. The red barrel moved from room D to room B at 14:00 (48 minutes ago); the red box moved from room A to room C at 14:34 (14 minutes ago); the red toolbox is missing from room D; I looked there at 14:17 (31 minutes ago) and it was not there; a new gray box appeared in room A at 14:33 (15 minutes ago); the red barrel is back in room B at 14:13 (35 minutes ago).
- **m02** Has room B been rearranged? -> Today, in room B: 3 changes. The red barrel moved from room D to room B at 14:00 (49 minutes ago); the gray traffic cone is missing from room B; I looked there at 13:30 (1 hour ago) and it was not there; the red barrel is back in room B at 14:13 (36 minutes ago).
- **m03** Is anything gone from room D? -> Today, in room D: 2 changes. The red barrel moved from room D to room B at 14:00 (49 minutes ago); the red toolbox is missing from room D; I looked there at 14:17 (31 minutes ago) and it was not there.
- **m04** Is everything in room A where it should be? -> Today, in room A: 2 changes. The red box moved from room A to room C at 14:34 (15 minutes ago); a new gray box appeared in room A at 14:33 (16 minutes ago).
- **m05** What differs in room C compared with before? -> Today, in room C: 1 change. The red box moved from room A to room C at 14:34 (15 minutes ago).
- **m06** Any objects that have vanished altogether? -> Today: 6 changes. The red barrel moved from room D to room B at 14:00 (49 minutes ago); the red box moved from room A to room C at 14:34 (15 minutes ago); the gray traffic cone is missing from room B; I looked there at 13:30 (1 hour ago) and it was not there; the red toolbox is missing from room D; I looked there at 14:17 (32 minutes ago) and it was not there; a new gray box appeared in room A at 14:33 (16 minutes ago); the red barrel is back in room B at 14:13 (36 minutes ago).
- **m07** When did the toolbox last show up on camera? -> I last saw the red toolbox in room D at 13:49 (1 hour ago), but it was not there when I checked later.
- **m08** What's the most recent place you saw the red box? -> I last saw the red box in room C at 14:34 (15 minutes ago).
- **m09** Do you remember when you spotted the fire extinguisher? -> I last saw the red fire extinguisher in room C at 14:36 (13 minutes ago).
- **m10** When did you last notice a wheelbarrow? -> I have no record of that.
- **m11** Is the red barrel in the same spot as at the start? -> No. The red barrel moved from room D to room B at 14:00 (50 minutes ago).
- **m12** Has the blue barrel been shifted at all? -> I haven't recorded the blue barrel moving. I have seen it 26 times so far. I last recorded it in room B.
- **m13** Did the toolbox get taken? -> Today: 6 changes. The red barrel moved from room D to room B at 14:00 (50 minutes ago); the red box moved from room A to room C at 14:34 (15 minutes ago); the gray traffic cone is missing from room B; I looked there at 13:30 (1 hour ago) and it was not there; the red toolbox is missing from room D; I looked there at 14:17 (32 minutes ago) and it was not there; a new gray box appeared in room A at 14:33 (16 minutes ago); the red barrel is back in room B at 14:13 (37 minutes ago).
- **m14** Is the red box staying in room A? -> No. The red box moved from room A to room C at 14:34 (15 minutes ago).
- **m15** Fill me in on the last stretch while I was gone. -> While you were away: 5 changes. The red barrel moved from room D to room B at 14:00 (50 minutes ago); the red box moved from room A to room C at 14:34 (15 minutes ago); the red toolbox is missing from room D; I looked there at 14:17 (32 minutes ago) and it was not there; a new gray box appeared in room A at 14:33 (17 minutes ago); the red barrel is back in room B at 14:13 (37 minutes ago).
- **m16** What went on in room D today? -> Here is what happened today in room D: 13:37 Saw a new red toolbox in room D. 13:37 Saw a new red barrel in room D. 13:39 Saw a new white traffic cone in room D. 13:41 Saw a new gray mannequin in room D. 13:42 Entered room D. 13:59 Entered room D. 14:00 The red barrel moved from room D to room B. 14:01 The red barrel was not where I last saw it. 14:16 Entered room D. 14:17 The red toolbox was not where I last saw it.
- **m17** What was the reason for you halting after I gave that order? -> At 13:56 (53 minutes ago): I stopped because you told me to. I have no recorded cause for it.
- **m18** Which rooms have gone unvisited for more than five minutes? -> Not checked for 5 minutes: room B (last at 14:16, 34 minutes ago); room C (last at 14:35, 15 minutes ago); room D (last at 14:27, 23 minutes ago). Checked recently: room A.
- **m19** How recently were you in room C? -> I was last in room C at 14:35 (15 minutes ago).
- **m20** Does room G exist? -> There is no room G. I know rooms A, B, C and D.

## External questions (10, written by the user)

**7 / 10** correct; routed 8 / 10, content 7 / 10, wrong claims 2, grounding violations 0, questions that used the LLM 1, motion started by a question 0; latency p50 0.51 s, max 8.48 s.

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

- **z01** what's been shuffled around since this morning? -> Today: 6 changes. The red barrel moved from room D to room B at 14:00 (50 minutes ago); the red box moved from room A to room C at 14:34 (16 minutes ago); the gray traffic cone is missing from room B; I looked there at 13:30 (1 hour ago) and it was not there; the red toolbox is missing from room D; I looked there at 14:17 (33 minutes ago) and it was not there; a new gray box appeared in room A at 14:33 (17 minutes ago); the red barrel is back in room B at 14:13 (37 minutes ago).
- **z02** tell me only the things that actually changed -> Today: 6 changes. The red barrel moved from room D to room B at 14:00 (50 minutes ago); the red box moved from room A to room C at 14:34 (16 minutes ago); the gray traffic cone is missing from room B; I looked there at 13:30 (1 hour ago) and it was not there; the red toolbox is missing from room D; I looked there at 14:17 (33 minutes ago) and it was not there; a new gray box appeared in room A at 14:33 (17 minutes ago); the red barrel is back in room B at 14:13 (37 minutes ago).
- **z03** is the barrel still in room D? -> No. The red barrel moved from room D to room B at 14:00 (50 minutes ago).
- **z04** I think someone took something, did they? -> Which kind of object do you mean?
- **z05** what did room C look like before versus now? -> Since you started me, in room C: 1 change. The red box moved from room A to room C at 14:34 (16 minutes ago).
- **z06** when did you last pass through room A? -> I was last in room A at 14:50 (just now).
- **z07** which rooms are overdue for a check? -> I have checked every room less than an hour ago.
- **z08** has the red box gone anywhere? -> Yes. The red box moved from room A to room C at 14:34 (16 minutes ago).
- **z09** is there anything in a room F? -> There is no room F. I know rooms A, B, C and D.
- **z10** quick recap, nothing extra -> Here is what happened today (the latest 10 of 43): 14:23 Done. 14:27 Entered room C. 14:31 Done. 14:33 Saw a new gray box in room A. 14:33 The path is blocked. 14:34 The red box moved from room A to room C. 14:35 I am not making progress. 14:35 I had to stop exploring: my path is blocked. 14:35 Entered room A. 14:36 The red box was not where I last saw it.
