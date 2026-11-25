# OFFLINE REPLAY (ollama, picker off, run brain/runs/incident_eval/20261125_111853)

# Incident graph evaluation (incident_blind_v1)

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

**14 / 20** correct; routed 16 / 20, content 15 / 20, wrong claims 3, grounding violations 0, questions that used the LLM 7, motion started by a question 0; latency p50 0.01 s, max 0.56 s.

| category | ok | n |
|---|---|---|
| changes | 1 | 4 |
| last_seen | 1 | 2 |
| history | 2 | 3 |
| timeline | 2 | 2 |
| why | 2 | 2 |
| rooms | 1 | 2 |
| honesty | 1 | 1 |
| negative | 4 | 4 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| b01 | anything look different in room D since you last went in? | what_changed | NO | must not say /blue box|yellow box|mannequin|extinguisher|blue barrel/ |
| b02 | when did you last spot the toolbox? | last_seen | yes |  |
| b03 | has the red box moved around at all today? | object_history | yes |  |
| b04 | walk me through what happened over the last ten minutes | timeline | yes |  |
| b05 | why did you stop earlier? | incident_why | yes |  |
| b06 | which rooms haven't you looked at yet? | unchecked_rooms | yes |  |
| b07 | is the red barrel still where it was? | object_history | yes |  |
| b08 | did someone mess with room A? |  | NO | missing /red box/ |
| b09 | where'd the red box go? | where_is | NO |  |
| b10 | give me a recap of your patrol | timeline | yes |  |
| b11 | any rooms you skipped? | recall | NO | missing /every room|all rooms|all four|no room/ |
| b12 | how come you paused after I told you to stop? | incident_why | yes |  |
| b13 | what's new around here in the last 10 minutes? | what_changed | yes |  |
| b14 | has anything been missing for more than five minutes? | what_changed | NO | must not say /blue box|yellow box|mannequin|extinguisher|blue barrel/; must not say /red box|red barrel/ |
| b15 | tell me the story of the red box | recall | NO | missing /room A/ |
| b16 | is room Q real? |  | yes |  |
| b17 | go to room C | go_to | yes |  |
| b18 | what do you see right now? | describe_view | yes |  |
| b19 | how many legs do you have? | status | yes |  |
| b20 | stop |  | yes |  |

Replies:

- **b01** anything look different in room D since you last went in? -> Today, in room D: 3 changes. The red barrel moved from room D to room B at 11:50 (31 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (28 minutes ago) and it was not there; the gray mannequin is back in room D at 11:53 (28 minutes ago).
- **b02** when did you last spot the toolbox? -> I last saw the red toolbox in room D at 11:34 (48 minutes ago), but it was not there when I checked later.
- **b03** has the red box moved around at all today? -> Yes. The red box moved from room A to room C at 12:03 (18 minutes ago).
- **b04** walk me through what happened over the last ten minutes -> I haven't recorded anything in the last 10 minutes.
- **b05** why did you stop earlier? -> At 11:35 (47 minutes ago): I stopped because you told me to. I have no recorded cause for it.
- **b06** which rooms haven't you looked at yet? -> I have checked every room less than an hour ago.
- **b07** is the red barrel still where it was? -> No. The red barrel moved from room D to room B at 11:50 (32 minutes ago).
- **b08** did someone mess with room A? -> I can tell you what changed in room A, when I was last there, or what happened there. Which of those do you want?
- **b09** where'd the red box go? -> The red box in room C, 0.0 metres ahead.
- **b10** give me a recap of your patrol -> Here is what happened today (the latest 10 of 46): 11:55 I had to stop exploring: my path is blocked. 11:55 Done. 11:56 The gray box is back in room B. 12:01 Entered room C. 12:02 Saw a new yellow box in room C. 12:03 The red box moved from room A to room C. 12:04 Entered room A. 12:05 The red box was not where I last saw it. 12:06 The gray traffic cone is back in room B. 12:08 Done.
- **b11** any rooms you skipped? -> I have no record of that.
- **b12** how come you paused after I told you to stop? -> At 11:35 (48 minutes ago): I stopped because you told me to. I have no recorded cause for it.
- **b13** what's new around here in the last 10 minutes? -> I haven't recorded anything in the last 10 minutes.
- **b14** has anything been missing for more than five minutes? -> Today: 8 changes. The red barrel moved from room D to room B at 11:50 (32 minutes ago); the red box moved from room A to room C at 12:03 (19 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (29 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (44 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **b15** tell me the story of the red box -> The red box in room C.
- **b16** is room Q real? -> There is no room Q. I know rooms A, B, C and D.
- **b17** go to room C -> path_check is not available in a replay
- **b18** what do you see right now? -> I can't answer that; it is not available in a replay.
- **b19** how many legs do you have? -> I have 6 legs.
- **b20** stop -> I could not stop.

## External questions (10, written by the user)

**0 / 0** correct; routed 0 / 0, content 0 / 0, wrong claims 0, grounding violations 0, questions that used the LLM 0, motion started by a question 0; latency p50 None s, max None s.

| category | ok | n |
|---|---|---|

| id | question | tools | ok | problems |
|---|---|---|---|---|

Replies:

