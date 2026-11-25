# OFFLINE REPLAY (ollama, picker on, run brain/runs/incident_eval/20261125_111853)

# Incident graph evaluation (incident_v4)

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

**12 / 20** correct; routed 13 / 20, content 16 / 20, wrong claims 2, grounding violations 0, questions that used the LLM 1, motion started by a question 0; latency p50 0.26 s, max 5.42 s.

| category | ok | n |
|---|---|---|
| changes | 3 | 6 |
| last_seen | 2 | 4 |
| history | 3 | 4 |
| timeline | 2 | 3 |
| rooms | 2 | 3 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| p01 | Were any of the items swapped around while I was gone? | what_changed | yes |  |
| p02 | Has the contents of room C been altered? | what_changed | yes |  |
| p03 | Something feels off in room D, is it? | incident_why | NO | missing /toolbox|barrel/ |
| p04 | Has room A lost anything since the start? | what_changed | yes |  |
| p05 | Did anything go astray? | what_changed | NO | must not say /blue box|yellow box|mannequin|extinguisher|blue barrel/ |
| p06 | Give me the differences between now and when you began. | timeline | NO | missing /barrel/; missing /toolbox/; must not say /blue box|yellow box|mannequin|extinguisher|blue barrel/ |
| p07 | At what time did you last have the toolbox in sight? | last_seen | yes |  |
| p08 | Where did the red box show up most recently? | last_seen | yes |  |
| p09 | How long has the mannequin been out of your view? | unchecked_rooms | NO | missing /mannequin/; missing /room D/ |
| p10 | When was a ladder last on your camera? |  | NO |  |
| p11 | Has the red barrel stayed in room D? | object_history | yes |  |
| p12 | Was the blue barrel ever shoved somewhere else? | object_history | yes |  |
| p13 | Is the red box in the spot where it began? | object_history | yes |  |
| p14 | Did the toolbox vanish? | what_changed | NO |  |
| p15 | Bring me up to date on what's been going on in room C. |  | NO |  |
| p16 | What have you been busy with since I gave you that first order? | timeline | yes |  |
| p17 | What led to you coming to a halt after my command? | incident_why | yes |  |
| p18 | Which rooms have you left alone for over ten minutes? | unchecked_rooms | yes |  |
| p19 | When did you last look in on room B? | unchecked_rooms | yes |  |
| p20 | Does a room H exist on your map? |  | NO |  |

Replies:

- **p01** Were any of the items swapped around while I was gone? -> While you were away: 8 changes. The red barrel moved from room D to room B at 11:50 (31 minutes ago); the red box moved from room A to room C at 12:03 (18 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (28 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (45 minutes ago); the blue barrel is back in room B at 11:39 (43 minutes ago); the gray mannequin is back in room D at 11:53 (28 minutes ago); and 2 more.
- **p02** Has the contents of room C been altered? -> Today, in room C: 1 change. The red box moved from room A to room C at 12:03 (18 minutes ago).
- **p03** Something feels off in room D, is it? -> At 11:35 (47 minutes ago): I stopped because you told me to. I have no recorded cause for it.
- **p04** Has room A lost anything since the start? -> Since you started me, in room A: 1 change. The red box moved from room A to room C at 12:03 (18 minutes ago).
- **p05** Did anything go astray? -> Today: 8 changes. The red barrel moved from room D to room B at 11:50 (32 minutes ago); the red box moved from room A to room C at 12:03 (19 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (29 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (43 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **p06** Give me the differences between now and when you began. -> Here is what happened today (the latest 10 of 46): 11:55 I had to stop exploring: my path is blocked. 11:55 Done. 11:56 The gray box is back in room B. 12:01 Entered room C. 12:02 Saw a new yellow box in room C. 12:03 The red box moved from room A to room C. 12:04 Entered room A. 12:05 The red box was not where I last saw it. 12:06 The gray traffic cone is back in room B. 12:08 Done.
- **p07** At what time did you last have the toolbox in sight? -> I last saw the red toolbox in room D at 11:34 (48 minutes ago), but it was not there when I checked later.
- **p08** Where did the red box show up most recently? -> I last saw the red box in room C at 12:03 (19 minutes ago).
- **p09** How long has the mannequin been out of your view? -> I have checked every room less than an hour ago.
- **p10** When was a ladder last on your camera? -> I can't do that. I don't know what a ladder is, and I have not seen one.
- **p11** Has the red barrel stayed in room D? -> No. The red barrel moved from room D to room B at 11:50 (32 minutes ago).
- **p12** Was the blue barrel ever shoved somewhere else? -> I haven't recorded the blue barrel moving. I have seen it 20 times so far. I last recorded it in room B.
- **p13** Is the red box in the spot where it began? -> No. The red box moved from room A to room C at 12:03 (19 minutes ago).
- **p14** Did the toolbox vanish? -> Today: 8 changes. The red barrel moved from room D to room B at 11:50 (32 minutes ago); the red box moved from room A to room C at 12:03 (19 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (29 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (44 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **p15** Bring me up to date on what's been going on in room C. -> I can tell you what changed in room C, when I was last there, or what happened there. Which of those do you want?
- **p16** What have you been busy with since I gave you that first order? -> Here is what happened today (the latest 10 of 46): 11:55 I had to stop exploring: my path is blocked. 11:55 Done. 11:56 The gray box is back in room B. 12:01 Entered room C. 12:02 Saw a new yellow box in room C. 12:03 The red box moved from room A to room C. 12:04 Entered room A. 12:05 The red box was not where I last saw it. 12:06 The gray traffic cone is back in room B. 12:08 Done.
- **p17** What led to you coming to a halt after my command? -> At 11:35 (48 minutes ago): I stopped because you told me to. I have no recorded cause for it.
- **p18** Which rooms have you left alone for over ten minutes? -> Not checked for 10 minutes: room B (last at 11:52, 30 minutes ago); room C (last at 12:04, 18 minutes ago); room D (last at 12:01, 21 minutes ago). Checked recently: room A.
- **p19** When did you last look in on room B? -> I was last in room B at 11:52 (31 minutes ago).
- **p20** Does a room H exist on your map? -> There is no room H. I know rooms A, B, C and D.

## External questions (10, written by the user)

**0 / 0** correct; routed 0 / 0, content 0 / 0, wrong claims 0, grounding violations 0, questions that used the LLM 0, motion started by a question 0; latency p50 None s, max None s.

| category | ok | n |
|---|---|---|

| id | question | tools | ok | problems |
|---|---|---|---|---|

Replies:

