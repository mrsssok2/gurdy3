# OFFLINE REPLAY (ollama, picker off, run brain/runs/incident_eval/20261125_111853)

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

**4 / 20** correct; routed 5 / 20, content 8 / 20, wrong claims 1, grounding violations 0, questions that used the LLM 13, motion started by a question 0; latency p50 0.43 s, max 0.59 s.

| category | ok | n |
|---|---|---|
| changes | 1 | 6 |
| last_seen | 1 | 4 |
| history | 1 | 4 |
| timeline | 0 | 3 |
| rooms | 1 | 3 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| p01 | Were any of the items swapped around while I was gone? | what_changed | yes |  |
| p02 | Has the contents of room C been altered? |  | NO | missing /red box/ |
| p03 | Something feels off in room D, is it? | explore | NO | missing /toolbox|barrel/ |
| p04 | Has room A lost anything since the start? |  | NO | missing /red box/ |
| p05 | Did anything go astray? | why | NO | missing /toolbox/ |
| p06 | Give me the differences between now and when you began. | what_changed | NO | must not say /blue box|yellow box|mannequin|extinguisher|blue barrel/ |
| p07 | At what time did you last have the toolbox in sight? | recall | NO | missing /toolbox/; missing /not there|missing|gone/; no clock time in the reply |
| p08 | Where did the red box show up most recently? | last_seen | yes |  |
| p09 | How long has the mannequin been out of your view? | recall | NO |  |
| p10 | When was a ladder last on your camera? |  | NO |  |
| p11 | Has the red barrel stayed in room D? | where_is | NO | missing /^\W*no/ |
| p12 | Was the blue barrel ever shoved somewhere else? | where_is | NO | missing /haven't recorded|no record|not moved|hasn't moved|did not move/ |
| p13 | Is the red box in the spot where it began? | find | NO | missing /^\W*no/; missing /room C/ |
| p14 | Did the toolbox vanish? | object_history | yes |  |
| p15 | Bring me up to date on what's been going on in room C. |  | NO |  |
| p16 | What have you been busy with since I gave you that first order? | recall | NO | missing /entered room/ |
| p17 | What led to you coming to a halt after my command? | why | NO | missing /told me to|you said|stop/ |
| p18 | Which rooms have you left alone for over ten minutes? | recall | NO | room B should be listed (not visited for 600 s); room D should be listed (not visited for 600 s); room C should be listed (not visited for 600 s) |
| p19 | When did you last look in on room B? | unchecked_rooms | yes |  |
| p20 | Does a room H exist on your map? |  | NO |  |

Replies:

- **p01** Were any of the items swapped around while I was gone? -> While you were away: 8 changes. The red barrel moved from room D to room B at 11:50 (31 minutes ago); the red box moved from room A to room C at 12:03 (18 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (28 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (45 minutes ago); the blue barrel is back in room B at 11:39 (43 minutes ago); the gray mannequin is back in room D at 11:53 (28 minutes ago); and 2 more.
- **p02** Has the contents of room C been altered? -> Which kind of object do you mean?
- **p03** Something feels off in room D, is it? -> explore is not available in a replay
- **p04** Has room A lost anything since the start? -> Which kind of object do you mean?
- **p05** Did anything go astray? -> I can't answer that.
- **p06** Give me the differences between now and when you began. -> Since you started me: 8 changes. The red barrel moved from room D to room B at 11:50 (32 minutes ago); the red box moved from room A to room C at 12:03 (19 minutes ago); the red toolbox is missing from room D; I looked there at 11:53 (29 minutes ago) and it was not there; a new gray box appeared in room B at 11:36 (46 minutes ago); the blue barrel is back in room B at 11:39 (43 minutes ago); the gray mannequin is back in room D at 11:53 (29 minutes ago); and 2 more.
- **p07** At what time did you last have the toolbox in sight? -> I have no record of that.
- **p08** Where did the red box show up most recently? -> I last saw the red box in room C at 12:03 (19 minutes ago).
- **p09** How long has the mannequin been out of your view? -> gray mannequin in room D.
- **p10** When was a ladder last on your camera? -> I can't do that. I don't know what a ladder is, and I have not seen one.
- **p11** Has the red barrel stayed in room D? -> The red barrel in room B, 0.0 metres ahead.
- **p12** Was the blue barrel ever shoved somewhere else? -> The blue barrel in room B.
- **p13** Is the red box in the spot where it began? -> find is not available in a replay
- **p14** Did the toolbox vanish? -> I haven't recorded the red toolbox moving. I have seen it 2 times so far. I last recorded it in room D. It is currently marked missing: it was not there when I checked.
- **p15** Bring me up to date on what's been going on in room C. -> I can tell you what changed in room C, when I was last there, or what happened there. Which of those do you want?
- **p16** What have you been busy with since I gave you that first order? -> I have no record of any missions.
- **p17** What led to you coming to a halt after my command? -> I can't answer that.
- **p18** Which rooms have you left alone for over ten minutes? -> I have no record of that.
- **p19** When did you last look in on room B? -> I was last in room B at 11:52 (31 minutes ago).
- **p20** Does a room H exist on your map? -> There is no room H. I know rooms A, B, C and D.

## External questions (10, written by the user)

**0 / 0** correct; routed 0 / 0, content 0 / 0, wrong claims 0, grounding violations 0, questions that used the LLM 0, motion started by a question 0; latency p50 None s, max None s.

| category | ok | n |
|---|---|---|

| id | question | tools | ok | problems |
|---|---|---|---|---|

Replies:

