# Incident graph evaluation (incident_v1)

Run: 2026-11-25T07:27:34Z commit 5407d61, model gurdy-sft-400, falls 0, ground-truth changes verified in Gazebo: True.

## Graph (events vs ground truth)

- moved objects found with the right rooms: 2 / 2; removed object reported missing: 0 / 1; false moved / missing reports for untouched objects: 16

  - red box: found=True delay 329.8 s
  - red barrel: found=True delay 741.4 s
  - toolbox (removed): found=False in None, delay None s
  - FALSE: object_missing gray traffic_cone room B
  - FALSE: object_missing gray traffic_cone room B
  - FALSE: object_missing gray traffic_cone room B
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing red toolbox room D
  - FALSE: object_missing gray box room A

## Own questions (20, written by the router's author)

**3 / 20** correct; routed 18 / 20, content 3 / 20, wrong claims 2, grounding violations 0, questions that used the LLM 0, motion started by a question 0; latency p50 0.96 s, max 30.27 s.

| category | ok | n |
|---|---|---|
| changes | 0 | 6 |
| last_seen | 1 | 4 |
| history | 2 | 4 |
| timeline | 0 | 3 |
| rooms | 0 | 3 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| q01 | Did anything change while I was gone? | what_changed | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| q02 | What has changed in room C since I left? | what_changed | NO | missing /red box/ |
| q03 | Anything new in room A since I left? | what_changed | NO | missing /red box/ |
| q04 | What has changed in room B since I left? | what_changed | NO | missing /barrel/ |
| q05 | Did anything change in room D while I was away? | what_changed | NO | missing /toolbox/; missing /barrel/; must not say /nothing (has )?changed/ |
| q06 | What changed in room Z since I left? |  | NO | missing /(don't|do not) know a room|no such room|not a room/ |
| q07 | When did you last see the toolbox, and where was it? | last_seen | NO | missing /not there|missing|gone/; clock 08:10 outside the ground-truth window |
| q08 | When was the red box last seen? | last_seen | NO | clock 08:22 outside the ground-truth window |
| q09 | When was the mannequin last spotted? | last_seen | NO | clock 08:19 outside the ground-truth window |
| q10 | When did you last see a chair? | last_seen | yes |  |
| q11 | Has the red box been moved? | object_history | yes |  |
| q12 | Did the blue box move at all? | object_history | NO | missing /haven't recorded|no record|not moved|hasn't moved|did not move/ |
| q13 | Has the red barrel shifted? | object_history | yes |  |
| q14 | Is the toolbox still where it was? | object_history | NO | missing /missing|not there|gone/ |
| q15 | Give me a recap of what went on while I was out. | timeline | NO | missing /entered room/ |
| q16 | Show me the event log for room B since I left. | timeline | NO | missing /room B/ |
| q17 | Why did you stop earlier? | incident_why | NO | missing /told me to|you said|stop/; no clock time in the reply |
| q18 | Which rooms haven't you looked at for 5 minutes? | unchecked_rooms | NO | room A was visited recently |
| q19 | When did you last check room C? | unchecked_rooms | NO | no clock time in the reply |
| q20 | When did you last visit room Z? |  | NO | missing /(don't|do not) know a room|no such room|not a room/ |

Replies:

- **q01** Did anything change while I was gone? -> The red box moved from room A to room C. The red barrel moved from room D to room B. The red box was not where I last saw it. The gray traffic cone was not where I last saw it. The red toolbox was not where I last saw it. The red barrel was not where I last saw it. The gray box was not where I last saw it. SAW A NEW GRAY BOX IN ROOM A. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back in room D. The red toolbox is back
- **q02** What has changed in room C since I left? -> Since you left, room C has changed nothing.
- **q03** Anything new in room A since I left? -> Since you left, room A had no changes.
- **q04** What has changed in room B since I left? -> Since you left, room B has changed nothing.
- **q05** Did anything change in room D while I was away? -> Nothing changed in room D while I was away.
- **q06** What changed in room Z since I left? -> Did you mean room B, room C or room D? I heard "room z".
- **q07** When did you last see the toolbox, and where was it? -> I last saw the red toolbox in room D at 08:10 (18 minutes ago).
- **q08** When was the red box last seen? -> I last saw the red box in room C at 08:22 (6 minutes ago).
- **q09** When was the mannequin last spotted? -> I last saw the gray mannequin in room D at 08:19 (10 minutes ago).
- **q10** When did you last see a chair? -> I have no record of that.
- **q11** Has the red box been moved? -> Yes. The red box moved from room A to room C at 08:00 (28 minutes ago).
- **q12** Did the blue box move at all? -> The blue box in room B has been seen ever since.
- **q13** Has the red barrel shifted? -> Yes. The red barrel moved from room D to room B at 08:07 (22 minutes ago).
- **q14** Is the toolbox still where it was? -> The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 21 minutes ago; The red toolbox in room D, 21 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20 minutes ago; The red toolbox in room D, 20
- **q15** Give me a recap of what went on while I was out. -> I have no record of anything while you were away.
- **q16** Show me the event log for room B since I left. -> I can't show that; no data.
- **q17** Why did you stop earlier? -> I ran out of time trying to finish exploring.
- **q18** Which rooms haven't you looked at for 5 minutes? -> The rooms that have not been checked for 5 minutes: room B; room C; room D; and room A.
- **q19** When did you last check room C? -> The last check for room C was 14 minutes ago.
- **q20** When did you last visit room Z? -> Did you mean room B, room C or room D? I heard "room z".

## External questions (10, written by the user)

**3 / 10** correct; routed 2 / 10, content 3 / 10, wrong claims 3, grounding violations 0, questions that used the LLM 8, motion started by a question 0; latency p50 6.31 s, max 10.8 s.

| category | ok | n |
|---|---|---|
| external | 3 | 10 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| x01 | did anything go missing while you were looking around? | recall | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| x02 | where's the barrel now, wasn't it somewhere else before? | where_is | yes |  |
| x03 | is room C any different from the first time you checked it? |  | NO | missing /red box/ |
| x04 | which things got moved? | recall | NO | must not say /blue box|yellow box|cone|mannequin|extinguisher|blue barrel/ |
| x05 | have you spotted the toolbox recently? | recall | NO | missing /not there|missing|gone|not since|no longer|haven't|have not/; no clock time in the reply |
| x06 | what's new in room B? | what_changed | yes |  |
| x07 | tell me everything that happened after I said stop | why | NO | missing /room/ |
| x08 | is the red box still where you first saw it? | object_history | NO | must not say /^\W*yes/ |
| x09 | which room did you visit last? | recall | yes |  |
| x10 | anything unusual today? | describe_view | NO | missing /toolbox|red box|barrel/ |

Replies:

- **x01** did anything go missing while you were looking around? -> I recall: fire extinguisher in room C; red box in room A; traffic cone in room B; barrel in room B; traffic cone in room A; box in room B; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A
- **x02** where's the barrel now, wasn't it somewhere else before? -> The red barrel in room B, 4.0 metres left; The blue barrel in room B, 5.0 metres left; The red barrel in room D, 7.0 metres behind-left.
- **x03** is room C any different from the first time you checked it? -> I can't do that. I don't know what a room is, and I have not seen one.
- **x04** which things got moved? -> The following objects were moved: fire extinguisher in room C; red box in room A; traffic cone in room B; barrel in room B; traffic cone in room A; box in room B; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box in room A; mannequin in room D; toolbox in room D; barrel in room D; traffic cone in room D; box in room C; box
- **x05** have you spotted the toolbox recently? -> red toolbox in room D.
- **x06** what's new in room B? -> In room B: The red barrel moved from room D to room B. The gray traffic cone was not where I last saw it. Saw a new gray traffic cone in room B. Saw a new blue barrel in room B. Saw a new blue box in room B. The gray traffic cone is back in room B. The gray traffic cone is back in room B. The gray traffic cone is back in room B.
- **x07** tell me everything that happened after I said stop -> explore started. Done.
- **x08** is the red box still where you first saw it? -> Yes. The red box moved from room A to room C at 08:00 (31 minutes ago).
- **x09** which room did you visit last? -> home; room A; room B; room C; room D.
- **x10** anything unusual today? -> I do not see any known objects right now.

---
# Analysis (written after the run, 2026-11-25; one single run, no variance; the router was NOT changed after seeing any result)

**Setup facts.** Eval world = `lab4_incident.sdf` (copy of lab4 with red box / red barrel / toolbox dynamic; `lab4.sdf` untouched). All three Gazebo operations were accepted and **verified by reading the pose back** (`gt.ops[*].verified` = true: both moves landed within 5 cm, toolbox gone from the pose stream). 0 falls, no agent errors. 514 graph events, 2407 edges. Run dir `brain/runs/incident_eval/20261125_072639/`.

## Graph
- Both moves were found with the right rooms (red box A->C after 330 s, red barrel D->B after 741 s: the delay is the time until the robot's round-2 look-around reached the new or old spot).
- **Toolbox removal: not detected as scored (0 / 1), but this is partly a scorer defect.** The detector calls the toolbox "red toolbox", the ground-truth key was `("toolbox", "")`, so its 12 `object_missing` events were counted as FALSE reports. Colour-agnostic matching would give removed object reported missing 1 / 1 (first at +746 s) and false reports 16 -> 4 (3 x "gray traffic cone missing in room B", 1 x "gray box missing in room A": detector class / colour flips). The as-run numbers above are kept unchanged; the scorer key is not fixed yet.
- **More serious, found in the events:** the graph logged `object_seen` for the toolbox every ~60 s **throughout the whole run, including from rooms C and A where it cannot be in view, and after it had been removed (until +1022 s)**. The semantic-map track's `last_seen` is only refreshed by a detection (`semantic_map.py:107`), so either the detector keeps producing a "toolbox" detection at that spot (false positive / projection) or the removed model is still rendered to the camera; I did not investigate which. Effect: "last seen" times are wrong (q07 said 08:10, 18 min ago, for an object removed ~25 min earlier... it was never really gone from the map), the missing flag flaps (12 missing events, interleaved with "is back"), and `x06` printed "gray traffic cone ... is back" 3 times. **The missing-object logic is not trustworthy on the real detector output; the unit tests used a clean mock.**

## Own questions: 3 / 20 (routing 18 / 20, content 3 / 20)
Root causes seen in the replies (not fixed):
1. **"since I left / while I was away" window = since the PREVIOUS user command.** In this protocol every question is itself a command, so after q01 the window became "since q01": q02-q05, q15, q16 answered "nothing changed" / "no record" although q01 (first question, correct window) listed both moves. Design flaw of the `away` definition for rapid question sequences, not noise.
2. **Room Z (q06, q20):** the room-name repair ("Did you mean room B, room C or room D?") runs before the incident router, so the honest "I don't know a room called ..." is never reached.
3. **q17 "Why did you stop earlier?"** answered with the latest stop-like event (an explore timeout) instead of the typed stop 25 minutes before.
4. **q18 / q19:** "every room not checked" (room A was listed although the robot stood in it: the room-node time is only updated while polling in the room and the last poll before the question was stale) and q19 gave only "14 minutes ago" without a clock time (the reply came from the planner path, not the template).
5. q12 "The blue box in room B has been seen ever since." and q14 (a repeated list) are planner / `recall` answers: those two questions did not reach the history template as intended.
Passes: q10 (no chair), q11, q13 (both moves, with rooms and clock times).

## External questions (user-written): 3 / 10 (routing 2 / 10 by my informative tool list, content 3 / 10, wrong claims 3)
8 of 10 went to the fine-tuned planner (the router recognised only x06 and x08), which mostly called `recall` / `where_is` / `describe_view` / `why` and printed the whole object memory ("I recall: ...", 3 wrong-claim hits: untouched objects listed as moved / missing) or said "I do not see any known objects right now" (x10) or "explore started. Done." (x07). Passes: x02 (barrel now in room B, was in D, via `where_is`), x06, x09. x08 got a "Yes." for a moved object (history template says "Yes. ... moved" to a question phrased the other way round: correct content, scorer expected a "no"; that is my expectation error, counted as a failure as written). The router was deliberately not tuned on these.

## Latency
Own questions p50 0.96 s, max 30.3 s (one planner-path answer); external p50 6.3 s, max 10.8 s (8 of 10 used the LLM). Template-routed answers cost no LLM call.

## Honest summary
The recording of moves works on real Gazebo data (2 / 2, right rooms, no motion false positives for the moved objects). Everything built on **missing** objects and **last-seen** times is unreliable because the detector keeps refreshing a removed object, and the question side is weak: the router covers few phrasings, and the `away` window and room-repair order are real bugs. Nothing here was re-run or tuned; fixes belong in a follow-up with a new held-out question set.
