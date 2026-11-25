> **RUN 1 FAILED (infrastructure): `status=sim_hang` after the world changes (Gazebo stopped publishing; detector frames frozen at 3508), no question was asked. The 0 / 30 below is "not asked", NOT a result. Nothing was changed in code; the evaluation was run again (see `phase6c_incident_eval.md`). Run dir `brain/runs/incident_eval/20261125_104035`.**

# Incident graph evaluation (incident_v3)

Run: 2026-11-25T10:41:11Z commit ece8552, model gurdy-sft-400, falls 0, ground-truth changes verified in Gazebo: True.

## Graph (events vs ground truth)

- moved objects found with the right rooms: 0 / 2; removed object reported missing: 0 / 1; false moved / missing reports for untouched objects: 0

  - red box: found=False delay None s
  - red barrel: found=False delay None s
  - toolbox (removed): found=False in None, delay None s

## Own questions (20, written by the router's author)

**0 / 20** correct; routed 0 / 20, content 0 / 20, wrong claims 0, grounding violations 0, questions that used the LLM 0, motion started by a question 0; latency p50 None s, max None s.

| category | ok | n |
|---|---|---|
| changes | 0 | 6 |
| last_seen | 0 | 4 |
| history | 0 | 4 |
| timeline | 0 | 3 |
| rooms | 0 | 3 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| m01 | Did I miss anything while I was out? |  | NO | not asked |
| m02 | Has room B been rearranged? |  | NO | not asked |
| m03 | Is anything gone from room D? |  | NO | not asked |
| m04 | Is everything in room A where it should be? |  | NO | not asked |
| m05 | What differs in room C compared with before? |  | NO | not asked |
| m06 | Any objects that have vanished altogether? |  | NO | not asked |
| m07 | When did the toolbox last show up on camera? |  | NO | not asked |
| m08 | What's the most recent place you saw the red box? |  | NO | not asked |
| m09 | Do you remember when you spotted the fire extinguisher? |  | NO | not asked |
| m10 | When did you last notice a wheelbarrow? |  | NO | not asked |
| m11 | Is the red barrel in the same spot as at the start? |  | NO | not asked |
| m12 | Has the blue barrel been shifted at all? |  | NO | not asked |
| m13 | Did the toolbox get taken? |  | NO | not asked |
| m14 | Is the red box staying in room A? |  | NO | not asked |
| m15 | Fill me in on the last stretch while I was gone. |  | NO | not asked |
| m16 | What went on in room D today? |  | NO | not asked |
| m17 | What was the reason for you halting after I gave that order? |  | NO | not asked |
| m18 | Which rooms have gone unvisited for more than five minutes? |  | NO | not asked |
| m19 | How recently were you in room C? |  | NO | not asked |
| m20 | Does room G exist? |  | NO | not asked |

Replies:

- **m01** Did I miss anything while I was out? -> 
- **m02** Has room B been rearranged? -> 
- **m03** Is anything gone from room D? -> 
- **m04** Is everything in room A where it should be? -> 
- **m05** What differs in room C compared with before? -> 
- **m06** Any objects that have vanished altogether? -> 
- **m07** When did the toolbox last show up on camera? -> 
- **m08** What's the most recent place you saw the red box? -> 
- **m09** Do you remember when you spotted the fire extinguisher? -> 
- **m10** When did you last notice a wheelbarrow? -> 
- **m11** Is the red barrel in the same spot as at the start? -> 
- **m12** Has the blue barrel been shifted at all? -> 
- **m13** Did the toolbox get taken? -> 
- **m14** Is the red box staying in room A? -> 
- **m15** Fill me in on the last stretch while I was gone. -> 
- **m16** What went on in room D today? -> 
- **m17** What was the reason for you halting after I gave that order? -> 
- **m18** Which rooms have gone unvisited for more than five minutes? -> 
- **m19** How recently were you in room C? -> 
- **m20** Does room G exist? -> 

## External questions (10, written by the user)

**0 / 10** correct; routed 0 / 10, content 0 / 10, wrong claims 0, grounding violations 0, questions that used the LLM 0, motion started by a question 0; latency p50 None s, max None s.

| category | ok | n |
|---|---|---|
| external | 0 | 10 |

| id | question | tools | ok | problems |
|---|---|---|---|---|
| z01 | what's been shuffled around since this morning? |  | NO | not asked |
| z02 | tell me only the things that actually changed |  | NO | not asked |
| z03 | is the barrel still in room D? |  | NO | not asked |
| z04 | I think someone took something, did they? |  | NO | not asked |
| z05 | what did room C look like before versus now? |  | NO | not asked |
| z06 | when did you last pass through room A? |  | NO | not asked |
| z07 | which rooms are overdue for a check? |  | NO | not asked |
| z08 | has the red box gone anywhere? |  | NO | not asked |
| z09 | is there anything in a room F? |  | NO | not asked |
| z10 | quick recap, nothing extra |  | NO | not asked |

Replies:

- **z01** what's been shuffled around since this morning? -> 
- **z02** tell me only the things that actually changed -> 
- **z03** is the barrel still in room D? -> 
- **z04** I think someone took something, did they? -> 
- **z05** what did room C look like before versus now? -> 
- **z06** when did you last pass through room A? -> 
- **z07** which rooms are overdue for a check? -> 
- **z08** has the red box gone anywhere? -> 
- **z09** is there anything in a room F? -> 
- **z10** quick recap, nothing extra -> 
