# "Bring me the blue can" takes, 2026-11-28 (scripts/demo_bring_gui.sh, shared GPU lock held per take, GT-graded)

| take | mode | GT result | reply | cause |
|---|---|---|---|---|
| gui 1 (06:48) | GUI, RTF ~0.2 | can never touched | "I couldn't get a grip ... after 3 attempts. It is still where it was." | arm never reached the pre-grasp: MoveIt cannot plan that goal (every run), the direct fallback waited in WALL time (`seconds + 10`) = ~2.6 s sim < 3 s trajectory. Hold check never ran. Fixed: sim-time waits (931cc05) |
| headless 1 (06:58) | headless | can STILL IN THE GRIPPER (z 0.245) | "I dropped the blue can on the way." (FALSE) | pick confirmed by the new hold check (33/33 mm, payload ratio 1.06); the carry drop check compared GT xy with the localisation pose while walking. Fixed: dropped = carried model on the floor (720a741) |
| headless 2 (07:07) | headless | grasp missed x3, can knocked over (z 0.033) | "couldn't get a grip ..." | real miss (fingers 0/0 x3); hold check MISSED / no weight, correct |
| **headless 3 (07:19)** | headless | **delivered**: carried 1.61 m, on the floor 0.30 m from the requester's start | **"Here's your blue can."** | hold check HOLDING (33/33 mm, payload ratio 1.06), 769 s |
| gui 2 (07:35) | GUI, RTF ~0.2 | grasp missed x3, can knocked 0.2 m, lying | "I couldn't get a grip on the blue can after 3 attempts. It is still where it was." (2nd sentence FALSE) | arm moves now work at RTF 0.2; real miss: attempt 1 closed on nothing at 1.2 cm estimate error and pushed the can, attempts 2-3 re-used the stale estimate (11 / 15 cm off). Reply fixed: no claim about where it is after a miss |

Open (not fixed): (1) re-grasp re-uses the old position estimate after a miss that moved the object -> re-align before each
re-grasp; (2) grasp success per try is ~2/3 for the can in the full fetch (today: smoke b 3/3, e 3/3 GT; takes: 2 of 4 grasps).

## After the grasp-reliability fixes (dd8b67a): 3 headless takes, one lock hold (08:04-08:30 UTC)

| take | GT result | reply | details |
|---|---|---|---|
| r1 | **delivered**: carried 1.64 m, standing on the floor 0.32 m from the requester's start | "Here's your blue can." | 1 attempt; fingers open 40/40 before planning, pre-grasp via MoveIt, settle 0.009 rad, sideways 1.7 mm, closed 33/33, hold HOLDING (ratio 1.06); 595 s |
| r2 | not picked, can never touched (moved 0.000 m) | "I found the blue can but I couldn't line my gripper up with it (residual 11 cm after 3 tries). ..." | alignment NOT_ALIGNED (11 cm, measured) then OCCLUDED x2; no grasp attempt; 216 s |
| r3 | **delivered**: carried 1.54 m, standing 0.21 m from the requester's start | "Here's your blue can." | 1 attempt; open 40/40, MoveIt, settle 0.010 rad, sideways 0.9 mm, closed 33/33, HOLDING (1.05); 339 s |

MoveIt: 0 x "Unable to sample any valid states" in all three (before: once in every run), every big move planned and
executed (5 / 3 / 5). Both grasps held on the first attempt. Remaining failure (r2): the approach + wrist servo ended with the
can 11 cm off and then judged the view 'implausible' twice -- an approach / servo problem before any grasp (also seen in
smoke_1128c can_1 and d can_2), not fixed here.
