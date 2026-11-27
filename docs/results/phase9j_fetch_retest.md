# Fetch retest (20261127_114257; scripts/fetch_retest.sh)

Target: blue can, GT (-1.30, 0.30), 1.33 m from the spawn point; command "pick up the blue can"; single tries.

| variant | what | success (sim GT) | robot claimed | grasp attempts |
|---|---|---|---|---|
| C | full fetch: search + Nav2 + detector / servo pose | 1/1 | 1/1 | 1 |

## Variant C: full fetch: search + Nav2 + detector / servo pose

| try | success (sim GT) | wall s | failure reason | robot said | GT evidence |
|---|---|---|---|---|---|
| 1 | yes | 253 | - | I picked up the blue can in room A and I'm holding it with the arm folded. | x/y only: can moved 0.55 m from its rest, ends 0.048 m from the folded grasp centre (base 0.174, -0.011) |

| attempt | outcome | reason | est. source | robot->obj (m) | est. error vs GT (m) | plan | centre shift (m) | settle pre / grasp (rad) | fingers closed / lifted (m) | target in scene |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | success |  | servo | 0.369 | 0.0088 | ok | 0.0026 | 0.0095 / 0.0075 | [0.032927496290495066, 0.03311991879712207] / [0.03783599972205537, 0.02821061111746929] | True |


## Notes (verified from simulation ground truth, not from the replies)
| try | success (sim GT) | time | failure reason |
|---|---|---|---|
| C1 | **yes** | 253 s wall (fresh stack: search, Nav2 approach, servo, grasp, lift, fold) | - |
| C2 | **not run** | stack start gave up after 3 x ~2 min (11:56-12:02 UTC) | infra: `lib_nav_start.sh` readiness gate raced (see below); no robot motion |

**C1 evidence.** (1) Lifted: GT x / y only (this run's `gt_pose` did not log z): after the lift + view-pose move the can stood at
base (-0.194, -0.013), 17 cm from its rest spot (-0.36) and 4.5 cm from the view-pose grasp centre (-0.239): it moved WITH the arm;
at the end it was at base (0.174, -0.011) = 4.8 cm from the folded grasp centre (0.127, 0): over the robot body, where a can
standing on the floor cannot be (base_link is 0.11 m above the floor and the can is 0.12 m tall, so the body is in the way). (2) Between the fingers after the
lift: sim finger joints 38 / 28 mm, opening 66 mm = the can's 66 mm diameter, neither closed. (3) Held at the end: GT position
at the folded grasp centre when the run ended (one sample). Detector + servo estimate 8.8 mm from GT; robot -> can 0.37 m at the
grasp; 1 attempt; centring shift 2.6 mm (re-plan ok); target obstacle in MoveIt; the search found the can and moved on (the
wrapper-tick fix works live). Height was not measured: `fetch_retest.sh` now logs GT z and samples the can 5 s after the reply.
**C2 start failure, cause found:** `lib_nav_start.sh` waited for EXACTLY 2 active controllers; with `arm:=true` there are 4 (the
arm + gripper controllers come up ~2.4 s after the leg controller in both runs). Run 1's check fell inside that window, run 2's
did not (all 4 activated, "controllers active=0" three times). Also explains this afternoon's one "Nav2 never became ready" in
sim_moveit. Fixed: `-ge 2` (no-arm stacks unchanged). Not re-run (Docker belongs to the humanoid).
Report fixes: raw JSON reply unwrapped, success judged from GT (`gt_verdict`, 6 tests incl. "claimed but still on the floor").
