# Overnight 2026-11-27 reliability run (docs/results/data/overnight_1127_smoke)

Run: 12:53:54 - 12:59:37 UTC (run.log). Success is judged from Gazebo ground truth, NOT from the robot's reply.

## a) Full fetch (search + Nav2 + detector), fresh stack per try

| try | class | distance (m) | success (sim GT) | time (s) | failure reason | robot said |
|---|---|---|---|---|---|---|
| can_1 | can | 1.2 | yes | 186 | - | I picked up the blue can in room A and I'm holding it with the arm folded. |

| class | success (sim GT) | rate |
|---|---|---|
| can | 1/1 | 100 % |
| **all** | **1/1** (of 1 planned) | 100 % |

Robot claimed success where ground truth says no: 0.

## b) Variants A (teleport + detector pose) and B (Nav2 + GT pose), blue can at its GT spot

Not run.

## c) Walking variance with the arm

Not run.

## d) YOLO v3.1 (render -> train -> eval), v3 vs v3.1; runtime detector NOT switched

Not run.

## Blocks, skips and failures

| event | details |
|---|---|
| - | no skipped block, no failed start |

What failed and why:

* nothing
