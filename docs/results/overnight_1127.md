# Overnight 2026-11-27 reliability run (docs/results/data/overnight_1127)

> **Run cut off by power loss at 02:36-02:42 UTC on 2026-11-28 (08:06-08:12 IST; last file write 02:36, laptop back up 02:42), during block d training epoch 136.** Everything below comes from the data saved before the cut; nothing was re-run except where a section says so.

Run: 18:05:46 - 00:43:11 UTC (run.log). Success is judged from Gazebo ground truth, NOT from the robot's reply.

## a) Full fetch (search + Nav2 + detector), fresh stack per try

| try | class | distance (m) | success (sim GT) | time (s) | failure reason | robot said |
|---|---|---|---|---|---|---|
| small_box_1 | small_box | 1.06 | no | 894 | no grasp attempt; robot said: I don't know any red cube, and I won't guess another object. | I don't know any red cube, and I won't guess another object. |
| small_box_2 | small_box | 1.91 | no | 525 | not lifted off the floor; not held 5 s after the reply [last attempt: missed: fingers wider than the object] | I couldn't get a grip on the red cube after 3 attempts. It is still where it was |
| small_box_3 | small_box | 2.22 | no | 810 | no grasp attempt; robot said: I don't know any red cube, and I won't guess another object. | I don't know any red cube, and I won't guess another object. |
| mini_toolbox_1 | mini_toolbox | 1.03 | no | 778 | no grasp attempt; robot said: I don't know any yellow mini toolbox, and I won't guess another object. | I don't know any yellow mini toolbox, and I won't guess another object. |
| mini_toolbox_2 | mini_toolbox | 1.8 | no | 138 | no grasp attempt; robot said: I found the yellow mini toolbox but I couldn't line my gripper up with it (residual 99 cm after 3 tries). Can you move i | I found the yellow mini toolbox but I couldn't line my gripper up with it (resid |
| mini_toolbox_3 | mini_toolbox | 2.3 | no | 763 | no grasp attempt; robot said: I don't know any yellow mini toolbox, and I won't guess another object. | I don't know any yellow mini toolbox, and I won't guess another object. |
| can_1 | can | 1.2 | yes | 189 | - | I picked up the blue can in room A and I'm holding it with the arm folded. |
| can_2 | can | 1.7 | no | 257 | not between the fingers after the lift; not held 5 s after the reply [last attempt: missed: fingers fully closed: nothing between them] | I couldn't get a grip on the blue can after 3 attempts. It is still where it was |
| can_3 | can | 2.4 | yes | 445 | - | I picked up the blue can in room B and I'm holding it with the arm folded. |
| mini_cone_1 | mini_cone | 1.17 | no | 514 | no grasp attempt; robot said: I don't know any orange mini cone, and I won't guess another object. | I don't know any orange mini cone, and I won't guess another object. |
| mini_cone_2 | mini_cone | 1.65 | no | 517 | no grasp attempt; robot said: I don't know any orange mini cone, and I won't guess another object. | I don't know any orange mini cone, and I won't guess another object. |
| mini_cone_3 | mini_cone | 2.34 | no | 511 | no grasp attempt; robot said: I don't know any orange mini cone, and I won't guess another object. | I don't know any orange mini cone, and I won't guess another object. |
| mini_extinguisher_1 | mini_extinguisher | 1.3 | no | 164 | not lifted off the floor; not held 5 s after the reply [last attempt: missed: fingers wider than the object] | I couldn't get a grip on the red mini extinguisher after 3 attempts. It is still |
| mini_extinguisher_2 | mini_extinguisher | 1.65 | no | 185 | no grasp attempt; robot said: I found the red mini extinguisher but I couldn't line my gripper up with it (residual 99 cm after 3 tries). Can you move | I found the red mini extinguisher but I couldn't line my gripper up with it (res |
| mini_extinguisher_3 | mini_extinguisher | 2.4 | no | 357 | not lifted off the floor; not held 5 s after the reply [last attempt: missed: fingers wider than the object] | I couldn't get a grip on the red mini extinguisher after 3 attempts. It is still |

| class | success (sim GT) | rate |
|---|---|---|
| small_box | 0/3 | 0 % |
| mini_toolbox | 0/3 | 0 % |
| can | 2/3 | 67 % |
| mini_cone | 0/3 | 0 % |
| mini_extinguisher | 0/3 | 0 % |
| **all** | **2/15** (of 15 planned) | 13 % |

Robot claimed success where ground truth says no: 0.

## b) Variants A (teleport + detector pose) and B (Nav2 + GT pose), blue can at its GT spot

**A** (teleport to the grasp_test pose + detector / servo pose (grasp pipeline only))

| try | success (sim GT) | time (s) | failure reason |
|---|---|---|---|
| 1 | yes | 104 | - |
| 2 | no | 17 | no grasp attempt; robot said: Something went wrong with that fetch request. |
| 3 | no | 0 | no grasp attempt; robot said: I don't know any blue can, and I won't guess another object. |

**B** (Nav2 approach + ground-truth object pose (approach only))

| try | success (sim GT) | time (s) | failure reason |
|---|---|---|---|
| 1 | no | 102 | not lifted off the floor; not between the fingers after the lift (no data); not held 5 s after the reply [last attempt: missed: no grasp plan: no feasible grasp (limits / collisions) at this xy] |
| 2 | yes | 107 | - |
| 3 | no | 76 | no grasp attempt; robot said: I found the blue can but I couldn't line my gripper up with it (residual 99 cm after 3 tries). Can you move it into the  |

## c) Walking variance with the arm

| config | runs | forward error mean ± SD (min-max) | turn error mean ± SD (min-max) | falls | arm max deviation (rad) |
|---|---|---|---|---|---|
| no arm | 10 | 10.2 ± 2.7 (5.1-13.4) % | 2.7 ± 0.4 (2.4-3.3) % | 0/10 | - |
| arm folded | 10 | 12.9 ± 3.7 (7.4-20.0) % | 2.4 ± 0.7 (1.2-3.2) % | 0/10 | max 0.049, mean 0.046 |
| arm folded + 80 g | 10 | 14.0 ± 5.0 (3.6-17.8) % | 2.9 ± 0.4 (2.1-3.3) % | 0/10 | max 0.079, mean 0.073 |

* arm folded vs no arm, forward: +2.8 points (95 % ~ -0.1 .. +5.6); acceptance +5 points: undecided
* arm folded vs no arm, turn: -0.3 points (95 % ~ -0.8 .. +0.2); acceptance +5 points: met
* arm folded + 80 g vs no arm, forward: +3.8 points (95 % ~ +0.3 .. +7.4); acceptance +5 points: undecided
* arm folded + 80 g vs no arm, turn: +0.2 points (95 % ~ -0.1 .. +0.5); acceptance +5 points: met

Walk runs: 30 with a result of 30 started.

## d) YOLO v3.1 (render -> train -> eval), v3 vs v3.1; runtime detector NOT switched

| stage | exit code | wall (min) | training budget (h) |
|---|---|---|---|
| render | 0 | 117 | 0 |
| train | cut off (power loss) | - | - |
| eval | cut off (power loss) | - | - |
| report | cut off (power loss) | - | - |

det_v31_s trained 136 epochs (last epoch 137, 2 session(s)) in 1.88 h (det_v3_s: 104 epochs in 2.09 h, stopped by the power cut); best epoch 113: val mAP50 0.952, mAP50-95 0.875 (val sets differ: v3.1 labels include the tiny objects).

**Acceptance, side by side (scripts/p9b_report.py)**

| check | target | det_v2_s | det_v3_s | det_v31_s |
|---|---|---|---|---|
| old classes mAP50, synth_v1 test | >= det_v2_s - 0.01 | 0.922 | 0.933 PASS | 0.928 PASS |
| old classes mAP50, synth_v2 test | >= det_v2_s - 0.01 | 0.871 | 0.875 PASS | 0.863 PASS |
| small classes mAP50, synth_v3_wrist test | >= 0.80 | - | 0.991 PASS | 0.988 PASS |
| small classes mAP50, synth_v3_body test | >= 0.70 | - | 0.974 PASS | 0.970 PASS |
| small classes mAP50, synth_v31_wrist test | >= 0.80 | - | 0.986 PASS | 0.984 PASS |
| small classes mAP50, synth_v31_body test | >= 0.70 | - | 0.832 PASS | 0.904 PASS |
| **acceptance (old + v3 sets)** | | | **PASS** | **PASS** |

Per small class, synth_v3_body test (AP50): det_v3_s: small_box 0.971, mini_toolbox 0.960, can 0.972, mini_cone 0.985, mini_extinguisher 0.980; det_v31_s: small_box 0.933, mini_toolbox 0.991, can 0.964, mini_cone 0.982, mini_extinguisher 0.979
Per small class, synth_v3_wrist test (AP50): det_v3_s: small_box 0.978, mini_toolbox 0.990, can 1.000, mini_cone 0.985, mini_extinguisher 1.000; det_v31_s: small_box 0.978, mini_toolbox 0.990, can 1.000, mini_cone 0.972, mini_extinguisher 1.000

**Recall by distance, det_v3_s**

Recall by distance, small classes, model `models/detector/det_v3_s.onnx`, split test, conf 0.35 (vision/recall_by_distance.py)

**datasets/synth_v3_body (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 256 | 253 | 0 | 3 | 98.4 % | 99.6 % | - |
| 1-2 m | 317 | 292 | 8 | 17 | 89.9 % | 96.2 % | 37.5 % |
| 2-3 m | 90 | 53 | 23 | 14 | 45.6 % | 75.5 % | 0.0 % |
| > 3 m | 188 | 7 | 172 | 9 | 2.7 % | 42.9 % | 1.2 % |
| all | 851 | 605 | 203 | 43 | 68.5 % | 95.2 % | 2.5 % |

**datasets/synth_v3_wrist (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 456 | 454 | 0 | 2 | 98.9 % | 99.3 % | - |
| 1-2 m | 32 | 6 | 4 | 22 | 12.5 % | 66.7 % | 0.0 % |
| 2-3 m | 34 | 1 | 21 | 12 | 2.9 % | 100.0 % | 0.0 % |
| > 3 m | 66 | 1 | 65 | 0 | 1.5 % | 100.0 % | 0.0 % |
| all | 588 | 462 | 90 | 36 | 77.7 % | 98.9 % | 0.0 % |

**datasets/synth_v31_body (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 256 | 253 | 0 | 3 | 98.4 % | 99.6 % | - |
| 1-2 m | 327 | 305 | 5 | 17 | 88.1 % | 94.4 % | 0.0 % |
| 2-3 m | 94 | 75 | 8 | 11 | 44.7 % | 56.0 % | 0.0 % |
| > 3 m | 194 | 104 | 85 | 5 | 3.1 % | 5.8 % | 0.0 % |
| all | 871 | 737 | 98 | 36 | 67.5 % | 79.8 % | 0.0 % |

**datasets/synth_v31_wrist (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 456 | 454 | 0 | 2 | 98.9 % | 99.3 % | - |
| 1-2 m | 32 | 6 | 4 | 22 | 12.5 % | 66.7 % | 0.0 % |
| 2-3 m | 34 | 1 | 21 | 12 | 2.9 % | 100.0 % | 0.0 % |
| > 3 m | 66 | 1 | 65 | 0 | 1.5 % | 100.0 % | 0.0 % |
| all | 588 | 462 | 90 | 36 | 77.7 % | 98.9 % | 0.0 % |

**Recall by distance, det_v31_s**

Recall by distance, small classes, model `models/detector/det_v31_s.onnx`, split test, conf 0.35 (vision/recall_by_distance.py)

**datasets/synth_v3_body (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 256 | 253 | 0 | 3 | 98.4 % | 99.6 % | - |
| 1-2 m | 317 | 292 | 8 | 17 | 91.5 % | 97.6 % | 50.0 % |
| 2-3 m | 90 | 53 | 23 | 14 | 65.6 % | 90.6 % | 39.1 % |
| > 3 m | 188 | 7 | 172 | 9 | 25.0 % | 100.0 % | 21.5 % |
| all | 851 | 605 | 203 | 43 | 76.1 % | 97.9 % | 24.6 % |

**datasets/synth_v3_wrist (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 456 | 454 | 0 | 2 | 98.9 % | 99.3 % | - |
| 1-2 m | 32 | 6 | 4 | 22 | 12.5 % | 66.7 % | 0.0 % |
| 2-3 m | 34 | 1 | 21 | 12 | 2.9 % | 100.0 % | 0.0 % |
| > 3 m | 66 | 1 | 65 | 0 | 1.5 % | 100.0 % | 0.0 % |
| all | 588 | 462 | 90 | 36 | 77.7 % | 98.9 % | 0.0 % |

**datasets/synth_v31_body (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 256 | 253 | 0 | 3 | 98.4 % | 99.6 % | - |
| 1-2 m | 327 | 305 | 5 | 17 | 90.8 % | 97.4 % | 0.0 % |
| 2-3 m | 94 | 75 | 8 | 11 | 63.8 % | 80.0 % | 0.0 % |
| > 3 m | 194 | 104 | 85 | 5 | 24.2 % | 45.2 % | 0.0 % |
| all | 871 | 737 | 98 | 36 | 75.3 % | 89.0 % | 0.0 % |

**datasets/synth_v31_wrist (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 456 | 454 | 0 | 2 | 98.9 % | 99.3 % | - |
| 1-2 m | 32 | 6 | 4 | 22 | 12.5 % | 66.7 % | 0.0 % |
| 2-3 m | 34 | 1 | 21 | 12 | 2.9 % | 100.0 % | 0.0 % |
| > 3 m | 66 | 1 | 65 | 0 | 1.5 % | 100.0 % | 0.0 % |
| all | 588 | 462 | 90 | 36 | 77.7 % | 98.9 % | 0.0 % |

Runtime detector default after the block: `det_v3_s.onnx` (unchanged)

## Blocks, skips and failures

| event | details |
|---|---|
| - | no skipped block, no failed start |

What failed and why:

* fetch small_box_1: no grasp attempt; robot said: I don't know any red cube, and I won't guess another object.
* fetch small_box_2: not lifted off the floor; not held 5 s after the reply [last attempt: missed: fingers wider than the object]
* fetch small_box_3: no grasp attempt; robot said: I don't know any red cube, and I won't guess another object.
* fetch mini_toolbox_1: no grasp attempt; robot said: I don't know any yellow mini toolbox, and I won't guess another object.
* fetch mini_toolbox_2: no grasp attempt; robot said: I found the yellow mini toolbox but I couldn't line my gripper up with it (residual 99 cm after 3 tries). Can you move i
* fetch mini_toolbox_3: no grasp attempt; robot said: I don't know any yellow mini toolbox, and I won't guess another object.
* fetch can_2: not between the fingers after the lift; not held 5 s after the reply [last attempt: missed: fingers fully closed: nothing between them]
* fetch mini_cone_1: no grasp attempt; robot said: I don't know any orange mini cone, and I won't guess another object.
* fetch mini_cone_2: no grasp attempt; robot said: I don't know any orange mini cone, and I won't guess another object.
* fetch mini_cone_3: no grasp attempt; robot said: I don't know any orange mini cone, and I won't guess another object.
* fetch mini_extinguisher_1: not lifted off the floor; not held 5 s after the reply [last attempt: missed: fingers wider than the object]
* fetch mini_extinguisher_2: no grasp attempt; robot said: I found the red mini extinguisher but I couldn't line my gripper up with it (residual 99 cm after 3 tries). Can you move
* fetch mini_extinguisher_3: not lifted off the floor; not held 5 s after the reply [last attempt: missed: fingers wider than the object]
* variant A try 2: no grasp attempt; robot said: Something went wrong with that fetch request.
* variant A try 3: no grasp attempt; robot said: I don't know any blue can, and I won't guess another object.
* variant B try 1: not lifted off the floor; not between the fingers after the lift (no data); not held 5 s after the reply [last attempt: missed: no grasp plan: no feasible grasp (limits / collisions) at this xy]
* variant B try 3: no grasp attempt; robot said: I found the blue can but I couldn't line my gripper up with it (residual 99 cm after 3 tries). Can you move it into the 
* yolo train: cut off by the power loss
* yolo eval: cut off by the power loss
* yolo report: cut off by the power loss

Failure reasons (count): not lifted off the floor; not held 5 s after the reply x3; no grasp attempt; robot said: I don't know any orange mini cone, and I won't guess another object. x3; cut off by the power loss x3; no grasp attempt; robot said: I don't know any red cube, and I won't guess another object. x2; no grasp attempt; robot said: I don't know any yellow mini toolbox, and I won't guess another object. x2; no grasp attempt; robot said: I found the yellow mini toolbox but I couldn't line my gripper up with it (residual 99 cm after 3 tries). Can you move i x1; not between the fingers after the lift; not held 5 s after the reply x1; no grasp attempt; robot said: I found the red mini extinguisher but I couldn't line my gripper up with it (residual 99 cm after 3 tries). Can you move x1; no grasp attempt; robot said: Something went wrong with that fetch request. x1; no grasp attempt; robot said: I don't know any blue can, and I won't guess another object. x1; not lifted off the floor; not between the fingers after the lift (no data); not held 5 s after the reply x1; no grasp attempt; robot said: I found the blue can but I couldn't line my gripper up with it (residual 99 cm after 3 tries). Can you move it into the  x1

## d) after the power loss: resumed 2026-11-28 (`scripts/p9b_v31_resume.sh`)

* Saved state at the cut: `results.csv` complete to **epoch 135** (6677 s of training = 1.85 h of the 2.1 h budget),
  `last.pt` from the same write (02:36 UTC), loads fine (optimizer state present). `train.log` stops inside epoch 136
  (batch 219/247); the tail of `results.csv` and `train.log` were NUL bytes (removed in the copies here). Best epoch so
  far: 113 (val mAP50 0.952, mAP50-95 0.875).
* `YOLO(last.pt).train(resume=True)`: Ultralytics printed "Resuming training ... from epoch 137 to 153 total epochs",
  then **stopped after one epoch (137, 86 s)**. With `time=2.1` set, Ultralytics re-derives the epoch count after every
  epoch from the mean epoch time *of the current session* (2.1 h / 86 s ≈ 88 epochs ≤ 137 → stop), so a resumed
  time-budgeted run cannot continue. Not retried: validation had not improved for 24 epochs (best 113, patience 40 would
  stop at 153 anyway), LR was already at the end of its cosine (4e-4), and epoch 137 scored 0.951 / 0.873 (≤ best).
  The epoch Ultralytics labelled 136 was the one cut off; it was not re-run.
* Exported `models/detector/det_v31_s.onnx` from best.pt (epoch 113), then the unchanged `p9b_v31_night.sh eval` and
  `report` stages (tables above). **det_v31_s passes the same acceptance targets.** Compared with det_v3_s: about the
  same on the v3 sets (within 0.003-0.012 mAP50; small_box body AP50 0.933 vs 0.971), better on the v3.1 body set (0.904
  vs 0.832) and at range: body-camera recall on all small objects at 2-3 m 65.6 % vs 45.6 % and beyond 3 m 25.0 % vs
  2.7 % (synth_v3_body test). **Runtime detector NOT switched** (`detector_node.py` DEFAULT_MODEL `det_v3_s.onnx`).

## Fetch failures by root cause (hand-written from the per-attempt logs; 13 failed of 15 + 4 of 6 A/B)

| rank | cause | tries | share of a) failures |
|---|---|---|---|
| 1 | target never found in the start room (search ended "I did not find it after 300 seconds") | small_box_1, small_box_3, mini_toolbox_1, mini_toolbox_3, mini_cone_1-3 | 7 / 13 |
| 2 | grasp closed but the object stayed on the floor | small_box_2, mini_extinguisher_1, mini_extinguisher_3, can_2 (+ B try 1) | 4 / 13 |
| 3 | wrist alignment never saw the object ("residual 99 cm") | mini_toolbox_2, mini_extinguisher_2 (+ B try 3) | 2 / 13 |
| - | test harness: 2nd / 3rd try on a stack after a pick + "put it down" | A try 2 (internal error), A try 3 (map lost the can), B try 3 | not robot results |

**1. Not found (7).** The robot starts in room A at (0, 0) and every planned target was in room A, 1.0-2.3 m away; the
sweep scans room A first (Nav2 goal (0, 0) then a turn of 54-108 s), then rooms B, D, C, so the 300 s limit is *not* why
these were missed (it only stopped the later rooms).
* *mini_cone 3/3 = colour naming (confirmed).* The robot did map the cones but named every one **"white mini cone"**
  (all 3 tries, `fetch/mini_cone_*/agent.log`; target orange, the second cone green). The colour is the median of the
  middle 50 % of the box (`detector_node.py:198`, `colors.color_name`); for the cone that crop is the narrow cone, its
  white reflective band (`vision/small_objects.py:65`) and background. Offline on 135 mini-cone boxes of the
  synth_v3_body test split the same rule gives white 85 %, gray 12 %, colour 3 % (`color_name` on the labelled boxes).
  **Fix:** name the colour from the saturated pixels only (median hue of pixels with s ≥ 0.18 when they are ≥ ~15 %
  of the crop, else white / gray), and give "orange cone" a colour-tolerant match when exactly one object of the class is
  known; regression test on the synthetic test splits per class (target: colour right on ≥ 90 % of cones / cans /
  cubes).
* *small_box 2/3, mini_toolbox 2/3 = cause NOT identified.* The target was in view during the room-A turn (objects in the
  same direction, e.g. the blue can at (-1.3, 0.3) next to the cube at (-0.8, 0.7), were announced) but no small box /
  mini toolbox was ever announced. The red cube at its default spot *was* mapped as "red small box" in 5 other tries,
  and the toolbox found in mini_toolbox_2 — so it works sometimes. The saved logs cannot tell a missed detection from a
  failed track confirmation (min_hits 8, mean score 0.6) or a wrong colour: the detector's sighting log (`log_path`) was
  off and the detector ran at ~1-2 frames/s wall (`frames` counter, CPUExecutionProvider). **Fix:** first make it
  measurable — turn on the sighting log in the fetch runs and record which of detect / confirm / colour drops the
  target; candidate fixes then are det_v31_s (recall at 1-2 m 91.5 vs 89.9 %, only a small gain) and a slower
  room-scan turn so a small object gets the 8 hits.

**2. Grasp slipped (4).** In small_box_2, mini_extinguisher_1 and _3 the first attempt closed **centred at the object's
width** (fingers 24/26, 23/28, 25/25 mm = 50 mm), and during the 5 cm lift the reading moved to **40 / 10 mm** (one finger
at its 40 mm open limit); all 6 later attempts in those tries read 40 / 10 at once. Ground truth: never lifted (z 0,
moved ≤ 2.2 cm). Cans (66 mm) stayed centred (33/33 → 36/30) and lifted in 2 of 3. The two fingers are independent
prismatic joints with the same effort and no mimic / centring (`gurdy_arm.xacro:137`), so nothing stops the pair from
sliding sideways as a unit under load; a finger at its limit is then no longer squeezing. can_2 is a plain drop (closed
32/23, lifted 0/0, pushed 11 cm). **Fix:** centre the fingers (mimic joint, or a centring term on the finger difference
in the gripper command), and re-run `scripts/grasp_test.sh` on the cube + extinguisher with finger states logged during
the lift before any full fetch run. Note: the per-attempt `est_err_m` (est vs GT) is not usable here — successful
grasps show 5-18 cm too, because `gt_base` uses the GT robot pose and the estimate the robot's own localisation.

**3. Alignment failed, "residual 99 cm" (2).** 99 cm was never measured: `fine_align` returns the placeholder 0.99 when
the servo got no observation at all (`brain/ros/fetch_robot.py:337`), and the reply prints it — a reply that states an
unmeasured number. At the end the object was **0.22-0.24 m from the base at ~45° behind-right** (base frame (-0.17, -0.17)
and (-0.14, -0.16)), while every successful approach left it 0.33-0.37 m straight behind (|y| ≤ 2.6 cm): after the Nav2
stand-off, the turn and the 0.25 m back-up the object was too close and off-axis for the wrist view pose. The servo's own
code (TARGET_LOST / OCCLUDED / TIMEOUT) was only kept in memory, not saved. **Fix:** (a) say "I couldn't see it with the
wrist camera" instead of a number and save the servo code in `attempts.jsonl`; (b) check the bearing / distance with
the body detector before backing up and back up only to the 0.35 m grasp distance; (c) when the wrist camera loses an
object whose prior is closer than the target distance, step away from it instead of toward it.

**Harness (A/B).** A try 1 and B try 2 succeeded; the tries after a pick + "put it down" on the same stack failed for
reasons that are not about grasping (A try 2 "Something went wrong" — no traceback saved, agent.log only has the start;
A try 3 "I don't know any blue can" — the teleport approach does not search and the map no longer had the can). Next
time: a fresh stack per A/B try, as in block a.

**Also:** every not-found try ended "I did not find it after 300 seconds" — the fixed limit that `phase9k-demo`
replaces with a map-sized one (not merged yet). Logs to add before the next run: detector sighting log, servo code per
attempt, agent stderr.
