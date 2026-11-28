# Gazebo fetch smoke 2026-11-28 (b): can x3, cube, cone skipped

Fresh stack per try, `scripts/overnight_1127.sh --smoke` (same placements as overnight a), TRY_TIMEOUT_S 1800, graded from
Gazebo ground truth (object z right after the reply and 5 s later; a carried can sits at z ~0.23). Code: missing-data rule
(36e5153), coupled fingers (75bd1ea), det_v31_s, coloured mini cones (b511e7d), sighting log on, map-sized search limit.
Data: `docs/results/data/smoke_1128b/`.

| try | object (placement) | result by ground truth | robot said | time |
|---|---|---|---|---|
| can 1 | blue can (1.2, -0.1), 1.2 m | **held** (z 0.230 / 0.233 after 5 s) | "I closed my gripper on the blue can and lifted it, but I can't confirm I'm holding it (my wrist camera gave me no picture to check). ..." | 130 s |
| can 2 | same | **held** (z 0.236 / 0.240) | same "can't confirm" reply | 91 s |
| can 3 | same | **held** (z 0.237 / 0.240) | "I picked up the blue can in room A and I'm holding it with the arm folded." | 231 s |
| cube | red cube (-0.8, 0.7), 1.06 m | **not picked** (never touched) | "I don't know any red cube, and I won't guess another object." (search ended after 468 s, map-sized limit) | 1309 s |
| cone | orange mini cone | **skipped** | - | - |

* Can: 3/3 held by ground truth, 0 releases (before the missing-data fix the same situation released the can). The robot
  confirmed only 1/3: in tries 1-2 the verify view got NO wrist frame although it asked again for >= 8 s of sim time (the
  servo's views before the grasp worked). Cause not found yet; `RclpyArmPort.wrist_view` now records why it came back
  empty (triggers sent, age of the last depth frame, camera_info / joint states) and the attempt record saves it.
* Cone skipped: det_v31_s finds 10 % of the coloured mini cones (`docs/results/cone_check.md`); it cannot pass before a
  retrain. In this run the orange cone at (3.2, -1.4) was mapped as a "brown mini toolbox" (46 sightings).

## Cube: detected, right class + colour, but never CONFIRMED as a track

From `cube/fetch/small_box_1/chat/sightings.jsonl` (855 sightings) and `cube_sightings.txt`:

* The cube was in the body camera's view in two passes of the room-A scan, both from ~1.0 m: t 20.4-23.3 s and
  t 52.5-54.1 s (sim). Every frame with a detection in those passes detected it: **11 sightings, all `small_box`, all
  colour `red`, score 0.84-0.89, position within 2-5 cm of the truth** (-0.76..-0.82, 0.73..0.77). Later frames with the
  cube in the image were 4.3-5.7 m away (rooms B / D): no detection there (beyond the small-object range).
* The detector node confirms a track only after `min_hits` 8 sightings spanning >= 2 s; an UNCONFIRMED track is dropped
  `ttl` 10 s after its last sighting. Pass 1 gave 6 hits (2.9 s), expired at ~33 s; pass 2 (29 s later) started a new
  track with 5 hits. 11 good hits in total, never 8 in one track -> never confirmed, never announced, so "red cube" had
  nothing to match ("I don't know any red cube").
* Why so few hits: the room-A scan turns the robot, and the detector runs at ~1-2 frames/s wall (CPU), so a 5 cm object
  at 1 m stays in the image for only ~3 s (pixel column 16 -> 266 within 2.9 s).

**Replay of the same sightings** through `SemanticMap` with other settings (`vision/replay_fetch_sightings.py`, all 5 fetch
logs of the day: 4 cans + the cube; `data/smoke_1128b/replay_confirmation.md`):

| setting | target confirmed | false tracks (sum, 5 tries) |
|---|---|---|
| node default (8 hits, span 2 s, ttl 10 s) | 4/5 (cube not) | 3 |
| **unconfirmed ttl 30 s** (60 / 120 s: same) | **5/5** | **3** (same) |
| 5 hits, ttl 10 s | 5/5 | 5 (+2) |

The 3 false tracks are the same in both: the orange cone as "brown mini toolbox" (cone retrain) and the blue barrel
estimated 0.73 m from its true spot. **Proposed fix:** detector_node `ttl` 10 -> 30 s for unconfirmed tracks (hits from
separate passes add up; the hit / span / score / scatter rules stay as strict). Evidence: 5 logs only, so check it in the
next run; optional second lever: a slower room scan turn (more frames per object). NOT changed yet.

## Cone retrain plan (for tonight's overnight run)

What changes: only the mini-cone mesh (now with normals = rendered in colour). Re-render ONLY the scenes that contain a
mini cone, with the same seeds, into new dataset dirs (synth_v32_body / _wrist = copies of v3.1 with those scenes
re-rendered; `vision/cone_check.py make` already rewrites a scene's cone meshes), then fine-tune det_v31_s.

| stage | size | cost (measured rate) |
|---|---|---|
| re-render | 350 scenes (body 203: train 146 / val 27 / test 30; wrist 147: 104 / 25 / 18) x 12 views | ~68 min (11.7 s per scene: overnight v3.1 render 600 scenes in 117 min; cone check 30 in 5.6-6.6 min) |
| fine-tune det_v31_s (v1 + v2 + v3.2) | same image count as v3.1 | 49.5 s / epoch (det_v31_s, 135 epochs in 1.85 h): 40 epochs ~33 min; or the full 2.1 h budget |
| eval: acceptance + recall by distance + cone check | | ~5 min (v3.1 eval 02:47-02:51) + ~5 min |
| **total** | | **~1 h 50 min** with a 40-epoch fine-tune, ~3 h 25 min with the 2.1 h budget |

Notes: train with `--epochs` (not `--time-h`): a time-budgeted Ultralytics run cannot be resumed after a power cut (it
stopped after 1 epoch this morning). Accept if the old-class / small-class targets still pass AND coloured mini-cone
recall on the cone check set returns to the white-cone level (85 %). Big traffic cones keep the old white mesh everywhere
(datasets and lab4), so they are consistent and not part of this.
