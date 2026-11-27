# Phase 9b prep: small pickable objects, lab4_fetch world, YOLO v3 dataset plan

Status 2026-11-26: models, world, dataset sampler, configs and tests are written, on branch `phase9a-arm`. **Nothing has
been rendered, trained or run in Gazebo.** lab4 (`lab4.sdf`, `lab4.places.yaml`, `lab4_gt.json`) is unchanged:
`gen_world_lab4.py --check` passes, and a test enforces it. The deployed detector (`det_v2_s`, 7 classes) and
`CLASSES` are untouched.

## 1. The five objects (`vision/small_objects.py`)

| class | geometry | mass | grasp (Phase 9a) | colour variants |
|---|---|---|---|---|
| `small_box` | 5 cm cube | 50 g | top-down, 5 cm | red, blue, green, yellow, purple |
| `mini_toolbox` | 12 × 5 × 6 cm body + raised handle (bar 8 × 2 × 1 cm, top at 9.5 cm, 2.5 cm finger gap) | 80 g | top-down on the handle, 2 cm | red, yellow, blue, orange |
| `can` | cylinder r 3.3 cm, h 12 cm (silver rims) | 60 g | side, 6.6 cm | red, blue, green, silver, yellow |
| `mini_cone` | 8 × 8 × 0.8 cm base + cone r 3.5 → 0.6 cm, h 10 cm (white band) | 30 g | side at 5 cm (4.4 cm wide) | orange, lime, pink, blue |
| `mini_extinguisher` | body r 2.5 cm, h 13 cm + valve, handle, label band; h 15 cm | 80 g | side, 5 cm | red, dark green, blue |

* **Collision = boxes / cylinders only** (stable DART contacts). The cone is 3 stacked cylinders at the mean radius of
  each third. The visual cone is the existing lathe mesh.
* **Mass and inertia are computed** from the collision solids at uniform density, scaled to the class mass (parallel-axis
  sum, CoM offset included). The values are never typed.
* **Friction:** μ = μ2 = 1.0 (the gripper pads are 1.0 too), torsional 0.5 (twist while lifting), `min_depth` 0.5 mm,
  `max_vel` 0.1.
* **Models are dynamic** in `lab4_fetch` and static in the dataset renders.
* **Colours** are bright, and shared between classes, so the detector must learn the shape, not "orange = cone".
  `lime` / `dark_green` are reported as "green" in the GT, for "the green cone" missions.
* **Standalone models:** `ros2_ws/src/gurdy_gazebo/models/small_<class>_<colour>/` (21 dirs), spawnable with
  `ros_gz_sim create -file .../model.sdf`. Not installed by CMake yet (`install(DIRECTORY ... models)` to add). Use the
  source path or `GZ_SIM_RESOURCE_PATH` until then.
* **Arm grasp data:** sizes, masses and grip widths are tested against `gurdy_core.arm.PICKABLE`. Grip width is measured
  on the collision solids at the grasp height. The mini toolbox handle was raised so that the finger tips stay ≥ 1 cm
  above the body. That changed the Phase 9a reach numbers for this class (zone r 3.0 cm, 2 cm disc still 100 %); the 9a
  plan and `arm_reach.md` are updated.

## 2. World `lab4_fetch` (`scripts/gen_world_lab4_fetch.py`)

`lab4.build()` output (same walls, doors and 10 big objects), with the world renamed, plus 8 dynamic small objects:

| room | objects |
|---|---|
| A | red cube (1.2, −1.4), blue can (−1.3, 0.3) |
| B | orange cone (3.2, −1.4), red mini extinguisher (5.3, 0.0) |
| C | yellow mini toolbox (0.9, 5.3), red can (1.3, 2.6) |
| D | blue cube (4.2, 5.4), green cone (5.3, 4.0) |

* **Duplicates for reference resolution:** can (blue A / red C), cube (red A / blue D), cone (orange B / green D).
* **Placement rules** (`validate()`, tested):
  * ≥ 0.55 m from walls, ≥ 0.9 m from big objects, ≥ 0.6 m between small objects, ≥ 1.0 m from doorways and the spawn,
    ≥ 0.8 m from room centres (Nav2 room goals)
  * **≥ 0.45 m from the walking routes** (room centre ↔ door, door ↔ door)
  * The robot needs room to turn its back to the object and stand about 0.3 m from it.
* **Ground truth:** `docs/results/data/lab4_fetch/lab4_fetch_gt.json`. The lab4 objects (pickable false) plus the small
  objects with 3D centre, size, yaw, mass, colour, room and model name.
* **Places:** `maps/lab4_fetch.places.yaml` (lab4 places, `map_id: lab4_fetch`). The occupancy map `maps/lab4_map.yaml`
  can be reused: same walls.
* **Important: every small object is below the LiDAR plane** (5–15 cm vs ~0.16 m). Neither SLAM nor the Nav2 costmaps
  nor the safety shield see them, and the robot can kick them. The placement keeps them off the likely routes, but
  that is not a guarantee. Fix later (not done): a depth-camera obstacle layer in the local costmap, or the semantic map's
  small objects as costmap obstacles.

## 3. YOLO v3 dataset (prepared, not rendered)

How the existing data was made (Phase 4, `vision/generate_dataset.py`):
* random rooms, textures and lights, objects from SDF primitives
* one static camera model (`rgbd_camera` + `boundingbox_camera`, gz Label system = class id + 1), moved between 12 views
  per scene
* labels come automatically from the boundingbox camera; one fresh Gazebo server per scene; scene-disjoint
  train/val/test (14/3/3 of 20)
* v1: 180 scenes / 2136 images / 30.5 min; v2: 240 scenes / 2250 images / 39.4 min (measured)

**New profiles** (`vision/scene_small.py`, selected with `generate_dataset.py plan --profile v3-body|v3-wrist`):
* **Every v3 scene = a v2 scene** (7 old classes, distractor families, same randomisation), plus:
  * 2–6 small labelled objects (random class, colour, yaw)
  * 0–5 small **unlabelled** distractors of other shapes (balls, flat discs, thin rods, tall thin bottles, flat plates),
    so that "small and round" does not mean "can"
  * lighting: v2 plus 30 % dim scenes (ambient 0.05–0.15, weak sun) and 20 % with a strong coloured lamp
* **v3-body:** the robot.yaml body camera (320 × 240, 60°), 0.10–0.30 m high. 70 % of views aimed at a small object
  0.3–2.0 m away, 20 % at an old-class object (old classes stay in view), 10 % random.
* **v3-wrist:** the arm.yaml wrist camera (320 × 240, hfov 1.2, 0.07–1.0 m).
  * 70 % top-down / oblique views (camera 0.10–0.45 m above the floor, pitched 35–90° down)
  * 30 % low side views (0.05–0.15 m, the side-grasp approach)
  * Gripper fingers are part of the camera model at the arm's geometry. They sit inside the 0.07 m near clip, so
    Gazebo clips them, exactly as on the robot model.
* **Class ids:** `CLASSES_V3` = the 7 old classes (ids 0–6, unchanged) + `small_box`, `mini_toolbox`, `can`,
  `mini_cone`, `mini_extinguisher` (7–11). v1 / v2 label files are reused as they are.
  * Each v3 dataset writes its `classes.json`; `finalize`, `check_labels.py` and `dataset_report.py` read it.
  * The render step accepts 12 label ids for v3 scenes. v1 / v2 behaviour is unchanged: the random streams are tested
    identical.
* **Merge:** `datasets/synth_v123.yaml` = v1 + v2 + v3_body + v3_wrist train/val, 12 names. Test splits stay separate.
* **Evaluation:** `vision/eval_detector.py --classes v3` adds the mAP over the 7 old classes and the number of
  small-class false positives on images without them (the regression guard).
* **Pipeline:** `scripts/p9b_small_dataset.sh plan | trial | render | finalize | probe | train | eval`.

**Size estimate** (from `plan` on 20 + 20 scenes, measured offline: centre-in-view instances per scene: body 20, wrist
16; the share that survives the label filters, ≥ 6 px and ≥ 80 px², is assumed 70–80 %):

| | scenes | images (12 views) | small-object labels | per small class |
|---|---|---|---|---|
| v3-body | 350 | ~4 200 | ~4 900 | ~980 |
| v3-wrist | 250 | ~3 000 | ~3 200 | ~630 |
| **new total** | 600 | **~7 200** | ~8 100 | **~1 600** (v2 had 1 563 mannequins, ~300 per other class) |
| + v1 + v2 (reused) | 420 | 4 386 | old classes | |

* **Render: ~1.7–2 h** (600 × ~10 s measured per v1/v2 scene, +20 % for the extra models). One Gazebo at a time.
* **Disk:** ~1 GB (estimate).
* **Training (ESTIMATE; no epoch time was recorded for det_v1 / det_v2):**
  * yolo11s, 320 px, batch 32 on the RTX 4050 (6 GB): VRAM ~3 GB (estimate), ~7.9k train images
  * at an assumed 150–300 img/s that is ~30–55 s/epoch
  * fine-tune from det_v2_s, 150 epochs (patience 40): **~1.3–2.5 h**; from scratch, 300 epochs: ~2.6–4.8 h
  * `probe` (3 epochs) measures the real s/epoch and VRAM before the long run
* **Small objects are tiny for the body camera:** a 5 cm cube is ~9 px at 1.5 m and ~7 px at 2 m (letterbox 256 × 320).
  Expect body-camera recall to fall beyond ~1.5 m. That is why the wrist camera views exist and why picking uses the
  final approach. Search for small objects with the body camera at ≤ 1.5 m.

**Acceptance for det_v3_s** (to agree):
* old classes: v1 test mAP50 ≥ 0.912 and v2 test ≥ 0.861 (det_v2_s: 0.922 / 0.871, so −0.01 at most)
* semantic_lab live: 11/11 objects, ≤ 7 false tracks (as det_v2_s)
* new classes: mAP50 ≥ 0.80 on v3-wrist test, ≥ 0.70 on v3-body test
* < 1 small-class false positive per 100 old-test images

## 4. What needs the GPU / Gazebo (tomorrow, one heavy job at a time)

1. **`trial`:** render 5 scenes per profile and look at the images and labels. Things to check:
   * small objects get boxes; the Label plugin works on the multi-collision models
   * wrist views are sensible
   * dim scenes are not black
2. **`render`, `finalize`** (~2 h, Gazebo).
3. **`probe`**, then **`train`** (GPU, ~1.3–2.5 h estimated), then export `det_v3_s.onnx`.
4. **`eval`:** old test sets (regression), new test sets, then semantic_lab in Gazebo.
5. **Runtime switch** (code, after a passing model):
   * `detector_node` and `OnnxDetector` with `names=CLASSES_V3` (or the names from the ONNX json)
   * `SMALL_ALIASES` into `resolve_class`
   * agent vocabulary (planner prompt / data) for the new classes
6. **lab4_fetch in Gazebo:**
   * the objects rest stably (no jitter / sliding at RTF < 1)
   * the robot walks the routes without touching them
   * a grasp test with the Phase 9a arm (contact physics first, fallback labelled)

## 5. Files

| file | what |
|---|---|
| `vision/small_objects.py` | catalogue, mass properties, model SDF, model dirs |
| `ros2_ws/src/gurdy_gazebo/models/small_*` | 21 generated models |
| `scripts/gen_world_lab4_fetch.py` | world generator |
| `ros2_ws/src/gurdy_gazebo/worlds/lab4_fetch.sdf` (+ 2 cone meshes) | generated world |
| `maps/lab4_fetch.places.yaml`, `docs/results/data/lab4_fetch/lab4_fetch_gt.json` | places, ground truth |
| `vision/scene_small.py` | v3 profiles |
| `vision/generate_dataset.py` | v3 hook; v1 / v2 unchanged |
| `vision/eval_detector.py` | `--classes v3` + regression lines |
| `vision/check_labels.py`, `vision/dataset_report.py` | read `classes.json` |
| `gurdy_core/gurdy_core/vision/classes.py` | `SMALL_CLASSES`, `CLASSES_V3`, `SMALL_ALIASES`, `SIZE_PRIOR_SMALL`; `CLASSES` unchanged |
| `datasets/synth_v123.yaml` | merged dataset |
| `scripts/p9b_small_dataset.sh` | pipeline |
| `vision/tests/test_small_objects.py` | 20 tests |

Tests: `vision/tests` 46 pass, `gurdy_core` vision + arm 35 pass (host venv: numpy, pyyaml, pillow, xacro).

## 6. Honest limits

* **No render or training yet.** Image and instance counts are extrapolated from the offline `plan` of 40 scenes. The
  label-survival share and the training speed are **estimates**.
* **Contact physics for 30–80 g objects in DART is unverified.** It may need finer steps or larger pads (Phase 9a plan §5).
* **Below the LiDAR plane:** the robot cannot avoid these objects except by placement.
* **Synthetic only:** primitive shapes and flat colours. No real-photo validation exists for any class.

## Known limitation: small objects > 2 m under-labelled in v3 (28 % body / 14 % wrist images); fixed in v3.1
Found by a label audit on 2026-11-27, during the det_v3_s training (not stopped). `vision/scene.py:yolo_lines` drops every box under
6 px on a side or 80 px²; that rule came from the large-object datasets. A 5 cm cube 2 m from the body camera is about 7 × 7 px
(49 px²), so it is **visible but unlabelled** (example: `s0091_v03`, the yellow small_box at 2.08 m). Counted from the per-frame meta
ground truth (centre in view, expected size below the rule, no label):

| set | images | small objects labelled | unlabelled, expected tiny | images with ≥ 1 | unlabelled, other (occluded / cut) |
|---|---|---|---|---|---|
| v3 body train | 2711 | 2816 | 1125 | 757 (28 %) | 244 |
| v3 wrist train | 2134 | 2258 | 413 | 292 (14 %) | 197 |
| v3 body test | 565 | 605 | 203 | 150 | 43 |
| v3 wrist test | 432 | 462 | 90 | 56 | 36 |

Effects:
* The model is taught that tiny far targets are background.
* The test sets follow the same rule, so the agreed acceptance numbers stay comparable as defined.
* Body-camera recall beyond ~2 m is lower than the mAP suggests. `vision/recall_by_distance.py` measures it per bin
  (< 1, 1-2, 2-3, > 3 m) over ALL objects in view, labelled or not.

The chair-labelled "T shape" in `wrist_s0011_v03` was checked too: it is the real chair model seen from low, correctly labelled.

**v3.1 (prepared, not rendered).**
* `generate_dataset.py render --labels v3.1` (default `v3`): small classes 3 px / 12 px², large classes keep 6 / 80.
* Every frame's meta now ALWAYS stores the unfiltered boxes (`boxes_raw`, `image_wh`, `label_rule`) and the objects' 3D ground truth
  (yaw, size). `generate_dataset.py relabel --labels ...` can change the rule later without Gazebo. Tests: `vision/tests/test_label_rules.py`.
* One command for the next night: `scripts/p9b_v31_night.sh`. It uses the same scene seeds as v3, then render, train det_v31_s, eval with
  the same targets plus recall by distance, and `docs/results/phase9b_v3_vs_v31.md` side by side.

**Fetch search (same day).** When the target is not in the map, the search visits ONE viewpoint per room and turns a full circle there,
then explores, then reports NOT_FOUND. It never walks closer inside a room, and lab4 room corners are ~2.8 m from the room point.
* Added: a multi-viewpoint room sweep (`RosBackend.search_view_radius_m`, default None = unchanged). Every point of the room lies within
  the radius of a viewpoint; the viewpoints are free-checked in the map, optional and nearest-neighbour ordered, with a full turn at each.
* `FetchConfig.search_view_radius_m` (default None) switches it on for a fetch search only. Tests in `brain/tests/test_search_sweep.py`.
* Not run in Gazebo.
