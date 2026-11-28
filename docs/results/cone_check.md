# Mini cone: white mesh vs coloured mesh, det_v31_s (2026-11-28)

**Cause of the white cones:** the lathe-cone OBJ had no vertex normals; Gazebo drew it white whatever its SDF material.
With per-vertex normals it renders its colour (2-scene trial of 4 variants: normals fix it, an .mtl alone does not:
`data/cone_check/variants.png`). Fixed for the small objects (b511e7d): Gazebo models `small_mini_cone_*` and the lab4_fetch
orange + lime cones. The big traffic cones and all existing dataset worlds still use the old (white) mesh.

**Check set:** the first 30 synth_v31_body TEST scenes that contain a mini cone, rendered twice, identical except for the
mini-cone mesh (`vision/cone_check.py make --variant white|normals`): 324 / 336 frames, 145 / 152 labelled mini cones
(cone variants in these scenes: orange, lime, pink, blue). Commands: `vision/recall_by_distance.py --only mini_cone`,
`vision/eval_detector.py --classes v3`, `vision/color_eval.py`, `data/cone_check/cone_confusion.py`.

| det_v31_s on mini cones | white mesh (as trained) | coloured mesh (fixed) |
|---|---|---|
| recall, all instances (conf 0.35) | 85.4 % | **10.4 %** |
| recall < 1 m / 1-2 m / 2-3 m / > 3 m | 100 / 98.2 / 78.3 / 39.3 % | 20.0 / 8.6 / 4.3 / 0.0 % |
| AP50 / recall@0.35 (eval_detector) | 0.937 / 0.917 | 0.193 / 0.112 |
| what the cones < 2 m are detected as (top score, conf 0.25) | mini_cone 107 of 111 | nothing 48, mini_extinguisher 26, mini_toolbox 25, **mini_cone 15** of 118 |
| colour named right, new rule (old rule) | 4.4 % (1.5 %) | **83.8 %** (74.6 %) |
| mAP50, 7 old classes | 0.920 | 0.921 (unchanged) |

**Conclusion:** the colour is fixed, but det_v31_s (like det_v3_s) learnt "mini cone = white cone": coloured mini cones are
mostly missed or taken for a mini extinguisher / mini toolbox. Recall drops noticeably (85 -> 10 %), so the detector needs
a re-render of the scenes with mini cones (now coloured) + fine-tune — NOT started (asked first). Until then a coloured
mini cone in lab4_fetch will normally not be found, and an orange / red cone can show up as a mini extinguisher.
