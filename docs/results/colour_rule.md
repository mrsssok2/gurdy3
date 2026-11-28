# Colour rule: saturated pixels only (2026-11-28)

Trigger: overnight_1127 named every mini cone "white mini cone" (`docs/results/overnight_1127.md`, cause 1).
Change: `gurdy_core/vision/colors.py` `color_name` = majority hue name of the saturated pixels (s >= 0.30, v >= 0.20) when
they are >= 15 % of the detector's crop; otherwise the old median rule (`color_name_v1`) names genuinely white / gray /
black objects. The crop itself (middle 50 % of the box, `detector_node.py`) is unchanged.

## Finding: the mini cones are rendered WHITE (model bug, not the colour rule)

Crops of labelled mini cones from the test images show a white cone with a grey band on a black base plate, whatever the
variant (orange / lime / pink / blue). The SDF gives the cone's mesh visual (`meshes/*_p1.obj`, generated lathe OBJ
without .mtl / normals) the variant colour, but Gazebo renders it white; the primitive parts (band cylinder, base box)
take their colours. The large traffic cones use the same lathe mesh and were also reported as "white traffic cone" in
the overnight logs. So no colour rule can name these cones orange — Gazebo really shows them white — and **both
det_v3_s and det_v31_s were trained on white cones**. Fixing the mesh colour (an .mtl with the colour, or primitives)
changes the cone's appearance for the detector: it needs a re-render of the cone images + retrain, or at least a
detection check in Gazebo, before it is switched on. NOT changed here.

## Per-class accuracy, before (old rule) / after (new rule)

Measured with `python3 vision/color_eval.py datasets/synth_v3_body datasets/synth_v3_wrist datasets/synth_v31_body datasets/synth_v31_wrist`. Colour naming per class (vision/color_eval.py): detector crop (middle 50 % of the labelled box) vs the painted variant from the scene file, split `test`. old = median RGB rule, new = saturated-pixel majority.

**datasets/synth_v3_body (test)**

| class | instances | old rule | new rule | new rule: most common wrong names |
|---|---|---|---|---|
| small_box | 75 | 93.3 % | 93.3 % | black 4, gray 1 |
| mini_toolbox | 150 | 64.7 % | 70.7 % | brown 25, yellow 15, black 3 |
| can | 145 | 94.5 % | 95.2 % | black 7 |
| mini_cone | 134 | 1.5 % | 4.5 % | white 89, gray 12, blue 10 |
| mini_extinguisher | 103 | 83.5 % | 84.5 % | black 15, gray 1 |
| **all** | 607 | 64.6 % | 67.1 % | |

**datasets/synth_v3_wrist (test)**

| class | instances | old rule | new rule | new rule: most common wrong names |
|---|---|---|---|---|
| small_box | 92 | 94.6 % | 94.6 % | black 3, gray 1, brown 1 |
| mini_toolbox | 102 | 74.5 % | 85.3 % | brown 10, black 3, yellow 1 |
| can | 88 | 81.8 % | 95.5 % | black 4 |
| mini_cone | 73 | 0.0 % | 1.4 % | white 66, gray 5, pink 1 |
| mini_extinguisher | 103 | 93.2 % | 95.1 % | black 4, yellow 1 |
| **all** | 458 | 72.3 % | 77.9 % | |

**datasets/synth_v31_body (test)**

| class | instances | old rule | new rule | new rule: most common wrong names |
|---|---|---|---|---|
| small_box | 90 | 95.6 % | 95.6 % | black 3, white 1 |
| mini_toolbox | 160 | 65.0 % | 71.2 % | brown 27, yellow 14, black 3 |
| can | 152 | 94.7 % | 95.4 % | black 6, yellow 1 |
| mini_cone | 142 | 1.4 % | 4.2 % | white 92, gray 15, blue 11 |
| mini_extinguisher | 104 | 83.7 % | 84.6 % | black 15, gray 1 |
| **all** | 648 | 65.3 % | 67.7 % | |

**datasets/synth_v31_wrist (test)**

| class | instances | old rule | new rule | new rule: most common wrong names |
|---|---|---|---|---|
| small_box | 92 | 94.6 % | 94.6 % | black 3, gray 1, brown 1 |
| mini_toolbox | 102 | 74.5 % | 85.3 % | brown 10, black 3, yellow 1 |
| can | 88 | 81.8 % | 95.5 % | black 4 |
| mini_cone | 73 | 0.0 % | 1.4 % | white 66, gray 5, pink 1 |
| mini_extinguisher | 103 | 93.2 % | 95.1 % | black 4, yellow 1 |
| **all** | 458 | 72.3 % | 77.9 % | |


Summary: better or equal on every class and set; cans +13.7 points on the wrist camera (81.8 -> 95.5 %), mini toolbox
+6 to +11 points; the mini cone stays ~0-5 % because it IS white in the renders (see above). Remaining errors: "black"
(shadowed faces / the toolbox lid line and handles), "brown" (orange toolbox in shade: hue < 40° and v < 0.6).
Tests: `gurdy_core/tests/test_vision.py` (rule cases + "not worse per class" on synth_v3_wrist test, skipped without
the dataset).
