Colour naming per class (vision/color_eval.py): detector crop (middle 50 % of the labelled box) vs the painted variant from the scene file, split `test`. old = median RGB rule, new = saturated-pixel majority.

**datasets/cone_check_white (test)**

| class | instances | old rule | new rule | new rule: most common wrong names |
|---|---|---|---|---|
| small_box | 43 | 100.0 % | 100.0 % | - |
| mini_toolbox | 94 | 68.1 % | 77.7 % | brown 15, black 3, yellow 2 |
| can | 67 | 98.5 % | 98.5 % | black 1 |
| mini_cone | 135 | 1.5 % | 4.4 % | white 87, gray 13, blue 11 |
| mini_extinguisher | 48 | 89.6 % | 89.6 % | black 5 |
| **all** | 387 | 56.3 % | 59.7 % | |

**datasets/cone_check_normals (test)**

| class | instances | old rule | new rule | new rule: most common wrong names |
|---|---|---|---|---|
| small_box | 43 | 100.0 % | 100.0 % | - |
| mini_toolbox | 98 | 65.3 % | 74.5 % | brown 19, black 3, yellow 2 |
| can | 70 | 98.6 % | 98.6 % | black 1 |
| mini_cone | 142 | 74.6 % | 83.8 % | brown 9, yellow 8, blue 4 |
| mini_extinguisher | 51 | 90.2 % | 90.2 % | black 5 |
| **all** | 404 | 81.2 % | 86.6 % | |


