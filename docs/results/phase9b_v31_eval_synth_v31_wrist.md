# Phase 4 detector evaluation — det_v31_s.onnx on datasets/synth_v31_wrist/test (432 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 5 | 0.200 | 0.500 | 0.200 |
| barrel | 7 | 0.714 | 1.000 | 0.571 |
| fire_extinguisher | 3 | 1.000 | 1.000 | 1.000 |
| traffic_cone | 5 | 0.600 | 1.000 | 0.600 |
| toolbox | 7 | 0.429 | 0.500 | 0.429 |
| mannequin | 3 | 0.333 | 1.000 | 0.333 |
| chair | 7 | 0.176 | 0.143 | 0.143 |
| small_box | 93 | 0.978 | 1.000 | 0.978 |
| mini_toolbox | 104 | 0.990 | 1.000 | 0.990 |
| can | 90 | 0.989 | 1.000 | 0.989 |
| mini_cone | 73 | 0.972 | 0.986 | 0.973 |
| mini_extinguisher | 104 | 0.990 | 1.000 | 0.981 |
| **mAP50** | | **0.698** | | |

mAP50 over the 7 old classes with GT: 0.493
small-class detections (score >= 0.35) in images without that class: 0 in 432 images

3D centre error (camera frame, 454 true positives): median 0.017 m, mean 0.032 m, p90 0.031 m; median range 0.34 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 150.1 |
| cpu | CPUExecutionProvider | 29.9 |

