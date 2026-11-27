# Phase 4 detector evaluation — det_v3_s.onnx on datasets/synth_v3_wrist/test (432 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 5 | 0.100 | 0.000 | 0.000 |
| barrel | 7 | 0.714 | 1.000 | 0.429 |
| fire_extinguisher | 3 | 1.000 | 1.000 | 1.000 |
| traffic_cone | 5 | 0.500 | 0.750 | 0.600 |
| toolbox | 7 | 0.443 | 0.500 | 0.286 |
| mannequin | 3 | 0.333 | nan | 0.000 |
| chair | 7 | 0.143 | 0.333 | 0.143 |
| small_box | 93 | 0.978 | 1.000 | 0.978 |
| mini_toolbox | 104 | 0.990 | 1.000 | 0.990 |
| can | 89 | 1.000 | 1.000 | 1.000 |
| mini_cone | 73 | 0.985 | 0.986 | 0.973 |
| mini_extinguisher | 103 | 1.000 | 1.000 | 0.990 |
| **mAP50** | | **0.682** | | |

mAP50 over the 7 old classes with GT: 0.462
small-class detections (score >= 0.35) in images without that class: 0 in 432 images

3D centre error (camera frame, 453 true positives): median 0.018 m, mean 0.033 m, p90 0.031 m; median range 0.34 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 116.0 |
| cpu | CPUExecutionProvider | 23.2 |

