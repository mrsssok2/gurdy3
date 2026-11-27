# Phase 4 detector evaluation — det_v3_s.onnx on datasets/synth_v1/test (314 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 55 | 0.807 | 0.730 | 0.836 |
| barrel | 94 | 0.979 | 1.000 | 0.979 |
| fire_extinguisher | 61 | 0.989 | 0.984 | 1.000 |
| traffic_cone | 90 | 0.944 | 0.977 | 0.933 |
| toolbox | 46 | 0.976 | 0.957 | 0.957 |
| mannequin | 74 | 0.941 | 0.919 | 0.919 |
| chair | 67 | 0.894 | 0.982 | 0.836 |
| small_box | 0 | nan | nan | nan |
| mini_toolbox | 0 | nan | nan | nan |
| can | 0 | nan | nan | nan |
| mini_cone | 0 | nan | nan | nan |
| mini_extinguisher | 0 | nan | nan | nan |
| **mAP50** | | **0.933** | | |

mAP50 over the 7 old classes with GT: 0.933
small-class detections (score >= 0.35) in images without that class: 0 in 314 images

3D centre error (camera frame, 323 true positives): median 0.067 m, mean 0.143 m, p90 0.237 m; median range 2.32 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 94.3 |
| cpu | CPUExecutionProvider | 28.9 |

