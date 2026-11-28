# Phase 4 detector evaluation — det_v31_s.onnx on datasets/synth_v1/test (314 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 55 | 0.775 | 0.759 | 0.800 |
| barrel | 94 | 0.979 | 1.000 | 0.968 |
| fire_extinguisher | 61 | 1.000 | 0.984 | 1.000 |
| traffic_cone | 90 | 0.954 | 0.977 | 0.933 |
| toolbox | 46 | 0.952 | 0.933 | 0.913 |
| mannequin | 74 | 0.957 | 0.945 | 0.932 |
| chair | 67 | 0.879 | 0.966 | 0.851 |
| small_box | 0 | nan | nan | nan |
| mini_toolbox | 0 | nan | 0.000 | nan |
| can | 0 | nan | 0.000 | nan |
| mini_cone | 0 | nan | 0.000 | nan |
| mini_extinguisher | 0 | nan | nan | nan |
| **mAP50** | | **0.928** | | |

mAP50 over the 7 old classes with GT: 0.928
small-class detections (score >= 0.35) in images without that class: 3 in 314 images

3D centre error (camera frame, 321 true positives): median 0.066 m, mean 0.147 m, p90 0.238 m; median range 2.33 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 153.2 |
| cpu | CPUExecutionProvider | 32.5 |

