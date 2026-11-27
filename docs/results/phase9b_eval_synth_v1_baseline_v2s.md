# Phase 4 detector evaluation — det_v2_s.onnx on datasets/synth_v1/test (314 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 55 | 0.725 | 0.745 | 0.745 |
| barrel | 94 | 0.979 | 1.000 | 0.957 |
| fire_extinguisher | 61 | 1.000 | 0.984 | 1.000 |
| traffic_cone | 90 | 0.954 | 0.966 | 0.933 |
| toolbox | 46 | 0.975 | 0.915 | 0.935 |
| mannequin | 74 | 0.928 | 0.908 | 0.932 |
| chair | 67 | 0.891 | 0.967 | 0.866 |
| **mAP50** | | **0.922** | | |

3D centre error (camera frame, 314 true positives): median 0.065 m, mean 0.138 m, p90 0.232 m; median range 2.32 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 115.6 |
| cpu | CPUExecutionProvider | 30.8 |

