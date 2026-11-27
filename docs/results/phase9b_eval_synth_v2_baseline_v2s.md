# Phase 4 detector evaluation — det_v2_s.onnx on datasets/synth_v2/test (342 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 31 | 0.544 | 0.465 | 0.645 |
| barrel | 63 | 0.933 | 0.935 | 0.921 |
| fire_extinguisher | 48 | 0.999 | 0.960 | 1.000 |
| traffic_cone | 31 | 0.935 | 0.935 | 0.935 |
| toolbox | 37 | 0.919 | 1.000 | 0.919 |
| mannequin | 217 | 0.937 | 0.923 | 0.935 |
| chair | 52 | 0.832 | 0.911 | 0.788 |
| **mAP50** | | **0.871** | | |

3D centre error (camera frame, 207 true positives): median 0.082 m, mean 0.281 m, p90 0.458 m; median range 2.71 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 147.9 |
| cpu | CPUExecutionProvider | 31.9 |

