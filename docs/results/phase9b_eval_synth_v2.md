# Phase 4 detector evaluation — det_v3_s.onnx on datasets/synth_v2/test (342 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 31 | 0.507 | 0.513 | 0.645 |
| barrel | 63 | 0.951 | 0.983 | 0.921 |
| fire_extinguisher | 48 | 0.977 | 0.958 | 0.958 |
| traffic_cone | 31 | 0.964 | 0.906 | 0.935 |
| toolbox | 37 | 0.919 | 0.971 | 0.919 |
| mannequin | 217 | 0.942 | 0.931 | 0.935 |
| chair | 52 | 0.868 | 0.898 | 0.846 |
| small_box | 0 | nan | nan | nan |
| mini_toolbox | 0 | nan | nan | nan |
| can | 0 | nan | nan | nan |
| mini_cone | 0 | nan | nan | nan |
| mini_extinguisher | 0 | nan | nan | nan |
| **mAP50** | | **0.875** | | |

mAP50 over the 7 old classes with GT: 0.875
small-class detections (score >= 0.35) in images without that class: 0 in 342 images

3D centre error (camera frame, 208 true positives): median 0.083 m, mean 0.286 m, p90 0.469 m; median range 2.71 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 136.4 |
| cpu | CPUExecutionProvider | 31.0 |

