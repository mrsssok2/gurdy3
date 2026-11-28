# Phase 4 detector evaluation — det_v31_s.onnx on datasets/synth_v2/test (342 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 31 | 0.434 | 0.471 | 0.516 |
| barrel | 63 | 0.920 | 0.983 | 0.921 |
| fire_extinguisher | 48 | 0.999 | 0.941 | 1.000 |
| traffic_cone | 31 | 0.935 | 0.935 | 0.935 |
| toolbox | 37 | 0.946 | 1.000 | 0.919 |
| mannequin | 217 | 0.935 | 0.923 | 0.945 |
| chair | 52 | 0.875 | 0.957 | 0.846 |
| small_box | 0 | nan | nan | nan |
| mini_toolbox | 0 | nan | nan | nan |
| can | 0 | nan | nan | nan |
| mini_cone | 0 | nan | nan | nan |
| mini_extinguisher | 0 | nan | nan | nan |
| **mAP50** | | **0.863** | | |

mAP50 over the 7 old classes with GT: 0.863
small-class detections (score >= 0.35) in images without that class: 0 in 342 images

3D centre error (camera frame, 202 true positives): median 0.082 m, mean 0.276 m, p90 0.399 m; median range 2.65 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 149.3 |
| cpu | CPUExecutionProvider | 33.0 |

