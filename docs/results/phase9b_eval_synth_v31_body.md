# Phase 4 detector evaluation — det_v3_s.onnx on datasets/synth_v31_body/test (576 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 61 | 0.638 | 0.750 | 0.590 |
| barrel | 36 | 0.962 | 1.000 | 0.889 |
| fire_extinguisher | 53 | 0.980 | 0.963 | 0.981 |
| traffic_cone | 66 | 0.948 | 0.953 | 0.924 |
| toolbox | 63 | 0.947 | 0.965 | 0.873 |
| mannequin | 194 | 0.913 | 0.924 | 0.881 |
| chair | 72 | 0.871 | 0.861 | 0.861 |
| small_box | 117 | 0.743 | 1.000 | 0.675 |
| mini_toolbox | 180 | 0.838 | 0.986 | 0.811 |
| can | 183 | 0.813 | 1.000 | 0.781 |
| mini_cone | 152 | 0.907 | 1.000 | 0.868 |
| mini_extinguisher | 123 | 0.861 | 1.000 | 0.813 |
| **mAP50** | | **0.869** | | |

mAP50 over the 7 old classes with GT: 0.894
small-class detections (score >= 0.35) in images without that class: 1 in 576 images

3D centre error (camera frame, 879 true positives): median 0.018 m, mean 0.090 m, p90 0.149 m; median range 1.47 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 152.3 |
| cpu | CPUExecutionProvider | 32.4 |

