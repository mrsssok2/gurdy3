# Phase 4 detector evaluation — det_v3_s.onnx on datasets/synth_v3_body/test (565 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 61 | 0.640 | 0.766 | 0.590 |
| barrel | 36 | 0.962 | 1.000 | 0.889 |
| fire_extinguisher | 53 | 0.980 | 0.963 | 0.981 |
| traffic_cone | 66 | 0.948 | 0.953 | 0.924 |
| toolbox | 63 | 0.947 | 0.965 | 0.873 |
| mannequin | 194 | 0.913 | 0.929 | 0.881 |
| chair | 72 | 0.871 | 0.849 | 0.861 |
| small_box | 78 | 0.971 | 0.947 | 0.923 |
| mini_toolbox | 153 | 0.960 | 0.973 | 0.935 |
| can | 149 | 0.972 | 1.000 | 0.953 |
| mini_cone | 135 | 0.985 | 0.992 | 0.963 |
| mini_extinguisher | 104 | 0.980 | 1.000 | 0.962 |
| **mAP50** | | **0.927** | | |

mAP50 over the 7 old classes with GT: 0.894
small-class detections (score >= 0.35) in images without that class: 8 in 565 images

3D centre error (camera frame, 875 true positives): median 0.018 m, mean 0.090 m, p90 0.149 m; median range 1.46 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 148.3 |
| cpu | CPUExecutionProvider | 31.2 |

