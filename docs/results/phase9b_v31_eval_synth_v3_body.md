# Phase 4 detector evaluation — det_v31_s.onnx on datasets/synth_v3_body/test (565 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 61 | 0.626 | 0.814 | 0.574 |
| barrel | 36 | 0.972 | 1.000 | 0.917 |
| fire_extinguisher | 53 | 0.978 | 0.944 | 0.962 |
| traffic_cone | 66 | 0.930 | 0.968 | 0.924 |
| toolbox | 63 | 0.961 | 0.935 | 0.921 |
| mannequin | 194 | 0.900 | 0.903 | 0.866 |
| chair | 72 | 0.868 | 0.886 | 0.861 |
| small_box | 78 | 0.933 | 0.860 | 0.949 |
| mini_toolbox | 153 | 0.991 | 0.921 | 0.993 |
| can | 149 | 0.964 | 0.911 | 0.960 |
| mini_cone | 135 | 0.982 | 0.930 | 0.978 |
| mini_extinguisher | 104 | 0.979 | 0.886 | 0.971 |
| **mAP50** | | **0.924** | | |

mAP50 over the 7 old classes with GT: 0.891
small-class detections (score >= 0.35) in images without that class: 48 in 565 images

3D centre error (camera frame, 942 true positives): median 0.018 m, mean 0.089 m, p90 0.136 m; median range 1.54 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 154.8 |
| cpu | CPUExecutionProvider | 31.9 |

