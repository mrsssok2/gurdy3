# Phase 4 detector evaluation — det_v31_s.onnx on datasets/synth_v31_body/test (576 images)

| class | GT | AP50 | precision@0.35 | recall@0.35 |
|---|---|---|---|---|
| box | 61 | 0.623 | 0.778 | 0.574 |
| barrel | 36 | 0.972 | 1.000 | 0.917 |
| fire_extinguisher | 53 | 0.978 | 0.944 | 0.962 |
| traffic_cone | 66 | 0.930 | 0.968 | 0.924 |
| toolbox | 63 | 0.961 | 0.935 | 0.921 |
| mannequin | 194 | 0.900 | 0.898 | 0.866 |
| chair | 72 | 0.869 | 0.899 | 0.861 |
| small_box | 117 | 0.835 | 0.989 | 0.786 |
| mini_toolbox | 180 | 0.921 | 0.988 | 0.906 |
| can | 183 | 0.906 | 0.994 | 0.858 |
| mini_cone | 152 | 0.939 | 0.979 | 0.921 |
| mini_extinguisher | 123 | 0.917 | 0.965 | 0.902 |
| **mAP50** | | **0.896** | | |

mAP50 over the 7 old classes with GT: 0.890
small-class detections (score >= 0.35) in images without that class: 3 in 576 images

3D centre error (camera frame, 949 true positives): median 0.017 m, mean 0.088 m, p90 0.136 m; median range 1.55 m

| provider | active | FPS (inference only) |
|---|---|---|
| cuda | CUDAExecutionProvider | 144.1 |
| cpu | CPUExecutionProvider | 30.7 |

