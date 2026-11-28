Recall by distance, mini_cone, model `models/detector/det_v31_s.onnx`, split test, conf 0.35 (vision/recall_by_distance.py)

**datasets/cone_check_white (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 51 | 51 | 0 | 0 | 100.0 % | 100.0 % | - |
| 1-2 m | 55 | 54 | 0 | 1 | 98.2 % | 100.0 % | - |
| 2-3 m | 23 | 20 | 0 | 3 | 78.3 % | 90.0 % | - |
| > 3 m | 28 | 17 | 11 | 0 | 39.3 % | 64.7 % | 0.0 % |
| all | 157 | 142 | 11 | 4 | 85.4 % | 94.4 % | 0.0 % |

| mini_cone | 145 | 0.937 | 0.978 | 0.917 |
| **mAP50** | | **0.918** | | |
mAP50 over the 7 old classes with GT: 0.920
Recall by distance, mini_cone, model `models/detector/det_v31_s.onnx`, split test, conf 0.35 (vision/recall_by_distance.py)

**datasets/cone_check_normals (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 55 | 55 | 0 | 0 | 20.0 % | 20.0 % | - |
| 1-2 m | 58 | 57 | 0 | 1 | 8.6 % | 8.8 % | - |
| 2-3 m | 23 | 20 | 0 | 3 | 4.3 % | 5.0 % | - |
| > 3 m | 28 | 17 | 11 | 0 | 0.0 % | 0.0 % | 0.0 % |
| all | 164 | 149 | 11 | 4 | 10.4 % | 11.4 % | 0.0 % |

| mini_cone | 152 | 0.193 | 1.000 | 0.112 |
| **mAP50** | | **0.835** | | |
mAP50 over the 7 old classes with GT: 0.921
