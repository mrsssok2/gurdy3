Recall by distance, small classes, model `models/detector/det_v31_s.onnx`, split test, conf 0.35 (vision/recall_by_distance.py)

**datasets/synth_v3_body (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 256 | 253 | 0 | 3 | 98.4 % | 99.6 % | - |
| 1-2 m | 317 | 292 | 8 | 17 | 91.5 % | 97.6 % | 50.0 % |
| 2-3 m | 90 | 53 | 23 | 14 | 65.6 % | 90.6 % | 39.1 % |
| > 3 m | 188 | 7 | 172 | 9 | 25.0 % | 100.0 % | 21.5 % |
| all | 851 | 605 | 203 | 43 | 76.1 % | 97.9 % | 24.6 % |

**datasets/synth_v3_wrist (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 456 | 454 | 0 | 2 | 98.9 % | 99.3 % | - |
| 1-2 m | 32 | 6 | 4 | 22 | 12.5 % | 66.7 % | 0.0 % |
| 2-3 m | 34 | 1 | 21 | 12 | 2.9 % | 100.0 % | 0.0 % |
| > 3 m | 66 | 1 | 65 | 0 | 1.5 % | 100.0 % | 0.0 % |
| all | 588 | 462 | 90 | 36 | 77.7 % | 98.9 % | 0.0 % |

**datasets/synth_v31_body (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 256 | 253 | 0 | 3 | 98.4 % | 99.6 % | - |
| 1-2 m | 327 | 305 | 5 | 17 | 90.8 % | 97.4 % | 0.0 % |
| 2-3 m | 94 | 75 | 8 | 11 | 63.8 % | 80.0 % | 0.0 % |
| > 3 m | 194 | 104 | 85 | 5 | 24.2 % | 45.2 % | 0.0 % |
| all | 871 | 737 | 98 | 36 | 75.3 % | 89.0 % | 0.0 % |

**datasets/synth_v31_wrist (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 456 | 454 | 0 | 2 | 98.9 % | 99.3 % | - |
| 1-2 m | 32 | 6 | 4 | 22 | 12.5 % | 66.7 % | 0.0 % |
| 2-3 m | 34 | 1 | 21 | 12 | 2.9 % | 100.0 % | 0.0 % |
| > 3 m | 66 | 1 | 65 | 0 | 1.5 % | 100.0 % | 0.0 % |
| all | 588 | 462 | 90 | 36 | 77.7 % | 98.9 % | 0.0 % |

