Recall by distance, small classes, model `models/detector/det_v3_s.onnx`, split test, conf 0.35 (vision/recall_by_distance.py)

**datasets/synth_v3_body (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 256 | 253 | 0 | 3 | 98.4 % | 99.6 % | - |
| 1-2 m | 317 | 292 | 8 | 17 | 89.9 % | 96.2 % | 37.5 % |
| 2-3 m | 90 | 53 | 23 | 14 | 45.6 % | 75.5 % | 0.0 % |
| > 3 m | 188 | 7 | 172 | 9 | 2.7 % | 42.9 % | 1.2 % |
| all | 851 | 605 | 203 | 43 | 68.5 % | 95.2 % | 2.5 % |

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
| 1-2 m | 327 | 305 | 5 | 17 | 88.1 % | 94.4 % | 0.0 % |
| 2-3 m | 94 | 75 | 8 | 11 | 44.7 % | 56.0 % | 0.0 % |
| > 3 m | 194 | 104 | 85 | 5 | 3.1 % | 5.8 % | 0.0 % |
| all | 871 | 737 | 98 | 36 | 67.5 % | 79.8 % | 0.0 % |

**datasets/synth_v31_wrist (test)**

| distance | instances | labelled | unlabelled, expected tiny | unlabelled, other (occluded / cut) | recall all | recall labelled | recall on tiny unlabelled |
|---|---|---|---|---|---|---|---|
| < 1 m | 456 | 454 | 0 | 2 | 98.9 % | 99.3 % | - |
| 1-2 m | 32 | 6 | 4 | 22 | 12.5 % | 66.7 % | 0.0 % |
| 2-3 m | 34 | 1 | 21 | 12 | 2.9 % | 100.0 % | 0.0 % |
| > 3 m | 66 | 1 | 65 | 0 | 1.5 % | 100.0 % | 0.0 % |
| all | 588 | 462 | 90 | 36 | 77.7 % | 98.9 % | 0.0 % |

