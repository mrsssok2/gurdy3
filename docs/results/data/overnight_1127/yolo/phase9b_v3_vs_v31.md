| check | target | det_v2_s | det_v3_s | det_v31_s |
|---|---|---|---|---|
| old classes mAP50, synth_v1 test | >= det_v2_s - 0.01 | 0.922 | 0.933 PASS | 0.928 PASS |
| old classes mAP50, synth_v2 test | >= det_v2_s - 0.01 | 0.871 | 0.875 PASS | 0.863 PASS |
| small classes mAP50, synth_v3_wrist test | >= 0.80 | - | 0.991 PASS | 0.988 PASS |
| small classes mAP50, synth_v3_body test | >= 0.70 | - | 0.974 PASS | 0.970 PASS |
| small classes mAP50, synth_v31_wrist test | >= 0.80 | - | 0.986 PASS | 0.984 PASS |
| small classes mAP50, synth_v31_body test | >= 0.70 | - | 0.832 PASS | 0.904 PASS |
| **acceptance (old + v3 sets)** | | | **PASS** | **PASS** |

Per small class, synth_v3_body test (AP50): det_v3_s: small_box 0.971, mini_toolbox 0.960, can 0.972, mini_cone 0.985, mini_extinguisher 0.980; det_v31_s: small_box 0.933, mini_toolbox 0.991, can 0.964, mini_cone 0.982, mini_extinguisher 0.979
Per small class, synth_v3_wrist test (AP50): det_v3_s: small_box 0.978, mini_toolbox 0.990, can 1.000, mini_cone 0.985, mini_extinguisher 1.000; det_v31_s: small_box 0.978, mini_toolbox 0.990, can 1.000, mini_cone 0.972, mini_extinguisher 1.000
