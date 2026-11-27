# Gurdy 3.0 Roadmap

**Gurdy 3.0 plan (decided 2026-11-21).** Gurdy 2.0 (github.com/mrsssok2/gurdy2, frozen at commit 4c7d047c490efefdce13526f18f7db52b700e327) is the baseline; its roadmap is kept below unchanged as history.
Gurdy 3.0 = in this order:
- [x] 1. **4-room world + held-out eval** (2026-11-22): `lab4` world (self-mapped, 16/16 doorway goals; `docs/results/lab4_world.md`) + 32-mission held-out suite committed before evaluation; 3 runs: mean 80 % (75-84 %), 0 falls, 0 real false claims (`docs/results/phase5_heldout_lab4.md`). Weak spot: object-search phrasings 5/15; no-go routing has no keepout filter
- [ ] 2. **Demo video** of the 4-room world (voice + navigation + object finding)
- [ ] 3. **Phase 6 — Robot Incident Graph** (resumes at step 2, see Phase 6 below)
- [ ] 4. **Extras** (after 1–3): Phase 5b VLN, 5.5 DPO, 5.10 VLM, YOLO retraining, Phase 8 drone, Phase 10 arm (mobile manipulation)

Docker images stay `gurdy2/*` (reused, not rebuilt); compose project `gurdy3`, containers `gurdy3-*` / `gurdy3_*`. Run only one stack at a time (same ports 8080 / 8443 / 11435).

---

# (Gurdy 2.0 roadmap, baseline)

Legend: `[ ]` todo · `[x]` done · **AC** = acceptance criteria (must be demonstrated live with
measured numbers before a phase is ticked). Each phase ends with a short demo (GIF/video + numbers).

RAM rule: Gazebo, RL training and the ops stack never run at the same time.

Structure (decided 2026-11-17): **Part 1 — CORE** = Phases 0–4, Phase 5 core, Phase 6, Phase 7; when it is done, **Gurdy 2.0 is complete**.
**Part 2 — EXTRAS** (after the core, see the banner before Phase 5b) = DPO (5.5), VLM (5.10), Phase 5b VLN, Experiment B v2, YOLO retraining, Phases 8–10.
Existing numbers are kept so references stay valid; extras that live inside a core phase are tagged **[EXTRA]** there.
Core Phase 5 = milestones 5.1–5.4, 5.6–5.9 (memory/reports) and the evaluation + demo of 5.11, **without** VLM and DPO: the core comparison is keyword vs base vs SFT (three systems).

# PART 1 — CORE (Gurdy 2.0)

---

## Phase 0 — Project setup
- [x] CLAUDE.md and ROADMAP.md
- [x] Monorepo folder structure approved (see CLAUDE.md)
- [x] `.gitignore`, LICENSE (MIT)
- [x] `.env.example` (names only), README stub
- [x] Repo moved to WSL filesystem (`~/projects/gurdy2`)

**Phase 0 done.**

---

## Phase 1 — Robot + ROS 2 base
- [x] Docker image: ROS 2 Jazzy + Gazebo Harmonic + ros2_control + MuJoCo; `compose.yaml` with WSLg GUI, port 8080 published
- [x] `robot.yaml` single source of truth (dimensions, masses, limits, damping, gains) + `gait.yaml` + loader/tests in `gurdy_core`
- [x] 18-DOF xacro (6 legs × hip-yaw, hip-pitch, knee), inertias computed from geometry, one-sided knee limits, joint damping/friction, IMU + depth camera frames (Gazebo sensor plugins in next step)
- [x] Gazebo spawn + `ros_gz_bridge` (clock, IMU, camera, depth, ground-truth odom); matching IMU + camera in MuJoCo
- [x] ros2_control via `gz_ros2_control` **effort** interface (joint_state_broadcaster + `gurdy_control/JointPDController` at 500 Hz, same kp/kd as MuJoCo)
- [x] `gurdy_core`: leg FK/IK + tripod gait math (pure Python, unit-tested)
- [x] Tripod gait node following `/cmd_vel` (vx, vy, yaw rate) — tracking results in `docs/results/phase1_tracking.md`
- [x] MuJoCo MJCF generated from the same `robot.yaml`; URDF↔MJCF parity test (masses, inertias, limits, damping)
- [x] Joint step-response parity test: same step command in Gazebo and MuJoCo, compare rise time, overshoot and steady sag (`docs/results/phase1_parity.md`)
- [x] One bringup launch + keyboard teleop (`docker compose up sim`; teleop via `docker compose exec`) — manual teleop session done (user, 2026-11-14)
- [x] Chatter/health metric script (`scripts/parity_experiments.sh stand`, metrics in `gurdy_core.experiments`)

**AC** — ✅ all met 2026-11-14 (results: `docs/results/phase1_parity.md`, `docs/results/phase1_tracking.md`)
- ✅ `docker compose up sim` opens Gazebo GUI through WSLg; RViz shows the robot model and TF.
- ✅ URDF passes `check_urdf`; all 18 joints present; total mass and inertias match `robot.yaml`; knee can never bend backwards (limits + IK test).
- ✅ Robot stands stably for 60 s with no chatter in Gazebo and MuJoCo (worst joint RMS velocity 4.5e-6 rad/s vs limit 1e-2; no armature, passive joint damping).
- ✅ Joint step response: rise time and overshoot match between Gazebo and MuJoCo (rise within 4 ms; tolerances in `gurdy_core.experiments.PARITY_TOL`).
- ✅ Teleop: walks forward, backward, strafes, turns in place (user session, no falls). Measured vx = 0.10 m/s → −8.3 %, yaw = 0.3 rad/s → −11.3 % (limit ±25 %); no fall in the 2-minute automated session.
- ✅ Same tripod gait runs in MuJoCo headless; forward speed +6.5 % vs Gazebo (limit ±30 %).
- ✅ Measured and recorded: Gazebo RTF headless 0.76–0.82, GUI 0.52–0.72; container RAM 245–290 MiB headless, 550–576 MiB GUI.
- ✅ `colcon test` (10) and `pytest` (gurdy_core 33, sim_mujoco 9) pass in the container.

**Phase 1 done.**

---

## Phase 2 — RL locomotion (MuJoCo → Gazebo)
**Final selected policy: `terrain_dr2` 15M** (mentions of `dr_energy_ft` 6M / `terrain_v1` as selected below are intermediate stages).
Plan approved 2026-11-14. Decisions: action option **A** (joint offsets around stance; B = residual on
tripod only as fallback); observation = 3-frame history of proprioception only (gravity vector, gyro,
joint pos/vel, previous action, command, gait clock) — no ground truth; asymmetric critic added by
2.7 at the latest (earlier if tracking stalls); SB3 PPO on CPU, `log_std_init` ≈ ln 0.35, entropy coef
≤ 0.001; reward = joint tracking term exp(−‖e_v‖²/σ_v² − e_ω²/σ_ω²) with command-relative sigmas
(κ 0.8 → 0.5) so standing can't farm tracking reward; optional time-boxed MJX spike after CPU works.

- [x] **2.1** `rl` image + compose profile; env skeleton; benchmark 1/4/6/8/10 workers + one PPO update
  — AC: steps/s and RAM per worker count recorded, n_envs chosen, container < 6 GB
  — done: batched VecEnv 10×4 = 40 envs, 6.8k env steps/s, PPO 3.6k samples/s, 1.75 GiB (`docs/results/phase2_bench.md`)
- [x] **2.2** Gymnasium env (flat): shared obs builder in `gurdy_core`, action mapping, termination,
  command sampling — AC: `check_env` passes; tests: obs layout/dims, no privileged inputs, targets
  always within joint limits, deterministic seeding
  — done: 11 env tests + VecEnv equivalence test pass (`rl/tests/`)
- [x] **2.3** Reward + sanity tests — AC: every term unit-tested; walking at 50 % of command clearly beats
  standing; tripod walker ≥ 2–3× standing under every move command (pure turning included);
  standing wins at zero command; tripod reference return recorded
  — done: tripod 3.7–4.0× standing (combined 11.9×), half speed 2.5–2.8×, zero cmd standing wins (`docs/results/phase2_reward.md`)
- [x] **2.4** `train.py` + configs (flat, no DR), TensorBoard, checkpoints + resume — AC: flat tracking
  error < 20 %, fall rate < 5 % over 100 episodes, resume verified, RAM peak + wall time recorded
  (time box ~60M steps, then asymmetric critic, then option B)
  — done (2026-11-15): from-scratch runs stayed below standing within 2M steps (best: C1, action EMA);
  chosen policy **BC-initialised from the tripod gait** + PPO, 60M steps in 3.84 h, 1.64 GiB:
  tracking error 5.7 % (100 eps), 0 % falls, resume verified (`docs/results/phase2_flat.md`)
- [x] **2.5** `eval.py`: same protocol for policy and tripod; tracking, falls, energy, cost of transport,
  videos — AC: `docs/results/phase2_flat.md`
  — done: policy 5.7 % vs tripod 16.1 % window error, both 0 falls; policy CoT 18.3 vs tripod 7.35 (2.9× energy)
- [ ] **2.6** (optional, time-boxed) MJX + JAX spike on the RTX 4050 — AC: go/no-go from measured
  throughput (≥ 10× end-to-end) and stand/step parity vs CPU MuJoCo
- [x] **2.7** Domain randomization (friction, masses, motor strength, damping, latency, sensor noise,
  pushes, leg failure) + asymmetric critic + energy efficiency (policy CoT 18.3 vs tripod 7.35) — AC: flat AC still met with DR; fall rate < 5 % at DR extremes
  — done (2026-11-15): `dr_energy_v1` (BC-init via flat_bc_v1), 60M steps / 5.03 h; **selected 40M**:
  0 % falls in all 9 DR scenarios, tracking 10.0–12.2 % (flat_bc_v1: 99 % under sensor noise), CoT 7.70
  (tripod 7.35, flat 18.3). Known issues: user target tracking < 10 % narrowly missed (10.9 %); after
  ~45M the policy stops starting left strafes from rest (start_test) → 2.7b (`docs/results/phase2_dr.md`)
- [x] **2.7b** Command distribution with axis-aligned + start-from-rest commands; `start_test`
  as a regression gate in eval; short fine-tune from 40M — AC: 0/48 failed starts, tracking < 10 %, CoT ≤ 11
  — done (2026-11-15): `dr_energy_ft` 10M (10M steps, 0.96 h): start gate 0/48, CoT 6.93, 0 % falls in all 9
  scenarios; tracking first reported 10.9 % (eval bug: graded on its own command mix) → **corrected 9.6 ± 0.4 %
  (met nominally)**; ft-6M 9.2 ± 0.6 % is better in all 9 scenarios (selection 6M vs 10M open).
  Found: dead zone — stands still on vx/vy ±0.03, wz ±0.10 (not reward-optimal; sampling) → fix in 2.8
- [ ] **2.8** Terrain curriculum (heightfield: rough 0.5–2 cm, slopes 5–12°, steps 1.5–3 cm) — AC:
  success rate per level; RL beats tripod on rough terrain. Also fix the small-command dead zone:
  ~15 % small commands (15–40 % of limit), zero commands 15 → 10 %, explicit `/cmd_vel` deadband
  (|v| < 0.015 m/s, |ω| < 0.05 rad/s), start gate extended to small commands (from rest + while walking)
  — in progress (2026-11-15): built `rl/envs/terrain.py` (heightfield per env, 10-level per-env curriculum,
  rough 0.2–4 cm / slopes 2–15° / steps 0.5–5 cm + 10 % flat plane), `gurdy_core/rl_command.py` deadband,
  small commands, `rl/eval.py --terrain` grid + 144-start gate; run `terrain_v1` (init ft-6M, 60M steps)
  training. Baseline ft-6M vs tripod (12 eps/cell): tripod drops to 67–75 % success on 3–3.5 cm steps,
  ft-6M tracking degrades on slopes ≥ 8° (27–74 % error); rough ≤ 2.5 cm easy for both
- [ ] **2.9** ONNX export + `gurdy_policy` ROS node + Gazebo deployment — AC: ONNX == torch (1e-5);
  walks in Gazebo without retraining; no fall in the 2-min protocol; tracking + sim-to-sim gap reported
  — prep done (2026-11-15, `dr_energy_ft` 6M, Gazebo NOT run yet): `rl/tools/export_onnx.py` →
  `models/dr_energy_ft_6M.{onnx,json,_parity.npz}`; ONNX vs torch max diff 3e-7; `gurdy_core/policy_runtime.py`
  (obs/deadband/EMA, == env, tested); node `gurdy_policy` (stand pose on cmd/sensor timeout, `mode`
  param policy|tripod); MuJoCo deployment-path protocol: all segments 2–9 %, no fall. Gazebo run done 2026-11-16 (`docs/results/phase2_deploy_gazebo.md`): 0 falls both models, but target
  (within ~5 pts of MuJoCo) NOT met: ft-6M stalls on 4 segments, terrain_v1 60M within ±25 % (gap up to 22 pts); latency
  + stance-load/contact differences diagnosed; decision pending on closing the gap. Was: re-export with terrain_v1
- [ ] **2.10** Experiment A (rough terrain, RL vs tripod, MuJoCo + Gazebo) and Experiment B (one leg
  disabled) — AC: results tables in `docs/results/`
  - Experiment A: MuJoCo part written (`docs/results/exp_A_terrain.md`, from existing evals): RL beats the tripod on steps >= 3 cm and in tracking error everywhere;
    the tripod does NOT fall or fail on slopes <= 15 deg (so "RL success > tripod" holds only on steps). Gazebo part not done.
  - Experiment B: leg failure (limp/locked, random leg + time, actor blind, critic told) built + smoke-tested 2026-11-16; training `leg_failure` started, eval plan
    in `docs/results/exp_B_leg_failure.md` (run after training)
  - Experiment B evaluated 2026-11-16: **criterion missed** (adapted policy 76 % limp / 50 % locked vs tripod 38 / 37 %, terrain_dr2 88 / 61 %); `leg_failure` not deployed;
    Gazebo part of Experiment A/B still open
  - [ ] **[EXTRA]** **Experiment B v2** (planned 2026-11-16, not started; later overnight run). Why: in v1 the actor sees only 3 frames (60 ms) and cannot tell which leg is dead.
    - Option 1, longer history: actor gets ~0.5–1 s of proprioception (25–50 frames at 50 Hz; 1D-conv or GRU encoder instead of frame stacking).
    - Option 2, RMA-style teacher–student: teacher actor gets the privileged failure info (leg id, limp/locked, time since failure) through a small latent;
      student estimates that latent from the 0.5–1 s history (supervised, then optional joint fine-tune). Only the student is deployed (no ground truth in obs).
    - Failure detector + fallback: detect a dead leg from joint tracking error / missing foot contact (per-leg residual over ~0.3 s), then switch to a
      fault-tolerant gait (5-leg crawl / tripod re-timed without the dead leg, in `gurdy_core`); report detection delay and false-alarm rate on healthy walking.
    - Compare, same protocol as v1 (`rl/eval_failure.py`, limp + locked, random leg/time): open-loop tripod (38 / 37 %), `terrain_dr2` 15M, `leg_failure` 30M,
      history policy, RMA student, detector + fault-tolerant gait. AC: best learned or hybrid controller beats the tripod after the failure in tracking error with
      falls <= 1 %, and no regression on healthy flat/terrain evals (incl. 15° slopes and the start gate).
    - Order: detector + fault-tolerant gait first (no training), then one overnight run (Option 2 preferred; Option 1 is its student without the teacher).

**AC**
- Training runs without freezing WSL (RAM stays under budget; numbers logged); resume from checkpoint works.
- Flat ground: velocity tracking error < 20 % over the command range; fall rate < 5 % over 100 eval episodes.
- Rough terrain: RL success rate higher than tripod gait (reported with numbers).
- Policy walks in Gazebo via `/cmd_vel` without retraining; tracking error reported for both sims.
- Results table for Experiments A and B in `docs/results/`.

---

## Phase 3 — Mapping + navigation
> Gazebo run 2026-11-16 (`docs/results/phase3_gazebo_results.md`): lidar SLAM with GT odom maps well (tripod 2.4 cm / policy 3.1 cm mean error, RTF 0.99); Nav2 with the policy deadlocks at start (MPPI first step 0.04 m/s < new 0.05 deadband); EKF-odometry map distorted despite accurate odometry (open). Checkboxes below not ticked until fixed.
- [~] Demo world with obstacles, rooms and coloured objects — `flat_lab` world written (generator + test); not yet run in Gazebo
- [~] Depth → laser scan (or 3D) for SLAM; slam_toolbox mapping (RTAB-Map optional) — code done (gz lidar + depth→scan, slam_toolbox params); **untested in sim** (`docs/results/phase3_test_plan.md`)
- [~] Nav2 configured for a legged base (velocity limits, footprint, controller tuning) driving `/cmd_vel` — MPPI/Omni params + launch done; **untested in sim**
- [~] Leg odometry + EKF (`odom_source:=ekf`) — code + offline MuJoCo drift test done (1–2 % of path with IMU yaw); **Gazebo GT-vs-EKF comparison not run** (test plan step 6)
- [ ] Works with both tripod gait and RL policy
- [x] Map save/load (`scripts/map_tools.sh`); `explore` behaviour: own frontier explorer (`frontier_explorer_node`), flat_lab from an empty map: 99.1 % (GT odom) / 98.5 % (EKF) of the interior mapped, returns to start — `docs/results/phase3_explore.md`

**AC**
- Builds a usable map of the demo world from teleop and from autonomous exploration. **Met for exploration (2026-11-16, flat_lab, 2 runs).**
- Nav2 reaches ≥ 8/10 goal poses without collision (reported for gait and policy).

---

## Phase 4 — Perception + voice
- [x] CUDA setup: GPU visible in Docker (`nvidia/cuda` image `nvidia-smi` OK, 2026-11-16); `vision` compose profile + `docker/vision` (CUDA 12.1, PyTorch, onnxruntime-gpu) written, **image not built yet**
- [ ] Object detection by name (YOLO small or small VLM), trained/run on local GPU, CPU/ONNX fallback; 3D position from depth
  - [x] prep code (2026-11-16, nothing run in Gazebo/GPU): 7 classes, synthetic dataset generator (Gazebo boundingbox_camera → YOLO labels + depth), `vision/train.py`, `vision/eval_detector.py`, `gurdy_core.vision` (decode/NMS/depth geometry/semantic map), `gurdy_perception` detector node (markers + JSON semantic map), `semantic_lab` world; tests 44 pass. Run commands: `docs/results/phase4_runbook.md`; license notes: `docs/vision_objects.md`
  - [x] 5-scene render trial 2026-11-16: `set_pose` on the static camera, `/cam/bbox`, shared stamps all OK; labels checked (`vision/check_labels.py`, `docs/results/phase4_trial/`)
  - [x] full dataset `datasets/synth_v1` rendered 2026-11-16: 180 scenes, 0 failed, 2136 images (1499/323/314), 3408 boxes; checked (`docs/results/phase4_dataset.md`)
  - [x] trained yolo11n + yolo11s (300 epochs each), exported ONNX 256x320, evaluated on the test split (`docs/results/phase4_detector.md`): **det_v1_s selected** (mAP50 0.916 / mAP50-95 0.855 vs 0.900 / 0.834; GPU 177 vs 197 FPS, CPU 35 vs 93 FPS)
  - [x] semantic_lab Gazebo test 2026-11-17 (explore + detector, `docs/results/phase4_detector.md`): 9/11 objects within 0.02-0.06 m (s run), mannequin never mapped (edge-truncated boxes are skipped by design), 2 unlabelled distractors detected as `box`; single runs
  - [x] Phase 4b (2026-11-17, `docs/results/phase4b.md`): vision image uses CUDA by default (Dockerfile `LD_LIBRARY_PATH`); truncated detections are mapped (position weight 0.25, edge-aware centre) -> mannequin found, 11/11 objects (was 9/11); track confirmation (hits + time span + mean score + scatter), 6 false tracks left (was 9 with the same sightings and the first rules; 4 with stricter rules)
  - [ ] **[EXTRA — YOLO retraining]** `det_v2_s`: yolo11s on v1 + `synth_v2` (240 scenes: 8 unlabelled distractor families, mannequin x3, 1563 mannequin boxes), training started 2026-11-17 06:02 UTC (`vision/runs/train_det_v2_s.log`); then evaluate on both test splits + semantic_lab, compare with `det_v1_s`
- [ ] `find_object(name)` / `go_to(object)` behaviours
- [ ] Cloud STT + TTS (Gnani Prisma / Timbre, key from env var only); offline Whisper fallback

**AC**
- Detects each demo-world object by name; precision/recall measured on a recorded set.
- "Find the red box" style command finds and approaches the object in sim.
- Works with network off (Whisper fallback); STT latency measured for both paths.
- Detector runs on GPU and on the CPU/ONNX fallback; FPS reported for both.

---

## Phase 5 — Gurdy Brain (LLM post-training + assistant)

Design (2026-11-17, plan only — nothing built or measured): **`docs/phase5_design.md`**. Feature areas with their acceptance
criteria are 5.A–5.K below (formerly numbered 5.1–5.11); the build order is milestones 5.1–5.11:

- [x] 5.1 Serving + VRAM/RAM/latency measurement (done 2026-11-17, `docs/results/phase5_budget.md`: set A peak 3.4 GiB VRAM / 4.4 GiB RAM, 8 GB WSL enough, Piper TTS; VLM not measured — 5.10)
- [x] 5.2 Tool layer + mock world + agent core (LangGraph, safety checker, mission executor, grounding check, keyword baseline) → 5.A  (done 2026-11-17; tested with a scripted LLM; real model comes in 5.3)
  — part 1 done (2026-11-17): `brain/tools` (24 specs, strict validator, `RobotBackend`, `ToolLayer` + FOCUS/state block), `brain/mock` (flat_lab world, A*, virtual clock, all 24 tools), 48 tests; still open: safety checker, mission executor, LangGraph graph, grounding check, keyword baseline
- [x] 5.3 Evaluation harness + 100-command test set; keyword parser vs base models → 5.K  (done 2026-11-17: keyword 55, Qwen3-1.7B base 53, 0.6B base 36 — `docs/results/phase5_eval_base.md`; open: real Whisper transcripts for category 9 (5.5), latency per stage)
  — part 1 done (2026-11-17): `brain` Docker image (compose profile `brain`, LangGraph included), `brain/eval` (testset_v1.yaml 100 cases, harness, oracle agent = 100 %), keyword baseline **55 / 100**
    (`docs/results/phase5_eval_baseline.md`), one-LLM re-plan after a failed mission step (ScriptedLLM tests), 3 agent bugs found by the set fixed; open: Ollama client + base-model runs (GPU), latency per stage, real Whisper transcripts for category 9
- [ ] 5.4 Automatic data generation + SFT (0.6B, 1.7B QLoRA) → 5.A
  — data generator built + smoke-tested (2026-11-17, `docs/results/phase5_datagen.md`); full generation started 2026-11-17 (`scripts/p5_datagen.sh`, output `brain/data/full`); open: SFT training script, pilot, full SFT 0.6B / 1.7B, learning curve (sequence length must be >= 3.1k: the 24-tool system prompt alone is ~2.4k tokens)
- [ ] 5.5 **[EXTRA]** DPO + GGUF/Ollama + four-system comparison, model card → 5.A  (core: GGUF/Ollama of the SFT model + model card is still needed for 5.6+)
- [x] 5.6 (live test 2026-11-18, `docs/results/phase5_live_test.md`; open issues listed there) ROS backend `gurdy3_agent` (Nav2/explorer actions, places), typed missions in Gazebo → 5.C, 5.F
- [ ] 5.7 Status, event bus, alerts, safety (e-stop gate, no-go zones) in ROS → 5.D, 5.H, 5.I
- [ ] 5.8 Voice (VAD, Whisper, TTS, streaming e-stop) → 5.B
- [ ] 5.9 Memory (SQLite, per map) + mission reports → 5.G, 5.J
  — memory + alerts done in the mock world (2026-11-18, `brain/memory`, `brain/agent/alerts.py`, `brain/tests/test_memory.py`); open: wire `MemoryBackend` into `start_agent_chat.sh` / RosBackend (live restart test), change detection (`object_moved`), mission report file (HTML/Markdown + photos), alert-manager latency log
- [ ] 5.10 **[EXTRA]** VLM `describe_view` → 5.E
- [ ] 5.11 Full evaluation (mock + Gazebo subset) + demo video → 5.K  (core: keyword vs base vs SFT, no VLM/DPO; the four-system table follows with the DPO extra)

### 5.A Core: tools, data, training
- [ ] Tool schema (24 tools, exact inputs/outputs in `docs/phase5_design.md` §2): walk, turn, go_to, find, explore, patrol, inspect, return_home, stop, mission_control (pause/resume/cancel), schedule, where_is, whats_nearby, where_am_i, relation, path_check, status, why, recall, describe_view, report, no_go_zone, save_place, ask_user
- [ ] Synthetic dataset (English); noisy-STT variants; multi-step missions; multi-turn dialogues; ambiguous and unsafe requests
- [ ] Held-out test split (incl. noisy-STT confusions)
- [ ] SFT (LoRA, PEFT/TRL) on Qwen3-0.6B locally (RTX 4050, 6 GB); Qwen3-1.7B with QLoRA 4-bit if it fits — Colab/Kaggle fallback
- [ ] **[EXTRA]** DPO: prefer clarification over guessing; refuse unsafe actions; prefer tool results over invented facts
- [ ] GGUF quantization, Ollama Modelfile; latency on local GPU and CPU
- [ ] Hugging Face model card (with eval table)
- [ ] LangGraph agent calling ROS 2 tools

**AC**
- Fine-tuned model beats keyword parser and base model on tool-call exact match, argument accuracy and direction errors (numbers in model card).
- Runs offline via Ollama on GPU and CPU-only; latency per command reported for both.

### 5.B Talking
- [ ] English text + voice (cloud STT/TTS, local Whisper + local TTS fallback)
- [ ] Multi-turn memory (pronouns/ellipsis: "and then left from there", "go back")
- [ ] Clarification questions on low confidence/ambiguity
- [ ] Confirmation step for risky actions (long trips, rough terrain, leaving mapped area)

**AC**
- Multi-turn test set (≥ 20 dialogues) ≥ 90 % correct reference resolution.
- Clarification asked on ≥ 90 % of ambiguous prompts; unnecessary clarification on clear prompts ≤ 10 %.
- Risky actions never execute without confirmation (0 violations in test set).
- Works with network off (Whisper fallback); STT/TTS latency measured for both paths.

### 5.C Spatial awareness (tool-grounded)
- [ ] Answers "what's nearby / where is X / left-right-behind of Y / which room / how far / what's blocking" via semantic map, costmap and TF tools
- [ ] Every spatial claim traceable to a tool call (logged); "I don't know / not seen yet" when the tool has no answer

**AC**
- Spatial QA set (≥ 30 questions in a sim world with ground truth): answer accuracy and distance error reported.
- Hallucination check: 0 objects/places stated that are not in the tool output (reported over the whole set).

### 5.D Proactive alerts
- [ ] Event → speech/text alerts: path blocked, new object seen, stuck, fall, leg failure detected, low battery, map changed
- [ ] Rate limiting/dedup so alerts are not spammed; severity levels

**AC**
- Each alert type triggered in sim by a scripted cause; detection-to-alert latency measured.
- False-alert rate over a 10-minute normal run reported (target 0 repeats of the same alert within the cooldown).

### 5.E Vision  [EXTRA — needs the VLM, milestone 5.10]
- [ ] "What do you see?" via a small VLM that fits 6 GB VRAM (e.g. ~2B class, 4-bit); one GPU workload at a time (unload LLM/detector as needed)
- [ ] Change detection vs earlier visits to the same place (objects added/removed/moved)
- [ ] Optional: OCR for signs/labels

**AC**
- VLM VRAM and latency measured; answers on a ≥ 30-image sim test set scored against ground truth.
- Change detection: precision/recall on scripted scene changes reported.

### 5.F Tasks and missions
- [ ] go to, find, explore, patrol, inspect, return home, multi-step missions
- [ ] Pause / resume / cancel mid-mission; schedules ("patrol in 5 minutes")

**AC**
- Spoken multi-step mission ("go to the kitchen, find the red box, come back") executed end-to-end in sim.
- Pause/resume/cancel work in every mission state (state-machine test).
- Scheduled task fires within ±2 s of its time.

### 5.G Memory
- [ ] Last-seen objects and places persist across sessions (SQLite/JSON)
- [ ] Mission history; Incident Graph queries (Phase 6 hook, stubbed until then)

**AC**
- After a restart, "Where did you last see the red box?" answers correctly from memory with timestamp.
- Memory answers carry their age; stale entries flagged.

### 5.H Self-awareness
- [ ] Status: battery, leg health, mode, speed, current task
- [ ] Explains decisions ("why did you stop?") from logged events/state, not invention

**AC**
- Status values match telemetry exactly (test).
- "Why" questions (≥ 20 cases) cite a real logged cause; unsupported cause = fail.

### 5.I Safety
- [ ] Voice e-stop ("stop"/"halt" and noisy variants) bypasses the LLM (keyword fast-path) and stops the robot
- [ ] No-go zones (set by voice, enforced in costmap)
- [ ] Refuses unsafe commands (stairs/edge, humans too close, outside zone)

**AC**
- E-stop: command-to-zero-velocity latency measured (target < 0.5 s sim time), ≥ 98 % recall on STT-variant set, works with LLM stopped.
- Unsafe-request refusal ≥ 95 %, with low over-refusal on safe commands (both reported).
- Robot never enters a no-go zone in 10 navigation runs.

### 5.J Reports
- [ ] End-of-mission summary: duration, distance, map snapshot, object photos with positions, alerts raised
- [ ] Spoken short version + saved HTML/Markdown report

**AC**
- Report generated for each demo mission; all listed objects match the semantic map; photos present.

### 5.K Evaluation
- [ ] ~100-command test set (English), incl. noisy STT, multi-step, ambiguous, unsafe
- [ ] Metrics: task success rate, tool-call exact match, latency (p50/p95), hallucination checks, refusal/over-refusal, clarification rate
- [ ] Compare keyword parser vs base vs SFT vs SFT+DPO

**AC**
- One results table (docs/results/phase5_eval.md) with all four systems; SFT+DPO best on success rate and hallucination rate, or the gap is reported honestly.
- End-to-end demo video: spoken mission, alert, spatial question, report.

---

## Phase 6 — Robot Incident Graph (robot-ops)
**Gurdy 3.0 (2026-11-25): "Incident Graph = temporal memory" built + unit-tested, NOT evaluated** — `docs/phase6_incident_graph.md` (SQLite temporal graph, recorder with move / missing detection, 6 query tools, deterministic routing + grounded answers, web Timeline panel + map arrows; held-out Gazebo scenario + 20 questions committed, not run: `scripts/incident_eval.sh`). The Redpanda / HDBSCAN ops track below stays paused.
**PAUSED (2026-11-18) after step 1** (incident data model + SQLite `IncidentStore`, `ops/incidents`, commit 405c10c). Phase 7 is done (except the public demo deployment), so Phase 6 **resumes at step 2** (event/metric schema, exporter, replay) next.
Design (2026-11-17, plan only): **`docs/phase6_design.md`** (incident definition, data flow, storage, root-cause linking, brain queries, stack budget, milestones 6.1–6.9, ~19 working days estimated).
- [ ] Telemetry/log exporter: ROS 2 → Redpanda (also from rosbag replay)
- [ ] Event normalizer (schema), fingerprinting, embeddings → Qdrant
- [ ] HDBSCAN clustering into incidents → PostgreSQL
- [ ] Redis/Celery workers; FastAPI backend
- [ ] LangGraph investigation agent (logs, config diffs, metrics) → root-cause reports
- [ ] Fault injection: high D-gain chatter, weak motors, slippery floor, disabled leg
- [ ] Recorded telemetry datasets so the stack runs without the simulator

**AC**
- Stack fits in RAM alongside nothing else heavy (measured), runs on recorded telemetry.
- Each injected fault type forms its own incident cluster (purity/ARI reported).
- Investigation agent names the correct root cause for ≥ 3 of 4 fault types.

---

## Phase 7 — Mission control + polish
**Status (2026-11-19): done except publishing the public demo (user pushes + enables GitHub Pages)** (and the open polish items at the end). Design: `docs/phase7_design.md`; decision taken: stdlib Python server + no-build static frontend instead of Next.js / rosbridge / three.js (nothing to install, runs in the existing images, offline-capable).
- [x] API server (`web/api`: REST + WebSocket + shared-token auth + chat log; E-STOP without the agent lock, latched `/estop` on the real robot + Release button)
- [x] Dashboard PWA (`web/app`): laptop dashboard + phone layout with tab bar, canvas map (robot, walked trail, planned path, remembered objects in their colours, collision-free labels), status / battery / legs / alerts, chat with suggestion chips, installable (manifest + service worker). Checked in Chrome on a laptop and on an Android phone, and with headless screenshots of both layouts
- [x] Mock world generated from the Gazebo `flat_lab` files (`brain/mock/lab_world.py`) so the demo shows what the real lab contains
- [x] Real robot in Gazebo: ROS telemetry (pose, `/map`, trail, Nav2 plan, semantic-map objects, battery, legs, alerts), camera panel with detection boxes; `scripts/start_web_robot.sh` (one heavy stack, RAM measured: peak 4.7 GiB WSL with LLM + voice)
- [x] Voice in the browser: hold-to-talk, Whisper on the GPU + Piper (`brain/voice/speech_service.py`), stop fast path, yes/no hint, hallucination filter, transcript bubble with confidence, spoken replies with mute; https with a local CA for LAN (`docs/phase7_voice.md`)
- [x] Private remote access from anywhere: Tailscale Serve (tailnet-only https, valid certificate), `scripts/start_web_robot.sh tailscale voice` (`docs/phase7_remote.md`); used from a phone on mobile data
- [x] Docker compose profile `web`, one-command start scripts; README section with screenshots
- [x] LLM-unreachable fix: agent says "My language model is not running right now.", start script checks the model from inside the container before READY
- [x] Public demo as a STATIC in-browser build (2026-11-19; Hugging Face now requires PRO for Docker Spaces on free CPU): the real Python simulation (mock flat_lab world + agent + smart keyword planner) runs in the browser via Pyodide in a Web Worker, same dashboard UI, one private world per tab, no server (`scripts/build_static_demo.py`, `web/static_demo/`; a 40-line langgraph stand-in with a parity test against the real one; measured 5.6 MiB first load, 8-11 s cold / ~3 s cached). GitHub Pages workflow `.github/workflows/pages.yml` -> https://mrsssok2.github.io/gurdy2/ and a static HF Space alternative (`scripts/demo_space.py`); `docs/phase7_demo_deploy.md`. **Code + tests done; the actual publishing needs the user to push and enable Pages** (the per-visitor server demo mode remains for self-hosting)
- [x] Static demo extras (2026-11-19): **Connect to my robot** (same page -> real robot server over Tailscale; CORS / WebSocket Origin allow-list on the server, token still required) and **hold-to-talk voice in the demo with Whisper in the browser** (transformers.js, lazy ~50 MB model download, vocabulary prompt / stop fast path / yes-no / hallucination filter shared with the server pipeline, speechSynthesis replies); tests `web/tests/test_origin.py`, `web/tests/test_static_demo.py`, `brain/tests/test_transcript_filter.py`, 31 browser checks (`scripts/run_browser_tests.sh`); `docs/phase7_demo_deploy.md`
- [~] GitHub Actions CI: `.github/workflows/pages.yml` runs the demo / web tests before publishing; a full CI (lint, ROS build/test job) and a project website are still open
- [ ] Optional: three.js 3D robot view, incidents page (after Phase 6 resumes)

**AC**
- Fresh clone → each phase's demo starts with one documented command.
- CI green on `main`.

---

# PART 2 — EXTRAS (after the core is complete)

Index (content stays where it was written): 5.5 DPO and 5.10 VLM (Phase 5 milestones, 5.E), Experiment B v2 (step 2.10), YOLO retraining (`det_v2_s`, Phase 4),
then Phase 5b VLN, Phase 8, Phase 9, Phase 10 below. Nothing in Part 2 blocks "Gurdy 2.0 complete".

---

## Phase 5b — End-to-end Vision-Language-Navigation experiment (planned, not started)
Question: can a small end-to-end policy distilled from the modular system (LLM brain + Nav2 + YOLO) match it?
- [ ] Record the modular system completing English missions as demonstrations (camera frames + instruction + `/cmd_vel`, plus outcome labels)
- [ ] Train a small VLN policy (camera + instruction → `/cmd_vel`) by imitation / distillation
- [ ] Compare modular vs end-to-end: success rate, time, collisions, unseen rooms, unseen phrasings
- [ ] Publish dataset and model on Hugging Face (with dataset/model cards)

**AC**
- One results table (docs/results/phase5b_vln.md), real measured numbers over a fixed mission set, incl. unseen rooms and unseen phrasings; gaps reported honestly.

---

## Phase 8b — LiDAR safety shield (control barrier function)
Last motion filter for every velocity source (agent, Nav2, teleop): minimally changes /cmd_vel so no LiDAR point gets
closer than d_safe to the footprint. Results: docs/results/phase8b_safety_shield.md.
- [x] Note on the cmd_vel chain + difference to the Nav2 collision monitor (shield before the E-STOP gate)
- [x] `gurdy_core/safety_shield.py` (CBF + QP, stdlib only) + `gurdy_safety/safety_shield_node` (/cmd_vel -> /cmd_vel_shielded, /shield/status, fail safe on stale / no scan)
- [x] Launch flag `shield:=false` (default off)
- [x] Unit tests + 2D kinematic simulator (walls, doorway, corner, turn, sudden obstacle, stale scan): 0 / 52 shielded collisions, doorway slowdown 0-2 %
- [ ] colcon build + node test in the ROS image
- [ ] Gazebo test (teleop / agent / Nav2 through the flat_lab doorway, with vs without shield); tune d_safe / alpha from the measured stopping distance

---

## Phase 8c — 3D object positions from the depth camera (picking prep)
Box + all depth pixels -> 3D centre, top, height, yaw, covariance; Track3D fusion. Plan and results: docs/results/phase8c_plan.md.
- [x] Offline code + tests (`gurdy_core/vision/object3d.py`, `transforms.py`, `synth_depth.py`), synthetic eval, 3D GT for semantic_lab (2026-11-26)
- [x] Wired into `detector_node` / `SemanticMap` behind `position_method:=object3d` (launch `object3d:=false` default), fitted floor plane (prior z -0.11 m), size-adaptive `box` fit, 3D fields in the semantic map JSON; host-venv tests (2026-11-27, branch `phase8c-wire`)
- [x] Gazebo accuracy-check script (teleport stops 0.6 / 1.0 / 1.5 / 2.5 m x 3 bearings, scorer vs 3D GT) — written, not run
- [ ] colcon build + node smoke test in the ROS image
- [ ] Gazebo convention checks (K, z-depth, floor z, noise) + the accuracy run; calibrate the covariance
- [ ] Depth-noise model in the gz sensor (robot model)
- [x] First grasp test script for Gazebo lab4_fetch (`scripts/grasp_test.sh`, 5 classes x 3 tries, contact grip, lift + hold; `docs/results/phase9g_grasp_test.md`) — written, not run
- [ ] Run it (after day_0927 01 / 04 / 09 PASS)
- [x] ROS fetch backend behind `--fetch` / `FETCH=true` (default off): find / search, approach, servo, grasp, verify, fold, carry, place, hand-over; fake-port tests (`docs/results/phase9h_ros_fetch.md`) — not run in ROS
- [ ] v3 detector (small classes) + live fetch run

---

## Phase 8 — Air-ground team (planned, not started)
A simulated drone scouts from above, maps, detects objects and sends goals to one or more hexapods;
the LLM brain commands the whole team through tools. Builds on Phases 3–7 (map, Nav2, YOLO, Brain,
Incident Graph, mission control).

- [ ] **8.1** Drone in Gazebo Harmonic: multicopter model (gz `MulticopterMotorModel` + IMU/baro/GPS),
  PX4 SITL (or a PX4-style ROS 2 flight controller if PX4 doesn't fit the RAM budget) via the
  uXRCE-DDS bridge; downward RGB-D camera ≤ 320×240. Measure RAM/RTF of drone alone and drone +
  1 hexapod before going further (RAM rule: 8 GB WSL, one heavy workload — may require headless).
- [ ] **8.2** Offboard control from ROS 2: take-off, hold, waypoint, land; `scan_area(polygon, altitude)`
  lawn-mower coverage pattern
- [ ] **8.3** Aerial mapping: top-down occupancy/elevation map from the drone camera, fused with (or
  seeding) the hexapod's slam_toolbox map; shared `map` frame between robots (ground-truth alignment
  first, then marker/feature-based)
- [ ] **8.4** Aerial detection: Phase 4 YOLO on the drone camera → object positions in the map frame
- [ ] **8.5** Team interfaces: namespaced robots (`/drone`, `/hexapod_1`, …), `go_to` / `inspect` actions,
  team state topic; hexapods keep speaking only `/cmd_vel` + Nav2 below
- [ ] **8.6** LLM brain tools for the team: `scan_area`, `go_to(robot, target)`, `inspect(object)`,
  `report`; the Phase 5 agent plans multi-robot missions ("drone, find the red box; nearest hexapod,
  inspect it")
- [ ] **8.7** Multiple hexapods + 1 drone: task allocation (greedy/auction baseline, then optional
  LLM-assisted), collision-free goal assignment
- [ ] **8.8** Telemetry from all robots into the Incident Graph; mission control shows the team
- [ ] **8.9** Worlds: rubble field and farm (rows, crops, obstacles), procedurally generated

**AC**
- Measured RAM/RTF for drone + 1 and + N hexapods (within the 8 GB WSL budget, reported).
- Drone covers a 20 × 20 m area and finds each target object (precision/recall reported); the assigned
  hexapod reaches each found object (success rate over ≥ 10 missions).
- A spoken/typed multi-step team mission is executed end-to-end in sim.
- 2+ hexapods + 1 drone: allocation completes all tasks without collisions in ≥ 8/10 runs.
- Injected faults on drone and hexapods appear as separate incidents with correct root causes.

---

## Phase 9 — Research experiment: hybrid walk + fly robot (planned, not started)
Can one robot both walk and hop/fly? Key constraint: **thrust-to-weight ≥ 2** (hover at ≤ 50 % thrust).

- [ ] **9.1** Design study (in `robot.yaml` style, single source of truth): (a) lighter legs with fewer DOF
  (e.g. 2-DOF legs) + separate rotors, or (b) legs that double as rotor arms (rotor at the hip/femur,
  legs fold for flight). Mass budget incl. motors, props, battery; thrust-to-weight, flight time and
  hover power estimated from component data, then **measured in sim**
- [ ] **9.2** Thrust model in MuJoCo (rotor force/torque on sites, motor lag, yaw torque) and in Gazebo
  (`MulticopterMotorModel`); parity test between the two like Phase 1 (hover thrust, step response)
- [ ] **9.3** RL skills: take-off from legs, stable hover, landing on legs (touchdown velocity limit),
  flight "hops" over obstacles that walking cannot cross; reuse the Phase 2 pipeline (DR, asymmetric critic)
- [ ] **9.4** Mode-switch policy: when to fly vs walk (energy/time cost model: walking CoT vs flight
  power), obstacle-triggered hops; compare against a rule-based switch
- [ ] **9.5** Experiment write-up: energy per metre walking vs flying, success rate over obstacle courses,
  failure modes

**AC**
- Design meets thrust-to-weight ≥ 2 in sim (measured), with mass and power budget documented.
- Take-off → hop over an obstacle taller than leg clearance → landing on legs: success rate over ≥ 50
  sim trials, landing touchdown speed reported.
- Mode-switch policy beats walk-only and fly-only on an obstacle course in time or energy (numbers).

---

## Phase 10 — Mobile manipulation (arm) (planned, not started)
- [ ] Mount a small arm on the hexapod (arm + VLA from the separate "Hukum" arm project)
- [ ] Payload-aware locomotion fine-tune (arm mass / pose changes the CoM; DR over payload)
- [ ] "Walk there and pick it up" missions: LLM brain → navigate to the named object → grasp

**AC**
- Walking tracking/fall rate with arm mounted reported against the arm-free policy; pick-up success rate over ≥ 20 sim trials (numbers).
