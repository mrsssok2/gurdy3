# Phase 5 design — Gurdy Brain

Status (2026-11-17): design, plus **5.2 part 1 built** (`brain/tools`, `brain/mock`, 48 tests: tool layer,
`RobotBackend`, mock world; agent graph, safety checker and mission executor still to do). Nothing measured yet.
Every VRAM / RAM / latency / training-time number in this file is an **estimate** unless it says
"measured"; milestone 5.1 replaces the estimates with measurements before anything else is built.

Scope: English only. ROADMAP Phase 5 feature areas 5.A–5.K (acceptance criteria live there);
this file is the how, and the build order (5.1–5.11, section 7).

Decisions that need the user's approval are collected in section 8.

---

## 1. Architecture

### 1.1 Principle: the LLM chooses, the tools know

The LLM does exactly three things: (1) turn an English utterance into tool calls, (2) decide to
ask / confirm / refuse, (3) phrase an answer from tool results. It holds **no facts** about the
world. Positions, distances, object lists, battery, "why did you stop" all come from a tool call in
the same turn, and a checker (1.5) rejects answers that state something no tool returned.

### 1.2 Layers

```
 mic ─ VAD ─ Whisper ─┬─ e-stop fast path (regex, no LLM) ──────────────► /estop
 text UI ─────────────┘                                                     │
                      ▼                                                     ▼
              ┌───────────────── LangGraph agent (brain/agent) ─────────┐  cmd_vel_gate_node
              │ route → plan (LLM) → safety check → execute → observe   │  (last node before /cmd_vel)
              │        ▲                 │ confirm / refuse      │      │
              │        └── ask user ◄────┘                       ▼      │
              │ respond (LLM) → grounding check → text ─► TTS            │
              └──────────────┬───────────────────────────────▲──────────┘
                             │ tool calls (typed)             │ events (alerts, task finished)
                    ┌────────▼────────────────────────────────┴───────┐
                    │ Tool layer (brain/tools): schemas + validation   │
                    │ RobotBackend protocol                            │
                    ├───────────────────────┬──────────────────────────┤
                    │ RosBackend            │ MockBackend              │
                    │ (gurdy3_agent, rclpy)  │ (pure Python, fast)      │
                    └──────────┬────────────┴──────────────────────────┘
        Nav2 actions · explorer action · semantic map · costmap · TF · status · events · memory DB
```

- **`RobotBackend`** is one Python protocol with one method per tool. The agent, the safety
  checker, the evaluation harness and the training-data generator only ever see this protocol, so
  the same agent code runs against Gazebo (`RosBackend`) and the mock world (`MockBackend`).
- **Mission executor** (deterministic state machine, not the LLM): motion tools do not block the
  dialogue. The LLM emits a plan (ordered tool calls); the executor runs it step by step
  (`pending → running → paused → done | failed | cancelled`). The robot stays able to answer
  "where are you?" or obey "stop" during a two-minute walk. A finished or failed step comes back as
  an event; on failure the LLM is asked once to re-plan or report.
- **Events** flow the other way (section 4.2): ROS monitors publish, the alert manager speaks.

### 1.3 Where each fact comes from (ROS 2 side)

A ROS 2 *action* is a long-running request with feedback and cancel (used for motion); a *service*
is a quick request/response; a *topic* is a stream.

| Fact / capability | ROS 2 source (existing unless marked NEW) |
|---|---|
| Robot pose | TF `map → base_link` (slam_toolbox / AMCL + `odom_tf_node` or EKF) |
| Go to a pose | Nav2 action `navigate_to_pose` (feedback: distance remaining, recoveries) |
| Path length / reachability / "what's blocking" | Nav2 action `compute_path_to_pose` + global costmap `/global_costmap/costmap` |
| Obstacles around the robot | local costmap + `/scan` (self-filtered) |
| Objects | `semantic_map/objects` (JSON, latched; `detector_node`): id, class, colour, x/y/z, hits, score, sigma, first/last seen |
| Mapped / unmapped area | `/map` + `gurdy_core.frontier.map_stats` |
| Exploration | `frontier_explorer_node` — today a run-to-completion node; **NEW**: wrap as action `explore` (start / cancel / feedback) |
| Short walk / turn | **NEW** `walk`/`turn` executor: closed loop on odometry, publishes into `cmd_vel_smoothed` so min-speed lift + collision monitor still apply |
| Places, rooms, home | **NEW** `maps/<map>.places.yaml`: named points and polygons (flat_lab: room A, room B, doorway, home), editable by `save_place` |
| Battery | **NEW** `battery_sim_node` (`sensor_msgs/BatteryState`; energy integrated from joint power — a model, always labelled *simulated*) |
| Leg health | **NEW** `leg_health_node`: sustained per-leg joint tracking error from `/joint_states` vs commands |
| Mode / current task | mission executor state; `/policy/status` |
| Why something happened | event log (section 4.2) |
| Camera frame | `camera/image` (320×240) for `describe_view` / `inspect` |
| E-stop, no-go zones | **NEW** `cmd_vel_gate_node`, Nav2 keepout filter (section 4.3) |

New messages go into `gurdy_interfaces` (`Event.msg`, `Explore.action`, `GetObjects.srv`).

### 1.4 Agent graph (LangGraph)

LangGraph is a library for writing an agent as an explicit graph of steps with shared state and
checkpoints — we get a testable state machine instead of a free-running loop.

| Node | LLM? | Job |
|---|---|---|
| `route` | no | e-stop regex; "yes/no" answers to a pending confirmation; everything else → `plan` |
| `plan` | yes | system prompt + tool schemas + state block (1.6) + utterance → tool call(s), `ask_user`, or refusal. Output constrained to a JSON schema (no unparseable calls) |
| `safety_check` | no | validate arguments, resolve targets, apply rules of section 4.3 → allow / needs confirmation / refuse (with reason) |
| `execute` | no | query tools run inline; motion tools are handed to the mission executor |
| `observe` | no | append tool results; loop to `plan` if the model asked for another query (max 4 LLM calls per turn) |
| `respond` | yes | phrase the answer from the tool results only |
| `grounding_check` | no | section 1.5; on failure regenerate once, then fall back to a template built from the tool result |

Simple commands ("go to room B") need one LLM call (`plan`), and the spoken acknowledgement is a
template ("Going to room B, 4.2 metres."), so latency is one short generation.

### 1.5 Grounding check ("never invented")

Run on every reply before it is spoken, and reused as the hallucination metric in evaluation:

1. Extract entities from the reply: object classes and aliases (`gurdy_core.vision.classes`),
   colours, place names, numbers with units (m, cm, %, °, s, min), direction words.
2. Each must be supported by a tool result **of this turn** (or by the user's own words): names
   must appear; numbers must match a returned value within rounding (distance ±0.1 m, percent ±1).
3. Direction words (left / right / behind / in front) are never computed by the LLM: tools return
   them ready-made (`bearing_word`), and the check compares words.
4. If the tool said `not_seen` / `unknown`, the reply must not contain a position.

`describe_view` output is the one exception: it is passed through marked as *visual description
(unverified)* and is never written to the semantic map or to memory as fact.

### 1.6 Prompt state block (small, regenerated every turn)

```
STATE  mode=idle task=none battery=82% (simulated) pose=room A
FOCUS  object=#4 box red   place=room B   last_task=go_to(room B) done
PENDING confirmation: none
KNOWN PLACES home, room A, room B, doorway
```

Pronouns and ellipsis ("go there", "and then left", "do it again") resolve against `FOCUS`, which
the tool layer updates deterministically — the model does not have to dig through history.

---

## 2. Tools

Conventions (all tools):

- Every result is `{ok, data, error}`; `error = {code, message}` with a fixed code list
  (`NOT_SEEN, AMBIGUOUS, UNKNOWN_PLACE, NO_PATH, BLOCKED, IN_NO_GO_ZONE, NEEDS_CONFIRMATION,
  REFUSED, BUSY, TIMEOUT, ESTOP, UNAVAILABLE`). `AMBIGUOUS` carries the candidates.
- Every fact carries `stamp` (sim time) and `age_s`; positions are in the `map` frame, metres.
- Objects are returned as
  `Obj = {id, class, color, x, y, distance_m, bearing_deg, bearing_word, place, last_seen_age_s, score}`
  where `bearing_word ∈ {ahead, ahead-left, left, behind-left, behind, behind-right, right, ahead-right}`.
- Directions are **enums, never signed numbers** (Gurdy 1.0 lesson: direction errors).
- Motion tools return immediately with `{task_id, state: "started", estimate: {distance_m, duration_s}}`;
  the final result below arrives as a `task_finished` event.
  `stopped_reason ∈ {reached, done, blocked, no_path, cancelled, timeout, estop, fell}`.

### 2.1 Motion and missions

| Tool | Inputs | Final result (`data`) |
|---|---|---|
| `walk` | `direction: forward\|backward\|left\|right`, `distance_m: 0.1–3.0` | `moved_m, stopped_reason` |
| `turn` | `direction: left\|right`, `angle_deg: 5–360` | `turned_deg, stopped_reason` |
| `go_to` | exactly one of: `place: str` · `object: {class, color?}` or `object_id: int` · `point: {x, y}`; optional `standoff_m` (objects, default 0.6) | `reached: bool, target: {...resolved}, final_distance_m, travelled_m, duration_s, stopped_reason` |
| `find` | `object: {class, color?}`, `approach: bool = false`, `max_time_s = 300` | `found: bool, object: Obj\|null, source: "memory"\|"live", searched_s, areas_searched`. Order: semantic map → explore frontiers → revisit viewpoints |
| `explore` | `max_time_s = 600`, `area: place?` | `mapped_pct, new_objects: [Obj], travelled_m, duration_s, stopped_reason` |
| `patrol` | `places: [str]` or `"all"`, `loops: 1–10` | per place: `reached`, `new / missing / moved: [Obj]`; totals |
| `inspect` | `object_id` or `object: {class, color?}` | `confirmed: bool, object: Obj, photo_path, moved_m` (vs stored position) |
| `return_home` | – | as `go_to` |
| `stop` | – | `stopped: true, cancelled_task` (immediate; also reachable without the LLM) |
| `mission_control` | `action: pause\|resume\|cancel` | `state, task` |
| `schedule` | `delay_s: int`, `call: {tool, args}` | `schedule_id, fires_at` |

### 2.2 Questions (all inline, read-only)

| Tool | Inputs | Result (`data`) |
|---|---|---|
| `where_is` | `object: {class, color?}` | `matches: [Obj]` (empty + `NOT_SEEN` if never seen) |
| `whats_nearby` | `radius_m = 2.0` | `objects: [Obj]` nearest first, `obstacles: [{bearing_word, distance_m}]` (costmap sectors), `place` |
| `where_am_i` | – | `x, y, heading_deg, place, home_distance_m, localisation: ok\|uncertain` |
| `relation` | `a: object ref`, `b: object ref` | `a_from_b: bearing_word (seen from the robot), distance_m, same_place: bool` |
| `path_check` | target as in `go_to` | `reachable, path_length_m, eta_s, blocked_by: Obj\|"unknown obstacle"\|null, leaves_mapped_area, crosses_no_go` |
| `status` | – | `battery_pct (simulated), legs: {name: ok\|degraded\|failed}, mode, task, speed_mps, uptime_s, estop, alerts_active` |
| `why` | `about: stop\|last_task\|alert\|refusal` | `events: [{stamp, type, severity, text}]` — logged causes only, empty list allowed |
| `recall` | `what: object\|mission\|place`, `query: {class?, color?, since_s?}` | stored records with `age_s`, `stale: bool`, `map_id` |
| `describe_view` | `question: str?` | `text` (VLM, unverified), `detections: [Obj]` (detector, verified), `photo_path` |
| `report` | `mission_id?` (default last) | `summary: {duration_s, distance_m, objects, alerts}, report_path` |

### 2.3 Configuration and dialogue

| Tool | Inputs | Result |
|---|---|---|
| `no_go_zone` | `action: add\|remove\|list`, `place: str` or `around_object_id + radius_m` | `zones: [...]` (remove always needs confirmation) |
| `save_place` | `name: str` (current pose) | `place` |
| `ask_user` | `question: str`, `options: [str]?` | – (ends the turn; used for clarification) |

24 tools. That is a long prompt for a 1.7B model; 5.3 measures tool-selection accuracy with the
full list. If it is poor before fine-tuning, the fallback is a two-stage call (pick the tool group
first), not fewer capabilities.

ROADMAP names map as: `find_object → find`, `query_map → where_is / whats_nearby / relation /
path_check / where_am_i`, `query_status → status`, `query_memory → recall`,
`query_incidents → why` (event log now, Incident Graph in Phase 6), `pause/resume/cancel → mission_control`.

---

## 3. Models, serving, VRAM / RAM budget

### 3.1 Measured starting point (2026-11-17)

- GPU: 6141 MiB total; **2570 MiB in use right now** by the `det_v2_s` training — nothing in
  Phase 5 touches the GPU until that container is gone.
- WSL: 7.9 GiB RAM + 8 GiB swap; 5.2 GiB used with the training running.
- Nav stack while exploring: top-8 process RSS sum 1.0 GiB (`phase3_explore.md`); Gazebo GUI
  550–576 MiB, headless 245–290 MiB (Phase 1).
- On disk: Qwen3-0.6B 1.5 GB, Qwen3-1.7B 3.8 GB, whisper-large-v3-turbo 1.6 GB (`~/hf_models`).

### 3.2 Model choices

| Role | Choice | Runs on | Est. VRAM | Notes |
|---|---|---|---|---|
| LLM (planner + responder) | **Qwen3-1.7B**, fine-tuned, GGUF Q4_K_M, context 4096, thinking off | GPU | 1.6–2.0 GB | Q5_K_M / Q8 compared in 5.5; **Qwen3-0.6B** Q8 (est. < 1 GB) is the CPU / low-VRAM fallback and the second row of the results table |
| STT | Whisper large-v3-turbo, CTranslate2 (`faster-whisper`), `int8_float16` | GPU | 1.0–1.6 GB | the downloaded HF weights need a one-off CT2 conversion; CPU `int8` measured as the alternative |
| VAD | Silero VAD | CPU | 0 | end-of-speech detection |
| Detector | YOLO11s ONNX (`det_v1_s` → `det_v2_s`), onnxruntime CUDA | GPU | 0.4–0.6 GB | measured 177 FPS GPU / 35 FPS CPU, so CPU is a real fallback at the 15 Hz camera |
| TTS | **Piper**, English medium voice, ONNX | CPU | 0 | runs as a separate process; alternative Kokoro-82M (Apache-2.0). Check the licence of the Piper build at install (the maintained fork is GPL) and keep it outside our MIT code, like Ultralytics |
| VLM | ~2B class, 4-bit GGUF; first candidate **Qwen3-VL-2B**, then Moondream2, SmolVLM2 | GPU, on demand | 2.0–2.8 GB | chosen by measurement in 5.10 on the 30-image sim set; loaded only for `describe_view` |
| Gazebo rendering | camera + depth + lidar (+ GUI) | GPU | unknown | measure in 5.1 |

### 3.3 Serving: Ollama (llama.cpp inside), transformers only for training

| Option | Verdict |
|---|---|
| **Ollama** | **Chosen** (already the plan in CLAUDE.md). GGUF, one container, loads / unloads models on request (`keep_alive`), serves the VLM too, JSON-schema constrained output, works CPU-only |
| llama.cpp `llama-server` | Same engine, finer control (GBNF grammars, exact chat template). Fallback if Ollama's template or tool handling does not reproduce the fine-tuned model's training format |
| vLLM | No: reserves most of the VRAM up front and is built for many users; we have one user and four models sharing 6 GB |
| transformers + PEFT | Training and the fp16 reference for the GGUF parity test only; too slow / too much VRAM for runtime |

Known trap, tested in 5.5: the chat template in the Ollama Modelfile must be identical to the
training template (Qwen3 tool-call tags, thinking disabled). A parity test compares GGUF output with
the transformers output on 50 prompts, the same idea as the ONNX parity test of the walking policy.

### 3.4 What runs at the same time

Estimated VRAM (to be replaced by measurements in 5.1):

| Set | Members | Est. total |
|---|---|---|
| **A — normal operation** | Gazebo rendering + detector + LLM + Whisper | 3.5–5.0 GB |
| **B — "what do you see?"** | Gazebo + detector + VLM; LLM and Whisper unloaded for the call, reloaded after | 3.0–4.2 GB |
| **C — tight fallback** | Gazebo + detector + LLM on GPU; Whisper on CPU; VLM swaps with the LLM | 2.5–3.4 GB |
| Training (SFT / DPO) | QLoRA alone, nothing else on the GPU, no Gazebo | 3.5–4.8 GB |

Rule proposed (replaces "one GPU workload at a time" for Phase 5, see section 8): **one training
job at a time and never next to a simulation; the runtime inference set is allowed together if the
measured peak stays ≤ 5.0 GB** (1 GB headroom for WSLg and fragmentation). If set A does not fit,
use set C. Model swaps for the VLM cost a reload (seconds, measured in 5.10), so `describe_view`
answers "Let me look…" first.

### 3.5 RAM and the WSL limit

Estimated RAM for a full voice run in Gazebo: sim + Nav2 + SLAM + perception ≈ 1.6–2.0 GiB
(1.0 GiB measured for the nav stack + sim headless, plus detector), Ollama 0.5–1.5 GiB (weights are
memory-mapped and partly stay in page cache), Whisper 0.8–1.2 GiB, agent + TTS 0.4 GiB, Docker /
system 1.0 GiB → **4.5–6 GiB of 8**. Fits on paper, with little margin once the Gazebo GUI and
VS Code server are open.

Recommendation: **raise `.wslconfig` to `memory=10GB`** (swap stays 8 GB) before 5.6, not higher:
Windows, Docker Desktop and the browser need the remaining ~6 GB, and the GPU driver also uses
shared system memory. QLoRA training itself needs < 6 GiB RAM and does not require the change.
5.1 measures the real peak; if it is below ~6.5 GiB the limit can stay at 8.

---

## 4. Memory, alerts, safety

### 4.1 Memory

**Session** (in RAM, checkpointed by LangGraph to SQLite so a crash does not lose the dialogue):
last 6 turns verbatim, the `FOCUS` slots (1.6), the pending confirmation, the active mission plan.
Older turns are dropped, not summarised by the LLM (a summary would be an ungrounded text).

**Persistent** — one SQLite file `data/memory/gurdy.db` (gitignored):

| Table | Content |
|---|---|
| `objects` | track id, class, colour, x/y/z, `map_id`, first / last seen, hits, score, state (`present \| missing \| moved`), photo path |
| `sightings` | object id, stamp, position — history for change detection |
| `places` | name, point or polygon, `map_id`, kind (`room \| waypoint \| hazard`) |
| `no_go_zones` | polygon, `map_id`, created by / when |
| `missions` | utterance, plan, per-step results, start / end, distance, alerts |
| `events` | copy of the event log (4.2) for `why` |
| `schedules` | pending scheduled calls |

Rules: every record is tied to a **`map_id`** — coordinates from one SLAM run mean nothing in
another, so cross-session memory only works with `nav:=localization` on a saved map. Every answer
from memory carries its age ("12 minutes ago"); older than a threshold (default 30 min sim time) is
flagged stale; an object not re-detected when its position is clearly in view becomes `missing`
(this is also the change detection of 5.E).

### 4.2 Event bus and proactive alerts

Transport: ROS topic `/gurdy/events` (`gurdy_interfaces/Event`: stamp, type, severity, source,
`dedup_key`, JSON data), mirrored to `logs/events/*.jsonl`. Phase 6 later reads the same stream
into Redpanda — no second telemetry format.

| Event | Producer | Trigger |
|---|---|---|
| `path_blocked` | nav monitor | Nav2 recoveries / plan failure on an active goal |
| `object_new`, `object_moved`, `object_missing` | semantic-map differ | confirmed track appears / differs from memory |
| `stuck` | motion monitor | commanded speed above the min-speed lift, odometry < 20 % of it for 5 s |
| `fall` | policy / IMU monitor | tilt beyond the fall threshold |
| `leg_failure` | `leg_health_node` | sustained tracking error on one leg |
| `battery_low`, `battery_critical` | `battery_sim_node` | 25 % / 10 % (simulated) |
| `map_changed` | costmap differ | new static obstacle on a previously free path |
| `task_started / finished / failed`, `estop`, `refusal` | executor, safety | lifecycle |

Alert manager (no LLM — templates filled from the event, so alerts are fast and cannot
hallucinate): severity `info | warning | critical`; one alert per `dedup_key` per cooldown (30 s
default, 5 min for info); critical interrupts speech and pauses the mission; info is only spoken
when idle. Detection-to-alert latency is logged per event for the AC.

### 4.3 Safety layer

Three independent levels; the lower ones do not import the LLM and keep working if it is stopped.

1. **`cmd_vel_gate_node`** (last node before `/cmd_vel`): `/estop` latched true → publishes zero
   velocity and blocks everything until an explicit release; also stops if the robot pose is inside
   a no-go zone margin or the event bus reports `fall`. The mission executor cancels Nav2 goals on
   the same signal.
2. **E-stop fast path**: a regex over stop words and their STT confusions ("stop", "halt",
   "freeze", "stap", "top"…) runs on every transcript *before* the LLM, on short streaming Whisper
   windows while the robot is moving, and on a button in the text UI. Target < 0.5 s sim time from
   transcript to zero velocity; the STT delay is reported separately.
3. **`safety_check` on every tool call** (deterministic rules on the *resolved* call):

| Outcome | Rules (thresholds in one YAML) |
|---|---|
| **Refuse** | target or path inside a no-go zone; target in unknown or occupied space; place tagged `hazard` (stairs, edge); closer than 1.0 m to a `mannequin` (stand-in for people); any motion except `stop` / `return_home` at critical battery or with a failed leg; requests to disable safety, hit or push things; anything outside the tool list |
| **Confirm first** | path longer than 8 m; goal leaves the mapped area; `explore` / `patrol` below 30 % battery; removing a no-go zone; clearing memory; `patrol` with more than 3 loops |
| **Clarify** | `AMBIGUOUS` (two red boxes), unknown place, low-confidence parse (missing required slot, or STT confidence below threshold) |

No-go zones are enforced twice: Nav2 costmap keepout filter (planner never routes through) and the
gate node (pose check). To verify in 5.7: that republishing the keepout mask updates a running Nav2.

A pending confirmation holds the exact resolved call; "yes" executes that call and nothing else,
anything other than a clear yes cancels it, and it expires after 30 s.

---

## 5. Evaluation

### 5.1 The 100-command test set (`brain/eval/testset_v1.yaml`)

| # | Category | Count | Expected behaviour |
|---|---|---|---|
| 1 | Single motion / navigation (`walk`, `turn`, `go_to` place / object / point, `return_home`, `stop`) | 20 | execute |
| 2 | Search and inspection (`find`, `inspect`, `explore`, `patrol`) | 12 | execute |
| 3 | Spatial questions (where, nearby, left / right of, which room, how far, what is blocking), incl. objects never seen | 14 | answer / "not seen yet" |
| 4 | Status, why, memory | 8 | answer |
| 5 | Multi-step missions (2–4 steps, incl. pause / cancel mid-way) | 10 | execute in order |
| 6 | Multi-turn dialogues (pronouns, ellipsis, corrections) | 8 | resolve reference |
| 7 | Ambiguous requests | 8 | clarify |
| 8 | Unsafe (refuse) and risky (confirm) | 10 | refuse / confirm |
| 9 | Noisy-STT versions of categories 1, 2, 7, 8 (real Whisper errors, 5.2 below) | 10 | as the clean version |

Each case: scenario (world, seed, robot pose, which objects are already in the semantic map,
battery, zones), utterance(s), expected behaviour class, allowed tool-call traces (argument
matchers), end-state predicates, answer facts. Test templates, phrasings and object / place
combinations are **held out** from the training generator.

Separate sets required by the ROADMAP ACs, built by the same machinery: spatial QA (≥ 30),
multi-turn (≥ 20 dialogues), why (≥ 20), e-stop STT variants, VLM images (≥ 30).

### 5.2 Automatic scoring

| Metric | How it is computed |
|---|---|
| Task success | end-state predicates on the backend: robot within tolerance of the target, object found, nothing moved for questions / refusals, mission steps in order |
| Tool-call exact match, argument accuracy | first call vs allowed traces; per-argument match |
| Direction errors | left / right / forward / backward swapped (own column — the Gurdy 1.0 failure) |
| Clarification | asked on ambiguous / unnecessary on clear |
| Refusal, over-refusal, confirmation violations | behaviour class vs expected; a risky action executed without confirmation counts as a violation (target 0) |
| Hallucination rate | grounding check (1.5) over all replies + probes about unseen objects and unknown places |
| Latency p50 / p95 | per stage: STT, LLM (time to first token, total), tools, TTS first audio; LLM calls per command |

Each system runs at temperature 0, three scenario seeds; results as mean ± SE over cases (lesson
from Phase 2: report SE, and evaluation inputs never depend on the training config).

### 5.3 Mock world vs Gazebo

**Mock world** (`brain/mock`, pure Python, no ROS, no GPU except the LLM):

- 2D occupancy grid generated from the same world descriptions as the Gazebo worlds (flat_lab,
  semantic_lab, plus random two-to-four-room layouts), ground-truth objects, a *seen* subset as the
  semantic map (objects become seen when inside the camera cone, with the measured position noise
  model `obs_sigma`).
- Kinematic robot on a grid planner at the measured speeds, virtual clock (a 2-minute walk takes
  milliseconds), simulated battery, scripted events (blocked door, leg failure, moved object).
- Implements `RobotBackend`, so agent, safety layer, mission executor, memory and scorer are the
  real code. Not simulated: walking dynamics, Nav2 behaviour, detector errors, timing.
- Budget: 100 commands ≈ a few minutes per system, limited by LLM latency → used for every
  prompt / model / adapter iteration and for the four-system table.

**Gazebo** (full stack, headless, one process group per case with a hard timeout — lesson from the
render job): a 20–25 case subset covering every category + the demo missions, run overnight.
Reported next to the mock result for the same cases; the pass / fail agreement rate tells us how
far the mock numbers can be trusted.

**Voice path** is scored separately so the text results stay repeatable: test utterances are
synthesised (TTS, speed / pitch variation, added noise), fed to Whisper → WER, e-stop recall,
STT latency; then end-to-end on the Gazebo subset.

Systems compared: keyword parser (regex baseline, written in 5.2) · base Qwen3 (prompt only) ·
SFT · SFT + DPO, each for 1.7B and 0.6B; plus Q4 vs fp16 for the selected model.

---

## 6. SFT + DPO plan

### 6.1 Automatic data generation (`brain/data_gen`)

1. **Sample a scenario** in the mock world (layout, objects, seen subset, pose, battery, zones, focus).
2. **Sample an intent** from a grammar over the 24 tools, including multi-step plans, follow-ups,
   ambiguous cases (two matching objects, missing slot), unsafe / risky cases (rules of 4.3) and
   questions about things never seen.
3. **Gold behaviour by construction**: the intent *is* the tool call; it is executed in the mock
   world, so the tool results in the example are real outputs, and the gold decision
   (execute / clarify / confirm / refuse) comes from the same `safety_check` code used at runtime.
4. **Utterance**: template grammar (many phrasings per intent, casual to precise) plus paraphrases
   written by the base Qwen3-1.7B, kept only if an automatic slot check finds the same class,
   colour, place, number and direction.
5. **Noisy-STT variants**: synthesise the utterance with the TTS (speed, pitch, noise), transcribe
   with Whisper, keep transcripts that differ → real error distribution of our own STT; plus a small
   confusion table (left / lift, right / write, four / for, cone / comb, patrol / petrol).
   Gold stays the clean intent, or "clarify" when the slot check says the meaning was lost.
6. **Gold answer**: template filled from the tool result (grounded by construction), several styles.
7. Validation: every example replays through the tool validators and the grounding check;
   duplicates removed; train / val / test split by template and by world.

Format: Qwen3 chat template with tool definitions, thinking disabled; loss on assistant tokens only.

### 6.2 Sizes

| Split | Size | Content |
|---|---|---|
| SFT train | ~12,000 | 5,000 single commands · 2,000 questions (spatial / status / memory) · 1,500 multi-step · 1,500 multi-turn · 2,000 clarify / confirm / refuse / not-seen; ~30 % noisy-STT |
| SFT val | 600 | same mix, other templates |
| DPO pairs | ~3,000 | see 6.4 |
| Test | 100 + AC sets | section 5, held out |

Starting sizes; a learning curve at 3k / 6k / 12k on the 0.6B model (cheap) shows whether more data helps.

### 6.3 QLoRA settings for 6 GB

| Setting | Qwen3-1.7B | Qwen3-0.6B |
|---|---|---|
| Loading | 4-bit NF4, double quantisation, bf16 compute (bitsandbytes) | bf16, no quantisation |
| LoRA | r 16, α 32, dropout 0.05, all attention + MLP projections | same |
| Sequence length | 2048 (tool block + state + dialogue; measured in 5.4, examples above the limit dropped) | 2048 |
| Batch | 2 × gradient accumulation 8, gradient checkpointing on | 8 × 2 |
| Optimiser | AdamW 8-bit paged, lr 2e-4, cosine, 3 % warm-up, 3 epochs | same, lr 3e-4 |
| Est. VRAM | 3.5–4.8 GB | 2–3 GB |
| Est. time | 3–6 h (overnight) | ~1 h |

Pilot first: 200 steps to measure VRAM, steps / s and a falling loss, then the full run with
checkpoints every 500 steps and resume. If 1.7B does not fit at 2048 tokens: batch 1, then shorter
tool descriptions, then Colab / Kaggle with the same script.

### 6.4 DPO (TRL, on top of the SFT adapter)

DPO (Direct Preference Optimisation) trains on pairs "this answer is better than that one" for the
same prompt; it sharpens behaviour choices that SFT alone leaves soft.

| Pair type | Chosen | Rejected | Share |
|---|---|---|---|
| Ambiguous request | `ask_user` | a guessed tool call | 20 % |
| Clear request | direct tool call | unnecessary question | 15 % |
| Unsafe request | refusal with the reason | the tool call | 15 % |
| Safe but scary-sounding | tool call | refusal | 10 % |
| Factual question | query tool call | answer from thin air | 20 % |
| Unseen object | "not seen yet" (+ offer to search) | invented position | 10 % |
| Noisy direction | correct direction / clarify | swapped direction | 10 % |

Rejected answers come first from **the SFT model's own mistakes** (run it over ~5,000 fresh
generated prompts, keep the failures), topped up with programmatic corruptions. The balanced pairs
(clarify vs unnecessary clarify, refuse vs over-refuse) are there so DPO does not just make the
model timid. Settings: β 0.1, lr 5e-6, 1–2 epochs, reference = frozen SFT model with reference
log-probs precomputed (no second model in VRAM), same 4-bit loading; est. 1–3 h. The TRL DPO API is
checked against the installed version in 5.5.

### 6.5 Export and comparison

Merge the adapter into the **fp16** base (not the 4-bit one) → GGUF → Q4_K_M (also Q5_K_M, Q8) →
Ollama Modelfile → parity test vs transformers → latency on GPU and CPU-only.

Results table (`docs/results/phase5_eval.md`): keyword parser, base, SFT, SFT + DPO × {0.6B, 1.7B},
all metrics of 5.2, on the same held-out set. If fine-tuning does not beat the base model on a
metric, the table says so. Model card with the same table on Hugging Face.

---

## 7. Milestones (build order)

Working time is an estimate for focused days; overnight = unattended, with log, checkpoints,
hard timeouts and a morning check.

| # | Milestone | Demo at the end | Est. | Overnight | GPU |
|---|---|---|---|---|---|
| 5.1 | **Serving + budget measurement**: `brain` compose profile, Ollama with base Qwen3 GGUFs, Whisper, TTS, VLM candidates; measure VRAM / RAM / latency alone and as sets A–C; decide set and WSL limit | budget table with measured numbers | 1 day | – | yes (after det_v2_s) |
| 5.2 | **Tool layer + mock world + agent core**: schemas, `RobotBackend`, `MockBackend`, safety checker, mission executor, LangGraph graph, grounding check, keyword-parser baseline; unit tests | type commands in a terminal, robot moves in the mock world | 3 days | – | no (can start while YOLO trains) |
| 5.3 | **Evaluation harness + test sets**: 100-set + AC sets, scorer, latency logging; first table: keyword parser vs base models | `phase5_eval.md` v0 | 2 days | – | LLM only |
| 5.4 | **Data generation + SFT**: generator, validation, TTS→Whisper noise round trip, pilot, full SFT for 0.6B and 1.7B, learning curve | SFT vs base on the 100-set | 2 days | 2 nights (noise round trip; SFT 1.7B) | training |
| 5.5 | **DPO + GGUF + comparison**: mistake mining, DPO, quantise, Modelfile, parity, four-system table, model card draft | offline model in Ollama, GPU and CPU latency | 2 days | 1 night (mining + DPO) | training |
| 5.6 | **ROS backend** (`gurdy3_agent`): Nav2 / explorer actions, object service, places file, walk / turn executor; text commands in Gazebo | "go to room B, find the red box, come back" typed, in Gazebo | 3 days | – | set A |
| 5.7 | **Status, events, alerts, safety in ROS**: battery sim, leg health + fault injection, event bus, alert manager, gate node / e-stop, keepout zones; scripted-cause tests | each alert triggered; e-stop latency; 10 no-go runs | 3 days | 1 night (no-go runs + false-alert run) | set A |
| 5.8 | **Voice**: audio through WSLg, VAD, Whisper, TTS, streaming e-stop, voice-path test set, latency | spoken mission; works with network off | 2 days | – | set A |
| 5.9 | **Memory + reports**: SQLite, restart test, staleness, change detection, mission report (HTML / Markdown + photos) | "where did you last see the red box?" after a restart; report file | 2 days | – | set A |
| 5.10 | **Vision**: VLM choice by measurement, load / unload, `describe_view`, 30-image set | "what do you see?" answered; VRAM + latency table | 1.5 days | – | set B |
| 5.11 | **Full evaluation + demo**: all sets, Gazebo subset, mock-vs-Gazebo agreement, final table, demo video, docs, ROADMAP boxes | results table + video | 2 days | 1–2 nights (Gazebo subset × systems) | set A |

Total: about **23–24 working days and 5–6 nights**. 5.2–5.5 need no Gazebo; 5.6 onwards needs the
full stack. Each milestone ends with a commit, measured numbers in `docs/results/`, and a demo.

Feature areas covered: 5.1–5.5 → 5.A; 5.6 → 5.C, 5.F; 5.7 → 5.D, 5.H, 5.I; 5.8 → 5.B;
5.9 → 5.G, 5.J; 5.10 → 5.E; 5.11 → 5.K.

### Code layout

```
brain/
├── tools/        schemas (pydantic → JSON schema), RobotBackend protocol, safety rules + YAML
├── agent/        LangGraph graph, prompts, mission executor, grounding check, alert manager
├── mock/         MockBackend, world generator, virtual clock
├── data_gen/     intent grammar, templates, paraphrase + noise pipeline, DPO pairs
├── train/        sft.py, dpo.py, export_gguf.sh, configs/
├── eval/         test sets, scorer, run_eval.py, keyword_parser.py
├── ollama/       Modelfiles
└── voice/        VAD, STT, TTS wrappers
ros2_ws/src/gurdy3_agent/     RosBackend, agent node, battery_sim, leg_health, monitors, cmd_vel_gate
docker/brain/                agent + voice runtime (FROM the ROS image); training image FROM the vision image
```

Containers: `ollama` (official image, GPU), `brain` (agent + voice, joins the sim container's ROS
graph — same `GZ_PARTITION` / discovery settings as the Phase 1 lesson), `brain-train` (only for
5.4 / 5.5). The existing `gurdy2/vision:cuda` and ROS images are not rebuilt for this.

---

## 8. Decisions and main risks

Decisions (user, 2026-11-17):

1. **GPU rule: allowed.** The runtime set (detector + LLM + Whisper + Gazebo) may run together **if the
   measured peak is <= 5.0 GB VRAM** (5.1 measures it). Training (SFT / DPO) still runs alone, never next
   to a simulation. If set A does not fit: set C (3.4). This replaces "one GPU workload at a time" for
   Phase 5 inference only.
2. **WSL memory: raise to 10 GB before 5.6** (the user edits the file; swap stays 8 GB). In
   `C:\Users\mahadev\.wslconfig`: `memory=8GB` -> `memory=10GB`, keep `swap=8GB`; then run
   `wsl --shutdown` in PowerShell and restart Docker Desktop. 5.1 still reports the real peak.
3. **TTS: measure both Piper and Kokoro-82M in 5.1.** Prefer Kokoro (Apache-2.0, better voice) if its
   time-to-first-audio is under about 1 s on this machine; otherwise Piper.
4. **Cloud voice (Gnani Prisma / Timbre): optional**, behind the same STT / TTS interface; the local path
   is built first and must work with the network off.

Risks, in order of expected trouble:

| Risk | Mitigation |
|---|---|
| Set A does not fit in 6 GB next to Gazebo rendering | measured first (5.1); set C; 0.6B model |
| 24 tools overload a 1.7B model before fine-tuning | measured in 5.3; two-stage tool selection; shorter schemas |
| Ollama template differs from the training template → fine-tuned model looks broken | parity test in 5.5; `llama-server` fallback |
| Microphone / speaker through WSLg into Docker unreliable | wav-file input and text UI are first-class; audio device tested on day 1 of 5.8 |
| Persistent object memory invalid across SLAM runs | `map_id` on every record; memory demo uses localization on a saved map |
| Synthetic templates → model overfits phrasing | held-out templates and worlds, paraphrases, real STT noise, learning curve |
| Mock results too optimistic | Gazebo subset + agreement rate reported next to every mock number |
| Remaining detector false tracks (6 in semantic_lab with det_v1_s) end up as "facts" | grounding guarantees tool-faithful, not world-true answers; replies carry score / age; `inspect` re-verifies; det_v2_s result decides how much this matters |
| Dynamic keepout mask not picked up by a running Nav2 | verify in 5.7; gate-node pose check is independent |

### Decision 2026-11-17: GGUF conversion tool
Convert with an **official tagged release of llama.cpp** from the ggml-org GitHub (clone at a pinned release tag, never master), run inside a container. The tag is recorded in
`docs/results/` when 5.5 does the first conversion. (An earlier attempt with an unpinned download of the converter was blocked.)
