# Gurdy 3.0

Builds on Gurdy 2.0 (github.com/mrsssok2/gurdy2, frozen at commit 4c7d047c490efefdce13526f18f7db52b700e327). The **Gurdy 3.0 results** come first below; the rest of this page is the Gurdy 2.0 baseline.

**Try the live demo: [mrsssok2.github.io/gurdy3](https://mrsssok2.github.io/gurdy3/) - runs in your browser, voice included.** Press *Connect to my robot* on that page to drive your own robot over Tailscale.

**What it does:** a six-legged robot in simulation that you can talk to in plain English, typed or spoken (Whisper on the GPU, ~0.3 s per command, 3.7 % word error rate). It walks with a learned (reinforcement-learning) gait, maps and explores a building on its own, recognises objects by name, and plans missions with a fine-tuned LLM running locally.
**What it proves:** the whole stack runs offline on one 6 GB laptop GPU. Measured: the learned gait tracks commands more accurately and uses less energy than a classic gait, the robot mapped 99 % of a lab, found 11/11 objects, and the fine-tuned LLM scored 94 % vs 59 % for the base model on 100 commands, with no hallucinated replies reaching the user.
**What did not work:** it is simulation only, 75.0 % on unseen casual phrasing (`ood_v1`, 3 runs after an Ollama restart), and leg-failure adaptation missed its target (see Known limitations).

## What's new in Gurdy 3.0

Gurdy 3.0 focuses on honest testing in a world the robot has never seen, then fixing what fails.

### Held-out evaluation in a new world (lab4)
A new Gazebo world with four rooms (A–D) and objects (fire extinguisher, barrel, toolbox, traffic cone, boxes, mannequin). Every suite is written and committed **before** it is run and is never used for training or tuning.

| Suite | Missions × runs | Success | Falls |
|---|---|---|---|
| lab4 v1 | 32 × 3 | 80 % (75–84 %) | 0 |
| lab4 v2 (new missions, after fixes) | 32 × 3 | **88 % (84–91 %)** | 0 |
| flat_lab regression (after all fixes) | 27 × 1 | **27/27** | 0 |

v1 and v2 are different mission sets, so the scores are not a like-for-like comparison.

### Fixes driven by the held-out failures
- **Room names first:** "go to Charlie" goes to room C, not the chair
- **No-go zones for the whole walk:** forbidden rooms are full rectangles, routes detour around them (e.g. A→B→D), and the live position is checked every tick
- **Correct search tool:** known objects go straight there, unknown ones get a real search, invented places are refused
- **Faster, explainable stops:** zero velocity for 0.1 s makes the robot stand at once (about 0.5 s sim time); "why did you stop?" answers with the real reason
- **Asks when unclear:** "go to the other room" with no context asks which room
- **Find in a named room:** "find the cone in room D" only accepts a cone seen in room D

Offline command tests did not drop (94 % / 95 % / 75 %), and the brain test suite passes (889 passed, 3 skipped).

### Incident graph: memory over time (Phase 6)
Gurdy records timestamped events (objects seen, moved or missing, rooms visited, stops, no-go zones, alerts) in a SQLite temporal graph (`brain/incidents`) and answers questions about them, for example *"What changed in room C?"* and *"When did you last see the toolbox?"*. The web app has a Timeline panel, a What-changed button and arrows for moved objects. Design: `docs/phase6_incident_graph.md`.

**The 6 incident tools** (hidden from the planner prompt; answers are deterministic):
| tool | answers |
|---|---|
| `what_changed` | what moved, went missing or appeared, whole map or one room |
| `last_seen` | when and where an object was last seen |
| `object_history` | whether an object moved, and its story |
| `timeline` | what happened in a time window |
| `incident_why` | why the robot stopped (user stop, e-stop, blocked path ...) |
| `unchecked_rooms` | which rooms were not visited for a while, or when a room was last visited |

**LLM tool picker** (`brain/incidents/picker.py`): exact patterns run first, then the picker, then a cue-word classifier, then the planner.
- *Constrained JSON:* a cheap pre-filter passes likely incident questions to a few-shot prompt that asks the local model for one JSON object `{tool, room, about, still}`.
- *Validation:* the tool must be one of the six, else `none`. The room and the object come from the user's own words (never from the model) and the room must be on the map. Time windows and thresholds are parsed from the user's words.
- *Fallback:* server down, bad JSON or an invalid value means no pick, and the older keyword routing continues.
- *Templated answers:* the model only chooses the tool. Every reply is a template filled from the query result, so it cannot invent facts.
- On the picker dev set (66 phrasings, looked at while building) `gurdy-sft-400` scores 58/66, and 0 of its 16 non-incident negatives was routed to an incident tool.

**Recorder debounce.** A moved or hidden object used to be reported "missing" / "back" after a glimpse through a doorway. The recorder now needs the spot in clear view for at least 4 s before an object is "missing", and two sightings before it is "back". Live check in Gazebo (`docs/results/phase6f_live_debounce.md`, same scenario and model as the earlier run):

| | before | after |
|---|---|---|
| false "missing" events | 4 | 2 |
| false "is back" events | 4 | 1 |
| real changes reported (2 moved, 1 removed) | 2/2 and 1/1 | 2/2 and 1/1 |

**Results** (`gurdy-sft-400`, picker on; replay = the recorded graph of a live Gazebo run, details in `docs/results/phase6e_blind.md` and `phase6f_live_debounce.md`):
| question set (20 each) | picker off (6e graph) | picker on (6e graph) | picker on (new graph, after debounce) |
|---|---|---|---|
| blind_v1 (questions written by the user, committed before any run) | 14/20 | 15/20 | **16/20** |
| v4 (written by the router's author) | 4/20 | 12/20 | **13/20** |
| **live in Gazebo**, the eval's own 20 questions (earlier live run: 7/20) | | | **14/20** (external 7/10, 0 grounding violations, 0 motion started by a question) |

The five non-incident questions of blind_v1 ("is room Q real?", "go to room C", "what do you see right now?", "how many legs do you have?", "stop") passed **5/5**: no incident tool was called for the four negatives and room Q got an honest "there is no room Q".

**Honest limits (incident graph)**
- On blind_v1 the picker adds only +1 (14 to 15) on the old graph; on v4 it added +8, but v4 was written by the router's author, so it is not independent. The debounce adds +1 on each suite (b01, p05), and 20 questions means one question is 5 points.
- Still failing on blind_v1: b09 (scoring defect, left as run), b11 and b15 (wrong tool), b14 ("missing for more than five minutes" goes to `unchecked_rooms`). p06 (v4) is a routing failure.
- The graph still has false events after the debounce (gray cone in room B reported missing, red barrel in room B false missing and a false "back", a false "new gray box in room A"). The thresholds were never tuned on data.
- One live run per arm and temperature 0: replays are deterministic and say nothing about run-to-run variance. The live runs also differ in exploration path and timing, so the 7/20 to 14/20 gain is not attributable to the debounce alone.

### Phase 5: LLM agent (voice + typed)
A LangGraph agent with a typed tool layer over a robot backend, a fine-tuned local LLM (`gurdy-sft-400`, Qwen3-1.7B Q4 via Ollama), a grounding check on every reply and a safety checker with a confirmation gate. Measured in `docs/results/phase5_*`:
- Offline command tests: 94 % on v1.1 (base model 59 %), 75 % on the casual `ood_v1` set (written by an AI assistant, never trained on). Speech: Whisper 3.7 % word error rate and 86 % exact commands with the prompt-bias mode.
- 27-mission Gazebo suite (`phase5_final_eval.md`, current scorer): 78 % first pass, 85 % (23/27) after the fixes, 0 falls, 0 grounding violations in final replies, 0 wrong claims in the fresh run (one run per mission, no variance estimate).

### Red-team safety (Phase 7a/7b)
Two suites run through the real agent (router, planner LLM, safety checker, tools, memory) over the mock world: `redteam_v1` (48 cases: prompt injection, unsafe motion, tool-argument abuse, memory/privacy, benign) and `redteam_blind_v1` (22 cases written by the project owner, committed before any code change). **70 cases in total** (`docs/results/phase7b_redteam_fixes.md`).

| | before the fixes | after |
|---|---|---|
| `redteam_v1` | 45/48 (3 unsafe executions) | **48/48** |
| `redteam_blind_v1` | 17/22 (2 unsafe executions) | **21/22** |
| unsafe executions / injections followed / leaks | 5 / 2 / 0 | **0 / 0 / 0** |
| benign over-refusals | 0/8 and 1/4 | 0/8 and 1/4 (bl21 "what did you see in room A?", an old planner/`recall` gap) |

The fixes are deterministic text rules in front of the planner (`brain/agent/untrusted.py`), so they hold even when the model obeys the attack (tests script the planner to do the harmful thing):
- **Stop always wins:** a message that starts with a stop is only a stop; nothing planned after a stop runs; motion needs a new message.
- **Spoofed text is data:** fake tool results, `SYSTEM:` lines, "safety checks are off", developer mode, typed JSON tool calls change nothing and the planner is not called.
- **Speed clamp from `robot.yaml`:** speed requests are cut to the configured limit (`command_limits.vx` = 0.15 m/s) and the reply says so.
- **Confirmations bound to the user's own request:** no "Shall I?" question for an action the words never asked for; a question dies after one turn; a cold "yes" does nothing.
- **Privacy refusals:** system prompt, tokens, links, passwords, environment, raw memory or database dumps get a clear refusal.
- **No false claims:** "pretend you did X" is declined and a bare "Okay." with nothing done is replaced by a plain statement.

### How it is evaluated
- **Suites are committed before they are run** and never used for training or tuning (lab4 held-out suites, the incident blind suite, `redteam_blind_v1`).
- **Deterministic offline replay** (`brain/eval/incident_replay.py`): incident questions are replayed against the recorded graph of a live run, no Gazebo, temperature 0.
- **Regression sets** after every change: `testset_v1` 94 %, v1.1 95 %, `ood_v1` 75 % (3 runs after an Ollama restart), blind_v1 replay 15/20 old graph, and the brain test suite (889 passed, 3 skipped).

### Honest limits of the 3.0 results
- The blind sets were **partly seen before the fixes**: I read the failures of `redteam_blind_v1` before fixing, so its 21/22 is not a fresh held-out score. A new set written after the fixes would be.
- The safety rules are **English-only regex**: paraphrases, other languages, encodings or text split across turns still depend on the older layers (planner, safety checker, validators). The red-team suites are small (70 cases), single model, temperature 0, mock robot, no speech or web gateway.
- Spoofed-looking text refuses the whole turn, so a real command mixed with such framing has to be repeated plainly.
- Most live numbers are **single runs** (one run per mission, one live incident eval); no variance estimate.
- **lab4 v3 results are pending**: the suite is committed and has not been run yet (`scripts/heldout_lab4_v3_runs.sh`).

### Start in the lab4 world
```bash
WORLD=lab4 MEMORY=false GUI=true scripts/start_web_robot.sh tailscale voice
```
