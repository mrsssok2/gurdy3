# Gurdy 3.0

Builds on Gurdy 2.0 (github.com/mrsssok2/gurdy2, frozen at commit 4c7d047c490efefdce13526f18f7db52b700e327). The **Gurdy 3.0 results** come first below; the rest of this page is the Gurdy 2.0 baseline.

**Try the live demo: [mrsssok2.github.io/gurdy3](https://mrsssok2.github.io/gurdy3/) - runs in your browser, voice included.** Press *Connect to my robot* on that page to drive your own robot over Tailscale.

**What it does:** a six-legged robot in simulation that you can talk to in plain English, typed or spoken (Whisper on the GPU, ~0.3 s per command, 3.7 % word error rate). It walks with a learned (reinforcement-learning) gait, maps and explores a building on its own, recognises objects by name, and plans missions with a fine-tuned LLM running locally.
**What it proves:** the whole stack runs offline on one 6 GB laptop GPU. Measured: the learned gait tracks commands more accurately and uses less energy than a classic gait, the robot mapped 99 % of a lab, found 11/11 objects, and the fine-tuned LLM scored 94 % vs 59 % for the base model on 100 commands, with no hallucinated replies reaching the user.
**What did not work:** it is simulation only, 72.5 % on unseen casual phrasing, and leg-failure adaptation missed its target (see Known limitations).

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

Offline command tests did not drop (94 % / 95 % / 75 %), and the brain test suite passes (800+ tests).

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

**Results** (offline replay on the recorded graph of one live Gazebo run, `gurdy-sft-400`; details in `docs/results/phase6e_blind.md`):
| question set (20 each) | picker off | picker on |
|---|---|---|
| blind_v1 (questions written by the user, committed before any run) | 14/20 | **15/20** |
| v4 (written by the router's author) | 4/20 | **12/20** |

On blind_v1 the five non-incident questions ("is room Q real?", "go to room C", "what do you see right now?", "how many legs do you have?", "stop") passed 5/5: no incident tool was called for the four negatives and room Q got an honest "there is no room Q". Offline command tests did not change (94 % / 95 % / 75 %), and the brain test suite passes (845 passed, 3 skipped).

**Honest limits**
- On the blind set the picker adds only +1 (14 to 15); on v4 it added +8. v4 was written by the author of the router, so it is not independent.
- Failures on blind_v1: "any rooms you skipped?" and "tell me the story of the red box" go to the wrong tool, "missing for more than five minutes" is read as a room-age threshold, one answer lists a false "mannequin is back" (graph noise), and one check was too strict (my scoring error, left as run).
- The graph still produces false "missing" / "back" reports (4 in the last live run). Phase 6e added a debounce (the spot must be in clear view for at least 4 s; "back" needs two sightings), but it is validated only by unit tests: the replay reuses the old recorder's events and cannot show it. It has not been run live yet.
- Replay numbers come from one graph and temperature 0, so the 3 runs were identical and say nothing about variance. 20 questions means one question is 5 points. The picker has not been run live in Gazebo.

### Start in the lab4 world
```bash
WORLD=lab4 MEMORY=false GUI=true scripts/start_web_robot.sh tailscale voice
```
