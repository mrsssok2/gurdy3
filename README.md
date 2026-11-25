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

### Incident graph: memory over time (in progress)
Gurdy records timestamped events (objects seen, moved or missing, rooms visited, stops, no-go zones, alerts) in a SQLite temporal graph and answers questions such as *"What changed in room C?"* and *"When did you last see the toolbox?"*. The web app has a Timeline panel and arrows for moved objects.

In a scripted test (2 objects moved, 1 removed), the graph found both moves in the right rooms and the removal, with false reports cut from 16 to 1 after fixing a sim-time/wall-clock bug. Question answering is still being improved.

### Start in the lab4 world
```bash
WORLD=lab4 MEMORY=false GUI=true scripts/start_web_robot.sh tailscale voice
```
