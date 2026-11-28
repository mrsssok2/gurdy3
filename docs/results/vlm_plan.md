# VLM scene queries: design (2026-11-28, branch `phase11-vlm`; CPU only, nothing run on the GPU)

Questions like "what's on the table?", "is the path to the kitchen clear?", "what colour is the box near the barrel?" are
answered with a small open vision-language model (VLM) looking at the body camera, **but the robot only states what its own
detector / map / path check confirm**. The VLM is a second witness, never the source of a fact.

## 1. Model (to be measured by `scripts/vlm_eval.sh`; nothing below is measured yet)

| candidate | why | risk |
|---|---|---|
| **Moondream 2** (~1.9 B; Ollama `moondream`, Q4 ~1.7 GB) | smallest; fits next to Gazebo + detector + gurdy-sft-400 (set A measured 3.4 GiB VRAM in 5.1; + ~1.7 GB is near the 5 GB rule) | the Ollama build is an older Moondream; JSON / schema following weak |
| **Qwen2.5-VL-3B** (Ollama `qwen2.5vl:3b`, Q4 ~3 GB) / Qwen2-VL-2B | better instruction following and JSON; colours and counting | does NOT fit next to the full set A: load on demand (keep_alive short, the planner LLM unloaded for the question) |
| Gemma 3 4B (vision) | strong | larger; same on-demand constraint |

Sizes are from memory of the Ollama library and are NOT verified here: the eval records the real VRAM peak (`nvidia-smi`) and
latency per model. Runtime: the project's Ollama (`/api/chat` with `images: [base64]` and `format: <JSON schema>`), the same
transport as the LLM; on-demand loading decided after the measurement. The VLM runs only when asked a scene question.

## 2. The tool `describe_view(question)`

Not added to the planner's tool list (the system prompt stays byte-identical: prompt cache, SFT data). Like the incident and
fetch routes: a deterministic route in front of the planner, behind a flag (`Agent(vlm=DescribeView(...))`, default off).

```
question --classify--> kind: see | on | colour | path | count | other   (+ target: class, colour, near-reference, room)
         --frame-----> the latest body-camera frame (<= 2 s old, else "I don't have a current view")
         --facts-----> detector detections IN THIS FRAME + semantic-map objects in the camera cone (3.5 m, +-45 deg)
                       + for path questions: path_check(room) and whether the route leaves in the camera's direction
         --VLM-------> constrained JSON {"answer", "objects": [{"name", "color", "where"}], "path_clear": true|false|null, "unsure"}
                       (the VLM does NOT see the facts: it is an independent witness)
         --check-----> per VLM object: confirmed | colour conflict | unconfirmed (a detector class nobody detected) |
                       outside the detector (table, ball, sofa ...)
         --reply-----> TEMPLATES from the checked fields only; the VLM's free text is never spoken
```

## 3. Grounding rules (code, `brain/vlm/ground.py`)

1. **An object is stated as present only if a fact supports it** (detector in this frame, or the map in the camera cone).
2. **The VLM alone never adds an object.** A detector class nobody detected → "the camera model mentions a cone, but my detector
   doesn't see one, so I'm not sure". A noun outside the detector's classes (table, ball) → "... mentions a table, which my
   detector can't confirm".
3. **Disagreement → "not sure", with both sides:** "I'm not sure about its colour: my detector says blue, the camera model says
   green." Agreement → a plain statement.
4. **A detector fact the VLM did not mention** is still true and is said ("I can also see a red fire extinguisher").
5. **Path questions:** the geometry (Nav2 path check + the costmap) decides. The VLM is asked only when the route leaves in the
   camera's direction (within +-45 deg); if it disagrees → "not sure" + what the path check says. Not in view → "the way there
   isn't in front of me, so I checked it on the map only".
6. **References are resolved on the facts**: "the box near the barrel" = the box nearest a barrel (<= 2 m), several equally
   near → ask which; none → "I don't see a box near a barrel".
7. **VLM unavailable / bad JSON / says unsure** → facts-only answer, says the camera model is not available / unsure.
8. Every reply passes an invariant check (tests, 500 random VLM outputs on real frames): any object word in a reply sentence
   is a fact, or the sentence says "not sure" / "can't confirm".

## 4. Data used now (CPU)

* **Frames from today's Gazebo runs:** the body-camera detector output per frame (`sightings.jsonl` of the 2026-11-28 fetch /
  bring-me runs: class, colour, 3D position, box, camera pose per frame; e.g. smoke_1128b cube log: 586 frames, 18 with a box
  and a barrel). These are the "facts" side of real frames. No body RGB image was saved by those runs (only wrist DEPTH frames).
* **RGB frames rendered in Gazebo today** (`datasets/cone_check_normals`, 04:05 UTC, 336 images with per-image GT): three are
  copied into `brain/tests/data/vlm/` (with meta + scene file for GT colours) as fixtures; the full set is the GPU eval set.
* The VLM in the tests is a scripted mock.

## 5. GPU eval for later (`scripts/vlm_eval.sh`, under the GPU lock; NOT run)

Per model, on N rendered frames with GT (default 60 of `cone_check_normals` + `synth_v31_body` test):
* VLM alone: object precision (hallucination rate = mentioned detector-class objects not in the frame), recall of the big
  classes, colour accuracy (GT colour from the scene's SDF colour), JSON validity, latency p50 / p95, VRAM peak.
* The full tool with GT as the detector ("perfect detector"): answers correct / not sure / **false statements** (target 0),
  and with corrupted facts (one fact dropped, one colour flipped): the tool must say "not sure" or leave the item out, never
  state the VLM's version as fact.

## 6. Built (2026-11-28, host tests only)

| file | what |
|---|---|
| `brain/vlm/frames.py` | `Frame` (image, detections, camera pose); `frames_from_sightings` (today's detector logs: one frame per timestamp; the `cam` quaternion -> viewing direction, checked on 855 real detections: all 18 outside +-35 deg are boxes cut by the image edge), `frame_from_render` (Gazebo render + GT; GT colours from the scene's SDF colour / small-object colour word) |
| `brain/vlm/questions.py`, `vocab.py` | deterministic question kinds; VLM words -> detector classes (crate -> box, cube -> small_box ...), size siblings (cube ~ box), nouns outside the detector |
| `brain/vlm/client.py` | JSON schema + parser; `OllamaVLM` (`/api/chat`, `images`, schema `format`, temperature 0; default model env `GURDY_VLM_MODEL` / moondream); `ScriptedVLM` |
| `brain/vlm/ground.py` | matching + templates + `check_reply` (the invariant); "near" = the nearest box within 3 m of a barrel, clearly nearer (0.5 m) than the next, else "Which one?" (a real frame of today has the blue box 2.4 m from the blue barrel) |
| `brain/vlm/tool.py` | `DescribeView`: frame (<= 2 s old) + facts (frame detections + map objects in the camera cone) + path check + VLM + check; a runtime guard falls back to the facts-only reply if any sentence states an unsupported object; `ros_frame_source` (RosExtras JPEG + detector boxes; NOT run in ROS) |
| `brain/agent/graph.py` | `Agent(vlm=...)`, `_vlm_route` after the incident route; default off (flag-off behaviour unchanged, tested) |
| `brain/vlm/eval.py`, `scripts/vlm_eval.sh` | the GPU eval (section 5); `MOCK=perfect|hallucinate` runs it without a model |

Tests `brain/tests/test_vlm.py` (33): today's frames (sighting log + 3 renders in `brain/tests/data/vlm`), every question
kind with agreeing / disagreeing / inventing / unsure / failing VLMs, stale view, path in view / not in view / unknown place,
the invariant on 500 random VLM outputs over 5 real frames (0 violations), the runtime guard, the agent flag, the Ollama request,
the ROS adapter, the eval metrics with a perfect and a hallucinating mock (the tool held: 0 false statements), the eval script
with a temporary lock file. Suites: brain 1369 passed (test_final_eval_wall_timeout left out: it takes the real GPU lock), web OK,
safety gate PASS, keyword v1 58 %.

**Found while building:** the reply checker first read the verb in "I **can** see" as the class `can` (fixed: verb uses are
removed before matching); the frames of today's runs have no RGB, so on them the tool correctly never calls the VLM (the tests give
the MOCK a stand-in image and say so).

**Next (needs the GPU, under the lock):** `scripts/vlm_eval.sh` (`PULL=1` the first time: the models are not installed here). Then
decide the model and on-demand loading from the measured VRAM; then save body RGB frames in the Gazebo runs (detector_node
`detector_log` saves detections only) so live frames can be replayed; then a Gazebo check through `ros_frame_source`.
