# Phase 9d: fetch commands for the agent (mock backend only)

Status 2026-11-26: implemented and tested **offline against the mocked robot**, on branch `phase9a-arm`. **No Gazebo, no
real arm, no LLM.** The DEV suite below was written by the assistant while building the code and is **not held-out**.
The user writes the blind held-out suite.

> **Update (Phase 9f, `docs/results/phase9f_small_items.md`):** LLM picker scaffolding for free phrasings
> (`brain/fetch/picker.py`, constrained JSON, colour / room must be the user's own words, safety before and after the
> model), tested with a FAKE LLM only; and a router fix ('that' / 'this' are pronouns only when no noun follows).

## 1. How it plugs into the agent (read first: Phases 5–7)

The agent (`brain/agent/graph.py`):
* e-stop and untrusted-text checks first
* then deterministic routes for special topics: Phase 6 incidents, a `nlu.parse` → tool → template answer that never
  goes through the planner
* then the fine-tuned planner, which emits tool calls from `brain/tools/specs.py`
* then the safety checker (`safety.yaml`), the executor, and the grounding check on the reply

**The fetch tools follow the incident pattern:**
* They are not in `brain/tools.TOOLS`. The planner prompt is unchanged, and the fine-tuned model never saw these tools.
* `Agent._fetch_route` (new, ~20 lines) runs before the incident route, but only if the backend has a `fetch_session`
  attribute. Every existing backend has none, so behaviour is unchanged. The full brain suite and the offline keyword
  baseline are identical: 57.0 % on testset v1 with and without the change, measured on a copy with the committed
  `graph.py`.
* A fetch command never reaches the planner, so the LLM cannot invent a pick-up. A text the router does not recognise
  returns None and goes to the planner as before.

```
utterance -> brain.fetch.router.parse (deterministic)            None -> planner, as before
          -> safety: refuse_reason (throw / people)  -> object resolution (map, colour, room; ask if ambiguous)
          -> safety: object_allowed (class, mass)    -> strict tool calls (brain.fetch.specs.check)
          -> FetchMachine (state machine over the backend)  -> Outcome (status, code, log, confirmed events)
          -> replies.outcome_reply (template from the Outcome) -> replies.safe (grounding rule) -> reply
```

## 2. Tools (strict specs, `brain/fetch/specs.py`)

| tool | arguments | notes |
|---|---|---|
| `pick_up` | `object {class (enum), color (enum), room}`, `room?` | the class enum includes the big classes and `mannequin`, so the SAFETY rule, not the schema, refuses them with a reason |
| `bring_to` | `target: user \| home \| room`, `room` (required iff `target = room`) | |
| `place` | `room` (a room or `home`) | |
| `hand_over` | none | |

Validation is the project's own strict validator: no extra keys, no type coercion. Every call the session makes is
validated (tested).

## 3. State machine (`brain/fetch/machine.py`)

```
pick:    FIND -> APPROACH -> FINE_ALIGN (<= 3 tries, residual <= 2 cm) -> GRASP (<= 3 attempts) -> VERIFY_GRASP -> FOLD_ARM => holding
deliver: CARRY (<= 0.08 m/s) -> PLACE (release)        => placed
                              -> HAND_OVER (user takes) => handed_over
```

Every transition is logged with time and result. `Outcome.confirmed` holds only events that really happened (`picked`,
`placed`, `handed_over`, `dropped`).

| failure | code | reply (template) | robot afterwards |
|---|---|---|---|
| not in the map, not found in any room | NOT_FOUND | "I couldn't find the red can: I looked in every room." | empty |
| no path to the object / destination | NO_PATH | "I can't get there: there is no path (…)." (+ "I'm still holding it.") | empty / holding |
| fine alignment > 2 cm after 3 tries | NOT_REACHABLE | "… couldn't line my gripper up with it (residual 6 cm after 3 tries). Can you move it into the open?" | empty |
| grasp / verify failed 3 times (incl. slip) | GRASP_FAILED | "I couldn't get a grip on the red cube after 3 attempts. It is still where it was." | empty |
| dropped while carrying | DROPPED | "I dropped the red can on the way. It's on the floor in room A. Shall I try to pick it up again?" (object position updated) | empty |
| user does not take it | NOT_TAKEN | "You didn't take the red can, so I'm still holding it. …" | holding |
| 'it' with nothing held | NOT_HOLDING | "I'm not holding anything." | empty |
| holding something else | BUSY | "I'm already holding the red cube. Tell me where to put it first." | holding |

## 4. Safety rules (`brain/fetch/safety.py`, all checked before anything moves)

1. **Only the five small pickable classes:** small_box, mini_toolbox, can, mini_cone, mini_extinguisher (Phase 9b).
   Big classes are refused with the reason, e.g. "The barrel is too big for me to pick up: its narrowest side is about
   56 cm and my gripper opens 7 cm and it weighs about 10 kg and I can carry 80 g."
2. **Mass ≤ 0.08 kg** (Phase 9a payload); a heavier object is refused (TOO_HEAVY).
3. **Never a person:**
   * any people word as the object ("pick up the person", "carry the child", "lift me", "pick her up")
   * the mannequin class
   * Reply: "I never pick up, carry or grab people."
4. **No throwing:** throw / toss / hurl / fling / chuck / lob, and "drop it on him". The reply offers to bring and hand it
   over instead.
5. **Carry speed ≤ 0.08 m/s:** `carry_speed()` is used for every motion while an object is held; the mock records every
   speed and the tests assert the limit.
6. **Grounding** (`replies.ground`): a sentence claiming picked up / got it / holding / put … in / placed / delivered /
   here you are / handed is allowed only if the matching event is in `Outcome.confirmed`, or the robot verifiably still
   holds the object. Otherwise the reply becomes "I could not confirm that it worked." Negated sentences ("I couldn't
   …") are not claims. All template replies pass (tested; the DEV runner checks every reply of every turn).
   * The rule caught two of my own template bugs while building: "Say 'put it down in room …'" in the NOT_TAKEN reply,
     and the BUSY reply before the session counted the robot's carrying state as confirmed. Both are fixed.

**Reference resolution:**
* Objects come from the semantic map (mock: lab4_fetch ground truth), filtered by colour and room.
* More than one candidate → "I know more than one can: the blue can in room A or the red can in room C. Which one?",
  answered by colour, room or class ("the red one", "the one in room C", "the mini cone"), narrowing again if needed.
* An unknown colour or object is never replaced by another ("I don't know any purple cube, and I won't guess another
  object").
* **Generic words:** "cone", "extinguisher", "toolbox" and "box" name a big lab class AND a small pickable one. If only
  small ones match (e.g. "the orange cone") the small one is meant. If both match, the robot asks. "big / traffic / fire
  …" means the big one, which is refused as too big.

## 5. Results (offline)

**Unit tests:** `brain/tests/test_fetch.py`, **58 pass**. They cover:
* specs
* throw / people / normal-request rules, classes and payload, carry speed
* catalog parity with `gurdy_core.arm`
* full state-machine log, 7 failure modes, regrasp, drop
* the grounding rule (8 cases)
* router (11 utterances, object room vs destination room)
* session: clarify, cone family, pronouns, no substitution, busy, all calls valid
* **agent integration through `Agent.say`**: routes only with a fetch backend; refusals execute nothing; "go to room B"
  still reaches the planner

The whole brain suite: **1007 passed, 3 skipped** (host venv with PyYAML + langgraph, `PYTHONPATH=gurdy_core`).

**DEV suite** (`brain/eval/fetch_dev_v1.yaml`, run with `python3 -m brain.eval.fetch_dev`; NOT held-out):

| group | cases | pass |
|---|---|---|
| fetch (bring / put / take / home / pick then hand / pick then place / ambiguity / cone family) | 16 | 16 |
| safety (too big ×5, people ×3, throw ×2, unknown object, nothing held, busy) | 13 | 13 |
| failures (not found, not reachable, grasp failed, regrasp OK, slip, dropped, drop + retry, not taken, no path ×2) | 10 | 10 |
| not fetch (must fall through to the planner) | 3 | 3 |
| **total** | **42** | **42** |

* Every reply of every turn passes the grounding rule.
* **One expectation was changed after the first run** (41/42 at first). In room A both the big red box and the small red
  cube exist, so "pick up the red box" is genuinely ambiguous: the robot asks instead of refusing. A new case
  `s02b_big_red_box` checks that "the big red box" is refused. Because I changed an expectation after seeing the output,
  this suite cannot measure generalisation. That is the held-out suite's job.

## 6. Honest limits

* **Mock only.** The mock's find / approach / align / grasp / carry are scripted: success unless a fault is injected,
  with time from distance and speed. None of the real-robot difficulty (Phase 9a final approach, contact grasping) is
  tested here.
* **Deterministic router:** only the phrasings it knows. Free phrasings ("could you go grab that tin thing for me") fall
  through to the planner, which has no fetch tools and will say it can't, or plan something else. An LLM picker (like
  the Phase 6d incident picker) is the next step and must be validated like that one.
* **The detector does not know the small classes yet** (Phase 9b retrain). In the mock the semantic map is the ground
  truth.
* **"user" = the operator at home (0, 0)** in the mock. There is no person tracking: "bring it to me" on the real robot
  needs the user's position (voice direction, phone location, or "I'm in room C").
* **Not wired to a real backend.** `RosBackend` has no `fetch_session`. That needs the arm, MoveIt, the wrist camera and
  the Phase 9a/9b/9c pieces in Gazebo.
* The web / voice paths inherit it through the agent; not tested.

## 7. Files

| file | what |
|---|---|
| `brain/fetch/` | `catalog`, `specs`, `safety`, `mock`, `machine`, `replies`, `router`, `session` |
| `brain/agent/graph.py` | `_fetch_route` (guarded; no effect without a fetch backend) |
| `brain/tests/test_fetch.py` | 58 tests |
| `brain/eval/fetch_dev_v1.yaml`, `brain/eval/fetch_dev.py` | DEV suite + runner |
