# Multi-step fetch commands (2026-11-28, branch `phase12-int`; fake robot only, nothing run in Gazebo / against Ollama)

"bring me the blue can, then the red cube", "pick up the can and put it in the kitchen".

**Plan** (`brain/fetch/tasks.py`): an ordered list of the existing fetch skills `bring` (pick up + carry + hand over / put down),
`pick_up` (pick up and hold), `put` (put what is held at a place), `hand_over` (give what is held to the user); at most four steps.
Only requests with two or more steps become plans; everything else goes to the single-step router exactly as before.

**Planner:** one LLM call with the plan schema as Ollama `format` (not run), then `validate`; a rejected / unavailable model falls
back to the deterministic keyword parser (clauses split at "then", ", then", "and" + a verb; a verb-less clause such as ", then the
red cube" repeats the previous verb and destination), validated the same way.

**Validator (no invented objects or steps):** every skill needs its verb in the user's words; every object (class + colour) must be
said and each said object is used exactly once (nothing dropped, nothing added); rooms must be said (letters or the map's room
names: hallway, office, living room, kitchen); the gripper must allow each step in order (one thing at a time; `put` / `hand_over`
need something held; a step that names an object cannot be `put` / `hand_over`); the user's own colour word wins over the model's.
Safety refusals (throwing, people) are checked before any plan is made, so they are never planned.

**Execution** (`FetchSession.run_plan`, behind `FetchSession(..., tasks=TaskPlanner(...))`, default off; ROS: `run_text --fetch
--fetch-tasks`): before EVERY step
* stop: the e-stop latched -> stop, list what was not done;
* battery: below `MIN_BATTERY_PCT` (25 %) -> stop, saying the measured level;
* time: more than `MAX_TASK_S` (30 min, robot clock) since the request -> stop;
* gripper: `bring` / `pick_up` need a free gripper, `put` / `hand_over` need something held -> that step is skipped with the reason.
During a step the e-stop wins: a primitive raises `FetchAborted`, nothing after it runs, the reply says what is held and what is not
done. One honest line per step: a done step speaks the machine's confirmed reply ("Here you are: the blue can."); a failed one is
"The red cube: <the machine's own failure reply>"; a `put` after a failed pick is skipped ("I'm not holding anything ..."); a
"which one?" question stops the plan and lists the steps not started.

**Tests** (`brain/tests/test_fetch_tasks.py`, 31, fake robot = FetchMock with new hooks `battery` / `battery_per_m` /
`faults["estop_at_s"]`, plus one on RosBackend + FakePort + FakeArmPort): keyword plans (including room names and "hand it to me"),
single-step texts not taken, validator rejections (invented object, invented step, dropped object, unsaid room, two things at
once, bad JSON), LLM plan used / invalid -> keyword / model down -> keyword, the schema in the Ollama request; runs: two deliveries,
second not found (first still reported done), pick + put in the kitchen by the fake robot's ground truth, put skipped after a
failed pick, gripper busy, low battery with the measured level, time budget, e-stop during a step, e-stop latched before, a
question mid-plan, safety refusal never planned, flag off unchanged; the same plan on the ROS fetch robot.
Suites: brain 1628 passed / 3 skipped (test_final_eval_wall_timeout left out: it takes the real GPU lock); fetch DEV 42/42
(unchanged); keyword v1 58 %; CI local yaml lint core web ros safety results PASS.

**Not done / limits:** not run in Gazebo or against Ollama; a clarifying question mid-plan does not resume the plan after the
answer (the user repeats the rest); battery / time thresholds are defaults, not measured needs; no `go_to` step (fetch skills only).
