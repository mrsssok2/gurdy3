# Phase 9h: fetch on the ROS backend (written 2026-11-27, NOT run in ROS / Gazebo)

Branch `phase8c-wire`. Until now only the mock (`brain/fetch/mock.py`) had a fetch backend. `brain/ros/fetch_robot.py`
(`RosFetchRobot`) implements the same primitive interface on the ROS side, so `FetchSession` + `FetchMachine` (router,
safety, clarifications, multi-object, grounding, replies) run unchanged on top of it.

**Behind a flag, default off.** It is used only with `run_text --fetch` / `FETCH=true scripts/start_web_robot.sh`. That
start command switches to world `lab4_fetch`, adds `arm:=true object3d:=true` and starts `move_group`. With the default
`FETCH=false` the stack and the agent are exactly as before: fetch phrases go to the planner.

## Architecture

* **Body:** `RosBackend` + `RosPort` (unchanged): Nav2, pose, `/cmd_vel`, semantic map, e-stop. The search for unseen
  objects is RosBackend's own `find` (room sweep + explore).
* **Arm:** new `ArmPort` protocol (`brain/ros/arm_port.py`):
  * `RclpyArmPort`: arm_controller trajectory action, MoveGroup action, gripper effort topic, `/joint_states`, triggered
    wrist camera, Nav2 `/speed_limit`. Own node + executor thread; same interfaces as `scripts/grasp_test.py`. Not run.
  * `FakeArmPort` (tests): a kinematic arm, synthetic wrist depth (synth_depth) and contact-like grasp rules over FakePort.
* **Blocking:** `FetchSession.handle` blocks inside `Agent.say` (agent lock held), so the fetch robot ticks the backend
  itself. The lock-free e-stop path (`AgentHost.estop` → `/estop`) still works: every wait loop checks
  `estop_active()` and raises `FetchAborted`. The session replies "I stopped (emergency stop). I'm still holding the X."
  or "... I'm not holding anything."; the queue and pending question are cleared. You cannot talk to the agent while a
  fetch runs, apart from the e-stop.

## Primitives

| step | real-robot behaviour |
|---|---|
| find | the object is a semantic-map track (`object3d` mode: x / y / z / yaw); names `obj<id>`. Nobody has seen one → `search(ref)` = RosBackend.find (sweep + explore), then the session looks again (hook used only when the backend has `search`; the mock has none, so it is unchanged) |
| approach | grasp-zone pose: the object must end up at the class zone BEHIND the robot. 16 candidate headings: base pose = object − R(yaw)·zone, footprint ring free in `/map`, path planned. Nav2 to a stand-off 0.25 m further out FACING the object, turn round there, back up 0.25 m on the pose (closed-loop `/cmd_vel`, the wrist camera looks behind). Turning in place right next to the object could sweep a leg into it |
| fine_align | arm to the view pose; `VisualServo` with the wrist camera; body steps = closed-loop `/cmd_vel` moves (≤ 6 cm, 5 mm tolerance); done when aligned or arm-reachable |
| grasp | wrist-view yaw (`gurdy_core.wrist_yaw`; confidence < 0.5 → roll 0) → `grasp_test.grasp_plan` at the servo estimate → open → pre-grasp (MoveIt if `/move_action` answers, else a joint goal; a MoveIt failure falls back) → straight approach (joint goal) → squeeze 3 N per finger (contact only) |
| verify | lift 5 cm (joint waypoints). Fingers neither fully closed (≤ 4 mm) nor more open than the object, **and** from the view pose nothing still STANDS at the old spot (`wrist_yaw.standing_points`: 0.5–2.5 cm band above the fitted floor). Failure → open, back off; the machine retries (3 attempts) |
| fold | `folded` (payload pose) |
| carry | Nav2 with `/speed_limit` = carry speed (≤ 0.08 m/s), lifted afterwards; fingers fully closed on the way = dropped (Nav2 cancelled, drop point reported) |
| place | the class grasp pose behind the robot (object back on the floor), open, back off, fold; the new position overrides the stale map track until the detector sees it again |
| hand over | turn so the arm (rear) points at the user, present the object (IK: 0.35 m behind, 15 cm up, horizontal approach; fallback `ready`), light grip 1 N; fingers closing = the user took it → open, fold; timeout 20 s → NOT_TAKEN (keeps holding) |

## Tests (`brain/tests/test_ros_fetch.py`, 20; fakes only)

* all 5 classes fetched end to end
* the approach puts the object in the zone behind the robot
* the cube's top grasp uses the wrist yaw, and MoveIt is used only for big moves; without MoveIt everything is a direct goal
* place in a room + the map override; pick, then place in two turns
* slip / miss detected by verify, 3 attempts; drop while carrying; user does not take it
* the hand-over turns the back to the user
* two objects one after another
* an unseen object is searched for (RosBackend.find mission)
* e-stop mid-task
* the mock session is unchanged; the hand-over pose is feasible

Suites: brain 1100 passed / 3 skipped (fetch DEV 42/42), gurdy_core 400, vision 51, web 118 OK.

## Not verified / limits

* **Nothing ran in ROS.** RclpyArmPort, `/speed_limit` (controller_server must subscribe, nav2_params), the MoveGroup
  execution, the trigger camera and the effort sign of the fingers all come from the grasp-test assumptions
  (`phase9g_grasp_test.md`).
* **The fake grasp is a rule**, not physics. Gazebo contact grasping is what `scripts/grasp_test.sh` will measure first;
  run it BEFORE `FETCH=true`.
* **Room names** come from the lab4 layout (`brain/fetch/mock.ROOMS`, as in the mock); only lab4 / lab4_fetch.
* **Body steps** go through `cmd_vel_smoothed` (min-speed lift 0.09 m/s, collision monitor): centimetre steps may
  overshoot. The servo re-measures, but the step tolerance is a guess.
* **The blind back-up** relies on the map footprint check plus the servo; the 9c low-obstacle layer is not used here.
* **The detector** must find the small classes (needs the v3 detector, Phase 9b, not trained); with det_v2_s no small
  object is ever in the semantic map, so fetch would always search and fail honestly.
