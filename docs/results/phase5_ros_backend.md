# Phase 5.6 prep — RosBackend (code only; nothing run in Gazebo)

`brain/ros/`: `backend.py` (`RosBackend`, same `RobotBackend` interface and result shapes as `MockBackend`), `port.py` (`RosPort`: what the backend
needs from ROS, plain Python types), `rclpy_port.py` (real adapter: rclpy node in a background thread; imports checked in the sim image, never run
against a live graph), `fake_port.py` (test double: fake topics / Nav2 / explore action servers), `relmotion.py` (closed-loop walk/turn), `run_text.py`
(type commands against the robot). Places/zones file: `maps/flat_lab.places.yaml`. Tests: `brain/tests/test_ros_backend.py` (35, no ROS).

Design: the backend is stepped by `tick()` (called inside `poll_events()`, so the agent loop drives it) and only reads time from the port (/clock).
Motion tools return `started` at once; the Nav2 goal is sent in the same call; the final result arrives as a `task_finished` / `task_failed` event.

## Tool -> ROS mapping

| Tool | ROS source | Status |
|---|---|---|
| `go_to`, `return_home` | place/object/point resolved here -> `compute_path_to_pose` check -> `navigate_to_pose`; object = nearest of 8 ring points at the standoff, goal yaw faces it | wired |
| `walk`, `turn` | `RelativeMotion` on TF pose, publishes `cmd_vel_smoothed` (min-speed lift + collision monitor stay in the path); stops on no progress for 8 s (`blocked`); a walk whose straight line enters a no-go zone is refused (`IN_NO_GO_ZONE`), leaving the zone you stand in is allowed, entering one while walking stops the task (`no_go_zone`); turning in place cannot enter a zone | wired (the "NEW executor" of design 1.3, as library code; no separate node) |
| `find` | semantic map first; else explore until the object appears, cancel explore, optionally approach | wired (live part needs the `explore` action server) |
| `explore` | `explore` action (`gurdy_interfaces/Explore`, `frontier_explorer_node serve_action:=true`); `area` -> place rectangle (point place = 4 m square) | wired; `UNAVAILABLE` while the server is down |
| `patrol` | sequence of `navigate_to_pose` goals over `maps/*.places.yaml`; a blocked place is recorded `reached: false` and the patrol goes on; new/missing/moved = semantic map diff start vs end | wired (a never-dropped track cannot become `missing`) |
| `inspect` | go to the object (standoff 0.6), face it, `confirmed` = track re-detected in the last 5 s | wired; `photo_path` = None |
| `stop` | cancel goal / explore, zero `cmd_vel_smoothed` | wired; works with no LLM |
| `mission_control` | pause = cancel goal, resume = re-send it (Nav2 has no pause); cancel | wired |
| `schedule` | backend timer on /clock | wired (lost on restart) |
| `where_is`, `relation`, `recall(object)`, `whats_nearby` (objects) | `semantic_map/objects` JSON | wired |
| `whats_nearby` (obstacles) | 8 sectors from `/scan` (the self-filtered one) | wired; sectors with a return only |
| `where_am_i` | TF `map -> base_link` + places file | wired; `localisation` is always `"ok"` (no covariance source yet) |
| `path_check` | `compute_path_to_pose`, `/map` cell of the goal (unknown = `leaves_mapped_area`), zones checked against the plan points | wired |
| `status` | speed (odom), `/policy/status`, IMU tilt, `/estop`, active alerts, `/battery` (SIMULATED), `/gurdy/leg_health` | wired (battery / legs stay unknown until their node runs) |
| `why` | event log: backend task events + alert topic | wired |
| `recall(mission/place)`, `report` | in-memory mission log / places file | wired; lost on restart (SQLite memory is 5.8) |
| `describe_view` | detections seen in the last 1.5 s (detector output); `unverified: true` | partial: no VLM, no photo |
| `no_go_zone`, `save_place` | places file (`no_go_zones`, `places`), enforced by the backend on goals, Nav2 plans, walks | partial: **no Nav2 keepout filter**, a plan that merely cuts a zone is refused |
| `ask_user` | handled by the tool layer | n/a |
| events | `/gurdy/alerts` String JSON `{stamp,type,severity,source,text,data}` (interim); `fall` / `estop` alerts end the active task; same type from monitor and backend within 3 s = one event | producer: `alert_monitor_node` (below) |
| e-stop | `RosBackend.estop(bool)` (not a tool) publishes latched `/estop`, cancels the task, zeroes `cmd_vel_smoothed`; **`policy_node` subscribes to `/estop`**: rate-limited move to the stand pose, ignores `/cmd_vel` until released (policy and tripod mode) | gate node (`cmd_vel` mux for Nav2 / teleop) still not built; `gait_node` does not listen to `/estop` |

## New nodes (code only, unit-tested without Gazebo; none has run in the sim yet)
| Node (package) | What | Notes |
|---|---|---|
| `frontier_explorer_node serve_action:=true` (`gurdy_navigation`) | `explore` action server: goal `max_time_s`, `area_rect`, `return_to_start`; feedback 1 Hz (`phase`, `elapsed_s`, `frontier_goals`, `frontiers_left`); cancel stops the Nav2 goal; goals while a run is active or Nav2 / map / TF are not ready are rejected | default (no `serve_action`) is unchanged: run to completion at launch. Backend sends `return_to_start: false` |
| `gurdy_interfaces/action/Explore` | the action type | new ament_cmake package, builds in 25 s |
| `battery_sim_node` (`gurdy_safety`) | `/battery` `BatteryState`, `location` / `frame_id` = `SIMULATED`, warns at start; 0.03 %/s moving, 0.002 %/s idle (sim time) | set the level: `ros2 param set /battery_sim_node level 15.0` |
| `leg_health_node` | `/gurdy/leg_health` (latched JSON `{"legs": {front_left: ok|failed ...}}`), from `|target - measured|` per leg (EMA 0.8 s) vs 0.15 rad **and** 3x the median of the other legs, held 2 s, only while >= 3 legs are moving; latched until `/leg_health/reset` (Trigger) | **thresholds are guesses, not calibrated on Gazebo**; a failure is invisible while standing; `fake_failed_leg:=rm` forces a failure without physics |
| `alert_monitor_node` | `/gurdy/alerts`: `path_blocked` (Nav2 goal aborted, from `/navigate_to_pose/_action/status`), `stuck` (commanded > 0.05 m/s but measured < 0.02 for 8 s), `fall` (tilt >= 60 deg for 0.5 s, re-armed below 30), `battery_low` 20 % / `battery_critical` 10 %, `leg_failure`, `object_new` (not for objects already in the first map snapshot) | each alert once per episode; pure logic in `gurdy_core/ops_monitors.py` (`AlertMonitor`, `LegHealth`, `BatteryModel`) |
Launch the three monitors: `ros2 launch gurdy_safety monitors.launch.py odom_topic:=/odom` (`/odometry/filtered` is the default; ground-truth odometry publishes `/odom`).

## Gaps that remain
1. `cmd_vel_gate_node` (a `cmd_vel` mux so Nav2 / teleop cannot drive during e-stop; today only the policy node obeys `/estop`), Nav2 keepout filter for zones.
2. Localisation quality (`where_am_i.localisation` is always `ok`), VLM / photos, persistent memory (SQLite per `map_id`), `mapped_pct` needs `interior_bounds` for the map (None otherwise).
3. Leg-health thresholds and the `stuck` / `fall` thresholds are unvalidated: measure healthy tracking error in Gazebo, then inject a limp / locked leg (the `leg_failure` env of Experiment B has the MuJoCo equivalent).
4. `object_missing` alert (only the mock emits it); the backend's `patrol` report covers missing objects.
Assumptions to check live: Nav2 `compute_path_to_pose` server is up with `navigate_to_pose`; `/imu` orientation is the body orientation; `/scan` bearing 0 = robot forward;
GoalStatus codes 4/5/6 = succeeded/canceled/aborted; semantic JSON `last_seen` is /clock time (design assumes it, `detector_node` stamps from sim time); `/odom` twist is the speed magnitude (frame does not matter).

## Found on the way
Fixed: `MockBackend.return_home` called `go_to`, so a failed leg refused it (the rule exempts `return_home`); it now passes `_tool="return_home"` like RosBackend (tests updated).
Checked live without Gazebo (sim image): the packages build, `RclpyPort` sees the `explore` server (`explore_available() == True`), a goal without Nav2 / map is rejected and surfaces as `aborted`.
Untested: the backend ran only against `FakePort`; the real adapter's Nav2 action callbacks / TF / QoS and the new nodes are unrun against Gazebo.
Tests: `docker compose run --rm brain python3 -m pytest -q brain/tests` (235 pass); monitors `gurdy_core/tests/test_ops_monitors.py` (rl image: `docker run --rm -v $PWD:/w -w /w --entrypoint python3 gurdy2/rl:cpu -m pytest -q gurdy_core/tests/test_ops_monitors.py`);
ROS nodes in the sim image after `colcon build --symlink-install --packages-up-to gurdy_safety gurdy_navigation gurdy_policy gurdy_interfaces` (`pytest ros2_ws/src/gurdy_safety/test ros2_ws/src/gurdy_policy/test`, 29 pass incl. e-stop).

## Gazebo test commands (later, when the GPU is free; one heavy workload at a time)
```bash
cd ~/projects/gurdy3 && docker ps        # must be empty
# 0. unit tests anywhere:  docker compose run --rm brain python3 -m pytest -q brain/tests/test_ros_backend.py
# 1. flat_lab + Nav2 localization on the saved GT map + detector (policy controller), headless. Arguments checked against sim.launch.py / phase4_semantic_run.sh.
docker compose run -d --rm --name gurdy_agent sim bash -c 'cd ~/gurdy2 && source ros2_ws/install/setup.bash && source scripts/lib_nav_start.sh && mkdir -p /tmp/p56 && \
  start_nav_stack /tmp/p56 gui:=false world:=flat_lab perception:=true target_frame:=map nav:=localization map:=flat_lab_tripod_gt navigation:=true \
  controller:=policy odom_source:=ground_truth && sleep infinity'
#    wait ~2 min:  docker exec gurdy_agent cat /tmp/p56/nav_ready.txt   (ends with "ready")
#    flat_lab objects are plain boxes / a cylinder, so the detector reports class `box` (colours red / blue / yellow; the cylinder becomes a box too).
# 1b. new nodes (same container, one more shell): rebuild once:  cd ros2_ws && colcon build --symlink-install && source install/setup.bash
docker exec -d gurdy_agent /entrypoint.sh bash -c 'cd ~/gurdy2 && ros2 run gurdy_navigation frontier_explorer_node --ros-args -p serve_action:=true -p log_path:=/tmp/p56/explore_log.json'
docker exec -d gurdy_agent /entrypoint.sh bash -c 'cd ~/gurdy2 && ros2 launch gurdy_safety monitors.launch.py odom_topic:=/odom'   # /odom with ground_truth odometry
#    checks: ros2 action list | grep explore;  ros2 topic echo /battery --once (location SIMULATED);  ros2 topic echo /gurdy/leg_health --once;
#    low battery:  ros2 param set /battery_sim_node level 19.5   -> /gurdy/alerts battery_low;  ros2 param set /leg_health_node fake_failed_leg rm -> leg_failure + status legs
#    e-stop:  ros2 topic pub --once -r 1 /estop std_msgs/msg/Bool '{data: true}'  (QoS latched: add --qos-durability transient_local --qos-reliability reliable) -> /policy/status stand:estop, legs go to stance, /cmd_vel ignored; data: false releases
# 2. smoke test of the adapter, no LLM (second terminal):
docker exec -it gurdy_agent /entrypoint.sh bash -c 'cd ~/gurdy2 && python3 - <<PY
import time
from brain.ros.rclpy_port import RclpyPort
from brain.ros import RosBackend
p = RclpyPort(map_id="flat_lab"); time.sleep(3)
b = RosBackend(p, "maps/flat_lab.places.yaml")
print("pose", p.pose(), "objects", len(p.semantic_objects() or []), "nav", p.nav_ready())
print(b.where_am_i()); print(b.status()); print(b.path_check(place="room b"))
print(b.walk("forward", 0.3))
for _ in range(100):
    time.sleep(0.3); b.tick()
    if b.task is None: break
print(b.events[-1])
print(b.go_to(place="room b"))
for _ in range(600):
    time.sleep(0.5); b.tick()
    if b.task is None: break
print(b.events[-1])
PY'
# 3. typed commands (keyword planner, or --ollama MODEL with OLLAMA_URL set):
docker exec -it gurdy_agent /entrypoint.sh bash -c 'cd ~/gurdy2 && python3 -m brain.ros.run_text --places maps/flat_lab.places.yaml --map-id flat_lab'
#    try: "walk forward half a metre", "turn left 90 degrees", "where is the red box", "go to room b", "come back", "stop" (mid-walk), "what is nearby"
# 4. stop:  docker rm -f gurdy_agent
```
Pass criteria (first run): pose + semantic objects arrive; `walk 0.3` moves 0.27-0.33 m; `turn left 90` ends within 5 deg; `go_to room b` returns `reached` with
final error < 0.25 m; `stop` mid-goal zeroes `cmd_vel` within 1 s (check `ros2 topic echo /cmd_vel`); `where_is` matches the detector's map. Record RTF and RAM.
Extra criteria: `b.explore(max_time_s=120)` runs, `/explore` feedback arrives, `b.stop()` cancels it (Nav2 goal cancelled, robot stops < 1 s); `b.find({"class":"box","color":"red"})` on an unmapped map explores then stops once the track appears; `b.walk("forward", 1.0)` through a `no_go_zone` is refused; `b.estop(True)` mid-walk: stand pose within ~2 s. Place names in `maps/flat_lab.places.yaml` match flat_lab only.
