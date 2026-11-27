"""Phase 3 checks that need no ROS runtime: footprint, Nav2 params vs robot.yaml, world layout,
launch files parse. Run: colcon test --packages-select gurdy_navigation (or pytest in this dir)."""
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "gurdy_core"))
sys.path.insert(0, str(PKG))

from gurdy_core.rl_command import MIN_ANGULAR_SPEED, MIN_LINEAR_SPEED, lift_to_min_speed  # noqa: E402
from gurdy_core.robot import load_robot  # noqa: E402
from gurdy_navigation import footprint as fp  # noqa: E402

ROBOT_YAML = REPO / "ros2_ws/src/gurdy_description/config/robot.yaml"


@pytest.fixture(scope="module")
def robot():
    return load_robot(ROBOT_YAML)


@pytest.fixture(scope="module")
def nav2():
    return yaml.safe_load((PKG / "config/nav2_params.yaml").read_text())


def _poly_from(text):
    return yaml.safe_load(text)


def test_footprint_contains_feet_and_body(robot):
    poly = np.array(fp.footprint_polygon(robot))
    hull = fp._hull(poly)
    for p in list(fp.foot_points_body(robot)) + [(0.15, 0.09), (-0.15, -0.09)]:
        # point inside convex CCW polygon: cross products all >= 0
        c = [fp.cross2(hull[(i + 1) % len(hull)] - hull[i], p - hull[i]) for i in range(len(hull))]
        assert min(c) >= -1e-9, p
    assert 0.25 < fp.max_extent(poly.tolist()) < 0.45


def test_committed_params_match_robot_yaml(robot, nav2):
    patched = fp.patch_nav2_params(nav2, robot)
    assert patched == nav2, "config/nav2_params.yaml is out of date vs robot.yaml (footprint / limits)"


def test_speed_limits_and_minimum_speed_chain(robot, nav2):
    lim = robot.command_limits
    f = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    assert f["vx_max"] <= lim["vx"] and f["vy_max"] <= lim["vy"] and f["wz_max"] <= lim["wz"]
    assert f["motion_model"] == "Omni"
    sm = nav2["velocity_smoother"]["ros__parameters"]
    # the smoother must NOT zero small commands (start deadlock): MPPI's first step from rest is ax_max * model_dt
    assert list(sm["deadband_velocity"]) == [0.0, 0.0, 0.0]
    first_step = f["ax_max"] * f["model_dt"]
    assert first_step < MIN_LINEAR_SPEED, "premise: MPPI's first step is below the minimum, so it must be lifted"
    assert np.hypot(*lift_to_min_speed([first_step, 0.0, 0.0])[:2]) == pytest.approx(MIN_LINEAR_SPEED)
    assert MIN_LINEAR_SPEED < 0.7 * lim["vx"] and MIN_ANGULAR_SPEED < 0.5 * lim["wz"]   # measured minimum (start check) is 0.6 vx
    assert sm["max_accel"][0] <= 0.5 and sm["max_accel"][2] <= 2.0     # gait node accel limits


def test_min_speed_node_is_in_the_chain(nav2):
    """smoother -> cmd_vel_min_speed_node -> collision_monitor -> /cmd_vel (topic names line up)."""
    src = (PKG / "gurdy_navigation/cmd_vel_min_speed_node.py").read_text()
    assert 'declare_parameter("input_topic", "cmd_vel_smoothed")' in src      # Nav2 smoother output
    assert 'declare_parameter("output_topic", "cmd_vel_min")' in src
    cm = nav2["collision_monitor"]["ros__parameters"]
    assert cm["cmd_vel_in_topic"] == "cmd_vel_min" and cm["cmd_vel_out_topic"] == "cmd_vel"
    assert "cmd_vel_min_speed_node" in (PKG / "setup.py").read_text()
    assert "cmd_vel_min_speed_node" in (PKG / "launch/nav.launch.py").read_text()


def test_goal_tolerances_cover_the_minimum_speed(nav2):
    """The robot cannot move slower than 0.09 m/s / 0.23 rad/s: tolerances must be wider than one control step."""
    gc = nav2["controller_server"]["ros__parameters"]["general_goal_checker"]
    pc = nav2["controller_server"]["ros__parameters"]["progress_checker"]
    step_xy = MIN_LINEAR_SPEED / nav2["controller_server"]["ros__parameters"]["controller_frequency"]
    step_yaw = MIN_ANGULAR_SPEED / nav2["controller_server"]["ros__parameters"]["controller_frequency"]
    assert gc["xy_goal_tolerance"] == pytest.approx(0.12) and gc["yaw_goal_tolerance"] == pytest.approx(0.2)
    assert gc["xy_goal_tolerance"] > 10 * step_xy and gc["yaw_goal_tolerance"] > 8 * step_yaw   # yaw: 0.2 rad = 8.7 steps at 0.23 rad/s (verified by the 5-goal run)
    assert pc["required_movement_radius"] < MIN_LINEAR_SPEED * pc["movement_time_allowance"]   # progress is possible


def test_collision_monitor_sees_the_lidar_plane(nav2):
    m = nav2["collision_monitor"]["ros__parameters"]["scan"]
    assert m["min_height"] < 0.05 < m["max_height"]        # lidar is 0.05 m above base_link


def test_lidar_config_in_robot_yaml(robot):
    ld = robot.sensors["lidar"]
    assert ld["rays"] >= 180 and ld["rate_hz"] == 10 and ld["range"][1] >= 6.0


def test_self_filter_masks_footprint_only(robot):
    poly = fp.footprint_polygon(robot)
    ang = np.linspace(-np.pi, np.pi, 360, endpoint=False)
    ex = fp.beam_exit_radius(poly, ang)
    assert np.all(np.isfinite(ex)) and ex.min() > 0.15            # lidar is inside the footprint
    assert ex.max() <= fp.max_extent(poly) + 1e-6
    # the exit point of every beam lies on the footprint boundary: grown polygon exits later
    assert np.all(fp.beam_exit_radius(poly, ang, margin=0.02) > ex)
    r = np.full(360, 0.2)
    r[0], r[1], r[2] = 0.12, ex[1] + 0.05, np.nan
    out = fp.filter_self_hits(r, ex)
    assert np.isinf(out[0])
    assert out[1] == pytest.approx(ex[1] + 0.05)                   # beyond the footprint: kept
    assert np.isnan(out[2])
    assert np.isinf(fp.filter_self_hits(np.full(360, 0.15), ex)).all()   # everything at range_min is inside


def test_self_filter_is_between_bridge_and_consumers(nav2):
    assert "/scan_raw" in (PKG.parent / "gurdy_gazebo/config/bridge.yaml").read_text()
    launch = (PKG / "launch/nav.launch.py").read_text()
    assert "scan_self_filter_node" in launch
    assert nav2["collision_monitor"]["ros__parameters"]["FootprintApproach"]["enabled"] is True


def _gen():
    spec = importlib.util.spec_from_file_location("gen_world", REPO / "scripts/gen_world_flat_lab.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_world_is_up_to_date_and_navigable(robot):
    g = _gen()
    assert g.OUT.read_text() == g.build(), "run scripts/gen_world_flat_lab.py"
    poly = fp.footprint_polygon(robot)
    width = max(abs(y) for _, y in poly) * 2          # footprint width across the doorway
    gap = g.DOOR_Y[1] - g.DOOR_Y[0] - g.WALL_T * 0     # wall thickness does not narrow a straight gap
    assert gap >= width + 0.3, f"doorway {gap:.2f} m too tight for footprint width {width:.2f} m"
    # spawn (origin) clear of walls and objects by > footprint radius
    r = fp.max_extent(poly)
    for name, kind, xy, size, _ in g.OBJECTS:
        half = max(size[0], size[1]) / 2 if kind == "box" else size[0]
        assert np.hypot(*xy) - half > r + 0.2, name
    # all obstacles taller than the lidar plane (~0.16 m above ground)
    for name, kind, xy, size, _ in g.OBJECTS:
        assert (size[2] if kind == "box" else size[1]) > 0.25, name


def test_odom_transform_helper():
    pytest.importorskip("rclpy")
    from nav_msgs.msg import Odometry
    from gurdy_navigation.odom_tf_node import to_transform
    m = Odometry()
    m.pose.pose.position.x, m.pose.pose.position.y = 1.0, 2.0
    m.pose.pose.orientation.w = 1.0
    t = to_transform(m, "odom", "base_link")
    assert (t.header.frame_id, t.child_frame_id) == ("odom", "base_link")
    assert t.transform.translation.x == 1.0 and t.transform.translation.y == 2.0


def test_launch_files_parse():
    pytest.importorskip("launch")
    pytest.importorskip("ament_index_python")
    import launch
    for path in [PKG / "launch/nav.launch.py", REPO / "ros2_ws/src/gurdy_bringup/launch/sim.launch.py"]:
        spec = importlib.util.spec_from_file_location(path.stem + "_t", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        assert isinstance(mod.generate_launch_description(), launch.LaunchDescription)


# ---------------------------------------------------------------- leg odometry + EKF (Phase 3 step 2)
def test_ekf_config_is_consistent():
    ekf = yaml.safe_load((PKG / "config/ekf.yaml").read_text())["ekf_filter_node"]["ros__parameters"]
    assert ekf["two_d_mode"] and ekf["publish_tf"] and ekf["world_frame"] == "odom" == ekf["odom_frame"]
    assert len(ekf["process_noise_covariance"]) == 225
    assert ekf["odom0"] == "/odom/legs" and ekf["imu0"] == "/imu/ekf"
    names = "x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az".split()
    on = lambda cfg: {n for n, f in zip(names, cfg) if f}                     # noqa: E731
    assert on(ekf["odom0_config"]) == {"vx", "vy", "vyaw"}
    assert on(ekf["imu0_config"]) == {"yaw", "vyaw"}                          # no accelerations (gait vibration)


def test_nav2_odom_topic_patch(nav2, robot):
    p = fp.patch_nav2_params(nav2, robot, odom_topic="/odometry/filtered")
    assert p["bt_navigator"]["ros__parameters"]["odom_topic"] == "/odometry/filtered"
    assert p["velocity_smoother"]["ros__parameters"]["odom_topic"] == "/odometry/filtered"
    # MPPI's current-velocity source: without this the EKF runs stalled (commands stayed at ax_max*dt, 2026-11-16)
    assert p["controller_server"]["ros__parameters"]["odom_topic"] == "/odometry/filtered"
    assert fp.patch_nav2_params(nav2, robot) == fp.patch_nav2_params(nav2, robot, odom_topic=None)


def test_leg_odometry_message_helpers():
    pytest.importorskip("rclpy")
    from sensor_msgs.msg import Imu
    from gurdy_core.leg_odometry import LegOdomEstimate
    from gurdy_navigation.leg_odometry_node import BIG, fill_imu_covariance, odometry_message
    est = LegOdomEstimate(True, np.array([0.1, -0.02]), 0.3, np.diag([1e-4, 2e-4, 3e-4]), ("lf", "rm", "lr"))
    m = odometry_message(Imu().header.stamp, "odom", "base_link", 1.0, 2.0, 0.5, est)
    assert (m.header.frame_id, m.child_frame_id) == ("odom", "base_link")
    assert m.twist.twist.linear.x == pytest.approx(0.1) and m.twist.twist.angular.z == pytest.approx(0.3)
    assert (m.twist.covariance[0], m.twist.covariance[7], m.twist.covariance[35]) == pytest.approx((1e-4, 2e-4, 3e-4))
    assert m.pose.pose.orientation.z == pytest.approx(np.sin(0.25))
    bad = odometry_message(Imu().header.stamp, "odom", "base_link", 0, 0, 0, LegOdomEstimate(False))
    assert bad.twist.covariance[0] == bad.twist.covariance[35] == BIG          # no stance feet -> "do not believe"
    imu = fill_imu_covariance(Imu(), 0.01, 2.5e-5, 4e-4)
    assert imu.angular_velocity_covariance[8] == 2.5e-5 and imu.orientation_covariance[0] == 0.01
    keep = Imu()
    keep.angular_velocity_covariance[0] = 1e-3
    assert fill_imu_covariance(keep, 0.01, 2.5e-5, 4e-4).angular_velocity_covariance[0] == 1e-3


def test_odometry_launch_actions():
    pytest.importorskip("launch_ros")
    pytest.importorskip("ament_index_python")
    spec = importlib.util.spec_from_file_location("nav_launch_t", PKG / "launch/nav.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert len(mod.odometry_actions("ground_truth", "", "/x")) == 1
    assert len(mod.odometry_actions("ekf", "", "/x")) == 2
    with pytest.raises(ValueError):
        mod.odometry_actions("magic", "", "/x")
    assert mod.ODOM_TOPIC["ekf"] == "/odometry/filtered"


# ---------------------------------------------------------------- odom_source:=ekf wiring (static review, step 3 follow-up)
def _launch_nodes(mod, source):
    """(package, executable, parameter dicts) of every Node the odometry actions would start."""
    out = []
    for a in mod.odometry_actions(source, "", "/share/gurdy_navigation"):
        params = []
        for p in (getattr(a, "_Node__parameters", None) or []):
            if isinstance(p, dict):
                params.append({getattr(k, "text", None) or "".join(getattr(s, "text", "") for s in k): v for k, v in p.items()})
        exe = "".join(s.text for s in a._Node__node_executable) if not isinstance(a._Node__node_executable, str) else a._Node__node_executable
        out.append((exe, params))
    return out


def _nav_launch_module():
    pytest.importorskip("launch_ros")
    pytest.importorskip("ament_index_python")
    spec = importlib.util.spec_from_file_location("nav_launch_t2", PKG / "launch/nav.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_ekf_mode_has_exactly_one_odom_to_base_link_publisher():
    mod = _nav_launch_module()
    ekf_nodes = [e for e, _ in _launch_nodes(mod, "ekf")]
    gt_nodes = [e for e, _ in _launch_nodes(mod, "ground_truth")]
    assert "odom_tf_node" not in ekf_nodes and gt_nodes == ["odom_tf_node"]      # GT TF node must not run with the EKF
    assert sorted(ekf_nodes) == ["ekf_node", "leg_odometry_node"]
    cfg = yaml.safe_load((PKG / "config/ekf.yaml").read_text())["ekf_filter_node"]["ros__parameters"]
    assert cfg["publish_tf"] is True                                              # the EKF is that one publisher
    # nothing else publishes odom -> base_link: the Gazebo bridge has no TF topic, the OdometryPublisher's tf stays in gz
    bridge = yaml.safe_load((REPO / "ros2_ws/src/gurdy_gazebo/config/bridge.yaml").read_text())
    assert not any("tf" in b["ros_topic_name"].lower().split("/") for b in bridge)
    # (static check of the whole repo: only odom_tf_node.py and the EKF broadcast odom->base_link)
    for f in (REPO / "ros2_ws/src").rglob("*.py"):
        if f.name not in ("odom_tf_node.py",) and "TransformBroadcaster" in f.read_text() and "test" not in f.parts[-2:-1]:
            pytest.fail(f"{f} broadcasts TF")


def test_ekf_mode_frames_and_clock():
    mod = _nav_launch_module()
    ekf_params = next(p for e, p in _launch_nodes(mod, "ekf") if e == "ekf_node")
    assert {"use_sim_time": True} in ekf_params                                  # EKF runs on the sim clock
    cfg = yaml.safe_load((PKG / "config/ekf.yaml").read_text())["ekf_filter_node"]["ros__parameters"]
    assert (cfg["odom_frame"], cfg["base_link_frame"], cfg["world_frame"]) == ("odom", "base_link", "odom")
    slam = yaml.safe_load((PKG / "config/slam_toolbox.yaml").read_text())["slam_toolbox"]["ros__parameters"]
    assert (slam["odom_frame"], slam["base_frame"], slam["map_frame"]) == ("odom", "base_link", "map")
    # frames of the messages the leg odometry node produces (the EKF reads the twist in child_frame_id = base_link)
    src = (PKG / "gurdy_navigation/leg_odometry_node.py").read_text()
    assert 'declare_parameter("odom_frame", "odom")' in src and 'declare_parameter("base_frame", "base_link")' in src
    # the IMU frame the EKF needs must exist in the URDF as a fixed child of base_link
    urdf = (REPO / "ros2_ws/src/gurdy_description/urdf/gurdy.urdf.xacro").read_text()
    assert '<child link="imu_link"/>' in urdf and urdf.index('<parent link="base_link"/>\n    <child link="imu_link"/>') > 0


def test_python_odometry_nodes_stamp_from_input_messages_not_the_clock():
    """leg_odometry_node / odom_tf_node run without use_sim_time (rclpy /clock costs ~40 % CPU): that is only correct while
    every output stamp is copied from an input message and no code reads the node clock."""
    for name in ("leg_odometry_node.py", "odom_tf_node.py"):
        src = (PKG / "gurdy_navigation" / name).read_text()
        assert "get_clock" not in src and ".now()" not in src, name
    leg = (PKG / "gurdy_navigation/leg_odometry_node.py").read_text()
    assert "odometry_message(msg.header.stamp" in leg                              # /odom/legs stamped by /joint_states
    assert "self.pub_imu.publish(msg)" in leg                                      # /imu/ekf keeps the /imu header
    assert "t.header.stamp = msg.header.stamp" in (PKG / "gurdy_navigation/odom_tf_node.py").read_text()


def test_explorer_registered_and_start_helper_waits_for_action_servers():
    setup = (PKG / "setup.py").read_text()
    assert "frontier_explorer_node = gurdy_navigation.frontier_explorer_node:main" in setup
    wait = (REPO / "scripts/wait_nav_ready.py").read_text()
    for server in ("navigate_to_pose", "compute_path_to_pose", "follow_path"):
        assert server in wait
    lib = (REPO / "scripts/lib_nav_start.sh").read_text()
    assert "wait_nav_ready.py" in lib and "NAV_START_ATTEMPTS" in lib
    for run in ("phase3_nav_run.sh", "phase3_explore_run.sh"):
        assert "start_nav_stack" in (REPO / "scripts" / run).read_text()
