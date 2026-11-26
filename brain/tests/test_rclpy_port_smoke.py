"""Smoke test: really construct RclpyPort (rclpy, subscriptions, publishers, timer) and shut it down. Skipped where rclpy is not installed (the brain image);
runs in the ROS image:  docker compose run --rm sim bash -c 'cd ~/gurdy2 && source /opt/ros/jazzy/setup.bash && python3 -m pytest -q brain/tests/test_rclpy_port_smoke.py'
The FakePort tests cannot catch a broken constructor (a shadowed `Clock` name crashed the first live start of 456b073)."""
import time

import pytest

rclpy = pytest.importorskip("rclpy")


@pytest.fixture()
def port():
    from brain.ros.rclpy_port import RclpyPort
    p = RclpyPort(map_id="smoke", node_name="gurdy_port_smoke", spin=True)
    yield p
    p.node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_rclpy_port_constructs_with_its_subscriptions_and_publishers(port):
    n = port.node
    subs = {t for t, _ in n.get_topic_names_and_types()}
    for topic in ("/clock", "/imu", "/map", "/scan", "/gurdy/alerts", "/policy/status", "/estop", "/cmd_vel_smoothed"):
        assert any(i.node_name == "gurdy_port_smoke" for i in n.get_subscriptions_info_by_topic(topic)) or \
            any(i.node_name == "gurdy_port_smoke" for i in n.get_publishers_info_by_topic(topic)), topic
    assert port.pose() is None and port.now() >= 0.0 and port.estop_active() is False and port.battery_pct() is None


def test_clock_message_and_zero_hold_work_on_a_real_node(port):
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock as ClockMsg
    got = []
    helper = rclpy.create_node("gurdy_port_smoke_helper")
    helper.create_subscription(Twist, "cmd_vel_smoothed", got.append, 10)
    clk = helper.create_publisher(ClockMsg, "/clock", 10)
    t_end = time.time() + 3.0
    m = ClockMsg()
    m.clock.sec = 42
    while time.time() < t_end and abs(port.now() - 42.0) > 1e-6:
        clk.publish(m)
        rclpy.spin_once(helper, timeout_sec=0.05)
    assert port.now() == pytest.approx(42.0)
    port.hold_zero(0.5)                                    # the 20 Hz wall timer must publish zeros for ~0.5 s
    t_end = time.time() + 1.5
    while time.time() < t_end:
        rclpy.spin_once(helper, timeout_sec=0.05)
    helper.destroy_node()
    assert len(got) >= 5 and all(g.linear.x == 0.0 and g.angular.z == 0.0 for g in got)


def test_compute_path_retries_once_after_clearing_the_global_costmap(port):
    calls = []
    port._plan.server_is_ready = lambda: True
    port._compute_path_once = lambda x, y, start=None: calls.append("plan") or (None if calls.count("plan") == 1 else {"length_m": 1.0, "points": []})
    port._clear_global_costmap = lambda: calls.append("clear") or True
    assert port.compute_path(1.0, 2.0) == {"length_m": 1.0, "points": []}
    assert calls == ["plan", "clear", "plan"]
    calls.clear()
    port._clear_global_costmap = lambda: calls.append("clear") or False      # service not there: no second plan
    port._compute_path_once = lambda x, y, start=None: calls.append("plan") or None
    assert port.compute_path(1.0, 2.0) is None and calls == ["plan", "clear"]
