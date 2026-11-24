"""Node-level tests without Gazebo: the PolicyNode logic with fake messages (needs rclpy + onnxruntime)."""
import os

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("onnxruntime")
from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import Imu, JointState  # noqa: E402

from gurdy_core.robot import load_robot  # noqa: E402

REPO = os.environ.get("GURDY_REPO", os.path.expanduser("~/gurdy2"))
MODEL = os.path.join(REPO, "models", "dr_energy_ft_6M.onnx")
pytestmark = pytest.mark.skipif(not os.path.exists(MODEL), reason="models/ not exported")


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    from gurdy_policy.policy_node import PolicyNode
    n = PolicyNode()
    n.published = []
    n.pub = type("P", (), {"publish": lambda _s, m: n.published.append(np.array(m.data))})()
    yield n
    n.destroy_node()


def stamp(t):
    return Time(sec=int(t), nanosec=int(round((t % 1) * 1e9)))


def feed(n, t, cmd=None, q=None):
    robot = n.robot
    js = JointState()
    js.header.stamp = stamp(t)
    js.name = list(robot.joint_names)
    js.position = (n.q_stance if q is None else q).tolist()
    js.velocity = [0.0] * 18
    imu = Imu()
    imu.header.stamp = stamp(t)
    imu.orientation.w = 1.0
    n.on_imu(imu)
    if cmd is not None:
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = cmd
        n.on_cmd(m)
    n.on_joint_states(js)


def run(n, t0, secs, cmd, hz=100):
    t = t0
    while t < t0 + secs:
        feed(n, t, cmd)
        t += 1.0 / hz
    return t


def test_policy_runs_at_50hz_and_moves_legs(node):
    t = run(node, 0.0, 2.0, (0.08, 0.0, 0.0))
    assert node.status == "policy"
    assert 95 <= len(node.published) <= 101                        # 50 Hz over 2 s
    assert np.ptp(np.array(node.published), axis=0).max() > 0.05   # joints actually move
    assert np.all(np.isfinite(node.published))


def test_cmd_timeout_goes_to_stand_pose_smoothly(node):
    t = run(node, 0.0, 2.0, (0.08, 0.0, 0.0))
    last_walk = node.published[-1].copy()
    for _ in range(300):                                           # no more /cmd_vel; sensors keep coming
        feed(node, t)
        t += 0.01
    assert node.status == "stand:cmd_vel timeout"
    assert np.max(np.abs(node.published[-1] - node.q_stance)) < 1e-9
    steps = np.abs(np.diff(np.array(node.published[-100:]), axis=0))
    assert steps.max() <= node.get_parameter("stand_speed").value * node.dt + 1e-9   # rate-limited ramp
    assert np.max(np.abs(last_walk - node.q_stance)) > 0.02        # it was really away from stance


def test_imu_timeout_goes_to_stand(node):
    t = run(node, 0.0, 1.0, (0.08, 0.0, 0.0))
    for _ in range(100):                                           # joint_states + cmd keep coming, IMU silent
        m = Twist()
        m.linear.x = 0.08
        node.on_cmd(m)
        js = JointState()
        js.header.stamp = stamp(t)
        js.name = list(node.robot.joint_names)
        js.position = node.q_stance.tolist()
        js.velocity = [0.0] * 18
        node.on_joint_states(js)
        t += 0.01
    assert node.status == "stand:imu timeout"


def test_resume_after_stand_restarts_from_rest_state(node):
    t = run(node, 0.0, 1.0, (0.08, 0.0, 0.0))
    for _ in range(100):
        feed(node, t)
        t += 0.01
    assert node.ctl.steps == 0
    t = run(node, t, 0.5, (0.08, 0.0, 0.0))
    assert node.status == "policy" and node.ctl.steps > 0


def test_mode_switch_to_tripod_and_back(node):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter("mode", value="tripod")])
    t = run(node, 0.0, 2.0, (0.08, 0.0, 0.0))
    assert node.status == "tripod" and np.ptp(np.array(node.published), axis=0).max() > 0.05
    res = node.set_parameters([Parameter("mode", value="bogus")])[0]
    assert not res.successful and node.mode == "tripod"
    node.set_parameters([Parameter("mode", value="policy")])
    run(node, t, 1.0, (0.08, 0.0, 0.0))
    assert node.status == "policy"


def _estop(node, active):
    from std_msgs.msg import Bool
    node.on_estop(Bool(data=active))


@pytest.mark.parametrize("mode", ["policy", "tripod"])
def test_estop_goes_to_stand_ignores_cmd_and_releases(node, mode):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter("mode", value=mode)])
    t = run(node, 0.0, 2.0, (0.08, 0.0, 0.0))
    assert node.status == mode
    _estop(node, True)
    t = run(node, t, 3.0, (0.08, 0.0, 0.0))               # /cmd_vel keeps coming: must be ignored
    assert node.status == "stand:estop"
    assert np.max(np.abs(node.published[-1] - node.q_stance)) < 1e-9
    steps = np.abs(np.diff(np.array(node.published[-100:]), axis=0))
    assert steps.max() <= node.get_parameter("stand_speed").value * node.dt + 1e-9      # rate-limited, no jump
    _estop(node, False)
    t = run(node, t, 0.3, None)
    assert node.status.startswith("stand") if mode == "policy" else node.status == "tripod"   # policy: stands until a fresh /cmd_vel; tripod: zero command
    run(node, t, 1.0, (0.08, 0.0, 0.0))
    assert node.status == mode


def test_exact_zero_command_stands_at_once_and_a_new_command_walks_again(node):
    t = run(node, 0.0, 2.0, (0.1, 0.0, 0.0))
    assert node.status == "policy"
    t = run(node, t, 0.3, (0.0, 0.0, 0.0))                         # the agent's "stop": zeros, well inside cmd_timeout (0.5 s)
    assert node.status == "stand:stop command"
    t = run(node, t, 0.2, (0.09, 0.0, 0.0))                        # the next command (Nav2, min-speed lifted) starts from rest
    assert node.status == "policy"


def test_single_zero_glitch_does_not_stop_the_walk(node):
    t = run(node, 0.0, 2.0, (0.1, 0.0, 0.0))
    feed(node, t, (0.0, 0.0, 0.0))
    t = run(node, t + 0.01, 0.3, (0.1, 0.0, 0.0))
    assert node.status == "policy"
