"""Integration tests that run the real ROS 2 nodes in one process.

Skipped automatically when ROS 2 (rclpy) is not installed. Run them after sourcing your
ROS 2 environment:   python -m pytest tests/test_ros_nodes.py -q
"""
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from deposition_kinematics_ros2.deposition_node import DepositionNode  # noqa: E402
from deposition_kinematics_ros2.encoder_noise_node import EncoderNoiseNode  # noqa: E402
from deposition_kinematics_ros2.joint_state_estimator_node import JointStateEstimatorNode  # noqa: E402
from deposition_kinematics_ros2.trajectory import build_arm  # noqa: E402


@pytest.fixture(scope="module")
def ros():
    rclpy.init(args=["--ros-args", "-p", "path:=circle", "-p", "dof:=2", "-p", "tool:=fixed",
                     "-p", "layers:=1", "-p", "speed:=120.0", "-p", "rate_hz:=50.0"])
    yield
    rclpy.shutdown()


def _spin(nodes, seconds):
    ex = SingleThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    end = time.time() + seconds
    while time.time() < end:
        ex.spin_once(timeout_sec=0.02)
    for n in nodes:
        ex.remove_node(n)


def test_deposition_node_publishes_consistent_data(ros):
    dep = DepositionNode()
    probe = Node("probe")
    js, pose, state, path = [], [], [], []
    probe.create_subscription(JointState, "joint_states", js.append, 100)
    probe.create_subscription(PoseStamped, "tool_pose", pose.append, 100)
    probe.create_subscription(String, "deposition/state", lambda m: state.append(m.data), 100)
    probe.create_subscription(Path, "toolpath", path.append, 10)
    _spin([dep, probe], 6.0)

    assert len(js) > 100 and len(pose) > 100
    assert js[0].name == ["joint1", "joint2", "z_lift"]
    assert len(js[0].position) == 3 and len(js[0].velocity) == 3
    assert len(path) >= 1 and len(path[0].poses) == len(dep.path.xyz)
    assert "DEPOSITING" in state and "APPROACH" in state

    # The published tool pose must equal forward kinematics of the published joint angles (mm -> m).
    arm = build_arm(2)
    for k in range(0, min(len(js), len(pose)), 17):
        x, y, _ = arm.fk(js[k].position[:2])
        assert pose[k].pose.position.x == pytest.approx(x / 1000.0, abs=2e-3)  # messages are not simultaneous
        assert pose[k].pose.position.y == pytest.approx(y / 1000.0, abs=2e-3)
    dep.destroy_node()
    probe.destroy_node()


def test_noise_and_estimator_pipeline(ros):
    dep = DepositionNode()
    noise = EncoderNoiseNode()
    est = JointStateEstimatorNode()
    probe = Node("probe2")
    filtered = []
    probe.create_subscription(JointState, "joint_states_filtered", filtered.append, 100)
    _spin([dep, noise, est, probe], 6.0)

    assert len(filtered) > 100
    assert filtered[0].name == ["joint1", "joint2"]  # the Z axis is not estimated
    assert len(filtered[0].velocity) == 2
    s = est.rmse.summary()
    assert s["n"] > 100
    # Plumbing check only: the scores exist and are finite. Estimation quality is tested
    # deterministically in test_estimation.py on realistic runs (this run is only a few seconds
    # of fast approach and return moves, which a filter tuned for slow deposition lags on).
    assert np.isfinite(s["velocity_rmse_raw"]) and np.isfinite(s["velocity_rmse_filtered"])
    for n in (dep, noise, est, probe):
        n.destroy_node()
