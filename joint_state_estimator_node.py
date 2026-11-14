"""
joint_state_estimator_node.py

Joint state estimation with the Linear Kalman Filter from the Polisher-State-Estimator
project: one two-state (angle, velocity) filter per revolute joint, fed by the noisy
encoder angles on /joint_states_noisy.

Publishes /joint_states_filtered (angle and velocity). If the true /joint_states are also
available (as in this simulation), it logs how much closer the filtered velocity is to the
truth than a plain finite difference of the noisy angle. Only samples where the arm is
moving are scored.

Parameters
----------
angle_noise_std     float  measurement noise std, rad; must match the sensor (default 0.002)
process_noise_q     float  process noise intensity q, (rad/s^2)^2*s (default 0.1)
report_period_s     float  how often to log the error summary, s; 0 disables (default 10.0)
"""
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .estimation import JointKalmanBank, RmseTracker


def _stamp_s(msg: JointState) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class JointStateEstimatorNode(Node):
    def __init__(self):
        super().__init__("joint_state_estimator")
        self.declare_parameter("angle_noise_std", 0.002)
        self.declare_parameter("process_noise_q", 0.1)
        self.declare_parameter("report_period_s", 10.0)
        sigma = float(self.get_parameter("angle_noise_std").value)
        q = float(self.get_parameter("process_noise_q").value)
        report = float(self.get_parameter("report_period_s").value)

        self.bank = JointKalmanBank(sigma, q)
        self.rmse = RmseTracker()
        self._truth: "OrderedDict[float, dict]" = OrderedDict()
        self._prev_noisy: dict = {}

        self.pub = self.create_publisher(JointState, "joint_states_filtered", 10)
        self.create_subscription(JointState, "joint_states_noisy", self._on_noisy, 10)
        self.create_subscription(JointState, "joint_states", self._on_truth, 50)
        if report > 0.0:
            self.create_timer(report, self._report)
        self.get_logger().info(f"Kalman estimator: sigma = {sigma} rad, q = {q}")

    def _on_truth(self, msg: JointState) -> None:
        vel = list(msg.velocity) if len(msg.velocity) == len(msg.name) else [0.0] * len(msg.name)
        self._truth[_stamp_s(msg)] = {n: (p, v) for n, p, v in zip(msg.name, msg.position, vel)}
        while len(self._truth) > 200:
            self._truth.popitem(last=False)

    def _on_noisy(self, msg: JointState) -> None:
        t = _stamp_s(msg)
        truth = self._truth.get(t)
        out = JointState()
        out.header = msg.header
        # Only score the estimator while the arm is moving (idle samples would flatter it).
        moving = truth is not None and any(
            abs(v) > 1e-3 for n, (p, v) in truth.items() if n.startswith("joint")
        )
        for name, angle in zip(msg.name, msg.position):
            if not name.startswith("joint"):
                continue
            est_angle, est_vel = self.bank.update(name, t, angle)
            out.name.append(name)
            out.position.append(float(est_angle))
            out.velocity.append(float(est_vel))

            prev = self._prev_noisy.get(name)
            self._prev_noisy[name] = (t, angle)
            if moving and truth is not None and name in truth and prev is not None and t > prev[0]:
                raw_vel = (angle - prev[1]) / (t - prev[0])
                true_angle, true_vel = truth[name]
                self.rmse.add(raw_vel, est_vel, true_vel, angle, est_angle, true_angle)
        self.pub.publish(out)

    def _report(self) -> None:
        s = self.rmse.summary()
        if s["n"] == 0:
            return
        self.get_logger().info(
            "velocity RMSE [rad/s]: raw finite difference %.4f -> Kalman %.4f | "
            "angle RMSE [rad]: raw %.5f -> Kalman %.5f  (%d samples)"
            % (s["velocity_rmse_raw"], s["velocity_rmse_filtered"],
               s["angle_rmse_raw"], s["angle_rmse_filtered"], s["n"])
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointStateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
