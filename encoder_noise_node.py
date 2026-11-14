"""
encoder_noise_node.py

Simulates cheap joint encoders: subscribes to the true /joint_states and republishes
the revolute joint angles with Gaussian noise on /joint_states_noisy. The Z axis is
passed through unchanged and velocities are dropped, because an encoder only gives angle.

Parameters
----------
angle_noise_std   float   standard deviation of the angle noise, rad (default 0.002, about 0.11 deg)
seed              int     random seed, for repeatable runs (default 42)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EncoderNoiseNode(Node):
    def __init__(self):
        super().__init__("encoder_noise")
        self.declare_parameter("angle_noise_std", 0.002)
        self.declare_parameter("seed", 42)
        self.sigma = float(self.get_parameter("angle_noise_std").value)
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))
        self.pub = self.create_publisher(JointState, "joint_states_noisy", 10)
        self.sub = self.create_subscription(JointState, "joint_states", self._on_js, 10)

    def _on_js(self, msg: JointState) -> None:
        out = JointState()
        out.header = msg.header  # same stamp, so the estimator can be compared with the truth
        out.name = list(msg.name)
        pos = []
        for name, value in zip(msg.name, msg.position):
            if name.startswith("joint"):
                value = value + float(self.rng.normal(0.0, self.sigma))
            pos.append(float(value))
        out.position = pos
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
