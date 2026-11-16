"""
deposition_node.py

ROS 2 node that plays a layer-by-layer deposition toolpath (WAAM / DED style) on a
planar 2R or 3R arm with a vertical Z axis. The inverse kinematics come from the
Deposition-Kinematics toolkit; this node adds the ROS interfaces.


"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker

from .trajectory import build_arm, build_path, build_trajectory

MM = 1e-3  # mm -> m


class DepositionNode(Node):
    def __init__(self):
        super().__init__("deposition_node")
        self.declare_parameter("path", "circle")
        self.declare_parameter("dof", 3)
        self.declare_parameter("tool", "tangent")
        self.declare_parameter("fixed_phi", 0.0)
        self.declare_parameter("speed", 25.0)
        self.declare_parameter("layers", 0)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("frame_id", "world")

        p = self.get_parameter
        self.kind = p("path").value
        self.dof = int(p("dof").value)
        self.tool = p("tool").value
        self.rate_hz = float(p("rate_hz").value)
        self.loop = bool(p("loop").value)
        self.frame_id = p("frame_id").value
        if self.tool not in ("fixed", "tangent"):
            raise ValueError("parameter 'tool' must be 'fixed' or 'tangent'")

        self.arm = build_arm(self.dof)
        self.path = build_path(self.kind, int(p("layers").value))
        self.traj = build_trajectory(
            self.arm, self.path, speed=float(p("speed").value), rate_hz=self.rate_hz,
            tool_mode=self.tool, fixed_phi=float(p("fixed_phi").value),
        )
        self.joint_names = [f"joint{i + 1}" for i in range(self.dof)] + ["z_lift"]

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "tool_pose", 10)
        self.pub_path = self.create_publisher(Path, "toolpath", 1)
        self.pub_bead = self.create_publisher(Marker, "deposited_bead", 10)
        self.pub_state = self.create_publisher(String, "deposition/state", 10)
        self.pub_progress = self.create_publisher(Float32, "deposition/progress", 10)

        self.path_msg = self._make_path_msg()
        self._t_start = None
        self._bead_pts: list = []
        self._bead_layer = -1
        self._last_state = ""
        self._finished = False

        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.create_timer(0.1, self._publish_bead)
        self.create_timer(1.0, self._publish_path)

        self.get_logger().info(
            f"{self.kind} toolpath, {self.dof}R arm, {self.path.n_layers} layers, "
            f"{self.traj.duration:.0f} s trajectory at {p('speed').value} mm/s"
        )

    # ---------------------------------------------------------------- messages
    def _make_path_msg(self) -> Path:
        msg = Path()
        msg.header.frame_id = self.frame_id
        for x, y, z in self.path.xyz:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = float(x) * MM
            ps.pose.position.y = float(y) * MM
            ps.pose.position.z = float(z) * MM
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _publish_path(self) -> None:
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(self.path_msg)

    @staticmethod
    def _state_name(depositing: bool, t: float, traj, k: int, finished: bool) -> str:
        if finished:
            return "DONE"
        if depositing:
            return "DEPOSITING"
        if t < traj.t[np.argmax(traj.depositing)]:
            return "APPROACH"
        if t > traj.t[len(traj.depositing) - 1 - np.argmax(traj.depositing[::-1])]:
            return "RETURN_HOME"
        return "LAYER_CHANGE"

    # ------------------------------------------------------------------- timer
    def _tick(self) -> None:
        now = self.get_clock().now()
        if self._t_start is None:
            self._t_start = now
        t = (now - self._t_start).nanoseconds * 1e-9
        if t >= self.traj.duration:
            if self.loop:
                self._t_start = now
                self._clear_bead()
                t = 0.0
            else:
                self._finished = True
                t = self.traj.duration

        q, qd, z, zd, depositing, layer = self.traj.at(t)
        stamp = now.to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = self.joint_names
        js.position = [float(v) for v in q] + [z * MM]
        js.velocity = [float(v) for v in qd] + [zd * MM]
        self.pub_js.publish(js)

        x, y, phi = self.arm.fk(q)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(x) * MM
        pose.pose.position.y = float(y) * MM
        pose.pose.position.z = z * MM
        pose.pose.orientation.z = math.sin(phi / 2.0)
        pose.pose.orientation.w = math.cos(phi / 2.0)
        self.pub_pose.publish(pose)

        if depositing:
            if layer != self._bead_layer:
                self._bead_layer = layer
                self._bead_pts = []
            self._bead_pts.append((float(x) * MM, float(y) * MM, z * MM))

        state = self._state_name(depositing, t, self.traj, layer, self._finished)
        if state != self._last_state:
            self._last_state = state
            suffix = f" (layer {layer + 1}/{self.traj.n_layers})" if state in ("DEPOSITING", "LAYER_CHANGE") else ""
            self.get_logger().info(state + suffix)
        self.pub_state.publish(String(data=state))
        self.pub_progress.publish(Float32(data=float(t / self.traj.duration)))

    # ------------------------------------------------------------------ markers
    def _publish_bead(self) -> None:
        if len(self._bead_pts) < 2:
            return
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "bead"
        m.id = max(self._bead_layer, 0)
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.003
        frac = self._bead_layer / max(self.traj.n_layers - 1, 1)
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.35 + 0.5 * frac, 0.05 + 0.2 * frac, 1.0
        from geometry_msgs.msg import Point

        m.points = [Point(x=x, y=y, z=z) for x, y, z in self._bead_pts]
        self.pub_bead.publish(m)

    def _clear_bead(self) -> None:
        self._bead_pts = []
        self._bead_layer = -1
        m = Marker()
        m.header.frame_id = self.frame_id
        m.action = Marker.DELETEALL
        self.pub_bead.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = DepositionNode()
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
