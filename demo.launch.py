import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "deposition_kinematics_ros2"


def _is_true(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _setup(context):
    cfg = {k: LaunchConfiguration(k).perform(context) for k in (
        "path", "dof", "tool", "speed", "layers", "loop", "rviz", "estimator", "angle_noise_std", "process_noise_q")}
    share = get_package_share_directory(PKG)
    with open(os.path.join(share, "urdf", f"planar_{int(cfg['dof'])}r.urdf")) as f:
        robot_description = f.read()

    nodes = [
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}], output="screen",
        ),
        Node(
            package=PKG, executable="deposition_node", output="screen",
            parameters=[{
                "path": cfg["path"], "dof": int(cfg["dof"]), "tool": cfg["tool"],
                "speed": float(cfg["speed"]), "layers": int(cfg["layers"]), "loop": _is_true(cfg["loop"]),
            }],
        ),
    ]
    if _is_true(cfg["estimator"]):
        sigma = float(cfg["angle_noise_std"])
        nodes += [
            Node(package=PKG, executable="encoder_noise_node", output="screen",
                 parameters=[{"angle_noise_std": sigma}]),
            Node(package=PKG, executable="joint_state_estimator_node", output="screen",
                 parameters=[{"angle_noise_std": sigma, "process_noise_q": float(cfg["process_noise_q"])}]),
        ]
    if _is_true(cfg["rviz"]):
        nodes.append(Node(
            package="rviz2", executable="rviz2", output="log",
            arguments=["-d", os.path.join(share, "rviz", "deposition.rviz")],
        ))
    return nodes


def generate_launch_description():
    def arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    return LaunchDescription([
        arg("path", "circle", "toolpath: circle | spiral"),
        arg("dof", "3", "arm joints: 2 | 3"),
        arg("tool", "tangent", "3R tool direction: fixed | tangent"),
        arg("speed", "25.0", "deposition travel speed, mm/s"),
        arg("layers", "0", "number of layers (0 = project default)"),
        arg("loop", "false", "restart after returning home"),
        arg("rviz", "true", "open RViz"),
        arg("estimator", "true", "run the noisy-encoder simulator and the Kalman joint-state estimator"),
        arg("angle_noise_std", "0.002", "simulated encoder noise, rad"),
        arg("process_noise_q", "0.1", "Kalman process noise q"),
        OpaqueFunction(function=_setup),
    ])
