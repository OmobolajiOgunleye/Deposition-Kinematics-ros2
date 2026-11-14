import os
from glob import glob

from setuptools import find_packages, setup

package_name = "deposition_kinematics_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Omobolaji Ogunleye",
    maintainer_email="bola.leyee@gmail.com",
    description="ROS 2 simulation of layer-by-layer metal deposition on a planar arm, with a Kalman joint-state estimator.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "deposition_node = deposition_kinematics_ros2.deposition_node:main",
            "encoder_noise_node = deposition_kinematics_ros2.encoder_noise_node:main",
            "joint_state_estimator_node = deposition_kinematics_ros2.joint_state_estimator_node:main",
        ],
    },
)
