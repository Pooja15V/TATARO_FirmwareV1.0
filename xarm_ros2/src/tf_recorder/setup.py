from setuptools import setup
import os
from glob import glob

package_name = "tf_recorder"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pooja",
    maintainer_email="thamizpooja1514@gmail.com",
    description="TF recorder for eye-in-hand hand-eye calibration (xArm Lite6 + RealSense D435i)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tf_recorder_node = tf_recorder.tf_recorder_node:main",
            "tf_pose_capture = tf_recorder.tf_pose_capture:main",
            "traj_replay_ros2 = tf_recorder.traj_replay_ros2:main",
        ],
    },
)
