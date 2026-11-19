#!/usr/bin/env python3
"""
Gazebo simulation for TATARO marker touching system.

Launches xArm Lite6 with gripper in Gazebo with table.world.

Launch:
  ros2 launch xarm_gazebo lite6_tataro_marker_simulation.launch.py

Then in another terminal:
  ros2 launch tataro_packages tatro_runtime_simulation.launch.py
  ros2 run tataro_packages tataro_touch_marker_centre --ros-args -p hover_m:=0.20
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    # Use the standard lite6_beside_table_gazebo.launch.py
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/lite6_beside_table_gazebo.launch.py']),
        launch_arguments={
            'add_gripper': 'true',
            'add_realsense_d435i': 'true',
        }.items(),
    )

    return LaunchDescription([
        robot_gazebo_launch,
    ])
