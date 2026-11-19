#!/usr/bin/env python3
"""
Gazebo launcher for TATARO rehabilitation workstation.

Launches Gazebo with:
- TATARO workstation SDF world (table, mounting plate, marker)
- xArm Lite6 robot with gripper
- RealSense D435i camera
- Complete TF tree
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Paths
    tataro_pkg = FindPackageShare('tataro_gazebo_environment')
    xarm_pkg = FindPackageShare('xarm_gazebo')
    xarm_desc_pkg = FindPackageShare('xarm_description')

    world_file = PathJoinSubstitution([tataro_pkg, 'worlds', 'tataro_workstation.sdf'])

    # Arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.150',
        description='IP address of the physical xArm robot'
    )

    add_gripper_arg = DeclareLaunchArgument(
        'add_gripper',
        default_value='true',
        description='Add Gripper Lite to the robot'
    )

    add_d435i_arg = DeclareLaunchArgument(
        'add_d435i',
        default_value='true',
        description='Add RealSense D435i camera to the robot'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time'
    )

    launch_description = LaunchDescription([
        # Set simulation time
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                             '/opt/ros/humble/lib'),

        robot_ip_arg,
        add_gripper_arg,
        add_d435i_arg,
        use_sim_time_arg,

        # Include xArm Gazebo launch - using lite6_beside_table_gazebo.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([xarm_pkg, 'launch', 'lite6_beside_table_gazebo.launch.py'])
            ),
            launch_arguments=[
                ('add_gripper', LaunchConfiguration('add_gripper')),
                ('add_realsense_d435i', LaunchConfiguration('add_d435i')),
            ]
        ),
    ])

    return launch_description
