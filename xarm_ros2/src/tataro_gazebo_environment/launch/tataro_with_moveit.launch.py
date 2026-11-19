#!/usr/bin/env python3
"""
Complete TATARO workstation launch with Gazebo, MoveIt2, and marker spawner.

Launches:
- Gazebo with TATARO workstation environment
- xArm Lite6 robot + gripper + camera
- MoveIt2 planning scene
- Marker spawning and pose estimation node
- RViz visualization
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction,
    SetEnvironmentVariable, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Paths
    tataro_pkg = FindPackageShare('tataro_gazebo_environment')
    xarm_pkg = FindPackageShare('xarm_gazebo')
    xarm_moveit_pkg = FindPackageShare('xarm_moveit_config')
    xarm_desc_pkg = FindPackageShare('xarm_description')

    # Arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.150',
        description='IP address of the physical xArm robot'
    )

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='lite6',
        description='Type of xArm robot'
    )

    add_gripper_arg = DeclareLaunchArgument(
        'add_gripper',
        default_value='true',
        description='Add Gripper Lite to the robot'
    )

    add_d435i_arg = DeclareLaunchArgument(
        'add_d435i',
        default_value='true',
        description='Add RealSense D435i camera'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for robot nodes'
    )

    rviz_config = PathJoinSubstitution([tataro_pkg, 'rviz', 'tataro_workstation.rviz'])

    launch_description = LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                             '/opt/ros/humble/lib'),

        robot_ip_arg,
        robot_type_arg,
        add_gripper_arg,
        add_d435i_arg,
        use_sim_time_arg,
        namespace_arg,

        # Set use_sim_time for all nodes
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # Launch Gazebo with xArm Lite6 (using xarm_gazebo)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([xarm_pkg, 'launch', 'lite6_beside_table_gazebo.launch.py'])
            ),
            launch_arguments=[
                ('add_gripper', LaunchConfiguration('add_gripper')),
                ('add_realsense_d435i', LaunchConfiguration('add_d435i')),
            ]
        ),

        # Launch MoveIt2 with robot_ip
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    xarm_moveit_pkg, 'launch',
                    'lite6_moveit_realmove.launch.py'
                ])
            ),
            launch_arguments=[
                ('robot_ip', LaunchConfiguration('robot_ip')),
            ]
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])

    return launch_description
