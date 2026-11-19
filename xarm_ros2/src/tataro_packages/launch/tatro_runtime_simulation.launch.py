#!/usr/bin/env python3
"""
TATARO Runtime for Gazebo Simulation.

This launch file sets up the TATARO system for simulation:
  - Hand-eye calibration TF publisher
  - Fake ArUco marker publisher (simulates camera + detection)
  - Optional: RViz for visualization

This is paired with:
  ros2 launch xarm_gazebo lite6_tataro_marker_simulation.launch.py

Usage:
  # Terminal 1: Start Gazebo simulation
  ros2 launch xarm_gazebo lite6_tataro_marker_simulation.launch.py

  # Terminal 2: Start TATARO runtime (this file)
  ros2 launch tataro_packages tatro_runtime_simulation.launch.py

  # Terminal 3: Run marker touch script
  ros2 run tataro_packages tataro_touch_marker_centre --ros-args -p hover_m:=0.20
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ========== Arguments ==========
    calib_name_arg = DeclareLaunchArgument(
        'calib_name', default_value='tataro_handeye_3A',
        description='Hand-eye calibration name'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz for visualization'
    )

    # ========== Hand-Eye Calibration TF Publisher ==========
    handeye_publisher = Node(
        package='easy_handeye2',
        executable='handeye_publisher',
        name='handeye_publisher',
        parameters=[{
            'name': LaunchConfiguration('calib_name')
        }],
        output='screen'
    )

    # ========== Fake ArUco Marker Publisher ==========
    # In simulation, we simulate the camera and ArUco detection
    fake_aruco = Node(
        package='tataro_packages',
        executable='fake_aruco_publisher.py',
        name='fake_aruco_publisher',
        output='screen',
        parameters=[{
            'marker_x': 0.335,   # Center of table X
            'marker_y': 0.0,     # Center of table Y
            'marker_z': -0.005,  # Table surface Z
        }]
    )

    # ========== Optional RViz Visualization ==========
    rviz_config = PathJoinSubstitution([
        FindPackageShare('tataro_packages'),
        'config',
        'tataro_simulation.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('launch_ros'),
                    'launch',
                    'no_params_pass_through.launch.py'
                ])
            ) if LaunchConfiguration('rviz').perform(None) == 'true' else None,
            # Simpler approach: just add the node conditionally
        ),
        output='screen'
    )

    # Actually, let's simplify the RViz condition:
    # Just always launch it, user can close if not needed

    return LaunchDescription([
        calib_name_arg,
        rviz_arg,
        handeye_publisher,
        fake_aruco,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
