"""
TATARO Therapy — Progressive Reach session launch.

Brings up (infrastructure + therapy logic, all in one launch):

  1. RealSense D455       — aligned depth, 1280x720x30 (same profile as the
                             other TATARO CV launch files).
  2. xArm robot driver    — add_gripper=true, add_realsense_d435i=false
                             (camera already up via (1)).
  3. tataro_button_monitor — BLE -> /tataro/button_pressed (std_msgs/Bool).
  4. tataro_therapy_fsm    — the progressive-reach state machine.

Unlike the marker-tracking loop scripts, both (3) and (4) are proper rclpy
nodes with no stdin/input() dependency, so they can be launched directly
here instead of requiring a separate manual terminal.

Bench-testing without the BLE board: keyboard_button_monitor.py publishes
the same /tataro/button_pressed topic, triggered by pressing 'n' instead of
the physical button. It reads raw stdin, so (like the marker-tracking loop
scripts) it CANNOT be launched here -- run it manually in its own terminal
instead of (3):

    ros2 run therapy_tasks tataro_keyboard_button_monitor

Only run one of tataro_button_monitor / tataro_keyboard_button_monitor at a
time -- comment out node (3) below if using the keyboard stand-in.

Usage:
    ros2 launch therapy_tasks tataro_therapy_session.launch.py
    ros2 launch therapy_tasks tataro_therapy_session.launch.py robot_ip:=192.168.1.150
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.1.150',
        description='xArm robot IP address',
    )

    # ── 1. RealSense D455 ────────────────────────────────────────────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
            ])
        ),
        launch_arguments={
            'publish_tf':           'true',
            'enable_rgbd':          'true',
            'enable_sync':          'true',
            'use_system_time':      'true',
            'align_depth.enable':   'true',
            'enable_depth':         'true',
            'pointcloud.enable':    'false',
            'color_module.profile': '1280x720x30',
        }.items(),
    )

    # ── 2. xArm robot driver ──────────────────────────────────────────────────
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'lite6_moveit_realmove.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip':            LaunchConfiguration('robot_ip'),
            'add_gripper':         'true',
            'add_realsense_d435i': 'false',
            'add_d435i_links':     'false',
        }.items(),
    )

    # ── 3. Button state publisher ────────────────────────────────────────────
    button_monitor_node = Node(
        package='therapy_tasks',
        executable='tataro_button_monitor',
        name='tataro_button_monitor',
        output='screen',
    )

    # ── 4. Progressive-reach therapy FSM ─────────────────────────────────────
    therapy_fsm_node = Node(
        package='therapy_tasks',
        executable='tataro_therapy_fsm',
        name='tataro_therapy_fsm',
        output='screen',
    )

    return LaunchDescription([
        robot_ip_arg,
        realsense_launch,
        robot_launch,
        button_monitor_node,
        therapy_fsm_node,
    ])
