"""
Launch hand-eye calibration data collection.

Launches in order:
  1. Intel RealSense D435i camera
  2. ChArUco board detector
  3. xArm Lite6 robot (MoveIt + hardware)
  4. TF recorder node (keyboard-driven)

Usage:
    ros2 launch tf_recorder record.launch.py
    ros2 launch tf_recorder record.launch.py robot_ip:=192.168.1.150
    ros2 launch tf_recorder record.launch.py output_csv:=/path/to/output.csv
    ros2 launch tf_recorder record.launch.py charuco_board_frame:=charuco_board_frame
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_csv = os.path.join(
        "/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0"
        "/xarm_ros2/install/tf_recorder/TF_Records",
        f"handeye_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )

    # ========== Arguments ==========
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.1.150',
        description='xArm robot IP address'
    )

    output_arg = DeclareLaunchArgument(
        'output_csv',
        default_value=default_csv,
        description='Absolute path to output CSV file',
    )

    charuco_frame_arg = DeclareLaunchArgument(
        'charuco_board_frame',
        default_value='charuco_board_frame',
        description='TF frame published by charuco_detector and consumed by the recorder',
    )

    # ========== 1. RealSense Camera ==========
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'publish_tf':            'true',
            'enable_rgbd':           'true',
            'enable_sync':           'true',
            'use_system_time':       'true',
            'align_depth.enable':    'true',
            'enable_depth':          'true',
            'pointcloud.enable':     'false',
            'color_module.profile':  '1280x720x30',
        }.items()
    )

    # ========== 2. ChArUco Detector ==========
    charuco_detector = Node(
        package='aruco_ros',
        executable='charuco_detector_node.py',
        name='charuco_detector',
        output='screen',
        parameters=[{
            'squares_x':     5,
            'squares_y':     5,
            'square_length': 0.030,
            'marker_length': 0.024,
            'min_corners':   6,
            'camera_frame':  'camera_color_optical_frame',
            'marker_frame':  LaunchConfiguration('charuco_board_frame'),
        }],
    )
    
    charuco_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='charuco_view',
        output='screen',
        remappings=[
            ('/image', '/charuco/result')
        ],
    )
    # ========== 3. Robot (MoveIt + hardware) ==========
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'lite6_moveit_realmove.launch.py'
            ])
        ),
        launch_arguments={
            'robot_ip':            LaunchConfiguration('robot_ip'),
            'add_gripper':         'true',
            'add_realsense_d435i': 'true',
            'add_d435i_links':     'false',
        }.items()
    )

    # ========== 4. TF Recorder ==========
    # prefix='xterm -e' opens a dedicated terminal window so the keyboard
    # listener gets a real TTY (ros2 launch connects stdin as a pipe, which
    # causes termios.tcgetattr to fail)
    recorder_node = Node(
        package='tf_recorder',
        executable='tf_recorder_node',
        name='tf_recorder',
        output='screen',
        emulate_tty=True,
        prefix='xterm -e',
        arguments=[LaunchConfiguration('output_csv')],
        parameters=[{
            'charuco_board_frame': LaunchConfiguration('charuco_board_frame'),
        }],
    )

    return LaunchDescription([
        robot_ip_arg,
        output_arg,
        charuco_frame_arg,
        realsense_launch,
        charuco_detector,
        charuco_view,
        robot_launch,
        recorder_node,
    ])
