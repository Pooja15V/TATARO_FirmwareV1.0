"""
TATARO CV — Detect cube object (continuous loop)
=================================================
Launch sequence:

  1. RealSense D455       — aligned depth, 1280×720×30
  2. track_cube.py        — Phase 1: lock table frame (markers 51/52/53)
                            Phase 2: track 3.5 cm cube (DICT_4X4, ID=1)
                            Displays camera feed + warped top-down table view.
                            Prints cube centre XYZ (cm) + RPY (deg) to console.
  3. xArm robot driver    — add_realsense_d435i=false (camera already up)

Usage:
    ros2 launch tataro_packages tataro_cv_detect_object_loop.launch.py
    ros2 launch tataro_packages tataro_cv_detect_object_loop.launch.py robot_ip:=192.168.1.150

Build before launch:
    cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
    colcon build --packages-select tataro_packages --symlink-install
    source install/setup.bash
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# os.path.realpath resolves symlinks — required under --symlink-install
_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
_PKG_DIR    = os.path.normpath(os.path.join(_LAUNCH_DIR, '..'))
_CV_DIR     = os.path.normpath(os.path.join(_LAUNCH_DIR, '..', '..', 'computer_vision'))


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

    # ── 2. Cube tracker — locks table frame then continuously tracks the cube ─
    #    DICT_4X4 ID=1, cube 3.5 cm, marker 3.0 cm.
    #    Outputs pose to console and shows two OpenCV windows:
    #      "TATARO — Camera Feed"  and  "Warped Table (top-down)"
    cube_tracker_proc = ExecuteProcess(
        cmd=['python3', os.path.join(_CV_DIR, 'track_cube.py')],
        cwd=_CV_DIR,
        output='screen',
        name='tataro_cube_tracker',
    )

    # ── 3. xArm robot driver ─────────────────────────────────────────────────
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

    return LaunchDescription([
        robot_ip_arg,
        realsense_launch,
        cube_tracker_proc,
        robot_launch,
    ])
