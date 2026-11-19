"""
TATARO CV — Detect and Touch Marker 40 (continuous loop)
==========================================================
Launch sequence (infrastructure only):

  1. RealSense D455       — aligned depth, 1280×720×30
  2. TATARO_CV_PipelineV1.0.py  — Phase 1: lock table (51/52/53) once
                                   Phase 2: track Marker 40, update warped view
  3. xArm robot driver    — add_realsense_d435i=false (camera already up)

NOTE: tataro_marker40_touch_loop.py is NOT launched here because ros2 launch
does not route stdin to subprocesses, so input() / Enter cannot be received.
Run it manually in a separate terminal AFTER this launch is up:

    cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/src/tataro_packages
    python3 tataro_marker40_touch_loop.py

Usage:
    ros2 launch tataro_packages tataro_cv_detect_touch_marker_loop.launch.py
    ros2 launch tataro_packages tataro_cv_detect_touch_marker_loop.launch.py robot_ip:=192.168.1.150
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

# os.path.realpath resolves symlinks so paths are correct under --symlink-install
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

    # ── 2. CV Pipeline V1.0 — visualisation ──────────────────────────────────
    #    Locks table frame once (Phase 1) then tracks Marker 40 (Phase 2).
    #    Displays camera feed + warped top-down table view.
    pipeline_proc = ExecuteProcess(
        cmd=['python3', os.path.join(_CV_DIR, 'TATARO_CV_PipelineV1.0.py')],
        cwd=_CV_DIR,
        output='screen',
        name='tataro_cv_pipeline',
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
        pipeline_proc,
        robot_launch,
    ])
