"""
TATARO CV — Table Detection + Marker 40 Touch
===============================================
Launch sequence:

  1. RealSense D435i          — aligned depth, 1280×720 colour
                                (identical config to tatro_runtime.launch.py)

  2. TATARO_CV_Pipeline.py   — live visualisation:
                                camera feed with marker axes, warped top-down
                                table view, prints pos_40_robot each frame

  3. xArm robot driver        — identical to tatro_runtime.launch.py except
                                add_realsense_d435i=false (camera already up)

  4. tataro_marker40_touch.py — starts after 10 s to let camera + robot settle;
                                waits for STABLE_N consistent readings of
                                pos_40_robot, then commands the calibrator pin:
                                  hover  5 cm above Marker 40
                                  descend to touch the marker centre
Usage:
    ros2 launch tataro_packages tataro_cv_table_detection.launch.py
    ros2 launch tataro_packages tataro_cv_table_detection.launch.py robot_ip:=192.168.1.150
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
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

    # ── 1. RealSense D435i — identical to tatro_runtime.launch.py ────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
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

    # ── 2. TATARO_CV_Pipeline.py — live visualisation ─────────────────────────
    pipeline_proc = ExecuteProcess(
        cmd=['python3', os.path.join(_CV_DIR, 'TATARO_CV_PipelineV1.0.py')],
        cwd=_CV_DIR,
        output='screen',
        name='tataro_cv_pipeline',
    )

    # ── 3. xArm robot driver — identical to tatro_runtime.launch.py ──────────
    #       add_realsense_d435i=false because the camera is already launched above
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

    # ── 4. Marker 40 touch — delayed 10 s to let camera + robot settle ────────
    touch_proc = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', os.path.join(_PKG_DIR, 'tataro_marker40_touch.py')],
                cwd=_PKG_DIR,
                output='screen',
                name='tataro_marker40_touch',
            )
        ],
    )

    return LaunchDescription([
        robot_ip_arg,
        realsense_launch,
        pipeline_proc,
        robot_launch,
        touch_proc,
    ])
