#!/usr/bin/env python3
"""
tatro_gazebo_calibration.launch.py
────────────────────────────────────────────────────────────────────────────
Full Gazebo + MoveIt2 + ArUco + EasyHandEye2 simulation for TATRO eye-in-hand
calibration. Replicates the physical setup in tatro_workspace.rviz.

Robot config mirrors the real system:
  • xArm Lite6
  • Robotiq gripper  (add_gripper:=true)
  • RealSense D435i on EEF  (add_realsense_d435i:=true add_d435i_links:=true)
  • hw_ns=ufactory

Key differences vs real system:
  • add_d435i_links:=true  — TF tree is complete in simulation (no calibration
    ambiguity; calibration verifies the procedure, not the actual extrinsics)
  • Gazebo provides ground truth camera pose for validation

Before running:
  1.  python3 scripts/generate_aruco_texture.py
  2.  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<pkg_root>/models
  3.  ros2 launch tatro_handeye_sim tatro_gazebo_calibration.launch.py

Usage:
  ros2 launch tatro_handeye_sim tatro_gazebo_calibration.launch.py
  ros2 launch tatro_handeye_sim tatro_gazebo_calibration.launch.py \\
      add_gripper:=true show_rviz:=true
"""

import os
import yaml
from pathlib import Path
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

try:
    from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
    from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file
    _UF_LIB = True
except ImportError:
    _UF_LIB = False


def launch_setup(context, *args, **kwargs):
    add_gripper         = LaunchConfiguration('add_gripper',         default='true')
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default='true')
    hw_ns               = LaunchConfiguration('hw_ns',               default='ufactory')
    no_gui_ctrl         = LaunchConfiguration('no_gui_ctrl',         default='false')
    show_rviz           = LaunchConfiguration('show_rviz',           default='true')
    handeye_name        = LaunchConfiguration('handeye_name',        default='tataro_handeye_3')

    # ── world file path ──────────────────────────────────────────────────
    pkg_root = Path(__file__).resolve().parent.parent
    world_file = str(pkg_root / 'worlds' / 'tatro_calibration.world')

    # ── MoveIt2 config (only if uf_ros_lib is available) ────────────────
    if _UF_LIB:
        ros2_control_params = generate_ros2_control_params_temp_file(
            os.path.join(get_package_share_directory('xarm_controller'),
                         'config', 'lite6_controllers.yaml'),
            prefix='',
            add_gripper=add_gripper.perform(context) in ('True', 'true'),
            add_bio_gripper=False,
            ros_namespace='',
            update_rate=1000,
            use_sim_time=True,
            robot_type='lite',
        )
        moveit_config = MoveItConfigsBuilder(
            context=context,
            controllers_name='fake_controllers',
            dof='6',
            robot_type='lite',
            hw_ns=hw_ns,
            limited='false',
            effort_control='false',
            velocity_control='false',
            ros2_control_plugin='gazebo_ros2_control/GazeboSystem',
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_realsense_d435i=add_realsense_d435i,
            add_d435i_links='true',  # full TF tree in simulation
        ).to_moveit_configs()
        moveit_config_dump = yaml.dump(moveit_config.to_dict())
    else:
        moveit_config_dump = ''

    # ── Gazebo launch ─────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world':          world_file,
            'server_required': 'true',
            'gui_required':    'true',
        }.items(),
    )

    # ── Robot state publisher ─────────────────────────────────────────────
    if _UF_LIB:
        rsp_params = [{'use_sim_time': True},
                      {'robot_description': moveit_config.to_dict()['robot_description']}]
    else:
        rsp_params = [{'use_sim_time': True}]

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=rsp_params,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # ── Spawn robot at TATRO position (matches physical setup) ───────────
    # pos=(-0.2, -0.5, 1.021) yaw=π/2 → link_base at world origin equivalent
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'UF_ROBOT',
            '-x', '-0.2',
            '-y', '-0.5',
            '-z', '1.021',
            '-Y', '1.5708',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ── MoveIt2 common (move_group, rviz, etc.) ───────────────────────────
    robot_moveit_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_moveit_config'),
                                  'launch', '_robot_moveit_common2.launch.py'])
        ),
        launch_arguments={
            'prefix':              '',
            'attach_to':           'world',
            'attach_xyz':          '"0 0 0"',
            'attach_rpy':          '"0 0 0"',
            'no_gui_ctrl':         no_gui_ctrl,
            'show_rviz':           show_rviz,
            'use_sim_time':        'true',
            'moveit_config_dump':  moveit_config_dump,
        }.items(),
    )

    # ── Controller spawners ───────────────────────────────────────────────
    controller_names = [
        'joint_state_broadcaster',
        'lite6_traj_controller',
    ]
    controller_nodes = [
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[name, '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}],
        )
        for name in controller_names
    ]

    # ── ArUco detector ────────────────────────────────────────────────────
    # Subscribes to simulated camera feed from Gazebo camera plugin.
    # For simulation, the camera topic is published by the D435i Gazebo plugin.
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        remappings=[
            ('/image',        '/camera/color/image_raw'),
            ('/camera_info',  '/camera/color/camera_info'),
        ],
        parameters=[{
            'marker_id':   0,
            'marker_size': 0.15,
            'reference_frame':      'camera_color_optical_frame',
            'camera_frame':         'camera_color_optical_frame',
            'marker_frame':         'aruco_marker_frame',
            'image_is_rectified':   True,
        }],
    )

    # ── TATRO workspace publisher ─────────────────────────────────────────
    ws_pkg = FindPackageShare('tatro_workspace_config')
    workspace_publisher = Node(
        package='tatro_workspace_config',
        executable='tatro_workspace_publisher',
        name='tatro_workspace_publisher',
        output='screen',
        parameters=[
            PathJoinSubstitution([ws_pkg, 'config', 'workspace_params.yaml']),
        ],
    )

    # ── EasyHandEye2 calibration node ─────────────────────────────────────
    handeye_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('easy_handeye2'), 'launch', 'calibrate.launch.py'])
        ),
        launch_arguments={
            'name':              handeye_name,
            'calibration_type':  'eye_in_hand',
            'robot_base_frame':  'link_base',
            'robot_effector_frame': 'link_eef',
            'tracking_base_frame':  'camera_color_optical_frame',
            'tracking_marker_frame': 'aruco_marker_frame',
        }.items(),
    )

    # ── Event-ordered startup ─────────────────────────────────────────────
    return [
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=rsp_node,
                on_start=gazebo_launch,
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=rsp_node,
                on_start=spawn_node,
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_node,
                on_exit=controller_nodes,
            )
        ),
        rsp_node,
        robot_moveit_common,
        aruco_node,
        workspace_publisher,
        handeye_launch,
    ]


def generate_launch_description():
    # Set GAZEBO_MODEL_PATH to include our models directory
    pkg_root = Path(__file__).resolve().parent.parent
    models_dir = str(pkg_root / 'models')
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_path = f"{models_dir}:{existing}" if existing else models_dir

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_path),

        DeclareLaunchArgument('add_gripper',         default_value='true'),
        DeclareLaunchArgument('add_realsense_d435i', default_value='true'),
        DeclareLaunchArgument('hw_ns',               default_value='ufactory'),
        DeclareLaunchArgument('no_gui_ctrl',         default_value='false'),
        DeclareLaunchArgument('show_rviz',           default_value='true'),
        DeclareLaunchArgument('handeye_name',        default_value='tataro_handeye_3'),

        OpaqueFunction(function=launch_setup),
    ])
