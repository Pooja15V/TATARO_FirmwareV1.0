"""
tatro_runtime_calibrationcheck — ArUco 7×7 calibration touch workflow

Launches:
  1. RealSense D435i              1280×720, aligned depth
  2. tataro_table_crop_node       live top-down table view (markers 51/52/53);
                                  visual check only — runs alongside other nodes
  3. tataro_aruco7x7_tf_publisher detects a single DICT_7X7_250 marker
                                  (default ID 40) and publishes its TF
  4. Lite6 robot driver           ros2_control + robot state publisher (TF tree)
  5. tataro_charuco_touch         waits for TFs, moves robot so the 10 cm
                                  calibration pin tip touches the marker origin;
                                  shuts the whole launch down on exit

Use a 7×7 ArUco marker with ID NOT in {51, 52, 53} (those are table markers).
Print one from: https://chev.me/arucogen/  →  DICT_7X7_250, any ID ≥ 54.

Usage:
    ros2 launch tataro_packages tatro_runtime_calibrationcheck.launch.py

Optional overrides:
    robot_ip:=192.168.1.150
    marker_id:=40
    marker_size_cm:=15.0
    calibration_pin_m:=0.10
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ───────────────────────────────────────────────────────

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.1.150',
        description='xArm Lite6 IP address'
    )
    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='40',
        description='DICT_7X7_250 marker ID to touch (not 51/52/53 — those are table markers)'
    )
    marker_size_arg = DeclareLaunchArgument(
        'marker_size_cm', default_value='15.0',
        description='Printed ArUco marker black-pattern side in cm'
    )
    pin_length_arg = DeclareLaunchArgument(
        'calibration_pin_m', default_value='0.10',
        description='Calibration pin length mounted on TCP (metres)'
    )

    # ── 1. RealSense D435i ─────────────────────────────────────────────────────

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            'tf_publish_rate':    '0.0',
            'publish_tf':         'false',   # robot_state_publisher owns camera TF via add_d435i_links
            'enable_sync':        'true',
            'align_depth.enable': 'true',
        }.items(),
    )

    # ── 2. Table crop visual check ────────────────────────────────────────────
    # Subscribes to the running camera, detects table markers 51/52/53, and
    # displays a live top-down perspective view for operator verification.
    # Does NOT publish any TF — visual check only.

    table_crop_node = Node(
        package='tataro_packages',
        executable='tataro_table_crop_node.py',
        name='tataro_table_crop_node',
        output='screen',
    )

    # ── 3. ArUco 7×7 TF publisher ──────────────────────────────────────────────
    # Detects a single DICT_7X7_250 marker and broadcasts
    # TF:  camera_color_optical_frame → calibration_marker_frame

    aruco_tf_node = Node(
        package='tataro_packages',
        executable='tataro_aruco7x7_tf_publisher.py',
        name='tataro_aruco7x7_tf_publisher',
        output='screen',
        parameters=[{
            'marker_id':      LaunchConfiguration('marker_id'),
            'marker_size_cm': LaunchConfiguration('marker_size_cm'),
            'camera_frame':   'camera_color_optical_frame',
            'marker_frame':   'calibration_marker_frame',
            'depth_radius':   10,
        }],
    )

    # ── 4. Lite6 robot driver ─────────────────────────────────────────────────
    # add_d435i_links:=true publishes the fixed joint link_eef → camera_link
    # via robot_state_publisher, connecting the camera TF subtree to the robot
    # tree and making link_base → calibration_marker_frame lookable.

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
            'add_realsense_d435i': 'true',
            'add_d435i_links':     'true',
        }.items(),
    )

    # ── 5. tataro_charuco_touch ────────────────────────────────────────────────
    # Waits for all TFs, computes target, moves robot, then exits.
    # charuco_frame is set to calibration_marker_frame (the aruco7x7 TF).
    # When this node exits the entire launch shuts down.

    touch_node = Node(
        package='tataro_packages',
        executable='tataro_charuco_touch.py',
        name='tataro_charuco_touch',
        output='screen',
        parameters=[{
            'robot_ip':          LaunchConfiguration('robot_ip'),
            'robot_base_frame':  'link_base',
            'charuco_frame':     'calibration_marker_frame',
            'calibration_pin_m': LaunchConfiguration('calibration_pin_m'),
            'speed_mm_s':        50.0,
            'mvacc_mm_s2':       100.0,
            'stable_frames':     10,
            'stable_tol_m':      0.003,
            'tf_wait_timeout':   120.0,
            'min_z_m':           0.05,
        }],
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=touch_node,
            on_exit=[Shutdown()],
        )
    )

    return LaunchDescription([
        robot_ip_arg,
        marker_id_arg,
        marker_size_arg,
        pin_length_arg,
        realsense_launch,
        table_crop_node,
        aruco_tf_node,
        robot_launch,
        touch_node,
        shutdown_on_exit,
    ])
