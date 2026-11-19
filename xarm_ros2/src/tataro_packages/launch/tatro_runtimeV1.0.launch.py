"""
TATRO Runtime Launch — Complete System with ChArUco Board Detection

Launches the entire TATRO system:
  - xArm Lite6 robot with MoveIt
  - Intel RealSense D435i camera
  - ChArUco board detector (6x6, 3.3cm squares, 2.7cm markers)
  - EasyHandEye calibration transform publisher

ChArUco board detection provides superior accuracy vs. single markers by using
multiple corners for pose estimation, eliminating planar ambiguity.

Usage:
    ros2 launch tataro_packages tatro_runtimeV1.0.launch.py \
      robot_ip:=192.168.1.150 \
      calib_name:=tataro_handeye_6
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ========== Arguments ==========
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.1.150',
        description='xArm robot IP address'
    )

    calib_name_arg = DeclareLaunchArgument(
        'calib_name', default_value='tatro_handeye_6',
        description='Hand-eye calibration name'
    )

    # ========== RealSense Camera ==========
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'publish_tf': 'false',
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'use_system_time': 'true',
            'align_depth.enable': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'false',
            'color_module.profile': '1280x720x30',
        }.items()
    )

    # ========== Robot MoveIt + Hardware Control ==========
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'lite6_moveit_realmove.launch.py'
            ])
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'add_gripper': 'true',
            'add_realsense_d435i': 'true',
            'add_d435i_links': 'false',
        }.items()
    )

    # ========== Hand-Eye Calibration TF Publisher ==========
    # Publishes TF transform from calibration file in project Calibration folder
    # Uses 'name' parameter which loads from CALIBRATIONS_DIRECTORY configured in easy_handeye2/__init__.py
    handeye_publisher = Node(
        package='easy_handeye2',
        executable='handeye_publisher',
        name='handeye_publisher',
        parameters=[{
            'name': LaunchConfiguration('calib_name')
        }],
        output='screen'
    )


    # ========== ChArUco Board Detection ==========
    # Detects 6x6 ChArUco board with 3.0cm squares and 2.4cm markers
    # Subscribes to: /camera/camera/color/image_raw, /camera/camera/color/camera_info
    # Publishes: /charuco/pose (PoseStamped), /charuco/result (Image), TF (camera → charuco_board_frame)
    charuco_detector = Node(
        package='aruco_ros',
        executable='charuco_detector_node.py',
        name='charuco_detector',
        output='screen',
        parameters=[{
            'squares_x': 5,          # Board columns (5x5 grid)
            'squares_y': 5,          # Board rows
            'square_length': 0.030,  # 3.0cm checker square side (matches your physical board)
            'marker_length': 0.024,  # 2.4cm ArUco marker side (matches your physical board)
            'min_corners': 6,        # Minimum detected corners to accept pose
            'camera_frame': 'camera_color_optical_frame',
            'marker_frame': 'charuco_board_frame',
        }],
    )

    # ========== ChArUco Detection Visualization ==========
    charuco_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='charuco_view',
        output='screen',
        remappings=[
            ('/image', '/charuco/result')
        ],
    )

    return LaunchDescription([
        robot_ip_arg,
        calib_name_arg,
        realsense_launch,
        robot_launch,
        handeye_publisher,
        charuco_detector,
        charuco_view,
    ])
