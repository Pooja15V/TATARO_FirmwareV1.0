"""
TATRO Runtime Launch — Hand-Eye Calibration with Flexible Board Detection

Launches the complete TATRO system with either ChArUco board or single ArUco marker detection.
Automatically loads the corresponding hand-eye calibration based on marker_type.

Usage:

    # With single ArUco marker (legacy)
    ros2 launch tataro_packages tatro_runtime.launch.py \
      robot_ip:=192.168.1.150 \
      calib_name:=tataro_handeye_3A \
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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
        'calib_name', default_value='tatro_handeye',
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
            'add_realsense_d435i': 'false',
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


    # ========== Single ArUco Marker Detection ==========
    aruco_detector = Node(
        package='aruco_ros',
        executable='aruco_detector_node.py',
        name='aruco_detector',
        namespace='aruco',
        output='screen',
        parameters=[{
            'marker_id': 0,
            'marker_size': 0.15,
            'camera_frame': 'camera_color_optical_frame',
            'marker_frame': 'aruco_marker_frame',
        }],
    )


    # ========== ArUco Detection Visualization ==========
    aruco_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='aruco_view',
        output='screen',
        remappings=[
            ('/image', '/aruco/result')
        ],
    )

    return LaunchDescription([
        robot_ip_arg,
        calib_name_arg,
        realsense_launch,
        robot_launch,
        handeye_publisher,
        aruco_detector,
        aruco_view,
    ])

