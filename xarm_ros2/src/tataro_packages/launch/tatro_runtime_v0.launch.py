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
        'calib_name', default_value='tataro_handeye_1A',
        description='Hand-eye calibration name (without .calib extension)'
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

    # ========== Hand-Eye Calibration TF (Static Transform) ==========
    # Publishing calibration as static transform from gripper to camera
    # Values loaded from Calibration/{calib_name}.calib file
    # Currently configured for tataro_handeye_1A
    handeye_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='handeye_publisher',
        arguments=[
            '--x', '0.06777397621604367',
            '--y', '-0.13029085143690755',
            '--z', '0.4248356374139688',
            '--qx', '0.1433242707589868',
            '--qy', '-0.001467472337038478',
            '--qz', '0.49003519034199805',
            '--qw', '0.8598380732224099',
            '--frame-id', 'uflite_gripper_link',
            '--child-frame-id', 'camera_color_optical_frame'
        ],
        output='screen'
    )

    # ========== ArUco Marker Detection ==========
    # Uses custom Python node that publishes TF transforms
    # Publishes: camera_color_optical_frame → aruco_marker_frame
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
        }]
    )

    # ========== ArUco Detection Visualization ==========
    aruco_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='aruco_view',
        output='screen',
        remappings=[
            ('/image', '/aruco/result')
        ]
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
