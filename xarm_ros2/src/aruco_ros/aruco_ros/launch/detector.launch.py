from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='0',
        description='ArUco marker ID to detect'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.15',
        description='Marker size in meters'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame', default_value='camera_color_optical_frame',
        description='Camera optical frame'
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_marker_frame',
        description='Output marker frame'
    )

    detector_node = Node(
        package='aruco_ros',
        executable='aruco_detector_node.py',
        parameters=[
            {'marker_id': LaunchConfiguration('marker_id')},
            {'marker_size': LaunchConfiguration('marker_size')},
            {'camera_frame': LaunchConfiguration('camera_frame')},
            {'marker_frame': LaunchConfiguration('marker_frame')},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(camera_frame_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(detector_node)

    return ld
