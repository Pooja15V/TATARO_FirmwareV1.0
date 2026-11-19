from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='582',
        description='Marker ID.'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.34',
        description='Marker size in m.'
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_marker_frame',
        description='Frame in which the marker pose will be referred.'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='',
        description='Reference frame.'
    )

    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='Corner Refinement.',
        choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'],
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame', default_value='camera_rgb_optical_frame',
        description='Camera optical frame.'
    )

    aruco_single_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': LaunchConfiguration('camera_frame'),
        'marker_frame': LaunchConfiguration('marker_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
    }

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        parameters=[aruco_single_params],
        remappings=[
            ('/image', '/camera/camera/color/image_raw'),
            ('/camera_info', '/camera/camera/color/camera_info'),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame_arg)
    ld.add_action(corner_refinement_arg)
    ld.add_action(camera_frame_arg)
    ld.add_action(aruco_single)

    return ld
