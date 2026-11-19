from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('squares_x',     default_value='5',
                              description='Board columns'),
        DeclareLaunchArgument('squares_y',     default_value='5',
                              description='Board rows'),
        DeclareLaunchArgument('square_length', default_value='0.030',
                              description='Checker square side in metres'),
        DeclareLaunchArgument('marker_length', default_value='0.024',
                              description='ArUco marker side in metres'),
        DeclareLaunchArgument('min_corners',   default_value='6',
                              description='Minimum ChArUco corners required to accept a pose'),
        DeclareLaunchArgument('camera_frame',  default_value='camera_color_optical_frame',
                              description='Camera optical frame'),
        DeclareLaunchArgument('marker_frame',  default_value='charuco_board_frame',
                              description='Output board TF frame'),

        Node(
            package='aruco_ros',
            executable='charuco_detector_node.py',
            name='charuco_detector',
            parameters=[{
                'squares_x':     LaunchConfiguration('squares_x'),
                'squares_y':     LaunchConfiguration('squares_y'),
                'square_length': LaunchConfiguration('square_length'),
                'marker_length': LaunchConfiguration('marker_length'),
                'min_corners':   LaunchConfiguration('min_corners'),
                'camera_frame':  LaunchConfiguration('camera_frame'),
                'marker_frame':  LaunchConfiguration('marker_frame'),
            }],
        ),
    ])
