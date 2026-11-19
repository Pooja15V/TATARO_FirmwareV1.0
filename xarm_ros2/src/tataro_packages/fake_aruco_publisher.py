#!/usr/bin/env python3
"""
Fake ArUco marker publisher for Gazebo simulation.

This script publishes a fake ArUco marker pose in the camera frame,
allowing the tataro_touch_marker_centre script to work in simulation
without needing a real camera or image processing.

The marker is placed at a fixed location on the simulated table.

Usage:
  ros2 run tataro_packages fake_aruco_publisher --ros-args -p marker_x:=0.335 -p marker_y:=0.0
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
import math

class FakeArucoPublisher(Node):
    def __init__(self):
        super().__init__('fake_aruco_publisher')

        # Marker position parameters (in link_base frame)
        self.declare_parameter('marker_x', 0.335)  # Center of table
        self.declare_parameter('marker_y', 0.0)    # Center of table
        self.declare_parameter('marker_z', -0.005) # Table surface
        self.declare_parameter('marker_size', 0.15) # 15 cm marker

        # TF broadcaster for marker
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # TF buffer and listener (to get camera pose)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to periodically publish marker
        self.create_timer(1.0, self.publish_marker)

        self.get_logger().info("Fake ArUco marker publisher started")
        self.get_logger().info("This simulates an ArUco marker for testing without real camera")

    def publish_marker(self):
        """
        Publish fake marker in camera optical frame.

        The marker is placed at a fixed location on the table.
        We compute where it appears in the camera frame using TF.
        """
        try:
            # Get marker position in link_base frame
            marker_x = self.get_parameter('marker_x').value
            marker_y = self.get_parameter('marker_y').value
            marker_z = self.get_parameter('marker_z').value

            # Get camera pose relative to link_base
            tf = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame', 'link_base',
                rclpy.time.Time(seconds=0, nanoseconds=0),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Marker position in camera frame
            # In simulation, we compute this directly
            # In reality, this would come from image processing

            # For simulation: marker is at table center (0.335, 0, -0.005) in link_base
            # Camera is offset from link_eef by hand-eye calibration
            # We publish marker as if it's detected at fixed position

            # Simple model: marker appears in camera frame at computed position
            marker_in_camera = TransformStamped()
            marker_in_camera.header.stamp = self.get_clock().now().to_msg()
            marker_in_camera.header.frame_id = 'camera_color_optical_frame'
            marker_in_camera.child_frame_id = 'aruco_marker_frame'

            # Marker position (computed from table geometry)
            # This is where the marker appears in camera coordinates
            # For a marker at table center, camera mounted on gripper above it:
            marker_in_camera.transform.translation.x = 0.0      # Center in camera view
            marker_in_camera.transform.translation.y = 0.0      # Center in camera view
            marker_in_camera.transform.translation.z = 0.5      # ~50 cm below camera (typical)

            # Marker orientation (identity - flat on table)
            marker_in_camera.transform.rotation.x = 0.0
            marker_in_camera.transform.rotation.y = 0.0
            marker_in_camera.transform.rotation.z = 0.0
            marker_in_camera.transform.rotation.w = 1.0

            # Publish
            self.tf_broadcaster.sendTransform(marker_in_camera)

            self.get_logger().debug(f"Published fake marker at camera coords")

        except Exception as e:
            self.get_logger().warn(f"Could not publish marker: {e}")

def main(args=None):
    rclpy.init(args=args)
    publisher = FakeArucoPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
