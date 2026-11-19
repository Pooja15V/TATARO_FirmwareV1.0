#!/usr/bin/env python3
"""
Add CalibrationPin mesh to RViz display
Run this after launching RViz to automatically add the mesh display
"""

import time
import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA


def publish_calibration_pin_marker():
    """Publish calibration pin as a marker"""
    rclpy.init()
    node = rclpy.create_node('calibration_pin_marker_publisher')

    # Create publisher
    marker_pub = node.create_publisher(Marker, '/calibration_pin_marker', 10)

    # Create marker
    marker = Marker()
    marker.header.frame_id = "link_tcp"
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = "calibration_pin"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD

    # Position (offset from link_tcp)
    marker.pose.position = Point(x=0.0, y=0.0, z=0.015)
    marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # Scale
    marker.scale.x = 0.001
    marker.scale.y = 0.001
    marker.scale.z = 0.001

    # Color - Light Blue
    marker.color = ColorRGBA(r=0.2, g=0.6, b=0.8, a=1.0)

    # Mesh resource path (use absolute path for reliability)
    marker.mesh_resource = "file:///home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/xarm_description/meshes/CalibrationPin.stl"
    marker.mesh_use_embedded_materials = True

    print("Publishing CalibrationPin marker...")
    print(f"Mesh resource: {marker.mesh_resource}")
    print(f"Frame: {marker.header.frame_id}")
    print(f"Position: ({marker.pose.position.x}, {marker.pose.position.y}, {marker.pose.position.z})")

    # Publish continuously
    rate = node.create_rate(10)  # 10 Hz
    try:
        while rclpy.ok():
            marker.header.stamp = node.get_clock().now().to_msg()
            marker_pub.publish(marker)
            print(".", end="", flush=True)
            rate.sleep()
    except KeyboardInterrupt:
        print("\nShutting down...")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    publish_calibration_pin_marker()
