#!/usr/bin/env python3
"""
ArUco marker pose publisher for TATARO workstation.

Simulates ArUco marker detection from RealSense camera and publishes:
- Detected marker pose in camera frame
- Marker pose transformed to robot base frame
- Marker TF frames
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    TransformStamped, PoseStamped, Point, Quaternion
)
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
import numpy as np
from scipy.spatial.transform import Rotation as R


class MarkerPosePublisher(Node):
    def __init__(self):
        super().__init__('marker_pose_publisher')

        # Parameters
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('publish_rate', 30)
        self.declare_parameter('use_sim_time', True)

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_sim_time = self.get_parameter('use_sim_time').value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.marker_pose_pub = self.create_publisher(
            PoseStamped, '/tataro/marker_pose_camera', 10
        )
        self.marker_pose_base_pub = self.create_publisher(
            PoseStamped, '/tataro/marker_pose_base', 10
        )

        # Create subscription to Gazebo model state for marker
        try:
            from gazebo_msgs.msg import ModelStates
            self.create_subscription(
                ModelStates, '/model_states', self.model_states_callback, 10
            )
            self.gazebo_available = True
        except Exception as e:
            self.get_logger().warn(f'Gazebo model_states subscription failed: {e}')
            self.gazebo_available = False

        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.timer_callback
        )

        # Marker pose (updated from Gazebo)
        self.marker_pose_world = None
        self.marker_detected = False

        # Camera to base transform (from robot configuration)
        # This is example - update based on actual camera mounting
        self.camera_to_base = self._get_camera_to_base_transform()

        self.get_logger().info(f'Marker pose publisher initialized (ID={self.marker_id})')

    def model_states_callback(self, msg):
        """Update marker pose from Gazebo model states."""
        try:
            idx = msg.name.index('aruco_marker')
            pose = msg.pose[idx]
            self.marker_pose_world = pose
            self.marker_detected = True
        except (ValueError, IndexError):
            self.marker_detected = False

    def timer_callback(self):
        """Publish marker poses and TF frames."""
        if not self.marker_detected or self.marker_pose_world is None:
            return

        current_time = self.get_clock().now()

        # Get marker pose in world frame
        marker_world = self.marker_pose_world

        # Publish marker in camera frame (simulated detection)
        marker_camera_pose = self._transform_pose_to_camera(marker_world)
        self.publish_marker_camera_pose(marker_camera_pose, current_time)

        # Publish marker in base frame
        marker_base_pose = self._transform_pose_to_base(marker_world)
        self.publish_marker_base_pose(marker_base_pose, current_time)

        # Publish TF frames
        self.publish_marker_tf(marker_base_pose, current_time)
        self.publish_camera_tf(current_time)

    def publish_marker_camera_pose(self, pose, stamp):
        """Publish marker pose in camera frame."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.pose = pose
        self.marker_pose_pub.publish(msg)

    def publish_marker_base_pose(self, pose, stamp):
        """Publish marker pose in robot base frame."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'link_base'
        msg.pose = pose
        self.marker_pose_base_pub.publish(msg)

    def publish_marker_tf(self, pose, stamp):
        """Publish marker TF frame."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'link_base'
        t.child_frame_id = 'marker_frame'
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_camera_tf(self, stamp):
        """Publish camera TF frame."""
        # Example camera mounting on robot end-effector
        # Update based on actual camera mounting in your setup
        camera_pos = [0.0, 0.0, 0.1]  # 10cm above base
        camera_rot = [0, 0, 0, 1]  # Identity rotation

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'link_base'
        t.child_frame_id = 'camera_color_optical_frame'
        t.transform.translation.x = float(camera_pos[0])
        t.transform.translation.y = float(camera_pos[1])
        t.transform.translation.z = float(camera_pos[2])
        t.transform.rotation.x = float(camera_rot[0])
        t.transform.rotation.y = float(camera_rot[1])
        t.transform.rotation.z = float(camera_rot[2])
        t.transform.rotation.w = float(camera_rot[3])
        self.tf_broadcaster.sendTransform(t)

    def _transform_pose_to_camera(self, world_pose):
        """Transform world frame pose to camera frame (simulated detection)."""
        # Invert camera_to_base to get base_to_camera
        base_to_camera_rot = R.from_quat([
            self.camera_to_base[3],  # x
            self.camera_to_base[4],  # y
            self.camera_to_base[5],  # z
            self.camera_to_base[6],  # w
        ]).inv()

        # Transform marker pose
        marker_rot = R.from_quat([
            world_pose.orientation.x,
            world_pose.orientation.y,
            world_pose.orientation.z,
            world_pose.orientation.w,
        ])

        camera_rot = base_to_camera_rot * marker_rot
        camera_quat = camera_rot.as_quat()

        # For this simplified version, return pose in base frame
        # (A proper implementation would apply actual camera extrinsics)
        pose = PoseStamped()
        pose.pose.position = world_pose.position
        pose.pose.orientation.x = float(camera_quat[0])
        pose.pose.orientation.y = float(camera_quat[1])
        pose.pose.orientation.z = float(camera_quat[2])
        pose.pose.orientation.w = float(camera_quat[3])
        return pose

    def _transform_pose_to_base(self, world_pose):
        """Transform world frame pose to base frame."""
        # For now, world frame = base frame (as defined in our setup)
        pose = PoseStamped()
        pose.pose = world_pose
        return pose

    def _get_camera_to_base_transform(self):
        """Get camera to base transformation."""
        # Example: camera mounted 10cm forward, slightly above base
        # [x, y, z, qx, qy, qz, qw]
        return [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
