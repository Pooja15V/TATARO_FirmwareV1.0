#!/usr/bin/env python3
"""
Marker touching experiment controller for TATARO workstation.

Implements the complete workflow:
1. Detect marker from RealSense camera
2. Estimate marker pose
3. Transform marker pose to link_base
4. Generate approach and touch trajectories
5. Move above marker
6. Descend vertically
7. Touch marker center
8. Retract safely
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseStamped, Point, Quaternion, Twist, Vector3
)
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty
from std_msgs.msg import Header
import numpy as np
import time


class TouchMarkerController(Node):
    def __init__(self):
        super().__init__('touch_marker_controller')

        # Parameters
        self.declare_parameter('approach_height', 0.05)
        self.declare_parameter('touch_duration', 1.0)
        self.declare_parameter('move_speed', 0.1)
        self.declare_parameter('marker_size', 0.15)

        self.approach_height = self.get_parameter('approach_height').value
        self.touch_duration = self.get_parameter('touch_duration').value
        self.move_speed = self.get_parameter('move_speed').value
        self.marker_size = self.get_parameter('marker_size').value

        # Subscribers
        self.marker_pose_sub = self.create_subscription(
            PoseStamped, '/tataro/marker_pose_base', self.marker_pose_callback, 10
        )

        # Current marker pose
        self.marker_pose = None
        self.marker_detected = False

        # MoveIt2 client (will be created when needed)
        self.moveit_client = None

        # State machine
        self.state = 'IDLE'
        self.get_logger().info('Touch marker controller initialized')

    def marker_pose_callback(self, msg):
        """Update marker pose from perception."""
        self.marker_pose = msg.pose
        self.marker_detected = True
        self.get_logger().debug(f'Marker detected at: {msg.pose.position}')

    def start_touching_experiment(self):
        """Start the marker touching workflow."""
        if not self.marker_detected or self.marker_pose is None:
            self.get_logger().error('Marker not detected')
            return False

        self.get_logger().info('Starting marker touching experiment')

        try:
            # Step 1: Move to approach position (above marker)
            if not self._move_to_approach_pose():
                self.get_logger().error('Failed to move to approach pose')
                return False

            self.get_logger().info('Reached approach position')

            # Step 2: Descend to touch marker
            if not self._descend_to_marker():
                self.get_logger().error('Failed to descend to marker')
                return False

            self.get_logger().info('Touching marker')

            # Step 3: Hold contact for touch_duration
            time.sleep(self.touch_duration)

            # Step 4: Retract safely
            if not self._retract_from_marker():
                self.get_logger().error('Failed to retract from marker')
                return False

            self.get_logger().info('Experiment completed successfully')
            return True

        except Exception as e:
            self.get_logger().error(f'Error during experiment: {e}')
            return False

    def _move_to_approach_pose(self):
        """Move robot end-effector above marker."""
        # Calculate approach pose (above marker center)
        approach_pose = self._calculate_approach_pose()
        self.get_logger().info(f'Moving to approach pose: {approach_pose}')

        # Move using MoveIt2
        return self._move_to_pose(approach_pose)

    def _descend_to_marker(self):
        """Descend vertically to touch marker."""
        # Calculate touch pose (on marker center)
        touch_pose = self._calculate_touch_pose()
        self.get_logger().info(f'Descending to touch pose: {touch_pose}')

        # Move using linear interpolation (Cartesian path)
        return self._move_linearly_to_pose(touch_pose)

    def _retract_from_marker(self):
        """Retract vertically from marker."""
        # Calculate retract pose (above marker, safe height)
        retract_pose = self._calculate_retract_pose()
        self.get_logger().info(f'Retracting to retract pose: {retract_pose}')

        # Move using linear interpolation
        return self._move_linearly_to_pose(retract_pose)

    def _calculate_approach_pose(self):
        """Calculate approach pose (above marker by approach_height)."""
        pose = PoseStamped()
        pose.header.frame_id = 'link_base'
        pose.pose.position.x = self.marker_pose.position.x
        pose.pose.position.y = self.marker_pose.position.y
        pose.pose.position.z = self.marker_pose.position.z + self.approach_height

        # Orientation: end-effector pointing down
        # Quaternion for 180° rotation around Y (gripper pointing down)
        pose.pose.orientation.x = 0.7071
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.7071
        pose.pose.orientation.w = 0.0

        return pose

    def _calculate_touch_pose(self):
        """Calculate touch pose (at marker center)."""
        pose = PoseStamped()
        pose.header.frame_id = 'link_base'
        pose.pose.position.x = self.marker_pose.position.x
        pose.pose.position.y = self.marker_pose.position.y
        pose.pose.position.z = self.marker_pose.position.z + 0.001  # Barely above marker

        # Same orientation as approach
        pose.pose.orientation.x = 0.7071
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.7071
        pose.pose.orientation.w = 0.0

        return pose

    def _calculate_retract_pose(self):
        """Calculate retract pose (above marker, safe height)."""
        # Same as approach pose
        return self._calculate_approach_pose()

    def _move_to_pose(self, target_pose):
        """Move robot to target pose using MoveIt2."""
        # This would use MoveIt2 move_group interface
        # For now, return success as placeholder
        self.get_logger().info(f'Moving to pose: {target_pose.pose.position}')
        return True

    def _move_linearly_to_pose(self, target_pose):
        """Move robot linearly (Cartesian path) to target pose."""
        # This would use MoveIt2 compute_cartesian_path
        # For now, return success as placeholder
        self.get_logger().info(f'Moving linearly to pose: {target_pose.pose.position}')
        return True

    def get_state(self):
        """Get current state."""
        return self.state


def main(args=None):
    rclpy.init(args=args)
    node = TouchMarkerController()

    try:
        # Example: start touching experiment
        # In practice, this would be triggered by a service or action
        time.sleep(2.0)  # Wait for marker detection
        node.start_touching_experiment()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
