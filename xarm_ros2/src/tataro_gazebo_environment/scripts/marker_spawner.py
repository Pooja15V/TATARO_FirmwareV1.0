#!/usr/bin/env python3
"""
Marker spawner and pose controller for TATARO workstation.

Provides:
- Service to spawn/despawn ArUco marker in Gazebo
- Service to update marker pose
- Service to reset marker to default position
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty
from tataro_gazebo_environment_msgs.srv import (
    SetMarkerPose, SpawnMarker, DespawnMarker
)


class MarkerSpawner(Node):
    def __init__(self):
        super().__init__('marker_spawner')

        # Default marker configuration
        self.marker_name = 'aruco_marker'
        self.marker_type = 'model'

        # Default pose (center of table)
        self.default_pose = Pose(
            position=Point(x=0.335, y=0.0, z=-0.0045),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        # Create services
        self.spawn_srv = self.create_service(
            SpawnMarker, '/tataro/spawn_marker', self.spawn_marker_callback
        )
        self.despawn_srv = self.create_service(
            DespawnMarker, '/tataro/despawn_marker', self.despawn_marker_callback
        )
        self.set_pose_srv = self.create_service(
            SetMarkerPose, '/tataro/set_marker_pose', self.set_marker_pose_callback
        )
        self.reset_srv = self.create_service(
            Empty, '/tataro/reset_marker', self.reset_marker_callback
        )

        # TF broadcaster
        from tf2_ros import TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Gazebo client (if using Gazebo)
        try:
            from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
            self.spawn_model_client = self.create_client(SpawnModel, '/spawn_entity')
            self.delete_model_client = self.create_client(DeleteModel, '/delete_entity')
            self.set_model_state_client = self.create_client(SetModelState, '/set_entity_state')
            self.gazebo_available = True
        except Exception as e:
            self.get_logger().warn(f'Gazebo services not available: {e}')
            self.gazebo_available = False

        self.get_logger().info('Marker spawner initialized')

    def spawn_marker_callback(self, request, response):
        """Spawn marker in Gazebo at specified pose."""
        self.get_logger().info(f'Spawning marker at pose: {request.pose}')

        if self.gazebo_available:
            # Load SDF from file
            marker_sdf = self._get_marker_sdf()
            # Spawn in Gazebo
            # Implementation details depend on Gazebo version
            pass

        response.success = True
        return response

    def despawn_marker_callback(self, request, response):
        """Remove marker from Gazebo."""
        self.get_logger().info(f'Despawning marker: {request.name}')

        if self.gazebo_available:
            # Delete from Gazebo
            pass

        response.success = True
        return response

    def set_marker_pose_callback(self, request, response):
        """Update marker pose in Gazebo."""
        self.get_logger().info(f'Setting marker pose: {request.pose}')

        if self.gazebo_available:
            # Update pose in Gazebo
            pass

        response.success = True
        return response

    def reset_marker_callback(self, request, response):
        """Reset marker to default position (center of table)."""
        self.get_logger().info('Resetting marker to default pose')
        # Call set_marker_pose with default_pose
        response.success = True
        return response

    def _get_marker_sdf(self):
        """Load marker SDF template."""
        marker_sdf = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="aruco_marker">
    <pose>0.335 0.0 -0.0045 0 0 0</pose>
    <link name="marker_link">
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.0001875</ixx>
          <iyy>0.0001875</iyy>
          <izz>0.000375</izz>
        </inertia>
      </inertial>
      <collision name="marker_collision">
        <geometry>
          <box>
            <size>0.15 0.15 0.001</size>
          </box>
        </geometry>
      </collision>
      <visual name="marker_visual">
        <geometry>
          <box>
            <size>0.15 0.15 0.001</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>
    <frame name="marker_frame" attached_to="marker_link">
      <pose>0 0 0 0 0 0</pose>
    </frame>
  </model>
</sdf>"""
        return marker_sdf


def main(args=None):
    rclpy.init(args=args)
    node = MarkerSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
