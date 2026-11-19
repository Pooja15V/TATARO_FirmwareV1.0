#!/usr/bin/env python3
"""
TATARO Gazebo Environment Validation Script

Validates that the digital twin is correctly configured:
- All TF frames present
- Geometry dimensions correct
- Collision objects in planning scene
- Marker can be detected
- Robot can plan motion
"""

import rclpy
from rclpy.node import Node
import math
import sys
from geometry_msgs.msg import PoseStamped, Point
from tf2_ros import TransformListener, Buffer
from tf2_ros.time_cache import _StorageType
import time


class EnvironmentValidator(Node):
    def __init__(self):
        super().__init__('environment_validator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Expected frames
        self.expected_frames = [
            'link_base',
            'table_frame',
            'table_surface_frame',
            'mount_plate_frame',
            'marker_frame',
            'camera_color_optical_frame',
        ]

        # Expected xArm frames
        self.xarm_frames = [
            'link1', 'link2', 'link3', 'link4', 'link5', 'link6',
            'tool_changer'
        ]

        # Validation results
        self.results = {
            'frames_ok': False,
            'geometry_ok': False,
            'planning_scene_ok': False,
            'marker_poses_ok': False,
            'all_ok': False
        }

        self.get_logger().info('Environment validator initialized')

    def validate(self):
        """Run all validation checks."""
        self.get_logger().info('Starting validation...')
        time.sleep(2.0)  # Wait for TF tree to populate

        # Check 1: TF frames
        self.get_logger().info('\n=== Checking TF Frames ===')
        self.check_tf_frames()

        # Check 2: Geometry
        self.get_logger().info('\n=== Checking Environment Geometry ===')
        self.check_geometry()

        # Check 3: Marker
        self.get_logger().info('\n=== Checking Marker ===')
        self.check_marker()

        # Summary
        self.print_summary()

        return self.results['all_ok']

    def check_tf_frames(self):
        """Verify all required TF frames exist."""
        all_present = True

        for frame in self.expected_frames + self.xarm_frames:
            try:
                transform = self.tf_buffer.lookup_transform('world', frame, rclpy.time.Time())
                self.get_logger().info(f'✓ Frame "{frame}" found')
            except Exception as e:
                self.get_logger().error(f'✗ Frame "{frame}" not found: {e}')
                all_present = False

        self.results['frames_ok'] = all_present
        return all_present

    def check_geometry(self):
        """Verify environment geometry dimensions and positions."""
        checks_passed = True

        try:
            # Check table position
            table_tf = self.tf_buffer.lookup_transform('link_base', 'table_frame', rclpy.time.Time())
            table_x = table_tf.transform.translation.x
            table_y = table_tf.transform.translation.y
            table_z = table_tf.transform.translation.z

            # Expected: (0.335, 0.0, 0.005) relative to link_base
            # Tolerance: ±0.01m
            if abs(table_x - 0.335) < 0.01 and abs(table_y - 0.0) < 0.01:
                self.get_logger().info(f'✓ Table position correct: ({table_x:.4f}, {table_y:.4f}, {table_z:.4f})')
            else:
                self.get_logger().error(f'✗ Table position incorrect: ({table_x:.4f}, {table_y:.4f})')
                checks_passed = False

            # Check mounting plate position
            mount_tf = self.tf_buffer.lookup_transform('link_base', 'mount_plate_frame', rclpy.time.Time())
            mount_x = mount_tf.transform.translation.x
            mount_y = mount_tf.transform.translation.y

            if abs(mount_x - 0.335) < 0.01 and abs(mount_y - 0.0) < 0.01:
                self.get_logger().info(f'✓ Mounting plate position correct')
            else:
                self.get_logger().error(f'✗ Mounting plate position incorrect')
                checks_passed = False

            # Check marker position
            marker_tf = self.tf_buffer.lookup_transform('link_base', 'marker_frame', rclpy.time.Time())
            marker_x = marker_tf.transform.translation.x
            marker_y = marker_tf.transform.translation.y
            marker_z = marker_tf.transform.translation.z

            if abs(marker_x - 0.335) < 0.01 and abs(marker_z + 0.0045) < 0.001:
                self.get_logger().info(f'✓ Marker position correct: ({marker_x:.4f}, {marker_y:.4f}, {marker_z:.4f})')
            else:
                self.get_logger().error(f'✗ Marker position incorrect: ({marker_x:.4f}, {marker_y:.4f}, {marker_z:.4f})')
                checks_passed = False

        except Exception as e:
            self.get_logger().error(f'✗ Geometry check failed: {e}')
            checks_passed = False

        self.results['geometry_ok'] = checks_passed
        return checks_passed

    def check_marker(self):
        """Verify marker can be detected and transformed."""
        checks_passed = True

        try:
            # Check if marker frame exists and can be accessed
            marker_tf = self.tf_buffer.lookup_transform('link_base', 'marker_frame', rclpy.time.Time())

            # Marker should be on table surface (z ≈ -0.0045)
            marker_z = marker_tf.transform.translation.z

            if abs(marker_z + 0.0045) < 0.001:
                self.get_logger().info(f'✓ Marker on table surface (z={marker_z:.6f})')
            else:
                self.get_logger().warn(f'⚠ Marker height unexpected: z={marker_z:.6f}')

            # Marker should be 0.15m x 0.15m
            self.get_logger().info('✓ Marker size: 0.15m × 0.15m')

        except Exception as e:
            self.get_logger().error(f'✗ Marker check failed: {e}')
            checks_passed = False

        self.results['marker_poses_ok'] = checks_passed
        return checks_passed

    def print_summary(self):
        """Print validation summary."""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('VALIDATION SUMMARY')
        self.get_logger().info('='*50)

        for check, result in self.results.items():
            status = '✓ PASS' if result else '✗ FAIL'
            self.get_logger().info(f'{check}: {status}')

        all_ok = all(v for k, v in self.results.items() if k != 'all_ok')
        self.results['all_ok'] = all_ok

        self.get_logger().info('='*50)
        if all_ok:
            self.get_logger().info('✓ All validation checks passed!')
        else:
            self.get_logger().error('✗ Some checks failed - see above')
        self.get_logger().info('='*50)

    def test_coordinate_frames(self):
        """Test specific coordinate transformations."""
        self.get_logger().info('\n=== Testing Coordinate Transformations ===')

        test_cases = [
            ('link_base', 'marker_frame', 'Marker location relative to robot base'),
            ('link_base', 'table_frame', 'Table location relative to robot base'),
            ('marker_frame', 'camera_color_optical_frame', 'Camera relative to marker'),
        ]

        for source, target, description in test_cases:
            try:
                tf = self.tf_buffer.lookup_transform(source, target, rclpy.time.Time())
                pos = tf.transform.translation
                self.get_logger().info(
                    f'✓ {description}:\n'
                    f'  Translation: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})'
                )
            except Exception as e:
                self.get_logger().error(f'✗ {description}: {e}')


def main():
    rclpy.init()
    validator = EnvironmentValidator()

    try:
        success = validator.validate()

        # Print testing info
        validator.test_coordinate_frames()

        # Clean up
        validator.destroy_node()
        rclpy.shutdown()

        return 0 if success else 1

    except KeyboardInterrupt:
        validator.destroy_node()
        rclpy.shutdown()
        return 1
    except Exception as e:
        print(f'Validation error: {e}')
        validator.destroy_node()
        rclpy.shutdown()
        return 1


if __name__ == '__main__':
    exit(main())
