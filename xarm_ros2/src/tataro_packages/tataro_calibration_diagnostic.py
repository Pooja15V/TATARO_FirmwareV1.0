#!/usr/bin/env python3
"""
Hand-eye calibration diagnostic: measure actual positioning error.

This script detects an ArUco marker and measures how far the robot actually is
from the marker center when it reaches the "target" position.

Run:
  ros2 run tataro_packages tataro_calibration_diagnostic
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import rclpy.duration
import rclpy.time

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    sys.exit("[ERROR] xarm Python SDK not found.\n        pip install xarm-python-sdk")

ROBOT_IP = '192.168.1.150'
BASE_FRAME = 'link_base'
MARKER_FRAME = 'aruco_marker_frame'

class TFNode(Node):
    def __init__(self):
        super().__init__('tataro_calibration_diagnostic')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

def lookup_marker(node: TFNode, timeout_s: float = 6.0):
    """Block until marker TF is available."""
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)
    while node.get_clock().now() < deadline:
        try:
            return node.tf_buffer.lookup_transform(
                BASE_FRAME, MARKER_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            rclpy.spin_once(node, timeout_sec=0.1)
    return None

def main():
    rclpy.init()
    node = TFNode()

    # Warm up TF buffer
    print("[TF] Filling buffer …")
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Lookup marker
    print(f"[TF] Looking up {MARKER_FRAME} in {BASE_FRAME} …")
    t = lookup_marker(node, timeout_s=6.0)

    if t is None:
        print(f"\n[ERROR] {MARKER_FRAME} not found in TF tree")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Extract marker position
    tr = t.transform.translation
    marker_x_m = tr.x
    marker_y_m = tr.y
    marker_z_m = tr.z
    marker_x = marker_x_m * 1000
    marker_y = marker_y_m * 1000
    marker_z = marker_z_m * 1000

    print(f"\n[MARKER] Detected at:")
    print(f"  x={marker_x:+.1f} mm, y={marker_y:+.1f} mm, z={marker_z:+.1f} mm")

    # Connect to robot
    print(f"\n[ROBOT] Connecting to {ROBOT_IP} …")
    arm = XArmAPI(ROBOT_IP, is_radian=False)
    code, state = arm.get_state()
    if code != 0:
        print(f"[ERROR] Cannot reach robot")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)

    _, current_pos = arm.get_position()
    print(f"[ROBOT] Connected. Current TCP: ({current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f})")

    # Move to marker center
    target_x = marker_x
    target_y = marker_y
    target_z = marker_z + 100  # 100mm above marker for safety

    print(f"\n[MOVE] Moving to marker hover position (neutral orientation):")
    print(f"  Target: x={target_x:.1f}, y={target_y:.1f}, z={target_z:.1f} mm")

    code = arm.set_position(target_x, target_y, target_z, 0, 0, 0,
                             speed=50, mvacc=100, wait=True)

    if code != 0:
        print(f"[ERROR] Motion failed with code={code}")
        arm.disconnect()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Read actual position after move
    code, actual_pos = arm.get_position()
    actual_x = actual_pos[0]
    actual_y = actual_pos[1]
    actual_z = actual_pos[2]

    print(f"\n[ACTUAL] TCP after move: ({actual_x:.1f}, {actual_y:.1f}, {actual_z:.1f})")

    # Calculate error
    error_x = actual_x - target_x
    error_y = actual_y - target_y
    error_z = actual_z - target_z
    error_xy = math.sqrt(error_x**2 + error_y**2)
    error_xyz = math.sqrt(error_x**2 + error_y**2 + error_z**2)

    print(f"\n[ERROR ANALYSIS]")
    print(f"  ΔX: {error_x:+.1f} mm")
    print(f"  ΔY: {error_y:+.1f} mm")
    print(f"  ΔZ: {error_z:+.1f} mm")
    print(f"  XY Error: {error_xy:.1f} mm (horizontal)")
    print(f"  XYZ Error: {error_xyz:.1f} mm (total)")

    # Assess quality
    print(f"\n[ASSESSMENT]")
    if error_xyz < 10:
        print(f"  ✅ EXCELLENT: Error < 10 mm")
    elif error_xyz < 25:
        print(f"  🟢 GOOD: Error < 25 mm")
    elif error_xyz < 50:
        print(f"  🟡 ACCEPTABLE: Error < 50 mm")
    elif error_xyz < 100:
        print(f"  🟠 POOR: Error < 100 mm (recalibrate recommended)")
    else:
        print(f"  ❌ UNACCEPTABLE: Error > 100 mm (major issue)")

    # Check if error is systematic
    if abs(error_x) > 20 or abs(error_y) > 20 or abs(error_z) > 20:
        print(f"\n[WARNING] Large systematic error detected")
        print(f"  → Calibration may be incorrect or camera may have drifted")
        print(f"  → Recommend re-running hand-eye calibration")

    arm.disconnect()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import math
    main()
