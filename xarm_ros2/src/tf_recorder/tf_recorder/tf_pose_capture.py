#!/usr/bin/env python3
"""
Capture robot TCP poses interactively for later automation.

Listens for keypresses:
  [S] - save current TCP pose to CSV
  [N] - move to next pose (just logs, you move manually)
  [Q] or Ctrl+C - quit

Output: robot_poses_<timestamp>.csv with columns:
  pose_id, timestamp_sec, timestamp_nanosec,
  link_tcp_x, link_tcp_y, link_tcp_z, link_tcp_qx, link_tcp_qy, link_tcp_qz, link_tcp_qw

Usage:
  ros2 run tf_recorder tf_pose_capture.py
"""

import csv
import os
import sys
import threading
import termios
import tty
from datetime import datetime
from pathlib import Path
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException


class PoseCaptureNode(Node):
    """Interactive pose capture for hand-eye calibration automation."""

    def __init__(self):
        super().__init__("tf_pose_capture")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Output CSV
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        poses_dir = Path(
            '/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0'
            '/xarm_ros2/install/tf_recorder/TF_Records/RobotPoseForCalibration'
        )
        poses_dir.mkdir(parents=True, exist_ok=True)
        self._output_csv = poses_dir / f'robot_poses_{timestamp}.csv'

        self._csv_lock = threading.Lock()
        self._pose_id = 0
        self._running = True

        self._init_csv()

        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self.get_logger().info(
            "\n" + "=" * 60 + "\n"
            "  Robot Pose Capture for Automation\n"
            + f"  Output: {self._output_csv}\n"
            "  Keys: [S] save   [N] next   [Q / Ctrl+C] quit\n"
            + "=" * 60
        )

    def _init_csv(self):
        self._output_csv.parent.mkdir(parents=True, exist_ok=True)
        with open(self._output_csv, "w", newline="") as f:
            csv.writer(f).writerow([
                "pose_id", "timestamp_sec", "timestamp_nanosec",
                "link_tcp_x", "link_tcp_y", "link_tcp_z",
                "link_tcp_qx", "link_tcp_qy", "link_tcp_qz", "link_tcp_qw",
            ])
        self.get_logger().info(f"Created: {self._output_csv}")

    def _lookup_tcp(self):
        """Look up T_base_tcp."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "link_base",
                "link_tcp",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return tf, None
        except (LookupException, ExtrapolationException, ConnectivityException) as exc:
            return None, str(exc)

    def _capture_pose(self):
        now = self.get_clock().now()
        ts = now.to_msg()

        tf, err = self._lookup_tcp()
        if tf is None:
            self.get_logger().warn(f"[Pose {self._pose_id:03d}] FAILED — {err}")
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        row = [
            self._pose_id,
            ts.sec,
            ts.nanosec,
            t.x, t.y, t.z,
            q.x, q.y, q.z, q.w,
        ]

        with self._csv_lock:
            with open(self._output_csv, "a", newline="") as f:
                csv.writer(f).writerow(row)
                f.flush()
                os.fsync(f.fileno())

        self.get_logger().info(
            f"[Pose {self._pose_id:03d}] SAVED  "
            f"TCP=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})"
        )
        self._pose_id += 1

    def _keyboard_loop(self):
        """Raw stdin listener for [S]ave, [N]ext, [Q]uit."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running and rclpy.ok():
                ch = sys.stdin.read(1)
                if ch.lower() == "s":
                    self._capture_pose()
                elif ch.lower() == "n":
                    self.get_logger().info(
                        f"→ Ready for pose {self._pose_id:03d}. "
                        "Move robot and press [S] to save."
                    )
                elif ch.lower() == "q" or ch == "\x03":
                    self.get_logger().info("Quit key received.")
                    rclpy.shutdown()
                    break
        except Exception as exc:
            self.get_logger().error(f"Keyboard thread error: {exc}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
