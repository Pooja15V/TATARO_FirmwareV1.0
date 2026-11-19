#!/usr/bin/env python3
"""
Replay robot poses from CSV file by displaying them.

Reads a robot_poses_*.csv file and displays each pose sequentially.
You manually move the robot via teach pendant to match each displayed pose.

Prerequisites:
  - xarm_moveit_config already launched with real robot
  - Robot ready for manual movement

Usage:
  python3 pose_replay.py /path/to/robot_poses_*.csv

Controls:
  [N] - move to next pose (display next target)
  [S] - skip current pose
  [Q] - quit
"""

import csv
import sys
import threading
import termios
import tty
import time
from pathlib import Path
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation


class PoseReplayNode(Node):
    """Display robot poses from CSV for manual movement."""

    def __init__(self, pose_csv: str):
        super().__init__("pose_replay")

        self.pose_csv = Path(pose_csv)
        if not self.pose_csv.exists():
            raise FileNotFoundError(f"Pose CSV not found: {self.pose_csv}")

        self._poses = self._load_poses()
        self._current_idx = 0
        self._running = True

        self.get_logger().info(
            "\n" + "=" * 70 + "\n"
            f"  Pose Replay from CSV\n"
            f"  File: {self.pose_csv.name}\n"
            f"  Total poses: {len(self._poses)}\n"
            "  Controls: [N] next   [S] skip   [Q] quit\n"
            + "=" * 70
        )

        threading.Thread(target=self._keyboard_loop, daemon=True).start()

    def _load_poses(self) -> List[Tuple[int, Pose]]:
        """Load poses from CSV. Return list of (pose_id, Pose)."""
        poses = []
        with open(self.pose_csv, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                pose_id = int(row["pose_id"])
                pose = Pose()
                pose.position.x = float(row["link_tcp_x"])
                pose.position.y = float(row["link_tcp_y"])
                pose.position.z = float(row["link_tcp_z"])
                pose.orientation.x = float(row["link_tcp_qx"])
                pose.orientation.y = float(row["link_tcp_qy"])
                pose.orientation.z = float(row["link_tcp_qz"])
                pose.orientation.w = float(row["link_tcp_qw"])
                poses.append((pose_id, pose))
        return poses

    def _display_pose(self, pose: Pose) -> bool:
        """Display target pose for manual movement."""
        pose_id, _ = self._poses[self._current_idx]

        q = np.array([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        ])
        rpy = Rotation.from_quat(q).as_euler("xyz", degrees=True)

        self.get_logger().info(
            f"\n[Pose {pose_id:03d}/{len(self._poses)-1:03d}]\n"
            f"  Position: X={pose.position.x:.4f} m  Y={pose.position.y:.4f} m  Z={pose.position.z:.4f} m\n"
            f"  Rotation:  R={rpy[0]:7.1f}°  P={rpy[1]:7.1f}°  Y={rpy[2]:7.1f}°\n"
            f"\n→ Move robot to this pose via teach pendant.\n"
            f"→ Once settled, press [N] to show next pose."
        )
        return True

    def _next_pose(self):
        """Show next pose in sequence."""
        if self._current_idx >= len(self._poses):
            self.get_logger().info("All poses completed!")
            return

        self._display_pose(self._poses[self._current_idx][1])
        self._current_idx += 1

    def _skip_pose(self):
        """Skip current pose and show next."""
        pose_id = self._poses[self._current_idx][0]
        self.get_logger().info(f"Skipped pose {pose_id:03d}")
        self._current_idx += 1
        if self._current_idx < len(self._poses):
            self._display_pose(self._poses[self._current_idx][1])

    def _keyboard_loop(self):
        """Listen for [N]ext, [S]kip, [Q]uit."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)

            # Start with first pose
            self._next_pose()

            while self._running and rclpy.ok():
                ch = sys.stdin.read(1)
                if ch.lower() == "n":
                    if self._current_idx < len(self._poses):
                        self._next_pose()
                    else:
                        self.get_logger().info("All poses completed. Press [Q] to quit.")
                elif ch.lower() == "s":
                    self._skip_pose()
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
    if len(sys.argv) < 2:
        print("Usage: python3 pose_replay.py <robot_poses_*.csv>")
        sys.exit(1)

    rclpy.init(args=args)
    try:
        node = PoseReplayNode(sys.argv[1])
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
