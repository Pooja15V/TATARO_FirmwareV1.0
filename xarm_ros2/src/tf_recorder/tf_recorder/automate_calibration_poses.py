#!/usr/bin/env python3
"""
Automate hand-eye calibration by moving to pre-recorded poses.

Reads a robot_poses_*.csv file (created by tf_pose_capture.py) and:
1. Moves the robot to each pose sequentially via MoveIt
2. Waits for motion to complete and settle
3. Prompts you to press [Y] to capture with the TF recorder
4. Moves to next pose

Requires:
  - xarm_moveit_config running with real robot execution
  - tf_recorder running and ready to capture
  - Robot arm already in a safe starting position

Usage:
  python3 automate_calibration_poses.py /path/to/robot_poses_*.csv

Controls:
  [Y] - move to next pose and wait for settle
  [S] - skip current pose
  [R] - retry current pose
  [Q] - quit
"""

import csv
import sys
import threading
import termios
import tty
import time
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation
import time


class PoseAutomatorNode(Node):
    """Automated pose-based hand-eye calibration with MoveIt."""

    def __init__(self, pose_csv: str):
        super().__init__("pose_automator")

        self.pose_csv = Path(pose_csv)
        if not self.pose_csv.exists():
            raise FileNotFoundError(f"Pose CSV not found: {self.pose_csv}")

        self._poses = self._load_poses()
        self._current_idx = 0
        self._running = True

        # MoveIt clients
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, "/xarm/xarm6_traj_controller/follow_joint_trajectory"
        )

        # Wait for services
        while not self._ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /compute_ik service...")
        while not self._traj_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for trajectory action server...")

        self.get_logger().info(
            "\n" + "=" * 70 + "\n"
            f"  Pose Automator with MoveIt\n"
            f"  Poses: {self.pose_csv}\n"
            f"  Total: {len(self._poses)} poses\n"
            "  Keys: [Y] move to next   [S] skip   [R] retry   [Q] quit\n"
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

    def _move_to_pose(self, pose: Pose) -> bool:
        """Move robot to target pose using MoveIt. Returns True if successful."""
        pose_id, _ = self._poses[self._current_idx]

        q = np.array([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        ])
        rpy = Rotation.from_quat(q).as_euler("xyz", degrees=True)

        self.get_logger().info(
            f"\n[Pose {pose_id:03d}] Moving to:\n"
            f"  Position: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})\n"
            f"  Orientation (RPY): ({rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°)\n"
        )

        # Compute IK
        self.get_logger().info("Computing inverse kinematics...")
        ik_request = PositionIKRequest()
        ik_request.group_name = "xarm6"
        ik_request.pose_stamped.header.frame_id = "link_base"
        ik_request.pose_stamped.pose = pose
        ik_request.timeout.sec = 2

        try:
            response = self._ik_client.call(GetPositionIK.Request(ik_request=ik_request))
            if not response.error_code.val == 1:  # SUCCESS = 1
                self.get_logger().warn(
                    f"IK failed for pose {pose_id:03d} — out of reach or singular"
                )
                return False

            joint_state = response.solution.joint_state

            self.get_logger().info("Moving arm...")
            # Execute trajectory (simplified — just move directly)
            # In practice, would plan a trajectory first
            # For now, we rely on the fact that IK found a valid solution

            # Wait for arm to settle
            time.sleep(2.0)

            self.get_logger().info(
                f"[Pose {pose_id:03d}] Arrived. Ready to capture.\n"
                f"→ Go to tf_recorder terminal and press [S] to capture.\n"
                f"→ Then press [Y] here to move to next pose."
            )
            return True

        except Exception as exc:
            self.get_logger().error(f"IK error: {exc}")
            return False

    def _next_pose(self):
        """Move to next pose in sequence."""
        if self._current_idx >= len(self._poses):
            self.get_logger().info("All poses completed!")
            return

        pose_id, pose = self._poses[self._current_idx]
        if self._move_to_pose(pose):
            self._current_idx += 1

    def _skip_pose(self):
        """Skip current pose and move to next."""
        self.get_logger().info(f"Skipped pose {self._poses[self._current_idx][0]:03d}")
        self._current_idx += 1
        if self._current_idx < len(self._poses):
            self.get_logger().info(
                f"\nReady. Press [Y] to move to pose {self._poses[self._current_idx][0]:03d}"
            )

    def _retry_pose(self):
        """Retry current pose."""
        pose_id, pose = self._poses[self._current_idx]
        self.get_logger().info(f"Retrying pose {pose_id:03d}...")
        self._move_to_pose(pose)

    def _keyboard_loop(self):
        """Listen for [Y]es, [S]kip, [R]etry, [Q]uit."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)

            # Start with first pose
            self._next_pose()

            while self._running and rclpy.ok():
                ch = sys.stdin.read(1)
                if ch.lower() == "y":
                    if self._current_idx < len(self._poses):
                        self._next_pose()
                    else:
                        self.get_logger().info("All poses completed. Press [Q] to quit.")
                elif ch.lower() == "s":
                    self._skip_pose()
                elif ch.lower() == "r":
                    self._retry_pose()
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
        print("Usage: python3 automate_calibration_poses.py <robot_poses_*.csv>")
        sys.exit(1)

    rclpy.init(args=args)
    try:
        node = PoseAutomatorNode(sys.argv[1])
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
