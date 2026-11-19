#!/usr/bin/env python3
"""
Replay robot poses from CSV file using xArm SDK.

Reads a robot_poses_*.csv file and sequentially moves the robot to each pose
using the xArm Python SDK directly.

Prerequisites:
  - xArm robot connected at the specified IP
  - Robot initialized and ready for motion

Usage:
  python3 pose_replay_sdk.py <robot_ip> /path/to/robot_poses_*.csv

Examples:
  python3 pose_replay_sdk.py 192.168.1.150 robot_poses_20260616_121947.csv
xarm_ros2/install/tf_recorder/TF_Records/RobotPoseForCalibration/robot_poses_20260616_132545.csv
Controls:
  [N] - move to next pose
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
from scipy.spatial.transform import Rotation

# Add xArm SDK to path
import os
sys.path.insert(0, '/home/pooja/TATRO/TATRO_Program_files/TATRO_Robot_Firmware/xArm-Python-SDK')

from xarm.wrapper import XArmAPI


class PoseReplaySDK:
    """Replay robot poses from CSV using xArm SDK."""

    def __init__(self, robot_ip: str, pose_csv: str):
        self.robot_ip = robot_ip
        self.pose_csv = Path(pose_csv)

        if not self.pose_csv.exists():
            raise FileNotFoundError(f"Pose CSV not found: {self.pose_csv}")

        # Connect to robot
        print(f"Connecting to robot at {robot_ip}...")
        self.arm = XArmAPI(robot_ip)
        time.sleep(1)

        # Initialize robot
        print("Initializing robot...")

        # Clear any existing errors
        self.arm.clean_error()
        time.sleep(0.5)

        # Set up motion
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)  # Mode 0: position control
        self.arm.set_state(state=0)  # State 0: ready
        time.sleep(0.5)

        # Move to home position first
        print("Moving to home position...")
        self.arm.move_gohome(wait=True)
        print("Home position reached.")
        time.sleep(1)

        # Load poses
        self._poses = self._load_poses()
        self._current_idx = 0
        self._running = True

        print(
            "\n" + "=" * 70 + "\n"
            f"  Pose Replay from CSV (xArm SDK)\n"
            f"  Robot IP: {robot_ip}\n"
            f"  File: {self.pose_csv.name}\n"
            f"  Total poses: {len(self._poses)}\n"
            "  Controls: [N] next   [S] skip   [Q] quit\n"
            + "=" * 70
        )

        threading.Thread(target=self._keyboard_loop, daemon=True).start()

    def _load_poses(self) -> List[Tuple[int, dict]]:
        """Load poses from CSV. Return list of (pose_id, pose_dict)."""
        poses = []
        with open(self.pose_csv, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                pose_id = int(row["pose_id"])
                pose = {
                    'x': float(row["link_tcp_x"]) * 1000,  # Convert m to mm
                    'y': float(row["link_tcp_y"]) * 1000,
                    'z': float(row["link_tcp_z"]) * 1000,
                    'qx': float(row["link_tcp_qx"]),
                    'qy': float(row["link_tcp_qy"]),
                    'qz': float(row["link_tcp_qz"]),
                    'qw': float(row["link_tcp_qw"]),
                }
                poses.append((pose_id, pose))
        return poses

    def _quat_to_euler(self, qx, qy, qz, qw) -> tuple:
        """Convert quaternion to Euler angles (roll, pitch, yaw in degrees)."""
        q = np.array([qx, qy, qz, qw])
        rpy = Rotation.from_quat(q).as_euler("xyz", degrees=True)
        return rpy[0], rpy[1], rpy[2]

    def _move_to_pose(self) -> bool:
        """Move robot to target pose. Returns True if successful."""
        pose_id, pose = self._poses[self._current_idx]

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self._quat_to_euler(
            pose['qx'], pose['qy'], pose['qz'], pose['qw']
        )

        print(
            f"\n[Pose {pose_id:03d}/{len(self._poses)-1:03d}]\n"
            f"  Position: X={pose['x']:.1f} mm  Y={pose['y']:.1f} mm  Z={pose['z']:.1f} mm\n"
            f"  Rotation:  R={roll:7.1f}°  P={pitch:7.1f}°  Y={yaw:7.1f}°\n"
        )

        try:
            print("Moving...")
            code = self.arm.set_position(
                x=pose['x'],
                y=pose['y'],
                z=pose['z'],
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                speed=50,  # mm/s
                mvacc=500,  # mm/s^2
                wait=True,
                timeout=10
            )

            if code == 0:
                print(f"✓ Pose {pose_id:03d} reached. Ready for capture.")
                print("→ Go to tf_recorder and press [S] to capture.")
                print("→ Then press [N] here to move to next pose.")
                return True
            else:
                # Clear error and try to recover
                print(f"✗ Motion failed with code {code}")
                self.arm.clean_error()
                time.sleep(0.5)
                print("Attempted error recovery. Press [N] to retry or [S] to skip.")
                return False

        except Exception as exc:
            print(f"✗ Error: {exc}")
            return False

    def _next_pose(self):
        """Move to next pose in sequence."""
        if self._current_idx >= len(self._poses):
            print("All poses completed!")
            return

        if self._move_to_pose():
            self._current_idx += 1

    def _skip_pose(self):
        """Skip current pose and move to next."""
        pose_id = self._poses[self._current_idx][0]
        print(f"Skipped pose {pose_id:03d}")
        self._current_idx += 1
        if self._current_idx < len(self._poses):
            print(f"Ready. Press [N] to move to pose {self._poses[self._current_idx][0]:03d}")

    def _keyboard_loop(self):
        """Listen for [N]ext, [S]kip, [Q]uit."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)

            # Start with first pose
            self._next_pose()

            while self._running:
                ch = sys.stdin.read(1)
                if ch.lower() == "n":
                    if self._current_idx < len(self._poses):
                        self._next_pose()
                    else:
                        print("All poses completed. Press [Q] to quit.")
                elif ch.lower() == "s":
                    self._skip_pose()
                elif ch.lower() == "q" or ch == "\x03":
                    print("Quit key received.")
                    self._running = False
                    break
        except Exception as exc:
            print(f"Keyboard error: {exc}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def cleanup(self):
        """Clean up and disconnect."""
        try:
            self.arm.motion_enable(enable=False)
            self.arm.disconnect()
            print("Robot disconnected.")
        except:
            pass


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 pose_replay_sdk.py <robot_ip> <robot_poses_*.csv>")
        print("Example: python3 pose_replay_sdk.py 192.168.1.150 robot_poses_20260616_121947.csv")
        sys.exit(1)

    robot_ip = sys.argv[1]
    pose_csv = sys.argv[2]

    try:
        replay = PoseReplaySDK(robot_ip, pose_csv)

        # Keep running until user quits
        while replay._running:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as exc:
        print(f"Error: {exc}")
        sys.exit(1)
    finally:
        try:
            replay.cleanup()
        except:
            pass


if __name__ == "__main__":
    main()
