#!/usr/bin/env python3
"""
Test version of pose_replay_sdk.py — simulates robot without actual connection.

Same interface as pose_replay_sdk.py but doesn't connect to real hardware.
Use this to verify the script logic and CSV loading before running on real robot.

Usage:
  python3 pose_replay_test.py /path/to/robot_poses_*.csv

Controls:
  [N] - move to next pose (simulated)
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


class PoseReplayTest:
    """Test/simulation version of pose replay."""

    def __init__(self, pose_csv: str):
        self.pose_csv = Path(pose_csv)

        if not self.pose_csv.exists():
            raise FileNotFoundError(f"Pose CSV not found: {self.pose_csv}")

        # Load poses
        self._poses = self._load_poses()
        self._current_idx = 0
        self._running = True

        print(
            "\n" + "=" * 70 + "\n"
            f"  Pose Replay TEST (Simulation Mode)\n"
            f"  File: {self.pose_csv.name}\n"
            f"  Total poses: {len(self._poses)}\n"
            "  Controls: [N] next   [S] skip   [Q] quit\n"
            + "=" * 70 + "\n"
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
        """Simulate moving robot to target pose."""
        pose_id, pose = self._poses[self._current_idx]

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self._quat_to_euler(
            pose['qx'], pose['qy'], pose['qz'], pose['qw']
        )

        print(
            f"\n[Pose {pose_id:03d}/{len(self._poses)-1:03d}]\n"
            f"  Position: X={pose['x']:.1f} mm  Y={pose['y']:.1f} mm  Z={pose['z']:.1f} mm\n"
            f"  Rotation:  R={roll:7.1f}°  P={pitch:7.1f}°  Y={yaw:7.1f}°"
        )

        # Simulate motion delay
        print("[SIM] Moving (1 second delay)...")
        time.sleep(1)

        print(f"✓ [SIM] Pose {pose_id:03d} reached (simulated).")
        print("→ Go to tf_recorder and press [S] to capture.")
        print("→ Then press [N] here to move to next pose.")
        return True

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


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 pose_replay_test.py <robot_poses_*.csv>")
        print("Example: python3 pose_replay_test.py robot_poses_20260616_132545.csv")
        sys.exit(1)

    pose_csv = sys.argv[1]

    try:
        replay = PoseReplayTest(pose_csv)

        # Keep running until user quits
        while replay._running:
            time.sleep(0.1)

        print("\nTest completed.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as exc:
        print(f"Error: {exc}")
        sys.exit(1)


if __name__ == "__main__":
    main()
