#!/usr/bin/env python3
"""
Replay calibration trajectories from robot memory.

Loads and plays Calibration_Pose_01 through Calibration_Pose_25 trajectories
stored on the robot. Press [N] to move to next pose/trajectory.

Prerequisites:
  - xArm robot connected at specified IP
  - Calibration pose trajectories already recorded on robot

Usage:
  python3 traj_replay.py <robot_ip>

Example:
  python3 traj_replay.py 192.168.1.150

Controls:
  [N] - move to next trajectory
  [S] - skip current trajectory
  [Q] - quit
"""

import sys
import threading
import termios
import tty
import time
from typing import List

import sys
sys.path.insert(0, '/home/pooja/TATRO/TATRO_Program_files/TATRO_Robot_Firmware/xArm-Python-SDK')

from xarm.wrapper import XArmAPI


class TrajectoryReplay:
    """Replay calibration trajectories from robot."""

    def __init__(self, robot_ip: str):
        self.robot_ip = robot_ip

        # Connect to robot
        print(f"Connecting to robot at {robot_ip}...")
        self.arm = XArmAPI(robot_ip)
        time.sleep(1)

        # Initialize robot
        print("Initializing robot...")
        self.arm.clean_error()
        time.sleep(0.5)

        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        time.sleep(0.5)

        # Move to home position
        print("Moving to home position...")
        self.arm.move_gohome(wait=True)
        print("Home position reached.")
        time.sleep(1)

        # Get available trajectories
        print("Loading available trajectories...")
        code, trajectories = self.arm.get_trajectories()

        if code != 0:
            raise RuntimeError(f"Failed to get trajectories (code {code})")

        # Filter for Calibration_Pose trajectories and sort by number
        self._trajectories = [
            traj['name'] for traj in trajectories
            if traj['name'].startswith('Calibration_Pose_')
        ]
        self._trajectories.sort(key=lambda x: int(x.split('_')[2]))

        self._current_idx = 0
        self._running = True

        print(
            "\n" + "=" * 70 + "\n"
            f"  Calibration Trajectory Replay\n"
            f"  Robot IP: {robot_ip}\n"
            f"  Total trajectories: {len(self._trajectories)}\n"
            "  Controls: [N] next   [S] skip   [Q] quit\n"
            + "=" * 70 + "\n"
        )

        if len(self._trajectories) == 0:
            raise RuntimeError("No Calibration_Pose trajectories found on robot")

        print(f"Found {len(self._trajectories)} calibration pose trajectories:")
        for i, traj_name in enumerate(self._trajectories):
            print(f"  {i:2d}: {traj_name}")
        print()

        threading.Thread(target=self._keyboard_loop, daemon=True).start()

    def _play_trajectory(self) -> bool:
        """Play current trajectory. Returns True if successful."""
        if self._current_idx >= len(self._trajectories):
            print("All trajectories completed!")
            return False

        traj_name = self._trajectories[self._current_idx]

        print(
            f"\n[Trajectory {self._current_idx:02d}/{len(self._trajectories)-1:02d}]\n"
            f"  Name: {traj_name}\n"
        )

        try:
            print(f"Loading {traj_name}...")
            code = self.arm.load_trajectory(traj_name, wait=True, timeout=10)

            if code != 0:
                print(f"✗ Failed to load {traj_name} (code {code})")
                return False

            print(f"Playing {traj_name}...")
            code = self.arm.playback_trajectory(times=1, wait=True)

            if code == 0:
                print(f"✓ {traj_name} completed successfully.")
                print("→ Go to tf_recorder and press [S] to capture.")
                print("→ Then press [N] here to move to next trajectory.")
                time.sleep(1)
                return True
            else:
                print(f"✗ Playback failed for {traj_name} (code {code})")
                self.arm.clean_error()
                time.sleep(0.5)
                return False

        except Exception as exc:
            print(f"✗ Error: {exc}")
            self.arm.clean_error()
            time.sleep(0.5)
            return False

    def _next_trajectory(self):
        """Play next trajectory in sequence."""
        if self._current_idx >= len(self._trajectories):
            print("All trajectories completed!")
            return

        if self._play_trajectory():
            self._current_idx += 1

    def _skip_trajectory(self):
        """Skip current trajectory and move to next."""
        traj_name = self._trajectories[self._current_idx]
        print(f"Skipped {traj_name}")
        self._current_idx += 1
        if self._current_idx < len(self._trajectories):
            next_name = self._trajectories[self._current_idx]
            print(f"Ready. Press [N] to play {next_name}")

    def _keyboard_loop(self):
        """Listen for [N]ext, [S]kip, [Q]uit."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)

            # Start with first trajectory
            self._next_trajectory()

            while self._running:
                ch = sys.stdin.read(1)
                if ch.lower() == "n":
                    if self._current_idx < len(self._trajectories):
                        self._next_trajectory()
                    else:
                        print("All trajectories completed. Press [Q] to quit.")
                elif ch.lower() == "s":
                    self._skip_trajectory()
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
    if len(sys.argv) < 2:
        print("Usage: python3 traj_replay.py <robot_ip>")
        print("Example: python3 traj_replay.py 192.168.1.150")
        sys.exit(1)

    robot_ip = sys.argv[1]

    try:
        replay = TrajectoryReplay(robot_ip)

        # Keep running until user quits
        while replay._running:
            time.sleep(0.1)

        print("\nTrajectory replay completed.")

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
