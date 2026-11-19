#!/usr/bin/env python3
"""
Replay calibration trajectories via ROS 2 xArm driver services.

Uses the existing xArm ROS 2 driver's playback_trajectory service.
Works seamlessly with MoveIt and TF recording without conflicts.

Prerequisites:
  - xarm_moveit_config launched with real robot
  - Calibration pose trajectories stored on robot memory
  - ROS 2 environment initialized

Usage:
  ros2 run tf_recorder traj_replay_ros2

Controls:
  [N] - play next trajectory
  [S] - skip current trajectory
  [Q] - quit
"""

import sys
import threading
import termios
import tty
import time
from typing import List

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import TrajPlay


class TrajectoryReplayROS2(Node):
    """Replay calibration trajectories via ROS 2 services."""

    def __init__(self):
        super().__init__('traj_replay_ros2')

        # Get available trajectories from xArm driver
        self._trajectories = [
            f'Calibration_Pose_{i:02d}' for i in range(1, 26)
        ]

        self._current_idx = 0
        self._running = True
        self._service_ready = False

        # Create client for playback_trajectory service
        self.playback_client = self.create_client(
            TrajPlay, '/playback_trajectory'
        )

        self.get_logger().info(
            "\n" + "=" * 70 + "\n"
            f"  Calibration Trajectory Replay (ROS 2)\n"
            f"  Total trajectories: {len(self._trajectories)}\n"
            "  Controls: [N] next   [S] skip   [Q] quit\n"
            + "=" * 70 + "\n"
        )

        print(f"Found {len(self._trajectories)} calibration pose trajectories:")
        for i, traj_name in enumerate(self._trajectories):
            print(f"  {i:2d}: {traj_name}")
        print()

    def _wait_for_service(self):
        """Wait for service, should be called after node starts spinning."""
        self.get_logger().info("Waiting for /playback_trajectory service...")
        max_wait_time = 60  # 60 seconds total
        start_time = time.time()

        while not self._service_ready:
            if self.playback_client.wait_for_service(timeout_sec=0.5):
                self._service_ready = True
                self.get_logger().info("✓ Service /playback_trajectory found!")
                break

            elapsed = time.time() - start_time
            if elapsed > max_wait_time:
                raise RuntimeError(
                    f"Service /playback_trajectory not found after {max_wait_time}s. "
                    "Make sure MoveIt launch is running with xArm driver initialized."
                )
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                self.get_logger().info(f"Service not ready, waiting... ({int(elapsed)}s elapsed)")
            time.sleep(0.1)

    def _play_trajectory(self) -> bool:
        """Play current trajectory via ROS 2 service. Returns True if successful."""
        if self._current_idx >= len(self._trajectories):
            self.get_logger().info("All trajectories completed!")
            return False

        traj_name = self._trajectories[self._current_idx]

        self.get_logger().info(
            f"\n[Trajectory {self._current_idx:02d}/{len(self._trajectories)-1:02d}]\n"
            f"  Name: {traj_name}\n"
        )

        try:
            # Create service request
            request = TrajPlay.Request()
            request.filename = traj_name
            request.times = 1
            request.double_speed = 1
            request.wait = True  # Wait for execution to complete

            self.get_logger().info(f"Playing {traj_name} via ROS 2 service...")

            # Call service
            future = self.playback_client.call_async(request)

            # Wait for result (with timeout)
            start_time = time.time()
            timeout = 60  # seconds
            while not future.done():
                if time.time() - start_time > timeout:
                    self.get_logger().error(f"Trajectory playback timeout for {traj_name}")
                    return False
                time.sleep(0.1)

            response = future.result()

            if response.ret == 0:
                self.get_logger().info(f"✓ {traj_name} completed successfully.")
                self.get_logger().info(
                    "→ Go to tf_recorder and press [S] to capture.\n"
                    "→ Then press [N] here to move to next trajectory."
                )
                time.sleep(1)
                return True
            else:
                self.get_logger().error(
                    f"✗ Playback failed for {traj_name} (ret={response.ret}): {response.message}"
                )
                return False

        except Exception as exc:
            self.get_logger().error(f"✗ Service call error: {exc}")
            return False

    def _next_trajectory(self):
        """Play next trajectory in sequence."""
        if self._current_idx >= len(self._trajectories):
            self.get_logger().info("All trajectories completed!")
            return

        if self._play_trajectory():
            self._current_idx += 1

    def _skip_trajectory(self):
        """Skip current trajectory and move to next."""
        traj_name = self._trajectories[self._current_idx]
        self.get_logger().info(f"Skipped {traj_name}")
        self._current_idx += 1
        if self._current_idx < len(self._trajectories):
            next_name = self._trajectories[self._current_idx]
            self.get_logger().info(f"Ready. Press [N] to play {next_name}")

    def _keyboard_loop(self):
        """Listen for [N]ext, [S]kip, [Q]uit."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)

            # Wait for service to be ready (with node spinning)
            self._wait_for_service()

            # Start with first trajectory
            self._next_trajectory()

            while self._running and rclpy.ok():
                ch = sys.stdin.read(1)
                if ch.lower() == "n":
                    if self._current_idx < len(self._trajectories):
                        self._next_trajectory()
                    else:
                        self.get_logger().info("All trajectories completed. Press [Q] to quit.")
                elif ch.lower() == "s":
                    self._skip_trajectory()
                elif ch.lower() == "q" or ch == "\x03":
                    self.get_logger().info("Quit key received.")
                    self._running = False
                    break
        except Exception as exc:
            self.get_logger().error(f"Keyboard error: {exc}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TrajectoryReplayROS2()

        # Start keyboard loop in daemon thread (will wait for service internally)
        threading.Thread(target=node._keyboard_loop, daemon=True).start()

        # Keep node spinning to allow ROS 2 discovery to work
        while node._running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

        print("\nTrajectory replay completed.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
