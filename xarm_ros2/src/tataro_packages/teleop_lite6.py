#!/usr/bin/env python3
"""
TATARO phone/VR teleoperation for the Lite6, adapted from:
https://github.com/SpesRobotics/teleop (teleop/xarm/__main__.py)

Run standalone (not a ROS2 node):
    python3 teleop_lite6.py

Then open the printed https://<this-machine-LAN-IP>:4443 URL on a
WebXR-capable phone browser (XR Browser on Android / WebXR Viewer on iOS),
on the same Wi-Fi network as this machine.

SAFETY: motion_enable() + set_state(state=0) below make the arm live the
moment this script runs, before any phone connects. Keep the E-stop within
reach for the first run and be ready for full-speed cartesian jumps if the
phone/controller moves fast.
"""

import numpy as np
from teleop import Teleop
import time
import transforms3d as t3d

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    raise ImportError(
        "xarm-python-sdk is not installed. Please install the dependency with `pip install xarm-python-sdk`."
    )

# TATARO Lite6 IP (see ReadMeV1.0) — the upstream teleop repo hardcodes
# 192.168.1.184 as a placeholder; ours is 192.168.1.150.
ROBOT_IP = "192.168.1.150"

# --- Safety clamp -----------------------------------------------------------
# On 2026-09-09 a WebXR tracking glitch on the phone produced a commanded pose
# ~8m away from the workspace (x=-7975mm, y=1192mm, z=4448mm), which tripped
# an uncleared error on the controller (code=1 / HAS_ERROR) and spammed
# rejected set_servo_cartesian calls. The Lite6's own controller refused the
# out-of-range command, but this clamp stops that garbage from ever reaching
# the arm in the first place, regardless of what the phone sends.
#
# Per-update (i.e. per WebSocket message, up to ~100Hz) limits on how far the
# commanded pose may move relative to the *last accepted* pose. Generous
# enough for deliberate fast phone motion, tight enough to catch a
# multi-meter tracking-loss jump.
MAX_TRANSLATION_JUMP_M = 0.15  # 150mm per update
MAX_ROTATION_JUMP_RAD = np.deg2rad(60)
REJECT_LOG_INTERVAL_S = 1.0  # rate-limit console spam if glitches persist


def pose_jump(pose_a, pose_b):
    """Translation (m) and rotation (rad) distance between two 4x4 poses."""
    translation_diff = np.linalg.norm(pose_b[:3, 3] - pose_a[:3, 3])
    r = pose_a[:3, :3].T @ pose_b[:3, :3]
    trace = np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)
    rotation_diff = np.arccos(trace)
    return translation_diff, rotation_diff


class Lite6Gripper:
    def __init__(self, arm: XArmAPI):
        self._arm = arm
        self._prev_gripper_state = None
        self._gripper_state = 0.0
        self._gripper_open_time = 0.0
        self._gripper_stopped = False

    def open(self):
        self._arm.open_lite6_gripper()

    def close(self):
        self._arm.close_lite6_gripper()

    def stop(self):
        self._arm.stop_lite6_gripper()

    def set_gripper_state(self, gripper_state: float) -> None:
        """Set gripper state and handle opening/closing logic."""
        self._gripper_state = gripper_state

        if self._gripper_state is not None:
            if (
                self._prev_gripper_state is None
                or self._prev_gripper_state != self._gripper_state
            ):
                if self._gripper_state < 1.0:
                    self._gripper_stopped = False
                    self.close()
                else:
                    self._gripper_open_time = time.time()
                    self._gripper_stopped = False
                    self.open()

            if (
                not self._gripper_stopped
                and self._gripper_state >= 1.0
                and time.time() - self._gripper_open_time > 1.0
            ):
                # If gripper was closed and now is open, stop the gripper
                self._gripper_stopped = True
                self.stop()

        self._prev_gripper_state = self._gripper_state

    def get_gripper_state(self) -> float:
        """Get current gripper state."""
        return self._gripper_state

    def reset_gripper(self) -> None:
        """Reset gripper state."""
        self._prev_gripper_state = None
        self._gripper_state = 0.0
        self._gripper_open_time = 0.0
        self._gripper_stopped = False


def get_pose(arm):
    ok, pose = arm.get_position()
    if ok != 0:
        raise RuntimeError(f"Failed to get arm position: {ok}")

    translation = np.array(pose[:3]) / 1000
    eulers = np.array(pose[3:])
    rotation = t3d.euler.euler2mat(eulers[0], eulers[1], eulers[2], "sxyz")
    pose = t3d.affines.compose(translation, rotation, np.ones(3))
    return pose


def servo(arm, pose):
    x = pose[0, 3] * 1000
    y = pose[1, 3] * 1000
    z = pose[2, 3] * 1000
    roll, pitch, yaw = t3d.euler.mat2euler(pose[:3, :3])
    error = arm.set_servo_cartesian([x, y, z, roll, pitch, yaw], speed=100, mvacc=100)
    return error


def main():
    arm = XArmAPI(ROBOT_IP, is_radian=True)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(1)
    arm.set_state(state=0)

    gripper = Lite6Gripper(arm)

    last_safe_pose = get_pose(arm)
    last_reject_log_time = 0.0

    def callback(pose, message):
        nonlocal last_safe_pose, last_reject_log_time

        translation_diff, rotation_diff = pose_jump(last_safe_pose, pose)
        if translation_diff > MAX_TRANSLATION_JUMP_M or rotation_diff > MAX_ROTATION_JUMP_RAD:
            now = time.time()
            if now - last_reject_log_time > REJECT_LOG_INTERVAL_S:
                print(
                    f"[SAFETY] Rejected pose jump: "
                    f"translation={translation_diff*1000:.1f}mm "
                    f"(max {MAX_TRANSLATION_JUMP_M*1000:.0f}mm), "
                    f"rotation={np.degrees(rotation_diff):.1f}deg "
                    f"(max {np.degrees(MAX_ROTATION_JUMP_RAD):.0f}deg) "
                    f"-- holding last safe pose. Check phone tracking."
                )
                last_reject_log_time = now
            return  # hold last_safe_pose, don't command the arm this frame

        servo(arm, pose)
        last_safe_pose = pose

        gripper_state = 1.0 if message['gripper'] == "close" else 0.0
        gripper.set_gripper_state(gripper_state)

    teleop = Teleop(natural_phone_orientation_euler=[0, 0, 0])
    teleop.set_pose(get_pose(arm))
    teleop.subscribe(callback)
    teleop.run()


if __name__ == "__main__":
    main()
