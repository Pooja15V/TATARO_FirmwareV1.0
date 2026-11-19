#!/usr/bin/env python3
"""
TATARO Therapy — Robot motion execution module.

Wraps XArmAPI connection, gripper control, and pose moves used by the
therapy FSM. Contains no state-machine or perception logic — it only knows
how to move the arm safely between poses it is told to reach.
"""

import time

from xarm.wrapper import XArmAPI

# Workspace safety bounds for link_tcp (mm) — from TATRO_PositionControl_GUI.py.
# X was briefly widened to 480mm to admit a candidate Level 1 X of 472.0mm,
# but that value produced repeated real-hardware faults and was replaced
# with X=440.0mm (see config/tataro_therapy_levels.yaml) -- reverted to the
# original conservative box now that nothing needs the wider bound.
POSITION_LIMITS_MM = {'X': (-440.0, 440.0), 'Y': (-440.0, 440.0), 'Z': (0.0, 683.5)}


class TherapyMotion:
    def __init__(self, robot_ip: str, motion_cfg: dict, gripper_cfg: dict):
        self._speed_reach = motion_cfg['speed_reach_mm_s']
        self._speed_grasp = motion_cfg['speed_grasp_mm_s']
        self._mvacc = motion_cfg['mvacc_mm_s2']
        self._hover_mm = motion_cfg['hover_offset_mm']  # currently unused by move_to_pose (see its
        # docstring -- translate-then-rotate-then-Z replaced the old rise/transit/descend scheme
        # that used this); kept/read so existing config files don't need to drop the key
        self._gripper_cfg = gripper_cfg
        self.arm = self._connect_robot(robot_ip)

    @staticmethod
    def _connect_robot(robot_ip: str) -> XArmAPI:
        print(f"[Motion] Connecting to {robot_ip} ...")
        arm = XArmAPI(robot_ip, is_radian=False)
        code, _ = arm.get_state()
        if code != 0:
            raise RuntimeError(f"Cannot reach robot at {robot_ip} (code={code})")
        arm.clean_error()
        arm.clean_warn()
        arm.motion_enable(enable=True)
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.5)
        print("[Motion] Connected and ready.")
        return arm

    def recover(self) -> None:
        self.arm.clean_error()
        self.arm.clean_warn()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.5)

    def speed_grasp(self) -> float:
        return self._speed_grasp

    @staticmethod
    def _check_workspace(x: float, y: float, z: float, name: str) -> None:
        limits = POSITION_LIMITS_MM
        if not (limits['X'][0] <= x <= limits['X'][1] and
                limits['Y'][0] <= y <= limits['Y'][1] and
                limits['Z'][0] <= z <= limits['Z'][1]):
            raise RuntimeError(
                f"Pose '{name}' at ({x:.1f}, {y:.1f}, {z:.1f}) mm is outside the "
                f"safe workspace X{limits['X']} Y{limits['Y']} Z{limits['Z']} — "
                "refusing to move."
            )

    def move_to_pose(self, x_mm: float, y_mm: float, z_mm: float,
                      roll: float, pitch: float, yaw: float,
                      speed: float, name: str = "", via_hover: bool = True) -> None:
        """
        Move the TCP to (x_mm, y_mm, z_mm, roll, pitch, yaw).

        When via_hover is True (the staged, safe path), changes exactly ONE
        thing at a time, in this fixed order, never combining any two:
          1. X       -- at current Y, Z, orientation.
          2. Orientation (roll, pitch, yaw together, as one rotation) --
             at (target X, current Y, current Z).
          3. Z       -- at (target X, current Y, target orientation).
          4. Y       -- at (target X, target Z, target orientation) -> final pose.
        Any step already matching the current pose (within tolerance) is
        skipped entirely rather than re-sent.

        This came from two real failures combining axes: rising straight up
        at the OLD XY toward a high transit Z hit ControllerError 22
        (kinematic/unreachable) even though each axis individually passed
        the workspace-box check; translating X and Y together toward a
        grid corner (max Y + max Z) put the controller into state=4 (stop)
        mid-move. Changing one axis at a time avoids ever asking for a
        combined pose that was only ever individually verified per axis,
        not jointly. For a level's grid points (fixed X/orientation, only
        Y/Z vary), steps 1-2 become no-ops and this reduces to Z-then-Y.
        """
        self._check_workspace(x_mm, y_mm, z_mm, name)
        TOL_MM, TOL_DEG = 0.1, 0.5

        if not via_hover:
            code = self.arm.set_position(
                x_mm, y_mm, z_mm, roll, pitch, yaw,
                speed=speed, mvacc=self._mvacc, wait=True,
            )
            if code != 0:
                raise RuntimeError(f"Move to '{name}' failed (code={code})")
            return

        code, cur = self.arm.get_position()
        if code != 0:
            raise RuntimeError(f"get_position failed before '{name}' (code={code})")
        cx, cy, cz, cr, cp, cyaw = cur[0], cur[1], cur[2], cur[3], cur[4], cur[5]

        # 1. X only.
        if abs(cx - x_mm) > TOL_MM:
            self._check_workspace(x_mm, cy, cz, f"{name} (X)")
            code = self.arm.set_position(
                x_mm, cy, cz, cr, cp, cyaw,
                speed=self._speed_reach, mvacc=self._mvacc, wait=True,
            )
            if code != 0:
                raise RuntimeError(f"X-move for '{name}' failed (code={code})")

        # 2. Orientation only (still at x_mm, cy, cz).
        if (abs(cr - roll) > TOL_DEG or abs(cp - pitch) > TOL_DEG or abs(cyaw - yaw) > TOL_DEG):
            self._check_workspace(x_mm, cy, cz, f"{name} (orientation)")
            code = self.arm.set_position(
                x_mm, cy, cz, roll, pitch, yaw,
                speed=self._speed_reach, mvacc=self._mvacc, wait=True,
            )
            if code != 0:
                raise RuntimeError(f"Reorient for '{name}' failed (code={code})")

        # 3. Z only (still at x_mm, cy, target orientation).
        if abs(cz - z_mm) > TOL_MM:
            self._check_workspace(x_mm, cy, z_mm, f"{name} (Z)")
            code = self.arm.set_position(
                x_mm, cy, z_mm, roll, pitch, yaw,
                speed=speed, mvacc=self._mvacc, wait=True,
            )
            if code != 0:
                raise RuntimeError(f"Z-move for '{name}' failed (code={code})")

        # 4. Y only -> final target pose.
        if abs(cy - y_mm) > TOL_MM:
            code = self.arm.set_position(
                x_mm, y_mm, z_mm, roll, pitch, yaw,
                speed=speed, mvacc=self._mvacc, wait=True,
            )
            if code != 0:
                raise RuntimeError(f"Y-move for '{name}' failed (code={code})")

    def open_gripper(self) -> None:
        code = self.arm.open_lite6_gripper()
        if code != 0:
            raise RuntimeError(f"open_lite6_gripper failed (code={code})")
        self.arm.set_tcp_load(
            self._gripper_cfg['tcp_load_open_kg'],
            self._gripper_cfg['tcp_load_open_com_mm'],
        )
        time.sleep(self._gripper_cfg['open_wait_s'])

    def close_gripper(self) -> None:
        code = self.arm.close_lite6_gripper()
        if code != 0:
            raise RuntimeError(f"close_lite6_gripper failed (code={code})")
        self.arm.set_tcp_load(
            self._gripper_cfg['tcp_load_closed_kg'],
            self._gripper_cfg['tcp_load_closed_com_mm'],
        )
        time.sleep(self._gripper_cfg['close_wait_s'])

    def disconnect(self) -> None:
        try:
            self.arm.disconnect()
        except Exception:
            pass
