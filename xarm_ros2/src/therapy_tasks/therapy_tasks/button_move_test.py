#!/usr/bin/env python3
"""
TATARO Therapy — Button-to-motion wiring test (DIAGNOSTIC ONLY).

NOT part of the therapy session and NOT related to config/tataro_therapy_levels.yaml
in any way. Only answers one question: does every button press actually move
the robot? On every Released->Pressed edge on /tataro/button_pressed
(published by either button_monitor.py [BLE] or keyboard_button_monitor.py
['n' key]), nudges the TCP up/down by a small fixed amount at whatever XY it
is already at when this script starts. No gripper action, no table poses, no
YAML config, no `calibrated` safety gate to worry about.

Safety:
  - Only ever moves in Z, at a fixed XY read once at startup (whatever pose
    the arm is already resting at).
  - Bump size and speed are deliberately small/slow (see BUMP_MM, SPEED_MM_S).
  - Uses the same workspace bounds check as therapy_tasks.motion.

Run manually (not via ros2 launch):

"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from xarm.wrapper import XArmAPI

ROBOT_IP = '192.168.1.150'
BUTTON_TOPIC = '/tataro/button_pressed'
BUMP_MM = 40.0          # small, fixed vertical nudge per press
SPEED_MM_S = 80.0       # slow -- this is a wiring test, not a therapy move
MVACC_MM_S2 = 500.0

# Same bounds as therapy_tasks.motion.POSITION_LIMITS_MM
POSITION_LIMITS_MM = {'X': (-440.0, 440.0), 'Y': (-440.0, 440.0), 'Z': (0.0, 683.5)}


def _check_workspace(x: float, y: float, z: float) -> None:
    limits = POSITION_LIMITS_MM
    if not (limits['X'][0] <= x <= limits['X'][1] and
            limits['Y'][0] <= y <= limits['Y'][1] and
            limits['Z'][0] <= z <= limits['Z'][1]):
        raise RuntimeError(
            f"Target ({x:.1f}, {y:.1f}, {z:.1f}) mm is outside the safe workspace "
            f"X{limits['X']} Y{limits['Y']} Z{limits['Z']} -- refusing to move."
        )


class ButtonMoveTestNode(Node):
    def __init__(self):
        super().__init__('tataro_button_move_test')

        self.get_logger().info(f"Connecting to {ROBOT_IP} ...")
        self.arm = XArmAPI(ROBOT_IP, is_radian=False)
        code, _ = self.arm.get_state()
        if code != 0:
            raise RuntimeError(f"Cannot reach robot at {ROBOT_IP} (code={code})")
        self.arm.clean_error()
        self.arm.clean_warn()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)

        code, cur = self.arm.get_position()
        if code != 0:
            raise RuntimeError(f"get_position failed (code={code})")
        self.x, self.y, self.z_base = cur[0], cur[1], cur[2]
        self.roll, self.pitch, self.yaw = cur[3], cur[4], cur[5]
        self.up = False  # toggles each press: base <-> base + BUMP_MM

        _check_workspace(self.x, self.y, self.z_base)
        _check_workspace(self.x, self.y, self.z_base + BUMP_MM)

        self.get_logger().info(
            f"Starting pose: ({self.x:.1f}, {self.y:.1f}, {self.z_base:.1f}) mm. "
            f"Each button press nudges Z by {BUMP_MM:.0f} mm at this fixed XY."
        )

        self._last_state = False
        self._press_count = 0
        self.create_subscription(Bool, BUTTON_TOPIC, self._on_button, 10)

    def _on_button(self, msg: Bool) -> None:
        # Released -> Pressed edge only, same convention as fsm_node.py
        if msg.data and not self._last_state:
            self._press_count += 1
            self.up = not self.up
            z = self.z_base + (BUMP_MM if self.up else 0.0)
            self.get_logger().info(f"Press #{self._press_count} -> moving to Z={z:.1f} mm")
            code = self.arm.set_position(
                self.x, self.y, z, self.roll, self.pitch, self.yaw,
                speed=SPEED_MM_S, mvacc=MVACC_MM_S2, wait=True,
            )
            if code != 0:
                self.get_logger().error(f"Move failed (code={code})")
            else:
                self.get_logger().info("Move OK.")
        self._last_state = msg.data


def main():
    rclpy.init()
    node = None
    try:
        node = ButtonMoveTestNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.arm.disconnect()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
