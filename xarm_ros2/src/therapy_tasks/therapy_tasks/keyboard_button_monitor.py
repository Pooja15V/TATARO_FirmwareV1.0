#!/usr/bin/env python3
"""
TATARO Therapy — Keyboard stand-in for the BLE button monitor.

Publishes the exact same std_msgs/Bool on /tataro/button_pressed as
button_monitor.py (the real XIAO-TATARO BLE device), but driven by pressing
the 'n' key at this terminal instead of the physical button. Useful for
bench-testing the FSM (fsm_node.py) when the BLE board isn't powered on /
within range, without touching any FSM or motion code — it only needs *a*
node publishing on this topic, it doesn't care which one.

Only run ONE of button_monitor.py / keyboard_button_monitor.py at a time —
running both would double-publish and could register extra repetitions.

NOTE: like tataro_marker40_touch_loop.py, this reads raw terminal input, and
`ros2 launch` does not route stdin to subprocesses. Do NOT add this to
tataro_therapy_session.launch.py — run it manually in its own terminal:

    ros2 run therapy_tasks tataro_keyboard_button_monitor

Press 'n' to simulate a button press. Press 'q' or Ctrl+C to quit.
"""

import sys
import termios
import threading
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

BUTTON_TOPIC = '/tataro/button_pressed'
PRESS_KEY = 'n'
QUIT_KEY = 'q'
RELEASE_DELAY_S = 0.15  # how long the "pressed" pulse stays True before dropping back to False


class KeyboardButtonMonitorNode(Node):
    def __init__(self):
        super().__init__('tataro_keyboard_button_monitor')
        self._pub = self.create_publisher(Bool, BUTTON_TOPIC, 10)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_keys, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"Publishing on {BUTTON_TOPIC} — press '{PRESS_KEY}' to simulate a button "
            f"press, '{QUIT_KEY}' or Ctrl+C to quit."
        )

    def _publish(self, pressed: bool) -> None:
        self._pub.publish(Bool(data=pressed))

    def _simulate_press(self) -> None:
        self._publish(True)
        self.get_logger().info("Simulated button PRESS")
        threading.Timer(RELEASE_DELAY_S, self._publish, args=(False,)).start()

    def _read_keys(self) -> None:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._stop.is_set() and rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == PRESS_KEY:
                    self._simulate_press()
                elif ch == QUIT_KEY:
                    self._stop.set()
                    rclpy.shutdown()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def stop(self) -> None:
        self._stop.set()


def main():
    rclpy.init()
    node = KeyboardButtonMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
