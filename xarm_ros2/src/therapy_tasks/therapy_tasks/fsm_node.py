#!/usr/bin/env python3
"""
TATARO Therapy — Progressive Reach finite state machine.

States
------
  INIT_TABLE       Verify table markers 51/52/53 once, then connect to the arm.
  GRASP_OBJECT     Move to the fixed grasp pose, close the gripper, lift.
  MOVE_TO_LEVEL    Present the held object at this repetition's target pose
                   (a level's fixed `pose_mm_deg`, or -- for a `grid` level --
                   a point drawn at random from the grid with no repeats
                   until every point has been used once).
  WAIT_FOR_BUTTON  Hold position until a Released->Pressed edge is seen, then
                   count one completed repetition. Repeats (re-entering
                   MOVE_TO_LEVEL for the next target) until `reps_target`
                   (config, default session.reps_per_level) repetitions are
                   done, then advances.
  ADVANCE_LEVEL    Move to the next level, or SESSION_COMPLETE if none left.
  SESSION_COMPLETE Hold the final pose, or restart from level 1 (config).

Perception (therapy_tasks.perception), robot motion
(therapy_tasks.motion.TherapyMotion) and button monitoring
(button_monitor.py, a separate node) are kept as separate modules; this node
only sequences them according to the FSM above. All poses and timing come
from config/tataro_therapy_levels.yaml — therapists can retune the session
without touching this file.

Calibration is gated per pose (see the YAML's SAFETY note), not globally --
`grasp_pose.calibrated` and each level's own `calibrated` flag are checked
right before that pose would be used, so an individually-verified level can
run while everything still marked PLACEHOLDER stays hard-blocked.
"""

import os
from enum import Enum, auto

import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from therapy_tasks.perception import verify_table_frame
from therapy_tasks.motion import TherapyMotion
from therapy_tasks.grid_select import TargetSelector

BUTTON_TOPIC = '/tataro/button_pressed'
STATE_TOPIC = '/tataro/therapy_state'


def _default_config_path() -> str:
    """
    Prefer the installed share/ copy (ament_index); fall back to the
    source-tree config/ next to this package, so the node also works when
    run straight from a colcon --symlink-install workspace or an uninstalled
    checkout during development.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('therapy_tasks'),
                             'config', 'tataro_therapy_levels.yaml')
    except Exception:
        this_dir = os.path.dirname(os.path.realpath(__file__))
        return os.path.normpath(os.path.join(this_dir, '..', 'config',
                                              'tataro_therapy_levels.yaml'))


class State(Enum):
    INIT_TABLE = auto()
    GRASP_OBJECT = auto()
    MOVE_TO_LEVEL = auto()
    WAIT_FOR_BUTTON = auto()
    ADVANCE_LEVEL = auto()
    SESSION_COMPLETE = auto()


class TherapyFSMNode(Node):
    def __init__(self):
        super().__init__('tataro_therapy_fsm')

        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value or _default_config_path()
        self.get_logger().info(f"Loading therapy config: {config_path}")
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self._button_pressed_event = False
        self._last_button_state = False
        self.create_subscription(Bool, BUTTON_TOPIC, self._on_button, 10)
        self._state_pub = self.create_publisher(String, STATE_TOPIC, 10)

        self.level_idx = 0
        self.motion: TherapyMotion | None = None
        self.state = State.INIT_TABLE
        self._reset_level_progress()

    def _reset_level_progress(self) -> None:
        """Called on entering a level for the first time (grasp -> level 1,
        advance -> next level, or session restart)."""
        self.rep_count = 0
        self._selector: TargetSelector | None = None  # built lazily in _next_target()
        self._last_pose: tuple | None = None  # last (x,y,z,roll,pitch,yaw) actually commanded

    # ── Button edge detection ───────────────────────────────────────────────

    def _on_button(self, msg: Bool) -> None:
        # Released -> Pressed edge only; repeated "pressed" notifications
        # while already pressed (or "released" notifications) are ignored.
        if msg.data and not self._last_button_state:
            self._button_pressed_event = True
        self._last_button_state = msg.data

    def _publish_state(self, extra: str = "") -> None:
        text = self.state.name + (f" — {extra}" if extra else "")
        self.get_logger().info(text)
        self._state_pub.publish(String(data=text))

    # ── State handlers ──────────────────────────────────────────────────────

    def _do_init_table(self) -> None:
        session_cfg = self.cfg['session']
        verify_table_frame(timeout_s=session_cfg.get('table_init_timeout_s', 60.0))
        self.motion = TherapyMotion(
            robot_ip=session_cfg['robot_ip'],
            motion_cfg=self.cfg['motion'],
            gripper_cfg=self.cfg['gripper'],
        )
        self.state = State.GRASP_OBJECT

    def _do_grasp_object(self) -> None:
        grasp = self.cfg['grasp_pose']
        name = grasp.get('name', 'grasp_pose')
        if not grasp.get('calibrated', False):
            raise RuntimeError(
                f"SAFETY GATE: grasp_pose '{name}' is not calibrated "
                "(calibrated: false). Verify pose_mm_deg on the real robot "
                "before grasping."
            )
        x_mm, y_mm, z_mm, roll, pitch, yaw = grasp['pose_mm_deg']

        self._publish_state(f"grasping '{name}'")
        self.motion.open_gripper()
        self.motion.move_to_pose(
            x_mm, y_mm, z_mm, roll, pitch, yaw,
            speed=self.motion.speed_grasp(), name=name,
        )
        self.motion.close_gripper()

        lift_mm = grasp.get('lift_z_offset_mm', 80.0)
        self.motion.move_to_pose(
            x_mm, y_mm, z_mm + lift_mm, roll, pitch, yaw,
            speed=self.motion.speed_grasp(), name="grasp_lift", via_hover=False,
        )
        self._reset_level_progress()
        self.state = State.MOVE_TO_LEVEL

    def _next_target(self, level: dict) -> tuple:
        """
        Return (x_mm, y_mm, z_mm, roll, pitch, yaw, label) for this
        repetition, via the shared TargetSelector (therapy_tasks.grid_select)
        so this logic can't drift from what standalone diagnostics use.
        """
        if self._selector is None:
            self._selector = TargetSelector(level)
        return self._selector.next()

    def _do_move_to_level(self) -> None:
        levels = self.cfg['levels']
        level = levels[self.level_idx]
        if not level.get('calibrated', False):
            raise RuntimeError(
                f"SAFETY GATE: level {self.level_idx + 1} '{level['name']}' is "
                "not calibrated (calibrated: false). Verify its pose(s) on the "
                "real robot before presenting it."
            )

        x_mm, y_mm, z_mm, roll, pitch, yaw, label = self._next_target(level)
        pose = (x_mm, y_mm, z_mm, roll, pitch, yaw)

        if pose != self._last_pose:
            self._publish_state(
                f"moving to level {self.level_idx + 1}/{len(levels)}: {label}"
            )
            self.motion.move_to_pose(
                x_mm, y_mm, z_mm, roll, pitch, yaw,
                speed=self.cfg['motion']['speed_reach_mm_s'], name=label,
            )
            self._last_pose = pose
        else:
            self._publish_state(f"holding position for repetition at: {label}")

        self.state = State.WAIT_FOR_BUTTON

    def _do_wait_for_button(self) -> None:
        """
        Wait for a Released->Pressed edge, then count it as one completed
        repetition. Repeats (re-entering MOVE_TO_LEVEL, which picks the next
        grid point for `grid` levels or re-holds the same pose for
        single-pose levels) until `reps_target` repetitions are done.
        """
        levels = self.cfg['levels']
        level = levels[self.level_idx]
        reps_target = level.get('reps', self.cfg['session'].get('reps_per_level', 10))

        self._button_pressed_event = False
        self._publish_state(
            f"waiting for press {self.rep_count + 1}/{reps_target} at level "
            f"{self.level_idx + 1}: {level['name']}"
        )
        while rclpy.ok() and not self._button_pressed_event:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.rep_count += 1
        self.get_logger().info(
            f"Repetition {self.rep_count}/{reps_target} completed at level "
            f"{self.level_idx + 1}: {level['name']}"
        )

        if self.rep_count >= reps_target:
            self.state = State.ADVANCE_LEVEL
        else:
            self.state = State.MOVE_TO_LEVEL

    def _do_advance_level(self) -> None:
        levels = self.cfg['levels']
        self.level_idx += 1
        self._reset_level_progress()
        if self.level_idx >= len(levels):
            self.state = State.SESSION_COMPLETE
        else:
            self.state = State.MOVE_TO_LEVEL

    def _do_session_complete(self) -> None:
        on_complete = self.cfg['session'].get('on_complete', 'hold')
        self._publish_state(f"session complete ({on_complete})")
        if on_complete == 'restart':
            self.level_idx = 0
            self._reset_level_progress()
            self.state = State.MOVE_TO_LEVEL
        else:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.5)

    # ── Main loop ────────────────────────────────────────────────────────────

    def run(self) -> None:
        handlers = {
            State.INIT_TABLE: self._do_init_table,
            State.GRASP_OBJECT: self._do_grasp_object,
            State.MOVE_TO_LEVEL: self._do_move_to_level,
            State.WAIT_FOR_BUTTON: self._do_wait_for_button,
            State.ADVANCE_LEVEL: self._do_advance_level,
            State.SESSION_COMPLETE: self._do_session_complete,
        }
        while rclpy.ok():
            self._publish_state()
            handlers[self.state]()


def main():
    rclpy.init()
    node = TherapyFSMNode()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        node.get_logger().error(f"Therapy session aborted: {exc}")
        raise
    finally:
        if node.motion is not None:
            node.motion.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
