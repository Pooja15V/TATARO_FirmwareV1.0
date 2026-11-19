"""
        #!/usr/bin/env python3
  
    TATARO Therapy — Level 1 grid wiring test (DIAGNOSTIC ONLY).

    Skips GRASP_OBJECT entirely -- no gripper action, nothing held. grasp_pose's
    `calibrated` flag is never read or touched by this script; it stays exactly
    as it is in the YAML. This only answers: on every button press, does the
    robot move to the next grid point, in the right direction?

    Uses the SAME Level 1 config data (config/tataro_therapy_levels.yaml) and
    the SAME shuffle-bag selection logic (therapy_tasks.grid_select.TargetSelector)
    as the real FSM, so this exercises the actual calibrated poses, not
    stand-in values -- just without the grasp step in front of them. Level 1's
    own `calibrated` flag IS still checked (must be true) -- only grasping is
    skipped, not safety.

    The first grid point is presented automatically at startup (no press
    needed, mirroring how the real FSM presents a level's first repetition
    right after grasping); each subsequent button press moves to the next one,
    after a fixed ADVANCE_DELAY_S pause (lets the patient settle/release before
    the arm moves again, and prevents a second press landing mid-pause from
    double-triggering, since callbacks block on this single-threaded node).

    Pass -p center_only:=true to target ONLY the level's verified `center` pose
    (the one pose actually jogged and confirmed by hand) instead of the full
    9-point grid -- every subsequent press just re-confirms that same pose
    (a no-op move, since move_to_pose skips axes already at target). Use this
    to validate the motion sequencing fix in isolation before trusting the
    grid corners, which were derived from the center pose's stated Y/Z bounds
    rather than each individually jogged.

    Run manually (not via ros2 launch):
        ros2 run therapy_tasks tataro_level1_grid_test
        ros2 run therapy_tasks tataro_level1_grid_test --ros-args -p center_only:=true
  

    import os
    import time

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import Bool
    import yaml

    from therapy_tasks.motion import TherapyMotion
    from therapy_tasks.grid_select import TargetSelector

    BUTTON_TOPIC = '/tataro/button_pressed'
    LEVEL_INDEX = 0  # Level 1
    ADVANCE_DELAY_S = 5.0  # pause after a press, before moving to the next pose


    def _default_config_path() -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory('therapy_tasks'),
                                'config', 'tataro_therapy_levels.yaml')
        except Exception:
            this_dir = os.path.dirname(os.path.realpath(__file__))
            return os.path.normpath(os.path.join(this_dir, '..', 'config',
                                                'tataro_therapy_levels.yaml'))


    class Level1GridTestNode(Node):
        def __init__(self):
            super().__init__('tataro_level1_grid_test')

            self.declare_parameter('center_only', False)
            self.center_only = self.get_parameter('center_only').value

            config_path = _default_config_path()
            self.get_logger().info(f"Loading therapy config: {config_path}")
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)

            self.level = cfg['levels'][LEVEL_INDEX]
            if not self.level.get('calibrated', False):
                raise RuntimeError(
                    f"Level {LEVEL_INDEX + 1} '{self.level['name']}' is not "
                    "calibrated (calibrated: false) -- refusing to move."
                )

            if self.center_only:
                if 'center' not in self.level:
                    raise RuntimeError(
                        f"Level {LEVEL_INDEX + 1} has no 'center' pose in the "
                        "YAML -- can't run with center_only:=true."
                    )
                self.selector = None
                fixed = self.level['fixed']
                center = self.level['center']
                self._center_pose = (
                    fixed['x_mm'], center['y_mm'], center['z_mm'],
                    fixed['roll_deg'], fixed['pitch_deg'], fixed['yaw_deg'],
                    f"{self.level['name']} / CENTER",
                )
            else:
                self.selector = TargetSelector(self.level)

            self.speed = cfg['motion']['speed_reach_mm_s']

            self.motion = TherapyMotion(
                robot_ip=cfg['session']['robot_ip'],
                motion_cfg=cfg['motion'],
                gripper_cfg=cfg['gripper'],
            )

            self.get_logger().warn(
                "GRASP_OBJECT is SKIPPED -- nothing is held. This only tests "
                "press -> next-grid-point motion."
            )
            if self.center_only:
                self.get_logger().warn(
                    "center_only:=true -- targeting ONLY the verified center "
                    "pose; grid points P1-P9 are not used this run."
                )

            self._last_state = False
            self._press_count = 0
            self.create_subscription(Bool, BUTTON_TOPIC, self._on_button, 10)

            self.get_logger().info("Presenting first target ...")
            self._move_to_next()
            self.get_logger().info(
                "Ready. Each button press moves to the next point in "
                f"'{self.level['name']}'."
            )

        def _move_to_next(self) -> None:
            if self.center_only:
                x_mm, y_mm, z_mm, roll, pitch, yaw, label = self._center_pose
            else:
                x_mm, y_mm, z_mm, roll, pitch, yaw, label = self.selector.next()
            self.get_logger().info(f"-> {label}  ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm")
            try:
                self.motion.move_to_pose(
                    x_mm, y_mm, z_mm, roll, pitch, yaw,
                    speed=self.speed, name=label,
                )
            except Exception as exc:
                self.get_logger().error(f"Move failed: {exc}")
            else:
                self.get_logger().info("Move OK.")

        def _on_button(self, msg: Bool) -> None:
            # Released -> Pressed edge only, same convention as fsm_node.py
            if msg.data and not self._last_state:
                self._press_count += 1
                self.get_logger().info(
                    f"Press #{self._press_count} -- waiting {ADVANCE_DELAY_S:.0f}s before advancing ..."
                )
                # Blocks this single-threaded node's callback processing for the
                # duration -- intentional, so a second press landing mid-pause
                # can't double-trigger an advance.
                time.sleep(ADVANCE_DELAY_S)
                self._move_to_next()
            self._last_state = msg.data


    def main():
        rclpy.init()
        node = None
        try:
            node = Level1GridTestNode()
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if node is not None:
                if node.motion is not None:
                    node.motion.disconnect()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


    if __name__ == '__main__':
        main()

"""

#!/usr/bin/env python3
"""
TATARO Therapy — Pose 1 -> Grasp -> Lift -> Level 1 grid test.

Sequence:
    1. Move to calibrated Pose 1 / object holder.
    2. Close the gripper.
    3. Lift vertically to Z=100 mm while keeping X/Y/RPY unchanged.
    4. Move to the first Level 1 grid point.
    5. Each subsequent button press advances to the next grid point.

The Level 1 grid uses the SAME configuration data and the SAME
TargetSelector shuffle-bag logic as the real FSM.

Safety:
    - Pose 1 is gated by grasp_pose.calibrated.
    - Level 1 is gated by level.calibrated.
    - No movement is attempted for an uncalibrated pose.
"""

import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
import yaml

from therapy_tasks.motion import TherapyMotion
from therapy_tasks.grid_select import TargetSelector


BUTTON_TOPIC = '/tataro/button_pressed'
LEVEL_INDEX = 0

ADVANCE_DELAY_S = 5.0

# ---------------------------------------------------------------------------
# Pose 1 — Approach pose
# ---------------------------------------------------------------------------

POSE1_X_MM = 250.4
POSE1_Y_MM = -209.3
POSE1_Z_MM = 263.2

POSE1_ROLL_DEG = 180.0
POSE1_PITCH_DEG = -82.1
POSE1_YAW_DEG = 0.0


# POSE1_X_MM = 268.7
# POSE1_Y_MM = 50.1
# POSE1_Z_MM = 269.9

# POSE1_ROLL_DEG = -164.4
# POSE1_PITCH_DEG = -74.6
# POSE1_YAW_DEG = 3.9

# ---------------------------------------------------------------------------
# Pose 2 — Grasp pose
# ---------------------------------------------------------------------------


# POSE2_X_MM = 273.7
# POSE2_Y_MM = 60.1
# POSE2_Z_MM = 39.9

# POSE2_ROLL_DEG = 164.4
# POSE2_PITCH_DEG = -74.6
# POSE2_YAW_DEG = 3.9

POSE2_X_MM = 256.1
POSE2_Y_MM = -221.0
POSE2_Z_MM = 34.3

POSE2_ROLL_DEG = 142.1
POSE2_PITCH_DEG = -85.4
POSE2_YAW_DEG = 36.9


# ---------------------------------------------------------------------------
# Pose 3 — Hover pose after grasp
# ---------------------------------------------------------------------------

# POSE3_X_MM = 273.7
# POSE3_Y_MM = 60.1
# POSE3_Z_MM = 197.5

# POSE3_ROLL_DEG = -164.4
# POSE3_PITCH_DEG = -74.6
# POSE3_YAW_DEG = 3.9

POSE3_X_MM = 256.1
POSE3_Y_MM = -221.0
POSE3_Z_MM = 197.5

POSE3_ROLL_DEG = 142.1
POSE3_PITCH_DEG = -85.4
POSE3_YAW_DEG = 36.9

# Desired vertical hover/lift height after grasping.
POSE1_LIFT_Z_MM = 100.0


def _default_config_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory('therapy_tasks'),
            'config',
            'tataro_therapy_levels.yaml',
        )

    except Exception:
        this_dir = os.path.dirname(os.path.realpath(__file__))

        return os.path.normpath(
            os.path.join(
                this_dir,
                '..',
                'config',
                'tataro_therapy_levels.yaml',
            )
        )


class Level1GridTestNode(Node):

    def __init__(self):

        super().__init__('tataro_level1_grid_test')

        self.declare_parameter('center_only', False)
        self.center_only = self.get_parameter('center_only').value

        # ------------------------------------------------------------------
        # Load configuration
        # ------------------------------------------------------------------

        config_path = _default_config_path()

        self.get_logger().info(
            f"Loading therapy config: {config_path}"
        )

        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        # ------------------------------------------------------------------
        # Check grasp calibration
        # ------------------------------------------------------------------

        grasp_cfg = cfg.get('grasp_pose')

        if grasp_cfg is None:
            raise RuntimeError(
                "No 'grasp_pose' section found in therapy configuration."
            )

        if not grasp_cfg.get('calibrated', False):
            raise RuntimeError(
                "grasp_pose.calibrated is false -- refusing to perform "
                "Pose 1 grasp."
            )

        # ------------------------------------------------------------------
        # Check Level 1 calibration
        # ------------------------------------------------------------------

        self.level = cfg['levels'][LEVEL_INDEX]

        if not self.level.get('calibrated', False):
            raise RuntimeError(
                f"Level {LEVEL_INDEX + 1} '{self.level['name']}' is not "
                "calibrated -- refusing to move."
            )

        # ------------------------------------------------------------------
        # Level 1 target selection
        # ------------------------------------------------------------------

        if self.center_only:

            if 'center' not in self.level:
                raise RuntimeError(
                    f"Level {LEVEL_INDEX + 1} has no 'center' pose."
                )

            self.selector = None

            fixed = self.level['fixed']
            center = self.level['center']

            self._center_pose = (
                fixed['x_mm'],
                center['y_mm'],
                center['z_mm'],
                fixed['roll_deg'],
                fixed['pitch_deg'],
                fixed['yaw_deg'],
                f"{self.level['name']} / CENTER",
            )

        else:
            self.selector = TargetSelector(self.level)

        # ------------------------------------------------------------------
        # Motion
        # ------------------------------------------------------------------

        self.speed = cfg['motion']['speed_reach_mm_s']

        self.motion = TherapyMotion(
            robot_ip=cfg['session']['robot_ip'],
            motion_cfg=cfg['motion'],
            gripper_cfg=cfg['gripper'],
        )

        self._last_state = False
        self._press_count = 0

        self.create_subscription(
            Bool,
            BUTTON_TOPIC,
            self._on_button,
            10,
        )

        # ------------------------------------------------------------------
        # Startup sequence:
        #
        # Pose 1
        #    ↓
        # Close gripper
        #    ↓
        # Lift Z to 100 mm
        #    ↓
        # Level 1 first grid point
        # ------------------------------------------------------------------

        self.get_logger().warn(
            "STARTUP SEQUENCE: Pose 1 -> CLOSE GRIPPER -> "
            "LIFT TO Z=100 mm -> LEVEL 1 GRID"
        )

        self._perform_grasp_sequence()

        self.get_logger().info(
            "Grasp/lift complete. Presenting first Level 1 target ..."
        )

        self._move_to_next()

        self.get_logger().info(
            "Ready. Each button press moves to the next point in "
            f"'{self.level['name']}'."
        )

    # ======================================================================
    # POSE 1 -> GRASP -> LIFT
    # ======================================================================

    def _perform_grasp_sequence(self) -> None:

        self.get_logger().info(
            "=================================================="
        )

        # ------------------------------------------------------------------
        # STEP 1 — Move to Pose 1 / Approach
        # ------------------------------------------------------------------

        self.get_logger().info(
            "STEP 1/4 — Moving to Pose 1 / Approach"
        )

        self.get_logger().info(
            f"Pose 1: "
            f"X={POSE1_X_MM:.1f}, "
            f"Y={POSE1_Y_MM:.1f}, "
            f"Z={POSE1_Z_MM:.1f}, "
            f"R={POSE1_ROLL_DEG:.1f}, "
            f"P={POSE1_PITCH_DEG:.1f}, "
            f"Y={POSE1_YAW_DEG:.1f}"
        )

        try:
            self.motion.move_to_pose(
                POSE1_X_MM,
                POSE1_Y_MM,
                POSE1_Z_MM,
                POSE1_ROLL_DEG,
                POSE1_PITCH_DEG,
                POSE1_YAW_DEG,
                speed=self.motion.speed_grasp(),
                name="Pose 1 / Approach",
                via_hover=False,
            )

        except Exception as exc:
            raise RuntimeError(
                f"Pose 1 move failed: {exc}"
            ) from exc

        self.get_logger().info("Pose 1 reached.")

        # ------------------------------------------------------------------
        # STEP 2 — Move to Pose 2 / Grasp
        # ------------------------------------------------------------------

        self.get_logger().info(
            "STEP 2/4 — Moving to Pose 2 / Grasp"
        )

        try:
            self.motion.move_to_pose(
                POSE2_X_MM,
                POSE2_Y_MM,
                POSE2_Z_MM,
                POSE2_ROLL_DEG,
                POSE2_PITCH_DEG,
                POSE2_YAW_DEG,
                speed=self.motion.speed_grasp(),
                name="Pose 2 / Grasp",
                via_hover=False,
            )

        except Exception as exc:
            raise RuntimeError(
                f"Pose 2 move failed: {exc}"
            ) from exc

        self.get_logger().info("Pose 2 reached.")

        # ------------------------------------------------------------------
        # STEP 3 — Close gripper
        # ------------------------------------------------------------------

        self.get_logger().info(
            "STEP 3/4 — Closing gripper..."
        )

        try:
            self.motion.close_gripper()

        except Exception as exc:
            raise RuntimeError(
                f"Gripper close failed: {exc}"
            ) from exc

        self.get_logger().info("Gripper closed.")

        # ------------------------------------------------------------------
        # STEP 4 — Move to Pose 3 / Hover
        # ------------------------------------------------------------------

        self.get_logger().info(
            "STEP 4/4 — Moving to Pose 3 / Hover"
        )

        try:
            self.motion.move_to_pose(
                POSE3_X_MM,
                POSE3_Y_MM,
                POSE3_Z_MM,
                POSE3_ROLL_DEG,
                POSE3_PITCH_DEG,
                POSE3_YAW_DEG,
                speed=self.motion.speed_grasp(),
                name="Pose 3 / Hover",
                via_hover=False,
            )

        except Exception as exc:
            raise RuntimeError(
                f"Pose 3 move failed: {exc}"
            ) from exc

        self.get_logger().info("Pose 3 reached.")

        self.get_logger().info(
            "=================================================="
        )

    # ======================================================================
    # LEVEL 1 GRID
    # ======================================================================

    def _move_to_next(self) -> None:

        if self.center_only:

            (
                x_mm,
                y_mm,
                z_mm,
                roll,
                pitch,
                yaw,
                label,
            ) = self._center_pose

        else:

            (
                x_mm,
                y_mm,
                z_mm,
                roll,
                pitch,
                yaw,
                label,
            ) = self.selector.next()

        self.get_logger().info(
            f"-> {label}  "
            f"({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm"
        )

        try:

            self.motion.move_to_pose(
                x_mm,
                y_mm,
                z_mm,
                roll,
                pitch,
                yaw,
                speed=self.speed,
                name=label,
            )

        except Exception as exc:

            self.get_logger().error(
                f"Move failed: {exc}"
            )

        else:

            self.get_logger().info(
                "Move OK."
            )

    # ======================================================================
    # BUTTON
    # ======================================================================

    def _on_button(self, msg: Bool) -> None:

        # Released -> Pressed edge only.
        if msg.data and not self._last_state:

            self._press_count += 1

            self.get_logger().info(
                f"Press #{self._press_count} -- "
                f"waiting {ADVANCE_DELAY_S:.0f}s before advancing ..."
            )

            time.sleep(ADVANCE_DELAY_S)

            self._move_to_next()

        self._last_state = msg.data


def main():

    rclpy.init()

    node = None

    try:

        node = Level1GridTestNode()

        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):

        pass

    except Exception as exc:

        if node is not None:
            node.get_logger().error(
                f"TEST FAILED: {exc}"
            )
        else:
            print(f"TEST FAILED: {exc}")

    finally:

        if node is not None:

            if node.motion is not None:
                node.motion.disconnect()

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()