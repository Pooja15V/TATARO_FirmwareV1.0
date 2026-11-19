#!/usr/bin/env python3
"""
tataro_charuco_touch — calibration-pin touch node.

Waits for the charuco_board_frame TF, computes the TCP target pose
(10 cm above the charuco origin along its local Z-axis), and moves
the robot using the xArm Python SDK.  No MoveIt Python bindings needed.

  TCP_position = charuco_origin + calibration_pin_m × Z_charuco_in_robot_frame
  TCP Z-axis   = −Z_charuco  (pin points toward the board)

Requires the full stack launched by tatro_runtime_calibrationcheck.launch.py:
  - Robot driver (ros2_control) — provides link_base TF
  - RealSense + charuco_detector — provides charuco_board_frame TF
  - This node connects to the robot simultaneously via XArmAPI (same proven
    pattern as tataro_touch_marker_centre in tatro_runtime.launch.py)

Parameters (all overridable via ros2 param or launch file):
    robot_ip           str    '192.168.1.150'
    robot_base_frame   str    'link_base'
    charuco_frame      str    'charuco_board_frame'
    calibration_pin_m  float  0.10    metres
    speed_mm_s         float  50.0    mm/s
    mvacc_mm_s2        float  100.0   mm/s²
    stable_frames      int    10
    stable_tol_m       float  0.003   metres (3 mm)
    tf_wait_timeout    float  90.0    seconds
    min_z_m            float  0.05    metres — safety floor
"""

import math
import sys
import threading
import time

import numpy as np

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    sys.exit(
        "[ERROR] xarm Python SDK not found.\n"
        "        pip install xarm-python-sdk"
    )


# ── Rotation helpers ───────────────────────────────────────────────────────────

def _quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)    ],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)    ],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _rot_to_rpy_deg(R):
    """3×3 rotation matrix → (roll, pitch, yaw) in degrees."""
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        roll  = math.atan2( R[2, 1],  R[2, 2])
        pitch = math.atan2(-R[2, 0],  sy)
        yaw   = math.atan2( R[1, 0],  R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2],  R[1, 1])
        pitch = math.atan2(-R[2, 0],  sy)
        yaw   = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


# ── Node ───────────────────────────────────────────────────────────────────────

class CharucoTouchNode(Node):

    def __init__(self):
        super().__init__('tataro_charuco_touch')

        self.declare_parameter('robot_ip',          '192.168.1.150')
        self.declare_parameter('robot_base_frame',  'link_base')
        self.declare_parameter('charuco_frame',     'charuco_board_frame')
        self.declare_parameter('calibration_pin_m', 0.10)
        self.declare_parameter('speed_mm_s',        50.0)
        self.declare_parameter('mvacc_mm_s2',       100.0)
        self.declare_parameter('stable_frames',     10)
        self.declare_parameter('stable_tol_m',      0.003)
        self.declare_parameter('tf_wait_timeout',   90.0)
        self.declare_parameter('min_z_m',           0.05)

        self.robot_ip      = self.get_parameter('robot_ip').value
        self.base_frame    = self.get_parameter('robot_base_frame').value
        self.charuco_frame = self.get_parameter('charuco_frame').value
        self.pin_m         = self.get_parameter('calibration_pin_m').value
        self.speed         = self.get_parameter('speed_mm_s').value
        self.mvacc         = self.get_parameter('mvacc_mm_s2').value
        self.stable_n      = self.get_parameter('stable_frames').value
        self.stable_tol    = self.get_parameter('stable_tol_m').value
        self.tf_timeout    = self.get_parameter('tf_wait_timeout').value
        self.min_z_m       = self.get_parameter('min_z_m').value

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'tataro_charuco_touch started — '
            f'base={self.base_frame}  charuco={self.charuco_frame}  '
            f'pin={self.pin_m*100:.0f}cm  robot={self.robot_ip}'
        )

    # ── TF helpers ─────────────────────────────────────────────────────────────

    def _wait_for_frame(self, frame):
        log = self.get_logger()
        log.info(f'Waiting for TF frame "{frame}"...')
        deadline = time.time() + self.tf_timeout
        while time.time() < deadline:
            try:
                self.tf_buffer.lookup_transform(
                    frame, frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1),
                )
                log.info(f'Frame "{frame}" is available.')
                return
            except Exception:
                time.sleep(0.5)
        raise RuntimeError(
            f'TF frame "{frame}" did not appear within {self.tf_timeout:.0f}s'
        )

    def _stable_charuco_tf(self):
        log      = self.get_logger()
        deadline = time.time() + self.tf_timeout
        history  = []
        last_q   = None
        log.info(
            f'Waiting for stable charuco pose '
            f'({self.stable_n} frames within {self.stable_tol*1000:.0f}mm)...'
        )
        while time.time() < deadline:
            try:
                tf  = self.tf_buffer.lookup_transform(
                    self.base_frame, self.charuco_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0),
                )
                tr  = tf.transform.translation
                ro  = tf.transform.rotation
                pos = np.array([tr.x, tr.y, tr.z])
                history.append(pos)
                last_q = np.array([ro.x, ro.y, ro.z, ro.w])
                if len(history) > self.stable_n:
                    history.pop(0)
                if len(history) == self.stable_n:
                    spread = float(np.max(np.std(np.array(history), axis=0)))
                    if spread < self.stable_tol:
                        mean_t = np.mean(np.array(history), axis=0)
                        log.info(f'Charuco pose stable (spread={spread*1000:.1f}mm)')
                        return mean_t, last_q
            except (LookupException, ConnectivityException, ExtrapolationException):
                history = []
            time.sleep(0.1)
        raise RuntimeError(
            f'Charuco pose did not stabilise within {self.tf_timeout:.0f}s — '
            'ensure the board is fully visible.'
        )

    # ── Target computation ─────────────────────────────────────────────────────

    def _compute_target(self, charuco_t, charuco_q):
        """
        Returns (x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg).
        TCP Z-axis = -Z_charuco so pin points toward the board.
        """
        R_c   = _quat_to_rot(charuco_q)
        z_c   = R_c[:, 2]                              # charuco +Z in robot frame
        t_tgt = charuco_t + self.pin_m * z_c           # offset by pin length

        # Build TCP rotation: approach axis (-Z_charuco), keep charuco X
        z_tcp = -z_c
        x_tcp =  R_c[:, 0]
        y_tcp =  np.cross(z_tcp, x_tcp)
        nrm   =  np.linalg.norm(y_tcp)
        if nrm < 1e-6:
            raise RuntimeError(
                'Degenerate TCP orientation — charuco X and Z are collinear.'
            )
        y_tcp /= nrm
        x_tcp  = np.cross(y_tcp, z_tcp)
        x_tcp /= np.linalg.norm(x_tcp)
        R_tcp  = np.column_stack([x_tcp, y_tcp, z_tcp])

        roll_deg, pitch_deg, yaw_deg = _rot_to_rpy_deg(R_tcp)
        x_mm = t_tgt[0] * 1000.0
        y_mm = t_tgt[1] * 1000.0
        z_mm = t_tgt[2] * 1000.0
        return x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg

    # ── Main execution sequence ─────────────────────────────────────────────────

    def run(self):
        log = self.get_logger()

        # 1. Wait for required TF frames
        log.info('═' * 56)
        log.info('  Step 1 — Waiting for TF frames')
        log.info('═' * 56)
        self._wait_for_frame(self.base_frame)
        self._wait_for_frame(self.charuco_frame)

        # 2. Get stable charuco pose
        log.info('═' * 56)
        log.info('  Step 2 — Reading stable ChArUco pose')
        log.info('═' * 56)
        charuco_t, charuco_q = self._stable_charuco_tf()
        log.info(
            f'  Charuco origin in {self.base_frame}: '
            f'x={charuco_t[0]*1000:+.1f}mm  '
            f'y={charuco_t[1]*1000:+.1f}mm  '
            f'z={charuco_t[2]*1000:+.1f}mm'
        )

        # 3. Compute TCP target
        log.info('═' * 56)
        log.info('  Step 3 — Computing TCP target')
        log.info('═' * 56)
        x_mm, y_mm, z_mm, roll, pitch, yaw = self._compute_target(charuco_t, charuco_q)
        log.info(
            f'  TCP target: '
            f'x={x_mm:+.1f}mm  y={y_mm:+.1f}mm  z={z_mm:+.1f}mm  '
            f'roll={roll:+.1f}°  pitch={pitch:+.1f}°  yaw={yaw:+.1f}°'
        )
        log.info(
            f'  Pin tip will touch charuco origin: '
            f'({charuco_t[0]*1000:+.1f}, {charuco_t[1]*1000:+.1f}, '
            f'{charuco_t[2]*1000:+.1f}) mm'
        )

        # Safety: minimum Z floor
        if z_mm / 1000.0 < self.min_z_m:
            raise RuntimeError(
                f'Target z={z_mm:.1f}mm is below the safety floor '
                f'({self.min_z_m*1000:.0f}mm). '
                'Check the charuco board position and calibration.'
            )

        # 4. Connect to robot via xArm SDK
        log.info('═' * 56)
        log.info('  Step 4 — Connecting to robot and executing motion')
        log.info('═' * 56)
        log.info(f'  Connecting to {self.robot_ip}...')
        arm = XArmAPI(self.robot_ip, is_radian=False)
        code, state = arm.get_state()
        if code != 0:
            raise RuntimeError(
                f'Cannot reach robot at {self.robot_ip} (code={code})'
            )

        # Read and log current TCP position
        _, cur_pos = arm.get_position()
        log.info(
            f'  Current TCP: '
            f'x={cur_pos[0]:+.1f}mm  y={cur_pos[1]:+.1f}mm  z={cur_pos[2]:+.1f}mm  '
            f'roll={cur_pos[3]:+.1f}°  pitch={cur_pos[4]:+.1f}°  yaw={cur_pos[5]:+.1f}°'
        )

        arm.clean_error()
        arm.clean_warn()
        arm.motion_enable(enable=True)
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.5)

        log.info(
            f'  Moving to calibration target at '
            f'speed={self.speed:.0f}mm/s  mvacc={self.mvacc:.0f}mm/s²...'
        )
        code = arm.set_position(
            x_mm, y_mm, z_mm,
            roll, pitch, yaw,
            speed=self.speed,
            mvacc=self.mvacc,
            wait=True,
        )

        if code == 0:
            _, final_pos = arm.get_position()
            log.info(
                f'  Final TCP: '
                f'x={final_pos[0]:+.1f}mm  y={final_pos[1]:+.1f}mm  '
                f'z={final_pos[2]:+.1f}mm  '
                f'roll={final_pos[3]:+.1f}°  pitch={final_pos[4]:+.1f}°  '
                f'yaw={final_pos[5]:+.1f}°'
            )
            pos_err = math.sqrt(
                (final_pos[0] - x_mm)**2 +
                (final_pos[1] - y_mm)**2 +
                (final_pos[2] - z_mm)**2
            )
            log.info(f'  Position error vs target: {pos_err:.1f}mm')
        else:
            arm.disconnect()
            raise RuntimeError(
                f'arm.set_position() failed (code={code}). '
                'Check robot state, collision limits, and reachability.'
            )

        arm.disconnect()

        log.info('═' * 56)
        log.info('  Touch completed successfully.')
        log.info('  Calibration pin tip is at the ChArUco board origin.')
        log.info('═' * 56)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CharucoTouchNode()

    executor    = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin, daemon=True, name='touch_executor'
    )
    spin_thread.start()

    exit_code = 0
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted by user.')
        exit_code = 130
    except RuntimeError as exc:
        node.get_logger().error(f'ABORTED — {exc}')
        exit_code = 1
    except Exception as exc:
        import traceback
        node.get_logger().error(f'Unexpected error: {exc}')
        node.get_logger().error(traceback.format_exc())
        exit_code = 1
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=3.0)

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
