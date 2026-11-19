#!/usr/bin/env python3
"""
tatro_calibration_poses.py  v1.0
──────────────────────────────────────────────────────────────────────────────
Moves the xArm Lite6 through 15 pre-computed eye-in-hand calibration poses
and integrates with EasyHandEye2 for automated sample collection.

Works with both simulation (Gazebo) and the real robot.

15-pose strategy
─────────────────
Group 1 (P01–P03) : straight top-down, 3 different wrist angles
                     → captures rotational diversity around approach axis
Group 2 (P04–P07) : 30° off-vertical, 4 cardinal azimuths
                     → translational diversity + mild tilt
Group 3 (P08–P11) : 45° off-vertical, 4 cardinal azimuths
                     → larger tilt for better rotational conditioning
Group 4 (P12–P15) : diagonal 35° tilt + mixed wrist rotation
                     → combined rotational and translational diversity

All positions expressed in link_base frame.

Prerequisites
─────────────
  pip install scipy
  sudo apt install ros-humble-moveit-py
  sudo apt install ros-humble-easy-handeye2      (optional)

Usage
──────
  # Dry-run: print pose table, no motion
  ros2 run tatro_handeye_sim tatro_calibration_poses \
      --ros-args -p dry_run:=true

  # Full calibration run
  ros2 run tatro_handeye_sim tatro_calibration_poses

  # Adjust settle time and calibration name
  ros2 run tatro_handeye_sim tatro_calibration_poses \
      --ros-args -p settle_time_s:=3.0 -p handeye_name:=tataro_handeye_3
"""

import sys
import time
import math

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

try:
    from easy_handeye2_msgs.srv import TakeSample, ComputeCalibration, SaveCalibration
    _EHE2 = True
except ImportError:
    _EHE2 = False

try:
    from moveit.planning import MoveItPy
    _MOVEIT_PY = True
except ImportError:
    _MOVEIT_PY = False

# ═══════════════════════════════════════════════════════════════════════════
# WORKSPACE CONSTANTS  (from workspace_params.yaml)
# ═══════════════════════════════════════════════════════════════════════════
MARKER_POS_LB = np.array([0.335, 0.000, -0.005])   # table centre, link_base frame
PLANNING_GROUP = 'lite6'
EEF_LINK       = 'link_tcp'
BASE_FRAME     = 'link_base'
VELOCITY_SCALING     = 0.20
ACCELERATION_SCALING = 0.15
DEFAULT_HANDEYE_NAME = 'tataro_handeye_3'

# D435i colour specs (for visibility estimates)
D435I_FOV_H_DEG = 69.4
D435I_IMG_W_PX  = 1280
D435I_IMG_H_PX  = 720
_FX = (D435I_IMG_W_PX / 2.0) / math.tan(math.radians(D435I_FOV_H_DEG / 2.0))

# ═══════════════════════════════════════════════════════════════════════════
# POSE DEFINITIONS
#   (camera_offset_from_marker, wrist_deg, label, description)
#
#   camera_offset : displacement in link_base from MARKER_POS_LB to
#                   the desired camera / EEF position
#   wrist_deg     : additional rotation around the approach axis
# ═══════════════════════════════════════════════════════════════════════════
POSE_DEFS = [
    # ── Group 1: Top-down (θ=0°) — vary wrist for rotational conditioning ──
    (np.array([ 0.000,  0.000,  0.350]),   0.0, "P01", "top_down_wrist_0deg"),
    (np.array([ 0.000,  0.000,  0.350]),  60.0, "P02", "top_down_wrist_+60deg"),
    (np.array([ 0.000,  0.000,  0.400]), -60.0, "P03", "top_down_high_wrist_-60deg"),

    # ── Group 2: 30° off-vertical, 4 cardinal azimuths ─────────────────────
    (np.array([-0.150,  0.000,  0.260]),   0.0, "P04", "tilt30_toward_robot"),
    (np.array([ 0.150,  0.000,  0.260]),   0.0, "P05", "tilt30_away_from_robot"),
    (np.array([ 0.000,  0.150,  0.260]),   0.0, "P06", "tilt30_robot_right"),
    (np.array([ 0.000, -0.150,  0.260]),   0.0, "P07", "tilt30_robot_left"),

    # ── Group 3: 45° off-vertical, 4 cardinal azimuths ─────────────────────
    (np.array([-0.198,  0.000,  0.198]),   0.0, "P08", "tilt45_toward_robot"),
    (np.array([ 0.198,  0.000,  0.198]),   0.0, "P09", "tilt45_away_from_robot"),
    (np.array([ 0.000,  0.198,  0.198]),   0.0, "P10", "tilt45_robot_right"),
    (np.array([ 0.000, -0.198,  0.198]),   0.0, "P11", "tilt45_robot_left"),

    # ── Group 4: Diagonal 35° tilt + wrist rotation ─────────────────────────
    (np.array([-0.114,  0.114,  0.229]),  45.0, "P12", "diag_NW_wrist_+45deg"),
    (np.array([-0.114, -0.114,  0.229]), -45.0, "P13", "diag_SW_wrist_-45deg"),
    (np.array([ 0.114,  0.114,  0.229]),  30.0, "P14", "diag_NE_wrist_+30deg"),
    (np.array([ 0.114, -0.114,  0.229]), -30.0, "P15", "diag_SE_wrist_-30deg"),
]


# ═══════════════════════════════════════════════════════════════════════════
# GEOMETRY HELPERS
# ═══════════════════════════════════════════════════════════════════════════

def compute_eef_pose(camera_offset: np.ndarray, wrist_deg: float):
    """
    Returns (position, quaternion_xyzw) for the EEF such that the EEF z-axis
    (camera optical axis) points from the camera position toward the marker.

    Assumption: camera optical z-axis ≈ link_tcp z-axis.
    If your physical mounting has a fixed offset, pre-multiply the inverse of
    that transform.

    Args:
        camera_offset: displacement from MARKER_POS_LB (m) in link_base frame
        wrist_deg:     additional rotation around approach axis (degrees)

    Returns:
        position (3,), quaternion [x, y, z, w]
    """
    camera_pos = MARKER_POS_LB + camera_offset

    z_ax = MARKER_POS_LB - camera_pos
    z_ax = z_ax / np.linalg.norm(z_ax)

    # Build orthonormal frame: project world-X onto the plane ⊥ to z_ax
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(ref, z_ax))) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    x_ax = ref - np.dot(ref, z_ax) * z_ax
    x_ax /= np.linalg.norm(x_ax)
    y_ax  = np.cross(z_ax, x_ax)

    R = np.column_stack([x_ax, y_ax, z_ax])

    w_rad = math.radians(wrist_deg)
    Rw = np.array([
        [ math.cos(w_rad), -math.sin(w_rad), 0.0],
        [ math.sin(w_rad),  math.cos(w_rad), 0.0],
        [0.0, 0.0, 1.0],
    ])
    R_final = R @ Rw

    rot = Rotation.from_matrix(R_final)
    return camera_pos, rot.as_quat()  # [x, y, z, w]


def estimate_visibility(camera_offset: np.ndarray) -> dict:
    dist = float(np.linalg.norm(camera_offset))
    if dist < 1e-6:
        return {'dist_m': 0.0, 'tilt_deg': 0.0, 'coverage_pct': 100.0, 'in_fov': False}

    z_comp   = abs(float(camera_offset[2]))
    tilt_deg = math.degrees(math.acos(min(z_comp / dist, 1.0)))

    apparent_size = 0.15 * math.cos(math.radians(tilt_deg))
    px_side = _FX * apparent_size / dist
    coverage_pct = (px_side ** 2) / (D435I_IMG_W_PX * D435I_IMG_H_PX) * 100.0

    half_angle_h = math.degrees(math.atan2(0.075, dist))   # 0.075 = half marker size
    in_fov = half_angle_h < D435I_FOV_H_DEG / 2.0

    return {
        'dist_m':       round(dist, 3),
        'tilt_deg':     round(tilt_deg, 1),
        'coverage_pct': round(coverage_pct, 1),
        'in_fov':       in_fov,
    }


def make_pose_stamped(position: np.ndarray, q_xyzw: np.ndarray,
                      frame_id: str) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x    = float(position[0])
    ps.pose.position.y    = float(position[1])
    ps.pose.position.z    = float(position[2])
    ps.pose.orientation.x = float(q_xyzw[0])
    ps.pose.orientation.y = float(q_xyzw[1])
    ps.pose.orientation.z = float(q_xyzw[2])
    ps.pose.orientation.w = float(q_xyzw[3])
    return ps


def print_pose_table(logger=None):
    sep = '═' * 96
    header = (
        f"\n{sep}\n"
        f"  TATRO Eye-in-Hand Calibration — 15 Poses  (link_base frame)\n"
        f"  Marker: {MARKER_POS_LB}  |  D435i FOV {D435I_FOV_H_DEG}° H  "
        f"res {D435I_IMG_W_PX}×{D435I_IMG_H_PX}\n"
        f"  Vis target: 10–40% image area\n"
        f"{'─'*96}\n"
        f"  {'#':<4} {'Label':<4} {'x(m)':>7} {'y(m)':>7} {'z(m)':>7} "
        f"{'Wrist°':>7} {'Dist m':>7} {'Tilt°':>6} {'Cov%':>5} "
        f"{'qx':>8} {'qy':>8} {'qz':>8} {'qw':>8} {'FOV':>4}"
    )
    lines = [header]

    for offset, wrist, label, desc in POSE_DEFS:
        pos, q = compute_eef_pose(offset, wrist)
        vis = estimate_visibility(offset)
        fov = '✓' if vis['in_fov'] else '✗'
        lines.append(
            f"  {label:<4} {label:<4} "
            f"{pos[0]:>7.3f} {pos[1]:>7.3f} {pos[2]:>7.3f} "
            f"{wrist:>7.1f} {vis['dist_m']:>7.3f} "
            f"{vis['tilt_deg']:>6.1f} {vis['coverage_pct']:>4.1f}% "
            f"{q[0]:>8.4f} {q[1]:>8.4f} {q[2]:>8.4f} {q[3]:>8.4f} {fov:>4}"
        )

    lines.append(sep)
    table = '\n'.join(lines)
    if logger:
        logger.info(table)
    else:
        print(table)
    return table


# ═══════════════════════════════════════════════════════════════════════════
# ROS 2 NODE
# ═══════════════════════════════════════════════════════════════════════════

class CalibrationRunner(Node):

    def __init__(self):
        super().__init__('tatro_calibration_poses')
        self.declare_parameter('dry_run',       False)
        self.declare_parameter('settle_time_s', 2.0)
        self.declare_parameter('handeye_name',  DEFAULT_HANDEYE_NAME)

        self._dry_run  = self.get_parameter('dry_run').value
        self._settle_s = self.get_parameter('settle_time_s').value
        self._ehe_name = self.get_parameter('handeye_name').value

        if _EHE2:
            self._svc_take    = self.create_client(TakeSample,
                                    f'/{self._ehe_name}/take_sample')
            self._svc_compute = self.create_client(ComputeCalibration,
                                    f'/{self._ehe_name}/compute_calibration')
            self._svc_save    = self.create_client(SaveCalibration,
                                    f'/{self._ehe_name}/save_calibration')

    def take_sample(self) -> bool:
        if self._dry_run:
            self.get_logger().info('  [DRY-RUN] take_sample skipped')
            return True
        if not _EHE2:
            self.get_logger().warn('  easy_handeye2_msgs not installed — sample skipped')
            return False
        if not self._svc_take.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'  /{self._ehe_name}/take_sample not available')
            return False
        fut = self._svc_take.call_async(TakeSample.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.result() is None:
            self.get_logger().error('  take_sample call timed out')
            return False
        self.get_logger().info(f'  Sample taken: {fut.result()}')
        return True

    def compute_and_save(self):
        if self._dry_run or not _EHE2:
            return
        if self._svc_compute.wait_for_service(timeout_sec=5.0):
            fut = self._svc_compute.call_async(ComputeCalibration.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            if fut.result():
                self.get_logger().info(f'Calibration result:\n{fut.result()}')
        if self._svc_save.wait_for_service(timeout_sec=5.0):
            fut = self._svc_save.call_async(SaveCalibration.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            self.get_logger().info('Calibration saved.')


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationRunner()

    print_pose_table(node.get_logger())

    if node._dry_run:
        node.get_logger().info('[DRY-RUN] Pose table complete — no motion executed.')
        node.destroy_node()
        rclpy.shutdown()
        return

    if not _MOVEIT_PY:
        node.get_logger().fatal(
            'moveit_py not installed.\n'
            '  sudo apt install ros-humble-moveit-py\n'
            'Use --ros-args -p dry_run:=true to inspect poses only.'
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info('Initialising MoveItPy...')
    moveit = MoveItPy(node_name='tatro_cal_moveit')
    arm    = moveit.get_planning_component(PLANNING_GROUP)

    def go_home():
        arm.set_start_state_to_current_state()
        arm.set_goal_state(configuration_name='home')
        plan = arm.plan()
        if plan:
            moveit.execute(plan.trajectory, controllers=[])
            time.sleep(1.5)
        else:
            node.get_logger().warn('Home plan failed.')

    node.get_logger().info('Moving to home...')
    go_home()

    success_count = 0
    failed_poses  = []

    for i, (offset, wrist_deg, label, desc) in enumerate(POSE_DEFS):
        pos, q = compute_eef_pose(offset, wrist_deg)
        vis    = estimate_visibility(offset)

        node.get_logger().info(
            f'\n── Pose {label} ({i+1}/15): {desc}\n'
            f'   EEF  ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m  '
            f'wrist={wrist_deg:+.0f}°\n'
            f'   dist={vis["dist_m"]:.3f} m  tilt={vis["tilt_deg"]:.1f}°  '
            f'coverage≈{vis["coverage_pct"]:.1f}%'
        )

        target = make_pose_stamped(pos, q, BASE_FRAME)
        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=target, pose_link=EEF_LINK)

        plan = arm.plan()
        if plan is None:
            node.get_logger().warn(f'   ✗ Planning FAILED for {label} — skipping.')
            failed_poses.append(label)
            continue

        if not moveit.execute(plan.trajectory, controllers=[]):
            node.get_logger().warn(f'   ✗ Execution FAILED for {label} — skipping.')
            failed_poses.append(label)
            continue

        time.sleep(node._settle_s)

        if node.take_sample():
            success_count += 1
            node.get_logger().info(f'   ✓ Sample {success_count} collected ({label})')

        time.sleep(0.3)

    node.get_logger().info(
        f'\n{"═"*60}\n'
        f'  Calibration collection complete\n'
        f'  Successful samples : {success_count} / 15\n'
        + (f'  Failed poses       : {", ".join(failed_poses)}\n' if failed_poses else '')
        + f'{"═"*60}'
    )

    if success_count >= 5:
        node.get_logger().info('Computing and saving calibration...')
        node.compute_and_save()
    else:
        node.get_logger().warn(
            f'Only {success_count} samples — minimum 5 required.'
        )

    node.get_logger().info('Returning to home...')
    go_home()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
