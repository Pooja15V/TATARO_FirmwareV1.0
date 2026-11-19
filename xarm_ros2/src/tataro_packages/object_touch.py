#!/usr/bin/env python3
"""
TATARO — Cube Object Real-Time Tracking & Touch
================================================
Phase 1 : Detect markers 51/52/53 once → lock table frame.
Phase 2 : Detection thread updates cube centre every frame
          (DICT_4X4, ID=1, 3.5 cm cube, 3.0 cm printed marker).
          Initial approach: hover → descend to TOUCH_Z_MM.
          Tracking loop: whenever the XY position changes by more than
          DEADBAND_CM, the robot TCP moves to the new position.
          Press Ctrl+C to exit.

All units: CV pipeline → cm  |  xArm SDK → mm.
"""

import os
import sys
import time
import threading

import cv2
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_CV_DIR   = os.path.normpath(os.path.join(_THIS_DIR, '..', 'computer_vision'))
sys.path.insert(0, _CV_DIR)

from xarm.wrapper import XArmAPI
from TATARO_CV_ROS2Camera                   import connect_camera, stop_camera, get_frame
from TATARO_CV_TableCropping_with_ARUCO_7x7 import detect_and_transform_table_using_aruco
from track_cube                             import detect_cube_faces, cube_pose_in_robot_frame

# ── Configuration ─────────────────────────────────────────────────────────────
ROBOT_IP       = '192.168.1.150'

# SDK Z (mm) when the flange contacts the cube.
# No calibrator pin — SDK_Z = TF(link_base→link_tcp)_Z directly.
#
# Measured manually (tf2_echo link_base link_tcp while holding cube):
#   TF Z = 0.019 m = 19 mm ≈ 20 mm  →  TOUCH_Z_MM = 20 mm
#
# Verify: run  ros2 run tf2_ros tf2_echo link_base link_tcp  during a touch
# move and confirm TF Z ≈ 0.020 m.  Adjust in 2 mm steps if needed.
TOUCH_Z_MM     = 20.0

HOVER_MM       = 50.0    # mm above TOUCH_Z_MM for safe initial approach
SPEED_APPROACH = 500.0   # mm/s for initial hover move
SPEED_TOUCH    = 20.0    # mm/s for initial descent to contact
SPEED_TRACK    = 1000.0  # mm/s for continuous tracking moves
MVACC_APPROACH = 1000.0  # mm/s²
MVACC_TRACK    = 5000.0  # mm/s²
MIN_Z_MM       = 15.0    # safety floor: 5 mm below TOUCH_Z_MM (no pin — flange clearance)

DEADBAND_CM         = 0.3   # 3 mm min XY displacement before issuing a new move
ORIENT_DEADBAND_DEG = 3.0   # ignore yaw residuals smaller than this

STABLE_N      = 10    # consecutive frames needed for [STABLE] tag
STABLE_TOL_CM = 0.5   # max per-axis std-dev across STABLE_N frames (~5 mm)
INIT_TIMEOUT  = 60.0  # seconds to find markers 51/52/53 at startup


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1 — Table frame initialisation (runs once)
# ─────────────────────────────────────────────────────────────────────────────

def _init_table_frame(pipe) -> dict:
    """
    Block until markers 51/52/53 are detected and the table frame is locked.
    Returns a dict with rvec_marker_wrt_camera and Table_corner_3D.
    Raises RuntimeError on timeout.
    """
    print(f"[Phase 1] Searching for markers 51, 52, 53  (timeout {INIT_TIMEOUT:.0f} s) ...")
    deadline = time.time() + INIT_TIMEOUT

    while time.time() < deadline:
        color_frame, depth_frame = get_frame(pipe, return_depth=True)
        if color_frame is None or depth_frame is None:
            time.sleep(0.05)
            continue

        result = detect_and_transform_table_using_aruco(color_frame.copy(), depth_frame)
        if result is None:
            continue

        (_, _, _,
         _, _, _, _,
         rvec_marker_wrt_camera, _,
         _, Table_corner_3D) = result

        print("[Phase 1] Table frame locked.")
        return {
            'rvec_marker_wrt_camera': rvec_marker_wrt_camera,
            'Table_corner_3D':        Table_corner_3D,
        }

    raise RuntimeError(
        f"Markers 51/52/53 not found within {INIT_TIMEOUT:.0f} s. "
        "Ensure all three markers are visible to the camera."
    )


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2 — Background detection thread
# ─────────────────────────────────────────────────────────────────────────────

def _detect_loop(pipe, frozen: dict, shared: dict, stop: threading.Event) -> None:
    """
    Daemon thread: detects the cube (ID=1, DICT_4X4) every frame.

    shared['latest_pos']  — cube centre XYZ (cm, robot frame), updated every frame.
    shared['latest_rvec'] — Rodrigues rotation of the best visible face in robot frame.
    shared['stable_pos']  — updated only when STABLE_N consecutive frames agree
                            within STABLE_TOL_CM; used for [STABLE] display tag.

    Tags:
      [STABLE]  stable_pos is valid — readings are consistent
      [CV]      cube detected but not yet stable
    """
    rvec_marker_wrt_camera = frozen['rvec_marker_wrt_camera']
    Table_corner_3D        = frozen['Table_corner_3D']
    history: list          = []

    while not stop.is_set():
        color_frame, depth_frame = get_frame(pipe, return_depth=True)
        if color_frame is None or depth_frame is None:
            time.sleep(0.02)
            continue

        faces = detect_cube_faces(color_frame, depth_frame)

        if not faces:
            history.clear()
            with shared['lock']:
                shared['latest_pos']  = None
                shared['latest_rvec'] = None
                shared['stable_pos']  = None
            continue

        pos, rvec, euler = cube_pose_in_robot_frame(
            faces, Table_corner_3D, rvec_marker_wrt_camera
        )

        history.append(pos.copy())
        if len(history) > STABLE_N:
            history.pop(0)

        stable = None
        if len(history) == STABLE_N:
            spread = float(np.max(np.std(np.array(history), axis=0)))
            if spread < STABLE_TOL_CM:
                stable = np.mean(np.array(history), axis=0)

        with shared['lock']:
            shared['latest_pos']  = pos.copy()
            shared['latest_rvec'] = rvec.copy()
            shared['stable_pos']  = stable.copy() if stable is not None else None

        tag = '[STABLE]' if stable is not None else '[CV]   '
        print(
            f"{tag}  cube_robot = {np.round(pos.ravel(), 2)} cm  "
            f"Yaw = {euler[2]:.1f}°  Faces = {len(faces)}",
            flush=True,
        )


# ─────────────────────────────────────────────────────────────────────────────
# Robot helpers
# ─────────────────────────────────────────────────────────────────────────────

def _connect_robot() -> XArmAPI:
    print(f"[ARM]  Connecting to {ROBOT_IP} ...")
    arm = XArmAPI(ROBOT_IP, is_radian=False)
    code, _ = arm.get_state()
    if code != 0:
        raise RuntimeError(f"Cannot reach robot at {ROBOT_IP} (code={code})")
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)
    print("[ARM]  Connected and ready.")
    return arm


def _recover(arm: XArmAPI) -> None:
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)


def _initial_approach(arm: XArmAPI, pos_cm: np.ndarray) -> tuple:
    """
    Move to hover above the first stable cube position, then descend to
    TOUCH_Z_MM.  Returns (roll, pitch, yaw) read from the arm — held constant
    throughout the tracking session.
    """
    x_mm    = float(pos_cm[0]) * 10.0
    y_mm    = float(pos_cm[1]) * 10.0
    z_hover = TOUCH_Z_MM + HOVER_MM

    if TOUCH_Z_MM < MIN_Z_MM:
        raise RuntimeError(
            f"TOUCH_Z_MM = {TOUCH_Z_MM:.1f} mm is below safety floor "
            f"({MIN_Z_MM:.0f} mm). Adjust TOUCH_Z_MM."
        )

    _recover(arm)
    _, cur = arm.get_position()
    roll, pitch, yaw = cur[3], cur[4], cur[5]

    print(f"[ARM]  Current TCP  : ({cur[0]:.1f}, {cur[1]:.1f}, {cur[2]:.1f}) mm  "
          f"RPY = ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)")
    print(f"[ARM]  Target XY    : ({x_mm:.1f}, {y_mm:.1f}) mm  (cube centre)")
    print(f"[ARM]  Hover Z      : {z_hover:.1f} mm")
    print(f"[ARM]  Touch Z      : {TOUCH_Z_MM:.1f} mm  (fixed — manual calibration)")

    print("\n[ARM]  ── Initial hover ─────────────────────────────────────────")
    code = arm.set_position(x_mm, y_mm, z_hover, roll, pitch, yaw,
                            speed=SPEED_APPROACH, mvacc=MVACC_APPROACH, wait=True)
    if code != 0:
        raise RuntimeError(f"Hover move failed (code={code})")
    print("[ARM]  Hover reached.")

    print("\n[ARM]  ── Descend to contact ────────────────────────────────────")
    code = arm.set_position(x_mm, y_mm, TOUCH_Z_MM, roll, pitch, yaw,
                            speed=SPEED_TOUCH, mvacc=MVACC_APPROACH, wait=True)
    if code != 0:
        raise RuntimeError(f"Touch move failed (code={code})")

    _, final = arm.get_position()
    print(f"[ARM]  Contact reached — TCP: "
          f"({final[0]:.1f}, {final[1]:.1f}, {final[2]:.1f}) mm")
    print("\n[ARM]  Tracking started — move the cube to follow.\n")
    print("─" * 60)

    return roll, pitch, yaw


# ─────────────────────────────────────────────────────────────────────────────
# Yaw helper
# ─────────────────────────────────────────────────────────────────────────────

def _compute_yaw_residual(rvec_robot: np.ndarray) -> float:
    """
    Extract the cube face's in-plane yaw in the robot frame and return the
    residual after removing multiples of 90° (cube has 4-fold symmetry).

    Steps:
      1. rvec → R (3×3)
      2. theta = atan2(R[1,0], R[0,0])  — face X-axis angle in robot XY plane
      3. theta mod 90° → [0°, 90°)
      4. Normalise to [-45°, 45°) — robot always takes the shortest rotation

    Returns residual in degrees, range [-45°, 45°).
    """
    R, _ = cv2.Rodrigues(rvec_robot.reshape(3, 1))
    theta_deg = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    theta_mod = theta_deg % 90.0
    if theta_mod > 45.0:
        theta_mod -= 90.0
    return theta_mod


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2 — Real-time tracking loop (main thread)
# ─────────────────────────────────────────────────────────────────────────────

def _tracking_loop(arm: XArmAPI, shared: dict, stop: threading.Event,
                   roll: float, pitch: float, initial_yaw: float) -> None:
    """
    Continuously moves the TCP to follow the cube centre.

    Position  : XY tracks shared['latest_pos'] whenever delta ≥ DEADBAND_CM.
    Orientation: yaw residual (mod 90°) applied on top of initial_yaw;
                 residuals < ORIENT_DEADBAND_DEG treated as zero.

    wait=True ensures one move is active at a time (no queue buildup).
    """
    last_xy      = None
    last_yaw_cmd = initial_yaw
    no_cube_warned = False

    while not stop.is_set():
        with shared['lock']:
            pos       = shared['latest_pos']
            rvec      = shared['latest_rvec']
            is_stable = shared['stable_pos'] is not None

        if pos is None or rvec is None:
            if not no_cube_warned:
                print("[TRACK]  Cube not visible — holding position.", flush=True)
                no_cube_warned = True
            time.sleep(0.05)
            continue

        no_cube_warned = False
        xy = pos[:2].copy()

        # ── Yaw residual ──────────────────────────────────────────────────────
        theta_residual = _compute_yaw_residual(rvec)
        if abs(theta_residual) < ORIENT_DEADBAND_DEG:
            theta_residual = 0.0
        desired_yaw = initial_yaw + theta_residual

        # ── Check if motion is needed ─────────────────────────────────────────
        xy_changed  = last_xy is None or np.linalg.norm(xy - last_xy) >= DEADBAND_CM
        yaw_changed = abs(desired_yaw - last_yaw_cmd) >= ORIENT_DEADBAND_DEG

        if not xy_changed and not yaw_changed:
            time.sleep(0.02)
            continue

        x_mm = float(pos[0]) * 10.0
        y_mm = float(pos[1]) * 10.0

        code = arm.set_position(
            x_mm, y_mm, TOUCH_Z_MM, roll, pitch, desired_yaw,
            speed=SPEED_TRACK, mvacc=MVACC_TRACK, wait=True,
        )

        if code == 0:
            last_xy      = xy
            last_yaw_cmd = desired_yaw
            tag = '[STABLE]' if is_stable else '[TRACK] '
            print(
                f"{tag}  TCP → ({x_mm:.1f}, {y_mm:.1f}, {TOUCH_Z_MM:.1f}) mm  "
                f"yaw = {desired_yaw:.1f}°  (residual = {theta_residual:+.1f}°)",
                flush=True,
            )
        else:
            print(f"[WARN]   Move failed (code={code}) — recovering ...", flush=True)
            _recover(arm)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("\n" + "═" * 60)
    print("  TATARO — Cube Object Real-Time Tracking & Touch")
    print("═" * 60)

    pipe       = connect_camera()
    arm        = None
    stop_event = threading.Event()

    try:
        # ── Phase 1: lock table frame ─────────────────────────────────────────
        frozen = _init_table_frame(pipe)

        arm = _connect_robot()

        shared: dict = {
            'lock':       threading.Lock(),
            'latest_pos':  None,   # cube centre XYZ (cm) — updated every detected frame
            'latest_rvec': None,   # rvec of best visible face in robot frame
            'stable_pos':  None,   # set only when STABLE_N frames agree within STABLE_TOL_CM
        }

        det_thread = threading.Thread(
            target=_detect_loop,
            args=(pipe, frozen, shared, stop_event),
            daemon=True,
        )
        det_thread.start()

        # ── Wait for first stable cube position before approaching ────────────
        print("\n[Phase 2]  Waiting for first stable cube detection ...")
        while not stop_event.is_set():
            with shared['lock']:
                first_pos = shared['stable_pos']
            if first_pos is not None:
                break
            time.sleep(0.1)

        print(f"[Phase 2]  First stable position: {np.round(first_pos, 2)} cm")

        # ── Initial approach ──────────────────────────────────────────────────
        roll, pitch, yaw = _initial_approach(arm, first_pos)

        # ── Phase 2: real-time tracking ───────────────────────────────────────
        _tracking_loop(arm, shared, stop_event, roll, pitch, yaw)

    except KeyboardInterrupt:
        print("\n[Exit]  Ctrl+C received — shutting down ...")

    finally:
        stop_event.set()
        stop_camera(pipe)
        if arm is not None:
            arm.disconnect()
            print("[ARM]   Disconnected.")
        print("[Exit]  Done.\n")


if __name__ == '__main__':
    main()
