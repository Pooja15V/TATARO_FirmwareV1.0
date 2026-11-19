#!/usr/bin/env python3
"""
TATARO — Marker 40 Real-Time Tracking (V1.0)
==============================================
Phase 1 : Detect markers 51/52/53 once → lock table frame.
Phase 2 : Detection thread updates Marker 40 position every frame.
           Initial approach: hover → descend to TOUCH_Z_MM.
           Tracking loop: whenever the XY position changes by more than
           DEADBAND_CM, the robot TCP moves to the new position.
           The robot follows the marker continuously as it is moved.
           No user input needed. Press Ctrl+C to exit.

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
from TATARO_CV_MarkerDetection_7x7          import detect_marker_7x7
from TATARO_CV_TableCropping_with_ARUCO_7x7 import detect_and_transform_table_using_aruco
from TATARO_CV_Pipeline                     import marker40_pose_in_robot_frame

# ── Configuration ─────────────────────────────────────────────────────────────
ROBOT_IP        = '192.168.1.150'
TOUCH_Z_MM      = 200.0   # TCP Z (mm) at marker surface — verified with tf2_echo
                           # SDK_Z = TF(link_base→link_tcp)_Z + 85 mm (pin offset)
                           # Safe TF Z floor: 100 mm → SDK floor: 185 mm
HOVER_MM        = 50.0    # mm above TOUCH_Z_MM for initial safe approach
SPEED_APPROACH  = 500.0    # mm/s for initial hover move
SPEED_TOUCH     = 20.0    # mm/s for initial descent to contact
SPEED_TRACK     = 1000.0    # mm/s for continuous tracking moves
MVACC_APPROACH  = 1000.0   # mm/s² for initial approach
MVACC_TRACK     = 5000.0   # mm/s² for tracking (snappy response)
MIN_Z_MM        = 200.0   # safety floor — must equal TOUCH_Z_MM
DEADBAND_CM          = 0.3    # 3 mm minimum XY displacement to send a new move command
ORIENT_DEADBAND_DEG  = 3.0    # degrees — ignore yaw residuals smaller than this
STABLE_N        = 10      # frames used to compute the [STABLE] display tag
STABLE_TOL_CM   = 0.5     # max per-axis std-dev for [STABLE] tag (~5 mm)
INIT_TIMEOUT    = 60.0    # seconds to find markers 51/52/53 at startup


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
    Daemon thread: detects Marker 40 every frame using the frozen table frame.

    shared['latest_pos'] — updated every detected frame (used by tracking loop).
    shared['stable_pos'] — updated only when STABLE_N consecutive frames agree
                           within STABLE_TOL_CM (used for the display tag only).

    Tags:
      [STABLE]  stable_pos is valid — readings are consistent
      [CV]      marker detected but not yet stable
    """
    rvec_marker_wrt_camera = frozen['rvec_marker_wrt_camera']
    Table_corner_3D        = frozen['Table_corner_3D']
    history: list          = []

    while not stop.is_set():
        color_frame, depth_frame = get_frame(pipe, return_depth=True)
        if color_frame is None or depth_frame is None:
            time.sleep(0.02)
            continue

        rvec_40, tvec_40 = detect_marker_7x7(color_frame, 40, depth_frame)
        if tvec_40 is None:
            history.clear()
            with shared['lock']:
                shared['latest_pos'] = None
                shared['stable_pos'] = None
            continue

        pos, _ = marker40_pose_in_robot_frame(
            rvec_40, tvec_40, Table_corner_3D, rvec_marker_wrt_camera
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
            shared['latest_pos'] = pos.copy()
            shared['stable_pos'] = stable.copy() if stable is not None else None

        tag = '[STABLE]' if stable is not None else '[CV]   '
        print(f"{tag}  pos_40_robot = {np.round(pos.ravel(), 2)}", flush=True)


def _detect_loop_poseANDorientation(pipe, frozen: dict, shared: dict,
                                     stop: threading.Event) -> None:
    """
    Orientation-aware version of _detect_loop.

    In addition to shared['latest_pos'] and shared['stable_pos'], also writes:
      shared['latest_rvec'] — rvec_40_robot (Rodrigues, 3-element) updated every
                              detected frame; None when marker is not visible.

    Use this thread (instead of _detect_loop) when calling
    _tracking_loop_poseANDorientation.
    """
    rvec_marker_wrt_camera = frozen['rvec_marker_wrt_camera']
    Table_corner_3D        = frozen['Table_corner_3D']
    history: list          = []

    while not stop.is_set():
        color_frame, depth_frame = get_frame(pipe, return_depth=True)
        if color_frame is None or depth_frame is None:
            time.sleep(0.02)
            continue

        rvec_40, tvec_40 = detect_marker_7x7(color_frame, 40, depth_frame)
        if tvec_40 is None:
            history.clear()
            with shared['lock']:
                shared['latest_pos']  = None
                shared['latest_rvec'] = None
                shared['stable_pos']  = None
            continue

        pos, rvec = marker40_pose_in_robot_frame(
            rvec_40, tvec_40, Table_corner_3D, rvec_marker_wrt_camera
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
        print(f"{tag}  pos_40_robot = {np.round(pos.ravel(), 2)}", flush=True)


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
    Move to hover above the first stable marker position, then descend to
    TOUCH_Z_MM.  Returns (roll, pitch, yaw) read from the arm before moving —
    these are held constant throughout the tracking session.
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
    print(f"[ARM]  Target XY    : ({x_mm:.1f}, {y_mm:.1f}) mm")
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
    print("\n[ARM]  Tracking started — move Marker 40 to follow.\n")
    print("─" * 60)

    return roll, pitch, yaw


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2 — Real-time tracking loop (main thread)
# ─────────────────────────────────────────────────────────────────────────────

def _tracking_loop(arm: XArmAPI, shared: dict, stop: threading.Event,
                   roll: float, pitch: float, yaw: float) -> None:
    """
    Continuously moves the TCP to follow Marker 40.

    Uses shared['latest_pos'] (updated every camera frame) so tracking responds
    immediately when the marker moves, without waiting for stability.

    wait=True ensures one command is active at a time — no queue buildup.
    When the current move completes, the freshest position is read and a new
    move issued if the XY delta exceeds DEADBAND_CM.
    """
    last_xy = None
    no_marker_warned = False

    while not stop.is_set():
        with shared['lock']:
            pos       = shared['latest_pos']
            is_stable = shared['stable_pos'] is not None

        if pos is None:
            if not no_marker_warned:
                print("[TRACK]  Marker 40 not visible — holding position.", flush=True)
                no_marker_warned = True
            time.sleep(0.05)
            continue

        no_marker_warned = False
        xy = pos[:2].copy()

        if last_xy is not None and np.linalg.norm(xy - last_xy) < DEADBAND_CM:
            time.sleep(0.02)
            continue

        x_mm = float(pos[0]) * 10.0
        y_mm = float(pos[1]) * 10.0

        code = arm.set_position(
            x_mm, y_mm, TOUCH_Z_MM, roll, pitch, yaw,
            speed=SPEED_TRACK, mvacc=MVACC_TRACK, wait=True,
        )

        if code == 0:
            last_xy = xy
            tag = '[STABLE]' if is_stable else '[TRACK] '
            print(f"{tag}  TCP → ({x_mm:.1f}, {y_mm:.1f}, {TOUCH_Z_MM:.1f}) mm", flush=True)
        else:
            print(f"[WARN]   Move failed (code={code}) — recovering ...", flush=True)
            _recover(arm)


def _compute_yaw_residual(rvec_40_robot: np.ndarray) -> float:
    """
    Extract the marker's in-plane rotation in the robot frame and return
    the residual after removing multiples of 90°.

    Method:
      1. Convert Rodrigues rvec → R_40_robot (3×3).
      2. Project the marker X-axis onto the robot XY plane:
             theta = atan2(R[1,0], R[0,0])
      3. Reduce modulo 90° → [0°, 90°).
      4. Normalise to [-45°, 45°) so the robot always takes the shortest
         rotation to the nearest 90° reference axis.

    0°/90°/180°/270° all map to 0° — no rotation commanded for those.
    Returns residual in degrees, range [-45°, 45°).
    """
    R, _ = cv2.Rodrigues(rvec_40_robot.reshape(3, 1))
    theta_deg = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    theta_mod = theta_deg % 90.0          # → [0°, 90°)
    if theta_mod > 45.0:
        theta_mod -= 90.0                 # → [-45°, 45°)
    return theta_mod


def _tracking_loop_poseANDorientation(arm: XArmAPI, shared: dict,
                                       stop: threading.Event,
                                       roll: float, pitch: float,
                                       initial_yaw: float) -> None:
    """
    Orientation-aware version of _tracking_loop.

    Position : XY tracks shared['latest_pos'] with DEADBAND_CM (identical to
               _tracking_loop).
    Orientation:
      - Reads shared['latest_rvec'] every iteration.
      - Computes the in-plane yaw residual via _compute_yaw_residual().
      - 0°, 90°, 180°, 270° → residual ≈ 0° → no yaw change.
      - |residual| < ORIENT_DEADBAND_DEG → treated as zero (deadband).
      - Otherwise: desired_yaw = initial_yaw + residual.
      - Roll and pitch remain fixed throughout.

    A combined move is issued whenever either the XY delta ≥ DEADBAND_CM
    or the yaw command would change by ≥ ORIENT_DEADBAND_DEG.

    Requires _detect_loop_poseANDorientation to be running as the detection
    thread (so that shared['latest_rvec'] is populated).
    """
    last_xy       = None
    last_yaw_cmd  = initial_yaw
    no_marker_warned = False

    while not stop.is_set():
        with shared['lock']:
            pos       = shared['latest_pos']
            rvec      = shared['latest_rvec']
            is_stable = shared['stable_pos'] is not None

        if pos is None or rvec is None:
            if not no_marker_warned:
                print("[TRACK]  Marker 40 not visible — holding position.", flush=True)
                no_marker_warned = True
            time.sleep(0.05)
            continue

        no_marker_warned = False
        xy = pos[:2].copy()

        # ── Yaw residual ─────────────────────────────────────────────────────
        theta_residual = _compute_yaw_residual(rvec)
        if abs(theta_residual) < ORIENT_DEADBAND_DEG:
            theta_residual = 0.0          # within deadband — treat as aligned
        desired_yaw = initial_yaw + theta_residual

        # ── Check if any motion is needed ─────────────────────────────────────
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
    print("  TATARO — Marker 40 Real-Time Tracking (V1.0)")
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
            'latest_pos':  None,   # updated every detected frame — used for tracking
            'latest_rvec': None,   # rvec_40_robot — populated by _detect_loop_poseANDorientation
            'stable_pos':  None,   # updated only when STABLE_N frames agree — display tag
        }

        # det_thread = threading.Thread(
        #     target=_detect_loop,
        #     args=(pipe, frozen, shared, stop_event),
        #     daemon=True,
        # )
        det_thread = threading.Thread(target=_detect_loop_poseANDorientation,
            args=(pipe, frozen, shared, stop_event),
            daemon=True,
        )
        det_thread.start()

        # ── Wait for first stable position before approaching ─────────────────
        print("\n[Phase 2]  Waiting for first stable Marker 40 detection ...")
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
        # _tracking_loop(arm, shared, stop_event, roll, pitch, yaw)
        _tracking_loop_poseANDorientation(arm, shared, stop_event, roll, pitch, yaw)

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
