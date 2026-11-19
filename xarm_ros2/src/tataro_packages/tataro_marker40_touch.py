#!/usr/bin/env python3
"""
TATARO — Marker 40 Touch
==========================
1. Connects to the RealSense via ROS2 (camera driver must already be running)
2. Detects markers 51/52/53 + Marker 40 and waits for STABLE_N consistent frames
3. Commands the xArm calibrator pin:
       hover  : TCP at TOUCH_Z_MM + HOVER_MM  (approach speed)
       descend: TCP at TOUCH_Z_MM              (slow touch speed)

   XY  — taken from the CV pipeline (pos_40_robot X/Y, reliable to ~2 mm)
   Z   — fixed constant TOUCH_Z_MM, calibrated by manual test with tf2_echo:
          TCP Z = 100 mm is the lowest safe height where the pin just
          reaches the table/marker surface without collision.
          CV Z is NOT used: depth-sensor noise makes it unreliable for Z.

All units: camera/CV → cm, xArm SDK → mm.
"""

import os
import sys
import time

import numpy as np

# ── Locate the computer_vision package ────────────────────────────────────────
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_CV_DIR   = os.path.normpath(os.path.join(_THIS_DIR, '..', 'computer_vision'))
sys.path.insert(0, _CV_DIR)

from xarm.wrapper import XArmAPI

from TATARO_CV_ROS2Camera                   import connect_camera, stop_camera, get_frame
from TATARO_CV_MarkerDetection_7x7          import detect_marker_7x7
from TATARO_CV_TableCropping_with_ARUCO_7x7 import detect_and_transform_table_using_aruco
from TATARO_CV_Pipeline                     import marker40_pose_in_robot_frame

# ── Configuration ──────────────────────────────────────────────────────────────
ROBOT_IP        = '192.168.1.150'
TOUCH_Z_MM      = 170.0  # TCP Z (mm) when pin tip is at the marker surface
                          # determined by manual test: tf2_echo link_base link_tcp
                          # at the lowest safe height without collision
HOVER_MM        = 50.0   # mm above TOUCH_Z_MM for the approach waypoint
SPEED_APPROACH  = 80.0   # mm/s — move to hover
SPEED_TOUCH     = 20.0   # mm/s — final descent
MVACC           = 100.0  # mm/s²
MIN_Z_MM        = 170.0  # safety floor — must equal TOUCH_Z_MM (pin hits table below this)
STABLE_N        = 10     # number of consecutive frames required for stable lock
STABLE_TOL_CM   = 0.5    # max per-axis std-dev across stable window (~5 mm, XY only matters)
DETECT_TIMEOUT  = 60.0   # seconds before giving up on detection


# ── Detection ──────────────────────────────────────────────────────────────────

def _stable_pos40_robot(pipe) -> np.ndarray:
    """
    Run detection until STABLE_N consecutive frames of pos_40_robot agree to
    within STABLE_TOL_CM on every axis.  Returns the mean position (cm, robot frame).
    Raises RuntimeError on timeout.
    """
    history  = []
    deadline = time.time() + DETECT_TIMEOUT
    print(f"[CV]  Waiting for {STABLE_N} stable frames of Marker 40 "
          f"(tol={STABLE_TOL_CM*10:.0f} mm) ...")

    while time.time() < deadline:
        color_frame, depth_frame = get_frame(pipe, return_depth=True)
        if color_frame is None or depth_frame is None:
            time.sleep(0.05)
            continue

        result = detect_and_transform_table_using_aruco(color_frame.copy(), depth_frame)
        if result is None:
            history.clear()
            continue

        (_, _, _, _, _, _, _,
         rvec_marker_wrt_camera, _,
         _, Table_corner_3D) = result

        rvec_40, tvec_40 = detect_marker_7x7(color_frame, 40, depth_frame)
        if tvec_40 is None:
            history.clear()
            continue

        pos_40_robot, _ = marker40_pose_in_robot_frame(
            rvec_40, tvec_40, Table_corner_3D, rvec_marker_wrt_camera
        )
        print("Marker40 robot =", pos_40_robot.ravel())

        history.append(pos_40_robot.copy())
        if len(history) > STABLE_N:
            history.pop(0)

        if len(history) == STABLE_N:
            spread = float(np.max(np.std(np.array(history), axis=0)))
            if spread < STABLE_TOL_CM:
                mean_pos = np.mean(np.array(history), axis=0)
                print(f"[CV]  Stable lock — spread={spread*10:.1f} mm | "
                      f"pos (cm) = {np.round(mean_pos, 2)}")
                return mean_pos

    raise RuntimeError(
        f"Marker 40 not stably detected within {DETECT_TIMEOUT:.0f} s. "
        "Check that markers 51/52/53 and 40 are all visible."
    )


# ── Robot motion ───────────────────────────────────────────────────────────────

def _touch_marker(pos_cm: np.ndarray) -> None:
    """
    Connect to the xArm and execute:
        1. Move to hover  (TCP at TOUCH_Z_MM + HOVER_MM), keeping current TCP orientation
        2. Descend to touch (TCP at TOUCH_Z_MM)
    XY from CV pipeline; Z fixed from manual calibration (TOUCH_Z_MM).
    All positions in mm for the xArm SDK.
    """
    x_mm    = float(pos_cm[0]) * 10.0
    y_mm    = float(pos_cm[1]) * 10.0
    z_touch = TOUCH_Z_MM
    z_hover = TOUCH_Z_MM 

    if z_touch < MIN_Z_MM:
        raise RuntimeError(
            f"TCP touch Z = {z_touch:.1f} mm is below the safety floor "
            f"({MIN_Z_MM:.0f} mm). Adjust TOUCH_Z_MM."
        )

    # ── Connect ────────────────────────────────────────────────────────────────
    print(f"[ARM] Connecting to {ROBOT_IP} ...")
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

    # Read current TCP to preserve tool orientation
    _, cur = arm.get_position()
    roll, pitch, yaw = cur[3], cur[4], cur[5]
    print(f"[ARM] Current TCP : ({cur[0]:.1f}, {cur[1]:.1f}, {cur[2]:.1f}) mm  "
          f"RPY = ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)")
    print(f"[ARM] Target XY   : ({x_mm:.1f}, {y_mm:.1f}) mm")
    print(f"[ARM] Hover Z     : {z_hover:.1f} mm  ({HOVER_MM:.0f} mm above touch)")
    print(f"[ARM] Touch Z     : {z_touch:.1f} mm  (fixed — manual calibration)")

    # ── Step 1: hover ─────────────────────────────────────────────────────────
    print(f"\n[ARM] ── Hover move ──────────────────────────────────────────────")
    code = arm.set_position(
        x_mm, y_mm, z_hover, roll, pitch, yaw,
        speed=SPEED_APPROACH, mvacc=MVACC, wait=True,
    )
    if code != 0:
        arm.disconnect()
        raise RuntimeError(f"Hover move failed (code={code})")
    print("[ARM] Hover reached.")

    # ── Step 2: descend to touch ──────────────────────────────────────────────
    print(f"\n[ARM] ── Touch move ──────────────────────────────────────────────")
    code = arm.set_position(
        x_mm, y_mm, z_touch, roll, pitch, yaw,
        speed=SPEED_TOUCH, mvacc=MVACC, wait=True,
    )
    if code != 0:
        arm.disconnect()
        raise RuntimeError(f"Touch move failed (code={code})")

    _, final = arm.get_position()
    print(f"[ARM] Touch complete — final TCP: "
          f"({final[0]:.1f}, {final[1]:.1f}, {final[2]:.1f}) mm")

    arm.disconnect()
    print("[ARM] Disconnected.")


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    print("\n" + "═" * 60)
    print("  TATARO — Marker 40 Touch Sequence")
    print("═" * 60)

    pipe = connect_camera()
    try:
        pos_cm = _stable_pos40_robot(pipe)
    finally:
        stop_camera(pipe)

    _touch_marker(pos_cm)

    print("\n" + "═" * 60)
    print("  Done — calibrator pin is at Marker 40 centre.")
    print("═" * 60 + "\n")


if __name__ == '__main__':
    main()
