#!/usr/bin/env python3
"""
TATARO — Colour-Cube Hover Test (NO gripper action)
==================================================================
Phase 1 : Detect markers 51/52/53 once -> lock table frame + build the
           table-boundary mask (same as TATARO_CV_ObjectDetectionPipeline.py).
Phase 2 : Poll TATARO_CV_ObjectDetectionPipeline's colour segmentation until
           a target-coloured cube gives STABLE_N consecutive readings that
           agree within STABLE_TOL_CM.
Phase 3 : Move the arm through a staged sequence of fixed, deliberately
           generous hover heights above the cube (HOVER_STEPS_MM), yaw set
           to the detected grasp angle. Each step prints the full target
           pose and requires its own typed 'go' confirmation before moving
           -- type anything else to stop the sequence at the current height.
           NO GRIPPER ACTION at any step.

Why the hover heights are fixed constants, not computed from the cube:
  The gripper's flange-to-fingertip Z offset has not been measured on real
  hardware yet.
    - tataro_marker40_touch_loopV1.0.py's TOUCH_Z_MM=200mm was calibrated
      for a *touch pin* tool.
    - object_touch.py's TOUCH_Z_MM=20mm was calibrated for the *bare flange*
      touching a cube directly (no tool at all).
    - Neither applies to the Gripper Lite, which has its own extension
      (~86.6mm flange-to-fingertip, per TATRO_PositionControl_GUI.py's
      TOOL_Z_OFFSET_MM) that has not been verified against this direct-SDK
      Cartesian convention (arm.set_position() with no arm.set_tcp_offset()
      registered -- same as the two scripts above).
  This script therefore only validates X/Y placement and yaw alignment from
  a safe, fixed hover height. It does not attempt to compute or guess a
  touch/grasp Z -- that is a deliberate follow-up step once this is proven
  safe, using either manual jogging (see TATRO_PositionControl_GUI.py) or a
  cautious incremental-descent script.

All units: CV pipeline -> cm | xArm SDK -> mm.

Usage:
    python3 tataro_cube_hover_test.py [colour]     # colour defaults to 'red'
"""

import os
import sys
import time

import cv2
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_CV_DIR   = os.path.normpath(os.path.join(_THIS_DIR, '..', 'computer_vision'))
sys.path.insert(0, _CV_DIR)

from xarm.wrapper import XArmAPI

from TATARO_CV_ROS2Camera import connect_camera, stop_camera, get_frame
from TATARO_CV_TableCropping_with_ARUCO_7x7 import detect_and_transform_table_using_aruco
from TATARO_CV_ObjectDetectionPipeline import detect_colored_cubes, cube_pose_in_robot_frame

# ── Configuration ─────────────────────────────────────────────────────────────
ROBOT_IP = '192.168.1.150'

# Staged step-down heights (mm), each gated behind its own typed 'go'
# confirmation -- NOT derived from any measured contact height, these only
# need to be "clearly, safely above the table no matter which tool is
# mounted." The highest previously-verified-safe flange height in this
# codebase is 250mm (tataro_marker40_touch_loopV1.0.py's hover, for a LONG
# touch-pin tool -- the flange sits much higher than that pin's tip).
#
# Do NOT add a step anywhere near object_touch.py's TOUCH_Z_MM=20mm -- that
# was measured for the BARE FLANGE with no tool mounted. The Gripper Lite
# currently mounted extends ~86.6mm below the flange (TOOL_Z_OFFSET_MM in
# TATRO_PositionControl_GUI.py), so commanding the flange to 20mm would try
# to drive the fingertips to roughly 20 - 86.6 = -67mm -- well below the
# table surface. Every step here stays well clear of that until the
# gripper's real flange-to-fingertip offset is measured on hardware
# (manual jog + TATRO_PositionControl_GUI.py, or a dedicated calibration
# step), at which point a proper touch/grasp Z can be computed instead of
# guessed.
HOVER_STEPS_MM = [300.0, 50.0]

SPEED_APPROACH = 300.0    # mm/s -- conservative for a first-ever move
MVACC_APPROACH = 800.0    # mm/s^2

STABLE_N       = 10       # consecutive frames required to call a detection stable
STABLE_TOL_CM  = 0.5      # max per-axis std-dev across STABLE_N frames (~5 mm)
INIT_TIMEOUT   = 60.0     # seconds to find markers 51/52/53
STABLE_TIMEOUT = 60.0     # seconds to get a stable cube reading


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1 — Table frame + boundary mask (headless, no debug windows)
# ─────────────────────────────────────────────────────────────────────────────

def _init_table_frame(pipe) -> dict:
    print(f"[Phase 1] Searching for markers 51, 52, 53  (timeout {INIT_TIMEOUT:.0f}s) ...")
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
         P1_2D, P2_2D, P3_2D, P4_2D,
         rvec_marker_wrt_camera, _,
         _, Table_corner_3D) = result

        # Same table-boundary mask as TATARO_CV_ObjectDetectionPipeline.py --
        # keeps colour matches restricted to the table, corner markers excluded.
        h_f, w_f = color_frame.shape[:2]
        src_pts = np.float32([P1_2D, P2_2D, P3_2D, P4_2D])
        table_mask = np.zeros((h_f, w_f), dtype=np.uint8)
        cv2.fillConvexPoly(table_mask, src_pts.astype(np.int32), 255)
        table_mask = cv2.erode(table_mask, np.ones((100, 100), np.uint8))

        print("[Phase 1] Table frame locked.")
        return {
            'rvec_marker_wrt_camera': rvec_marker_wrt_camera,
            'Table_corner_3D':        Table_corner_3D,
            'table_mask':             table_mask,
        }

    raise RuntimeError(
        f"Markers 51/52/53 not found within {INIT_TIMEOUT:.0f}s. "
        "Ensure all three markers are visible to the camera."
    )


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2 — Wait for a stable cube reading
# ─────────────────────────────────────────────────────────────────────────────

def _wait_stable_cube(pipe, frozen, target_color, timeout=STABLE_TIMEOUT):
    rvec_marker_wrt_camera = frozen['rvec_marker_wrt_camera']
    Table_corner_3D        = frozen['Table_corner_3D']
    table_mask              = frozen['table_mask']

    history = []
    deadline = time.time() + timeout
    print(f"[Phase 2] Waiting for a stable '{target_color}' cube detection "
          f"(timeout {timeout:.0f}s) ...")

    while time.time() < deadline:
        color_frame, depth_frame = get_frame(pipe, return_depth=True)
        if color_frame is None or depth_frame is None:
            time.sleep(0.02)
            continue

        detections = detect_colored_cubes(color_frame, depth_frame, table_mask=table_mask)
        matches = [d for d in detections if d['color'] == target_color]

        if not matches:
            history.clear()
            continue

        best = max(matches, key=lambda d: d['area'])
        pos_robot, yaw_deg = cube_pose_in_robot_frame(
            best, Table_corner_3D, rvec_marker_wrt_camera
        )

        history.append(pos_robot.copy())
        if len(history) > STABLE_N:
            history.pop(0)

        print(f"[CV]  {target_color} pos={np.round(pos_robot, 2)} cm  "
              f"yaw={yaw_deg:.1f} deg  (stabilising, {len(history)}/{STABLE_N})")

        if len(history) == STABLE_N:
            positions = np.array(history)
            spread = float(np.max(np.std(positions, axis=0)))
            if spread < STABLE_TOL_CM:
                mean_pos = positions.mean(axis=0)
                # Use the most recent yaw reading, not an average -- mod-90
                # wraparound makes averaging yaw across samples unsafe near
                # the 0/90 boundary.
                print(f"[Phase 2] Stable: pos={np.round(mean_pos, 2)} cm  "
                      f"yaw={yaw_deg:.1f} deg")
                return mean_pos, yaw_deg

    raise RuntimeError(
        f"No stable '{target_color}' cube detection within {timeout:.0f}s. "
        "Check the cube is on the table, in view, and HSV-calibrated."
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


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    target_color = sys.argv[1] if len(sys.argv) > 1 else 'red'

    print("\n" + "=" * 60)
    print(f"  TATARO -- Colour-Cube Hover Test  (target colour: {target_color})")
    print("  NO DESCENT. NO GRIPPER ACTION. Validates X/Y/yaw only.")
    print("=" * 60)

    pipe = connect_camera()
    arm  = None

    try:
        # ── Phase 1: lock table frame ───────────────────────────────────────
        frozen = _init_table_frame(pipe)

        # ── Phase 2: wait for a stable cube reading ─────────────────────────
        pos_robot_cm, yaw_deg = _wait_stable_cube(pipe, frozen, target_color)

        # ── Connect to the robot only after we already have a target pose ──
        arm = _connect_robot()

        _, cur = arm.get_position()
        roll, pitch = cur[3], cur[4]
        print(f"[ARM]  Current TCP  : ({cur[0]:.1f}, {cur[1]:.1f}, {cur[2]:.1f}) mm  "
              f"RPY = ({roll:.1f}, {pitch:.1f}, {cur[5]:.1f}) deg")

        x_mm = float(pos_robot_cm[0]) * 10.0
        y_mm = float(pos_robot_cm[1]) * 10.0
        target_yaw = yaw_deg   # GRIPPER_YAW_OFFSET_DEG is 0.0 pending calibration

        # ── Staged step-down: each level needs its own typed confirmation ──
        for i, z_mm in enumerate(HOVER_STEPS_MM):
            print(f"\n[PLAN] Step {i + 1}/{len(HOVER_STEPS_MM)} -- target hover pose:")
            print(f"       X={x_mm:.1f} mm  Y={y_mm:.1f} mm  "
                  f"Z={z_mm:.1f} mm (fixed step, NOT cube-derived)")
            print(f"       Roll={roll:.1f} deg  Pitch={pitch:.1f} deg  Yaw={target_yaw:.1f} deg")
            print("       This move does NOT touch the gripper.")

            confirm = input(f"       Type 'go' to descend to {z_mm:.0f}mm, "
                             f"'stop' to end the sequence here: ")
            if confirm.strip().lower() != 'go':
                print("[Stop] Sequence ended by user.")
                break

            code = arm.set_position(
                x_mm, y_mm, z_mm, roll, pitch, target_yaw,
                speed=SPEED_APPROACH, mvacc=MVACC_APPROACH, wait=True,
            )
            if code != 0:
                raise RuntimeError(f"Move to Z={z_mm:.1f}mm failed (code={code})")

            _, final = arm.get_position()
            print(f"[ARM]  Reached — TCP: "
                  f"({final[0]:.1f}, {final[1]:.1f}, {final[2]:.1f}) mm  "
                  f"RPY=({final[3]:.1f}, {final[4]:.1f}, {final[5]:.1f}) deg")

        print("\n[Done] Visually check:")
        print("       1) Is the gripper positioned directly above the cube (X/Y correct)?")
        print("       2) Does its rotation look aligned with a cube edge (yaw correct)?")
        print("       Report both back before we plan the next (real descent-calibration) step.")

    except KeyboardInterrupt:
        print("\n[Exit] Ctrl+C received.")

    finally:
        stop_camera(pipe)
        if arm is not None:
            arm.disconnect()
            print("[ARM]  Disconnected.")
        print("[Exit] Done.\n")


if __name__ == '__main__':
    main()
