#!/usr/bin/env python3
"""
TATARO — Marker 40 Detect & Touch Loop
========================================
Phase 1 : Detect markers 51/52/53 once to lock the table frame.
Phase 2 : Continuously detect Marker 40 and print pos_40_robot.
          Press Enter  → execute hover + touch.
          Press Ctrl+C → exit cleanly.
After each touch the program immediately returns to detection so that
Marker 40 can be repositioned before the next touch.
All units: CV pipeline → cm | xArm SDK → mm.
"""

import os
import sys
import time
import threading

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
TOUCH_Z_MM      = 170.0   # TCP Z (mm) at marker surface — verified with tf2_echo
                           # SDK_Z = TF(link_base→link_tcp)_Z + 85 mm (pin length)
                           # Safe TF Z floor: 100 mm → SDK floor: 185 mm
HOVER_MM        = 50.0    # mm above TOUCH_Z_MM for the approach waypoint
SPEED_APPROACH  = 80.0    # mm/s  — hover move
SPEED_TOUCH     = 20.0    # mm/s  — final descent
MVACC           = 100.0   # mm/s²
MIN_Z_MM        = 170.0   # safety floor — must equal TOUCH_Z_MM
STABLE_N        = 10      # consecutive frames required for stable position lock
STABLE_TOL_CM   = 0.5     # max per-axis std-dev across the window (~5 mm)
INIT_TIMEOUT    = 60.0    # seconds to find markers 51/52/53 at startup


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1 — Table frame initialisation (runs once)
# ─────────────────────────────────────────────────────────────────────────────

def _init_table_frame(pipe) -> dict:
    """
    Block until markers 51/52/53 are detected and the table frame is locked.
    Returns a dict with rvec_marker_wrt_camera and Table_corner_3D.
    Raises RuntimeError if markers are not found within INIT_TIMEOUT seconds.
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

        print("[Phase 1] Table frame locked — entering detection loop.")
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
    Daemon thread: detects Marker 40 continuously using the frozen table frame.

    Prints pos_40_robot every frame.
    Tags each line [STABLE] once STABLE_N consecutive frames agree within
    STABLE_TOL_CM; otherwise tags [CV].
    Updates shared['stable_pos'] so the main thread can read it on Enter.
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
            shared['stable_pos'] = stable.copy() if stable is not None else None

        tag = '[STABLE]' if stable is not None else '[CV]   '
        print(f"{tag}  pos_40_robot = {np.round(pos.ravel(), 2)}", flush=True)


# ─────────────────────────────────────────────────────────────────────────────
# Robot connection and touch sequence
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


def _touch_marker(arm: XArmAPI, pos_cm: np.ndarray) -> None:
    """
    Hover + descend using the persistent arm connection.
    XY from CV pipeline (reliable ~2 mm).
    Z fixed at TOUCH_Z_MM — depth sensor Z is not used (±5–15 mm noise).
    """
    x_mm    = float(pos_cm[0]) * 10.0
    y_mm    = float(pos_cm[1]) * 10.0
    z_hover = TOUCH_Z_MM + HOVER_MM
    z_touch = TOUCH_Z_MM

    if z_touch < MIN_Z_MM:
        raise RuntimeError(
            f"Touch Z = {z_touch:.1f} mm is below safety floor ({MIN_Z_MM:.0f} mm). "
            "Adjust TOUCH_Z_MM."
        )

    # Re-enable before every touch in case the robot faulted while idle
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.3)

    _, cur = arm.get_position()
    roll, pitch, yaw = cur[3], cur[4], cur[5]
    print(f"[ARM]  Current TCP  : ({cur[0]:.1f}, {cur[1]:.1f}, {cur[2]:.1f}) mm  "
          f"RPY = ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)")
    print(f"[ARM]  Target XY    : ({x_mm:.1f}, {y_mm:.1f}) mm")
    print(f"[ARM]  Hover Z      : {z_hover:.1f} mm  ({HOVER_MM:.0f} mm above touch)")
    print(f"[ARM]  Touch Z      : {z_touch:.1f} mm  (fixed — manual calibration)")

    print("\n[ARM]  ── Hover move ────────────────────────────────────────────")
    code = arm.set_position(x_mm, y_mm, z_hover, roll, pitch, yaw,
                            speed=SPEED_APPROACH, mvacc=MVACC, wait=True)
    if code != 0:
        raise RuntimeError(f"Hover move failed (code={code})")
    print("[ARM]  Hover reached.")

    print("\n[ARM]  ── Touch move ────────────────────────────────────────────")
    code = arm.set_position(x_mm, y_mm, z_touch, roll, pitch, yaw,
                            speed=SPEED_TOUCH, mvacc=MVACC, wait=True)
    if code != 0:
        raise RuntimeError(f"Touch move failed (code={code})")

    _, final = arm.get_position()
    print(f"[ARM]  Touch complete — TCP: "
          f"({final[0]:.1f}, {final[1]:.1f}, {final[2]:.1f}) mm")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("\n" + "═" * 60)
    print("  TATARO — Marker 40 Detect & Touch Loop")
    print("═" * 60)

    pipe       = connect_camera()
    arm        = None
    stop_event = threading.Event()

    try:
        # ── Phase 1 ──────────────────────────────────────────────────────────
        frozen = _init_table_frame(pipe)

        # Single robot connection held for the whole session
        arm = _connect_robot()

        # Shared state between the detection thread and the main thread
        shared: dict = {
            'lock':       threading.Lock(),
            'stable_pos': None,
        }

        # Start background detection (daemon so it exits when main exits)
        det_thread = threading.Thread(
            target=_detect_loop,
            args=(pipe, frozen, shared, stop_event),
            daemon=True,
        )
        det_thread.start()

        print("\n[Phase 2]  Continuous Marker 40 detection running.")
        print("           Wait for [STABLE] readings, then press Enter to touch.")
        print("           Press Ctrl+C to exit.\n")
        print("─" * 60)

        # ── Phase 2 — detect → wait for Enter → touch → repeat ───────────────
        while True:
            # Block here until the user presses Enter.
            # The detection thread continues printing pos_40_robot in the background.
            input()

            with shared['lock']:
                pos = shared['stable_pos']

            if pos is None:
                print("[WARN]  No stable position yet. "
                      "Wait for [STABLE] readings before touching.\n")
                continue

            print(f"\n[TOUCH]  Stable pos_40_robot = {np.round(pos, 2)}")
            print("─" * 60)

            _touch_marker(arm, pos)

            print("─" * 60)
            print("[INFO]  Touch done.  Reposition Marker 40, then press Enter for next touch.\n")
            print("─" * 60)

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
