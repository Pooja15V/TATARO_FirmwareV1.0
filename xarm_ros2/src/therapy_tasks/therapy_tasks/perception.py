#!/usr/bin/env python3
"""
TATARO Therapy — Perception module.

One job: verify the tabletop (markers 51/52/53) is visible and lock the
table frame ONCE at session startup. Progressive-reach poses are pre-defined
in the table frame (see config/tataro_therapy_levels.yaml) and converted to
the robot base frame with the fixed table->robot transform
(R_robot_table / t_robot_table) — the same calibration constants
TATARO_CV_Transformations.py uses for marker tracking. No further detection
is required once the session starts.
"""

import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
# .../src/therapy_tasks/therapy_tasks/perception.py -> .../src/computer_vision
_CV_DIR = os.path.normpath(os.path.join(_THIS_DIR, '..', '..', 'computer_vision'))
if _CV_DIR not in sys.path:
    sys.path.insert(0, _CV_DIR)

import numpy as np

from TATARO_CV_ROS2Camera import connect_camera, get_frame
from TATARO_CV_TableCropping_with_ARUCO_7x7 import detect_and_transform_table_using_aruco
from TATARO_CV_Transformations import R_robot_table, t_robot_table


def verify_table_frame(timeout_s: float = 60.0) -> None:
    """
    Block until markers 51/52/53 are all detected at least once.

    This is a startup safety check, not a live tracking dependency — the
    table->robot transform used by table_xy_to_robot_mm() is a fixed
    calibration constant, independent of any single detection. Raises
    RuntimeError on timeout.

    NOTE: connect_camera()/stop_camera() manage their own rclpy context, and
    stop_camera() calls rclpy.shutdown() — which would tear down the FSM
    node's rclpy context too if called from the same process. We
    deliberately never call stop_camera() here; the idle camera subscriber
    node is simply left running for the rest of the process lifetime.
    """
    print(f"[Perception] Verifying table markers 51/52/53 (timeout {timeout_s:.0f}s) ...")
    pipe = connect_camera()
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        color_frame, depth_frame = get_frame(pipe, return_depth=True)
        if color_frame is None or depth_frame is None:
            time.sleep(0.05)
            continue

        result = detect_and_transform_table_using_aruco(color_frame.copy(), depth_frame)
        if result is not None:
            print("[Perception] Table frame verified.")
            return

    raise RuntimeError(
        f"Markers 51/52/53 not found within {timeout_s:.0f}s. "
        "Ensure all three table markers are visible to the camera."
    )


def table_xy_to_robot_mm(x_cm: float, y_cm: float) -> np.ndarray:
    """
    Convert an (x, y) point on the TABLE plane (cm) to the robot base frame
    (mm), using the fixed table->robot calibration constants (R_robot_table,
    t_robot_table). Returns a 2-element numpy array [x_mm, y_mm].

    Z is intentionally NOT derived from this transform. Every existing
    TATARO script (tataro_marker40_touch_loopV1.0.py, object_touch.py) only
    trusts X/Y from this transform for planar tracking and always uses a
    separately hand-calibrated Z constant (verified with
    `ros2 run tf2_ros tf2_echo link_base link_tcp`) for actual robot height
    -- the table->robot Z axis/offset is not calibrated precisely enough for
    direct use. Therapy level/grasp Z heights follow the same convention:
    they are specified directly in the robot frame in the YAML config
    (z_mm), not converted from table_xy_cm.
    """
    point_table = np.array([[x_cm], [y_cm], [0.0]], dtype=np.float64)
    point_robot_cm = R_robot_table @ (point_table - t_robot_table)
    return point_robot_cm.flatten()[:2] * 10.0
