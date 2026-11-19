#!/usr/bin/env python3
"""
tataro_table_crop_node — live table-crop visual check for the calibration workflow.

Subscribes to the already-running RealSense topics (camera must be launched first),
detects table markers 51 / 52 / 53, and displays a live perspective-corrected
top-down view alongside the masked table boundary.

Runs until Ctrl-C or the ros2 launch shuts down.  Does NOT launch its own
camera driver — it connects to whatever realsense2_camera instance is already
publishing on /camera/camera/color/image_raw and the aligned depth topic.
"""

import sys
from pathlib import Path

# resolve() follows the install symlink back to the source tree, so
# parent.parent always gives xarm_ros2/src/ regardless of how the script is launched.
_CV_DIR = str(Path(__file__).resolve().parent.parent / "computer_vision")
if _CV_DIR not in sys.path:
    sys.path.insert(0, _CV_DIR)

import cv2
from TATARO_CV_ROS2Camera import connect_camera, stop_camera
from TATARO_CV_TableCropping_with_ARUCO_7x7 import detect_and_transform_table_using_aruco


def main():
    pipe = connect_camera()
    try:
        while True:
            result = detect_and_transform_table_using_aruco(pipe)
            if result is None or result[0] is None:
                continue

            (color_frame, masked_table, warped_table,
             _P1, _P2, _P3, _P4,
             _rvec, _tvec, _rm, _corner) = result

            cv2.imshow("TATARO — Table boundary", color_frame)
            cv2.imshow("TATARO — Table crop",     masked_table)
            cv2.imshow("TATARO — Top-down view",  warped_table)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        stop_camera(pipe)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
