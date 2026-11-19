import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from TATARO_CV_ROS2Camera import connect_camera, stop_camera, get_frame
from TATARO_CV_MarkerDetection_7x7 import detect_marker_7x7, camera_matrix, dist_coeffs
from TATARO_CV_TableCropping_with_ARUCO_7x7 import (
    detect_and_transform_table_using_aruco,
    project_point,
)
from TATARO_CV_Transformations import camera_to_robot, R_robot_table

_FONT = cv2.FONT_HERSHEY_SIMPLEX


def marker40_pose_in_robot_frame(rvec_40, tvec_40, Table_corner_3D, rvec_marker_wrt_camera):
    """
    Return (position_robot [3], rvec_robot [3]) for marker 40.

    position : X/Y/Z in cm in robot base frame
    rvec_robot: Rodrigues rotation of marker 40 expressed in robot base frame
    """
    pos_robot = camera_to_robot(tvec_40.flatten(), Table_corner_3D, rvec_marker_wrt_camera)

    # Rotation chain: camera -> table -> robot
    R_cam_table, _ = cv2.Rodrigues(rvec_marker_wrt_camera)
    R_40_cam,    _ = cv2.Rodrigues(rvec_40)
    R_robot_cam    = R_robot_table @ R_cam_table.T
    R_40_robot     = R_robot_cam @ R_40_cam
    rvec_40_robot, _ = cv2.Rodrigues(R_40_robot)
    # print("Marker40 robot =", pos_robot.ravel())
    # print("Marker40 robot =", rvec_40_robot.ravel())

    euler = Rot.from_matrix(R_40_robot).as_euler('xyz', degrees=True)
    # print("RPY =", euler)
    return pos_robot, rvec_40_robot.flatten()


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1 — Table frame initialisation (runs once at startup)
# ─────────────────────────────────────────────────────────────────────────────

def _phase1_init_table(pipe):
    """
    Block until markers 51/52/53 are detected and the table frame is locked.

    Returns a dict of frozen values for Phase 2, or None if the user quits.
    """
    print("[Phase 1] Searching for markers 51, 52, 53 ...")

    while True:
        color_frame, depth_frame = get_frame(pipe, return_depth=True)

        if color_frame is None or depth_frame is None:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return None
            continue

        result = detect_and_transform_table_using_aruco(color_frame.copy(), depth_frame)

        if result is None:
            display = color_frame.copy()
            cv2.putText(display, "Phase 1: Searching for markers 51, 52, 53 ...",
                        (20, 40), _FONT, 0.7, (0, 165, 255), 2)
            cv2.imshow("TATARO - Camera Feed", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return None
            continue

        (annotated, masked_table, warped_table,
         P1_2D, P2_2D, P3_2D, P4_2D,
         rvec_marker_wrt_camera, tvec_marker_wrt_camera,
         rm_marker51_wrt_camera, Table_corner_3D) = result

        # Pre-compute warp matrix M from frozen corner points.
        # This is reused every Phase-2 frame instead of recomputing from
        # freshly detected markers — eliminating frame-to-frame jitter.
        h_w, w_w = warped_table.shape[:2]
        src_pts = np.float32([P1_2D, P2_2D, P3_2D, P4_2D])
        dst_pts = np.float32([[0, 0], [w_w, 0], [w_w, h_w], [0, h_w]])
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)

        cv2.putText(annotated, "Phase 1: Table frame locked!",
                    (20, 40), _FONT, 0.7, (0, 255, 0), 2)
        cv2.imshow("TATARO - Camera Feed", annotated)
        cv2.imshow("Table alone (Phase 1 reference)", masked_table)
        cv2.imshow("Warped Table (top-down)", warped_table)
        cv2.waitKey(1)

        print("[Phase 1] Table frame locked — entering Phase 2.")

        return {
            'rvec_marker_wrt_camera': rvec_marker_wrt_camera,
            'Table_corner_3D':        Table_corner_3D,
            'M':                      M,
            'warped_size':            (w_w, h_w),   # (width, height) for cv2.warpPerspective
            'pixels_per_cm_x':        w_w / 85.0,
            'pixels_per_cm_y':        h_w / 85.0,
        }


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2 — Continuous Marker 40 detection (uses frozen table frame)
# ─────────────────────────────────────────────────────────────────────────────

def run_pipeline():
    pipe = connect_camera()
    try:
        # ── Phase 1 ──────────────────────────────────────────────────────────
        table = _phase1_init_table(pipe)
        if table is None:
            print("[Abort] Table initialisation cancelled.")
            return

        rvec_marker_wrt_camera = table['rvec_marker_wrt_camera']
        Table_corner_3D        = table['Table_corner_3D']
        M                      = table['M']
        warped_size            = table['warped_size']
        pixels_per_cm_x        = table['pixels_per_cm_x']
        pixels_per_cm_y        = table['pixels_per_cm_y']

        # R_cam_table is constant for the session — derive once here
        R_cam_table, _ = cv2.Rodrigues(rvec_marker_wrt_camera)

        # ── Phase 2 ──────────────────────────────────────────────────────────
        print("[Phase 2] Continuous Marker 40 detection running ...")

        while True:
            color_frame, depth_frame = get_frame(pipe, return_depth=True)

            if color_frame is None or depth_frame is None:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            annotated    = color_frame.copy()
            warped_table = cv2.warpPerspective(color_frame, M, warped_size)

            cv2.putText(annotated, "Phase 2: Table frame locked",
                        (20, 40), _FONT, 0.6, (0, 255, 0), 1)

            # ── Detect Marker 40 ──────────────────────────────────────────────
            rvec_40, tvec_40 = detect_marker_7x7(color_frame, 40, depth_frame)

            if tvec_40 is not None:
                # ── Transform to robot base frame ─────────────────────────────
                pos_40_robot, rvec_40_robot = marker40_pose_in_robot_frame(
                    rvec_40, tvec_40, Table_corner_3D, rvec_marker_wrt_camera
                )
                # print("Marker40 robot from pipeline =", pos_40_robot.ravel())

                # Marker 40 position in table frame (for warped display)
                point_table = R_cam_table.T @ (
                    tvec_40.reshape(3, 1) - Table_corner_3D.reshape(3, 1)
                )

                x_pix = int(point_table[0, 0] * pixels_per_cm_x)
                y_pix = int(point_table[1, 0] * pixels_per_cm_y)
                cv2.circle(warped_table, (x_pix, y_pix), 8, (0, 0, 255), -1)
                cv2.putText(
                    warped_table,
                    f"({point_table[0, 0]:.1f}, {point_table[1, 0]:.1f})",
                    (x_pix + 10, y_pix),
                    _FONT, 0.5, (0, 0, 255), 1,
                )

                # Camera feed: 3-axis frame + dot
                pt = project_point(tvec_40.flatten(), camera_matrix)
                if pt is not None:
                    cv2.drawFrameAxes(annotated, camera_matrix, dist_coeffs,
                                      rvec_40, tvec_40, 5)
                    cv2.circle(annotated, pt, 6, (0, 255, 0), -1)

                    pt_w = cv2.perspectiveTransform(
                        np.float32([[[pt[0], pt[1]]]]), M
                    )
                    pw = tuple(pt_w[0, 0].astype(int))
                    cv2.circle(warped_table, pw, 8, (0, 255, 0), -1)
                    cv2.putText(warped_table, "ID40", (pw[0] + 6, pw[1] - 6),
                                _FONT, 0.6, (0, 255, 0), 2)

            else:
                cv2.putText(annotated, "Marker 40 not detected",
                            (20, 70), _FONT, 0.6, (0, 0, 255), 2)

            cv2.imshow("TATARO - Camera Feed", annotated)
            cv2.imshow("Warped Table (top-down)", warped_table)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        stop_camera(pipe)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run_pipeline()
