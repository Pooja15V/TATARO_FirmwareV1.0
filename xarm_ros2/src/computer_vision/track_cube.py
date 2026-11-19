"""
track_cube.py — Tracks a 3.5 cm cube fitted with DICT_4X4 ID-1 ArUco markers
                (3 cm printed size, one marker per face) using the static D455
                camera and the TATARO table-frame calibration.

Phase 1 : Lock table frame from markers 51 / 52 / 53  (DICT_7X7)
Phase 2 : Detect all visible ID-1 faces, compute cube centre in robot frame,
          display on camera feed and warped top-down view.

Output per frame (printed to console):
    Cube Robot XYZ (cm)  —  position of cube centre in robot base frame
    RPY (deg)            —  roll / pitch / yaw of the most-frontal visible face
"""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from TATARO_CV_ROS2Camera import connect_camera, stop_camera, get_frame
from TATARO_CV_MarkerDetection_7x7 import camera_matrix, dist_coeffs
from TATARO_CV_TableCropping_with_ARUCO_7x7 import (
    detect_and_transform_table_using_aruco,
    project_point,
)
from TATARO_CV_Transformations import camera_to_robot, R_robot_table

_FONT = cv2.FONT_HERSHEY_SIMPLEX

# ── Cube / marker geometry ────────────────────────────────────────────────────
CUBE_MARKER_ID = 1
CUBE_SIZE_CM   = 3.5          # outer cube edge length
MARKER_SIZE_CM = 3.0          # printed black ArUco pattern size
CUBE_HALF_CM   = CUBE_SIZE_CM / 2.0   # 1.75 cm — marker face centre to cube centre

_DICT_4X4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# solvePnP object-point template for a MARKER_SIZE_CM × MARKER_SIZE_CM marker
_HALF_M = MARKER_SIZE_CM / 2.0
_OBJ_PTS = np.array([
    [-_HALF_M,  _HALF_M, 0],
    [ _HALF_M,  _HALF_M, 0],
    [ _HALF_M, -_HALF_M, 0],
    [-_HALF_M, -_HALF_M, 0],
], dtype=np.float32)

# ── False-positive rejection thresholds ───────────────────────────────────────
# A 3 cm marker on a cube face viewed at an oblique angle can be as small as
# ~10 px wide → area ~100 px².  Keep the area floor low; the Z check below
# is the primary guard against false positives.
MIN_MARKER_AREA_PX = 100

# Depth range for a valid marker on the table (mm).
MIN_DEPTH_MM = 100
MAX_DEPTH_MM = 2000

# Robot-frame Z bounds (cm): cube centre must be above the table surface.
# Table surface ≈ Z_robot = 0 cm; cube centre ≈ +1.75 cm.
# The false positive showed Z = -9.06 cm — this filter catches it.
MIN_Z_ROBOT_CM = -5.0
MAX_Z_ROBOT_CM = 15.0


# ── Depth helper ──────────────────────────────────────────────────────────────
def _sample_depth(depth_frame, u, v, radius=8):
    """Return median non-zero depth (mm) in a square patch centred on (u, v)."""
    h, w = depth_frame.shape
    samples = []
    for dv in range(-radius, radius + 1):
        for du in range(-radius, radius + 1):
            nu, nv = u + du, v + dv
            if 0 <= nv < h and 0 <= nu < w:
                z = depth_frame[nv, nu]
                if z > 0 and not np.isnan(z):
                    samples.append(float(z))
    return float(np.median(samples)) if samples else 0.0


# ── Per-face detection ────────────────────────────────────────────────────────
def detect_cube_faces(color_frame, depth_frame):
    """
    Detect all visible ID-1 faces of the cube (DICT_4X4_50).

    For each detected face the cube centre in camera frame is estimated by
    offsetting the marker face centre by CUBE_HALF_CM along the inward face
    normal (opposite the marker Z-axis, which points toward the camera).

    Returns a list of dicts — one per detected face:
        cube_center_cam : np.ndarray [3]   — cube centre in camera frame (cm)
        tvec_face       : np.ndarray [3,1] — marker face centre in camera frame (cm)
        rvec_face       : np.ndarray [3,1] — Rodrigues rotation of the face
        corners         : np.ndarray [4,2] — pixel corners
        area            : float            — pixel area (larger = more frontal / reliable)

    Returns [] when no ID-1 marker is found.
    """
    params = cv2.aruco.DetectorParameters()
    params.errorCorrectionRate    = 0.6   # default; allows minor bit noise on real markers
    params.minMarkerPerimeterRate = 0.03  # default; don't reject small markers at distance
    detector = cv2.aruco.ArucoDetector(_DICT_4X4, params)

    gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return []

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    h_d, w_d = depth_frame.shape

    faces = []
    for i, found_id in enumerate(ids.flatten()):
        if found_id != CUBE_MARKER_ID:
            continue

        mc = corners[i].reshape(4, 2)

        # ── Filter 1: minimum pixel area ─────────────────────────────────────
        area = cv2.contourArea(mc.astype(np.float32))
        if area < MIN_MARKER_AREA_PX:
            continue

        u, v = int(mc[:, 0].mean()), int(mc[:, 1].mean())

        if not (0 <= v < h_d and 0 <= u < w_d):
            continue

        # ── Filter 2: depth in valid range ───────────────────────────────────
        z_depth = _sample_depth(depth_frame, u, v, radius=8)
        if not (MIN_DEPTH_MM < z_depth < MAX_DEPTH_MM):
            continue

        Z = z_depth / 10.0      # mm → cm
        tvec_face = np.array([
            [(u - cx) * Z / fx],
            [(v - cy) * Z / fy],
            [Z],
        ], dtype=np.float64)

        success, rvec_face, _ = cv2.solvePnP(
            _OBJ_PTS, mc.astype(np.float32), camera_matrix, dist_coeffs
        )
        if not success:
            continue

        R_face, _ = cv2.Rodrigues(rvec_face)
        # Marker Z-axis (R_face[:, 2]) points toward the camera (outward from face).
        # Cube centre = face centre − outward_normal × CUBE_HALF_CM
        outward_cam     = R_face[:, 2]
        cube_center_cam = tvec_face.flatten() - outward_cam * CUBE_HALF_CM

        faces.append({
            'cube_center_cam': cube_center_cam,
            'tvec_face':       tvec_face,
            'rvec_face':       rvec_face,
            'corners':         mc,
            'area':            area,   # already computed above for Filter 1
        })

    return faces


# ── Robot-frame pose ──────────────────────────────────────────────────────────
def cube_pose_in_robot_frame(faces, Table_corner_3D, rvec_marker_wrt_camera):
    """
    Aggregate all visible face detections into a single cube pose.

    Position  — weighted average of cube-centre estimates (weight = pixel area).
    Rotation  — taken from the largest-area (most frontal) visible face.

    Returns (pos_robot [3, cm], rvec_robot [3], euler_rpy [3, deg])
    """
    centers = np.array([f['cube_center_cam'] for f in faces])    # (N, 3)
    areas   = np.array([f['area'] for f in faces], dtype=np.float64)
    cube_center_cam = np.average(centers, axis=0, weights=areas)

    pos_robot = camera_to_robot(cube_center_cam, Table_corner_3D, rvec_marker_wrt_camera)

    best_face      = max(faces, key=lambda f: f['area'])
    R_cam_table, _ = cv2.Rodrigues(rvec_marker_wrt_camera)
    R_face_cam, _  = cv2.Rodrigues(best_face['rvec_face'])
    R_robot_cam    = R_robot_table @ R_cam_table.T
    R_face_robot   = R_robot_cam @ R_face_cam
    rvec_robot, _  = cv2.Rodrigues(R_face_robot)
    euler          = Rot.from_matrix(R_face_robot).as_euler('xyz', degrees=True)

    return pos_robot, rvec_robot.flatten(), euler


# ── Phase 1 — lock table frame ────────────────────────────────────────────────
def _phase1_init_table(pipe):
    print("[Phase 1] Searching for table markers 51, 52, 53 ...")

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
            'warped_size':            (w_w, h_w),
            'pixels_per_cm_x':        w_w / 85.0,
            'pixels_per_cm_y':        h_w / 85.0,
        }


# ── Phase 2 — continuous cube tracking ───────────────────────────────────────
def run_pipeline():
    pipe = connect_camera()
    try:
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

        R_cam_table, _ = cv2.Rodrigues(rvec_marker_wrt_camera)

        print("[Phase 2] Cube tracking running  (DICT_4X4, ID=1, 3.0 cm marker, 3.5 cm cube) ...")

        while True:
            color_frame, depth_frame = get_frame(pipe, return_depth=True)
            if color_frame is None or depth_frame is None:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            annotated    = color_frame.copy()
            warped_table = cv2.warpPerspective(color_frame, M, warped_size)

            cv2.putText(annotated, "Phase 2: Table locked | Cube tracking (ID=1, 4x4)",
                        (20, 40), _FONT, 0.6, (0, 255, 0), 1)

            faces = detect_cube_faces(color_frame, depth_frame)

            if faces:
                pos_robot, rvec_robot, euler = cube_pose_in_robot_frame(
                    faces, Table_corner_3D, rvec_marker_wrt_camera
                )

                # ── Filter 3: robot-frame Z sanity check ─────────────────────
                # Cube centre must be above the table (Z_robot ≈ +1.75 cm).
                # Negative or very high Z means a false positive — skip frame.
                if not (MIN_Z_ROBOT_CM < pos_robot[2] < MAX_Z_ROBOT_CM):
                    cv2.putText(annotated,
                                f"False positive rejected (Z={pos_robot[2]:.1f} cm)",
                                (20, 70), _FONT, 0.6, (0, 100, 255), 2)
                    cv2.imshow("TATARO - Camera Feed", annotated)
                    cv2.imshow("Warped Table (top-down)", warped_table)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue

                print(
                    f"[Cube] Robot XYZ (cm): {pos_robot.round(2)}"
                    f"  RPY (deg): {euler.round(1)}"
                    f"  Visible faces: {len(faces)}"
                )

                # ── Draw every detected face on camera feed ───────────────────
                for f in faces:
                    cv2.polylines(annotated, [f['corners'].astype(np.int32)],
                                  True, (0, 200, 255), 2)
                    cv2.drawFrameAxes(annotated, camera_matrix, dist_coeffs,
                                      f['rvec_face'], f['tvec_face'], 2.0)

                # ── Cube centre dot on camera feed ────────────────────────────
                best = max(faces, key=lambda f: f['area'])
                pt   = project_point(best['cube_center_cam'], camera_matrix)
                if pt is not None:
                    cv2.circle(annotated, pt, 7, (0, 255, 0), -1)
                    cv2.putText(annotated, "Cube centre", (pt[0] + 8, pt[1] - 8),
                                _FONT, 0.5, (0, 255, 0), 1)

                # ── Cube centre projected into warped top-down view ───────────
                cube_table = R_cam_table.T @ (
                    best['cube_center_cam'].reshape(3, 1) - Table_corner_3D.reshape(3, 1)
                )
                x_pix = int(cube_table[0, 0] * pixels_per_cm_x)
                y_pix = int(cube_table[1, 0] * pixels_per_cm_y)
                cv2.circle(warped_table, (x_pix, y_pix), 8, (0, 0, 255), -1)
                cv2.putText(
                    warped_table,
                    f"({pos_robot[0]:.1f}, {pos_robot[1]:.1f})",
                    (x_pix + 10, y_pix),
                    _FONT, 0.5, (0, 0, 255), 1,
                )

                # ── Pose overlay on camera feed ───────────────────────────────
                cv2.putText(annotated,
                            f"X={pos_robot[0]:.1f}  Y={pos_robot[1]:.1f}  Z={pos_robot[2]:.1f} cm",
                            (20, 70), _FONT, 0.6, (0, 255, 255), 2)
                cv2.putText(annotated,
                            f"R={euler[0]:.1f}  P={euler[1]:.1f}  Yaw={euler[2]:.1f} deg",
                            (20, 100), _FONT, 0.6, (0, 255, 255), 2)
                cv2.putText(annotated,
                            f"Visible faces: {len(faces)}",
                            (20, 130), _FONT, 0.5, (200, 200, 0), 1)

            else:
                cv2.putText(annotated, "Cube (ID=1, 4x4) not detected",
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
