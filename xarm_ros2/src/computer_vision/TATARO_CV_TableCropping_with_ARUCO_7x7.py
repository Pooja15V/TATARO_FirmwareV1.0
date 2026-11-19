

import cv2
import numpy as np
from TATARO_CV_ROS2Camera        import connect_camera, stop_camera, get_frame
from TATARO_CV_MarkerDetection_7x7 import detect_marker_7x7
from TATARO_CV_GSO               import gram_schmidt_orthonormalization


# ── Dictionary & table geometry constants ──────────────────────────────────────

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)

MARKER_SIZE_CM  = 5.0           # black ArUco pattern only (cm)
BORDER_OFFSET_CM = 1.5          # white border around marker (cm)

TABLE_WIDTH_CM  = 85
TABLE_HEIGHT_CM = 85

# Camera calibration parameters (from /camera/camera/color/camera_infofor 640x480)
# camera_matrix = np.array([
#     [387.464,   0,     323.627],
#     [  0,     387.464, 241.00 ],
#     [  0,       0,       1    ],
# ])
# dist_coeffs = np.zeros(5)


# Camera calibration parameters (from /camera/camera/color/camera_info — D435i 1280x720)
camera_matrix = np.array([
    [644.9842529296875, 0, 650.7435302734375],
    [0, 643.9988403320312, 363.3885192871094],
    [0, 0, 1]
], dtype=np.float64)
dist_coeffs = np.array([
    -0.05567498505115509,
     0.06433625519275665,
    -0.00040179200004786253,
     0.00031893057166598737,
    -0.02026429958641529
], dtype=np.float64)

# ── Geometry helpers ───────────────────────────────────────────────────────────

def project_point(P, camera_matrix):
    P  = P.flatten()
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    if P[2] == 0:
        return None
    x = (P[0] / P[2]) * fx + cx
    y = (P[1] / P[2]) * fy + cy
    return int(x), int(y)


def perspective_transformation(frame, P1_2D, P2_2D, P3_2D, P4_2D,
                                TABLE_WIDTH_CM, TABLE_HEIGHT_CM, pixels_per_cm):
    width_pixel  = int(TABLE_WIDTH_CM  * pixels_per_cm)
    height_pixel = int(TABLE_HEIGHT_CM * pixels_per_cm)
    src_pts = np.float32([P1_2D, P2_2D, P3_2D, P4_2D])
    dst_pts = np.float32([
        [0,           0           ],
        [width_pixel, 0           ],
        [width_pixel, height_pixel],
        [0,           height_pixel],
    ])
    matrix       = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped_table = cv2.warpPerspective(frame, matrix, (width_pixel, height_pixel))
    return warped_table


def compute_scale(P1_2D, P2_2D, camera_matrix):
    table_width_pixels = np.linalg.norm(np.array(P2_2D) - np.array(P1_2D))
    fx     = camera_matrix[0, 0]
    fy     = camera_matrix[1, 1]
    scale_x = fx / table_width_pixels
    scale_y = fy / table_width_pixels
    return (scale_x + scale_y) / 2


def draw_axes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, marker_size_cm=5):
    for i in range(len(rvecs)):
        marker_center_3D = tvecs[i].reshape(3, 1)
        marker_center_2D = project_point(marker_center_3D, camera_matrix)
        x_offset = int(marker_size_cm * 2)
        y_offset = int(marker_size_cm * 2)
        pt1 = (marker_center_2D[0] - x_offset, marker_center_2D[1] - y_offset)
        pt2 = (marker_center_2D[0] + x_offset, marker_center_2D[1] + y_offset)
        pt3 = (marker_center_2D[0] - x_offset, marker_center_2D[1] + y_offset)
        pt4 = (marker_center_2D[0] + x_offset, marker_center_2D[1] - y_offset)
        cv2.line(frame, pt1, pt2, (0, 255, 255), 2)
        cv2.line(frame, pt3, pt4, (0, 255, 255), 2)
    return frame


def apply_polygon_mask(frame, P1_2D, P2_2D, P3_2D, P4_2D):
    if None in [P1_2D, P2_2D, P3_2D, P4_2D]:
        print("Error: Invalid or missing table area points.")
        return frame
    points = np.array([P1_2D, P2_2D, P3_2D, P4_2D], dtype=np.int32).reshape((-1, 1, 2))
    mask   = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [points], 255)
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
    return masked_frame



_detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def detect_and_transform_table_using_aruco(color_frame, depth_frame):
    """
    Single-frame attempt to detect markers 51/52/53 and compute the warped table.
    Returns the result tuple on success, or None if any marker is missing.
    """
    gray_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
    _, ids, _ = _detector.detectMarkers(gray_frame)
    if ids is None:
        return None

    # Marker 51 → table BOTTOM-LEFT corner
    rvec_marker1, tvec_marker1 = detect_marker_7x7(color_frame, 51, depth_frame)

    # Marker 52 → table BOTTOM-RIGHT corner
    rvec_marker2, tvec_marker2 = detect_marker_7x7(color_frame, 52, depth_frame)

    # Marker 53 → table TOP-LEFT corner
    rvec_marker3, tvec_marker3 = detect_marker_7x7(color_frame, 53, depth_frame)

    if tvec_marker1 is None or tvec_marker2 is None or tvec_marker3 is None:
        return None

    # GSO: (bottom_left=51, bottom_right=52, top_left=53)
    homogeneous_transformation_matrix = gram_schmidt_orthonormalization(
        tvec_marker1, tvec_marker3, tvec_marker2
    )
    tvec_marker_wrt_camera = homogeneous_transformation_matrix[:3, 3]
    rm_marker_wrt_camera   = homogeneous_transformation_matrix[:3, :3]
    rvec_marker_wrt_camera, _ = cv2.Rodrigues(rm_marker_wrt_camera)

    # Rotation of the bottom-left marker (51) in its own frame
    rm_marker51_wrt_camera, _ = cv2.Rodrigues(rvec_marker1)
    rm_marker52_wrt_camera, _ = cv2.Rodrigues(rvec_marker2)
    rm_marker53_wrt_camera, _ = cv2.Rodrigues(rvec_marker3)

    # Table corner (center of table, at marker 51 center minus half marker + border)
    corner_offset = np.array([
        [-MARKER_SIZE_CM / 2],
        [-MARKER_SIZE_CM / 2],
        [0],
    ])
    Table_corner_3D = tvec_marker1.reshape(3, 1) + (rm_marker51_wrt_camera @ corner_offset)
    # print("Table corner =", Table_corner_3D)

    # P1: marker 51 (bottom-left) - offset outward from black pattern + border
    bottomLeft_offset = np.array([
        [-MARKER_SIZE_CM / 2 - BORDER_OFFSET_CM],
        [-MARKER_SIZE_CM / 2 - BORDER_OFFSET_CM],
        [0],
    ])
    P1_3D = tvec_marker1.reshape(3, 1) + (rm_marker51_wrt_camera @ bottomLeft_offset)

    # P2: marker 52 (bottom-right) - offset outward in X, inward in Y
    bottomRight_offset = np.array([
        [MARKER_SIZE_CM / 2 + BORDER_OFFSET_CM],
        [-MARKER_SIZE_CM / 2 - BORDER_OFFSET_CM],
        [0],
    ])
    P2_3D = tvec_marker2.reshape(3, 1) + (rm_marker52_wrt_camera @ bottomRight_offset)

    # P4: marker 53 (top-left) - offset outward in Y, inward in X
    topLeft_offset = np.array([
        [-MARKER_SIZE_CM / 2 - BORDER_OFFSET_CM],
        [MARKER_SIZE_CM / 2 + BORDER_OFFSET_CM],
        [0],
    ])
    P4_3D = tvec_marker3.reshape(3, 1) + (rm_marker53_wrt_camera @ topLeft_offset)

    # P3: top-right corner (derive from P2 and P4)
    P3_3D = P2_3D + (P4_3D - P1_3D)

    # Project 3-D table corners to 2-D image coordinates
    P1_2D = project_point(P1_3D.flatten(), camera_matrix)
    P2_2D = project_point(P2_3D.flatten(), camera_matrix)
    P3_2D = project_point(P3_3D.flatten(), camera_matrix)
    P4_2D = project_point(P4_3D.flatten(), camera_matrix)

    if None in [P1_2D, P2_2D, P3_2D, P4_2D]:
        print("[ERROR] Table projection failed due to invalid 3D points.")
        return None
    
    # print("Marker51 center :", tvec_marker1.ravel())
    # print("Table_corner_3D :", Table_corner_3D.ravel())
    # print("Offset:", (Table_corner_3D - tvec_marker1).ravel())

    # Draw table boundary on the color frame
    cv2.line(color_frame, P1_2D, P2_2D, (255, 0, 0), 2)
    cv2.line(color_frame, P1_2D, P4_2D, (255, 0, 0), 2)
    cv2.line(color_frame, P2_2D, P3_2D, (255, 0, 0), 2)
    cv2.line(color_frame, P4_2D, P3_2D, (255, 0, 0), 2)
    cv2.drawFrameAxes(color_frame, camera_matrix, dist_coeffs,
                      rvec_marker_wrt_camera, P1_3D, 10)

    masked_table = apply_polygon_mask(color_frame, P1_2D, P2_2D, P3_2D, P4_2D)

    corner_pixel = project_point(Table_corner_3D.flatten(), camera_matrix)
    if corner_pixel is not None:
        cv2.circle(masked_table, corner_pixel, 8, (0, 255, 255), -1)

    marker_width_pixels = np.linalg.norm(np.array(P1_2D) - np.array(P2_2D))
    pixels_per_cm       = marker_width_pixels / TABLE_WIDTH_CM
    warped_table        = perspective_transformation(
        color_frame, P1_2D, P2_2D, P3_2D, P4_2D,
        TABLE_WIDTH_CM, TABLE_HEIGHT_CM, pixels_per_cm
    )

    return (
        color_frame,
        masked_table,
        warped_table,
        P1_2D, P2_2D, P3_2D, P4_2D,
        rvec_marker_wrt_camera,
        tvec_marker_wrt_camera,
        rm_marker51_wrt_camera,
        Table_corner_3D,
    )


# ── Standalone viewer ──────────────────────────────────────────────────────────

_FONT = cv2.FONT_HERSHEY_SIMPLEX

def main():
    pipe = connect_camera()
    try:
        while True:
            color_frame, depth_frame = get_frame(pipe, return_depth=True)
            cv2.imshow("TATARO - Camera Feed", color_frame)

            if color_frame is None or depth_frame is None:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            display = color_frame.copy()

            try:
                result = detect_and_transform_table_using_aruco(color_frame, depth_frame)
            except Exception as e:
                print(f"[ERROR] Detection failed: {e}")
                result = None

            if result is not None:
                (annotated, masked_table, warped_table,
                 P1_2D, P2_2D, P3_2D, P4_2D,
                 rvec_marker_wrt_camera, tvec_marker_wrt_camera,
                 rm_marker51_wrt_camera, Table_corner_3D) = result

                cv2.imshow("TATARO - Camera Feed",      annotated)
                cv2.imshow("Table alone",               masked_table)
                cv2.imshow("Warped Table (top-down)",   warped_table)
            else:
                cv2.putText(display, "Searching for markers 51, 52, 53...",
                            (20, 40), _FONT, 0.7, (0, 165, 255), 2)
                cv2.imshow("TATARO - Camera Feed", display)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        stop_camera(pipe)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
