import cv2
import numpy as np
from TATARO_CV_ROS2Camera import connect_camera, stop_camera, get_frame


# ── Dictionary & geometry constants ───────────────────────────────────────────

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)

marker_length = 5.0   # cm — black ArUco pattern only (total printed 8 cm incl. 1.5 cm white border per side)
axis_length   =  5.0   # cm

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



# ── Pose solving (planar-ambiguity safe) ──────────────────────────────────────

# A flat square seen near face-on has two valid poses (tilted +/- about the view
# axis); plain solvePnP flips between them frame to frame. IPPE_SQUARE returns
# both; keep the clearly better one, otherwise the one closest to last frame.
_half_pat         = marker_length / 2.0
MARKER_OBJ_POINTS = np.array([
    [-_half_pat,  _half_pat, 0],
    [ _half_pat,  _half_pat, 0],
    [ _half_pat, -_half_pat, 0],
    [-_half_pat, -_half_pat, 0],
], dtype=np.float32)

_AMBIGUITY_RATIO = 1.5   # reproj-error ratio below which the two poses count as ambiguous
_prev_rvecs      = {}    # marker_id -> last chosen rvec


def solve_marker_pose(img_points, marker_id):
    """Return (success, rvec, tvec) for one marker's 4 image corners."""
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
        MARKER_OBJ_POINTS, img_points.reshape(4, 2).astype(np.float32),
        camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if n == 0:
        return False, None, None

    best = 0
    errs = np.asarray(errs).flatten()
    prev = _prev_rvecs.get(marker_id)
    if n > 1 and prev is not None and errs[1] < _AMBIGUITY_RATIO * errs[0]:
        R_prev = cv2.Rodrigues(prev)[0]
        dists  = [np.linalg.norm(cv2.Rodrigues(r)[0] - R_prev) for r in rvecs[:n]]
        best   = int(np.argmin(dists))

    _prev_rvecs[marker_id] = rvecs[best]
    return True, rvecs[best], tvecs[best]


# ── Depth sampling ─────────────────────────────────────────────────────────────

def _sample_depth(depth_frame, u, v, radius=10):

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


# ── Marker detection ───────────────────────────────────────────────────────────

def detect_marker_7x7(color_frame, marker_id, depth_frame):
    """
    Detect a single ArUco 7x7 marker by ID.
    Returns (rvec, tvec) with Z overridden by the RealSense depth sensor.
    Returns (None, None) if the marker is not found.

    Parameters
    ----------
    color_frame  : np.ndarray BGR uint8
    marker_id    : int — target ArUco ID
    depth_frame  : np.ndarray uint16 mm (from TATARO_CV_ROS2Camera.get_frame(pipe, return_depth=True))
    """
    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return None, None

    for i, found_id in enumerate(ids.flatten()):
        if found_id != marker_id:
            continue

        markerCorner  = corners[i]
        marker_center = np.mean(markerCorner.reshape(4, 2), axis=0)
        u, v = int(marker_center[0]), int(marker_center[1])

        h, w = depth_frame.shape
        if not (0 <= v < h and 0 <= u < w):
            print(f"[ERROR] Depth pixel ({u}, {v}) out of bounds for marker {marker_id}.")
            return None, None

        z_depth = _sample_depth(depth_frame, u, v, radius=10)
        if z_depth <= 0:
            print(f"[ERROR] No valid depth near ({u}, {v}) for marker {marker_id}.")
            return None, None

        # Derive X, Y directly from the projection formula so tvec round-trips
        # exactly back to pixel (u, v).  solvePnP X/Y would be inconsistent if
        # its internal Z differs from the sensor depth.
        Z  = z_depth / 10.0   # mm -> cm
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        X  = (u - cx) * Z / fx
        Y  = (v - cy) * Z / fy
        tvec = np.array([[X], [Y], [Z]], dtype=np.float64)

        # solvePnP for rotation only (use black-pattern size = marker_length cm)
        success, rvec, _ = solve_marker_pose(markerCorner, marker_id)

        if not success:
            return None, None

        return rvec, tvec

    return None, None


# ── Standalone viewer ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    parameters = cv2.aruco.DetectorParameters()
    detector   = cv2.aruco.ArucoDetector(dictionary, parameters)
    pipe       = connect_camera()

    try:
        while True:
            color_frame, depth_frame = get_frame(pipe, return_depth=True)
            if color_frame is None:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            display = color_frame.copy()

            try:
                gray_frame = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray_frame)

                if ids is not None and len(corners) > 0:
                    ids_flat = ids.flatten()

                    for i, (markerCorner, markerID) in enumerate(zip(corners, ids_flat)):
                        marker_corners = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = marker_corners

                        topLeft     = (int(topLeft[0]),     int(topLeft[1]))
                        topRight    = (int(topRight[0]),    int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft  = (int(bottomLeft[0]),  int(bottomLeft[1]))

                        cv2.line(display, topLeft,     topRight,    (0, 255, 255), 1)
                        cv2.line(display, topRight,    bottomRight, (0, 255, 255), 1)
                        cv2.line(display, bottomRight, bottomLeft,  (0, 255, 255), 1)
                        cv2.line(display, bottomLeft,  topLeft,     (0, 255, 255), 1)

                        cv2.putText(display, str(markerID),
                                    (topLeft[0], topLeft[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        success, rvec, tvec = solve_marker_pose(markerCorner, int(markerID))

                        if success:
                            cv2.drawFrameAxes(display, camera_matrix, dist_coeffs,
                                              rvec, tvec, axis_length)

                            cX = int(markerCorner[:, :, 0].mean())
                            cY = int(markerCorner[:, :, 1].mean())
                            cv2.circle(display, (cX, cY), 6, (0, 0, 255), -1)

            except Exception as exc:
                cv2.putText(display, f"Detection error: {exc}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("ArUco 7x7 Detection - TATARO ROS2", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        stop_camera(pipe)
        cv2.destroyAllWindows()
