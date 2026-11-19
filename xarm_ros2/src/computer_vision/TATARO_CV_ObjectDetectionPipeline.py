"""
TATARO_CV_ObjectDetectionPipeline.py — Colour-cube tracking + gripper
                                        alignment for a parallel gripper
                                        (Gripper Lite), no ArUco tags on
                                        the objects.

Phase 1 : Lock table frame from markers 51 / 52 / 53  (DICT_7X7, done once).
Phase 2 : Every frame, segment cubes by HSV colour inside the table mask,
          lock onto ONE target cube, and express its centre + yaw in the
          robot base frame (camera -> table -> robot via camera_to_robot()).
Phase 3 : (robot mode) Press 'g' in the camera window to start. Background
          thread moves the Lite6 TCP above the target and keeps it there: XY follows the cube, yaw rotates the
          jaws onto the cube's faces — same structure as
          tataro_packages/tataro_marker40_touch_loopV1.0.py (deadbands,
          wait=True, one command in flight). NO descent, NO gripper action.

Grasp convention (matches ufactory_vision/ggcnn_grasping_demo/grasp/robot_grasp.py):
    - Fixed top-down approach: roll/pitch held at the arm's current values,
      only yaw is servoed.
    - Gripper open/close is a separate concern — not commanded here.

Output per detected cube (robot frame):
    position_robot : [X, Y, Z] cm  — cube centre
    yaw_deg         : float        — gripper approach yaw, wrapped to the
                                      cube's 90°-symmetric faces

Usage:
    python3 TATARO_CV_ObjectDetectionPipeline.py [colour] [--no-robot]
        colour      : red / green / blue / yellow  (default: largest cube of any colour)
        --no-robot  : vision only — print + display poses, never connect to the arm

TUNE BEFORE USE:
    - COLOR_RANGES   : HSV thresholds per colour, calibrate under your
                        actual workspace lighting (TATARO_CV_HSVPicker.py).
    - CUBE_HEIGHT_CM : physical cube edge length.
    - MIN_AREA_PX / DEPTH range / Z-robot sanity bounds — same false-positive
      guards as track_cube.py, tune once cubes are on the table.
    - GRIPPER_YAW_OFFSET_DEG : calibrate against the real Gripper Lite —
      the angle between "0 yaw" of the jaws and "0 yaw" of a cube edge as
      minAreaRect defines it. Start at 0 and adjust from observed alignment.
    - TRACK_Z_MM : fixed hover height. The Gripper Lite flange-to-fingertip
      offset is still unmeasured (see tataro_cube_hover_test.py), so this is
      a known-safe height, NOT derived from the cube. Do not lower it until
      that offset is calibrated.
"""

import sys
import time
import threading

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

# ── Cube geometry ──────────────────────────────────────────────────────────
CUBE_HEIGHT_CM = 3.0                    # measured
CUBE_HALF_CM   = CUBE_HEIGHT_CM / 2.0

# ── Colour thresholds (HSV, OpenCV ranges: H 0-179, S/V 0-255) ─────────────
# Calibrated via TATARO_CV_HSVPicker.py against the real cubes under actual
# workspace lighting (red/green/blue). Re-run the picker any time lighting
# changes materially.
COLOR_RANGES = {
    'red':    [((0,   54,  54), (6,   147, 120)),
               # Widened 2026-10-12: cube measured H 174-177, S 127-211,
               # V 101-150 — old (169,172,69)-(179,255,139) kept only ~25%.
               ((165, 110, 60), (179, 255, 200))],
    'green':  [((71,  72,  34), (94,  251, 93))],
    'blue':   [((92,  117, 70), (105, 255, 169))],
    # TODO: 'yellow' not yet sampled — run TATARO_CV_HSVPicker.py with a
    # yellow cube on the table, then paste its printed range in here.
    'yellow': [((20,  100, 100), (35,  255, 255))],   # placeholder, uncalibrated
}

# ── Grasp convention ────────────────────────────────────────────────────────
GRIPPER_YAW_OFFSET_DEG = 0.0   # TODO: calibrate on real hardware

# ── Target locking ──────────────────────────────────────────────────────────
LOCK_RADIUS_CM = 5.0    # once locked, only accept the detection nearest the
                        # last target position, and only within this radius —
                        # stops the tracker hopping to a different cube
LOST_FRAMES    = 15     # consecutive misses before the lock is released
STABLE_N       = 10     # frames that must agree before the first robot move
STABLE_TOL_CM  = 0.5    # max per-axis std-dev across STABLE_N frames (~5 mm)

# ── Robot (same conventions as tataro_marker40_touch_loopV1.0.py) ───────────
ROBOT_IP            = '192.168.1.150'
TRACK_Z_MM          = 300.0   # fixed SDK Z while tracking — highest verified-safe
                              # hover from tataro_cube_hover_test.py; NOT cube-derived
SPEED_APPROACH      = 300.0   # mm/s   first move to the cube
MVACC_APPROACH      = 800.0   # mm/s^2
SPEED_TRACK         = 500.0   # mm/s   continuous tracking moves
MVACC_TRACK         = 3000.0  # mm/s^2
DEADBAND_CM         = 0.3     # min XY change (3 mm) before sending a new move
ORIENT_DEADBAND_DEG = 3.0     # min yaw change before sending a new move
MIN_REACH_MM        = 120.0   # reject targets too close to the base column
MAX_REACH_MM        = 400.0   # Lite6 reach is 440 mm — keep a margin
PRINT_PERIOD_S      = 0.5     # console pose print rate (window still updates every frame)

# ── False-positive rejection (same pattern as track_cube.py) ───────────────
# MIN_AREA_PX was originally copied from track_cube.py's ArUco-face threshold
# without checking real scale here: at whole-workspace zoom a 3cm cube is only
# ~15-20px across on screen (~200-300px^2) — the old 400 floor was silently
# rejecting valid detections whenever the apparent blob dipped even slightly
# (angle/shadow/motion blur), causing intermittent "no detection" flicker.
MIN_AREA_PX   = 80
MIN_DEPTH_MM  = 100
MAX_DEPTH_MM  = 2000
MIN_Z_ROBOT_CM = -5.0
MAX_Z_ROBOT_CM = 15.0


# ── Depth helper (identical to track_cube.py / MarkerDetection_7x7) ────────
def _sample_depth(depth_frame, u, v, radius=8):
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


# ── Colour segmentation ─────────────────────────────────────────────────────
def detect_colored_cubes(color_frame, depth_frame, color_ranges=COLOR_RANGES,
                          table_mask=None):
    """
    Segment each colour, find blob(s), and back-project the top-face centre.

    table_mask : optional uint8 array (frame-sized, 255 inside the table
                 quadrilateral / 0 outside) from _phase1_init_table(). When
                 given, colour matches outside the table are discarded before
                 contour-finding — keeps background objects (posters, shelves,
                 the arm itself) from ever being considered as cube candidates.

    Returns a list of dicts, one per detected cube-top blob:
        color         : str
        u, v          : int          — pixel centroid of the top face
        area          : float        — pixel area (bigger = more reliable)
        top_center_cam: np.ndarray[3]— top-face centre in camera frame (cm)
        yaw_img_deg   : float        — minAreaRect angle in the image plane
        contour       : ndarray
    """
    hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    h_d, w_d = depth_frame.shape

    # Small kernel: at a ~15-20px cube footprint, a 5x5 open/close was eroding
    # a marginal blob away before the area check even ran.
    kernel = np.ones((3, 3), np.uint8)
    detections = []

    for name, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        if table_mask is not None:
            mask &= table_mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA_PX:
                continue

            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            u, v = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

            if not (0 <= v < h_d and 0 <= u < w_d):
                continue

            z_depth = _sample_depth(depth_frame, u, v, radius=8)
            if not (MIN_DEPTH_MM < z_depth < MAX_DEPTH_MM):
                continue

            Z = z_depth / 10.0   # mm -> cm
            top_center_cam = np.array([
                (u - cx) * Z / fx,
                (v - cy) * Z / fy,
                Z,
            ], dtype=np.float64)

            # In-plane rotation of the blob (0-90 deg, cube is 90 deg symmetric)
            (_, _), (_, _), angle = cv2.minAreaRect(c)

            detections.append({
                'color': name,
                'u': u, 'v': v,
                'area': area,
                'top_center_cam': top_center_cam,
                'yaw_img_deg': angle,
                'contour': c,
            })

    return detections


# ── Robot-frame pose ─────────────────────────────────────────────────────────
def cube_pose_in_robot_frame(det, Table_corner_3D, rvec_marker_wrt_camera):
    """
    Same transform chain as marker40_pose_in_robot_frame() / track_cube.py's
    cube_pose_in_robot_frame(): camera -> table -> robot.

    Position: top-face centre offset down by CUBE_HALF_CM along the table
    normal (approx. the camera's -Z as seen through the table rotation),
    to get the cube's volumetric centre instead of its top face.

    Yaw: the blob's image-plane rotation, carried through the same
    R_robot_cam chain as an in-plane (Z-axis) rotation, then wrapped to the
    cube's 90 deg symmetry and offset by GRIPPER_YAW_OFFSET_DEG.
    """
    R_cam_table, _ = cv2.Rodrigues(rvec_marker_wrt_camera)
    R_robot_cam = R_robot_table @ R_cam_table.T

    # Cube centre = top-face centre, pushed down (toward table) by half height.
    # Table normal in camera frame is the table frame's Z axis: R_cam_table[:, 2].
    table_normal_cam = R_cam_table[:, 2]
    cube_center_cam = det['top_center_cam'] - table_normal_cam * CUBE_HALF_CM

    pos_robot = camera_to_robot(cube_center_cam, Table_corner_3D, rvec_marker_wrt_camera)

    # Carry the image-plane yaw through the same rotation chain as a
    # pure Z-axis (camera optical axis) rotation — valid because the D455
    # looks ~straight down at the table.
    R_obj_cam, _ = cv2.Rodrigues(
        np.array([0, 0, np.deg2rad(det['yaw_img_deg'])])
    )
    R_obj_robot = R_robot_cam @ R_obj_cam
    euler = Rot.from_matrix(R_obj_robot).as_euler('xyz', degrees=True)

    # Cube is 90 deg symmetric -> wrap into [0, 90), then apply the
    # jaws-vs-edge calibration offset.
    yaw_deg = (euler[2] % 90.0) + GRIPPER_YAW_OFFSET_DEG

    return pos_robot, yaw_deg


# ── Phase 1 — lock table frame (identical to track_cube.py) ─────────────────
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

        # Frame-sized binary mask, 255 inside the table quadrilateral / 0
        # outside — lets Phase 2 restrict colour search to just the table.
        h_f, w_f = color_frame.shape[:2]
        table_mask = np.zeros((h_f, w_f), dtype=np.uint8)
        cv2.fillConvexPoly(table_mask, src_pts.astype(np.int32), 255)
        # Erode inward: the 51/52/53 corner markers sit AT the polygon's own
        # vertices, so the raw mask doesn't exclude them. Their black/white
        # squares have noisy, unstable hue at low saturation/value and can
        # randomly false-match a colour range. Shrinking the mask pushes the
        # boundary away from the markers while still covering the interior
        # workspace where cubes actually sit.
        # TODO: 100px is a rough guess from on-screen marker size in early
        # testing, not measured — tighten/loosen if real cubes near the table
        # edge get clipped, or if marker false-positives still get through.
        table_mask = cv2.erode(table_mask, np.ones((100, 100), np.uint8))

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
            'table_mask':             table_mask,
        }


# ── Target selection / locking ──────────────────────────────────────────────
def _select_target(candidates, locked_pos, target_color):
    """
    candidates : list of (det, pos_robot, yaw_deg) that passed the Z guard.
    locked_pos : last target position (cm) or None if not locked yet.

    Not locked -> largest blob of target_color (any colour if None).
    Locked     -> the candidate nearest locked_pos, within LOCK_RADIUS_CM.
    """
    if target_color is not None:
        candidates = [c for c in candidates if c[0]['color'] == target_color]
    if not candidates:
        return None

    if locked_pos is None:
        return max(candidates, key=lambda c: c[0]['area'])

    nearest = min(candidates, key=lambda c: np.linalg.norm(c[1] - locked_pos))
    if np.linalg.norm(nearest[1] - locked_pos) > LOCK_RADIUS_CM:
        return None
    return nearest


def _nearest_equivalent_yaw(cube_yaw_deg, ref_yaw_deg):
    """
    The cube is 90 deg symmetric, so cube_yaw + k*90 are all valid grasps.
    Return the one closest to ref_yaw_deg (the gripper's current yaw) so the
    wrist never rotates more than 45 deg to align.
    """
    k = np.round((ref_yaw_deg - cube_yaw_deg) / 90.0)
    return float(cube_yaw_deg + k * 90.0)


# ── Robot helpers (same as tataro_marker40_touch_loopV1.0.py) ───────────────
def _connect_robot():
    from xarm.wrapper import XArmAPI

    print(f"[ARM]  Connecting to {ROBOT_IP} ...")
    arm = XArmAPI(ROBOT_IP, is_radian=False)
    code, _ = arm.get_state()
    if code != 0:
        raise RuntimeError(f"Cannot reach robot at {ROBOT_IP} (code={code})")
    _recover(arm)
    print("[ARM]  Connected and ready.")
    return arm


def _recover(arm):
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)


def _reachable(x_mm, y_mm):
    r = float(np.hypot(x_mm, y_mm))
    return MIN_REACH_MM <= r <= MAX_REACH_MM


# ── Phase 3 — robot tracking thread ──────────────────────────────────────────
def _set_status(shared, text):
    with shared['lock']:
        shared['robot_status'] = text
    print(f"[ROBOT] {text}", flush=True)


def _robot_loop(arm, shared, stop):
    """
    Background thread. Waits for 'g' pressed in the camera window (the main
    thread sets shared['go']) and a stable, reachable target, then keeps the
    TCP at TRACK_Z_MM above the cube with the jaws aligned.

    Reads shared['latest_pos'] / ['latest_yaw'] (updated every camera frame
    by the main thread) and shared['stable_pos'] (first approach only).
    wait=True keeps exactly one move in flight — no queue build-up.
    """
    go = shared['go']

    # ── Wait for 'g', then a stable + reachable target ─────────────────────
    while not stop.is_set():
        _set_status(shared, "press 'g' in camera window to start")
        while not stop.is_set() and not go.wait(0.1):
            pass
        if stop.is_set():
            return

        _set_status(shared, "waiting for stable target ...")
        while not stop.is_set():
            with shared['lock']:
                first_pos = shared['stable_pos']
                first_yaw = shared['latest_yaw']
            if first_pos is not None and first_yaw is not None:
                break
            time.sleep(0.1)
        if stop.is_set():
            return

        x_mm = float(first_pos[0]) * 10.0
        y_mm = float(first_pos[1]) * 10.0
        if _reachable(x_mm, y_mm):
            break
        _set_status(shared, f"target out of reach ({np.hypot(x_mm, y_mm):.0f} mm, "
                            f"allowed {MIN_REACH_MM:.0f}-{MAX_REACH_MM:.0f}) — move cube, press 'g'")
        go.clear()
        time.sleep(2.0)
    if stop.is_set():
        return

    _recover(arm)
    _, cur = arm.get_position()
    roll, pitch, cur_yaw = cur[3], cur[4], cur[5]
    yaw = _nearest_equivalent_yaw(first_yaw, cur_yaw)

    print(f"[ARM]  Current TCP : ({cur[0]:.1f}, {cur[1]:.1f}, {cur[2]:.1f}) mm  "
          f"RPY = ({roll:.1f}, {pitch:.1f}, {cur_yaw:.1f}) deg")
    print(f"[PLAN] First target: X={x_mm:.1f}  Y={y_mm:.1f}  Z={TRACK_Z_MM:.1f} mm "
          f"(fixed hover)  yaw={yaw:.1f} deg")
    _set_status(shared, "approaching target ...")

    code = arm.set_position(x_mm, y_mm, TRACK_Z_MM, roll, pitch, yaw,
                            speed=SPEED_APPROACH, mvacc=MVACC_APPROACH, wait=True)
    if code != 0:
        _set_status(shared, f"approach move failed (code={code}) — tracking stopped")
        return
    _set_status(shared, "TRACKING — move the cube to follow")
    print("-" * 60)

    last_xy, last_yaw = first_pos[:2].copy(), yaw
    lost_warned       = False

    # ── Continuous tracking ─────────────────────────────────────────────────
    while not stop.is_set():
        with shared['lock']:
            pos     = shared['latest_pos']
            cube_yw = shared['latest_yaw']

        if pos is None or cube_yw is None:
            if not lost_warned:
                print("[TRACK]  Target not visible — holding position.", flush=True)
                lost_warned = True
            time.sleep(0.05)
            continue
        lost_warned = False

        xy          = pos[:2].copy()
        desired_yaw = _nearest_equivalent_yaw(cube_yw, last_yaw)

        xy_changed  = np.linalg.norm(xy - last_xy) >= DEADBAND_CM
        yaw_changed = abs(desired_yaw - last_yaw) >= ORIENT_DEADBAND_DEG
        if not xy_changed and not yaw_changed:
            time.sleep(0.02)
            continue

        x_mm = float(pos[0]) * 10.0
        y_mm = float(pos[1]) * 10.0
        if not _reachable(x_mm, y_mm):
            print(f"[TRACK]  Target out of reach ({np.hypot(x_mm, y_mm):.0f} mm) "
                  "— holding position.", flush=True)
            time.sleep(0.2)
            continue

        code = arm.set_position(x_mm, y_mm, TRACK_Z_MM, roll, pitch, desired_yaw,
                                speed=SPEED_TRACK, mvacc=MVACC_TRACK, wait=True)
        if code == 0:
            last_xy, last_yaw = xy, desired_yaw
            print(f"[TRACK]  TCP -> ({x_mm:.1f}, {y_mm:.1f}, {TRACK_Z_MM:.1f}) mm  "
                  f"yaw = {desired_yaw:.1f} deg", flush=True)
        else:
            print(f"[WARN]   Move failed (code={code}) — recovering ...", flush=True)
            _recover(arm)


# ── Phase 2 — colour-cube tracking loop (main thread: camera + display) ─────
def run_pipeline(target_color=None, use_robot=True):
    pipe       = connect_camera()
    arm        = None
    stop_event = threading.Event()
    shared = {
        'lock':       threading.Lock(),
        'latest_pos': None,   # target centre (robot frame, cm) — every frame
        'latest_yaw': None,   # target yaw (robot frame, deg)  — every frame
        'stable_pos': None,   # mean over STABLE_N agreeing frames — first approach
        'go':         threading.Event(),   # set by 'g' in the camera window
        'robot_status': 'OFF' if not use_robot else 'connecting ...',
    }

    try:
        table = _phase1_init_table(pipe)
        if table is None:
            print("[Abort] Table initialisation cancelled.")
            return

        rvec_marker_wrt_camera = table['rvec_marker_wrt_camera']
        Table_corner_3D        = table['Table_corner_3D']
        M                      = table['M']
        warped_size            = table['warped_size']
        table_mask             = table['table_mask']

        if use_robot:
            arm = _connect_robot()
            threading.Thread(target=_robot_loop, args=(arm, shared, stop_event),
                             daemon=True).start()

        print("[Phase 2] Tracking "
              f"{target_color or 'largest cube (any colour)'} "
              f"({', '.join(COLOR_RANGES)}) — press 'q' in the window or Ctrl+C to quit.")

        locked_pos = None
        misses     = 0
        history    = []
        last_print = 0.0

        while True:
            color_frame, depth_frame = get_frame(pipe, return_depth=True)
            if color_frame is None or depth_frame is None:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            annotated    = color_frame.copy()
            warped_table = cv2.warpPerspective(color_frame, M, warped_size)

            # ── Detect + robot-frame pose for every cube on the table ───────
            candidates = []
            for det in detect_colored_cubes(color_frame, depth_frame, table_mask=table_mask):
                pos_robot, yaw_deg = cube_pose_in_robot_frame(
                    det, Table_corner_3D, rvec_marker_wrt_camera
                )
                if MIN_Z_ROBOT_CM < pos_robot[2] < MAX_Z_ROBOT_CM:
                    candidates.append((det, pos_robot, yaw_deg))
                    cv2.drawContours(annotated, [det['contour']], -1, (0, 200, 255), 1)

            target = _select_target(candidates, locked_pos, target_color)

            # ── Update lock + shared state for the robot thread ─────────────
            if target is None:
                misses += 1
                history.clear()
                if misses >= LOST_FRAMES:
                    locked_pos = None
                with shared['lock']:
                    shared['latest_pos'] = None
                    shared['latest_yaw'] = None
                    shared['stable_pos'] = None
            else:
                det, pos_robot, yaw_deg = target
                misses     = 0
                locked_pos = pos_robot.copy()

                history.append(pos_robot.copy())
                if len(history) > STABLE_N:
                    history.pop(0)
                stable = None
                if len(history) == STABLE_N:
                    pts = np.array(history)
                    if float(np.max(np.std(pts, axis=0))) < STABLE_TOL_CM:
                        stable = pts.mean(axis=0)

                with shared['lock']:
                    shared['latest_pos'] = pos_robot.copy()
                    shared['latest_yaw'] = float(yaw_deg)
                    shared['stable_pos'] = stable

                tag = '[STABLE]' if stable is not None else '[CV]    '
                if time.time() - last_print >= PRINT_PERIOD_S:
                    last_print = time.time()
                    print(f"{tag} {det['color']} Robot XYZ (cm): {pos_robot.round(2)}"
                          f"  Yaw (deg): {yaw_deg:.1f}  Area: {det['area']:.0f}", flush=True)

                cv2.drawContours(annotated, [det['contour']], -1, (0, 255, 0), 2)
                cv2.circle(annotated, (det['u'], det['v']), 6, (0, 255, 0), -1)
                cv2.putText(annotated,
                            f"TARGET {det['color']} X={pos_robot[0]:.1f} Y={pos_robot[1]:.1f} "
                            f"Z={pos_robot[2]:.1f} yaw={yaw_deg:.0f}",
                            (det['u'] + 8, det['v'] - 8), _FONT, 0.5, (0, 255, 0), 1)

                pt_w = cv2.perspectiveTransform(np.float32([[[det['u'], det['v']]]]), M)
                pw = tuple(pt_w[0, 0].astype(int))
                cv2.circle(warped_table, pw, 8, (0, 0, 255), -1)
                cv2.putText(warped_table, det['color'], (pw[0] + 6, pw[1] - 6),
                            _FONT, 0.5, (0, 0, 255), 1)

            status = ("LOCKED" if target is not None
                      else ("SEARCHING" if locked_pos is None else "LOST (holding)"))
            cv2.putText(annotated,
                        f"Phase 2: Table locked | Target {status}",
                        (20, 40), _FONT, 0.6, (0, 255, 0) if target is not None else (0, 0, 255), 2)
            with shared['lock']:
                robot_status = shared['robot_status']
            cv2.putText(annotated, f"Robot: {robot_status}   ('g' = start, 'q' = quit)",
                        (20, 70), _FONT, 0.6, (255, 255, 0), 2)

            cv2.imshow("TATARO - Camera Feed", annotated)
            cv2.imshow("Warped Table (top-down)", warped_table)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('g') and use_robot:
                shared['go'].set()

    except KeyboardInterrupt:
        print("\n[Exit] Ctrl+C received — shutting down ...")

    finally:
        stop_event.set()
        if arm is not None:
            arm.set_state(4)        # stop any move in flight
            arm.disconnect()
            print("[ARM]  Stopped and disconnected.")
        stop_camera(pipe)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    args  = [a for a in sys.argv[1:] if not a.startswith('--')]
    color = args[0] if args else None
    if color is not None and color not in COLOR_RANGES:
        sys.exit(f"Unknown colour '{color}'. Choose from: {', '.join(COLOR_RANGES)}")
    run_pipeline(target_color=color, use_robot='--no-robot' not in sys.argv)
