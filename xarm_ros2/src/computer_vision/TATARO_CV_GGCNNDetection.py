"""
TATARO_CV_GGCNNDetection.py — GGCNN-based grasp-point detection for the
static D455, adapted from xArm-Developer/ufactory_vision's
ggcnn_grasping_demo/grasp/ggcnn_torch.py.

Separate from, and does NOT modify, TATARO_CV_ObjectDetectionPipeline.py's
colour-cube pipeline — that stays as-is for cubes. This module is the
general-purpose path for objects that aren't reliably colour-distinguishable
(cylinders etc.), and works from shape/depth alone.

Key differences from upstream (all deliberate, for a STATIC eye-to-hand
camera instead of their eye-in-hand wrist camera):
  - No threading/queue: single-shot inference per call.
  - No robot_z-dependent local-max tracking (that logic exists purely to
    smooth an eye-in-hand camera's improving view as the arm approaches) --
    always take the global-max grasp point within each candidate crop.
  - Output is fed through TATARO's existing camera_to_robot() +
    R_robot_table rotation chain (identical to how detect_colored_cubes /
    cube_pose_in_robot_frame work), not ufactory's live
    EULER_EEF_TO_COLOR_OPT eye-in-hand pose composition.
  - Localize-then-zoom: GGCNN was trained on Cornell Grasping Dataset
    close-up crops where the object fills a good fraction of a 300x300
    patch. At this camera's whole-workspace zoom a cube is only ~15-20px
    across in the full frame -- feeding a fixed raw-frame crop (as upstream
    does, tuned for a close eye-in-hand view) would show the model a tiny
    speck in mostly background, nothing like its training distribution.
    Instead: find_candidate_regions() locates rough object blobs from depth
    alone (no colour needed -- "what's sitting higher than the table"), then
    each candidate gets a small window cropped tightly around it and
    upscaled to the model's expected 300x300 input. The final 3D position
    is computed from the true original-frame pixel + real depth reading,
    not the resized crop, so this zoom step does not distort the geometry
    -- it only helps the model see something resembling its training scale.
    Angle is preserved exactly under this uniform resize (isotropic scaling
    doesn't change in-plane rotation); width is in the *resized* crop's
    pixel space and is not currently converted to a real-world size.

Model: GGCNN (Cornell-dataset pretrained, ggcnn/models/ggcnn_epoch_23_cornell)
-- NOT fine-tuned on this workspace. Validate visually (see
TATARO_CV_GGCNNViewer.py) before trusting it for real motion.

Grasp angle convention: the model outputs an angle via
atan2(sin(2*theta), cos(2*theta))/2, range [-pi/2, pi/2) -- inherently
180-degree periodic, matching a parallel gripper's own symmetry. Unlike the
cube pipeline's 90-degree wrap (cube-specific symmetry), no extra mod-90
wrapping is applied here.
"""

import os
import sys

import cv2
import numpy as np
import scipy.ndimage as ndimage
import torch
from scipy.spatial.transform import Rotation as Rot

_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_GGCNN_DIR = os.path.join(_THIS_DIR, 'ggcnn')
if _GGCNN_DIR not in sys.path:
    sys.path.insert(0, _GGCNN_DIR)

from TATARO_CV_MarkerDetection_7x7 import camera_matrix
from TATARO_CV_Transformations import camera_to_robot, R_robot_table

MODEL_PATH = os.path.join(_GGCNN_DIR, 'models', 'ggcnn_epoch_23_cornell')

MODEL_INPUT_SIZE = 300   # px -- fixed by the GGCNN architecture

# TODO: tune ZOOM_WINDOW_PX against real objects/lighting. This is the size
# (in ORIGINAL frame pixels) of the window cropped around each candidate
# before upscaling to MODEL_INPUT_SIZE. Too small -> object fills the whole
# crop with no context margin for the grasp rectangle. Too large -> back to
# the original "tiny speck in mostly background" problem. Start with
# something a few times the object's on-screen size (~15-20px cubes at this
# zoom -> start around 80-120px) and adjust after visual validation.
ZOOM_WINDOW_PX = 100

# ── Depth-based candidate localisation (no colour) ──────────────────────────
MIN_CANDIDATE_AREA_PX = 40    # smaller than the colour pipeline's floor --
                               # this only needs to seed a crop centre, not
                               # itself be a precise detection
MAX_CANDIDATE_AREA_PX = 4000  # TODO: tune against real objects. Confirmed via
                               # live camera testing (2026-08-14): a genuine
                               # ~24000px^2 blob shows up consistently near the
                               # table's edge, most likely a depth-sensor
                               # artifact (stereo noise from reflectance/
                               # texture at that spot) rather than any bug in
                               # the local-baseline math (verified clean on
                               # synthetic boundary tests). A real cube/
                               # cylinder at this camera's working distance is
                               # roughly 200-3000px^2 -- this cap rejects
                               # oversized artifacts generically, without
                               # hardcoding a pixel location tied to this one
                               # camera/table arrangement.
HEIGHT_ABOVE_TABLE_MM = 15    # min residual above the LOCAL baseline (mm) to
                               # count as "an object". A real 3cm cube's true
                               # signal is ~30mm, so this keeps solid margin
                               # below that while rejecting a few mm of
                               # residual sensor noise.
BASELINE_KERNEL_PX = 151      # box-blur kernel used to estimate the local
                               # table baseline -- must be well larger than
                               # any real object's footprint (~15-60px here)
                               # so an object only dilutes, not dominates,
                               # its own local baseline.
MAX_CANDIDATES = 5


def find_candidate_regions(depth_frame_mm, table_mask=None,
                            min_area_px=MIN_CANDIDATE_AREA_PX,
                            max_area_px=MAX_CANDIDATE_AREA_PX,
                            height_thresh_mm=HEIGHT_ABOVE_TABLE_MM,
                            baseline_kernel_px=BASELINE_KERNEL_PX,
                            max_candidates=MAX_CANDIDATES):
    """
    Locate rough object blobs from depth alone -- deliberately colour-blind,
    so it generalises to cylinders / anything else, not just calibrated
    colours. "Object" = a contiguous region whose depth is meaningfully
    closer to the camera than the LOCAL table baseline (i.e. sticking up off
    the table surface), restricted to table_mask if given.

    Uses a LOCAL baseline (large-kernel box-blurred depth), not a single
    global scalar. A real table surface is rarely perfectly planar (wear,
    slight warping) -- a single global median falsely flags any section
    that's locally low relative to the WHOLE table's median even with
    nothing on it, and can bury a small object's real signal in that same
    global noise floor. A local baseline acts as a high-pass filter: it
    tracks gradual table-level variation while still flagging small,
    localised bumps (actual objects). cv2.blur is used rather than a large
    median blur because cv2.medianBlur only supports kernel sizes 3 or 5 on
    16-bit input -- a box filter has no such limit and is the right tool
    here regardless (its own smoothing IS the point).

    Returns a list of dicts: {'u', 'v', 'area'}, sorted largest-area first,
    capped at max_candidates. NOT a final detection -- just a seed point for
    GGCNNDetector.detect_at()'s zoomed crop.
    """
    depth_f = depth_frame_mm.astype(np.float32)
    valid = depth_frame_mm > 0
    if table_mask is not None:
        valid &= (table_mask > 0)

    if not np.any(valid):
        return []

    # True masked local average via normalised box filtering: sum(depth *
    # valid) / sum(valid) in each window. This is NOT the same as filling
    # invalid pixels with a constant then blurring -- a constant fill creates
    # a sharp value cliff at every mask boundary (not just corners), and a
    # wide box blur bleeds that cliff up to ~kernel_radius px into the valid
    # region as a spurious residual, mimicking a fake "object" near ANY
    # table edge. Masked averaging only ever combines genuinely valid
    # samples, so it has no boundary artifact.
    valid_f = valid.astype(np.float32)
    depth_masked = depth_f * valid_f

    k = baseline_kernel_px | 1
    sum_depth = cv2.boxFilter(depth_masked, ddepth=-1, ksize=(k, k), normalize=False)
    sum_valid = cv2.boxFilter(valid_f, ddepth=-1, ksize=(k, k), normalize=False)

    local_baseline = np.zeros_like(depth_f)
    has_support = sum_valid > 1e-3   # windows with at least ~1 valid sample
    local_baseline[has_support] = sum_depth[has_support] / sum_valid[has_support]

    # Positive residual = this pixel is closer to the camera than its own
    # local neighbourhood's baseline = sticking up off the table.
    residual = local_baseline - depth_f

    object_mask = np.zeros(depth_frame_mm.shape, dtype=np.uint8)
    object_mask[valid & has_support & (residual > height_thresh_mm)] = 255

    kernel = np.ones((3, 3), np.uint8)
    object_mask = cv2.morphologyEx(object_mask, cv2.MORPH_OPEN, kernel)
    object_mask = cv2.morphologyEx(object_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area_px or area > max_area_px:
            continue
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        u, v = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
        candidates.append({'u': u, 'v': v, 'area': area})

    candidates.sort(key=lambda d: d['area'], reverse=True)
    return candidates[:max_candidates]


# ── Depth-patch preprocessing (adapted from ufactory_vision's ggcnn_torch.py) ─
def _process_depth_patch(depth_m_patch, out_size=MODEL_INPUT_SIZE):
    """
    depth_m_patch: float32 metres, NaN where invalid. Inpaints NaN holes,
    resizes to out_size x out_size. Same inpainting approach as upstream's
    process_depth_image, just operating on an already-cropped patch (the
    localise-then-zoom step above replaces upstream's own cropping logic).
    """
    patch = cv2.copyMakeBorder(depth_m_patch, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
    nan_mask = np.isnan(patch).astype(np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    nan_mask = cv2.dilate(nan_mask, kernel, iterations=1)
    patch = np.where(nan_mask == 1, 0, patch).astype(np.float32)

    scale = np.abs(patch).max()
    if scale < 1e-6:
        scale = 1.0
    patch = patch / scale
    patch = cv2.inpaint(patch, nan_mask, 1, cv2.INPAINT_NS)
    patch = patch[1:-1, 1:-1] * scale

    patch = cv2.resize(patch, (out_size, out_size), interpolation=cv2.INTER_AREA)
    nan_mask_small = cv2.resize(nan_mask[1:-1, 1:-1], (out_size, out_size),
                                 interpolation=cv2.INTER_NEAREST)
    return patch, nan_mask_small


class GGCNNDetector:
    """
    GGCNN grasp-point detector for a static camera, using the
    localise-then-zoom strategy described at the top of this file.

    Usage:
        det = GGCNNDetector()
        results = det.detect(color_frame, depth_frame_mm, table_mask=table_mask)
        # results: list of dicts, best-quality first, each with
        #   u, v            -- pixel location in the ORIGINAL full frame
        #   z_m             -- depth at (u, v), metres
        #   ang_rad         -- grasp angle, image-plane radians
        #   width_px        -- grasp width in the RESIZED crop's pixel space
        #                      (not yet converted to a real-world size)
        #   quality         -- GGCNN's own position-quality score, 0-1
        #   candidate_area  -- pixel area of the depth blob that seeded this crop
    """

    def __init__(self, model_path=MODEL_PATH, device=None):
        self.device = device or torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.load(model_path, map_location=self.device, weights_only=False)
        self.model.eval()
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]

    def _infer_patch(self, depth_m_patch):
        """Run the model on one already-cropped-and-resized depth patch."""
        patch, nan_mask = _process_depth_patch(depth_m_patch)
        depth_in = np.clip(patch - np.nanmean(patch), -1, 1).astype(np.float32)
        depthT = torch.from_numpy(depth_in.reshape(1, 1, *depth_in.shape)).to(self.device)

        with torch.no_grad():
            pos_out, cos_out, sin_out, width_out = self.model(depthT)

        points = pos_out.cpu().numpy().squeeze()
        points[nan_mask.astype(bool)] = 0
        cos_map = cos_out.cpu().numpy().squeeze()
        sin_map = sin_out.cpu().numpy().squeeze()
        ang_map = np.arctan2(sin_map, cos_map) / 2.0
        width_map = width_out.cpu().numpy().squeeze() * 150.0

        points = ndimage.gaussian_filter(points, 5.0)
        ang_map = ndimage.gaussian_filter(ang_map, 2.0)
        width_map = ndimage.gaussian_filter(width_map, 2.0)
        points = np.clip(points, 0.0, 1.0 - 1e-3)

        return points, ang_map, width_map

    def detect_at(self, depth_frame_mm, u_center, v_center,
                   zoom_px=ZOOM_WINDOW_PX):
        """
        Crop a zoom_px x zoom_px window around (u_center, v_center), run
        GGCNN on it, and map the best grasp pixel back to the ORIGINAL
        frame's coordinates. Returns a result dict, or None if no valid
        depth was available in the window.
        """
        h, w = depth_frame_mm.shape
        half = zoom_px // 2
        u0, u1 = max(0, u_center - half), min(w, u_center + half)
        v0, v1 = max(0, v_center - half), min(h, v_center + half)
        if u1 - u0 < 4 or v1 - v0 < 4:
            return None

        depth_m = depth_frame_mm[v0:v1, u0:u1].astype(np.float32) / 1000.0
        depth_m[depth_frame_mm[v0:v1, u0:u1] == 0] = np.nan
        if np.all(np.isnan(depth_m)):
            return None

        crop_h, crop_w = depth_m.shape
        points, ang_map, width_map = self._infer_patch(depth_m)

        max_idx = np.unravel_index(np.argmax(points), points.shape)
        quality = float(points[max_idx])
        ang = float(ang_map[max_idx])
        width_px = float(width_map[max_idx])

        # Map from the MODEL_INPUT_SIZE-resized patch back to this crop's
        # own pixel space, then back to the ORIGINAL full-frame pixel space.
        frac_v = max_idx[0] / MODEL_INPUT_SIZE
        frac_u = max_idx[1] / MODEL_INPUT_SIZE
        u_full = int(round(u0 + frac_u * crop_w))
        v_full = int(round(v0 + frac_v * crop_h))

        z_mm = depth_frame_mm[v_full, u_full]
        if z_mm == 0:
            return None

        return {
            'u': u_full, 'v': v_full,
            'z_m': float(z_mm) / 1000.0,
            'ang_rad': ang,
            'width_px': width_px,
            'quality': quality,
        }

    def detect(self, color_frame, depth_frame_mm, table_mask=None,
               min_quality=0.2, zoom_px=ZOOM_WINDOW_PX):
        """
        Full pipeline: find_candidate_regions() to seed crop centres, then
        detect_at() each one. color_frame is accepted for interface
        symmetry with detect_colored_cubes() but not currently used (GGCNN
        operates on depth alone).

        Returns a list of result dicts (see class docstring), best quality
        first, filtered to quality >= min_quality.
        """
        candidates = find_candidate_regions(depth_frame_mm, table_mask=table_mask)

        results = []
        for cand in candidates:
            r = self.detect_at(depth_frame_mm, cand['u'], cand['v'], zoom_px=zoom_px)
            if r is None or r['quality'] < min_quality:
                continue
            r['candidate_area'] = cand['area']
            results.append(r)

        results.sort(key=lambda r: r['quality'], reverse=True)
        return results


# ── Robot-frame pose ─────────────────────────────────────────────────────────
def grasp_pose_in_robot_frame(result, Table_corner_3D, rvec_marker_wrt_camera,
                               camera_matrix_=camera_matrix):
    """
    Same transform chain as cube_pose_in_robot_frame() / marker40_pose_in_robot_frame():
    camera -> table -> robot. Position from the GGCNN pixel+depth via the
    same pinhole back-projection used everywhere else in this codebase.
    Orientation is the model's grasp angle carried through R_robot_cam as an
    in-plane (Z-axis) rotation -- NOT wrapped mod-90 (see module docstring:
    GGCNN's angle is already 180-degree periodic, unlike the cube's 90-degree
    shape symmetry).

    Returns (pos_robot [3] cm, yaw_deg).
    """
    fx = camera_matrix_[0, 0]; fy = camera_matrix_[1, 1]
    cx = camera_matrix_[0, 2]; cy = camera_matrix_[1, 2]
    u, v = result['u'], result['v']
    Z = result['z_m'] * 100.0   # m -> cm

    point_cam = np.array([
        (u - cx) * Z / fx,
        (v - cy) * Z / fy,
        Z,
    ], dtype=np.float64)

    pos_robot = camera_to_robot(point_cam, Table_corner_3D, rvec_marker_wrt_camera)

    R_cam_table, _ = cv2.Rodrigues(rvec_marker_wrt_camera)
    R_robot_cam = R_robot_table @ R_cam_table.T
    R_grasp_cam, _ = cv2.Rodrigues(np.array([0.0, 0.0, result['ang_rad']]))
    R_grasp_robot = R_robot_cam @ R_grasp_cam

    euler = Rot.from_matrix(R_grasp_robot).as_euler('xyz', degrees=True)
    yaw_deg = euler[2]

    return pos_robot, yaw_deg
