"""
TATARO_CV_HSVPicker.py — Interactive HSV range calibration tool for
                          TATARO_CV_ObjectDetectionPipeline.py's COLOR_RANGES.

Move the on-screen crosshair (WASD, Shift+WASD for bigger steps) over a
coloured cube and press SPACE to sample it — no need to relocate cubes to a
fixed spot. Accumulate several samples (different lighting/angle/cube) per
colour, then print a COLOR_RANGES-ready dict to paste straight into
TATARO_CV_ObjectDetectionPipeline.py.

Sampling is crosshair+keypress rather than mouse-click: this OpenCV/Qt build
cannot allocate a callback-bindable window handle here (cv2.setMouseCallback
raises "NULL window handler" even with cv2.startWindowThread() and multiple
imshow()/waitKey() cycles first) — a documented Qt5-highgui limitation, not
fixable from application code. Every other window in this codebase
(track_cube.py, TATARO_CV_PipelineV1.0.py, ...) already avoids
setMouseCallback for the same reason.

Controls
--------
  1 / 2 / 3 / 4   select active colour: red / green / blue / yellow
  w / a / s / d   move the crosshair up / left / down / right (small step)
  W / A / S / D   same, but a bigger step (Shift+key)
  r               recentre the crosshair
  + / -           grow / shrink the sample box — match it to the object's
                  on-screen size so it doesn't scoop up background pixels
  SPACE           sample the patch under the crosshair into the active
                  colour's pool
  m               toggle live mask preview for the active colour's current
                  accumulated range
  c               clear all samples for the active colour
  p               print the accumulated COLOR_RANGES dict to the console
  q               quit

Notes
-----
Red wraps hue 0/179 in OpenCV's HSV. This tool detects that automatically:
if the active colour's samples straddle both ends of the hue axis, it splits
them into two (lo, hi) bands, matching COLOR_RANGES['red']'s existing
two-range format.
"""

import cv2
import numpy as np

from TATARO_CV_ROS2Camera import connect_camera, stop_camera, get_frame

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_WINDOW = "TATARO - HSV Picker"

ACTIVE_COLORS = ['red', 'green', 'blue', 'yellow']
_KEY_TO_COLOR = {ord('1'): 'red', ord('2'): 'green', ord('3'): 'blue', ord('4'): 'yellow'}
_COLOR_SWATCH = {  # BGR, just for on-screen labels
    'red':    (0, 0, 255),
    'green':  (0, 255, 0),
    'blue':   (255, 0, 0),
    'yellow': (0, 255, 255),
}

PATCH_RADIUS_DEFAULT = 4    # px, sample square half-size — start small, grow with +/- if needed.
                             # At "whole workspace" zoom a cube can be ~15px across on screen;
                             # a box bigger than the object mixes in background pixels and
                             # blows the accumulated range wide open (e.g. table gray + cube
                             # red spans nearly full saturation). Match the box to the object.
PATCH_RADIUS_MIN = 2
PATCH_RADIUS_MAX = 40

PCTL_LO, PCTL_HI = 3, 97    # percentile trim instead of raw min/max — tolerates a couple of
                             # percent of edge/anti-aliasing pixels without blowing out the
                             # range. It is NOT a substitute for sizing the box to the object:
                             # a box that's mostly background (e.g. >5-10% contamination) will
                             # still blow the range wide open no matter the trim — shrink the
                             # box with +/- until it visibly fits inside the object first.
H_MARGIN = 5
S_MARGIN = 15
V_MARGIN = 15
HUE_WRAP_SPLIT = 90    # samples on either side of this hue are treated as separate bands

ROI_STEP = 10          # px per WASD press
ROI_STEP_BIG = 40       # px per Shift+WASD press
_ROI_MOVE_KEYS = {      # key -> (dx, dy)
    ord('a'): (-ROI_STEP, 0),      ord('d'): (ROI_STEP, 0),
    ord('w'): (0, -ROI_STEP),      ord('s'): (0, ROI_STEP),
    ord('A'): (-ROI_STEP_BIG, 0),  ord('D'): (ROI_STEP_BIG, 0),
    ord('W'): (0, -ROI_STEP_BIG),  ord('S'): (0, ROI_STEP_BIG),
}


class _Picker:
    def __init__(self):
        self.active = 'red'
        self.samples = {c: [] for c in ACTIVE_COLORS}   # color -> list of (h,s,v) pixels
        self.show_mask = False
        self.patch_radius = PATCH_RADIUS_DEFAULT

    def grow_patch(self, delta):
        self.patch_radius = int(np.clip(self.patch_radius + delta,
                                         PATCH_RADIUS_MIN, PATCH_RADIUS_MAX))

    def sample_patch(self, hsv_frame, cx, cy):
        h, w = hsv_frame.shape[:2]
        r = self.patch_radius
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        patch = hsv_frame[y0:y1, x0:x1].reshape(-1, 3)
        self.samples[self.active].append(patch)
        n_patches = len(self.samples[self.active])
        n_px = sum(p.shape[0] for p in self.samples[self.active])
        print(f"[{self.active}] +{patch.shape[0]} px sampled "
              f"({n_patches} samples, {n_px} px total)")

    def clear_active(self):
        self.samples[self.active] = []
        print(f"[{self.active}] samples cleared")

    def ranges_for(self, color):
        """Return list of (lo, hi) HSV tuples for `color`, or None if no samples yet."""
        if not self.samples[color]:
            return None
        pts = np.concatenate(self.samples[color], axis=0)  # (N, 3)
        hues = pts[:, 0]

        low_band  = pts[hues <  HUE_WRAP_SPLIT]
        high_band = pts[hues >= HUE_WRAP_SPLIT]

        bands = []
        # Wrap-around case: samples on both sides AND both near the 0/179 edges.
        wraps = (low_band.size and high_band.size
                 and low_band[:, 0].min() < 15 and high_band[:, 0].max() > 165)

        groups = [low_band, high_band] if wraps else [pts]
        for g in groups:
            if g.size == 0:
                continue
            # Percentile trim (not raw min/max): tolerates a bit of edge/background
            # contamination in a patch without one stray pixel blowing the range open.
            h_lo = max(0,   int(np.percentile(g[:, 0], PCTL_LO)) - H_MARGIN)
            h_hi = min(179, int(np.percentile(g[:, 0], PCTL_HI)) + H_MARGIN)
            s_lo = max(0,   int(np.percentile(g[:, 1], PCTL_LO)) - S_MARGIN)
            s_hi = min(255, int(np.percentile(g[:, 1], PCTL_HI)) + S_MARGIN)
            v_lo = max(0,   int(np.percentile(g[:, 2], PCTL_LO)) - V_MARGIN)
            v_hi = min(255, int(np.percentile(g[:, 2], PCTL_HI)) + V_MARGIN)
            bands.append(((h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi)))
        return bands

    def print_all(self):
        print("\n" + "=" * 60)
        print("COLOR_RANGES = {")
        any_data = False
        for color in ACTIVE_COLORS:
            bands = self.ranges_for(color)
            if bands is None:
                print(f"    # '{color}': no samples yet")
                continue
            any_data = True
            band_str = ",\n               ".join(f"({lo}, {hi})" for lo, hi in bands)
            print(f"    '{color}': [{band_str}],")
        print("}")
        print("=" * 60)
        if not any_data:
            print("(no colours sampled yet — press SPACE over a cube first)")


def _mask_preview(hsv_frame, bands):
    mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
    for lo, hi in bands:
        mask |= cv2.inRange(hsv_frame, np.array(lo), np.array(hi))
    return mask


def run_picker():
    picker = _Picker()
    pipe = connect_camera()
    roi = None   # (cx, cy) — set to frame centre on the first frame

    print("[HSV Picker] 1/2/3/4=colour | wasd/WASD=move crosshair | r=recentre | "
          "+/-=resize box | SPACE=sample | m=mask | c=clear | p=print | q=quit")

    try:
        while True:
            color_frame, _ = get_frame(pipe, return_depth=False)
            if color_frame is None:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            h, w = color_frame.shape[:2]
            if roi is None:
                roi = [w // 2, h // 2]
            cx, cy = roi

            display = color_frame.copy()

            swatch = _COLOR_SWATCH[picker.active]
            cv2.putText(display, f"Active: {picker.active}  "
                                  f"({sum(p.shape[0] for p in picker.samples[picker.active])} px sampled)  "
                                  f"box={picker.patch_radius * 2 + 1}px",
                        (20, 40), _FONT, 0.7, swatch, 2)
            cv2.putText(display, "1/2/3/4=colour  wasd/WASD=move  r=recentre  +/-=resize  "
                                  "SPACE=sample  m=mask  c=clear  p=print  q=quit",
                        (20, 70), _FONT, 0.5, (255, 255, 255), 1)

            if picker.show_mask:
                hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
                bands = picker.ranges_for(picker.active)
                if bands:
                    mask = _mask_preview(hsv, bands)
                    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    display = cv2.addWeighted(display, 0.5, mask_bgr, 0.5, 0)

            # Crosshair + sample-patch box — resize with +/- to fit the object, then SPACE.
            r = picker.patch_radius
            cv2.rectangle(display, (cx - r, cy - r), (cx + r, cy + r), swatch, 2)
            cv2.drawMarker(display, (cx, cy), swatch, cv2.MARKER_CROSS, 20, 2)

            cv2.imshow(_WINDOW, display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key in _KEY_TO_COLOR:
                picker.active = _KEY_TO_COLOR[key]
                print(f"[HSV Picker] Active colour -> {picker.active}")
            elif key in _ROI_MOVE_KEYS:
                dx, dy = _ROI_MOVE_KEYS[key]
                r = picker.patch_radius
                roi[0] = int(np.clip(roi[0] + dx, r, w - 1 - r))
                roi[1] = int(np.clip(roi[1] + dy, r, h - 1 - r))
            elif key == ord('r'):
                roi[0], roi[1] = w // 2, h // 2
            elif key in (ord('+'), ord('=')):
                picker.grow_patch(+1)
            elif key in (ord('-'), ord('_')):
                picker.grow_patch(-1)
            elif key == ord(' '):
                hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
                picker.sample_patch(hsv, cx, cy)
            elif key == ord('m'):
                picker.show_mask = not picker.show_mask
            elif key == ord('c'):
                picker.clear_active()
            elif key == ord('p'):
                picker.print_all()

    finally:
        picker.print_all()
        stop_camera(pipe)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run_picker()
