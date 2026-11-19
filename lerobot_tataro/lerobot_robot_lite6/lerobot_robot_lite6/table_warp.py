"""Overhead image -> top-down view of just the table, registered on the table's ArUco markers.

Same idea as xarm_ros2/src/computer_vision/TATARO_CV_TableCropping_with_ARUCO_7x7.py (markers 51/52/53 at three
table corners, 4th corner derived), but 2-D only: the outer corner of each marker in the 640x480 overhead image
is a table corner, and a perspective warp maps the table to a square image. No depth / intrinsics needed.

Detect ONCE per session (or per recorded episode) and reuse the warp for every frame: the arm or a hand can cover
a marker mid-episode, and the camera doesn't move during a session. Because the warp is registered on the table,
a bumped overhead camera (it moved ~10 px between 2026-11-06 and 2026-11-10) no longer shifts the images.

In the 640x480 overhead view: 52 = top-left, 51 = top-right, 53 = bottom-right (bottom-left is behind the arm).
"""

from __future__ import annotations

import cv2
import numpy as np

DICT = cv2.aruco.DICT_7X7_250
IDS = {"tl": 52, "tr": 51, "br": 53}
OUT_PX = 336  # output is OUT_PX x OUT_PX for the whole table (85 cm -> ~2.5 mm / px)
INSET_PX = 6  # trim this much of the table edge (output px) so no floor / desk strip remains


class MarkersNotFound(RuntimeError):
    pass


class TableWarp:
    def __init__(self, H: np.ndarray, size: int = OUT_PX):
        self.H = np.asarray(H, dtype=np.float64)
        self.size = size

    @classmethod
    def from_image(cls, rgb: np.ndarray, size: int = OUT_PX, inset: int = INSET_PX) -> "TableWarp":
        det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), cv2.aruco.DetectorParameters())
        corners, ids, _ = det.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        found = {} if ids is None else {int(i): corners[k][0] for k, i in enumerate(ids.ravel())}
        missing = [i for i in IDS.values() if i not in found]
        if missing:
            raise MarkersNotFound(f"table markers {missing} not visible in the overhead image (found {sorted(found)})")
        cen = np.mean([found[i].mean(0) for i in IDS.values()], axis=0)
        outer = {k: found[i][np.argmax(np.linalg.norm(found[i] - cen, axis=1))] for k, i in IDS.items()}
        tl, tr, br = outer["tl"], outer["tr"], outer["br"]
        bl = tl + br - tr
        s = float(size)
        dst = np.float32([[-inset, -inset], [s + inset, -inset], [s + inset, s + inset], [-inset, s + inset]])
        return cls(cv2.getPerspectiveTransform(np.float32([tl, tr, br, bl]), dst), size)

    def __call__(self, rgb: np.ndarray) -> np.ndarray:
        return cv2.warpPerspective(rgb, self.H, (self.size, self.size), flags=cv2.INTER_AREA)

    def points(self, px: np.ndarray) -> np.ndarray:
        """Raw overhead pixel(s) (N, 2) -> warped table pixel(s)."""
        return cv2.perspectiveTransform(np.asarray(px, np.float32).reshape(-1, 1, 2), self.H).reshape(-1, 2)
