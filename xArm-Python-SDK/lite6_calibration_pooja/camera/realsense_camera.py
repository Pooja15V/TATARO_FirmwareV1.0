"""
realsense_camera.py
-------------------
Intel RealSense D435i interface for the eye-in-hand calibration system.

Responsibilities:
  - Initialize the D435i (RGB + depth streams).
  - Align depth to the color frame so a color pixel maps to a valid depth.
  - Provide live color frames for display.
  - Read depth at a pixel.
  - Expose color intrinsics (fx, fy, cx, cy).
  - Convert a clicked pixel (u, v) + depth Z into camera coordinates (Xc, Yc, Zc).

Coordinate convention (camera frame, in millimeters):
  Xc = (u - cx) * Z / fx
  Yc = (v - cy) * Z / fy
  Zc = Z
"""

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "pyrealsense2 is required. Install with: pip install pyrealsense2"
    ) from exc


class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        # Align depth -> color so each color pixel has matching depth.
        self.align = rs.align(rs.stream.color)

        self.profile = None
        self.depth_scale = None        # meters per depth unit
        self.intrinsics = None         # color stream intrinsics

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def start(self):
        self.profile = self.pipeline.start(self.config)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()  # e.g. 0.001 m/unit

        color_stream = self.profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        print("[Camera] Started.")
        print(f"[Camera] depth_scale = {self.depth_scale} m/unit")
        print(f"[Camera] intrinsics  = fx={self.intrinsics.fx:.2f} "
              f"fy={self.intrinsics.fy:.2f} cx={self.intrinsics.ppx:.2f} "
              f"cy={self.intrinsics.ppy:.2f}")

    def stop(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        print("[Camera] Stopped.")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ------------------------------------------------------------------ #
    # Frame acquisition
    # ------------------------------------------------------------------ #
    def get_frames(self):
        """Return (color_image, depth_frame) with depth aligned to color.

        color_image : np.ndarray (H, W, 3) BGR uint8
        depth_frame : rs.depth_frame  (use get_depth_mm for values)
        """
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_frame

    # ------------------------------------------------------------------ #
    # Depth helpers
    # ------------------------------------------------------------------ #
    def get_depth_mm(self, depth_frame, u, v, kernel=2):
        """Median depth in mm around pixel (u, v).

        A small median window rejects zero/invalid pixels and noise.
        Returns 0.0 if no valid depth is found.
        """
        u, v = int(round(u)), int(round(v))
        vals = []
        for dv in range(-kernel, kernel + 1):
            for du in range(-kernel, kernel + 1):
                uu, vv = u + du, v + dv
                if 0 <= uu < self.width and 0 <= vv < self.height:
                    d = depth_frame.get_distance(uu, vv)  # meters
                    if d > 0:
                        vals.append(d)
        if not vals:
            return 0.0
        return float(np.median(vals)) * 1000.0  # -> millimeters

    # ------------------------------------------------------------------ #
    # Pixel -> camera coordinate conversion
    # ------------------------------------------------------------------ #
    def pixel_to_camera(self, u, v, z_mm):
        """Convert pixel (u, v) + depth z_mm into camera coords (Xc, Yc, Zc) in mm."""
        fx = self.intrinsics.fx
        fy = self.intrinsics.fy
        cx = self.intrinsics.ppx
        cy = self.intrinsics.ppy

        Xc = (u - cx) * z_mm / fx
        Yc = (v - cy) * z_mm / fy
        Zc = z_mm
        return Xc, Yc, Zc

    def get_camera_coords_at_pixel(self, depth_frame, u, v):
        """Convenience: depth lookup + conversion. Returns (Xc, Yc, Zc) mm or None."""
        z = self.get_depth_mm(depth_frame, u, v)
        if z <= 0:
            return None
        return self.pixel_to_camera(u, v, z)
