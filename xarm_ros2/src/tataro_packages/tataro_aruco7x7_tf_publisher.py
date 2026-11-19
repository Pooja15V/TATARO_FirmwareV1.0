#!/usr/bin/env python3
"""
tataro_aruco7x7_tf_publisher — Single ArUco DICT_7X7_250 marker TF publisher.

Detects one ArUco 7×7 marker by ID and continuously broadcasts its pose
as a TF frame.  Designed for the calibration touch workflow as a drop-in
replacement for the charuco detector when the board cannot be detected flat.

Use a marker ID that is NOT 51, 52, or 53 (those are the table-corner markers).
A printed 7×7 marker of any other ID works — e.g., ID 54.

Parameters:
    marker_id       int    54        target ArUco DICT_7X7_250 marker ID
    marker_size_cm  float  15.0      black-pattern side in cm (NOT the printed square)
    camera_frame    str    'camera_color_optical_frame'
    marker_frame    str    'calibration_marker_frame'
    depth_radius    int    10        depth sampling radius in pixels

Subscribes:
    /camera/camera/color/image_raw
    /camera/camera/aligned_depth_to_color/image_raw

Publishes:
    TF:  camera_frame → marker_frame  (updated every detected frame)
"""

import sys
import threading
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

# Reuse the calibrated camera matrix from the CV pipeline
_CV_DIR = str(Path(__file__).parent.parent / "computer_vision")
if _CV_DIR not in sys.path:
    sys.path.insert(0, _CV_DIR)

# 1280×720 calibrated intrinsics (do not change)
_CAMERA_MATRIX = np.array([
    [643.9336547851562, 0,               650.7435302734375],
    [0,                642.9497680664062, 363.3885192871094],
    [0,                0,                 1               ],
], dtype=np.float64)

_DIST_COEFFS = np.array([
    -0.05567498505115509,
     0.06433625519275665,
    -0.00040179200004786253,
     0.00031893057166598737,
    -0.02026429958641529,
], dtype=np.float64)


def _sample_depth(depth_frame, u, v, radius=10):
    h, w = depth_frame.shape
    samples = []
    for dv in range(-radius, radius + 1):
        for du in range(-radius, radius + 1):
            nu, nv = u + du, v + dv
            if 0 <= nv < h and 0 <= nu < w:
                z = depth_frame[nv, nu]
                if z > 0:
                    samples.append(float(z))
    return float(np.median(samples)) if samples else 0.0


class Aruco7x7TFPublisher(Node):

    def __init__(self):
        super().__init__('tataro_aruco7x7_tf_publisher')

        self.declare_parameter('marker_id',      54)
        self.declare_parameter('marker_size_cm', 15.0)
        self.declare_parameter('camera_frame',   'camera_color_optical_frame')
        self.declare_parameter('marker_frame',   'calibration_marker_frame')
        self.declare_parameter('depth_radius',   10)

        self.marker_id    = self.get_parameter('marker_id').value
        self.marker_size  = self.get_parameter('marker_size_cm').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.depth_radius = self.get_parameter('depth_radius').value

        self._dict       = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
        self._detector   = cv2.aruco.ArucoDetector(
            self._dict, cv2.aruco.DetectorParameters()
        )

        self._bridge  = CvBridge()
        self._lock    = threading.Lock()
        self._color   = None
        self._depth   = None
        self._tf_pub  = TransformBroadcaster(self)

        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._cb_color, 10
        )
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self._cb_depth, 10
        )
        self.create_timer(1.0 / 15.0, self._detect_and_publish)   # 15 Hz

        self.get_logger().info(
            f'Aruco7x7TFPublisher started — '
            f'marker_id={self.marker_id}  '
            f'size={self.marker_size}cm  '
            f'publishing TF: {self.camera_frame} → {self.marker_frame}'
        )

    def _cb_color(self, msg):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._color = bgr.copy()
        except Exception as exc:
            self.get_logger().error(f'Color CvBridge error: {exc}')

    def _cb_depth(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self._lock:
                self._depth = depth.copy()
        except Exception as exc:
            self.get_logger().error(f'Depth CvBridge error: {exc}')

    def _detect_and_publish(self):
        with self._lock:
            color = self._color.copy() if self._color is not None else None
            depth = self._depth.copy() if self._depth is not None else None

        if color is None or depth is None:
            return

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None:
            return

        for i, found_id in enumerate(ids.flatten()):
            if found_id != self.marker_id:
                continue

            marker_corners = corners[i]
            center = np.mean(marker_corners.reshape(4, 2), axis=0)
            u, v   = int(center[0]), int(center[1])

            h, w = depth.shape
            if not (0 <= v < h and 0 <= u < w):
                return

            z_depth = _sample_depth(depth, u, v, self.depth_radius)
            if z_depth <= 0:
                self.get_logger().warn(
                    f'No valid depth near ({u},{v}) for marker {self.marker_id}',
                    throttle_duration_sec=2.0,
                )
                return

            # Back-project center to 3-D (consistent with detect_marker_7x7)
            Z  = z_depth / 10.0   # mm → cm
            fx = _CAMERA_MATRIX[0, 0]
            fy = _CAMERA_MATRIX[1, 1]
            cx = _CAMERA_MATRIX[0, 2]
            cy = _CAMERA_MATRIX[1, 2]
            X  = (u - cx) * Z / fx
            Y  = (v - cy) * Z / fy
            tvec = np.array([[X], [Y], [Z]], dtype=np.float64)

            # Rotation from solvePnP
            half = self.marker_size / 2.0
            obj_pts = np.array([
                [-half,  half, 0],
                [ half,  half, 0],
                [ half, -half, 0],
                [-half, -half, 0],
            ], dtype=np.float32)
            img_pts = marker_corners.reshape(4, 2).astype(np.float32)
            ok, rvec, _ = cv2.solvePnP(
                obj_pts, img_pts, _CAMERA_MATRIX, _DIST_COEFFS
            )
            if not ok:
                return

            # Convert rvec → quaternion
            rot_mat, _ = cv2.Rodrigues(rvec)
            q = R.from_matrix(rot_mat).as_quat()   # x, y, z, w

            # Broadcast TF (positions in metres)
            tf = TransformStamped()
            tf.header.stamp    = self.get_clock().now().to_msg()
            tf.header.frame_id = self.camera_frame
            tf.child_frame_id  = self.marker_frame
            tf.transform.translation.x = float(tvec[0, 0]) / 100.0   # cm → m
            tf.transform.translation.y = float(tvec[1, 0]) / 100.0
            tf.transform.translation.z = float(tvec[2, 0]) / 100.0
            tf.transform.rotation.x = float(q[0])
            tf.transform.rotation.y = float(q[1])
            tf.transform.rotation.z = float(q[2])
            tf.transform.rotation.w = float(q[3])
            self._tf_pub.sendTransform(tf)
            return   # only process the first match


def main(args=None):
    rclpy.init(args=args)
    node = Aruco7x7TFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
