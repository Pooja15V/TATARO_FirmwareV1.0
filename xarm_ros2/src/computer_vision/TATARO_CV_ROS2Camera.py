
import cv2
import numpy as np
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


COLOR_TOPIC      = "/camera/camera/color/image_raw"
COLOR_INFO_TOPIC = "/camera/camera/color/camera_info"
DEPTH_TOPIC      = "/camera/camera/aligned_depth_to_color/image_raw"   # requires align_depth.enable:=true
DEPTH_INFO_TOPIC = "/camera/camera/depth/camera_info"

FRAME_TIMEOUT_SEC = 20.0


class _CameraNode(Node):
    """
    Subscribes to the four RealSense topics published by:
        ros2 launch realsense2_camera rs_launch.py tf_publish_rate:=0.0 enable_sync:=true
    Caches the latest frames and camera intrinsics. Thread-safe.
    """

    def __init__(self):
        super().__init__("tataro_cv_camera_node")
        self._bridge    = CvBridge()
        self._lock      = threading.Lock()
        self._color     = None   # BGR uint8
        self._depth     = None   # uint16 mm
        self._color_K   = None   # (3,3) float64
        self._color_D   = None   # (n,) float64
        self._depth_K   = None   # (3,3) float64
        self._depth_D   = None   # (n,) float64
        self._color_evt = threading.Event()

        self.create_subscription(Image,      COLOR_TOPIC,      self._cb_color,      10)
        self.create_subscription(CameraInfo, COLOR_INFO_TOPIC, self._cb_color_info, 10)
        self.create_subscription(Image,      DEPTH_TOPIC,      self._cb_depth,      10)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self._cb_depth_info, 10)
        self.get_logger().info(
            f"[CAM] Subscribed to {COLOR_TOPIC}, {DEPTH_TOPIC}, "
            f"{COLOR_INFO_TOPIC}, {DEPTH_INFO_TOPIC}"
        )

    def _cb_color(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._color = bgr.copy()
            self._color_evt.set()
        except Exception as exc:
            self.get_logger().error(f"[CAM] Color frame error: {exc}")

    def _cb_depth(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self._lock:
                self._depth = depth.copy()
        except Exception as exc:
            self.get_logger().error(f"[CAM] Depth frame error: {exc}")

    def _cb_color_info(self, msg: CameraInfo) -> None:
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        with self._lock:
            self._color_K = K
            self._color_D = D

    def _cb_depth_info(self, msg: CameraInfo) -> None:
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        with self._lock:
            self._depth_K = K
            self._depth_D = D

    def get_latest_frames(self) -> tuple:
        """Return (color_bgr, depth_mm) copies; either may be None."""
        with self._lock:
            c = self._color.copy() if self._color is not None else None
            d = self._depth.copy() if self._depth is not None else None
        return c, d

    def get_color_intrinsics(self) -> tuple:
        """Return (K 3x3, D n-vec) for the color camera, or (None, None)."""
        with self._lock:
            K = self._color_K.copy() if self._color_K is not None else None
            D = self._color_D.copy() if self._color_D is not None else None
        return K, D

    def get_depth_intrinsics(self) -> tuple:
        """Return (K 3x3, D n-vec) for the depth camera, or (None, None)."""
        with self._lock:
            K = self._depth_K.copy() if self._depth_K is not None else None
            D = self._depth_D.copy() if self._depth_D is not None else None
        return K, D


class ROS2CameraInterface:
    """
    Subscriber-only interface to a running RealSense ROS2 driver.

    Does NOT launch the driver — connect() expects the driver to already be running:
        ros2 launch realsense2_camera rs_launch.py tf_publish_rate:=0.0 enable_sync:=true

    Depth is from /camera/camera/depth/image_rect_raw (non-aligned, uint16 mm).
    To get 3-D position from a color-frame pixel, sample depth at that pixel
    and back-project using the depth camera intrinsics from get_depth_intrinsics().
    The D435i baseline (~5 cm) introduces a small pixel offset between the two
    frames; for working distances >40 cm this error is typically <5 mm.

    Usage:
        pipe = connect_camera()
        color, depth = get_frame(pipe, return_depth=True)
        K_color, D_color = get_color_intrinsics(pipe)
        K_depth, D_depth = get_depth_intrinsics(pipe)
        stop_camera(pipe)
    """

    def __init__(self):
        self._node:        _CameraNode | None           = None
        self._executor:    SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None       = None

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        self._node     = _CameraNode()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="ros2cam_executor"
        )
        self._spin_thread.start()
        print(f"[CAM] Waiting for first color frame (timeout {FRAME_TIMEOUT_SEC:.0f}s)...")
        if not self._node._color_evt.wait(timeout=FRAME_TIMEOUT_SEC):
            self.stop()
            raise RuntimeError(
                f"[CAM] No frame received on '{COLOR_TOPIC}' within {FRAME_TIMEOUT_SEC:.0f}s.\n"
                "Start the RealSense driver first:\n"
                "  ros2 launch realsense2_camera rs_launch.py "
                "tf_publish_rate:=0.0 enable_sync:=true"
            )
        print("[CAM] Connected — frames streaming.")

    def stop(self) -> None:
        print("[CAM] Disconnecting from ROS2 camera topics...")
        try:
            if self._executor:
                self._executor.shutdown()
                self._executor = None
            if self._node:
                self._node.destroy_node()
                self._node = None
            if rclpy.ok():
                rclpy.shutdown()
            if self._spin_thread and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=3.0)
                self._spin_thread = None
        except Exception as exc:
            print(f"[CAM] Warning during shutdown: {exc}")
        print("[CAM] Disconnected.")

    def get_frame(self, return_depth: bool = False) -> tuple:
        if self._node is None:
            return None, None
        color, depth = self._node.get_latest_frames()
        if color is None:
            return None, None
        return (color, depth) if return_depth else (color, None)

    def get_color_intrinsics(self) -> tuple:
        if self._node is None:
            return None, None
        return self._node.get_color_intrinsics()

    def get_depth_intrinsics(self) -> tuple:
        if self._node is None:
            return None, None
        return self._node.get_depth_intrinsics()


# ── Public API ─────────────────────────────────────────────────────────────────

def connect_camera() -> ROS2CameraInterface:
    """
    Connect to an already-running RealSense ROS2 driver and return the camera interface.
    Blocks until the first color frame arrives (or raises RuntimeError on timeout).
    """
    pipe = ROS2CameraInterface()
    pipe.connect()
    return pipe


def get_frame(pipe: ROS2CameraInterface, return_depth: bool = False) -> tuple:
    """
    Return (color_bgr, depth_uint16_mm) from the latest ROS2 frame.

    depth is from /camera/camera/depth/image_rect_raw (non-aligned, uint16 mm).
    Returns (None, None) if no frame is available yet.
    """
    return pipe.get_frame(return_depth=return_depth)


def get_color_intrinsics(pipe: ROS2CameraInterface) -> tuple:
    """Return (K 3x3, D n-vec) from /camera/camera/color/camera_info, or (None, None)."""
    return pipe.get_color_intrinsics()


def get_depth_intrinsics(pipe: ROS2CameraInterface) -> tuple:
    """Return (K 3x3, D n-vec) from /camera/camera/depth/camera_info, or (None, None)."""
    return pipe.get_depth_intrinsics()


def stop_camera(pipe: ROS2CameraInterface) -> None:
    """Disconnect from ROS2 camera topics, destroy the subscriber node, close CV windows."""
    pipe.stop()
    cv2.destroyAllWindows()
