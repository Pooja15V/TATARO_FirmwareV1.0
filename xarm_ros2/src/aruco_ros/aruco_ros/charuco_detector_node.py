#!/usr/bin/env python3
"""
ChArUco board detector for hand-eye calibration.

Detects a ChArUco board and broadcasts T_camera_board as a TF.
Multiple corners per frame eliminates the single-marker planar ambiguity.

Parameters (set via launch file or ros2 run --param):
  squares_x      (int,   default 5)      board columns
  squares_y      (int,   default 5)      board rows
  square_length  (float, default 0.030)  checker square side, metres
  marker_length  (float, default 0.024)  ArUco marker side, metres
  min_corners    (int,   default 6)      minimum detected corners to accept pose
  camera_frame   (str,   default 'camera_color_optical_frame')
  marker_frame   (str,   default 'charuco_board_frame')

Must match the parameters used when the board was printed.

Publishes (in camera_frame):
  /charuco/pose  (geometry_msgs/PoseStamped)    position + quaternion
  /charuco/rvec  (geometry_msgs/Vector3Stamped) raw Rodrigues rotation
                 vector (radians), same rotation as the pose quaternion,
                 same header.stamp - for consumers that want the rvec
                 form without round-tripping through the quaternion.

Usage:
  ros2 run aruco_ros charuco_detector_node.py
  ros2 launch aruco_ros charuco_detector.launch.py
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R


class CharucoDetectorNode(Node):
    def __init__(self):
        super().__init__('charuco_detector')

        self.declare_parameter('squares_x',     5)
        self.declare_parameter('squares_y',     5)
        self.declare_parameter('square_length', 0.030)
        self.declare_parameter('marker_length', 0.024)
        self.declare_parameter('min_corners',   6)
        self.declare_parameter('camera_frame',  'camera_color_optical_frame')
        self.declare_parameter('marker_frame',  'charuco_board_frame')

        squares_x     = self.get_parameter('squares_x').value
        squares_y     = self.get_parameter('squares_y').value
        square_length = self.get_parameter('square_length').value
        marker_length = self.get_parameter('marker_length').value
        self.min_corners  = self.get_parameter('min_corners').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value

        # ── Build board and detector ──────────────────────────────────────────
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length,
            marker_length,
            dictionary
        )

        aruco_params = cv2.aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        charuco_params = cv2.aruco.CharucoParameters()
        charuco_params.tryRefineMarkers = True

        self.detector = cv2.aruco.CharucoDetector(
            self.board, charuco_params, aruco_params
        )

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.bridge        = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_matrix = None
        self.dist_coeffs   = None
        self.camera_info_received = False

        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_callback, 10
        )
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.image_callback, 10
        )
        self.pose_pub   = self.create_publisher(PoseStamped,   '/charuco/pose', 10)
        self.rvec_pub   = self.create_publisher(Vector3Stamped, '/charuco/rvec', 10)
        self.result_pub = self.create_publisher(Image,         '/charuco/result', 10)

        self.get_logger().info(
            f'ChArUco detector started  '
            f'board={squares_x}x{squares_y}  '
            f'square={square_length*100:.1f}cm  '
            f'marker={marker_length*100:.1f}cm  '
            f'min_corners={self.min_corners}'
        )

    # ── Camera info ───────────────────────────────────────────────────────────

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d, dtype=np.float64)
            self.camera_info_received = True
            self.get_logger().info('Camera intrinsics loaded from camera_info')

    # ── Detection ─────────────────────────────────────────────────────────────

    def image_callback(self, msg):
        if not self.camera_info_received:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            charuco_corners, charuco_ids, _, _ = self.detector.detectBoard(gray)

            if charuco_ids is None or len(charuco_ids) < self.min_corners:
                if charuco_ids is not None:
                    self.get_logger().warn(
                        f'Only {len(charuco_ids)} corners detected (need {self.min_corners}) — skipped',
                        throttle_duration_sec=2.0
                    )
                result_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                result_msg.header = msg.header
                self.result_pub.publish(result_msg)
                return

            # Estimate pose — uses all detected corners, no planar ambiguity
            # NOTE: cv2.aruco.estimatePoseCharucoBoard was removed in the new
            # ArUco API (OpenCV >= 4.7, confirmed on 4.11 here). The
            # replacement is board.matchImagePoints() + cv2.solvePnP().
            obj_points, img_points = self.board.matchImagePoints(charuco_corners, charuco_ids)

            if obj_points is None or len(obj_points) < 4:
                return

            success, rvec, tvec = cv2.solvePnP(
                obj_points, img_points,
                self.camera_matrix, self.dist_coeffs
            )

            if not success:
                return

            self.broadcast_transform(rvec, tvec, msg.header.stamp)

            t = tvec.flatten()
            self.get_logger().info(
                f'Tracking: {len(charuco_ids)} corners  '
                f'pos=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m in {self.camera_frame}',
                throttle_duration_sec=1.0
            )

            pose_msg = self.make_pose_msg(rvec, tvec, msg.header)
            self.pose_pub.publish(pose_msg)

            rvec_msg = self.make_rvec_msg(rvec, msg.header)
            self.rvec_pub.publish(rvec_msg)

            frame = self.draw_detection(frame, charuco_corners, charuco_ids, rvec, tvec)

        except Exception as exc:
            self.get_logger().error(f'image_callback error: {exc}')

        result_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        result_msg.header = msg.header
        self.result_pub.publish(result_msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _rvec_to_quat(self, rvec):
        rot, _ = cv2.Rodrigues(rvec)
        return R.from_matrix(rot).as_quat()  # x, y, z, w

    def make_pose_msg(self, rvec, tvec, header):
        pose = PoseStamped()
        pose.header = header
        q = self._rvec_to_quat(rvec)
        t = tvec.flatten()
        pose.pose.position.x = float(t[0])
        pose.pose.position.y = float(t[1])
        pose.pose.position.z = float(t[2])
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])
        return pose

    def make_rvec_msg(self, rvec, header):
        """Raw Rodrigues rotation vector (radians) straight from
        estimatePoseCharucoBoard, alongside the quaternion in pose_msg -
        avoids downstream consumers having to round-trip quaternion->rotvec
        themselves."""
        v = Vector3Stamped()
        v.header = header
        r = rvec.flatten()
        v.vector.x = float(r[0])
        v.vector.y = float(r[1])
        v.vector.z = float(r[2])
        return v

    def broadcast_transform(self, rvec, tvec, stamp):
        q = self._rvec_to_quat(rvec)
        t = tvec.flatten()
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = self.camera_frame
        tf.child_frame_id  = self.marker_frame
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(tf)

    def draw_detection(self, frame, charuco_corners, charuco_ids, rvec, tvec):
        cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)
        cv2.drawFrameAxes(
            frame, self.camera_matrix, self.dist_coeffs,
            rvec, tvec, 0.05
        )
        n = len(charuco_ids)
        cv2.putText(
            frame, f'ChArUco  corners={n}',
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
        )
        return frame


def main(args=None):
    rclpy.init(args=args)
    node = CharucoDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
