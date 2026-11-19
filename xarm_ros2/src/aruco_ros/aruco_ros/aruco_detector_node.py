#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R


class ArUcoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_frame', 'aruco_marker_frame')

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Camera intrinsics (will be loaded from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # ArUco setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Subscribe to camera info (to get calibration)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Subscribe to camera feed
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.result_pub = self.create_publisher(Image, '/aruco/result', 10)

        self.get_logger().info(f'ArUco Detector started - Looking for marker ID: {self.marker_id}')

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            self.camera_info_received = True
            self.get_logger().info(f'✅ Loaded camera intrinsics from camera_info')

    def image_callback(self, msg):
        try:
            # Wait for camera calibration
            if not self.camera_info_received:
                return

            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect markers
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray_frame)

            if ids is not None:
                ids_flat = ids.flatten()

                # Look for our target marker
                if self.marker_id in ids_flat:
                    idx = np.where(ids_flat == self.marker_id)[0][0]
                    markerCorner = corners[idx]

                    # Estimate pose
                    obj_points = np.array([
                        [-self.marker_size / 2,  self.marker_size / 2, 0],
                        [ self.marker_size / 2,  self.marker_size / 2, 0],
                        [ self.marker_size / 2, -self.marker_size / 2, 0],
                        [-self.marker_size / 2, -self.marker_size / 2, 0]
                    ], dtype=np.float32)

                    img_points = markerCorner.reshape(4, 2).astype(np.float32)
                    n_solutions, rvecs, tvecs, reproj_errors = cv2.solvePnPGeneric(
                        obj_points, img_points, self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    # Solutions are sorted by reprojection error. If the two
                    # planar-ambiguity candidates score nearly the same, the
                    # orientation is unreliable (near face-on view) — skip frame.
                    ambiguous = (
                        n_solutions >= 2
                        and reproj_errors[0][0] > 0.75 * reproj_errors[1][0]
                    )

                    if ambiguous:
                        self.get_logger().warn(
                            'Ambiguous marker pose (view too face-on) — frame skipped',
                            throttle_duration_sec=2.0
                        )
                    elif n_solutions > 0:
                        rvec, tvec = rvecs[0], tvecs[0]
                        # Publish pose
                        pose_msg = self.rvec_tvec_to_pose(rvec, tvec, msg.header)
                        self.pose_pub.publish(pose_msg)

                        # Broadcast TF
                        self.broadcast_transform(rvec, tvec, msg.header.stamp)

                        # Draw on frame
                        frame = self.draw_detection(frame, markerCorner, rvec, tvec)

            # Publish result image
            result_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            result_msg.header = msg.header
            self.result_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')

    def rvec_tvec_to_pose(self, rvec, tvec, header):
        pose_msg = PoseStamped()
        pose_msg.header = header

        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()  # [x, y, z, w]

        pose_msg.pose.position.x = float(tvec[0][0])
        pose_msg.pose.position.y = float(tvec[1][0])
        pose_msg.pose.position.z = float(tvec[2][0])

        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        return pose_msg

    def broadcast_transform(self, rvec, tvec, timestamp):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()

        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = self.marker_frame

        transform.transform.translation.x = float(tvec[0][0])
        transform.transform.translation.y = float(tvec[1][0])
        transform.transform.translation.z = float(tvec[2][0])

        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(transform)

    def draw_detection(self, frame, markerCorner, rvec, tvec):
        marker_corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = marker_corners

        topLeft = (int(topLeft[0]), int(topLeft[1]))
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

        # Draw marker box
        cv2.line(frame, topLeft, topRight, (0, 255, 255), 2)
        cv2.line(frame, topRight, bottomRight, (0, 255, 255), 2)
        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 255), 2)
        cv2.line(frame, bottomLeft, topLeft, (0, 255, 255), 2)

        # Draw center
        img_pts, _ = cv2.projectPoints(
            np.array([[0, 0, 0]], dtype=np.float32),
            rvec, tvec,
            self.camera_matrix,
            self.dist_coeffs
        )
        cX = int(img_pts[0][0][0])
        cY = int(img_pts[0][0][1])
        cv2.circle(frame, (cX, cY), 6, (0, 0, 255), -1)

        # Draw axes
        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

        # Draw marker ID
        cv2.putText(frame, f'ID: {self.marker_id}',
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        return frame


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
