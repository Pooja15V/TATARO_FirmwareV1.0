import time
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
from xarm.wrapper import XArmAPI
from google import genai

class AutoGeminiHandEyeCalibrator(Node):
    def __init__(self):
        super().__init__('auto_gemini_handeye_calibrator')
        self.bridge = CvBridge()
        self.client = genai.Client()

        # 1. Connect to UFACTORY Lite 6
        ROBOT_IP = "192.168.1.150"
        self.get_logger().info(f"Connecting to Lite 6 at {ROBOT_IP}...")
        self.arm = XArmAPI(ROBOT_IP)
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)      # Position control mode
        self.arm.set_state(0)     # Motion ready
        self.get_logger().info("Lite 6 connected and ready.")

        # 2. Camera Buffers & Subscriptions
        self.latest_color = None
        self.latest_depth = None
        self.intrinsics = None

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)

        # 3. Calibration Storage
        self.recorded_flange_poses = []  # List of 4x4 T_base_flange
        self.recorded_cam_points = []    # List of 4D [x, y, z, 1.0]

    def info_cb(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5]
            }

    def color_cb(self, msg: Image):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_cb(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # ==========================================
    # Trajectory Generation
    # ==========================================
    def compute_look_at_euler(self, camera_pos, target_pos, roll_offset_deg=0.0):
        forward = target_pos - camera_pos
        forward = forward / np.linalg.norm(forward)

        world_up = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(forward, world_up)) > 0.99:
            world_up = np.array([0.0, 1.0, 0.0])

        right = np.cross(world_up, forward)
        right = right / np.linalg.norm(right)

        up = np.cross(forward, right)
        up = up / np.linalg.norm(up)

        # Tool coordinate frame where -Z points along visual line of sight
        rot_matrix = np.column_stack((right, up, -forward))
        if roll_offset_deg != 0.0:
            twist = R.from_euler('z', roll_offset_deg, degrees=True).as_matrix()
            rot_matrix = rot_matrix @ twist

        return R.from_matrix(rot_matrix).as_euler('xyz', degrees=True)

    def generate_hemisphere_poses(self, center_xyz, radius_mm=270.0):
        poses = []
        # Apex Pose (Directly Overhead)
        apex_pos = np.array([center_xyz[0], center_xyz[1], center_xyz[2] + radius_mm])
        poses.append(np.concatenate([apex_pos, [180.0, 0.0, 0.0]]))

        # Ring 1: 6 Poses at 65 deg elevation
        elev1 = np.radians(65.0)
        for i in range(6):
            azimuth = 2.0 * np.pi * i / 6.0
            x = center_xyz[0] + radius_mm * np.cos(elev1) * np.cos(azimuth)
            y = center_xyz[1] + radius_mm * np.cos(elev1) * np.sin(azimuth)
            z = center_xyz[2] + radius_mm * np.sin(elev1)
            twist = 15.0 if i % 2 == 0 else -15.0
            rot = self.compute_look_at_euler(np.array([x, y, z]), center_xyz, twist)
            poses.append(np.concatenate([[x, y, z], rot]))

        # Ring 2: 8 Poses at 45 deg elevation (High angular variation)
        elev2 = np.radians(45.0)
        for i in range(8):
            azimuth = (2.0 * np.pi * i / 8.0) + (np.pi / 8.0)
            x = center_xyz[0] + radius_mm * np.cos(elev2) * np.cos(azimuth)
            y = center_xyz[1] + radius_mm * np.cos(elev2) * np.sin(azimuth)
            z = center_xyz[2] + radius_mm * np.sin(elev2)
            twist = 20.0 if i % 2 == 0 else -20.0
            rot = self.compute_look_at_euler(np.array([x, y, z]), center_xyz, twist)
            poses.append(np.concatenate([[x, y, z], rot]))

        return poses

    # ==========================================
    # Gemini Point Extraction & Back-Projection
    # ==========================================
    def extract_point_from_gemini(self, rgb_image):
        _, buf = cv2.imencode('.jpg', rgb_image)
        prompt = """
        Identify the red cuboid or target calibration marker on the table.
        Return strictly valid JSON:
        {"point": [y, x]}
        where coordinates are normalized integers from 0 to 1000 representing the exact center point.
        """
        response = self.client.models.generate_content(
            model="gemini-robotics-er-2-preview",
            contents=[genai.types.Part.from_bytes(data=buf.tobytes(), mime_type="image/jpeg"), prompt],
            config=genai.types.GenerateContentConfig(temperature=0.0)
        )
        raw = response.text.strip().replace('```json', '').replace('```', '')
        data = json.loads(raw)
        return data["point"]  # [norm_y, norm_x]

    def pixel_to_camera_point(self, norm_pt):
        h, w = self.latest_depth.shape[:2]
        u = int((norm_pt[1] / 1000.0) * w)
        v = int((norm_pt[0] / 1000.0) * h)

        # 5x5 median patch
        patch = self.latest_depth[max(0, v-2):min(h, v+3), max(0, u-2):min(w, u+3)]
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None

        z_m = float(np.median(valid)) / 1000.0
        x_m = (u - self.intrinsics['cx']) * z_m / self.intrinsics['fx']
        y_m = (v - self.intrinsics['cy']) * z_m / self.intrinsics['fy']
        return np.array([x_m, y_m, z_m, 1.0])

    def get_current_flange_matrix(self):
        """Reads arm flange pose and constructs 4x4 T_base_flange transformation matrix."""
        code, pose = self.arm.get_position(is_radian=False)
        if code != 0:
            return None
        # pose: [x, y, z, roll, pitch, yaw] in mm and degrees
        t = np.array(pose[:3]) / 1000.0  # Convert to meters
        rot = R.from_euler('xyz', pose[3:], degrees=True).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = t
        return T

    # ==========================================
    # Execution & Optimization Loop
    # ==========================================
    def run_auto_calibration(self):
        self.get_logger().info("Waiting for RealSense RGB and Depth feeds...")
        while self.latest_color is None or self.latest_depth is None or self.intrinsics is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        # Workspace target center in robot base frame (mm)
        TARGET_CENTER = np.array([450.0, 60.0, 15.0])
        poses = self.generate_hemisphere_poses(TARGET_CENTER, radius_mm=270.0)
        self.get_logger().info(f"Generated {len(poses)} hemisphere viewpoints.")

        for i, pose in enumerate(poses):
            x, y, z, roll, pitch, yaw = pose
            self.get_logger().info(f"[{i+1}/{len(poses)}] Moving Lite 6 to: X={x:.1f}, Y={y:.1f}, Z={z:.1f} | Roll={roll:.1f}, Pitch={pitch:.1f}, Yaw={yaw:.1f}")

            ret = self.arm.set_position(
                x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw,
                is_radian=False, speed=40, mvacc=400, wait=True
            )

            if ret != 0:
                self.get_logger().warn(f"Pose {i+1} rejected by kinematics. Error: {self.arm.error_code}. Skipping.")
                self.arm.clean_warn()
                self.arm.clean_error()
                continue

            # Settle mechanical vibration
            time.sleep(1.2)
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.1)

            # Query Gemini ER 2
            try:
                norm_pt = self.extract_point_from_gemini(self.latest_color)
                p_cam = self.pixel_to_camera_point(norm_pt)
                T_flange = self.get_current_flange_matrix()

                if p_cam is not None and T_flange is not None:
                    self.recorded_flange_poses.append(T_flange)
                    self.recorded_cam_points.append(p_cam)
                    self.get_logger().info(f"  Captured Pair: Cam 3D = [{p_cam[0]:.3f}, {p_cam[1]:.3f}, {p_cam[2]:.3f}] m")
                else:
                    self.get_logger().warn("  Invalid depth or flange pose. Sample discarded.")
            except Exception as e:
                self.get_logger().error(f"  Gemini query failed on sample {i+1}: {e}")

        # Return arm to safe retreat pose
        self.get_logger().info("Sampling finished. Returning to safe standoff position...")
        self.arm.set_position(x=300.0, y=0.0, z=300.0, roll=180.0, pitch=0.0, yaw=0.0, is_radian=False, speed=40, wait=True)
        self.arm.disconnect()

        # Run Hand-Eye Optimizer
        self.solve_hand_eye()

    def solve_hand_eye(self):
        N = len(self.recorded_cam_points)
        if N < 8:
            self.get_logger().error(f"Insufficient samples collected ({N}/8 required). Calibration aborted.")
            return

        self.get_logger().info(f"Solving Hand-Eye optimization over {N} observation pairs...")

        def loss_func(params):
            t_cam = params[0:3]
            r_cam = R.from_rotvec(params[3:6]).as_matrix()
            T_flange_cam = np.eye(4)
            T_flange_cam[:3, :3] = r_cam
            T_flange_cam[:3, 3] = t_cam

            p_base_target = params[6:9]
            residuals = []
            for T_bf, p_c in zip(self.recorded_flange_poses, self.recorded_cam_points):
                p_base_calc = (T_bf @ T_flange_cam @ p_c)[:3]
                residuals.extend(p_base_calc - p_base_target)
            return np.sum(np.square(residuals))

        # Initial seed: estimated physical bracket values
        # [tx, ty, tz, rx, ry, rz (Rodrigues vector for ~90 deg yaw), target_x, target_y, target_z]
        init_guess = np.array([
            0.069, -0.032, 0.019,
            0.0, 0.0, 1.5708,
            0.450, 0.060, 0.015
        ])

        res = minimize(loss_func, init_guess, method='Nelder-Mead', options={'maxiter': 5000})

        t_opt = res.x[0:3]
        r_opt = R.from_rotvec(res.x[3:6])
        q_opt = r_opt.as_quat()  # [qx, qy, qz, qw]
        rms_error_mm = np.sqrt(res.fun / N) * 1000.0

        print("\n" + "=" * 60)
        print("          GEMINI ROBOTICS ER 2 CALIBRATION RESULT          ")
        print("=" * 60)
        print(f"Residual RMS Invariance Error: {rms_error_mm:.2f} mm")
        print(f"Translation [X, Y, Z] (meters):")
        print(f"  x: {t_opt[0]:.6f}")
        print(f"  y: {t_opt[1]:.6f}")
        print(f"  z: {t_opt[2]:.6f}")
        print(f"Rotation Quaternion [qx, qy, qz, qw]:")
        print(f"  qx: {q_opt[0]:.6f}")
        print(f"  qy: {q_opt[1]:.6f}")
        print(f"  qz: {q_opt[2]:.6f}")
        print(f"  qw: {q_opt[3]:.6f}")
        print("-" * 60)
        print("Ready-to-run Static TF Publisher Command:")
        print(f"ros2 run tf2_ros static_transform_publisher \\\n"
              f"  --x {t_opt[0]:.6f} --y {t_opt[1]:.6f} --z {t_opt[2]:.6f} \\\n"
              f"  --qx {q_opt[0]:.6f} --qy {q_opt[1]:.6f} --qz {q_opt[2]:.6f} --qw {q_opt[3]:.6f} \\\n"
              f"  --frame-id link_eef --child-frame-id camera_color_optical_frame")
        print("=" * 60 + "\n")

def main(args=None):
    rclpy.init(args=args)
    calibrator = AutoGeminiHandEyeCalibrator()
    try:
        calibrator.run_auto_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()