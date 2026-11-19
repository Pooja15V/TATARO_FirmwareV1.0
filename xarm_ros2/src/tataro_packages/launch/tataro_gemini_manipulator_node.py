import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
import time

from google import genai
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from xarm.wrapper import XArmAPI

class GeminiRoboticsNode(Node):
    def __init__(self):
        super().__init__('gemini_robotics_node')
        self.bridge = CvBridge()
        self.client = genai.Client()
        self.latest_color = None
        self.latest_depth = None
        self.intrinsics = None

        # 1. Connect to xArm hardware
        ROBOT_IP = "192.168.1.150"
        self.get_logger().info(f"Connecting to xArm at {ROBOT_IP}...")
        try:
            self.arm = XArmAPI(ROBOT_IP)
            self.arm.clean_warn()
            self.arm.clean_error()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)      # Position control mode
            self.arm.set_state(state=0)  # Ready state
            self.get_logger().info("xArm connected and armed successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to xArm: {e}")
            self.arm = None

        # 2. Setup TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 3. Subscriptions
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(String, '/robot_command', self.command_cb, 10)

        self.get_logger().info("Gemini Robotics ROS 2 node ready. Waiting for commands on /robot_command...")

    def info_cb(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = {
                "fx": msg.k[0], "fy": msg.k[4],
                "cx": msg.k[2], "cy": msg.k[5]
            }

    def color_cb(self, msg: Image):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_cb(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def command_cb(self, msg: String):
        prompt_text = msg.data
        if self.latest_color is None or self.latest_depth is None or self.intrinsics is None:
            self.get_logger().warn("Sensors not yet calibrated or publishing frames.")
            return

        self.get_logger().info(f"Processing task: \"{prompt_text}\"")

        _, buffer = cv2.imencode('.jpg', self.latest_color)
        image_bytes = buffer.tobytes()

        api_prompt = f"""
        Task instruction: '{prompt_text}'
        Locate the red cuboid mentioned in the instruction.
        Return strictly valid JSON with no markdown formatting:
        {{"label": "red_cuboid", "point": [y, x]}}
        where y and x are normalized integer coordinates scaled from 0 to 1000 representing the center grasp point.
        """

        # Model hierarchy: Try robotics endpoint first, fall back to high-capacity multimodal models
        candidate_models = [
            "gemini-robotics-er-2-preview",
            "gemini-3.8-flash",
            "gemini-3.1-pro-preview",
        ]

        response = None
        for model_name in candidate_models:
            try:
                self.get_logger().info(f"Querying model: {model_name}...")
                response = self.client.models.generate_content(
                    model=model_name,
                    contents=[
                        genai.types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg"),
                        api_prompt
                    ],
                    config=genai.types.GenerateContentConfig(
                        temperature=0.1
                    )
                )
                if response and response.text:
                    self.get_logger().info(f"Success from {model_name}")
                    break
            except Exception as e:
                self.get_logger().warn(f"Model {model_name} unavailable: {e}. Trying fallback...")
                time.sleep(1.0)

        if response is None or not response.text:
            self.get_logger().error("All Gemini model endpoints failed.")
            return

        try:
            raw_text = response.text.strip().replace('```json', '').replace('```', '')
            parsed = json.loads(raw_text)
            norm_y, norm_x = parsed["point"]
            self.dispatch_target(norm_y, norm_x)
        except Exception as e:
            self.get_logger().error(f"Failed to parse model output: {response.text} | Error: {e}")
    
    
    def dispatch_target(self, norm_y, norm_x):
        h, w = self.latest_depth.shape[:2]
        u = int((norm_x / 1000.0) * w)
        v = int((norm_y / 1000.0) * h)

        patch = self.latest_depth[max(0, v-2):min(h, v+3), max(0, u-2):min(w, u+3)]
        valid = patch[patch > 0]
        if len(valid) == 0:
            self.get_logger().error("Zero depth reading at selected coordinate.")
            return
        z_m = float(np.median(valid)) / 1000.0

        x_c = (u - self.intrinsics["cx"]) * z_m / self.intrinsics["fx"]
        y_c = (v - self.intrinsics["cy"]) * z_m / self.intrinsics["fy"]

        self.get_logger().info(f"Target Camera Frame 3D: X={x_c:.3f}m, Y={y_c:.3f}m, Z={z_m:.3f}m")

        # Create PointStamped
        pt_cam = PointStamped()
        pt_cam.header.frame_id = 'camera_color_optical_frame'
        pt_cam.header.stamp = self.get_clock().now().to_msg()
        pt_cam.point.x = x_c
        pt_cam.point.y = y_c
        pt_cam.point.z = z_m

        try:
            # Transform from eye-in-hand camera to robot base
            transform = self.tf_buffer.lookup_transform(
                'link_base',
                'camera_color_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pt_base = do_transform_point(pt_cam, transform)

            target_x = pt_base.point.x
            target_y = pt_base.point.y
            target_z = pt_base.point.z

            self.get_logger().info(f"==> Target Robot Base Frame 3D: X={target_x:.3f}m, Y={target_y:.3f}m, Z={target_z:.3f}m")

            if self.arm is not None:
                self.execute_pick_and_lift(target_x, target_y, target_z)

        except Exception as e:
            self.get_logger().error(f"TF lookup/transform to link_base failed: {e}")

    def execute_pick_and_lift(self, x_m, y_m, z_m):
        """Commands xArm via XArmAPI to execute physical pick and lift."""
        # Convert meters to millimeters for xArm SDK
        x = x_m * 1000.0
        y = y_m * 1000.0
        z = z_m * 1000.0

        # Standard top-down grasp orientation (wrist pointing straight down)
        # Adjust roll/pitch/yaw if your end-effector requires a specific tool orientation
        roll, pitch, yaw = 180.0, 0.0, 0.0

        self.get_logger().info(f"Executing pick at Base Coordinates (mm): X={x:.1f}, Y={y:.1f}, Z={z:.1f}")

        # 1. Open gripper fully
        self.arm.set_gripper_position(800, wait=True)

        # 2. Pre-grasp approach: 80mm above target to avoid table collisions
        self.arm.set_position(x=x, y=y, z=z + 80.0, roll=roll, pitch=pitch, yaw=yaw, speed=100, wait=True)

        # 3. Descend to target pick height
        self.arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=50, wait=True)

        # 4. Close gripper on cuboid
        self.arm.set_gripper_position(50, wait=True)
        time.sleep(0.5)

        # 5. Lift target 150mm vertically
        self.arm.set_position(x=x, y=y, z=z + 150.0, roll=roll, pitch=pitch, yaw=yaw, speed=80, wait=True)
        self.get_logger().info("Pick and lift task complete!")

def main(args=None):
    rclpy.init(args=args)
    node = GeminiRoboticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()