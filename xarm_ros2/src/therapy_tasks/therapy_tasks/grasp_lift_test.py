#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanSingleStraight, PlanExec

from therapy_tasks.motion import TherapyMotion


# ============================================================
# TATARO GRASP + LIFT TEST
#
# Uses UFACTORY xArm planning services:
#   /xarm_straight_plan
#   /xarm_exec_plan
#
# NO Level-1 motion.
# ============================================================

ROBOT_IP = "192.168.1.150"

# Calibrated grasp pose from robot TCP reading
GRASP_X_MM = -7.7
GRASP_Y_MM = -187.8
GRASP_Z_MM = 53.6

GRASP_ROLL_DEG = -155.2
GRASP_PITCH_DEG = -84.6
GRASP_YAW_DEG = -22.1

# Approach / lift distance
LIFT_OFFSET_MM = 100.0

# Planning frame
BASE_FRAME = "link_base"


def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    Convert XYZ roll/pitch/yaw in degrees to quaternion.

    Returns:
        x, y, z, w
    """

    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)

    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)

    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def make_pose(x_mm, y_mm, z_mm):
    """
    Create a geometry_msgs/Pose in metres.

    Position comes from the calibrated robot-base-frame pose.
    Orientation remains fixed at the calibrated grasp orientation.
    """

    pose = Pose()

    pose.position.x = x_mm / 1000.0
    pose.position.y = y_mm / 1000.0
    pose.position.z = z_mm / 1000.0

    qx, qy, qz, qw = rpy_to_quaternion(
        GRASP_ROLL_DEG,
        GRASP_PITCH_DEG,
        GRASP_YAW_DEG,
    )

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    return pose


class PlannedGraspTest(Node):

    def __init__(self):
        super().__init__("tataro_grasp_lift_planned_test")

        self.plan_client = self.create_client(
            PlanSingleStraight,
            "/xarm_straight_plan",
        )

        self.exec_client = self.create_client(
            PlanExec,
            "/xarm_exec_plan",
        )

    def wait_for_services(self):

        self.get_logger().info(
            "Waiting for /xarm_straight_plan ..."
        )

        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return False

        self.get_logger().info(
            "Waiting for /xarm_exec_plan ..."
        )

        while not self.exec_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return False

        self.get_logger().info(
            "Planning services available."
        )

        return True

    def plan_and_execute(self, pose, description):

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(description)
        self.get_logger().info("=" * 60)

        request = PlanSingleStraight.Request()
        request.target = pose

        self.get_logger().info(
            "Target position: "
            f"x={pose.position.x:.4f} m, "
            f"y={pose.position.y:.4f} m, "
            f"z={pose.position.z:.4f} m"
        )

        self.get_logger().info(
            "Target quaternion: "
            f"[{pose.orientation.x:.5f}, "
            f"{pose.orientation.y:.5f}, "
            f"{pose.orientation.z:.5f}, "
            f"{pose.orientation.w:.5f}]"
        )

        future = self.plan_client.call_async(request)

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            self.get_logger().error(
                "Planning request did not complete."
            )
            return False

        response = future.result()

        if response is None:
            self.get_logger().error(
                "No planning response received."
            )
            return False

        if not response.success:
            self.get_logger().error(
                "xarm_straight_plan FAILED."
            )
            return False

        self.get_logger().info(
            "Planning succeeded."
        )

        exec_request = PlanExec.Request()
        exec_request.wait = True

        exec_future = self.exec_client.call_async(exec_request)

        while rclpy.ok() and not exec_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not exec_future.done():
            self.get_logger().error(
                "Execution request did not complete."
            )
            return False

        exec_response = exec_future.result()

        if exec_response is None:
            self.get_logger().error(
                "No execution response received."
            )
            return False

        if not exec_response.success:
            self.get_logger().error(
                "xarm_exec_plan FAILED."
            )
            return False

        self.get_logger().info(
            "Execution succeeded."
        )

        return True


def main():

    rclpy.init()

    node = PlannedGraspTest()

    print("")
    print("=" * 70)
    print("TATARO — PLANNED GRASP + LIFT DIAGNOSTIC TEST")
    print("=" * 70)

    print("")
    print("Grasp pose:")
    print(f"  X     = {GRASP_X_MM} mm")
    print(f"  Y     = {GRASP_Y_MM} mm")
    print(f"  Z     = {GRASP_Z_MM} mm")
    print(f"  Roll  = {GRASP_ROLL_DEG} deg")
    print(f"  Pitch = {GRASP_PITCH_DEG} deg")
    print(f"  Yaw   = {GRASP_YAW_DEG} deg")

    pregrasp_z = GRASP_Z_MM + LIFT_OFFSET_MM

    print("")
    print("Pre-grasp:")
    print(f"  X     = {GRASP_X_MM} mm")
    print(f"  Y     = {GRASP_Y_MM} mm")
    print(f"  Z     = {pregrasp_z} mm")

    print("")
    print("Sequence:")
    print("  1. Open gripper")
    print("  2. PLAN + MOVE to pre-grasp")
    print("  3. PLAN + MOVE vertically to grasp")
    print("  4. Close gripper")
    print("  5. PLAN + MOVE vertically back to pre-grasp")
    print("  6. HOLD")

    print("")
    print("NO LEVEL 1 MOTION WILL BE PERFORMED.")
    print("")

    answer = input(
        "Type YES to start the planned grasp + lift test: "
    ).strip()

    if answer != "YES":
        print("Test cancelled.")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:

        # ----------------------------------------------------
        # Connect gripper only.
        # Arm movement is handled by xArm planner.
        # ----------------------------------------------------

        print("")
        print("[1/5] Connecting to robot / gripper...")

        motion = TherapyMotion(
            ROBOT_IP,
            {
                "speed_reach_mm_s": 200.0,
                "speed_grasp_mm_s": 60.0,
                "mvacc_mm_s2": 1000.0,
                "hover_offset_mm": 60.0,
            },
            {
                "close_wait_s": 1.0,
                "open_wait_s": 1.0,
                "tcp_load_open_kg": 0.277,
                "tcp_load_open_com_mm": [0, 0, 30],
                "tcp_load_closed_kg": 0.35,
                "tcp_load_closed_com_mm": [0, 0, 40],
            },
        )

        print("      Robot connected.")

        # ----------------------------------------------------
        # Open gripper
        # ----------------------------------------------------

        print("")
        print("[2/5] Opening gripper...")

        motion.open_gripper()

        print("      Gripper OPEN.")

        # ----------------------------------------------------
        # Wait for planning services
        # ----------------------------------------------------

        print("")
        print("[3/5] Checking planning services...")

        if not node.wait_for_services():
            raise RuntimeError(
                "MoveIt/xArm planning services unavailable."
            )

        # ----------------------------------------------------
        # Pre-grasp
        # ----------------------------------------------------

        pregrasp_pose = make_pose(
            GRASP_X_MM,
            GRASP_Y_MM,
            pregrasp_z,
        )

        print("")
        print("[4/5] Planning to PRE-GRASP...")

        if not node.plan_and_execute(
            pregrasp_pose,
            "MOVE TO PRE-GRASP",
        ):
            raise RuntimeError(
                "Pre-grasp planning/execution failed."
            )

        print("")
        print("Pre-grasp reached.")

        time.sleep(0.5)

        # ----------------------------------------------------
        # Grasp
        # ----------------------------------------------------

        grasp_pose = make_pose(
            GRASP_X_MM,
            GRASP_Y_MM,
            GRASP_Z_MM,
        )

        print("")
        print("Planning vertical descent to GRASP...")

        if not node.plan_and_execute(
            grasp_pose,
            "MOVE TO GRASP",
        ):
            raise RuntimeError(
                "Grasp planning/execution failed."
            )

        print("")
        print("Grasp pose reached.")

        # ----------------------------------------------------
        # Close gripper
        # ----------------------------------------------------

        print("")
        print("[5/5] Closing gripper...")

        motion.close_gripper()

        print("      Gripper CLOSED.")

        time.sleep(1.0)

        # ----------------------------------------------------
        # Lift back to pre-grasp
        # ----------------------------------------------------

        print("")
        print("Planning +100 mm vertical LIFT...")

        if not node.plan_and_execute(
            pregrasp_pose,
            "LIFT OBJECT +100 MM",
        ):
            raise RuntimeError(
                "Lift planning/execution failed."
            )

        print("")
        print("=" * 70)
        print("SUCCESS")
        print("=" * 70)
        print("")
        print("Object should now be held at:")
        print(f"  Z = {pregrasp_z} mm")
        print("")
        print("Robot will HOLD.")
        print("Gripper will remain CLOSED.")
        print("NO automatic release.")
        print("")

        # Keep node alive while robot holds.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)

    except KeyboardInterrupt:

        print("")
        print("Interrupted by user.")

    except Exception as exc:

        print("")
        print("!" * 70)
        print("TEST FAILED")
        print("!" * 70)
        print(str(exc))
        print("!" * 70)

    finally:

        print("")
        print("Disconnecting...")

        try:
            motion.disconnect()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()