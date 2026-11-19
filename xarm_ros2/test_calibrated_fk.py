#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xArm-Python-SDK')

from xarm.wrapper import XArmAPI

# Connect to robot
arm = XArmAPI('192.168.1.150', is_radian=True)
print("Connected to robot")

# Test forward kinematics with calibration
joint_angles = [0, 0, 0, 0, 0, 0]  # Home position
code, pose = arm.get_forward_kinematics(joint_angles, input_is_radian=False, return_is_radian=False)

if code == 0:
    print(f"\n✓ Forward Kinematics (CALIBRATED):")
    print(f"  Joint angles: {joint_angles}")
    print(f"  End-effector pose: {pose}")
    print(f"  Format: [x, y, z, roll, pitch, yaw]")
    print(f"  This includes calibrated parameters!")
else:
    print(f"Error code: {code}")

arm.disconnect()
