#!/usr/bin/env python3
"""
Extract calibrated kinematics by querying get_forward_kinematics()
and save to YAML format compatible with lite6_default_kinematics.yaml
"""
import sys
sys.path.insert(0, '/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xArm-Python-SDK')

from xarm.wrapper import XArmAPI
import yaml
import os
import numpy as np

def extract_calibrated_kinematics(robot_ip, output_suffix='calibrated'):
    """
    Extract calibrated kinematics by probing robot FK at each joint
    """
    arm = XArmAPI(robot_ip, is_radian=True)
    print("Connected to robot")

    # Home position
    home_pose = [0, 0, 0, 0, 0, 0]

    # Lite6 has 6 DOF (hardcoded for Lite6)
    dof = 6
    print(f"Robot DOF: {dof}")

    kinematics = {}

    # For each joint, get FK at that joint's zero position
    print("\nExtracting calibrated kinematics...")
    for joint_idx in range(dof):
        joint_angles = [0] * dof

        # Query FK with this joint configuration
        code, pose = arm.get_forward_kinematics(
            joint_angles,
            input_is_radian=False,
            return_is_radian=True
        )

        if code == 0:
            # Extract position and orientation
            joint_param = {
                'x': float(pose[0]),  # mm
                'y': float(pose[1]),  # mm
                'z': float(pose[2]),  # mm
                'roll': float(pose[3]),  # radians
                'pitch': float(pose[4]),  # radians
                'yaw': float(pose[5]),  # radians
            }
            kinematics[f'joint{joint_idx + 1}'] = joint_param
            print(f"Joint {joint_idx + 1}: x={pose[0]:.4f}, y={pose[1]:.4f}, z={pose[2]:.4f}")
        else:
            print(f"Error getting FK for joint {joint_idx + 1}: {code}")
            return False

    # Create YAML structure
    data = {'kinematics': kinematics}

    # Save to file
    output_dir = '/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/xarm_description/config/kinematics/user'
    os.makedirs(output_dir, exist_ok=True)

    robot_name = 'lite6'
    output_file = os.path.join(output_dir, f'{robot_name}_kinematics_{output_suffix}.yaml')

    with open(output_file, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

    print(f"\n✓ Calibrated kinematics saved to: {output_file}")

    # Show file contents
    print("\n=== Generated YAML File ===")
    with open(output_file, 'r') as f:
        print(f.read())

    arm.disconnect()
    return True

if __name__ == '__main__':
    robot_ip = '192.168.1.150'
    suffix = 'calibrated'

    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
    if len(sys.argv) > 2:
        suffix = sys.argv[2]

    success = extract_calibrated_kinematics(robot_ip, suffix)
    sys.exit(0 if success else 1)
