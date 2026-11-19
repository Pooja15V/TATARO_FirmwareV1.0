#!/bin/bash
# TATARO Gazebo Digital Twin - Launch Script
# Complete simulation with Gazebo, MoveIt2, and RViz

set -e

echo "======================================"
echo "TATARO Gazebo Simulation Launcher"
echo "======================================"
echo ""

# Navigate to workspace
cd /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2

# Source ROS2 and workspace
echo "Setting up environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✓ Environment configured"
echo ""

# Launch parameters
ROBOT_IP="192.168.1.150"
ADD_GRIPPER="true"
ADD_D435I="true"
USE_SIM_TIME="true"

echo "Launch Parameters:"
echo "  Robot IP: $ROBOT_IP"
echo "  Add Gripper: $ADD_GRIPPER"
echo "  Add D435i Camera: $ADD_D435I"
echo "  Use Sim Time: $USE_SIM_TIME"
echo ""

echo "Starting TATARO Gazebo simulation..."
echo "This will launch:"
echo "  • Gazebo with workstation environment"
echo "  • xArm Lite6 robot with gripper"
echo "  • RealSense D435i camera"
echo "  • MoveIt2 motion planning"
echo "  • RViz visualization"
echo "  • Marker spawner and pose publisher"
echo ""

# Launch the simulation
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py \
  robot_type:=lite6 \
  add_gripper:=$ADD_GRIPPER \
  add_d435i:=$ADD_D435I \
  use_sim_time:=$USE_SIM_TIME

# Note: This will run in foreground. Press Ctrl+C to stop.
