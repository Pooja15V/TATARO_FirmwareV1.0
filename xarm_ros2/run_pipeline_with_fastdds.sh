#!/bin/bash
set -e

cd "$(dirname "$0")"

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Configure FastDDS with larger buffers
export FASTDDS_QOS_PROFILES_FILE="$(pwd)/fastdds_profile.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Kill any existing RealSense drivers
pkill -f "realsense2_camera" || true
sleep 2

# Start RealSense driver with sync enabled
echo "Starting RealSense driver..."
ros2 launch realsense2_camera rs_launch.py tf_publish_rate:=0.0 enable_sync:=true align_depth.enable:=true &
DRIVER_PID=$!
sleep 5

# Run the pipeline
echo "Starting TATARO CV Pipeline..."
python3 src/computer_vision/TATARO_CV_PipelineV1.0.py

# Cleanup
kill $DRIVER_PID 2>/dev/null || true
