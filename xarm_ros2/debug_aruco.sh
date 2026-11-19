#!/bin/bash
# Debug ArUco detection issues

echo "========================================="
echo "TATARO ArUco Detection Debug"
echo "========================================="
echo ""

source ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/install/setup.bash

echo "1. Check if ArUco node is running..."
ros2 node list | grep aruco
echo ""

echo "2. Check marker_id parameter..."
ros2 param get /aruco/aruco_detector marker_id
echo ""

echo "3. Check marker_size parameter..."
ros2 param get /aruco/aruco_detector marker_size
echo ""

echo "4. Check available topics..."
echo "Topics from aruco:"
ros2 topic list | grep aruco
echo ""

echo "5. Check camera topics..."
echo "Camera topics:"
ros2 topic list | grep camera | head -10
echo ""

echo "6. Try to read marker topic (will timeout after 5 seconds)..."
echo "If you see marker data, detection is working!"
timeout 5 ros2 topic echo /aruco/marker 2>/dev/null || echo "❌ No markers detected"
echo ""

echo "7. View TF tree..."
echo "Looking for aruco_marker_frame..."
ros2 tf2_ros tf2_echo camera_rgb_optical_frame aruco_marker_frame 2>/dev/null || echo "❌ Transform not found"
echo ""

echo "========================================="
echo "TROUBLESHOOTING TIPS:"
echo "========================================="
echo "✓ Is your marker ID 0? Check: ros2 param set /aruco/aruco_detector marker_id 0"
echo "✓ Is your marker 15cm? Check: ros2 param set /aruco/aruco_detector marker_size 0.15"
echo "✓ Is the camera streaming? Check: ros2 run rqt_image_view rqt_image_view"
echo "✓ Is lighting good? ArUco needs good contrast"
echo "✓ Try with: ros2 launch aruco_ros detector.launch.py marker_id:=0 marker_size:=0.15"
echo ""
