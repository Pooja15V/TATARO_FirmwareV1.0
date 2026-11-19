#!/bin/bash
# TATARO Gazebo Digital Twin - Test Commands
# Complete set of commands for testing and validating the simulation environment

set -e

echo "======================================"
echo "TATARO Gazebo Digital Twin Test Suite"
echo "======================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ============================================================
# SETUP AND INITIALIZATION
# ============================================================

echo -e "${GREEN}=== SETUP ===${NC}"
echo ""

# Source ROS2 setup
echo "Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Source workspace
echo "Sourcing xarm_ros2 workspace..."
cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
source install/setup.bash

echo -e "${GREEN}✓ Environment ready${NC}"
echo ""

# ============================================================
# TEST 1: Basic Gazebo Launch
# ============================================================

test_gazebo_launch() {
    echo -e "${YELLOW}TEST 1: Basic Gazebo Launch${NC}"
    echo "Command: ros2 launch tataro_gazebo_environment gazebo_tataro_workstation.launch.py"
    echo ""
    echo "This will start:"
    echo "  - Gazebo with TATARO workstation world"
    echo "  - xArm Lite6 robot"
    echo "  - RealSense D435i camera"
    echo "  - Table, mounting plate, and marker geometry"
    echo ""
    echo "Usage:"
    echo "  ros2 launch tataro_gazebo_environment gazebo_tataro_workstation.launch.py \\"
    echo "    robot_ip:=192.168.1.150 \\"
    echo "    add_gripper:=true \\"
    echo "    add_d435i:=true \\"
    echo "    use_sim_time:=true"
    echo ""
}

# ============================================================
# TEST 2: Complete Environment with MoveIt2
# ============================================================

test_moveit_launch() {
    echo -e "${YELLOW}TEST 2: Complete Environment with MoveIt2${NC}"
    echo "Command: ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py"
    echo ""
    echo "This will start:"
    echo "  - Gazebo with complete workstation"
    echo "  - MoveIt2 motion planning"
    echo "  - RViz visualization"
    echo "  - Marker spawner and pose publisher"
    echo ""
    echo "Usage:"
    echo "  ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py \\"
    echo "    robot_type:=lite6 \\"
    echo "    add_gripper:=true \\"
    echo "    add_d435i:=true \\"
    echo "    use_sim_time:=true"
    echo ""
}

# ============================================================
# TEST 3: Validate Environment
# ============================================================

test_validate_environment() {
    echo -e "${YELLOW}TEST 3: Validate Environment${NC}"
    echo "Command: ros2 run tataro_gazebo_environment validate_environment.py"
    echo ""
    echo "This will check:"
    echo "  - All TF frames are present"
    echo "  - Geometry dimensions and positions"
    echo "  - Marker detection capability"
    echo "  - Collision objects"
    echo ""
    echo "Usage (in separate terminal while Gazebo/MoveIt2 is running):"
    echo "  ros2 run tataro_gazebo_environment validate_environment.py"
    echo ""
}

# ============================================================
# TEST 4: Check TF Tree
# ============================================================

test_tf_tree() {
    echo -e "${YELLOW}TEST 4: Check TF Tree${NC}"
    echo ""
    echo "View TF tree structure:"
    echo "  ros2 run tf2_tools view_frames"
    echo "  # This creates a frames.pdf file"
    echo ""
    echo "List all frames:"
    echo "  ros2 run tf2_ros tf2_monitor"
    echo ""
    echo "Check specific transformation (robot base to marker):"
    echo "  ros2 run tf2_ros tf2_echo link_base marker_frame"
    echo ""
}

# ============================================================
# TEST 5: Monitor Topics
# ============================================================

test_monitor_topics() {
    echo -e "${YELLOW}TEST 5: Monitor Topics${NC}"
    echo ""
    echo "Monitor marker pose in base frame:"
    echo "  ros2 topic echo /tataro/marker_pose_base"
    echo ""
    echo "Monitor marker pose in camera frame:"
    echo "  ros2 topic echo /tataro/marker_pose_camera"
    echo ""
    echo "List all topics:"
    echo "  ros2 topic list"
    echo ""
    echo "List all services:"
    echo "  ros2 service list"
    echo ""
}

# ============================================================
# TEST 6: Marker Manipulation
# ============================================================

test_marker_manipulation() {
    echo -e "${YELLOW}TEST 6: Marker Manipulation${NC}"
    echo ""
    echo "Reset marker to default position (center of table):"
    echo "  ros2 service call /tataro/reset_marker std_srvs/Empty"
    echo ""
    echo "Set marker pose to custom location:"
    echo "  ros2 service call /tataro/set_marker_pose \\"
    echo "    tataro_gazebo_environment_msgs/SetMarkerPose \\"
    echo "    'pose: {position: {x: 0.4, y: 0.1, z: -0.0045}, orientation: {w: 1.0}}'"
    echo ""
    echo "Common test positions:"
    echo "  - Center of table: x=0.335, y=0.0, z=-0.0045"
    echo "  - Left side:       x=0.335, y=-0.2, z=-0.0045"
    echo "  - Right side:      x=0.335, y=0.2, z=-0.0045"
    echo "  - Front:           x=0.5, y=0.0, z=-0.0045"
    echo "  - Back:            x=0.2, y=0.0, z=-0.0045"
    echo ""
}

# ============================================================
# TEST 7: Motion Planning
# ============================================================

test_motion_planning() {
    echo -e "${YELLOW}TEST 7: Motion Planning${NC}"
    echo ""
    echo "In RViz:"
    echo "  1. Add 'PlanningScene' display (if not already added)"
    echo "  2. Verify table and mounting plate are visible"
    echo "  3. Use 'Motion Planning' plugin to plan motions"
    echo "  4. Robot should avoid collisions with table"
    echo ""
    echo "Example: Move to marker approaching pose"
    echo "  - Set target pose above marker"
    echo "  - Click 'Plan and Execute'"
    echo "  - Robot should move without hitting table"
    echo ""
}

# ============================================================
# TEST 8: Camera Verification
# ============================================================

test_camera() {
    echo -e "${YELLOW}TEST 8: Camera Verification${NC}"
    echo ""
    echo "Check camera topics:"
    echo "  ros2 topic list | grep camera"
    echo ""
    echo "View RGB image:"
    echo "  ros2 run image_view image_view image:=/camera/color/image_raw"
    echo ""
    echo "View depth image:"
    echo "  ros2 run image_view image_view image:=/camera/aligned_depth_to_color/image_raw"
    echo ""
    echo "Verify point cloud:"
    echo "  ros2 topic echo /camera/depth/color/points"
    echo ""
}

# ============================================================
# TEST 9: Coordinate System Verification
# ============================================================

test_coordinates() {
    echo -e "${YELLOW}TEST 9: Coordinate System Verification${NC}"
    echo ""
    echo "Verify reference frame orientation:"
    echo "  - X-axis (red): Points forward (toward far table edge)"
    echo "  - Y-axis (green): Points right"
    echo "  - Z-axis (blue): Points upward"
    echo ""
    echo "Expected coordinates relative to link_base:"
    echo "  - Table center:        (0.335,  0.0, -0.005)"
    echo "  - Mounting plate:      (0.335,  0.0, -0.0075)"
    echo "  - Marker (default):    (0.335,  0.0, -0.0045)"
    echo "  - Robot base:          (0,      0,    0)"
    echo ""
    echo "Verify in RViz:"
    echo "  1. Select 'link_base' frame"
    echo "  2. Check coordinate axes are correct"
    echo "  3. Look at interactive markers for geometry positions"
    echo ""
}

# ============================================================
# TEST 10: Marker Touching Workflow
# ============================================================

test_marker_touching() {
    echo -e "${YELLOW}TEST 10: Marker Touching Workflow${NC}"
    echo ""
    echo "Manual workflow test:"
    echo "  1. Start complete environment:"
    echo "     ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py"
    echo ""
    echo "  2. In another terminal, set marker position:"
    echo "     ros2 service call /tataro/set_marker_pose \\"
    echo "       tataro_gazebo_environment_msgs/SetMarkerPose \\"
    echo "       'pose: {position: {x: 0.4, y: 0.05, z: -0.0045}}'"
    echo ""
    echo "  3. In RViz, plan motion to approach marker"
    echo ""
    echo "  4. Execute planned motion"
    echo ""
    echo "  5. Verify marker pose updated in TF:"
    echo "     ros2 run tf2_ros tf2_echo link_base marker_frame"
    echo ""
    echo "  6. (Optional) Start touch controller:"
    echo "     ros2 run tataro_gazebo_environment touch_marker_controller.py"
    echo ""
}

# ============================================================
# TEST 11: Performance Metrics
# ============================================================

test_performance() {
    echo -e "${YELLOW}TEST 11: Performance Metrics${NC}"
    echo ""
    echo "Monitor simulation performance:"
    echo "  - Check Gazebo stats: Window → Plotting → see FPS"
    echo "  - Monitor CPU usage: top"
    echo "  - Check message rates: ros2 topic hz /topic_name"
    echo ""
    echo "Example: Check marker publishing rate"
    echo "  ros2 topic hz /tataro/marker_pose_base"
    echo ""
    echo "Expected:"
    echo "  - Gazebo: 30+ FPS"
    echo "  - Marker: 30 Hz"
    echo "  - TF tree: 100+ Hz"
    echo "  - Simulation time factor: 1.0"
    echo ""
}

# ============================================================
# TEST 12: Troubleshooting
# ============================================================

test_troubleshooting() {
    echo -e "${YELLOW}TEST 12: Troubleshooting Commands${NC}"
    echo ""
    echo "Check for errors:"
    echo "  - Gazebo log: tail -f ~/.gazebo/server-default.log"
    echo "  - ROS warnings: grep -i warning /tmp/ros_logs/*"
    echo ""
    echo "Verify required packages:"
    echo "  - Check xarm_ros2: ros2 pkg list | grep xarm"
    echo "  - Check MoveIt2: ros2 pkg list | grep moveit"
    echo "  - Check gazebo_ros: ros2 pkg list | grep gazebo"
    echo ""
    echo "Test communication:"
    echo "  - Test TF publishing: ros2 topic hz /tf"
    echo "  - Test Gazebo topics: ros2 topic list | grep gazebo"
    echo ""
}

# ============================================================
# RUN ALL TESTS
# ============================================================

echo "Showing test descriptions (tests not executed)."
echo "Copy and paste commands from below into separate terminals."
echo ""

test_gazebo_launch
test_moveit_launch
test_validate_environment
test_tf_tree
test_monitor_topics
test_marker_manipulation
test_motion_planning
test_camera
test_coordinates
test_marker_touching
test_performance
test_troubleshooting

echo -e "${GREEN}✓ Test suite documentation printed${NC}"
echo ""
echo "Next steps:"
echo "  1. Open terminal 1: ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py"
echo "  2. Open terminal 2: ros2 run tataro_gazebo_environment validate_environment.py"
echo "  3. Open terminal 3: Test individual commands from above"
echo ""
