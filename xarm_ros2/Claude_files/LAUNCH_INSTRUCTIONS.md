# TATARO Gazebo Simulation - Launch Instructions

## 🎯 Quick Start (Copy & Paste)

### Terminal 1: Launch Full Simulation
```bash
cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py \
  robot_type:=lite6 \
  add_gripper:=true \
  add_d435i:=true \
  use_sim_time:=true
```

This launches:
- ✅ Gazebo with TATARO workstation (table, mounting plate, marker)
- ✅ xArm Lite6 robot with Gripper Lite
- ✅ RealSense D435i camera
- ✅ MoveIt2 motion planning
- ✅ RViz visualization
- ✅ Marker spawner node
- ✅ Marker pose publisher (30 Hz)

---

### Terminal 2: Validate Environment (Optional)
```bash
cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run tataro_gazebo_environment validate_environment.py
```

This verifies:
- ✅ All TF frames present
- ✅ Geometry positioned correctly
- ✅ Marker detection ready
- ✅ Collision checking enabled

---

### Terminal 3: Test Marker Movement (Optional)
```bash
cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Move marker left
ros2 service call /tataro/set_marker_pose \
  tataro_gazebo_environment_msgs/SetMarkerPose \
  "pose: {position: {x: 0.335, y: -0.2, z: -0.0045}}"

# Move marker right
ros2 service call /tataro/set_marker_pose \
  tataro_gazebo_environment_msgs/SetMarkerPose \
  "pose: {position: {x: 0.335, y: 0.2, z: -0.0045}}"

# Reset to center
ros2 service call /tataro/reset_marker std_srvs/Empty
```

---

## 📋 Launch Variants

### Minimal (Gazebo only, no planning)
```bash
ros2 launch tataro_gazebo_environment gazebo_tataro_workstation.launch.py \
  robot_ip:=192.168.1.150 \
  add_gripper:=true \
  add_d435i:=true \
  use_sim_time:=true
```

### Full (with MoveIt2, RViz, marker control)
```bash
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py \
  robot_type:=lite6 \
  add_gripper:=true \
  add_d435i:=true \
  use_sim_time:=true
```

### Without Camera
```bash
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py \
  add_gripper:=true \
  add_d435i:=false \
  use_sim_time:=true
```

### Without Gripper
```bash
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py \
  add_gripper:=false \
  add_d435i:=true \
  use_sim_time:=true
```

---

## ✅ What to Expect

### Gazebo Window
- Dark ground plane
- Light wood table (0.85m × 0.85m)
- Dark steel mounting plate (centered on table)
- White ArUco marker (0.15m × 0.15m) on table center
- xArm Lite6 robot (mounted on plate)
- Red/green/blue coordinate axes at robot base
- Directional light from above
- Point light near work area

### RViz Window
- Robot model (xArm with gripper)
- Table, mounting plate, marker geometries
- TF coordinate frames with axes (X=red, Y=green, Z=blue)
- Interactive markers for motion planning
- Motion Planning plugin ready
- Camera feeds (if D435i enabled)

### Console Output
- Gazebo initialization messages
- MoveIt2 loading planning scene
- RViz startup messages
- TF publishing at 100+ Hz
- Marker publishing at 30 Hz

---

## 🔧 Troubleshooting

### Issue: "Package not found"
```bash
# Rebuild the package
colcon build --packages-select tataro_gazebo_environment

# Resource workspace
source install/setup.bash
```

### Issue: Gazebo doesn't start (headless/remote)
```bash
# Set virtual display
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1

# Or use software rendering
export MESA_GL_VERSION_OVERRIDE=4.5
export LIBGL_ALWAYS_SOFTWARE=1
```

### Issue: RViz window blank
```
In RViz:
1. Top-left "Fixed Frame" dropdown
2. Change to "link_base"
3. Click "Reset view" in View menu
4. Displays should appear
```

### Issue: Marker not visible
```bash
# Check if marker is being published
ros2 topic list | grep marker

# Verify marker pose
ros2 topic echo /tataro/marker_pose_base
```

### Issue: Motion planning very slow
```
In RViz Motion Planning plugin:
1. Increase "Planning Time" to 5.0 seconds
2. Reduce "Goal Tolerance"
3. Click "Plan" to compute trajectory first
```

---

## 📊 System Requirements

- **OS**: Ubuntu 22.04 (with ROS2 Humble)
- **CPU**: i7+ (or equivalent)
- **GPU**: NVIDIA GPU recommended (but CPU works)
- **RAM**: 8GB minimum
- **Display**: X11 display or virtual display

### Performance Notes
- **Gazebo FPS**: 30+ (should be smooth)
- **Marker publish rate**: 30 Hz
- **TF publish rate**: 100+ Hz
- **Real-time factor**: 1.0 (matches wall clock)

---

## 📚 File Organization

```
xarm_ros2/
├── src/tataro_gazebo_environment/
│   ├── worlds/
│   │   └── tataro_workstation.sdf (complete environment)
│   ├── urdf/
│   │   └── tataro_environment.urdf.xacro
│   ├── launch/
│   │   ├── gazebo_tataro_workstation.launch.py
│   │   └── tataro_with_moveit.launch.py (this one!)
│   ├── config/
│   │   └── tataro_planning_scene.yaml
│   ├── rviz/
│   │   └── tataro_workstation.rviz
│   ├── scripts/
│   │   ├── marker_spawner.py
│   │   ├── marker_pose_publisher.py
│   │   ├── touch_marker_controller.py
│   │   └── validate_environment.py
│   └── package.xml, CMakeLists.txt
│
└── Claude_files/
    ├── START_HERE.md
    ├── Gazebo_implementation-ReadMe.md
    ├── TF_tree_and_coordinates.md
    └── Example_test_commands.sh
```

---

## 🎓 Next Steps After Launch

1. **Explore Gazebo**
   - Zoom/rotate/pan the view
   - Click objects to inspect them
   - Verify physics is working

2. **Check RViz**
   - View robot model
   - Check TF tree
   - Enable Motion Planning plugin

3. **Test Motion Planning**
   - Use interactive marker to set goal
   - Click "Plan and Execute"
   - Observe collision avoidance

4. **Test Marker**
   - Move marker to different positions
   - Replan motions
   - Verify robot avoids table

5. **Monitor Sensors**
   - Check camera topics
   - View RGB and depth images
   - Monitor point cloud

---

## 🆘 Getting Help

### Check Package
```bash
ros2 pkg prefix tataro_gazebo_environment
```

### View Logs
```bash
tail -f ~/.ros/log/latest_run/*/
```

### Monitor Topics
```bash
ros2 topic list
ros2 topic echo /topic_name
ros2 topic hz /topic_name
```

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
```

---

## ✨ You're All Set!

Copy the launch command from **Terminal 1** above and run it now.

**Have fun with TATARO! 🚀**
