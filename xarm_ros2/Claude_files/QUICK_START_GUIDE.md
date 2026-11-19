# TATARO Gazebo Digital Twin - Quick Start Guide

Get the simulation running in 5 minutes!

## Prerequisites

- ROS2 Humble installed
- Gazebo Harmonic installed
- xarm_ros2 workspace built
- MoveIt2 installed

## Step 1: Build the Package

```bash
cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
colcon build --packages-select tataro_gazebo_environment
source install/setup.bash
```

## Step 2: Start the Complete Simulation

Open a terminal and run:

```bash
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py \
  robot_type:=lite6 \
  add_gripper:=true \
  add_d435i:=true \
  use_sim_time:=true
```

This will launch:
- **Gazebo** with the workstation environment (table, mounting plate, marker)
- **xArm Lite6** robot with gripper
- **RealSense D435i** camera
- **MoveIt2** motion planning
- **RViz** visualization
- **Marker tools** (spawner and pose publisher)

Wait for "all systems ready" message.

## Step 3: Verify the Environment

Open another terminal and validate:

```bash
ros2 run tataro_gazebo_environment validate_environment.py
```

You should see:
```
✓ Frame "link_base" found
✓ Frame "table_frame" found
✓ Frame "marker_frame" found
✓ Table position correct: (0.3350, 0.0000, 0.0050)
✓ Marker position correct: (0.3350, 0.0000, -0.0045)
...
✓ All validation checks passed!
```

## Step 4: View the Simulation

In **Gazebo**:
- You'll see the robot on the table
- The table, mounting plate, and marker are visible
- The camera is mounted on the robot
- The world is lit with proper shadows

In **RViz**:
- The robot model (Lite6 + Gripper)
- The table and mounting plate as collision objects
- The marker (ArUco fiducial)
- All coordinate frames (red=X, green=Y, blue=Z)
- The grid and workspace

## Step 5: Move the Robot

In **RViz** Motion Planning plugin:

1. Go to the `Motion Planning` tab
2. Click on the `Planning` sub-tab
3. Use the interactive marker (6-DOF) to set a target position
4. Click `Plan` to compute trajectory
5. Click `Execute` to move the robot

The robot will avoid collisions with the table!

## Step 6: Reposition the Marker

Open a new terminal:

```bash
# Reset marker to table center
ros2 service call /tataro/reset_marker std_srvs/Empty

# Move marker to a custom location
ros2 service call /tataro/set_marker_pose \
  tataro_gazebo_environment_msgs/SetMarkerPose \
  "pose: {position: {x: 0.4, y: 0.1, z: -0.0045}}"
```

The marker moves in both Gazebo and RViz!

## Step 7: Check Marker Pose

In another terminal:

```bash
# Monitor marker position (base frame)
ros2 topic echo /tataro/marker_pose_base

# Check marker transform
ros2 run tf2_ros tf2_echo link_base marker_frame
```

You'll see the marker's position relative to the robot.

## Common Tasks

### Plan Motion to Marker

```python
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped

# Get move group
move_group = MoveGroupCommander("lite6_arm")

# Target above marker (5cm clearance)
target = PoseStamped()
target.header.frame_id = "link_base"
target.pose.position.x = 0.335  # marker X
target.pose.position.y = 0.0    # marker Y
target.pose.position.z = 0.045  # 5cm above marker

# Plan and execute
move_group.set_pose_target(target)
plan = move_group.plan()
move_group.execute(plan, wait=True)
```

### Move Marker to New Location

```bash
ros2 service call /tataro/set_marker_pose \
  tataro_gazebo_environment_msgs/SetMarkerPose \
  "pose: {
    position: {x: 0.5, y: -0.2, z: -0.0045},
    orientation: {w: 1.0}
  }"
```

### Check All Active Topics

```bash
ros2 topic list
```

### Monitor Performance

Check Gazebo statistics in the GUI:
- Windows → Plotting → see real-time FPS

Expected: **30+ FPS**

## Troubleshooting

### "Marker frame not found"

**Solution**: Wait 2-3 seconds for TF tree to initialize, then retry.

```bash
sleep 3
ros2 run tataro_gazebo_environment validate_environment.py
```

### "Robot starts in collision"

**Solution**: Move to home position first:

```bash
# In Python with MoveIt2
move_group.set_named_target("home")
move_group.execute(move_group.plan())
```

### Gazebo not starting

**Solution**: Check graphics setup:

```bash
glxinfo | grep "OpenGL version"
```

If no output, update graphics drivers or use software rendering:

```bash
export LIBGL_ALWAYS_INDIRECT=1
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py
```

### RViz shows "No transform from world to link_base"

**Solution**: RViz needs a moment to initialize. 
- Wait 3-5 seconds after launch
- Change Fixed Frame to `world` then back to `link_base`

## Key Differences from Physical System

| Aspect | Simulation | Physical |
|--------|-----------|----------|
| Time factor | 1.0 (real-time) | Real-time |
| Marker collision | Soft (can be pushed) | Hard (rigid) |
| Camera noise | None (ideal) | Real noise |
| Network latency | None | ~20-50ms |
| Power constraints | None | Battery/power cord |

## Next Steps

1. **Run validation**: `ros2 run tataro_gazebo_environment validate_environment.py`
2. **Test motion planning**: Use RViz Motion Planning plugin
3. **Implement experiments**: Write Python scripts using MoveIt2 and ROS2
4. **Calibration**: Use marker detection for hand-eye calibration
5. **Data collection**: Record rosbag data for analysis

## Useful Commands Reference

```bash
# Launch simulation
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py

# Validate environment
ros2 run tataro_gazebo_environment validate_environment.py

# View TF tree
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo link_base marker_frame

# Monitor topics
ros2 topic echo /tataro/marker_pose_base
ros2 topic hz /tataro/marker_pose_base

# Control marker
ros2 service call /tataro/reset_marker std_srvs/Empty
ros2 service call /tataro/set_marker_pose tataro_gazebo_environment_msgs/SetMarkerPose "pose: {position: {x: 0.4, y: 0.1, z: -0.0045}}"

# View frame structure
ros2 run tf2_ros tf2_monitor
```

## File Organization

```
tataro_gazebo_environment/
├── worlds/
│   └── tataro_workstation.sdf          ← Complete Gazebo world
├── urdf/
│   └── tataro_environment.urdf.xacro   ← Environment geometry
├── launch/
│   ├── gazebo_tataro_workstation.launch.py
│   └── tataro_with_moveit.launch.py    ← Complete system launch
├── scripts/
│   ├── marker_spawner.py               ← Marker control
│   ├── marker_pose_publisher.py        ← Pose estimation
│   ├── touch_marker_controller.py      ← Experiment workflow
│   └── validate_environment.py         ← Validation
├── config/
│   └── tataro_planning_scene.yaml      ← MoveIt2 setup
├── rviz/
│   └── tataro_workstation.rviz         ← RViz config
└── Claude_files/
    ├── Gazebo_implementation-ReadMe.md
    ├── TF_tree_and_coordinates.md
    └── QUICK_START_GUIDE.md            ← You are here
```

## Support

- **Issues**: Check RViz error messages
- **Logs**: `ros2 topic echo` to monitor topics
- **Validation**: Run `validate_environment.py` to diagnose setup
- **Documentation**: See `Gazebo_implementation-ReadMe.md`

---

**You're ready to go!** The digital twin is now running. 🚀
