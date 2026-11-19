# TATARO Rehabilitation Workstation - Gazebo Digital Twin

Complete Gazebo simulation environment for the TATARO upper-limb rehabilitation robot workstation.

## Overview

This package provides a metrically accurate digital twin of the TATARO rehabilitation workstation including:

- **Workstation Table**: 0.85m × 0.85m square tabletop
- **Mounting Plate**: Steel plate for robot base attachment
- **xArm Lite6 Robot**: 6-DOF collaborative manipulator
- **Gripper Lite**: Parallel gripper for object manipulation
- **RealSense D435i Camera**: RGB-D camera for perception
- **ArUco Marker**: Fiducial marker (Dict4x4, ID: 0) for localization
- **Complete TF Tree**: All coordinate frames properly linked
- **MoveIt2 Integration**: Motion planning with environment collision checking
- **Physics Simulation**: Stable contact dynamics and collision detection

## Coordinate System

### Reference Frame: `link_base`

Located at the robot mounting flange on the mounting plate.

```
  X → forward (toward far/bottom edge of table)
  Y → right side of table
  Z → upward
  
  Origin (link_base): (0, 0, 0)
```

### Environment Geometry

| Element | Dimensions | Position | Frame |
|---------|-----------|----------|-------|
| **Tabletop** | 0.85m × 0.85m × 0.02m | (0.335, 0.0, -0.005)m | `table_frame` |
| **Mounting Plate** | 0.19m × 0.12m × 0.005m | (0.335, 0.0, -0.0075)m | `mount_plate_frame` |
| **ArUco Marker** | 0.15m × 0.15m × 0.001m | (0.335, 0.0, -0.0045)m | `marker_frame` |
| **Robot Base** | - | (0, 0, 0) | `link_base` |

### Coordinate Frames

- `link_base` - Robot base (reference frame)
- `table_frame` - Table center
- `mount_plate_frame` - Mounting plate surface
- `marker_frame` - Fiducial marker center
- `camera_color_optical_frame` - RealSense camera optical center
- `camera_depth_frame` - RealSense depth camera
- `link1` through `link6` - xArm Lite6 joint frames
- `tool_changer` - Tool mounting interface

## Installation

### Prerequisites

```bash
# ROS2 Humble
# Gazebo Harmonic
# xarm_ros2 workspace
# MoveIt2
```

### Build

```bash
cd ~/xarm_ros2
colcon build --packages-select tataro_gazebo_environment
source install/setup.bash
```

## Usage

### 1. Launch Gazebo with TATARO Workstation

```bash
ros2 launch tataro_gazebo_environment gazebo_tataro_workstation.launch.py \
  robot_ip:=192.168.1.150 \
  add_gripper:=true \
  add_d435i:=true \
  use_sim_time:=true
```

### 2. Launch Complete Environment with MoveIt2

```bash
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py \
  robot_type:=lite6 \
  add_gripper:=true \
  add_d435i:=true \
  use_sim_time:=true
```

### 3. Spawn Marker at Different Locations

```bash
# Service call to set marker pose
ros2 service call /tataro/set_marker_pose tataro_gazebo_environment_msgs/SetMarkerPose \
  "pose: {position: {x: 0.4, y: 0.1, z: -0.0045}}"
```

### 4. Reset Marker to Default Position

```bash
ros2 service call /tataro/reset_marker std_srvs/Empty
```

### 5. Start Marker Touching Experiment

```bash
# The touch_marker_controller node can be started with:
ros2 run tataro_gazebo_environment touch_marker_controller.py
```

## Package Structure

### World Files
- `worlds/tataro_workstation.sdf` - Complete Gazebo SDF world with all environment geometry

### URDF/Xacro
- `urdf/tataro_environment.urdf.xacro` - Environment URDF (table, mounting plate, marker)

### Launch Files
- `launch/gazebo_tataro_workstation.launch.py` - Basic Gazebo launcher
- `launch/tataro_with_moveit.launch.py` - Complete environment with MoveIt2

### Python Scripts
- `scripts/marker_spawner.py` - Manage marker spawning and pose updates
- `scripts/marker_pose_publisher.py` - Publish marker pose from Gazebo model state
- `scripts/touch_marker_controller.py` - Execute marker-touching experiment workflow

### Configuration
- `config/tataro_planning_scene.yaml` - MoveIt2 planning scene with collision objects
- `rviz/tataro_workstation.rviz` - RViz visualization configuration

## RViz Visualization

The default RViz configuration shows:
- Robot model (xArm Lite6 + Gripper)
- TF coordinate frames (X=Red, Y=Green, Z=Blue)
- Planning scene (table, mounting plate, marker)
- Grid and coordinate axes
- Point clouds (if camera is active)

## Motion Planning with Environment

### Add Collision Objects to Planning Scene

```python
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Table
table = CollisionObject()
table.id = "workstation_table"
table.header.frame_id = "link_base"
table.operation = CollisionObject.ADD

primitive = SolidPrimitive()
primitive.type = SolidPrimitive.BOX
primitive.dimensions = [0.85, 0.85, 0.02]  # length, width, height

pose = Pose()
pose.position.x = 0.335
pose.position.z = -0.005

table.primitives.append(primitive)
table.primitive_poses.append(pose)

# Add to planning scene
planning_scene_interface.add_object(table)
```

### Plan Motion While Avoiding Table

```python
from moveit_commander import MoveGroupCommander

move_group = MoveGroupCommander("lite6_arm")
move_group.set_pose_target(target_pose)
move_group.plan()
move_group.execute(plan, wait=True)
```

## Marker Touching Experiment Workflow

The system supports the complete workflow:

1. **Detect marker** from RealSense camera (simulated)
2. **Estimate marker pose** in camera frame
3. **Transform pose** to `link_base` reference frame
4. **Generate approach trajectory** (above marker, 5cm clearance)
5. **Move to approach position** using MoveIt2
6. **Descend vertically** to marker center (linear motion)
7. **Touch marker** (hold contact for configured duration)
8. **Retract safely** back to approach position

### Complete Example

```python
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import time

rclpy.init()
move_group = MoveGroupCommander("lite6_arm")

# 1. Get marker pose (published by marker_pose_publisher)
marker_pose = PoseStamped()
marker_pose.header.frame_id = "link_base"
marker_pose.pose.position.x = 0.335
marker_pose.pose.position.y = 0.0
marker_pose.pose.position.z = -0.0045

# 2. Create approach pose (5cm above marker)
approach_pose = PoseStamped()
approach_pose.header.frame_id = "link_base"
approach_pose.pose.position.x = marker_pose.pose.position.x
approach_pose.pose.position.y = marker_pose.pose.position.y
approach_pose.pose.position.z = marker_pose.pose.position.z + 0.05

# 3. Move to approach position
move_group.set_pose_target(approach_pose)
plan = move_group.plan()
move_group.execute(plan, wait=True)

# 4. Create touch pose (at marker center)
touch_pose = PoseStamped()
touch_pose.header.frame_id = "link_base"
touch_pose.pose.position = marker_pose.pose.position

# 5. Descend to touch (linear/Cartesian path)
waypoints = [approach_pose.pose, touch_pose.pose]
plan, fraction = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# 6. Hold contact
time.sleep(1.0)

# 7. Retract
move_group.set_pose_target(approach_pose)
plan = move_group.plan()
move_group.execute(plan, wait=True)
```

## Testing & Validation

### Check TF Tree

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific frame transformation
ros2 run tf2_ros tf2_echo link_base marker_frame

# Monitor all frames
ros2 run tf2_ros tf2_monitor
```

### Monitor Topics

```bash
# Marker pose in base frame
ros2 topic echo /tataro/marker_pose_base

# Marker pose in camera frame
ros2 topic echo /tataro/marker_pose_camera

# Camera RGB image
ros2 topic echo /camera/color/image_raw

# Depth image
ros2 topic echo /camera/aligned_depth_to_color/image_raw
```

### Validate Collision Detection

```bash
# In RViz:
# 1. Add "PlanningScene" display
# 2. Verify table and mounting plate are visible
# 3. Try planning motion - robot should avoid collisions
```

### Test Marker Position Updates

```bash
# Move marker to different position
ros2 service call /tataro/set_marker_pose tataro_gazebo_environment_msgs/SetMarkerPose \
  "pose: {position: {x: 0.5, y: 0.1, z: -0.0045}, orientation: {w: 1.0}}"

# Verify marker moved in RViz and Gazebo
```

## Physics Configuration

The Gazebo ODE physics engine is configured for:

- **Solver**: Quick solver type
- **Max Step Size**: 0.001s (1000 Hz)
- **Real-time Factor**: 1.0 (matches wall clock)
- **Contact Parameters**:
  - CFM: 0 (no constraint force mixing)
  - ERP: 0.2 (energy reduction parameter)
  - Contact max velocity: 100 m/s
  - Surface layer: 0.001m

## Camera Simulation

RealSense D435i simulated parameters:
- **Resolution**: 640×480 (RGB and Depth)
- **FPS**: 30 Hz
- **Depth Range**: 0.1m - 10m
- **FOV**: 69° horizontal, 42° vertical
- **Intrinsic parameters**: Matched to real D435i

## Performance Notes

- **Motion Planning Time**: 0.5-2.0s for complex paths
- **System Requirements**: i7+ CPU with RTX 3060+
- **Simulation Speed**: Real-time (1.0 time factor)
- **Marker Detection**: 30 Hz simulated frequency

## Troubleshooting

### Marker not detected
- Verify marker pose is within robot workspace bounds
- Check camera frame is being published in TF tree
- Ensure `marker_pose_publisher.py` node is running
- Check topic `/tataro/marker_pose_base` is publishing

### Collision/Planning errors
- Verify collision objects are in planning scene
- Check robot doesn't start in collision (use `litex_home` position)
- Increase planning attempts with `planner_attempts`
- Check IK solver configuration in MoveIt2

### TF tree incomplete
- Verify all nodes launched successfully
- Check `/tf` and `/tf_static` topics publishing
- Use `ros2 run tf2_tools view_frames > frames.pdf` for diagnosis
- Check for frame name typos in services/actions

### Gazebo not launching
- Verify `gazebo_ros` package is installed
- Check world file path in launch file
- Ensure GPU/graphics are available
- Check Gazebo logs: `~/.gazebo/server-default.log`

## Creating Custom Scenarios

### Change Marker Position

In `tataro_workstation.sdf`, modify the marker pose:

```xml
<model name="aruco_marker">
  <pose>0.4 0.05 -0.0045 0 0 0</pose>
  <!-- ... -->
</model>
```

### Add Additional Markers

Duplicate the marker model in the SDF and update:
- Model name
- Frame names
- ID number

### Modify Table Dimensions

Update in both `tataro_workstation.sdf` and `tataro_environment.urdf.xacro`:

```xml
<box size="0.85 0.85 0.02"/>  <!-- length, width, height -->
```

## References

- **xArm Documentation**: https://github.com/xArm-Developer/xarm_ros2
- **Gazebo Harmonic**: https://gazebosim.org/
- **MoveIt2**: https://moveit.ros.org/
- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **RealSense D435i**: https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html

## Citation

If you use this digital twin in research:

```bibtex
@software{tataro_gazebo_2024,
  title={TATARO Rehabilitation Workstation Gazebo Digital Twin},
  author={TATARO Team},
  year={2024}
}
```

## License

MIT License

## Support

Questions or issues:
- Email: thamizpooja1514@gmail.com
- Documentation: See files in `tataro_gazebo_environment` package
