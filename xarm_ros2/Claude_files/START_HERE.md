# 🚀 TATARO Gazebo Simulation - START HERE

## Quick Launch (5 minutes)

### Step 1: Open a Terminal
```bash
cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
```

### Step 2: Source Environment
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Step 3: Launch Simulation
```bash
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py \
  robot_type:=lite6 \
  add_gripper:=true \
  add_d435i:=true \
  use_sim_time:=true
```

**What you'll see**:
- Gazebo window opens with the workstation
- RViz window shows the robot and environment  
- TF tree with all coordinate frames
- Table, mounting plate, and ArUco marker visible

---

## 📊 What's Included

```
TATARO Rehabilitation Workstation
├── 🤖 Robot: xArm Lite6 (6-DOF collaborative arm)
├── 🖐️  Gripper: Gripper Lite (parallel gripper)
├── 📷 Camera: RealSense D435i (RGB-D sensor)
├── 📋 Table: 0.85m × 0.85m workstation table
├── 🔧 Mount: Steel mounting plate
├── 📌 Marker: ArUco fiducial (Dict4x4, ID 0)
└── 🎯 Planning: MoveIt2 motion planning integrated
```

---

## ✅ Verify Installation

In a **new terminal**, run:
```bash
cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
source install/setup.bash
ros2 run tataro_gazebo_environment validate_environment.py
```

---

## 📚 Documentation

| File | Purpose |
|------|---------|
| **Gazebo_implementation-ReadMe.md** | Complete technical reference |
| **TF_tree_and_coordinates.md** | Coordinate system reference |
| **Example_test_commands.sh** | Test and validation commands |

---

## 🎮 Using Gazebo & RViz

### Gazebo Controls
- **Zoom**: Scroll wheel
- **Rotate**: Right-click + drag
- **Pan**: Middle-click + drag

### RViz Motion Planning
1. Set target pose using interactive marker
2. Click **Plan and Execute**
3. Robot avoids table automatically

---

## 📍 Coordinate Reference

```
Reference Frame: link_base (robot base at origin)

         +Y (right/green)
          ↑
          |
    +Z ↑  |
  (up)|  |
      | /
      |/____→ +X (forward/red)
```

### Key Positions (meters, relative to link_base)
- **Table center**: (0.335, 0.0, -0.005)
- **Marker default**: (0.335, 0.0, -0.0045)

---

## 🚀 You're Ready!

```bash
ros2 launch tataro_gazebo_environment tataro_with_moveit.launch.py
```

Enjoy! 🎉
