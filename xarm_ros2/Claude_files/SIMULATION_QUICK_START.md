# Gazebo Simulation → Real Robot: Quick Start Guide

## **FILES CREATED**

✅ **World file**: `xarm_gazebo/worlds/tataro_marker_table.world`
- 85×85 cm table with ArUco marker at center
- Physics engine configured
- Exact geometry matching physical setup

✅ **Simulation launch**: `xarm_gazebo/launch/lite6_tataro_marker_simulation.launch.py`
- Gazebo with fake controller
- MoveIt2 planning (no real hardware)
- RViz visualization

✅ **Runtime launch**: `tataro_packages/launch/tatro_runtime_simulation.launch.py`
- Hand-eye calibration publisher
- Fake ArUco marker publisher
- RViz

✅ **Fake marker publisher**: `tataro_packages/fake_aruco_publisher.py`
- Simulates camera + ArUco detection
- Publishes fake marker pose
- Works with same tataro_touch_marker_centre script

---

## **SIMULATION WORKFLOW (3 Terminals)**

### **Terminal 1: Start Gazebo Simulation**

```bash
cd /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
source install/setup.bash

ros2 launch xarm_gazebo lite6_tataro_marker_simulation.launch.py
```

**Expected**: Gazebo window opens with table and robot

---

### **Terminal 2: Start TATARO Runtime**

```bash
cd /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
source install/setup.bash

ros2 launch tataro_packages tatro_runtime_simulation.launch.py
```

**Expected**: RViz opens with robot model + marker visible

---

### **Terminal 3: Run Marker Touch Script**

```bash
cd /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
source install/setup.bash

ros2 run tataro_packages tataro_touch_marker_centre --ros-args -p hover_m:=0.20
```

**Expected Output** (SUCCESS):
```
[MARKER]  aruco_marker_frame  in link_base
  x= +335.0  y=   +0.0  z=  -5.0 mm

[TARGET]  Hover position (marker center + 200 mm vertical offset)
  x= +335.0  y=   +0.0  z= +195.0 mm

[MOVE] Moving to hover position above marker ... arrived ✓

[SUCCESS] TCP is 200 mm above marker center at:
  Position: x=335.0, y=0.0, z=195.0 mm
  Orientation: roll=180.0°  pitch=0.0°  yaw=0.0°
```

---

## **WHAT TO VERIFY IN SIMULATION**

- ✅ Gazebo shows table + marker + robot
- ✅ RViz shows all frames correctly
- ✅ Marker detected (aruco_marker_frame published)
- ✅ Robot moves to hover position without collisions
- ✅ Orientation (180, 0, 0) doesn't cause arm twisting
- ✅ Script completes with [SUCCESS]

If all pass → **Ready for real robot!**

---

## **TRANSITION TO REAL ROBOT**

### **Only change the launch commands:**

```bash
# SIMULATION (what you just did):
ros2 launch xarm_gazebo lite6_tataro_marker_simulation.launch.py
ros2 launch tataro_packages tatro_runtime_simulation.launch.py

# REAL ROBOT (replace with):
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py \
  robot_ip:=192.168.1.150 \
  add_gripper:=true \
  add_realsense_d435i:=true \
  add_d435i_links:=false

ros2 launch tataro_packages tatro_runtime.launch.py \
  robot_ip:=192.168.1.150 \
  calib_name:=tataro_handeye_3A
```

**The script stays the same:**
```bash
# Works for BOTH simulation and real:
ros2 run tataro_packages tataro_touch_marker_centre --ros-args -p hover_m:=0.20
```

---

## **REAL ROBOT FIRST RUN**

```bash
# Terminal 1: Real robot
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py \
  robot_ip:=192.168.1.150 \
  add_gripper:=true \
  add_realsense_d435i:=true \
  add_d435i_links:=false

# Terminal 2: TATARO runtime
ros2 launch tataro_packages tatro_runtime.launch.py \
  robot_ip:=192.168.1.150 \
  calib_name:=tataro_handeye_3A

# Terminal 3: Run with SAFE hover height first
ros2 run tataro_packages tataro_touch_marker_centre \
  --ros-args -p hover_m:=0.30  # 300 mm (extra safe)
```

If successful, decrease hover:
```bash
ros2 run tataro_packages tataro_touch_marker_centre \
  --ros-args -p hover_m:=0.20  # 200 mm (normal)
```

---

## **TROUBLESHOOTING**

| Issue | Solution |
|-------|----------|
| Gazebo won't start | `sudo apt install gazebo` |
| Marker not detected | Check `fake_aruco_publisher` is running |
| Robot won't move | Check MoveIt is configured in RViz |
| Real robot position off | Run `tataro_calibration_diagnostic.py` |
| Real robot can't reach | Increase `hover_m` to 0.30 or 0.40 |

---

## **NEXT: Full Documentation**

See these files in Claude_files/ for complete details:
- `COMPLETE_TF_AND_CALIBRATION_ANALYSIS.md` - TF tree deep dive
- `HAND_EYE_CALIBRATION_FIX_GUIDE.md` - Calibration troubleshooting

Good luck! 🚀
