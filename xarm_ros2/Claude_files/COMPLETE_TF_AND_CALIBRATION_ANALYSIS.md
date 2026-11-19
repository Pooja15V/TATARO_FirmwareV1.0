# Complete TF Tree, Calibration, and Marker Touch Pipeline Analysis
## xArm Lite6 + RealSense D435i + Hand-Eye Calibration

**System**: UFactory Lite6 with Gripper Lite + Intel D435i (on link_eef)  
**Launch**: `lite6_moveit_realmove.launch.py` with `add_d435i_links:=false`  
**Calibration**: `tataro_handeye_3A`  
**Objective**: Touch marker centre with mm-level accuracy

---

## **PART 1: COMPLETE TF TREE ANALYSIS**

### **1.1 TF Tree Structure (with add_d435i_links=false)**

```
world
  └── link_base (robot base, mounted on table at z=0)
        │
        ├── link_1
        │    └── link_2
        │         └── link_3
        │              └── link_4
        │                   └── link_5
        │                        └── link_6
        │                             └── link_eef (end-effector/gripper base)
        │                                  └── link_eef_tcp (tool center point)
        │                                  │
        │                                  └── [D435i Camera - Published by Hand-Eye Calibration]
        │                                       ├── camera_link (camera housing)
        │                                       │    └── camera_color_frame
        │                                       │         └── camera_color_optical_frame
        │                                       │    └── camera_depth_frame
        │                                       │         └── camera_depth_optical_frame
        │
        └── [planning frame] (MoveIt planning group)
             └── manipulator

aruco_marker_frame (published by ArUco detector in camera_color_optical_frame)
```

### **1.2 Key Frame Definitions**

| Frame | Parent | Description | Notes |
|-------|--------|-------------|-------|
| `link_base` | `world` | Robot mounting point | Origin at robot flange top surface (z=0) |
| `link_1` to `link_6` | Sequential | xArm joint frames | Standard DH parameters |
| `link_eef` | `link_6` | End-effector base | Gripper attachment point |
| `link_eef_tcp` | `link_eef` | Tool center point | Offset from gripper base |
| `camera_link` | `link_eef` | Camera housing frame | Published by hand-eye calibration |
| `camera_color_frame` | `camera_link` | Color sensor frame | RealSense internal frame |
| `camera_color_optical_frame` | `camera_color_frame` | Optical frame (image coords) | **KEY FRAME** |
| `camera_depth_frame` | `camera_link` | Depth sensor frame | Separate from color |
| `aruco_marker_frame` | `camera_color_optical_frame` | Detected marker pose | Published by ArUco detector |

### **1.3 Why add_d435i_links=false Matters**

**When `add_d435i_links=false` (YOUR SETUP):**
- ✅ Camera internal frames NOT defined in URDF
- ✅ No "nominal" camera offset that conflicts with calibration
- ✅ **Hand-eye calibration is the ONLY source of truth for camera-to-EEF transform**
- ✅ Avoids TF duplication/conflicts

**Complete TF Lookup Chain:**
```
link_base → link_1 → ... → link_6 → link_eef → camera_link → camera_color_optical_frame
```

---

## **PART 2: HAND-EYE CALIBRATION ANALYSIS**

### **2.1 tataro_handeye_3A.calib File**

```yaml
parameters:
  name: tataro_handeye_3
  calibration_type: eye_in_hand
  robot_base_frame: link_base
  robot_effector_frame: link_eef
  tracking_base_frame: camera_color_optical_frame
  tracking_marker_frame: aruco_marker_frame

transform:
  translation:
    x: 0.06529188359968596      # 65.29 mm forward
    y: 0.02683690968181594      # 26.84 mm right
    z: 0.02684959591361675      # 26.85 mm upward
  rotation:
    x: -0.0020731694569079855   # Quat X
    y: 0.009958334296928828     # Quat Y
    z: -0.6828369163337878      # Quat Z
    w: 0.730500020012455        # Quat W
```

### **2.2 Interpretation**

**eye_in_hand**: Camera mounted on gripper (link_eef)

**Transform meaning:**
- Camera is **65.29 mm forward, 26.84 mm right, 26.85 mm up** relative to gripper
- Rotation defined by quaternion (slight tilt, not neutral)
- Published as: `T(link_eef → camera_color_optical_frame)`

**This is loaded by:** `easy_handeye2/handeye_publisher` node when you specify `calib_name:=tataro_handeye_3A`

---

## **PART 3: MARKER TOUCH TRANSFORMATION CHAIN**

### **3.1 Complete Transformation Sequence**

```
Step 1: ArUco detects marker in camera image
        → Publishes marker pose in camera_color_optical_frame

Step 2: TF2 transforms marker to robot base
        marker_pose_link_base = T_fk * T_handeye * marker_pose_camera

        Where:
          T_fk = forward kinematics from joint angles
          T_handeye = tataro_handeye_3A (65mm, 27mm, 27mm + rotation)
          marker_pose_camera = ArUco detector output

Step 3: Compute hover target
        target_x = marker_x
        target_y = marker_y
        target_z = marker_z + hover_offset (e.g., 0.200 m)

Step 4: Plan trajectory
        MoveIt2 plans from current_pose → target_pose
        
Step 5: Execute motion
        Robot moves via joint controllers
        TCP reaches target position
```

### **3.2 Mathematical Formulation**

```
T(link_base ← aruco_marker) = 
    T(link_base ← link_eef) * 
    T(link_eef ← camera_color_optical_frame) * 
    T(camera_color_optical_frame ← aruco_marker)

Substituting:
    = J(θ) * H * M

Where:
  J(θ) = Jacobian/FK from joint angles θ
  H = hand-eye calibration transform
  M = ArUco marker detection transform
```

### **3.3 ROS2 TF Lookup**

```bash
# Get complete chain
ros2 run tf2_ros tf2_echo link_base aruco_marker_frame

# Breaks down to:
ros2 run tf2_ros tf2_echo link_base link_eef          # FK (dynamic)
ros2 run tf2_ros tf2_echo link_eef camera_color_optical_frame  # Calibration (static)
ros2 run tf2_ros tf2_echo camera_color_optical_frame aruco_marker_frame  # Detection
```

---

## **PART 4: MOTION STRATEGY**

### **4.1 Safe Touch Sequence**

```
PHASE 1: APPROACH
  Detect marker → compute hover position → move to hover (200 mm above)

PHASE 2: VERIFY
  Read actual TCP position → check error < 25 mm

PHASE 3: DESCENT
  Slow descent to touch position (table surface + 10 mm offset)

PHASE 4: HOLD
  Maintain contact briefly

PHASE 5: RETREAT
  Upward motion back to safe height
```

### **4.2 Configuration**

```python
HOVER_OFFSET_M = 0.200              # 200 mm above marker
MARKER_SURFACE_Z = -0.005           # Table surface in link_base
TOUCH_OFFSET_Z = 0.010              # 10 mm above table (safety)
APPROACH_SPEED = 50                 # mm/s
DESCENT_SPEED = 10                  # mm/s (slow)
TARGET_ORIENTATION = (180, 0, 0)    # Downward-looking gripper
```

### **4.3 Orientation: (180, 0, 0)**

**Why 180° roll:**
- Your gripper is mounted on link_eef
- You want it pointing downward at table
- (180, 0, 0) makes the gripper face down without arm twisting
- Natural approach vector perpendicular to table

---

## **PART 5: VALIDATION COMMANDS**

### **5.1 Verify TF Tree Is Complete**

```bash
# Terminal 1: Launch system
ros2 launch tataro_packages tatro_runtime.launch.py \
  robot_ip:=192.168.1.150 \
  calib_name:=tataro_handeye_3A

# Terminal 2: Check frames
ros2 run tf2_tools view_frames
# Opens /tmp/frames.pdf with complete tree

# Verify critical transforms
ros2 run tf2_ros tf2_echo link_base link_eef
# Updates continuously (FK)

ros2 run tf2_ros tf2_echo link_eef camera_color_optical_frame
# Static (from calibration)

ros2 run tf2_ros tf2_echo camera_color_optical_frame aruco_marker_frame
# Updates when marker visible
```

### **5.2 Verify Hand-Eye Calibration**

```bash
# Check it's loaded
ros2 param get /handeye_publisher name
# Expected: tataro_handeye_3A

# Verify calibration file
python3 << 'EOF'
import yaml
with open('.../Calibration/tataro_handeye_3A.calib') as f:
    c = yaml.safe_load(f)
    t = c['transform']['translation']
    print(f"TX={t['x']*1000:.1f} mm, TY={t['y']*1000:.1f} mm, TZ={t['z']*1000:.1f} mm")
EOF
```

### **5.3 Verify Marker Detection**

```bash
# Check marker frame exists
ros2 topic echo /tf | grep aruco_marker_frame

# See marker pose
ros2 topic echo /aruco/markers --once
```

### **5.4 Run Complete Diagnostic**

```bash
python3 /path/to/tataro_calibration_diagnostic.py

# Reports:
# - Marker detected at: (X, Y, Z)
# - Target computed at: (X, Y, Z+0.200)
# - Robot moved to: (X', Y', Z')
# - Error: ΔX, ΔY, ΔZ
# - PASS if error < 25 mm XY, < 50 mm Z
```

---

## **PART 6: COMPLETE EXECUTION**

### **Terminal 1: Start System**

```bash
cd /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2

ros2 launch tataro_packages tatro_runtime.launch.py \
  robot_ip:=192.168.1.150 \
  calib_name:=tataro_handeye_3A

# Keep this running
# Verify: ArUco detection window opens, marker visible
```

### **Terminal 2: Run Marker Touch**

```bash
source ./install/setup.bash

# First time: measure error
python3 ./src/tataro_packages/tataro_calibration_diagnostic.py

# If error acceptable:
ros2 run tataro_packages tataro_touch_marker_centre \
  --ros-args -p hover_m:=0.20

# If successful:
# - Marker detected
# - Robot moves to hover position (200 mm above)
# - Shows [SUCCESS]
```

### **Terminal 3: Monitor (Optional)**

```bash
# Watch TF
ros2 run tf2_ros tf2_monitor link_base aruco_marker_frame

# Or watch TCP in RViz
# Should see TCP move to marker position ±25 mm
```

---

## **ACCEPTANCE CRITERIA**

✅ **System is ready when:**

1. `tatro_runtime.launch.py` starts without errors
2. `tataro_handeye_3A` is loaded by handeye_publisher
3. TF tree shows: `link_base → ... → link_eef → camera_color_optical_frame`
4. ArUco marker detected when visible to camera
5. `tataro_calibration_diagnostic` reports: XY error < 25 mm
6. `tataro_touch_marker_centre` completes successfully
7. Physical marker center ≈ RViz marker center (±15 mm)

---

## **TROUBLESHOOTING**

| Problem | Diagnosis | Solution |
|---------|-----------|----------|
| Marker not detected | Lighting, visibility, size | Check camera feed in rqt_image_view |
| Error > 50 mm | Calibration issue | Try tataro_handeye_3C or 3D |
| Arm twists | Orientation wrong | Already using (180, 0, 0) - should work |
| Can't reach position | Outside workspace | Increase hover_m to 0.30 |

**All documentation saved to:** `/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/Claude_files/`
