# CalibrationPin RViz Visualization Guide

**Setup:** UFactory xArm Lite6 + uFlite Gripper + CalibrationPin (ArUco)

---

## Option 1: Quick Visualization (Easiest) 🚀

### Step 1: Launch RViz with Robot
```bash
# Terminal 1: Launch robot + MoveIt + RViz
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py \
  robot_ip:=192.168.1.150 \
  add_gripper:=true \
  add_realsense_d435i:=true \
  add_d435i_links:=false
```

### Step 2: Add CalibrationPin Mesh to RViz Display

1. In RViz, click **"Add"** (bottom left panel)
2. Select **"Mesh Resource"** display type
3. Configure with these settings:

```yaml
Display Name: CalibrationPin
Type: Mesh Resource
Topic: /tf
Mesh Resource: package://xarm_description/meshes/CalibrationPin.stl
Selectable: false

Transform:
  Frame: link_tcp
  Position X: 0
  Position Y: 0
  Position Z: 0.015
  Orientation X: 0
  Orientation Y: 0
  Orientation Z: 0
  Orientation W: 1

Color: RGBA (0.2, 0.6, 0.8, 1.0)
```

### Step 3: Visualize
- The CalibrationPin mesh should now appear extending from the gripper TCP
- It will move with the robot as you control it

---

## Option 2: Integrated URDF Approach (Best for Persistent Visualization) 🎯

### Files Already Created

✅ **calibration_pin.urdf.xacro** - Defines the calibration pin link  
✅ **calibrationpin_uflite_gripper_macro.xacro** - Gripper + calibration pin combined  
✅ **CalibrationPin.stl** - 3D mesh file ready to use

### How to Use

The new macro can be used in place of the standard gripper macro:

**In any robot URDF that includes gripper:**

```xml
<!-- OLD WAY (without calibration pin) -->
<xacro:uflite_gripper_macro prefix="" attach_to="link_eef" />

<!-- NEW WAY (with calibration pin) -->
<xacro:calibrationpin_uflite_gripper_macro prefix="" attach_to="link_eef" />
```

### Full Robot Description Example

Create file: `lite6_with_calibration_pin.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="xarm_lite6">

  <!-- Import components -->
  <xacro:include filename="$(find xarm_description)/urdf/lite6/lite6.urdf.xacro" />
  <xacro:include filename="$(find xarm_description)/urdf/gripper/calibrationpin_uflite_gripper_macro.xacro" />

  <!-- Robot with calibration pin gripper -->
  <xacro:lite6_urdf prefix="" />
  <xacro:calibrationpin_uflite_gripper_macro 
    prefix="" 
    attach_to="link_eef" 
    attach_xyz="0 0 0" 
    attach_rpy="0 0 0" />

</robot>
```

---

## Expected Result in RViz

```
World
├── link_base (robot base)
├── link1-6 (arm joints)
├── link_eef (end-effector frame)
│   ├── uflite_gripper_link (gripper)
│   │   ├── uflite_finger1
│   │   ├── uflite_finger2
│   │   └── link_tcp (tool center point)
│   │       └── calibration_pin_link ⭐ (with CalibrationPin.stl mesh)
│   └── camera_color_optical_frame (if add_realsense_d435i:=true)
```

---

## Step-by-Step RViz Setup

### Method A: Using Display Panel

1. **Start RViz** (opens with lite6_moveit_realmove.launch.py)
2. **Left Panel → Displays**
3. **Click "Add" button at bottom**
4. **Choose "Mesh Resource"**
5. **Fill in properties:**
   - Mesh Resource: `package://xarm_description/meshes/CalibrationPin.stl`
   - Frame: `link_tcp`
   - Position Offset Z: `0.015` (15mm above TCP)

### Method B: Save RViz Config

Save your RViz configuration after adding the mesh:
```bash
File → Save Config As → lite6_with_pin.rviz
```

Use it next time:
```bash
rviz2 -d lite6_with_pin.rviz
```

---

## Integration with Hand-Eye Calibration

### Launch Full System with CalibrationPin Visible

**Terminal 1: Robot + Camera + Visualization**
```bash
ros2 launch tataro_packages tatro_runtime.launch.py \
  robot_ip:=192.168.1.150 \
  calib_name:=tataro_handeye_3A
```

**Terminal 2: Run Hand-Eye Calibration (Optional)**
```bash
ros2 launch easy_handeye2 calibrate.launch.py \
  name:=tataro_handeye_cal \
  calibration_type:=eye_in_hand \
  robot_base_frame:=link_base \
  robot_effector_frame:=link_eef \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=aruco_marker_frame
```

---

## File Locations

```
xarm_ros2/
├── xarm_description/
│   ├── meshes/
│   │   └── CalibrationPin.stl ✅
│   └── urdf/
│       └── gripper/
│           ├── calibration_pin.urdf.xacro ✅
│           └── calibrationpin_uflite_gripper_macro.xacro ✅
└── Claude_files/
    └── CalibrationPin_Visualization_Guide.md ✅
```

---

## Visualization Properties

| Property | Value |
|----------|-------|
| Mesh File | CalibrationPin.stl |
| File Size | 91.1 KB |
| Vertices | 934 |
| Faces | 1,864 |
| Dimensions | 38.1 × 88.1 × 20.0 mm |
| Attachment Point | link_tcp (Z offset: 15mm) |
| Color (RViz) | Light Blue RGBA(0.2, 0.6, 0.8, 1.0) |

---

## Troubleshooting

### Issue: Mesh not visible in RViz

**Solution 1:** Check frame is publishing
```bash
ros2 topic echo /tf | grep link_tcp
```

**Solution 2:** Verify mesh file exists
```bash
ls -la /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/xarm_description/meshes/CalibrationPin.stl
```

**Solution 3:** Check RViz fixed frame
- In RViz left panel: Set Fixed Frame to `link_base`

### Issue: Mesh in wrong position

**Solution:** Adjust Z offset in RViz display:
- Increase Z offset if mesh appears inside gripper
- Decrease Z offset if mesh appears too far

### Issue: Mesh appears too large/small

**Solution:** Modify scale in URDF macro:
```xml
<mesh filename="..." scale="0.001 0.001 0.001" />
<!--  0.001 = 0.1% (typical for mm→m conversion) -->
```

---

## Next Steps

1. ✅ **Visualize:** Add CalibrationPin mesh to RViz (Option 1 above)
2. ✅ **Test Movement:** Run robot and verify pin follows TCP
3. ✅ **Hand-Eye Calibration:** Use with tataro_touch_marker_centre script
4. ✅ **Detection:** Verify ArUco marker visible in camera feed with pin
5. 📊 **Monitor:** Watch marker reprojection during operation

---

**Created:** 2026-06-05  
**Status:** Ready for RViz ✅  
**Files Created:** 3/3 ✅
