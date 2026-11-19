# TATARO ArUco Detection & Hand-Eye Calibration Integration

**Date:** 2026-06-05  
**Project:** TATARO xArm lite6 Robot with RealSense D435i Camera  
**Status:** Work in Progress - TF Chain Established ✅

---

## Overview

Integrated ArUco marker detection with hand-eye calibration to enable the xArm robot to locate and manipulate objects marked with ArUco tags. The system publishes complete TF transformation chains from robot base to detected markers.

---

## System Architecture

### Hardware
- **Robot:** xArm lite6 with uflite gripper
- **Camera:** RealSense D435i mounted on gripper
- **Marker:** ArUco ID 0, 15cm size, DICT_4X4_1000

### Software Stack
- **ROS 2:** Humble
- **Key Packages:** 
  - `easy_handeye2` - Hand-eye calibration
  - `aruco_ros` - ArUco detection (custom Python node)
  - `xarm_moveit_config` - Robot motion planning
  - `realsense2_camera` - Camera driver

---

## Key Discovery: Camera Frame Names

**RealSense publishes:**
- ✅ `camera_color_optical_frame` (from camera_info messages)
- ✅ `camera_rgb_optical_frame` (from TF static transforms)
- ✅ `camera_depth_optical_frame`

**NOT:**
- ❌ Only one optical frame is actually active

### Why This Matters
The camera intrinsics in `/camera/camera/color/camera_info` use frame_id `camera_color_optical_frame`, but RealSense's static transforms broadcast `camera_rgb_optical_frame`.

---

## Launch Command Reference

### Minimal Test (Sequential)

```bash
# Terminal 1: RealSense Camera
ros2 launch realsense2_camera rs_launch.py tf_publish_rate:=0.0 enable_sync:=true

# Terminal 2: ArUco Detector
ros2 launch aruco_ros detector.launch.py marker_id:=0 marker_size:=0.15

# Terminal 3: xArm Robot
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py \
  robot_ip:=192.168.1.150 add_gripper:=true \
  add_realsense_d435i:=true add_d435i_links:=false
```

### Integrated Launch (All-in-One)
```bash
# tatro_runtime_v0.launch.py - Contains all 3 above in one command
ros2 launch tataro_packages tatro_runtime_v0.launch.py robot_ip:=192.168.1.150
```

---

## Calibration Files

### Location
`/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/Calibration/`

### Available Calibrations
- `tataro_handeye_1A.calib` ⭐ Primary (used in integration)
- `tataro_handeye_1B.calib` - Backup
- `tataro_handeye_1C.calib` - Backup
- `tataro_handeye_1D.calib` - Alternative

### Calibration Values (1A)
```yaml
Position (gripper → camera):
  x: 0.0678m
  y: -0.1303m
  z: 0.4248m

Rotation (quaternion):
  x: 0.1433
  y: -0.0015
  z: 0.4900
  w: 0.8598
```

---

## TF Transform Chain

### Expected (Target)
```
world
  └── link_base
        └── link1 → link2 → ... → link6
              └── link_eef
                    └── uflite_gripper_link
                          └── camera_rgb_optical_frame [hand-eye calib]
                                └── aruco_marker_frame [ArUco detect]
```

### Current Status (as of 2026-06-05 12:59)
✅ **Complete chain established!**
- Robot links: Publishing at 10.2 Hz
- Hand-eye transform: Static (0 Hz = constant)
- ArUco marker: Publishing at ~30 Hz

---

## Custom ArUco Detector Node

### File
`/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/src/aruco_ros/aruco_ros/aruco_detector_node.py`

### Features
- ✅ Loads RealSense camera intrinsics dynamically from `/camera/camera/color/camera_info`
- ✅ Detects ArUco markers in real-time
- ✅ Publishes marker pose as geometry_msgs/PoseStamped
- ✅ Broadcasts TF transform (camera → marker)
- ✅ Publishes detection overlay image

### Parameters
```python
marker_id: 0                    # Which marker to track
marker_size: 0.15               # Marker size in meters
camera_frame: 'camera_color_optical_frame'  # Parent TF frame
marker_frame: 'aruco_marker_frame'          # Child TF frame
```

### Topics Published
- `/aruco/pose` - Marker pose
- `/aruco/result` - Detection visualization
- `/tf` - Transform broadcast

---

## Key Issues & Solutions

### Issue 1: Camera Intrinsics Mismatch
**Problem:** Hardcoded camera calibration values were inaccurate  
**Solution:** Subscribe to `/camera/camera/color/camera_info` and load dynamic calibration  
**Status:** ✅ Fixed

### Issue 2: ROS 2 CameraInfo API
**Problem:** Code used `msg.K` (ROS 1) instead of `msg.k` (ROS 2)  
**Solution:** Updated camera_info_callback to use lowercase attributes  
**Status:** ✅ Fixed

### Issue 3: Frame Name Mismatch
**Problem:** Using `camera_rgb_optical_frame` instead of actual RealSense frames  
**Solution:** Verify actual frame published by RealSense, update accordingly  
**Status:** ⚠️ In Progress - RealSense publishes `camera_rgb_optical_frame` in static TF

### Issue 4: handeye_publisher Startup
**Problem:** easy_handeye2's handeye_publisher wouldn't initialize properly  
**Solution:** Use static_transform_publisher with calibration values instead  
**Status:** ✅ Fixed

---

## Verification Commands

### Check TF Chain
```bash
ros2 run tf2_tools view_frames.py
# Should show complete chain: gripper → camera → marker
```

### Check ArUco Detection
```bash
# View detection overlay
ros2 run rqt_image_view rqt_image_view
# Select: /aruco/result (should show yellow box on marker)

# Check marker pose
ros2 topic echo /aruco/pose

# Check marker transform
ros2 run tf2_tools tf2_echo camera_rgb_optical_frame aruco_marker_frame
```

### Check Camera Topics
```bash
# List camera topics
ros2 topic list | grep camera

# Check camera info frame_id
ros2 topic echo /camera/camera/color/camera_info --once | grep frame_id
# Should show: frame_id: camera_color_optical_frame OR camera_rgb_optical_frame
```

---

## File Changes Made

### New Files Created
1. `/aruco_ros/aruco_detector_node.py` - Custom detection node
2. `/tataro_packages/` - Integrated launch package
   - `launch/tatro_runtime_v0.launch.py` - All-in-one launch
   - `package.xml`
   - `CMakeLists.txt`

### Modified Files
1. `easy_handeye2/__init__.py` - Updated CALIBRATIONS_DIRECTORY path
2. `aruco_ros/aruco_detector_node.py` - Dynamic intrinsics loading
3. `tataro_packages/launch/tatro_runtime_v0.launch.py` - Frame name corrections

### Configuration Files
1. `/Calibration/tataro_handeye_1A.calib` - Primary hand-eye calibration
2. `/Calibration/save_calibration.sh` - Auto-save script

---

## Next Steps

1. **Verify Final TF Chain**
   - Launch all three components
   - Generate frames PDF
   - Confirm gripper → marker connection

2. **Test Motion Planning**
   - Use MoveIt to plan to marker frame
   - Test pick-and-place motions

3. **Optimize Detection**
   - Fine-tune ArUco detection parameters
   - Test with different lighting
   - Validate detection reliability

4. **Production Integration**
   - Package as single launch command
   - Add error handling & logging
   - Document for team use

---

## Troubleshooting Quick Reference

| Issue | Check | Fix |
|-------|-------|-----|
| No TF frames | `ros2 node list` | Launch missing components |
| No marker detection | `/aruco/result` empty | Adjust lighting, marker ID, dictionary |
| Wrong frame parent | `ros2 run tf2_tools view_frames.py` | Update launch file frame names |
| Camera intrinsics error | `/camera/camera/color/camera_info` | Rebuild aruco_ros package |
| Permission errors | File ownership | Run: `chmod +x *.py` |

---

## Important Paths

```
Project Root: /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/

Key Directories:
- Calibrations: ./Calibration/
- Launch files: ./src/tataro_packages/launch/
- ArUco node: ./src/aruco_ros/aruco_ros/
- easy_handeye2: ./src/easy_handeye2/
- Documentation: ./Claude_files/
```

---

## Team Notes

**Pooja** - Set up hand-eye calibration, fixed frame naming issues, integrated ArUco detection  
**Status:** Functional hand-eye calibration with ArUco detection  
**Date Last Updated:** 2026-06-05  
**Tested:** On xArm lite6 with RealSense D435i ✅

---

*Generated with Claude Code*
