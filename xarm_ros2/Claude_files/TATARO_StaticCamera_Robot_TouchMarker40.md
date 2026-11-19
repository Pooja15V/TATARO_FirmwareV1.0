# TATARO — Static Camera + Robot: Touch Marker 40

**Session goal:** Build a ROS2 pipeline that uses the static RealSense D455 camera to detect ArUco Marker 40 and command the xArm Lite6 calibrator pin to physically touch its centre.

---

## System Overview

| Component | Details |
|-----------|---------|
| Camera | Intel RealSense D455 (D435i-compatible), 1280×720, aligned depth |
| Robot | UFFactory xArm Lite6 |
| Tool | Calibrator pin (~85 mm TCP tool offset) |
| Markers | DICT_7X7_250: IDs 51 (bottom-left), 52 (bottom-right), 53 (top-left) define table frame; ID 40 is touch target |
| OS / ROS | Ubuntu 22.04 / ROS2 Humble |

---

## Files Created / Modified

### 1. `src/computer_vision/TATARO_CV_Pipeline.py` *(created)*
Central pipeline combining all CV modules.

**Key exported function:**
```python
marker40_pose_in_robot_frame(rvec_40, tvec_40, Table_corner_3D, rvec_marker_wrt_camera)
# Returns: (pos_robot [3 floats, cm], rvec_40_robot [3])
```

**Pipeline steps:**
1. `connect_camera()` → ROS2 RealSense subscriber (aligned depth required)
2. `detect_and_transform_table_using_aruco()` → markers 51/52/53 + GSO + warp
3. `detect_marker_7x7(color_frame, 40, depth_frame)` → marker 40 in camera frame
4. `marker40_pose_in_robot_frame()` → position + Rodrigues rotation in robot base frame
5. Axes drawn on camera feed via `cv2.drawFrameAxes`; dot + label on warped top-down view via `cv2.perspectiveTransform`

> **Constraint:** Do NOT modify the pose estimation logic in this file.

---

### 2. `src/tataro_packages/tataro_marker40_touch.py` *(created)*
Standalone script: stable CV detection → xArm SDK → hover + touch.

**Key configuration constants:**
```python
ROBOT_IP        = '192.168.1.150'
TOUCH_Z_MM      = 170.0   # SDK Z when pin tip is at marker surface (see Z section below)
HOVER_MM        = 50.0    # mm above TOUCH_Z_MM for approach waypoint
SPEED_APPROACH  = 80.0    # mm/s
SPEED_TOUCH     = 20.0    # mm/s
MVACC           = 100.0   # mm/s²
MIN_Z_MM        = 170.0   # safety floor (must equal TOUCH_Z_MM)
STABLE_N        = 10      # consecutive frames required for stable lock
STABLE_TOL_CM   = 0.5     # max per-axis std-dev across stable window (~5 mm)
DETECT_TIMEOUT  = 60.0    # seconds before giving up on detection
```

**Execution flow:**
1. `connect_camera()` → subscribe to RealSense (camera driver must already be running)
2. `_stable_pos40_robot(pipe)` → detection loop, returns mean of STABLE_N frames when spread < STABLE_TOL_CM
3. `_touch_marker(pos_cm)`:
   - XY from CV pipeline (pos_40_robot X/Y, reliable to ~2 mm)
   - Z fixed at `TOUCH_Z_MM` (CV Z **not used** — depth sensor Z noise is ±5–15 mm)
   - Hover move → descend to touch

**Path resolution (symlink-install compatible):**
```python
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_CV_DIR   = os.path.normpath(os.path.join(_THIS_DIR, '..', 'computer_vision'))
sys.path.insert(0, _CV_DIR)
```

---

### 3. `src/tataro_packages/launch/tataro_cv_table_detection.launch.py` *(rewritten)*
Launches all four processes in order.

**Path resolution (use `realpath` not `abspath` for symlink-install):**
```python
_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
_PKG_DIR    = os.path.normpath(os.path.join(_LAUNCH_DIR, '..'))
_CV_DIR     = os.path.normpath(os.path.join(_LAUNCH_DIR, '..', '..', 'computer_vision'))
```

**Launch sequence:**

| # | Process | Notes |
|---|---------|-------|
| 1 | `realsense2_camera rs_launch.py` | Identical config to `tatro_runtime.launch.py` — aligned depth, 1280×720×30 |
| 2 | `python3 TATARO_CV_Pipeline.py` | Live visualisation; prints `pos_40_robot` each frame |
| 3 | `lite6_moveit_realmove.launch.py` | `add_realsense_d435i=false` (camera already up), `add_gripper=true` |
| 4 | `python3 tataro_marker40_touch.py` | `TimerAction(period=10.0)` — waits for camera + robot to settle |

**RealSense launch args:**
```python
'publish_tf':           'true',
'enable_rgbd':          'true',
'enable_sync':          'true',
'use_system_time':      'true',
'align_depth.enable':   'true',
'enable_depth':         'true',
'pointcloud.enable':    'false',
'color_module.profile': '1280x720x30',
```

**Usage:**
```bash
ros2 launch tataro_packages tataro_cv_table_detection.launch.py
# or
ros2 launch tataro_packages tataro_cv_table_detection.launch.py robot_ip:=192.168.1.150
```

**Build before launch:**
```bash
cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
colcon build --packages-select tataro_packages --symlink-install
source install/setup.bash
```

---

### 4. `src/tataro_packages/CMakeLists.txt` *(modified)*
Added `tataro_marker40_touch.py` to `install(PROGRAMS ...)` section.

---

## Coordinate System & Z Calibration

### Frame chain
```
Camera frame
    ↓  detect_and_transform_table_using_aruco() + GSO
Table frame  (origin = marker 51 position, axes from 51→52, 51→53)
    ↓  R_robot_table, t_robot_table (TATARO_CV_Transformations.py)
Robot frame  (= xArm link_base frame for XY)
```

**Hardcoded transform (in `TATARO_CV_Transformations.py`):**
```python
R_robot_table = [[0,1,0],[-1,0,0],[0,0,1]]
t_robot_table = [42.5, 76.0, 0.5]  # cm
```

### Units
- CV pipeline output (`pos_40_robot`): **cm**
- xArm SDK (`set_position`): **mm** → multiply CV output × 10

### SDK Z vs TF Z relationship

**Discovered offset:**
```
SDK_Z  =  TF(link_base → link_tcp)_Z  +  85 mm
```

The 85 mm offset is the TCP tool offset (calibrator pin length) configured in the robot controller. The SDK reports the **pin tip** position; `tf2_echo link_base link_tcp` reports the **flange (link_tcp)** position.

| SDK Z commanded | TF Z observed | Outcome |
|-----------------|---------------|---------|
| 100 mm | 15 mm | **Collision** — pin hits table |
| 170 mm | 85 mm | Near table surface |
| 185 mm | 100 mm | Safe — pin just above table |

**Safe TF Z floor:** 100 mm (manually verified — below this the pin hits the table)  
**Corresponding SDK Z floor:** 100 + 85 = **185 mm**

**Current `TOUCH_Z_MM = 170.0`** — still being calibrated. Target value is **185 mm** once verified safe.

### Why CV Z is not used for touch
Depth sensor Z readings for a flat marker at ~70 cm range vary ±5–15 mm frame-to-frame. Using a **fixed, manually calibrated `TOUCH_Z_MM`** is safer and more repeatable. CV XY is reliable (~2 mm accuracy) and is used for the XY target.

---

## Stable Detection Logic

```
history = []
loop:
    get frame → detect table markers → detect marker 40
    compute pos_40_robot
    append to history (keep last STABLE_N)
    if len(history) == STABLE_N:
        spread = max per-axis std-dev
        if spread < STABLE_TOL_CM (0.5 cm = 5 mm):
            return mean(history)  ← stable lock
```

XY-only stability matters for the touch command. Z stability is less critical because `TOUCH_Z_MM` is fixed.

---

## Errors Encountered & Fixes

| Error | Root Cause | Fix |
|-------|-----------|-----|
| `RuntimeError: No frame received on '/camera/camera/color/image_raw' within 20s` | Camera driver not running before touch script | Launch sequence now starts RealSense first; `TimerAction(10s)` before touch script |
| `file was not found in the share directory` | `os.path.abspath` doesn't resolve symlinks | Changed to `os.path.realpath` in launch file |
| `RuntimeError: Target z = 5.0 mm is below the safety floor (50 mm)` | `MIN_Z_MM` too high, `z_touch` computed wrong | Fixed `z_touch = TOUCH_Z_MM` (fixed constant, not from CV) |
| `PIN_LENGTH_MM undefined` after rename | Variable renamed but references not all updated | Replaced all remaining references |
| `z_mm undefined` | Old variable name after rename to `z_touch` | Replaced `z_mm` with `z_touch` in `set_position` call |
| **Physical collision** — pin hit table | SDK Z=100 mm → TF Z=15 mm (below safe 100 mm) | `TOUCH_Z_MM` must be ≥ 185 mm |
| `STABLE_TOL_CM` too tight | Depth sensor Z noise makes spread > 0.2 cm | Relaxed from 0.2 → 0.5 cm |
| RealSense warnings for unsupported params | `robot_ip`, `use_system_time`, `color_module.profile` not recognised by rs_launch | Harmless — camera auto-selected correct 1280×720×30 profile |

---

## Current Status

- Pipeline confirmed working: camera → detect 51/52/53 → GSO → table crop → detect 40 → print `pos_40_robot`
- XY targeting confirmed accurate (~2 mm)
- Z calibration in progress: last tested `TOUCH_Z_MM = 150 mm` → collision; analysis says correct value is **185 mm**
- `TOUCH_Z_MM` and `MIN_Z_MM` in `tataro_marker40_touch.py` need to be set to **185.0** after next manual verification with `tf2_echo link_base link_tcp`

### Verification procedure after changing TOUCH_Z_MM

1. Set `TOUCH_Z_MM = 185.0` and `MIN_Z_MM = 185.0`
2. Launch: `ros2 launch tataro_packages tataro_cv_table_detection.launch.py`
3. In a separate terminal: `ros2 run tf2_ros tf2_echo link_base link_tcp`
4. When touch command executes, confirm TF Z ≈ 100 mm
5. If pin still collides, increase `TOUCH_Z_MM` by 10 mm increments; if too high (doesn't reach marker), decrease

---

## Key Source Files

| File | Role |
|------|------|
| `src/computer_vision/TATARO_CV_Pipeline.py` | Full CV pipeline + `marker40_pose_in_robot_frame()` |
| `src/computer_vision/TATARO_CV_Transformations.py` | `camera_to_robot()`, `R_robot_table`, `t_robot_table` |
| `src/computer_vision/TATARO_CV_TableCropping_with_ARUCO_7x7.py` | `detect_and_transform_table_using_aruco()` + GSO |
| `src/computer_vision/TATARO_CV_MarkerDetection_7x7.py` | `detect_marker_7x7()`, `camera_matrix`, `dist_coeffs` |
| `src/computer_vision/TATARO_CV_ROS2Camera.py` | `connect_camera()`, `get_frame()`, `stop_camera()` |
| `src/tataro_packages/tataro_marker40_touch.py` | Stable detection + xArm SDK touch sequence |
| `src/tataro_packages/launch/tataro_cv_table_detection.launch.py` | Launch all 4 processes |
| `src/tataro_packages/tataro_charuco_touch.py` | Reference for xArm SDK usage pattern |
