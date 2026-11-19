# TATARO CV Calibration Check — Session Summary

## 1. Primary Request and Intent

Starting from a previous session where 4 ROS2 CV files were created, the user requested:

1. Complete the pending `TATARO_CV_CharucoCalibrationWorkflow.py` — a 5-step monolithic Python workflow using `subprocess.Popen` and `MoveItPy`
2. Fix black windows in the workflow (table detection images not showing)
3. Redesign to proper ROS2 architecture with a launch file (`tatro_runtime_calibrationcheck.launch.py`) and a standalone node (`tataro_charuco_touch.py`) using no subprocess.Popen inside nodes
4. Fix a series of runtime errors (MoveItPy not installed, wrong charuco board parameters, wrong TF frame names)
5. Replace charuco board detection with a single ArUco 7×7 DICT_7X7_250 marker (ID 40) because the camera angle prevents flat board detection
6. Run the full launch and verify the system starts correctly

**CRITICAL CALIBRATION CONSTRAINT:** Always use 1280×720 calibration values. Never revert to old 640×480 placeholders:
```python
camera_matrix = np.array([
    [643.9336547851562, 0, 650.7435302734375],
    [0, 642.9497680664062, 363.3885192871094],
    [0, 0, 1]
], dtype=np.float64)
dist_coeffs = np.array([
    -0.05567498505115509, 0.06433625519275665,
    -0.00040179200004786253, 0.00031893057166598737,
    -0.02026429958641529
], dtype=np.float64)
```

---

## 2. Key Technical Concepts

- **XArmAPI SDK**: `from xarm.wrapper import XArmAPI` — direct SDK control used in `tataro_touch_marker_centre` and adopted for `tataro_charuco_touch.py`. Uses `arm.set_position(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, speed=50, mvacc=100, wait=True)` with `is_radian=False`
- **ROS2 launch architecture**: `tatro_runtime_calibrationcheck.launch.py` with `RegisterEventHandler(OnProcessExit → Shutdown())` to shut down entire stack when touch node exits
- **TF2**: `Buffer`, `TransformListener`, `lookup_transform` for `link_base → calibration_marker_frame` chain
- **ArUco detection**: `DICT_7X7_250`, marker ID 40, `cv2.aruco.ArucoDetector`, depth back-projection for 3D position
- **TF broadcaster**: `TransformBroadcaster` publishes camera → marker frame at 15 Hz
- **Rotation math**: `_quat_to_rot`, `_rot_to_rpy_deg`, TCP orientation: `z_tcp = -z_marker` (pin points toward marker)
- **Depth back-projection** (cm): `Z = z_depth_mm / 10.0`, `X = (u-cx)*Z/fx`, `Y = (v-cy)*Z/fy`
- **ROS2 node params**: All config as `declare_parameter` / `get_parameter` so launch file can override
- **MultiThreadedExecutor** in background thread for rclpy spin while `node.run()` blocks on main thread
- **MoveIt**: `move_group` running but NOT used for motion — XArmAPI used directly
- **Camera topics**: `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`
- **`add_realsense_d435i:=true`** in robot launch — publishes camera TF frames as part of robot URDF tree

---

## 3. Files Created / Modified

### `xarm_ros2/src/computer_vision/TATARO_CV_CharucoCalibrationWorkflow.py`
Monolithic 5-step workflow (superseded by the launch-file architecture but still present). Uses `subprocess.Popen`, `MoveItPy`, `_CalibrationNode`, `TF2`. Shutdown order: calibration node first, then `stop_camera()` (which calls `rclpy.shutdown()`), then kill subprocesses.

### `xarm_ros2/src/tataro_packages/tataro_charuco_touch.py`
**Final version** — uses XArmAPI not MoveIt.

Parameters: `robot_ip`, `robot_base_frame`, `charuco_frame`, `calibration_pin_m`, `speed_mm_s`, `mvacc_mm_s2`, `stable_frames`, `stable_tol_m`, `tf_wait_timeout`, `min_z_m`

Key flow in `node.run()`:
1. `_wait_for_frame(base_frame)` + `_wait_for_frame(charuco_frame)`
2. `_stable_charuco_tf()` — 10 consecutive frames within 3mm spread
3. `_compute_target()` — `t_tgt = charuco_t + pin_m * z_charuco`; `z_tcp = -z_charuco`
4. Connect XArmAPI, `arm.set_position(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, speed=50, mvacc=100, wait=True)`
5. Log final position and error

```python
def _compute_target(self, charuco_t, charuco_q):
    R_c = _quat_to_rot(charuco_q)
    z_c = R_c[:, 2]
    t_tgt = charuco_t + self.pin_m * z_c
    z_tcp = -z_c
    x_tcp = R_c[:, 0]
    y_tcp = np.cross(z_tcp, x_tcp); y_tcp /= np.linalg.norm(y_tcp)
    x_tcp = np.cross(y_tcp, z_tcp); x_tcp /= np.linalg.norm(x_tcp)
    R_tcp = np.column_stack([x_tcp, y_tcp, z_tcp])
    roll_deg, pitch_deg, yaw_deg = _rot_to_rpy_deg(R_tcp)
    return x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg  # positions in mm
```

### `xarm_ros2/src/tataro_packages/tataro_aruco7x7_tf_publisher.py`
New node — detects DICT_7X7_250 marker ID 40, broadcasts TF at 15 Hz.

Key parameters:
```python
self.declare_parameter('marker_id',      40)      # default 40 (not 51/52/53)
self.declare_parameter('marker_size_cm', 15.0)
self.declare_parameter('camera_frame',   'camera_color_optical_frame')
self.declare_parameter('marker_frame',   'calibration_marker_frame')
self.declare_parameter('depth_radius',   10)
```

Subscriptions:
- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`

Detection at 15 Hz via timer:
1. `detectMarkers()` with DICT_7X7_250
2. Depth back-projection: `Z=z_mm/10.0`, `X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`
3. `solvePnP` for rotation
4. `TransformBroadcaster`: `camera_frame → marker_frame`

Uses hardcoded 1280×720 calibration:
```python
_CAMERA_MATRIX = np.array([
    [643.9336547851562, 0, 650.7435302734375],
    [0, 642.9497680664062, 363.3885192871094],
    [0, 0, 1]], dtype=np.float64)
```

### `xarm_ros2/src/tataro_packages/launch/tatro_runtime_calibrationcheck.launch.py`
Current version — 4 components:
1. `realsense`: `tf_publish_rate=0.0`, `enable_sync=true`, `align_depth.enable=true`
2. `aruco_tf_node`: `tataro_aruco7x7_tf_publisher.py` — `marker_id=40`, `marker_frame=calibration_marker_frame`
3. `robot_launch`: `lite6_moveit_realmove.launch.py` with `add_realsense_d435i=true`
4. `touch_node`: `tataro_charuco_touch.py` with `charuco_frame=calibration_marker_frame`

Shutdown on touch node exit:
```python
shutdown_on_exit = RegisterEventHandler(OnProcessExit(target_action=touch_node, on_exit=[Shutdown()]))
```

### `xarm_ros2/src/aruco_ros/aruco_ros/launch/charuco_detector.launch.py`
Updated defaults to match the actual board:
- `square_length`: `0.040` → `0.030` (3.0 cm)
- `marker_length`: `0.030` → `0.024` (2.4 cm)

### `xarm_ros2/src/tataro_packages/CMakeLists.txt`
Updated to install new scripts:
```cmake
install(PROGRAMS
        tataro_touch_marker_centre
        tataro_calibration_diagnostic.py
        tataro_charuco_touch.py
        tataro_aruco7x7_tf_publisher.py
        DESTINATION lib/${PROJECT_NAME})
```

---

## 4. Bugs Fixed

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| Black CV windows | `cv2.waitKey(1)` never rendered frames | `cv2.waitKey(100)` loop until keypress |
| Wrong TF frame name | `CHARUCO_FRAME = "charuco"` | Changed to `"charuco_board_frame"` |
| Shutdown lifecycle crash | `stop_camera()` called `rclpy.shutdown()` before destroying calibration node | Destroy calibration node/executor FIRST, then stop camera |
| MoveItPy ImportError | `tataro_charuco_touch.py` imported `MoveItPy` which is not installed | Replaced with XArmAPI (same pattern as working `tataro_touch_marker_centre`) |
| charuco_detector exit code 1 | Wrong board params: `square=4.0cm, marker=3.0cm` but actual board is `square=3.0cm, marker=2.4cm` | Updated defaults in `charuco_detector.launch.py` |
| Board not detected flat | Camera angle too steep for flat board detection | Switched to single ArUco 7×7 marker (ID 40) |
| `rgb_camera.color_profile` param error | Wrong realsense param name | Removed — defaults to 1280×720×30 anyway |

---

## 5. Pending Issue — TF Tree Disconnection

### Symptom
The touch node gets stuck at:
```
Waiting for stable charuco pose (10 frames within 3mm)...
```
for ~34 seconds until Ctrl+C.

### Root Cause
`calibration_marker_frame` is published relative to `camera_color_optical_frame` (camera TF tree), but `link_base` is in the robot TF tree. **These two trees are not connected**, so `lookup_transform(link_base, calibration_marker_frame)` silently fails in a loop.

The `_wait_for_frame()` check uses `lookup_transform(frame, frame)` (self-to-self), which always succeeds as long as the frame exists *anywhere* in *any* TF tree — this is why "Frame is available" is logged even though the cross-tree lookup fails.

### Evidence
MoveIt warning in the launch output:
```
[move_group] Unable to transform object from frame 'calibration_marker_frame'
to planning frame 'world' ... because they are not part of the same tree.
```

### Required Fix
Add `easy_handeye2 handeye_publisher` to `tatro_runtime_calibrationcheck.launch.py`, exactly as `tatro_runtimeV1.0.launch.py` does:

```python
from launch_ros.actions import Node

handeye_publisher = Node(
    package='easy_handeye2',
    executable='handeye_publisher',
    name='handeye_publisher',
    parameters=[{
        'calibration_file': 'tataro_handeye_5A',
    }],
)
```

This publishes the calibrated `link_base → camera_color_optical_frame` static TF, bridging the two trees so `lookup_transform(link_base, calibration_marker_frame)` can succeed.

### Diagnosis Command
```bash
ros2 run tf2_tools view_frames
```
Generates `frames.pdf` showing the actual TF tree — should show whether `camera_color_optical_frame` is connected to `link_base`.

---

## 6. Architecture Overview

```
tatro_runtime_calibrationcheck.launch.py
│
├── realsense2_camera          → /camera/camera/color/image_raw
│                              → /camera/camera/aligned_depth_to_color/image_raw
│
├── tataro_aruco7x7_tf_publisher  ← subscribes to camera topics
│                              → TF: camera_color_optical_frame → calibration_marker_frame
│
├── lite6_moveit_realmove      → TF: world → link_base → ... → link_tcp
│   (add_realsense_d435i=true) → TF: link_base → ... → camera_color_optical_frame (URDF)
│
├── [MISSING] easy_handeye2    → TF static: link_base → camera_color_optical_frame
│   handeye_publisher              (the calibrated bridge between robot and camera trees)
│
└── tataro_charuco_touch       ← looks up TF: link_base → calibration_marker_frame
                               → arm.set_position() via XArmAPI SDK
                               → exits → triggers Shutdown() of entire launch
```

---

## 7. Launch Command

```bash
ros2 launch tataro_packages tatro_runtime_calibrationcheck.launch.py
```

Optional overrides:
```bash
ros2 launch tataro_packages tatro_runtime_calibrationcheck.launch.py \
    robot_ip:=192.168.1.150 \
    marker_id:=40 \
    marker_size_cm:=15.0 \
    calibration_pin_m:=0.10
```

---

## 8. Next Steps

1. **Add `handeye_publisher` to the calibrationcheck launch file** (see fix above)
2. Rebuild: `cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2 && colcon build --packages-select tataro_packages && source install/setup.bash`
3. Place ArUco marker ID 40 (DICT_7X7_250, 15 cm) flat in camera view
4. Relaunch and verify the touch sequence completes
