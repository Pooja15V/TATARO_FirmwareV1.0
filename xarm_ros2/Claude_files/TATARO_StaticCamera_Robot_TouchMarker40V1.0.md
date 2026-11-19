# TATARO — Static Camera + Robot: Touch Marker 40 (V1.0)

**Session goal:** Extend the base pipeline into a two-phase frozen-table architecture, add an Enter-triggered touch loop, and build a real-time continuous tracking script with optional yaw orientation alignment.

> Previous session: [TATARO_StaticCamera_Robot_TouchMarker40.md](TATARO_StaticCamera_Robot_TouchMarker40.md)

---

## System Overview

| Component | Details |
|-----------|---------|
| Camera | Intel RealSense D455, 1280×720, aligned depth |
| Robot | UFFactory xArm Lite6 |
| Tool | Calibrator pin (~85 mm TCP tool offset) |
| Markers | DICT_7X7_250: IDs 51 (bottom-left), 52 (bottom-right), 53 (top-left) = table frame; ID 40 = touch target |
| OS / ROS | Ubuntu 22.04 / ROS2 Humble |

---

## Architecture Change: Two-Phase Frozen-Table Pipeline

### Problem with V0 pipeline
In the original `TATARO_CV_Pipeline.py`, markers 51/52/53 were re-detected **every frame**. This caused frame-to-frame jitter in the table frame estimate (GSO re-solved each frame), making Marker 40 positions flicker even when the marker was stationary.

### V1.0 solution: Phase 1 locks the table, Phase 2 only tracks Marker 40

```
Phase 1 (once):
  Camera → detect 51/52/53 → GSO → store frozen:
    rvec_marker_wrt_camera, Table_corner_3D, warp matrix M

Phase 2 (loop):
  Camera → cv2.warpPerspective(frame, M, warped_size)  ← no 51/52/53 re-detection
         → detect Marker 40 → marker40_pose_in_robot_frame()
         → update pos_40_robot
```

The warp matrix `M` is computed once from the frozen corner points and reused every frame. Eliminating per-frame table re-detection removes jitter entirely.

---

## Files Created / Modified

### 1. `src/computer_vision/TATARO_CV_PipelineV1.0.py` *(created)*

Two-phase pipeline with live visualisation.

**`marker40_pose_in_robot_frame()` — identical to V0, do not modify:**
```python
def marker40_pose_in_robot_frame(rvec_40, tvec_40, Table_corner_3D, rvec_marker_wrt_camera):
    """Returns (position_robot [3, cm], rvec_robot [3])"""
    pos_robot = camera_to_robot(tvec_40.flatten(), Table_corner_3D, rvec_marker_wrt_camera)
    R_cam_table, _ = cv2.Rodrigues(rvec_marker_wrt_camera)
    R_40_cam,    _ = cv2.Rodrigues(rvec_40)
    R_robot_cam    = R_robot_table @ R_cam_table.T
    R_40_robot     = R_robot_cam @ R_40_cam
    rvec_40_robot, _ = cv2.Rodrigues(R_40_robot)
    euler = Rot.from_matrix(R_40_robot).as_euler('xyz', degrees=True)
    return pos_robot, rvec_40_robot.flatten()
```

**Phase 1 — `_phase1_init_table(pipe)`:**
- Loops until markers 51/52/53 are detected
- Stores: `rvec_marker_wrt_camera`, `Table_corner_3D`
- Computes frozen warp matrix:
  ```python
  src_pts = np.float32([P1_2D, P2_2D, P3_2D, P4_2D])
  dst_pts = np.float32([[0,0],[w_w,0],[w_w,h_w],[0,h_w]])
  M = cv2.getPerspectiveTransform(src_pts, dst_pts)
  ```
- Returns dict: `{rvec_marker_wrt_camera, Table_corner_3D, M, warped_size, pixels_per_cm_x, pixels_per_cm_y}`

**Phase 2 — `run_pipeline()` inner loop:**
```python
warped_table = cv2.warpPerspective(color_frame, M, warped_size)  # frozen M
rvec_40, tvec_40 = detect_marker_7x7(color_frame, 40, depth_frame)
if tvec_40 is not None:
    pos_40_robot, rvec_40_robot = marker40_pose_in_robot_frame(...)
```

**Import note:** filename contains `.` — not a valid Python module name. Touch scripts import from `TATARO_CV_Pipeline` (V0, no version suffix) since `marker40_pose_in_robot_frame` is identical in both.

---

### 2. `src/tataro_packages/tataro_marker40_touch_loop.py` *(created)*

Enter-triggered detect-and-touch loop with persistent robot connection.

**Configuration:**
```python
ROBOT_IP       = '192.168.1.150'
TOUCH_Z_MM     = 170.0   # SDK Z at marker surface
HOVER_MM       = 50.0    # mm above TOUCH_Z_MM
SPEED_APPROACH = 80.0    # mm/s — hover move
SPEED_TOUCH    = 20.0    # mm/s — final descent
MVACC          = 100.0   # mm/s²
MIN_Z_MM       = 170.0   # safety floor (must equal TOUCH_Z_MM)
STABLE_N       = 10
STABLE_TOL_CM  = 0.5
INIT_TIMEOUT   = 60.0
```

**Threading pattern:**
```python
shared = {'lock': threading.Lock(), 'stable_pos': None}
det_thread = threading.Thread(target=_detect_loop, args=(...), daemon=True)
det_thread.start()
# main thread:
input()  # wait for Enter
with shared['lock']:
    pos = shared['stable_pos']
_touch_marker(arm, pos)
```

**`_touch_marker(arm, pos_cm)`:**
- `x_mm = float(pos_cm[0]) * 10.0` — convert cm → mm
- `y_mm = float(pos_cm[1]) * 10.0`
- `z_hover = TOUCH_Z_MM + HOVER_MM`  ← **critical: not just TOUCH_Z_MM**
- Calls `arm.clean_error()` + `arm.motion_enable()` before each touch (recovery)
- `wait=True` on both hover and touch moves

**Bug in original `tataro_marker40_touch.py` (fixed here):**
```python
# WRONG (original):
z_hover = TOUCH_Z_MM
# CORRECT:
z_hover = TOUCH_Z_MM + HOVER_MM
```

---

### 3. `src/tataro_packages/tataro_marker40_touch_loopV1.0.py` *(created)*

Real-time continuous tracking — no Enter needed; robot follows marker as it moves.

**Configuration (current working values):**
```python
ROBOT_IP             = '192.168.1.150'
TOUCH_Z_MM           = 200.0    # SDK Z — verified safe (TF Z ≈ 115 mm > 100 mm floor)
HOVER_MM             = 50.0
SPEED_APPROACH       = 500.0    # mm/s — initial hover move
SPEED_TOUCH          = 20.0     # mm/s — initial descent
SPEED_TRACK          = 1000.0   # mm/s — continuous tracking moves
MVACC_APPROACH       = 1000.0   # mm/s²
MVACC_TRACK          = 5000.0   # mm/s² — snappy response
MIN_Z_MM             = 200.0    # must equal TOUCH_Z_MM
DEADBAND_CM          = 0.3      # 3 mm minimum XY displacement before sending move
ORIENT_DEADBAND_DEG  = 3.0      # degrees — ignore yaw residuals smaller than this
STABLE_N             = 10
STABLE_TOL_CM        = 0.5
INIT_TIMEOUT         = 60.0
```

**Speed and acceleration physics:**

For a trapezoidal velocity profile:
- Crossover distance: `d_cross = SPEED² / MVACC`
- If `d > d_cross` (long move): `t = d/SPEED + SPEED/MVACC`
- If `d < d_cross` (short move): `t = 2 × √(d / MVACC)`

At `SPEED_TRACK=1000 mm/s`, `MVACC_TRACK=5000 mm/s²`:
- `d_cross = 1000²/5000 = 200 mm` — moves shorter than 200 mm use the short-move formula
- 10 mm move: `t = 2×√(10/5000) ≈ 90 ms`

xArm Lite6 hardware max Cartesian speed: ~1000 mm/s (SDK clamps higher values).

**Update rate of `pos_40_robot`:**
- Camera: 30 fps → ~33 ms per frame
- Processing overhead: ~2–5 ms
- Effective update rate: **~25–30 Hz**

---

### 4. Functions in `tataro_marker40_touch_loopV1.0.py`

#### `_init_table_frame(pipe)` — Phase 1
Blocks until markers 51/52/53 detected. Returns `{rvec_marker_wrt_camera, Table_corner_3D}`. Raises `RuntimeError` on timeout.

#### `_detect_loop(pipe, frozen, shared, stop)` — position only (original)
```python
shared = {
    'lock':       threading.Lock(),
    'latest_pos': None,   # updated every detected frame
    'stable_pos': None,   # updated only when STABLE_N frames agree within STABLE_TOL_CM
}
```
Tags each console line `[STABLE]` or `[CV]`. Do not modify.

#### `_detect_loop_poseANDorientation(pipe, frozen, shared, stop)` — NEW
Identical to `_detect_loop` but also writes `shared['latest_rvec']`:
```python
pos, rvec = marker40_pose_in_robot_frame(...)
with shared['lock']:
    shared['latest_pos']  = pos.copy()
    shared['latest_rvec'] = rvec.copy()   # Rodrigues rvec_40_robot, shape (3,)
    shared['stable_pos']  = ...
```
Use this thread when calling `_tracking_loop_poseANDorientation`.

#### `_tracking_loop(arm, shared, stop, roll, pitch, yaw)` — XY tracking only (original)
- Reads `latest_pos` under lock
- Sends move only if XY changed by more than `DEADBAND_CM`
- `wait=True` prevents command queue buildup
- Do not modify.

#### `_compute_yaw_residual(rvec_40_robot)` — NEW helper
```python
def _compute_yaw_residual(rvec_40_robot: np.ndarray) -> float:
    R, _ = cv2.Rodrigues(rvec_40_robot.reshape(3, 1))
    theta_deg = np.degrees(np.arctan2(R[1, 0], R[0, 0]))  # in-plane X-axis angle
    theta_mod = theta_deg % 90.0                            # reduce to [0°, 90°)
    if theta_mod > 45.0:
        theta_mod -= 90.0                                   # normalize to [-45°, 45°)
    return theta_mod
```

**Why 90° periodicity:**
ArUco marker 40 has 4-fold rotational symmetry — rotating it 90° produces an identical appearance. The robot cannot distinguish 0°, 90°, 180°, 270°. So only the residual after removing 90° multiples is meaningful for alignment.

#### `_tracking_loop_poseANDorientation(arm, shared, stop, roll, pitch, initial_yaw)` — NEW
Extends `_tracking_loop` with yaw correction:
```python
# Read latest rvec under lock
with shared['lock']:
    pos   = shared['latest_pos']
    rvec  = shared['latest_rvec']

# Compute desired yaw
theta_residual = _compute_yaw_residual(rvec)   # [-45°, 45°)
desired_yaw = initial_yaw + theta_residual

# Move if XY changed OR yaw changed beyond deadband
xy_changed  = np.linalg.norm(pos[:2] - last_pos[:2]) > DEADBAND_CM
yaw_changed = abs(theta_residual) > ORIENT_DEADBAND_DEG

if xy_changed or yaw_changed:
    arm.set_position(x_mm, y_mm, TOUCH_Z_MM, roll, pitch, desired_yaw,
                     speed=SPEED_TRACK, mvacc=MVACC_TRACK, wait=True)
```

#### Summary table

| Function | Role | Status |
|----------|------|--------|
| `_detect_loop` | Position only | Original — do not modify |
| `_detect_loop_poseANDorientation` | Position + `latest_rvec` | New |
| `_compute_yaw_residual(rvec)` | Extracts yaw residual ∈ [-45°, 45°) | New helper |
| `_tracking_loop` | XY tracking, fixed yaw | Original — do not modify |
| `_tracking_loop_poseANDorientation` | XY + yaw alignment | New |

**`shared` dict keys:**
```python
shared = {
    'lock':       threading.Lock(),
    'latest_pos': None,   # np.ndarray [3], cm
    'latest_rvec': None,  # np.ndarray [3], Rodrigues rvec_40_robot
    'stable_pos': None,   # np.ndarray [3], cm — smoothed
}
```

---

### 5. `src/tataro_packages/launch/tataro_cv_detect_touch_marker_loop.launch.py` *(created, then modified)*

**Current launch sequence (3 processes only):**

| # | Process |
|---|---------|
| 1 | RealSense D455 — aligned depth, 1280×720×30 |
| 2 | `TATARO_CV_PipelineV1.0.py` — Phase 1 lock + Phase 2 visualisation |
| 3 | `lite6_moveit_realmove.launch.py` — `add_realsense_d435i=false` |

**Touch script is NOT in the launch file.** ROS2 `ExecuteProcess` does not route stdin to subprocesses — `input()` blocks indefinitely and Enter can never be received.

**Two-terminal workflow:**
```bash
# Terminal 1:
ros2 launch tataro_packages tataro_cv_detect_touch_marker_loop.launch.py

# Terminal 2 (after launch is stable):
cd ~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/src/tataro_packages
python3 tataro_marker40_touch_loop.py       # Enter-triggered version
# or
python3 tataro_marker40_touch_loopV1.0.py  # real-time tracking version
```

---

### 6. `src/tataro_packages/CMakeLists.txt` *(modified)*

```cmake
install(PROGRAMS
        tataro_touch_marker_centre
        tataro_calibration_diagnostic.py
        tataro_charuco_touch.py
        tataro_aruco7x7_tf_publisher.py
        tataro_table_crop_node.py
        tataro_marker40_touch.py
        tataro_marker40_touch_loop.py
        tataro_marker40_touch_loopV1.0.py
        DESTINATION lib/${PROJECT_NAME})
```

---

## Coordinate System & Z Calibration

*(Unchanged from V0 — reproduced for completeness)*

### SDK Z vs TF Z

```
SDK_Z  =  TF(link_base → link_tcp)_Z  +  85 mm
```

The 85 mm offset is the calibrator pin TCP tool offset. SDK reports pin tip; `tf2_echo` reports flange.

| SDK Z | TF Z | Status |
|-------|------|--------|
| 170 mm | 85 mm | Too low — risk of collision |
| 185 mm | 100 mm | Safe floor |
| 200 mm | 115 mm | **Current working value** |

**Verification:**
```bash
ros2 run tf2_ros tf2_echo link_base link_tcp
# During touch: confirm TF Z ≈ 115 mm (200 - 85 = 115, safely above 100 mm floor)
```

---

## Errors Encountered & Fixes

| Error | Root Cause | Fix |
|-------|-----------|-----|
| Enter key not working in launch | `ros2 launch` does not route stdin to `ExecuteProcess` subprocesses | Removed touch script from launch; run in separate terminal |
| Physical collision at `TOUCH_Z_MM=170` | SDK Z=170 → TF Z=85 mm (below 100 mm safe floor) | Raised to `TOUCH_Z_MM=200` (TF Z=115 mm) |
| `z_hover` bug: pin approached at wrong height | `z_hover = TOUCH_Z_MM` instead of `z_hover = TOUCH_Z_MM + HOVER_MM` | Fixed in all new scripts |
| Orientation edit modified existing `_detect_loop` | Interrupted; user requested separate new functions | Reverted; added `_detect_loop_poseANDorientation` and `_tracking_loop_poseANDorientation` as completely separate new functions |

---

## How to Switch to Orientation Tracking

`main()` currently calls the position-only functions. To enable yaw alignment, change two lines:

```python
# CURRENT (position only):
det_thread = threading.Thread(target=_detect_loop,
                              args=(pipe, frozen, shared, stop_event), daemon=True)
...
_tracking_loop(arm, shared, stop_event, roll, pitch, yaw)

# SWITCH TO (position + yaw alignment):
det_thread = threading.Thread(target=_detect_loop_poseANDorientation,
                              args=(pipe, frozen, shared, stop_event), daemon=True)
...
_tracking_loop_poseANDorientation(arm, shared, stop_event, roll, pitch, yaw)
```

`shared['latest_rvec']` is already initialised to `None` in `main()`.

---

## Key Source Files

| File | Role |
|------|------|
| `src/computer_vision/TATARO_CV_Pipeline.py` | V0 pipeline — `marker40_pose_in_robot_frame()` (do not modify) |
| `src/computer_vision/TATARO_CV_PipelineV1.0.py` | V1.0 two-phase frozen-table pipeline with visualisation |
| `src/computer_vision/TATARO_CV_Transformations.py` | `camera_to_robot()`, `R_robot_table`, `t_robot_table` |
| `src/computer_vision/TATARO_CV_TableCropping_with_ARUCO_7x7.py` | `detect_and_transform_table_using_aruco()` + GSO |
| `src/computer_vision/TATARO_CV_MarkerDetection_7x7.py` | `detect_marker_7x7()`, `camera_matrix`, `dist_coeffs` |
| `src/computer_vision/TATARO_CV_ROS2Camera.py` | `connect_camera()`, `get_frame()`, `stop_camera()` |
| `src/tataro_packages/tataro_marker40_touch_loop.py` | Phase 1 lock + Enter-triggered touch loop |
| `src/tataro_packages/tataro_marker40_touch_loopV1.0.py` | Real-time continuous tracking + optional yaw alignment |
| `src/tataro_packages/launch/tataro_cv_detect_touch_marker_loop.launch.py` | Launch infrastructure (camera + CV pipeline + robot driver) |
