# Eye-in-hand calibration - UFACTORY Lite 6 + RealSense

Ported from a Doosan (`drfl_pybind`) version. Only `camera/robot_interface.py`
changed — everything else (CSV format, affine-fit math, camera pixel->3D
conversion) is robot-agnostic and was carried over unchanged.

## Setup

1. Install the xArm SDK (from the folder you already have):
   ```bash
   cd xArm-Python-SDK-master
   pip install .
   ```
   If you'd rather not install it, set `XARM_SDK_DIR` at the top of
   `camera/robot_interface.py` to point at that folder instead.

2. Install the other dependencies:
   ```bash
   pip install pyrealsense2 opencv-python numpy
   ```

3. Set your robot's IP address in `collect_calibration_points.py` and
   `test_calibration.py` (`ROBOT_IP = "192.168.1.xxx"`).

## Workflow

1. **`python collect_calibration_points.py`**
   Connects to the robot and puts it in manual/freedrive mode
   (`set_mode(2)` — Lite 6's equivalent of jogging with a teach pendant,
   since it has no physical pendant). Hand-guide the arm to a point (or
   jog it in UFACTORY Studio), click the matching pixel in the camera
   window, confirm/type the robot X/Y. Repeat for 10-20 spread-out points.
   Saves `calibration_points.csv`.

2. **`python compute_calibration.py`**
   Fits the affine map `Robot_XY = A @ Camera_XY + t` from the CSV,
   prints RMSE + per-point residuals, saves `calibration_data.pkl`.

3. **`python test_calibration.py`**
   Loads the pickle, click any pixel to see the predicted robot X/Y and a
   computed hover Z. Press `m` to actually send the robot there.

 

| Lite 6 (`xarm` SDK)                        |
|---------------------------------------------|
| `XArmAPI(ip)` + `motion_enable(True)`        |
| `set_mode(2)` — joint teaching / freedrive   |
| `set_mode(0)` — position control             |
| `get_position(is_radian=False)`              |
| `get_servo_angle(is_radian=False)`           |
| `set_position(..., wait=True)`               |
| `set_servo_angle(angle=joints, wait=True)`   |
| `move_gohome(wait=True)`                     |

Note: Lite 6 has no physical teach pendant. `enable_manual_mode()` in
`robot_interface.py` puts the arm in joint-teaching mode so you can hand-guide
it directly, which is the closest equivalent. You can also jog it visually
in UFACTORY Studio if you prefer a screen + buttons over hand-guiding.
