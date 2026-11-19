# TATARO — Lite6 joint‑5 offset and eye‑in‑hand calibration

Record of finding and correcting a joint‑zero error on the TATARO Lite6, then calibrating the wrist‑mounted D435i (eye‑in‑hand) with the Cyberwave SDK.

_Last updated: 2026-10-16_

---

## 1. Summary

| Item | Result |
|---|---|
| Root cause of failed hand‑eye calibration | **Joint 5 zero offset: true angle = reported angle + 5.2° (±0.1°)**. Joints 2/3/4 ≈ 0 |
| Correction | `lite6_kinematics_tataro.yaml` (xarm_ros2, `kinematics_suffix:=tataro`) |
| Hand‑eye result (corrected kinematics) | **residual 1.18 mm**, leave‑one‑out stability 0.19 mm, solvers agree within 0.3 mm |
| Same data, stock kinematics | residual 15.6 mm (solvers disagree by up to 321 mm) |
| Camera mount (link_eef → camera_color_optical_frame) | **t = (66.8, −34.3, 23.3) mm, rpy = (−0.63, −0.54, 90.53)°** |
| Status | Correction file saved, **not yet active** (rebuild + launch argument needed, §6). Mount not yet applied to URDF/twin |

---

## 2. Symptom

The first SDK hand‑eye run (`runs/20261014_102451`, 20 captures) looked "GOOD" on stability (2 mm) but was inconsistent:

- residual 13–14 mm (max 32 mm), solvers disagreed by 25–34 mm
- every image was clean (16 corners, ~0.2 px), so the error was in the arm poses

---

## 3. Diagnosis

| Check | Result |
|---|---|
| URDF FK vs controller `get_forward_kinematics()` | agree to 1.7 mm / 0.15° → model and controller share the same error |
| Commanded vs measured joints (`is_real=True`) | 0.03° |
| Stale camera frames | ruled out — frames exposed after the arm settled (timestamps) |
| Camera mount / board | user confirmed rigid / taped; camera axis within 0.3° of joint‑6 axis |
| D435i accelerometer at (0,0,0,0,0,0)° | flange **4.0° off vertical** (pitch), repeatable to 0.05°, rotates with joint 1 → in the arm, not the table (table tilt 0.9°) |
| UFactory Studio / SDK | no user zero‑calibration function; `set_servo_zero` is debug‑only ("do not use") — **not used** |

UFactory support could not be reached, so the offset was identified and corrected in software.

---

## 4. Joint‑offset identification

Automated run, 16 poses with joint 4 between −67° and +80° (needed to separate joint 5 from joints 2/3), camera (ChArUco) + IMU recorded at each pose, joint offsets fitted together with camera mount, board pose, IMU rotation and table tilt.

| Model | Reprojection | j2 | j3 | j4 | j5 |
|---|---|---|---|---|---|
| No correction | 60 px | 0 | 0 | 0 | 0 |
| **j5 only** | **4.6 px** | 0 | 0 | 0 | **+5.18°** |
| j2–j5 free | 4.4 px | −1.3° | −0.7° | +0.15° | +5.21° |

Joint 5 is the only significant offset (±0.08° leave‑one‑out); freeing j2–j4 gains only ~0.1 px.

**Independent confirmation: wrist‑flip test (2026‑10‑16, ruler only, no sensors).** The same tool‑tip point was commanded with the tool pointing down in two wrist configurations, A = (0, 36.4, 50.9, 0, 14.5, 0)° and B = (0, 36.4, 50.9, −180, −14.5, 0)°. With correct zeros the tips would coincide. They were **~27 mm apart**, B further from the base, which gives asin(27 / 294) = **5.3°**. That matches the fitted 5.2°.

---

## 5. Eye‑in‑hand calibration (dome)

- Board centre ~0.24 m in front of the base (must be ≤ ~0.30 m — beyond that the dome is out of the Lite6's reach).
- 14 poses on a virtual dome, radius 0.30 m: **top 5, left 6, right 3**, camera aimed at the board centre.
- Solved with `cyberwave.calibration.HandEyeSession` (Park), device intrinsics (fx 908.11, fy 907.88, cx 642.38, cy 369.84).

| | Stock kinematics | **Corrected (j5 +5.2°)** |
|---|---|---|
| Residual | 15.64 mm (max 29.5) | **1.18 mm (max 2.8)** |
| Rotation residual | 2.70° | **0.43°** |
| Leave‑one‑out stability | 1.01 mm | **0.19 mm** |
| Park / Horaud / Tsai agreement | 2 mm (Andreff 321 mm) | **≤ 0.3 mm** (Daniilidis 18.6 mm, Andreff failed — known fragile) |
| vs CAD mount | 13.7 mm, 3.2° | **1.9 mm, 1.0°** |

Cross‑check: the joint‑offset fit (independent data) gave the mount as (65.4, −33.4, 22.7) mm — within 1.7 mm.

The ~1–2 mm that remains is most likely the 3D‑printed camera stand flexing with pose.

---

## 6. Using the correction

The file: `xarm_ros2/xarm_description/config/kinematics/user/lite6_kinematics_tataro.yaml` — stock Lite6 kinematics with joint‑5 origin pitch `−0.090757` rad (= +5.2° about joint 5's own axis).

1. **Rebuild once** (the install space has no `user/` folder yet):
   ```bash
   colcon build --symlink-install --packages-select xarm_description
   ```
2. **Pass the suffix**, e.g.:
   ```bash
   ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.150 add_gripper:=true kinematics_suffix:=tataro
   ```
   or add `'kinematics_suffix': 'tataro'` to `launch_arguments` in `tatro_runtime.launch.py`, `tataro_cv_table_detection.launch.py`, `tataro_therapy_session.launch.py`.

Without the suffix, the stock `default/lite6_default_kinematics.yaml` is used (no correction). With the suffix but no rebuild, xacro fails with "file not found".

**Covered by the correction:** MoveIt planning/IK, RViz/TF, joint_states → poses, vision via TF.
**Not covered:** the controller's own Cartesian commands (`set_position`, `move_line`), UFactory Studio Cartesian jogging, controller‑side self‑collision/boundaries.

> The hand‑eye mount in §5 is valid **only with the corrected kinematics**. Using it with stock kinematics reintroduces ~15 mm errors.

---

## 7. Files

| Path | Content |
|---|---|
| `~/lite6_cyberwave_check/handeye/tataro_handeye.py` | Manual capture/solve tool (live residual, timestamp‑checked frames, measured joints) |
| `~/lite6_cyberwave_check/handeye/offset_id/plan_poses.py` | Random safe pose planner (joint‑offset ID) |
| `…/offset_id/plan_dome.py` | Dome pose planner (top/left/right, IK on corrected model) |
| `…/offset_id/run_poses.py` | Executes a pose plan, records camera + IMU (15°/s, aborts on any controller error) |
| `…/offset_id/fit_offsets.py`, `compare_models.py` | Joint‑offset fitting / model comparison |
| `…/offset_id/solve_dome_handeye.py` | Hand‑eye solve, stock vs corrected |
| `…/offset_id/run_20261015_171416`, `_171659`, `_172129` | Joint‑offset ID data (16 poses) |
| `…/offset_id/run_20261015_192318/` | Dome data + `handeye_result.json` |
| `…/handeye/imu_data/20261015_zero_pose.json` | IMU reading at the zero pose |
| venv | `~/.venvs/tataro-handeye` (cyberwave[calibration] 0.7.3, OpenCV GUI, pyrealsense2, xArm SDK, scipy) |

---

## 8. Lessons / gotchas

- **Free the D435i:** Edge Core recreates the camera container within ~45 s. Use `sudo systemctl stop cyberwave-edge-core` (also stops the arm driver) and restart with `sudo systemctl start cyberwave-edge-core`. Close `realsense-viewer` too.
- **Do not use `camera.calibration.set()` on the Lite6 twin** — the D435i is a sensor inside the arm twin, so it would move the arm twin itself.
- **Controller TCP offset is z = 83.6 mm** — controller poses are the gripper tip, not the flange.
- **`set_servo_angle(wait=True)` can return while the arm is still moving** — wait on measured joints instead.
- The arm creeps 0.01–0.02° for seconds after stopping; allow ~3 s settle and a 0.05° capture tolerance.
- Joint 6 capped at ±125° for the camera cable; keep the board ≤ 0.30 m from the base for a full dome.
- Cosmetic: the stock solver set includes Daniilidis/Andreff, which are unreliable on this data — judge by Park/Horaud/Tsai.

---

## 9. Next steps

1. Rebuild `xarm_description` and verify the xacro‑generated model matches the corrected FK at the dome poses (expected < 0.01 mm).
2. Add `kinematics_suffix:=tataro` to the TATARO launch files.
3. Save the mount as `xarm_ros2/Calibration/tataro_handeye_dome.calib` (easy_handeye2 format) and use `calib_name:=tataro_handeye_dome`.
4. Apply joint‑5 correction + camera mount to `~/lite6_cyberwave_check/lite6.urdf`, then push to the Cyberwave asset/twin.
5. Re‑measure `R_robot_table` / `t_robot_table` in `TATARO_CV_Transformations.py` if they were measured by touching markers with the arm (they contain the old error).
6. Optional: stiffer camera stand to remove the remaining ~1–2 mm; contact UFactory (reseller / GitHub issues) for an official joint‑zero recalibration.
