# TATARO on Cyberwave — Lite6 + Gripper Lite + RealSense D435i

Status record for bringing the TATARO rehabilitation robot (UFactory Lite 6, Gripper Lite, wrist‑mounted Intel RealSense D435i) onto Cyberwave, following <https://docs.cyberwave.com/robots/add-your-robot>.

_Last updated: 2026-10-13_

---

## 1. Status

| Level | Meaning | Status |
|---|---|---|
| **L0** Model | Twin renders and moves in the free Playground | ✅ Passed 2026-10-09 |
| **L1** Sim | Cloud MuJoCo holds a commanded pose under gravity | ✅ Passed via dashboard (0.17–0.34° error, zero drift). ⚠️ SDK `joints.set` is **not** consumed by cloud MuJoCo |
| **L2** Telemetry | Live twin mirrors the real robot | ✅ Within 0.05° when settled, ~0.4–0.5 s lag through the cloud |
| **L3** Control | Teleop with a safe state on network loss | ✅ Keyboard + SDK control and gripper working. ⏳ Network‑cut safety evidence not yet recorded |
| **L4** Learn | Record → train → deploy a policy | Not started |

Live camera: ✅ D435i colour 1280×720 @ 30 fps and depth 1280×720 (sent at 5 fps).

---

## 2. Key identifiers

| Item | Value |
|---|---|
| Workspace | `pooja-vinayagamoorthys-workspace` |
| Asset | `pooja-vinayagamoorthys-workspace/tataro` (UUID `879f3f60-94f7-4bd7-bb53-6233dbb93c79`, private) |
| Environment | **TATARO** — <https://cyberwave.com/pooja-vinayagamoorthys-workspace/envs/tatarov1> (UUID `810a182b-d950-4b0e-a9af-66faab2589c7`) |
| Live twin | **TATARO Lite6** — `72434eab-22ee-480f-9a27-f8db2c1a551c` |
| Old test twin (unbound) | `a9d38285-a66f-475e-93cc-2aa3efada1ce` in *Quickstart Environment* |
| Controller | **TATARO keyboard controller** `ddb34c5c-92e5-46f0-a4ef-09bfe40e71a9` (driver‑driven keyboard) |
| Lite6 controller | `192.168.1.150` (firmware v2.8.2); this PC is `192.168.1.100` on Ethernet `eno1` |
| D435i | librealsense serial **`141722076951`** (USB/V4L serial `131323060798` — do **not** use this one in Cyberwave) |
| API key file | `~/.config/cyberwave/env` (mode 600). Load with `set -a; . ~/.config/cyberwave/env; set +a` |

---

## 3. Files

| Path | Content |
|---|---|
| `~/lite6_cyberwave_check/lite6.urdf` + `meshes/` | Fixed URDF that the asset was built from |
| `~/lite6_cyberwave_check/upload/` | Upload ZIP and thumbnail |
| `~/lite6_cyberwave_check/tataro-lite6-driver/` | **Driver source** (Cyberwave `BaseDriver`) — `hardware.py` (xArm SDK), `driver.py` (interface + safety gate) |
| `~/lite6_cyberwave_check/l0_playground_test.py` | L0 joint sweep |
| `~/lite6_cyberwave_check/l1_sim_test.py` | L1 MuJoCo hold test |
| `~/lite6_cyberwave_check/l2_mirror_check.py` | Real‑vs‑twin mirror/lag check |
| `~/lite6_cyberwave_check/sysid_collect.py`, `sysid_data/` | System identification runs and data |

> ⚠️ `~/Downloads/lite6.urdf` is **corrupt** (all six arm joints had the same bogus origin, link2 mesh offset −0.591 m). The schema validator did not catch it — always render / FK‑check a URDF before uploading.

---

## 4. Robot model (asset)

- Kinematics: official UFactory `lite6_default_kinematics.yaml` (FK matches the real TCP to ~4 mm).
- `link_base` inertia estimated from the collision mesh (not measured).
- Gripper mass 0.277 kg (matches the controller's payload setting).
- Fixed base, simulators: Playground + MuJoCo; gravity −9.81, timestep 2 ms.
- **Position actuators** (critically damped): kp = 680 / 2180 / 1020 / 210 / 190 / 130 for joints 1–6, 500 for the gripper. Without explicit actuators Cyberwave generates torque motors and the arm never follows position targets.
- Measured friction / damping: joint2 2.0 N·m / 6.0 N·m·s/rad, joint3 1.6 / 1.2 (joints 1, 4–6 still the URDF default 1 / 1).
- Sensors: `d435i_color` (camera) on `camera_color_optical_frame`, `d435i_depth` (depth_camera) on `camera_depth_optical_frame`, both rotated 180° about X (Cyberwave cameras look along −Z), 1280×720, range 0.3–3 m. **The depth sensor is required** — without it Cyberwave opens the camera as a plain webcam and fails.
- Scene (TATARO environment): work table 0.92 m high, robot base at z = 0.930.

> Twins keep **their own copy** of the schema. Changes to the asset must also be applied to existing twins (verify with an environment MuJoCo export).

---

## 5. Driver (edge)

Two services run on this PC under Cyberwave Edge Core (systemd `cyberwave-edge-core`, starts at boot):

| Service / container | Image | Role |
|---|---|---|
| `cyberwave-driver-72434eab-lite6` | `tataro-lite6-driver:0.3.3` (**local, unpublished**) | Arm telemetry + control |
| `cyberwave-driver-72434eab-camera` | `cyberwaveos/camera-driver:latest-realsense` (`--privileged`) | D435i colour + depth streaming |

Driver settings (twin `metadata.drivers`): host networking, `XARM_IP=192.168.1.150`, `LITE6_MOTION_ENABLED=true`, `LITE6_MAX_SPEED_DEG=20`, `LITE6_MAX_STEP_DEG=20`, `CYBERWAVE_MQTT_PROTOCOL=5`. Camera: `serial_number=141722076951`, `is_depth_camera=true`, `resolution=1280x720`, `depth_fps=5`.

### Safety behaviour of the arm driver
- Accepts commands **only while a controller is attached** (otherwise idle / `NO_OP`).
- Every joint target is checked: within URDF limits − 1° margin, at most **20° from the measured pose**, arm error‑free, status data fresh (< 0.5 s). Otherwise **rejected** + twin alert.
- Motion through the xArm controller's online joint planner at ≤ 20 °/s; untouched joints hold their measured position.
- Stops the arm on `stop`, on controller removal and on driver shutdown.
- Alerts on controller errors (e.g. e‑stop code 2), stale status stream and rejected targets.
- `LITE6_MOTION_ENABLED=false` → dry run (logs what it would do, never moves).

### Gripper (Gripper Lite — binary, no position feedback)
- Commands `grip` (close) / `release` (open), or `finger_joint1` / `finger_joint2` targets (both fingers = one gripper).
- Key jogs: any increase opens, any decrease closes; typed values: ≥ 4.45 mm open, below closes.
- The twin shows the **last commanded** finger state (0 or 8.9 mm), not a measurement.

---

## 6. Operating it

### Keyboard (dashboard, LIVE mode, TATARO Lite6)

| Keys | Action |
|---|---|
| W / Q | joint1 + / − |
| R / E | joint2 + / − |
| Y / T | joint3 + / − |
| S / A | joint4 + / − |
| F / D | joint5 + / − |
| H / G | joint6 + / − |
| **2** | release (open gripper) |
| **1** | grip (close gripper) |
| **M** | stop |
| X / Z, V / C | finger_joint1, finger_joint2 + / − |

Each key press ≈ 2.9° (0.05 rad).

**Rules**
- From the folded **zero pose, raise joint3 (Y) first**. Moving joint2 first triggers the controller's self‑collision stop (C22). Joint3 cannot go below −2.5° (T is refused at zero).
- Keep the e‑stop in reach. After an e‑stop: release it, then **Clear Error** in UFactory Studio.
- A safe working start pose: (0, 10, 70, 0, 60, 0)°.

### Python SDK
```python
import math
from cyberwave import Cyberwave
cw = Cyberwave()                                   # reads CYBERWAVE_API_KEY
arm = cw.twin(twin_id="72434eab-22ee-480f-9a27-f8db2c1a551c")
cw.affect("live")
arm.policy.ensure_attached()                       # wait for the MQTT connection before sending
arm.joints.set({"joint3": math.radians(20)})       # absolute target, radians
```

### Common commands
```bash
sudo systemctl restart cyberwave-edge-core     # apply twin/driver changes (dashboard restart can get stuck)
cyberwave edge status                           # service + containers
docker logs -f cyberwave-driver-72434eab-lite6  # arm driver log (Moving / REJECTED / Gripper lines)
docker stop cyberwave-driver-72434eab-camera    # free the D435i for realsense-viewer / TATARO_CV scripts
```

---

## 7. Troubleshooting (things that already happened)

| Symptom | Cause | Fix |
|---|---|---|
| Arm driver missing, log says *"Failed to pull tataro-lite6-driver… no local copy"* | Local image deleted (e.g. `docker image prune`) | Rebuild: `cd ~/lite6_cyberwave_check/tataro-lite6-driver && docker build -t tataro-lite6-driver:0.3.3 .` then restart edge core |
| Red *"docker pull failed … 404"* on every start | Image only exists locally | Harmless; goes away once the image is published |
| Dashboard keys do nothing | Controller with `input_device: sdk` (Python SDK) attached | Attach a keyboard controller (TATARO keyboard controller) |
| Key presses "REJECTED … outside limits / step exceeds 20°" | Safety gate working (e.g. joint3 below 0, or a saved pose > 20° away) | Use the other direction / move in smaller steps |
| Dashboard restart says *"restart already in progress"* | Stuck edge‑core state | `sudo systemctl restart cyberwave-edge-core` |
| Arm error 2 "Emergency IO triggered" | E‑stop pressed | Release e‑stop, Clear Error in UFactory Studio |
| Camera fails on `/dev/video2` | No depth sensor declared / wrong serial | Keep `d435i_depth` sensor and serial `141722076951` |
| Camera MQTT disconnects every ~2 min | Too much upload over the Wi‑Fi hotspot (raw depth at 30 fps) | `depth_fps=5`; prefer Ethernet for internet |
| `Docker … nvidia-container-runtime not found` | NVIDIA runtime set as default but toolkit missing | `sudo apt-get install nvidia-container-toolkit` + `nvidia-ctk runtime configure` |
| `pip install cyberwave-cli` broke viser / lerobot | CLI pins `rich==15` | CLI lives in its own venv `~/.venvs/cyberwave-cli` (linked as `~/.local/bin/cyberwave`) |

---

## 8. Known gaps / open questions

- **SDK → cloud MuJoCo:** `joints.set` in simulation mode is not applied (dashboard Joint targets works). Question for Cyberwave support.
- **Measured joint state:** the backend's stored joint state (`GET /twins/{uuid}/joint-states`) never shows the driver's telemetry as measured; the Cyberwave assistant therefore reports "no measured joint state". Question for Cyberwave support.
- **System identification:** joint2/3 friction and current→torque factor (~6.5 / 5.7 N·m/A) are first estimates (±10 %); joints 1, 4–6 unmeasured; the Lite6 "estimated joint torque" is model‑computed, use servo currents.
- Dashboard **Stop** action shows *Unavailable*; **Move end effector / Move to point** need Cartesian commands (not implemented).
- Saved poses (e.g. *Zero Pose*) are rejected when > 20° away.

---

## 9. Next steps

1. **Publish the driver image** to private Docker Hub `pooja15v/tataro-lite6-driver` (create the private repo, `docker login` + `sudo docker login`), then point the twin at it. (`registry.cyberwave.com` is Enterprise‑only and does not resolve for this account.)
2. **L3 safety evidence:** cut the network during a slow teleop move → confirm the arm is safe and an alert is raised.
3. Allow single large targets (saved poses) to execute slowly through the controller planner.
4. Fix the dashboard **Stop** action; optionally bind both fingers to the same keys.
5. Set `metadata.drivers` on the **asset** so new TATARO twins get the driver automatically.
6. **L4:** dock the camera, record teleop episodes, export (LeRobot), fine‑tune SmolVLA/OpenVLA, assign as controller.
