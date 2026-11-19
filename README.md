# TATARO Firmware V1.0

Software for TATARO: a UFactory **Lite6** arm with a **Gripper Lite**, a wrist RealSense D435i and an overhead
RealSense D455, doing colour-cube pick-and-place (cube -> bowl) and therapy tasks.

| Folder | What it is |
|---|---|
| `xarm_ros2/` | ROS 2 Humble workspace: UFactory's [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) (branch `humble`, upstream `d0b9511`) **plus TATARO packages** in `src/` (`tataro_packages`, `computer_vision`, `therapy_tasks`, `sensors`, `tatro_handeye_sim`, `tataro_gazebo_environment`, `tf_recorder`, `realsense2_ros_mqtt_bridge`), calibration results (`Calibration/`), Isaac Sim / MOCAP validation work and the TATARO camera-mount changes in `xarm_description`. |
| `xArm-Python-SDK/` | UFactory's [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK) (upstream `a1adbae`) plus `lite6_calibration_pooja/`. |
| `lerobot_tataro/` | LeRobot plugin for the Lite6 (`--robot.type=lite6`), recording / training / evaluation scripts (ACT, `eval_offline.py --live`, `run_policy.py`), the Gemini Robotics-ER / local scripted pick (`scripts/gemini_pick.py`) and the calibrated MuJoCo scene (`sim/`). See `lerobot_tataro/README.md`. |

Datasets are **not** in this repo: they are in [TATARO_LeRobot_Dataset](https://github.com/Pooja15V/TATARO_LeRobot_Dataset)
(clone it into `lerobot_tataro/datasets/`).

## Build (ROS 2)
```bash
cd xarm_ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```
Note: upstream xarm_ros2 normally pulls `xarm_sdk/cxx` as a git submodule; here it is included as plain files.

## LeRobot
```bash
conda activate lerobot          # python 3.12, lerobot installed from source
cd lerobot_tataro && pip install -e lerobot_robot_lite6
```

Third-party code keeps its original licenses (see the LICENSE files inside `xarm_ros2/` and `xArm-Python-SDK/`).
