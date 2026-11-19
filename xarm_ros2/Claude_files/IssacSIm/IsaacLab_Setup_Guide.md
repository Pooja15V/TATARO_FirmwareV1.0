# Isaac Lab Setup Guide — Lite6 Robot + TATARO Workbench

**Date:** 2026-06-16  
**Status:** ✅ Complete and Verified  
**Model:** UFACTORY Lite6 (6-DOF) + Lite Gripper + RealSense D435i  
**Environment:** Ubuntu 22.04, RTX 4060 Laptop GPU, Isaac Lab 0.54.4

---

## 🎯 Quick Start

### Launch the Robot in Isaac Lab

```bash
conda activate env_isaaclab
export OMNI_KIT_ACCEPT_EULA=YES
cd ~/IsaacLab
./isaaclab.sh -p ~/lite6_isaac/launch_lite6.py
```

Or with the improved script:
```bash
./isaaclab.sh -p ~/lite6_isaac/launch_lite6_proper.py
```

**Headless mode** (no GUI, for training):
```bash
./isaaclab.sh -p ~/lite6_isaac/launch_lite6.py --headless
```

---

## 📦 Installation Summary

### Environment
- **Conda env:** `env_isaaclab` (Python 3.11)
- **Isaac Sim:** 5.1.0
- **Isaac Lab:** 0.54.4
- **Torch:** 2.7.0 + cu128
- **RL Frameworks:** RSL-RL, SKRL, Stable-Baselines3, RL-Games (all installed)

### Key Files

| File | Location | Purpose |
|------|----------|---------|
| Robot USD | `~/lite6_isaac/lite6_gripper_d435i.usd` | Physics + rendering asset |
| Workbench USD | `~/lite6_isaac/lite6_gripper_d435i_workbench.usd` | Robot + table geometry |
| Robot URDF | `~/lite6_isaac/lite6_gripper_d435i_clean.urdf` | Source robot description |
| Launch script | `~/lite6_isaac/launch_lite6.py` | Main simulation entry point |
| Launch script (v2) | `~/lite6_isaac/launch_lite6_proper.py` | Improved hierarchy version |
| Xacro config | `~/lite6_isaac/tataro_device_urdf.xacro` | Robot + workbench URDF template |

### Source Assets
- **Lite6 URDF xacro:** `~/xarm_ros2/install/xarm_description/share/xarm_description/urdf/xarm_device.urdf.xacro`
- **Workbench mesh:** `~/xarm_ros2/install/xarm_description/share/xarm_description/meshes/tataro_setup/`
  - Visual: `tataro_physicalsetup.stl`
  - Collision: `tataro_physicalsetup.stl`

---

## 🤖 Robot Configuration

### Kinematic Chain
```
world
  ├─ link_base (fixed to world)
  ├─ link1 (joint1: revolute, ±178°)
  ├─ link2 (joint2: revolute, ±150°)
  ├─ link3 (joint3: revolute, ±179°)
  ├─ link4 (joint4: revolute, ±180°)
  ├─ link5 (joint5: revolute, ±124°)
  ├─ link6 (joint6: revolute, ±180°)
  ├─ link_eef (fixed end-effector frame)
  ├─ uflite_gripper_link (Lite Gripper)
  └─ link_tcp (tool center point, Z=83.6mm from gripper base)
```

### Actuators
- **Type:** Implicit PD (position control)
- **All joints:** Stiffness=400, Damping=40
- **Control:** `robot.set_joint_position_target(target_pos)`

### Sensors (Available)
- **D435i Camera:** Mounted on wrist, configurable for RGB/depth/segmentation
- **Contact sensors:** Available on all links (enable via config)
- **Joint state:** Position, velocity, torque (read via `robot.data`)

---

## 📋 Launch Behavior

When you run `launch_lite6.py`, the script:

1. **Initializes Isaac Sim** with GPU physics (120 Hz)
2. **Spawns ground plane** and dome lighting
3. **Loads robot USD** at `/World/Lite6`
4. **Resets simulation** and prints:
   ```
   [INFO] Lite6 loaded.
   [INFO] Bodies: ['world', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6']
   [INFO] Joints: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
   [INFO] Default joint pos: [[0. 0. 0. 0. 0. 0.]]
   ```
5. **Animates sinusoidal motion** on joints 1, 2, 4 for 3+ seconds
6. **Outputs end-effector pose** before exiting (headless mode)

---

## 🛠️ How to Modify

### Change Control Gains
Edit `launch_lite6.py` line 44-48:
```python
actuators={
    "arm": ImplicitActuatorCfg(
        joint_names_expr=["joint[1-6]"],
        stiffness=400.0,  # ← Change here (default 400)
        damping=40.0,     # ← Change here (default 40)
    ),
},
```

### Add Objects to Grasp
In `main()`, after `sim.reset()`:
```python
from isaaclab.assets import RigidObject, RigidObjectCfg

cube_cfg = RigidObjectCfg(
    prim_path="/World/Cube",
    spawn=sim_utils.CuboidCfg(size=(0.05, 0.05, 0.05)),
)
cube = RigidObject(cube_cfg)
```

### Add Vision Sensor (D435i)
```python
from isaaclab.sensors import Camera, CameraCfg

camera_cfg = CameraCfg(
    prim_path="/World/Lite6/link6/Camera",
    height=480, width=640,
    data_types=["rgb", "depth"],
)
camera = Camera(camera_cfg)
```

---

## 🚀 Next Steps: RL Training Pipeline

### 1. Build Manager-Based RL Environment
Create `task_lite6_pick.py`:
```python
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg

env_cfg = ManagerBasedRLEnvCfg(
    scene=SceneConfig(...),  # robot + objects
    observations=ObsManagerCfg(...),  # state/vision
    actions=ActionManagerCfg(...),  # joint targets
    rewards=RewardManagerCfg(...),  # task rewards
    terminations=TerminationManagerCfg(...),  # episode end
)

env = ManagerBasedRLEnv(env_cfg)
```

### 2. Define Rewards (Example)
- **Reach reward:** Distance from gripper to target object
- **Grasp reward:** Contact force magnitude
- **Place reward:** Object position at goal location

### 3. Train with SKRL
```bash
python -m skrl.trainers.torch \
  --task lite6_pick \
  --experiment_name my_first_policy \
  --env_config task_lite6_pick.py
```

### 4. Evaluate & Export
```python
policy = env.agent.policy
policy.eval()
with torch.no_grad():
    action, _ = policy(observations)
```

### 5. Deploy to Real Robot
Transfer trained policy weights to real Lite6 (same kinematics/joint order).

---

## 🔧 Machine-Specific Notes

### RTX 4060 Laptop (8 GB VRAM)
- ✅ **Simulation:** Runs stably at 120 Hz with 1 robot
- ⚠️ **Training:** Fit ~4-8 parallel envs; use `--num-envs` flag
- ⚠️ **Memory:** Swapfile increased to 16 GB (created 2026-06-16)

### CPU Governor
```bash
# Check current
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
# Set to performance (optional, speeds up startup)
sudo cpupower frequency-set -g performance
```

### Network Issues
- Isaac Sim's first launch compiles shaders (slow on powersave governor)
- Subsequent launches use cache (~1 min startup)
- If hung on plugin load, check `/tmp/isaaclab/logs/` for errors

---

## 📚 Related Documentation

- **Isaac Lab Docs:** https://isaac-sim.github.io/IsaacLab/main/
- **Isaac Lab GitHub:** https://github.com/isaac-sim/IsaacLab
- **RL Framework Docs:**
  - SKRL: https://github.com/isaac-sim/skrl
  - RSL-RL: https://github.com/NVIDIA-Omniverse/IsaacLab/tree/main/source/isaaclab_rl/rsl_rl
- **ROS 2 xarm packages:** `~/xarm_ros2/`

---

## ✅ Verification Checklist

- [x] Isaac Lab 0.54.4 installed in `env_isaaclab`
- [x] Lite6 URDF generated with gripper + D435i
- [x] USD converted with correct joint limits & dynamics
- [x] GUI launches without duplicate arms
- [x] Robot animates smoothly under PD control
- [x] Bodies: 7 (world + 6 links) ✓
- [x] Joints: 6 (joint1-6) ✓
- [x] Physics timestep: 1/120 s ✓

---

**Ready to train!** 🎯  
For questions or issues, check logs at `~/.nvidia-omniverse/logs/` or `/tmp/isaaclab/logs/`.
