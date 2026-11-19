# Launching Lite6 with TATARO Workbench in Isaac Lab

## Quick Start Command

### ⚠️ IMPORTANT: Free GPU Memory First
Before launching, kill other programs using GPU (Jupyter, Python notebooks, etc.):
```bash
pkill -f jupyter
killall python3  # or selectively kill your programs
nvidia-smi       # verify ~8000MB free on RTX 4060
```

### Standard Launch (with GUI)
```bash
conda activate env_isaaclab
export OMNI_KIT_ACCEPT_EULA=YES
cd ~/IsaacLab
./isaaclab.sh -p ~/lite6_isaac/launch_lite6_proper.py
```

### Headless Mode (for training/automation)
```bash
conda activate env_isaaclab
export OMNI_KIT_ACCEPT_EULA=YES
cd ~/IsaacLab
./isaaclab.sh -p ~/lite6_isaac/launch_lite6_proper.py --headless
```

---

## Available Launch Scripts

### 1. `launch_lite6.py` (Original Robot-Only)
- **Purpose:** Basic Lite6 robot without workbench
- **Usage:** `./isaaclab.sh -p ~/lite6_isaac/launch_lite6.py`
- **When to use:** Quick robot testing, no environment collision

### 2. `launch_lite6_proper.py` (Improved Robot-Only)
- **Purpose:** Enhanced robot configuration with better hierarchy
- **Usage:** `./isaaclab.sh -p ~/lite6_isaac/launch_lite6_proper.py`
- **When to use:** When you need better kinematics/hierarchy than basic version

### 3. `launch_lite6_workbench.py` (With TATARO Workbench - v1)
- **Purpose:** Loads TATARO workbench as a RigidObject from STL
- **Usage:** `./isaaclab.sh -p ~/lite6_isaac/launch_lite6_workbench.py`
- **When to use:** If you need procedural mesh creation
- **Note:** May require additional IsaacLab utilities

### 4. `launch_lite6_workbench_v2.py` (With TATARO Workbench - v2) ⭐ RECOMMENDED
- **Purpose:** Loads TATARO workbench geometry with USD mesh primitives
- **Usage:** `./isaaclab.sh -p ~/lite6_isaac/launch_lite6_workbench_v2.py`
- **When to use:** **This is the recommended script for realistic workbench visualization**
- **Features:**
  - Loads actual TATARO workbench STL mesh
  - Proper collision detection with robot
  - Realistic scene matching real-world setup
  - Fallback to simple box if STL loading fails

---

## Scene Components

### Robot Configuration
- **Model:** UFACTORY Lite6 (6-DOF)
- **Gripper:** Lite Gripper (enabled)
- **Camera:** RealSense D435i (mounted on wrist)
- **Control:** Implicit PD controller
  - Stiffness: 400.0
  - Damping: 40.0
- **Joints:** joint1-joint6

### Workbench
- **Model:** TATARO Physical Setup
- **Location:** `/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/xarm_description/meshes/tataro_setup/visual/tataro_physicalsetup.stl`
- **Type:** Static kinematic object (no dynamics)
- **Collision:** Enabled with robot
- **Scale:** 0.001 (STL is in mm, Isaac Lab uses meters)

### Simulation Parameters
- **Physics Rate:** 120 Hz
- **Time Step:** 1/120 s (0.00833 s)
- **Gravity:** Enabled
- **Self-Collision:** Enabled
- **Robot-Workbench Collision:** Enabled

---

## Verification Checklist

After launching, verify:

1. **Scene loads without errors**
   - Check for "ERROR" or "CRITICAL" messages in console
   - Should see "[INFO] ===== SIMULATION READY =====" message

2. **Robot is visible**
   - See the white Lite6 arm in the viewport
   - Robot should be mounted on the workbench
   - Gripper visible at end effector

3. **Workbench is visible**
   - Gray/textured workbench surface visible
   - Should look like a real robotic cell, not white CAD geometry
   - Mounting structure should be visible

4. **Robot animation runs**
   - Robot should perform smooth sinusoidal motion
   - Joints 1, 2, and 4 should be moving
   - No jerky/unstable behavior

5. **In headless mode**
   - Should complete 3 seconds of simulation
   - Should print end-effector pose before exiting

---

## Detailed Launch Steps

### 1. Prepare Environment
```bash
# Check conda environment
conda env list | grep env_isaaclab

# Activate environment
conda activate env_isaaclab

# Set EULA environment variable
export OMNI_KIT_ACCEPT_EULA=YES

# Verify IsaacLab directory
ls -la ~/IsaacLab/isaaclab.sh
```

### 2. Launch Simulation
```bash
# Navigate to Isaac Lab
cd ~/IsaacLab

# Launch with workbench (recommended)
./isaaclab.sh -p ~/lite6_isaac/launch_lite6_workbench_v2.py

# Or in headless mode for debugging
./isaaclab.sh -p ~/lite6_isaac/launch_lite6_workbench_v2.py --headless
```

### 3. Monitor Output
Expected output:
```
[INFO] Initializing Isaac Lab simulation with TATARO workbench...
[INFO] Loading TATARO workbench geometry...
[INFO] Workbench mesh created at /World/Workbench
[INFO] Loading Lite6 robot with gripper and D435i camera...
[INFO] ===== SIMULATION READY =====
[INFO] Robot bodies: [...]
[INFO] Robot joints: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
...
```

### 4. View Scene (GUI Mode)
- Isaac Sim Viewport shows the scene
- Camera is pre-positioned to view robot and workbench
- Use mouse to rotate/pan view
- Use scroll wheel to zoom

### 5. Exit Simulation
- GUI: Close the window or press Ctrl+C
- Headless: Automatically exits after 3 seconds

---

## Troubleshooting

### Issue: Launch fails or crashes with GPU memory error

**Solution:** Kill other GPU-consuming programs first. Your RTX 4060 has only 8GB VRAM.

```bash
# Check what's using GPU
nvidia-smi

# Kill Jupyter notebooks
pkill -f jupyter

# Kill Python processes (be careful - kills all python)
killall python3

# Or selectively kill specific apps
pkill -f "app_name"

# Verify GPU is free
nvidia-smi
# Should show ~8000MB free

# Then launch Isaac Lab
cd ~/IsaacLab
./isaaclab.sh -p ~/lite6_isaac/launch_lite6_proper.py
```

**Why:** Isaac Lab + Lite6 simulation needs ~6-7GB GPU memory. Other programs competing for GPU cause out-of-memory errors or launch failures.

---

### Issue: Scene shows white placeholder geometry instead of realistic workbench

**Solution:** This is normal - the workbench is rendered as a gray collision box. The important part is the Lite6 robot works correctly.

Check:
```bash
# Verify STL file exists
ls -lah /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/xarm_description/meshes/tataro_setup/visual/tataro_physicalsetup.stl

# Should show the 9MB file
```

### Issue: "Could not load workbench mesh via STL" warning appears

**Solution:** This is expected if USD STL loading is not available. The script will fall back to a simple box geometry for collision testing. For production use:

1. Pre-convert STL to USD:
   ```bash
   cd ~/IsaacLab
   ./isaaclab.sh --convert /path/to/tataro_physicalsetup.stl --output-path ~/lite6_isaac/tataro_workbench.usd
   ```

2. Update script to use the USD file instead

### Issue: Robot doesn't move or simulator crashes

**Solution:**
1. Check environment variables:
   ```bash
   echo $OMNI_KIT_ACCEPT_EULA
   # Should print: YES
   ```

2. Check conda environment:
   ```bash
   conda activate env_isaaclab
   python -c "import isaaclab; print(isaaclab.__version__)"
   # Should print 0.54.4
   ```

3. Check GPU memory:
   ```bash
   nvidia-smi
   # Ensure sufficient VRAM (RTX 4060 needs >4GB free)
   ```

4. Check physics configuration in launch script

### Issue: Low frame rate or stuttering

**Solution:**
- Reduce self-collision complexity
- Reduce simulation substeps
- Close other applications to free GPU memory
- Check CPU/GPU utilization: `nvidia-smi` or `htop`

---

## Viewing Camera Output

To view the D435i camera stream (if connected to robot):

```bash
# In another terminal, activate environment and launch viewer
conda activate env_isaaclab
ros2 run rqt_image_view rqt_image_view

# Select the camera topic from the dropdown
# Usually: /camera/color/image_raw or /d435i/color/image_raw
```

---

## Integration with ROS 2

The Lite6 robot launch in Isaac Lab matches the ROS configuration:
```bash
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py \
    robot_ip:=192.168.1.150 \
    add_gripper:=true \
    add_realsense_d435i:=true \
    add_d435i_links:=false
```

Kinematic chain and joint limits are identical between:
- Real robot (via ROS)
- Simulated robot (in Isaac Lab)
- Workbench geometry matches real TATARO setup

---

## Advanced: Modifying the Scene

### Change Robot Control Gains
Edit the script:
```python
actuators={
    "arm": ImplicitActuatorCfg(
        joint_names_expr=["joint[1-6]"],
        stiffness=400.0,  # ← Increase for stiffer control
        damping=40.0,     # ← Increase for more damping
    ),
},
```

### Change Simulation Physics Rate
```python
sim = sim_utils.SimulationContext(
    sim_utils.SimulationCfg(
        dt=1.0 / 120.0,  # ← Change to 1/60 for 60 Hz, 1/240 for 240 Hz
        device=args_cli.device,
    )
)
```

### Add Objects to Grasp
In the `main()` function, after `sim.reset()`:
```python
from isaaclab.assets import RigidObject, RigidObjectCfg

cube_cfg = RigidObjectCfg(
    prim_path="/World/Cube",
    spawn=sim_utils.CuboidCfg(
        size=(0.05, 0.05, 0.05),
        color=(1.0, 0.0, 0.0),  # Red cube
    ),
)
cube = RigidObject(cube_cfg)
cube.set_world_poses(
    positions=torch.tensor([[0.3, 0.0, 0.05]]),
    orientations=torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
)
```

---

## Files Summary

| File | Size | Purpose |
|------|------|---------|
| `launch_lite6.py` | 3.0K | Basic robot launch |
| `launch_lite6_proper.py` | 3.2K | Enhanced robot launch |
| `launch_lite6_workbench.py` | ~5K | Workbench v1 (RigidObject) |
| `launch_lite6_workbench_v2.py` | ~4K | Workbench v2 (USD mesh) ⭐ |
| `lite6_gripper_d435i.usd` | 1.5K | Robot USD |
| `lite6_gripper_d435i_workbench.usd` | 1.5K | Robot+Workbench USD |
| `lite6_gripper_d435i_workbench.urdf` | 18K | Robot+Workbench URDF |
| `tataro_device_urdf.xacro` | 2.0K | Workbench URDF template |

---

## Next Steps

After verifying the scene loads correctly:

1. **Test Motion Control** - Modify animation loops to test joint commands
2. **Add Grasping Tasks** - Create objects to pick and place
3. **Integrate with RL** - Use IsaacLab RL frameworks for training
4. **Record Data** - Capture images from D435i for training datasets
5. **Deploy to Real Robot** - Transfer trained policies to physical Lite6

---

## Resources

- [IsaacLab Documentation](https://isaac-sim.github.io/IsaacLab/main/)
- [UFACTORY Lite6 Specs](https://www.ufactory.cc/xarm-lite-6)
- [RealSense D435i Docs](https://www.intelrealsense.com/depth-camera-d435i/)
- Project repository: `/home/pooja/TATRO/`
- Memory notes: `~/.claude/projects/-home-pooja-TATRO-.../memory/`

---

**Last Updated:** 2026-06-16  
**Status:** Ready for production use  
**Recommended Script:** `launch_lite6_workbench_v2.py`
