# Hand-Eye Calibration Fix Guide
## UFactory Lite6 + RealSense D435i + ArUco Marker

---

## **PART 1: DIAGNOSE THE PROBLEM**

### **Step 1: Measure Actual Position Error**

First, quantify how far off the robot is:

```bash
# Terminal 2 (while tatro_runtime.launch.py is running):
cd /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2

# Rebuild with diagnostic script
colcon build --symlink-install --packages-select tataro_packages
source ./install/setup.bash

# Run diagnostic
ros2 run tataro_packages tataro_calibration_diagnostic
```

**Expected output:**
```
[MARKER] Detected at: x=+488.3 mm, y=+11.2 mm, z=+20.4 mm
[ROBOT] Connected. Current TCP: (275.4, 1.4, 415.9)
[MOVE] Moving to marker hover position (neutral orientation):
  Target: x=488.3, y=11.2, z=120.4 mm
[ACTUAL] TCP after move: (x=?, y=?, z=?)
[ERROR ANALYSIS]
  ΔX: ? mm
  ΔY: ? mm
  ΔZ: ? mm
  XY Error: ? mm (horizontal)
  XYZ Error: ? mm (total)
```

**What this tells you:**
- ✅ XYZ Error < 10 mm: Calibration is good, no fix needed
- 🟡 XYZ Error 10-50 mm: Acceptable, but recalibration recommended
- ❌ XYZ Error > 50 mm: Major issue, must recalibrate

---

## **PART 2: ROOT CAUSE ANALYSIS**

### **Issue 1: Poor Calibration Quality**

**Evidence from your data:**
- Session 1 has ±65 mm variation in Z
- Session 2 has ±43 mm variation in Z
- Session 3 has ±60 mm variation in Z
- **Ideal:** ±2-5 mm (very tight)

**Likely causes:**
1. ❌ **Too few calibration samples** (need 15-25 minimum)
2. ❌ **Bad sample distribution** (all poses in similar area)
3. ❌ **Camera not rigidly mounted** (flexes during motion)
4. ❌ **ArUco detection failing on some poses** (lighting issues)
5. ❌ **Outlier poses included** (pose 3E: Z=177.59 vs others: 26.85)

---

### **Issue 2: Possible Outliers in Calibration**

Look at Session 3:
```
3A: TX=65.29, TY=26.84, TZ=26.85 mm  ← Consistent
3B: TX=65.29, TY=26.84, TZ=26.85 mm  ← Identical to 3A
3C: TX=66.88, TY=30.01, TZ=27.20 mm  ← Very close
3D: TX=67.10, TY=30.00, TZ=26.69 mm  ← Very close
3E: TX=81.45, TY=4.82,  TZ=177.59 mm ← ❌ OUTLIER!
```

**Pose 3E is clearly an outlier.** It was probably a bad sample.

---

## **PART 3: SOLUTION STRATEGY**

### **Option A: Use Better Calibration (Quickest)**

Some of your calibrations are more consistent than others. Compare:

```bash
# Test 3A (current) vs others
python3 << 'EOF'
import yaml

# Test 3A (current)
with open('/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/Calibration/tataro_handeye_3A.calib') as f:
    calib = yaml.safe_load(f)
    print(f"Current (3A): TZ={calib['transform']['translation']['z']*1000:.2f} mm")

# Test 3B, 3C, 3D (exclude 3E which is outlier)
for letter in ['B', 'C', 'D']:
    with open(f'/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/Calibration/tataro_handeye_3{letter}.calib') as f:
        calib = yaml.safe_load(f)
        tz = calib['transform']['translation']['z']*1000
        print(f"3{letter}: TZ={tz:.2f} mm")
EOF
```

**Try with different calibrations:**

```bash
# Test with 3C (middle ground from Session 3)
ros2 launch tataro_packages tatro_runtime.launch.py \
  robot_ip:=192.168.1.150 \
  calib_name:=tataro_handeye_3C

# Then in Terminal 2:
ros2 run tataro_packages tataro_calibration_diagnostic
```

---

### **Option B: Re-Calibrate Properly (RECOMMENDED)**

If error > 30 mm, recalibration is needed. Follow this procedure:

#### **Step 1: Prepare for Calibration**

```bash
# Stop all running nodes
# Terminal 1: [CTRL+C] on tatro_runtime.launch.py

# Start fresh calibration stack
ros2 launch easy_handeye2 calibrate.launch.py \
  name:=tataro_handeye_4 \
  calibration_type:=eye_in_hand \
  robot_base_frame:=link_base \
  robot_effector_frame:=link_eef \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=aruco_marker_frame
```

#### **Step 2: Check Camera & Marker Visibility**

```bash
# Terminal 2:
# In RViz, you should see:
# 1. Robot with camera on link_eef
# 2. ArUco marker frame in camera view
# 3. Marker should be clearly visible and well-lit

# If marker not visible:
# - Increase lighting
# - Ensure marker is >= 10x10 cm and crisp
# - Check marker ID matches (marker_id=0)
```

#### **Step 3: Collect Calibration Samples**

**CRITICAL: Good sample distribution**

Move the robot to **at least 15-20 different poses** such that:
- ✅ Marker stays visible in all poses
- ✅ Marker is at different angles (not all front-facing)
- ✅ Camera is at different distances (30-80 cm from marker)
- ✅ Marker is in different parts of camera FOV
- ✅ Roll/pitch/yaw varies

**Do NOT:**
- ❌ Take all samples from the same location
- ❌ Move robot too fast (ArUco detection might fail)
- ❌ Occlude marker with gripper
- ❌ Use dim lighting

#### **Step 4: Run Calibration**

```bash
# In the easy_handeye2 GUI:
# 1. Press "Compute Calibration"
# 2. Wait for result
# 3. Check reported error (should be < 3 mm)
# 4. If error > 5 mm, redo with better samples
# 5. Save calibration
```

#### **Step 5: Validate Result**

```bash
# Run diagnostic with new calibration
ros2 launch tataro_packages tatro_runtime.launch.py \
  robot_ip:=192.168.1.150 \
  calib_name:=tataro_handeye_4

# Terminal 2:
ros2 run tataro_packages tataro_calibration_diagnostic

# Check error < 25 mm
```

---

## **PART 4: QUICK CHECKLIST FOR EACH OPTION**

### **Option A: Use Better Calibration**

- [ ] Run diagnostic with current 3A
- [ ] Record XYZ error value
- [ ] If error > 30 mm: Proceed to Option B
- [ ] If error < 30 mm: Try 3C or 3D (exclude 3E)
- [ ] Run diagnostic again with new calibration
- [ ] Compare errors, use best one

### **Option B: Re-Calibrate**

- [ ] Backup current calibrations (tar czf backup.tar.gz Calibration/)
- [ ] Start easy_handeye2 calibrate
- [ ] Improve lighting conditions
- [ ] Ensure marker is large (15cm+) and crisp
- [ ] Collect 20+ samples with good distribution
- [ ] Run calibration, verify error < 3 mm
- [ ] Save as tataro_handeye_4A
- [ ] Validate with diagnostic

---

## **PART 5: PREVENTIVE MEASURES**

To avoid this in the future:

### **1. Camera Mounting**
- Use rigid metal bracket, NOT plastic
- Tighten all bolts/screws regularly
- Check for play: camera should not shift under finger pressure

### **2. ArUco Marker**
- Use high-quality printed marker (laser printer, matte finish)
- Minimum size: 10cm x 10cm (prefer 15cm x 15cm)
- Mount on flat, rigid surface
- Ensure good lighting (>200 lux)

### **3. Calibration Procedure**
- Use 20+ samples minimum
- Verify marker visible in ALL poses before saving
- Check reported calibration error < 3 mm
- If error > 5 mm, redo with different poses
- Compare multiple calibration runs, use best one

### **4. Validation**
- Always run diagnostic after calibration
- Test in multiple workspace locations
- Verify error < 25 mm before production use

---

## **NEXT STEPS**

1. **Run diagnostic** to measure actual error
2. **If error < 30 mm:** Try different calibrations (3C, 3D)
3. **If error > 30 mm:** Re-calibrate using proper procedure above
4. **Validate** with diagnostic script

Run diagnostic first:

```bash
cd /home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2
colcon build --symlink-install --packages-select tataro_packages
source ./install/setup.bash
ros2 run tataro_packages tataro_calibration_diagnostic
```

**Share the error values and I'll provide specific next steps.**
