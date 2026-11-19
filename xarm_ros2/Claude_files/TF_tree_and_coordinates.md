# TATARO Workstation - TF Tree and Coordinate System

## Complete Coordinate Frame Hierarchy

```
world (Gazebo world frame)
├── link_base (Robot base - REFERENCE FRAME)
│   ├── link1 (Joint 1)
│   │   └── link2 (Joint 2)
│   │       └── link3 (Joint 3)
│   │           └── link4 (Joint 4)
│   │               └── link5 (Joint 5)
│   │                   └── link6 (Joint 6)
│   │                       └── tool_changer (Tool flange)
│   │                           └── gripper_base (Gripper Lite base)
│   │
│   ├── camera_link (RealSense D435i housing)
│   │   ├── camera_depth_frame (Depth camera frame)
│   │   │   └── camera_depth_optical_frame (Depth optical frame)
│   │   └── camera_color_frame (RGB camera frame)
│   │       └── camera_color_optical_frame (RGB optical frame)
│   │
│   ├── table_link (Workstation table)
│   │   └── table_frame (Table center)
│   │       └── table_surface_frame (Table surface)
│   │
│   ├── mount_plate_link (Steel mounting plate)
│   │   └── mount_plate_frame (Mounting plate center)
│   │
│   └── marker_link (ArUco fiducial marker)
│       └── marker_frame (Marker center)
│
└── ground_plane (Gazebo ground reference)
```

## Reference Frame: `link_base`

**Location**: Robot mounting flange on the steel mounting plate

**Orientation**:
```
  X-axis (Red):    Forward → toward far edge of table
  Y-axis (Green):  Right → toward right side of table  
  Z-axis (Blue):   Upward → toward ceiling
```

**Origin**: (0, 0, 0)

## Environment Geometry - Absolute Positions

All positions are defined relative to the **world frame** (Gazebo origin).

### Robot Base (link_base)
- **Frame**: `link_base`
- **Position**: (0, 0, 0) - at world origin
- **Orientation**: (0, 0, 0, 1) [identity]
- **Purpose**: Robot coordinate origin

### Workstation Table
- **Frame**: `table_link` → `table_frame`
- **Position**: (0.335, 0.0, -0.005)
- **Dimensions**: 0.85m × 0.85m × 0.02m (length × width × height)
- **Material**: Light wood laminate
- **Collision**: Yes
- **Movable**: No (static)
- **Purpose**: Work surface for rehabilitation tasks

### Table Corners (in world frame)
```
Top Left:     (-0.09, -0.425, -0.005)
Top Right:    (-0.09,  0.425, -0.005)
Bottom Left:  ( 0.76, -0.425, -0.005)
Bottom Right: ( 0.76,  0.425, -0.005)
```

### Steel Mounting Plate
- **Frame**: `mount_plate_link` → `mount_plate_frame`
- **Position**: (0.335, 0.0, -0.0075)
- **Dimensions**: 0.19m × 0.12m × 0.005m (length × width × height)
- **Material**: Steel (dark metallic gray)
- **Collision**: Yes
- **Movable**: No (static)
- **Purpose**: Robot base attachment surface
- **Mount holes**: 4 (at ±0.08m x, ±0.04m y)

### ArUco Marker
- **Frame**: `marker_link` → `marker_frame`
- **Position (default)**: (0.335, 0.0, -0.0045)
- **Dimensions**: 0.15m × 0.15m × 0.001m
- **Type**: ArUco Dict4x4, ID: 0
- **Collision**: Yes (soft contact)
- **Movable**: Yes (floating joint)
- **Purpose**: Fiducial for localization and touch experiments

### RealSense D435i Camera
- **Frame hierarchy**:
  - `camera_link` (housing)
    - `camera_depth_frame` (depth sensor)
      - `camera_depth_optical_frame` (depth optical center)
    - `camera_color_frame` (RGB sensor)
      - `camera_color_optical_frame` (RGB optical center)

- **Typical position**: 0.0m, 0.0m, 0.1m relative to tool_changer
- **Orientation**: Normal mounting on end-effector
- **FOV**: 69° horizontal × 42° vertical
- **Range**: 0.1m - 10m depth

## Relative Transformations

### From link_base (robot reference)

| To Frame | Position (m) | Rotation (deg) | Notes |
|----------|--------------|----------------|-------|
| table_frame | (0.335, 0.0, 0.005) | Identity | Table center above base |
| mount_plate_frame | (0.335, 0.0, 0.0) | Identity | Mounting plate surface |
| marker_frame | (0.335, 0.0, -0.0045) | Identity | Marker on table |
| camera_color_optical_frame | (0.0, 0.0, 0.1) | Variable | Depends on mounting |

### From marker_frame

| To Frame | Distance (m) |
|----------|--------------|
| table_surface_frame | 0 (coplanar) |
| link_base | 0.335 (X), -0.0045 (Z) |
| camera_color_optical_frame | Variable (depends on camera pose) |

## Robot Workspace Relative to Table

### Workspace Bounds (link_base frame)
- **X (forward/backward)**:
  - Min: -0.2m (behind robot)
  - Max: 0.9m (far edge of table + beyond)
  
- **Y (left/right)**:
  - Min: -0.5m (left of table)
  - Max: 0.5m (right of table)
  
- **Z (up/down)**:
  - Min: -0.1m (below table)
  - Max: 0.8m (above table)

### Table Workspace (safe reaching zone)
- **X**: 0.2m to 0.7m (reachable table area)
- **Y**: -0.3m to 0.3m (table width with clearance)
- **Z**: -0.005m to 0.05m (on and above table)

## Transformation Chain for Marker Touching

```
Camera detects marker
        ↓
marker_pose_camera_frame
        ↓
[Transform: camera_color_optical_frame → link_base]
        ↓
marker_pose_base_frame (link_base)
        ↓
Approach pose = marker_pose + (0, 0, 0.05)
        ↓
Move to approach
        ↓
Descent to marker_pose
        ↓
Touch marker
        ↓
Retract to approach
```

## Dynamic TF Updates

### Static Transforms (published once on startup)
- `world` → `link_base`
- `link_base` → `table_frame`
- `link_base` → `mount_plate_frame`
- `link_base` → `camera_link`

### Dynamic Transforms (updated continuously)
- Joint transforms: `link_base` → `link1` → `link2` ... `link6`
- Marker transform: `link_base` → `marker_frame` (published at 30 Hz)
- Camera frames: Published at sensor rate

## Checking Coordinate Systems

### In RViz

1. **Enable TF display**:
   - Displays panel → TF → Enable
   - Shows axes for each frame (X=Red, Y=Green, Z=Blue)

2. **Change fixed frame**:
   - Global Options → Fixed Frame → link_base
   - Reorients all visualizations relative to robot base

3. **Verify geometry alignment**:
   - Table should extend in +X (forward) and ±Y (sides)
   - Marker should be on table surface (-Z 0.0045m from link_base)
   - Camera should point toward table

### Command Line

**List all frames**:
```bash
ros2 run tf2_ros tf2_monitor
```

**Check specific transform**:
```bash
ros2 run tf2_ros tf2_echo link_base marker_frame
```

Output shows:
```
Translation: [0.335, 0.0, -0.0045]
Rotation: in Quaternion [x, y, z, w]
```

**Get transform at specific time**:
```bash
ros2 run tf2_ros tf2_echo link_base marker_frame --echo-rate 5
```

## Physics Interaction Frames

### Collision Geometry
```
Table (box):
  Dimensions: 0.85 × 0.85 × 0.02
  Center: table_frame
  Friction: μ = 0.6

Mounting plate (box):
  Dimensions: 0.19 × 0.12 × 0.005
  Center: mount_plate_frame
  Friction: μ = 0.7

Marker (box):
  Dimensions: 0.15 × 0.15 × 0.001
  Center: marker_frame
  Friction: μ = 0.3 (low, can slide)
```

### Contact Dynamics
- **Contact stiffness (kp)**: 1000 N/m (table), 1500 N/m (mounting plate)
- **Contact damping (kd)**: 100 N·s/m
- **Surface layer**: 0.001m (penetration tolerance)
- **Gravity**: 9.81 m/s² downward

## RealSense Camera Frames

### Depth Camera
- **Name**: `camera_depth_frame` / `camera_depth_optical_frame`
- **Resolution**: 640 × 480
- **Depth range**: 0.1 - 10.0 m
- **FPS**: 30 Hz
- **Intrinsics**: Calibrated for D435i

### RGB/Color Camera  
- **Name**: `camera_color_frame` / `camera_color_optical_frame`
- **Resolution**: 640 × 480
- **FOV**: 69° H × 42° V
- **FPS**: 30 Hz
- **Intrinsics**: Calibrated for D435i

### Optical Frame Convention
RealSense uses OpenCV convention:
- **X**: right
- **Y**: down  
- **Z**: forward (into image)

Different from robot frame! Requires transformation.

## Visualization Axes Convention

In RViz and Gazebo:
```
  Z ↑ (Blue)
    |
    +--→ X (Red) - forward
   /
  Y (Green) - right
```

This matches a right-hand coordinate system:
- **X × Y = Z** (right-hand rule)
- Standard for robotics

## Troubleshooting Frame Issues

### Frame not found error

**Check 1**: Is frame being published?
```bash
ros2 run tf2_ros tf2_monitor | grep marker_frame
```

**Check 2**: Check frame spelling and case (case-sensitive):
```bash
ros2 run tf2_ros tf2_monitor | grep -i marker
```

**Check 3**: Verify parent frame exists:
```bash
ros2 run tf2_ros tf2_echo world marker_frame
```

### Discontinuous transforms

**Symptom**: Frame jumps around, not smooth

**Cause**: Multiple publishers of same transform

**Fix**: Ensure only one node publishes each transform

### Incorrect transforms

**Symptom**: Marker appears at wrong location

**Check**: 
1. Verify expected position: `ros2 run tf2_ros tf2_echo link_base marker_frame`
2. Check position in Gazebo GUI
3. Validate in RViz with correct fixed frame selected

## Marker Movement Bounds

When repositioning marker within workspace:

```
Safe X range: 0.2 to 0.7 m    (reachable table area)
Safe Y range: -0.3 to 0.3 m   (table width)
Safe Z range: -0.005 to -0.005 m (on table surface)

Keep marker centered on table!
```

## Reference Equations

### Transform Composition
```
T_A_C = T_A_B × T_B_C
(frame A to C = A to B × B to C)
```

### Position in link_base from camera detection

```
p_base = T_base_camera × p_camera
```

Where:
- `p_camera`: marker pose detected by D435i
- `T_base_camera`: camera to base transformation (from TF)
- `p_base`: marker pose in robot reference frame

### Approach pose calculation

```
p_approach = p_marker + [0, 0, h_approach]
           = [0.335, 0.0, -0.0045] + [0, 0, 0.05]
           = [0.335, 0.0, 0.0045]
```

## Summary Table

| Frame | Type | Parent | Fixed | Position | Notes |
|-------|------|--------|-------|----------|-------|
| world | Gazebo | - | Yes | (0,0,0) | Ground reference |
| link_base | Robot | world | Yes | (0,0,0) | Robot origin |
| table_frame | Env | link_base | Yes | (0.335,0,-0.005) | Table center |
| mount_plate_frame | Env | link_base | Yes | (0.335,0,-0.0075) | Mount surface |
| marker_frame | Env | link_base | No | (0.335,0,-0.0045) | Marker (movable) |
| camera_color_optical_frame | Sensor | tool_changer | Yes | Variable | Camera (30Hz) |
| link1-link6 | Robot | cascading | Dynamic | - | Joint frames |
