"""
robot_interface.py
------------------
For *calibration* we only need to READ the robot's TCP pose after you jog it
into place, either by hand-guiding it in joint-teaching (freedrive) mode or
by jogging it in UFACTORY Studio. The wrapper exposes:

    connect / disconnect
    get_xy()  -> (Xr, Yr) from get_position()

If the xarm SDK is unavailable (e.g. running off the robot PC) the interface
degrades gracefully and you can type coordinates by hand instead.
"""

import os
import sys

# -------------------------------------------------------------------------
# If the `xarm` package is NOT installed (pip install .) and NOT already on
# your PYTHONPATH, set the path to the folder that CONTAINS the `xarm/`
# package here. Example:
#   XARM_SDK_DIR = "/home/you/Desktop/xArm-Python-SDK-master"
# Leave as None if you already ran `pip install .` inside the SDK folder.
# -------------------------------------------------------------------------
XARM_SDK_DIR = None  # e.g. "/home/you/Desktop/xArm-Python-SDK-master"

if XARM_SDK_DIR and os.path.isdir(XARM_SDK_DIR):
    sys.path.insert(0, XARM_SDK_DIR)

try:
    from xarm.wrapper import XArmAPI
    _HAS_XARM = True
except ImportError:
    XArmAPI = None
    _HAS_XARM = False


class RobotInterface:
    def __init__(self):
        self.arm = None
        self.connected = False

    @property
    def available(self):
        return _HAS_XARM

    def connect(self, ip="192.168.1.xxx"):
        if not _HAS_XARM:
            print("[Robot] xarm SDK not available - manual coordinate entry only.")
            return False
        self.arm = XArmAPI(ip)
        self.arm.motion_enable(enable=True)
        self.arm.clean_error()
        self.arm.clean_warn()
        # Start in position-control mode (mode 0). Switch to mode 2 with
        # enable_manual_mode() when you want to hand-guide/jog the arm.
        self.arm.set_mode(0)
        self.arm.set_state(0)
        self.connected = True
        print(f"[Robot] Connected @ {ip}")
        return True

    # -------------------------------------------------------------- #
    # Mode switching (equivalent of Doosan's MANUAL / AUTONOMOUS)
    # -------------------------------------------------------------- #
    def enable_manual_mode(self):
        """Joint teaching / freedrive - hand-guide the arm to jog it.

        Equivalent of using the Doosan teach pendant. Lite 6 has no
        physical pendant, so this is the closest equivalent: the arm
        becomes torque-compliant and you move it by hand (or jog it in
        UFACTORY Studio).
        """
        if not self.connected:
            print("[Robot] Not connected; cannot switch mode.")
            return False
        self.arm.set_mode(2)
        self.arm.set_state(0)
        print("[Robot] Manual (joint teaching / freedrive) mode ON.")
        return True

    def enable_auto_mode(self):
        """Back to position-control mode for code-driven moves."""
        if not self.connected:
            print("[Robot] Not connected; cannot switch mode.")
            return False
        self.arm.set_mode(0)
        self.arm.set_state(0)
        return True

    # -------------------------------------------------------------- #
    # Reading pose / joints
    # -------------------------------------------------------------- #
    def get_pose(self):
        """Return full TCP pose [X, Y, Z, Roll, Pitch, Yaw] in mm/deg, or None."""
        if not self.connected:
            return None
        code, pose = self.arm.get_position(is_radian=False)
        if code != 0 or pose is None or len(pose) < 6:
            return None
        return [float(v) for v in pose]

    def get_xy(self):
        """Return (Xr, Yr) of the current TCP pose, or None if unavailable."""
        pose = self.get_pose()
        if pose is None:
            return None
        return float(pose[0]), float(pose[1])

    def get_joints(self):
        """Return the current 6 joint angles (deg), or None."""
        if not self.connected:
            return None
        code, angles = self.arm.get_servo_angle(is_radian=False)
        if code != 0 or angles is None or len(angles) < 6:
            return None
        return [float(v) for v in angles[:6]]

    # -------------------------------------------------------------- #
    # Motion (code-driven; requires AUTO mode)
    # -------------------------------------------------------------- #
    def move_to(self, x, y, z, rx, ry, rz, speed=100, mvacc=1000):
        """Linear move to a Cartesian pose [x,y,z,roll,pitch,yaw] (mm/deg).

        Switches to position-control mode (motion is not driven by code
        while in manual/freedrive mode), then set_position(..., wait=True).
        """
        if not self.connected:
            print("[Robot] Not connected; cannot move.")
            return False

        self.enable_auto_mode()

        pos = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        print(f"[Robot] set_position -> {[round(p, 2) for p in pos]} "
              f"speed={speed} mvacc={mvacc}")
        ret = self.arm.set_position(x=pos[0], y=pos[1], z=pos[2],
                                     roll=pos[3], pitch=pos[4], yaw=pos[5],
                                     speed=speed, mvacc=mvacc, wait=True)
        print(f"[Robot] set_position returned: {ret}")
        print("[Robot] move complete.")
        return True

    def go_home(self, joint_vel=30.0, joint_acc=1000):
        """Move the robot to its predefined home position (all joints 0)."""
        if not self.connected:
            print("[Robot] Not connected; cannot home.")
            return False
        self.enable_auto_mode()
        print("[Robot] move_gohome() ...")
        self.arm.move_gohome(speed=joint_vel, mvacc=joint_acc, wait=True)
        print("[Robot] home complete.")
        return True

    def move_joints(self, joints, joint_vel=30.0, joint_acc=1000):
        """Joint move to a 6-value joint configuration (deg). For a saved home."""
        if not self.connected:
            print("[Robot] Not connected; cannot move.")
            return False
        self.enable_auto_mode()
        j = [float(v) for v in joints]
        print(f"[Robot] set_servo_angle -> {[round(v, 2) for v in j]}")
        self.arm.set_servo_angle(angle=j, speed=joint_vel, mvacc=joint_acc,
                                  is_radian=False, wait=True)
        print("[Robot] joint move complete.")
        return True

    def disconnect(self):
        if self.connected and self.arm is not None:
            try:
                self.arm.disconnect()
            except Exception:
                pass
        self.connected = False
        print("[Robot] Disconnected.")
