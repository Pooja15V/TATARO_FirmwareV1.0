#!/usr/bin/env python3
"""
TCP Position Control GUI — TATARO / Lite6 + UFLite Gripper

Run with:  python3  TATRO_PositionControl_GUI.py
           (requires python3.10 — lite6_kin.so is a CPython 3.10 extension)

Kinematic chain (verified from URDF + mesh bounds):
  link6  → joint_eef  (xyz 0 0 0)      → link_eef
         → gripper_fix (xyz 0 0 0)      → uflite_gripper_link   (Z=0, same as link_eef)
         → joint_tcp   (xyz 0 0 0.0836) → link_tcp              <── user enters this frame

  Camera stand is a side bracket (X+67 mm from axis) — NOT in the axial chain.
  gripper shell.stl starts at Z=0 (confirmed), so no hidden plate offset.
  Total link6 → link_tcp = 83.6 mm along tool Z.

lite6_kin.computeIK() solves for link_eef (= link6 flange).
This GUI subtracts TOOL_Z_OFFSET_MM (83.6 mm) along the current tool Z-axis
to convert link_tcp pose → link_eef pose before calling IK.
"""

import sys
import os

# lite6_kin.cpython-310 lives in TATRO_Robot_Firmware; add it to the path
_FIRMWARE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', '..', '..', '..', '..', '..', '..', '..', '..',
    'TATRO', 'TATRO_Program_files', 'TATRO_Robot_Firmware'
)
_FIRMWARE_DIR = os.path.normpath(
    os.path.join('/home', os.environ.get('USER', 'pooja'),
                 'TATRO', 'TATRO_Program_files', 'TATRO_Robot_Firmware')
)
if _FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, _FIRMWARE_DIR)

import math
import tkinter as tk
from tkinter import ttk
import numpy as np
from lite6_kin import Lite6Kinematics

robot = Lite6Kinematics()

# Physical link6 → link_tcp offset:
#   camera bracket plate (joint_camera_stand, realsense_d435i.urdf.xacro):  3.0 mm
#   uflite_gripper_link → link_tcp (joint_tcp, uflite_gripper.urdf.xacro): 83.6 mm
#   total:                                                                  86.6 mm
TOOL_Z_OFFSET_MM = 86.6

JOINT_LIMITS = [
    (-360, 360),   # J1
    (-300, 150),   # J2
    (-3.5, 300),   # J3
    (-360, 360),   # J4
    (-124, 124),   # J5
    (-360, 360),   # J6
]

# Workspace limits for link_tcp (with gripper extended, Z floor is lower than bare flange)
POSITION_LIMITS    = {'X': (-440, 440), 'Y': (-440, 440), 'Z': (0, 683.5)}
ORIENTATION_LIMITS = {'Roll': (-180, 180), 'Pitch': (-180, 180), 'Yaw': (-180, 180)}

# Initial pose in link_tcp coordinates
INITIAL_TCP_XYZ = [88.3, 0.6, 72.51]
INITIAL_TCP_RPY = [180.0, 0.0, 0.0]   # degrees


# ── Maths ─────────────────────────────────────────────────────────────────────

def rad2deg(radians):
    return np.rad2deg(radians).tolist()


def deg2rad(degrees):
    return np.deg2rad(degrees).tolist()


def clamp_joint_angles(joint_angles_deg):
    return [
        max(min(a, JOINT_LIMITS[i][1]), JOINT_LIMITS[i][0])
        for i, a in enumerate(joint_angles_deg)
    ]


def is_within_workspace(x, y, z):
    return (
        POSITION_LIMITS['X'][0] <= x <= POSITION_LIMITS['X'][1] and
        POSITION_LIMITS['Y'][0] <= y <= POSITION_LIMITS['Y'][1] and
        POSITION_LIMITS['Z'][0] <= z <= POSITION_LIMITS['Z'][1]
    )


def rpy_to_rotation_matrix(roll_rad, pitch_rad, yaw_rad):
    """ZYX extrinsic (xArm convention): R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    cr, sr = math.cos(roll_rad),  math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad),   math.sin(yaw_rad)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp],  [0,  1,  0], [-sp, 0, cp]])
    Rx = np.array([[1, 0,   0],  [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def tcp_to_flange(tcp_xyzrpy_mm_deg):
    """
    Convert link_tcp pose [x, y, z mm | roll, pitch, yaw deg] →
    link_eef pose [x, y, z mm | roll, pitch, yaw rad] ready for computeIK().

    link_tcp = link_eef + TOOL_Z_OFFSET_MM * tool_Z_axis
    so: link_eef = link_tcp - TOOL_Z_OFFSET_MM * tool_Z_axis
    """
    x, y, z, roll_deg, pitch_deg, yaw_deg = tcp_xyzrpy_mm_deg
    roll_rad  = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad   = math.radians(yaw_deg)

    R = rpy_to_rotation_matrix(roll_rad, pitch_rad, yaw_rad)
    tool_z = R[:, 2]                                # tool Z-axis in base frame

    p_tcp    = np.array([x, y, z])
    p_flange = p_tcp - TOOL_Z_OFFSET_MM * tool_z   # step back to flange

    return [float(p_flange[0]), float(p_flange[1]), float(p_flange[2]),
            roll_rad, pitch_rad, yaw_rad]


# ── GUI ───────────────────────────────────────────────────────────────────────

def run(robot_main):
    root = tk.Tk()
    root.title("TATARO — link_tcp Position Control")
    root.resizable(False, False)

    BG       = '#1e1e2e'
    FG       = '#cdd6f4'
    ACCENT   = '#89b4fa'
    GREEN    = '#a6e3a1'
    RED      = '#f38ba8'
    ORANGE   = '#fab387'
    ENTRY_BG = '#313244'
    SEP_CLR  = '#45475a'
    F_BODY   = ('Consolas', 11)
    F_HEAD   = ('Consolas', 12, 'bold')
    F_TINY   = ('Consolas', 9)

    root.configure(bg=BG)
    style = ttk.Style()
    style.theme_use('clam')
    style.configure('TFrame',      background=BG)
    style.configure('TLabel',      background=BG, foreground=FG,      font=F_BODY)
    style.configure('Head.TLabel', background=BG, foreground=ACCENT,  font=F_HEAD)
    style.configure('Hint.TLabel', background=BG, foreground=SEP_CLR, font=F_TINY)

    pad = {'padx': 10, 'pady': 4}

    outer = ttk.Frame(root, padding=16)
    outer.grid(row=0, column=0)

    def hsep(row):
        tk.Frame(outer, bg=SEP_CLR, height=1).grid(
            row=row, column=0, columnspan=3, sticky='ew', pady=6)

    def make_entry(var):
        return tk.Entry(outer, textvariable=var, width=14,
                        bg=ENTRY_BG, fg=FG, insertbackground=FG,
                        relief='flat', font=F_BODY, bd=4)

    # ── Title ──────────────────────────────────────────────────────────────
    ttk.Label(outer, text="link_tcp Position Control", style='Head.TLabel').grid(
        row=0, column=0, columnspan=3, pady=(0, 2))
    ttk.Label(outer, text=f"(tool offset {TOOL_Z_OFFSET_MM} mm along tool-Z  →  IK solved for link_eef)",
              style='Hint.TLabel').grid(row=1, column=0, columnspan=3, pady=(0, 6))

    # ── Position ───────────────────────────────────────────────────────────
    hsep(2)
    ttk.Label(outer, text="Position  (link_tcp, mm)", style='Head.TLabel').grid(
        row=3, column=0, columnspan=3, sticky='w')

    pos_vars = {}
    for i, (label, lim, init) in enumerate([
        ('X', POSITION_LIMITS['X'],  INITIAL_TCP_XYZ[0]),
        ('Y', POSITION_LIMITS['Y'],  INITIAL_TCP_XYZ[1]),
        ('Z', POSITION_LIMITS['Z'],  INITIAL_TCP_XYZ[2]),
    ]):
        ttk.Label(outer, text=f"  {label}").grid(row=4+i, column=0, sticky='e', **pad)
        var = tk.StringVar(value=str(init))
        make_entry(var).grid(row=4+i, column=1, sticky='w', **pad)
        ttk.Label(outer, text=f"{lim[0]} … {lim[1]} mm",
                  style='Hint.TLabel').grid(row=4+i, column=2, sticky='w')
        pos_vars[label] = var

    # ── Orientation ────────────────────────────────────────────────────────
    hsep(7)
    ttk.Label(outer, text="Orientation  (link_tcp, °)", style='Head.TLabel').grid(
        row=8, column=0, columnspan=3, sticky='w')

    ori_vars = {}
    for i, (label, lim, init) in enumerate([
        ('Roll',  ORIENTATION_LIMITS['Roll'],  INITIAL_TCP_RPY[0]),
        ('Pitch', ORIENTATION_LIMITS['Pitch'], INITIAL_TCP_RPY[1]),
        ('Yaw',   ORIENTATION_LIMITS['Yaw'],   INITIAL_TCP_RPY[2]),
    ]):
        ttk.Label(outer, text=f"  {label}").grid(row=9+i, column=0, sticky='e', **pad)
        var = tk.StringVar(value=str(init))
        make_entry(var).grid(row=9+i, column=1, sticky='w', **pad)
        ttk.Label(outer, text=f"{lim[0]} … {lim[1]} °",
                  style='Hint.TLabel').grid(row=9+i, column=2, sticky='w')
        ori_vars[label] = var

    # ── Buttons ────────────────────────────────────────────────────────────
    hsep(12)
    btn_row = ttk.Frame(outer)
    btn_row.grid(row=13, column=0, columnspan=3, pady=4)

    def make_btn(text, color, col):
        b = tk.Button(btn_row, text=text, bg=color, fg='#1e1e2e',
                      activebackground=color, activeforeground='#1e1e2e',
                      font=('Consolas', 10, 'bold'), relief='flat',
                      padx=10, pady=6, cursor='hand2', width=15)
        b.grid(row=0, column=col, padx=6)
        return b

    btn_send = make_btn("Send Position", ACCENT,  0)
    btn_grip = make_btn("Open Gripper",  GREEN,   1)
    btn_init = make_btn("Go to Initial", ORANGE,  2)

    # ── Status bar ─────────────────────────────────────────────────────────
    hsep(14)
    sf = ttk.Frame(outer)
    sf.grid(row=15, column=0, columnspan=3, sticky='ew')
    ttk.Label(sf, text="Status:", font=('Consolas', 10, 'bold'),
              background=BG, foreground=FG).grid(row=0, column=0, sticky='e', padx=(0, 6))
    status_var = tk.StringVar(value="Ready")
    status_lbl = tk.Label(sf, textvariable=status_var, bg=BG, fg=GREEN,
                          font=F_BODY, anchor='w', width=48)
    status_lbl.grid(row=0, column=1, sticky='w')

    # ── Flange pose readout (for debugging / verification) ─────────────────
    hsep(16)
    ttk.Label(outer, text="Flange (link_eef) sent to IK", style='Head.TLabel').grid(
        row=17, column=0, columnspan=3, sticky='w')
    flange_var = tk.StringVar(value="X=--  Y=--  Z=--  R=--  P=--  Y=--")
    tk.Label(outer, textvariable=flange_var, bg=ENTRY_BG, fg=FG,
             font=('Consolas', 10), anchor='w', padx=6, pady=3).grid(
        row=18, column=0, columnspan=3, sticky='ew', padx=10, pady=2)

    # ── Joint angles ───────────────────────────────────────────────────────
    hsep(19)
    ttk.Label(outer, text="Joint Angles (°)", style='Head.TLabel').grid(
        row=20, column=0, columnspan=3, sticky='w')

    joint_vars = [tk.StringVar(value="  --  ") for _ in range(6)]
    jf = ttk.Frame(outer)
    jf.grid(row=21, column=0, columnspan=3, sticky='w', pady=(4, 4))
    for i in range(6):
        r, c = divmod(i, 3)
        ttk.Label(jf, text=f"J{i+1}:", font=('Consolas', 10, 'bold'),
                  background=BG, foreground=ACCENT).grid(
            row=r, column=c*2, sticky='e', padx=(8, 2), pady=2)
        tk.Label(jf, textvariable=joint_vars[i], bg=ENTRY_BG, fg=FG,
                 font=('Consolas', 10), width=9, anchor='e', relief='flat').grid(
            row=r, column=c*2+1, sticky='w', padx=(0, 10), pady=2)

    # ── Callbacks ──────────────────────────────────────────────────────────
    gripper_state = {'is_open': False}

    def set_status(msg, ok=True):
        status_var.set(msg)
        status_lbl.config(fg=GREEN if ok else RED)

    def parse_pose():
        try:
            xyz = [float(pos_vars[k].get()) for k in ('X', 'Y', 'Z')]
        except ValueError:
            set_status("Invalid position — enter numeric values", ok=False)
            return None, None
        try:
            rpy = [float(ori_vars[k].get()) for k in ('Roll', 'Pitch', 'Yaw')]
        except ValueError:
            set_status("Invalid orientation — enter numeric values", ok=False)
            return None, None
        return xyz, rpy

    def compute_and_send(tcp_xyz, tcp_rpy_deg, label="Sent"):
        tcp_pose = tcp_xyz + tcp_rpy_deg            # [x,y,z, roll,pitch,yaw(deg)]
        flange_pose = tcp_to_flange(tcp_pose)       # [x,y,z, roll,pitch,yaw(rad)]

        fx, fy, fz = flange_pose[:3]
        fr, fp, fw = [math.degrees(a) for a in flange_pose[3:]]
        flange_var.set(
            f"X={fx:+.1f}  Y={fy:+.1f}  Z={fz:+.1f}  "
            f"R={fr:+.1f}°  P={fp:+.1f}°  Y={fw:+.1f}°")

        ik_joints_rad = robot.computeIK(flange_pose)
        if ik_joints_rad is None or len(ik_joints_rad) != 6:
            set_status("IK failed — pose is unreachable", ok=False)
            for v in joint_vars:
                v.set("  --  ")
            return

        ik_joints_deg = clamp_joint_angles(rad2deg(ik_joints_rad))
        for i, v in enumerate(joint_vars):
            v.set(f"{ik_joints_deg[i]:+.2f}")

        set_status(
            f"{label}  tcp X={tcp_xyz[0]:.1f}  Y={tcp_xyz[1]:.1f}  Z={tcp_xyz[2]:.1f} mm",
            ok=True)
        print(f"[TCP GUI] {label}")
        print(f"  link_tcp  : {[round(v, 2) for v in tcp_xyz]}  RPY {tcp_rpy_deg}")
        print(f"  link_eef  : {[round(v, 2) for v in flange_pose[:3]]}")
        print(f"  joints(°) : {[round(a, 2) for a in ik_joints_deg]}")
        robot_main.apply_angles(ik_joints_deg)

    def send_position():
        tcp_xyz, tcp_rpy_deg = parse_pose()
        if tcp_xyz is None:
            return
        if not is_within_workspace(*tcp_xyz):
            set_status(
                f"Out of workspace!  "
                f"Z must be {POSITION_LIMITS['Z'][0]}…{POSITION_LIMITS['Z'][1]} mm",
                ok=False)
            return
        compute_and_send(tcp_xyz, tcp_rpy_deg, label="Sent")

    def toggle_gripper():
        if gripper_state['is_open']:
            robot_main.close_gripper()
            btn_grip.config(text="Open Gripper", bg=GREEN)
            gripper_state['is_open'] = False
            set_status("Gripper closed", ok=True)
        else:
            robot_main.open_gripper()
            btn_grip.config(text="Close Gripper", bg=RED)
            gripper_state['is_open'] = True
            set_status("Gripper opened", ok=True)

    def go_to_initial():
        for i, k in enumerate(('X', 'Y', 'Z')):
            pos_vars[k].set(str(INITIAL_TCP_XYZ[i]))
        for i, k in enumerate(('Roll', 'Pitch', 'Yaw')):
            ori_vars[k].set(str(INITIAL_TCP_RPY[i]))
        compute_and_send(INITIAL_TCP_XYZ, INITIAL_TCP_RPY, label="Initial")

    btn_send.config(command=send_position)
    btn_grip.config(command=toggle_gripper)
    btn_init.config(command=go_to_initial)

    root.mainloop()


# ── Entry point — connects to the real robot ──────────────────────────────────
if __name__ == '__main__':
    import time
    from xarm.wrapper import XArmAPI

    ROBOT_IP      = '192.168.1.150'
    ANGLE_SPEED   = 20    # deg/s
    ANGLE_ACC     = 1000  # deg/s²

    class RobotMain:
        def __init__(self, arm):
            self._arm = arm
            arm.clean_warn()
            arm.clean_error()
            arm.motion_enable(True)
            arm.set_mode(0)
            arm.set_state(0)
            time.sleep(0.5)
            arm.set_tcp_load(0.35, [0, 0, 40])
            print(f"[ROBOT] Connected — state={arm.state}  error={arm.error_code}")

        def apply_angles(self, joint_angles_deg):
            code = self._arm.set_servo_angle(
                angle=joint_angles_deg,
                speed=ANGLE_SPEED, mvacc=ANGLE_ACC,
                wait=True, radius=0.0)
            if code != 0:
                print(f"[ROBOT] set_servo_angle error code={code}")

        def open_gripper(self):
            code = self._arm.open_lite6_gripper()
            if code != 0:
                print(f"[ROBOT] open_lite6_gripper error code={code}")
            else:
                self._arm.set_tcp_load(0.277, [0, 0, 30])

        def close_gripper(self):
            code = self._arm.close_lite6_gripper()
            if code != 0:
                print(f"[ROBOT] close_lite6_gripper error code={code}")
            else:
                self._arm.set_tcp_load(0.35, [0, 0, 40])

    print(f"[ROBOT] Connecting to {ROBOT_IP} …")
    arm        = XArmAPI(ROBOT_IP, baud_checkset=False)
    robot_main = RobotMain(arm)
    run(robot_main)
    arm.disconnect()
