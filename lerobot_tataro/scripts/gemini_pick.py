#!/usr/bin/env python
"""Gemini Robotics-ER 2 plans and points, this script moves the TATARO Lite6: pick a cube, drop it in the bowl.

Gemini sees the overhead / wrist images and calls 5 functions (get_overhead_image, move_above, grasp_and_lift,
place_in_bowl, go_home). This script turns its points into robot millimetres and moves the arm:
  overhead point -> snap to the coloured blob -> TableWarp (markers 51/52/53) -> table->robot map (fitted on demos)
  wrist point    -> snap to the coloured blob -> ray from the wrist camera (hand-eye tataro_handeye_11A) -> cube top
Approach = the same camera-centred pregrasp as the recorded demos (record_handguided.py).

    conda activate lerobot
    export GEMINI_API_KEY=...            # never paste the key into code / chat
    python gemini_pick.py --task "Put the green cube in the bowl."              # real arm, Enter before every move
    python gemini_pick.py --dry-run 36                                          # no arm: replay zone demo 36
    python gemini_pick.py --local --task "Put the green cube in the bowl."      # no Gemini / API: colour + fixed steps
    python gemini_pick.py --local --dry-run 36                                  # both: no arm, no API

--dry-run EP replays record_zone001_20261110 episode EP (camera frames + recorded poses) instead of the arm and
prints where every move WOULD go next to the demo's real grasp. Run it first.

Real arm: every motion is printed and waits for Enter (type n = cancel) unless --no-confirm. KEEP THE E-STOP IN REACH.
Everything (images, Gemini's text, targets) is saved to ~/lerobot_outputs/gemini_runs/<time>/.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from record_handguided import CAM_K, CAM_QUAT, CAM_T_MM, around_base, camera_centred_pregrasp  # noqa: E402

from lerobot_robot_lite6.pose_utils import R_from_rel_rotvec  # noqa: E402
from lerobot_robot_lite6.table_warp import TableWarp  # noqa: E402

DATA = HERE.parent / "datasets" / "tataro"
MODEL = "gemini-robotics-er-2-preview"
W, H = 640, 480
TCP_Z_EXPECTED = 83.6  # mm, Gripper Lite fingertips; the dataset / this script use TCP (fingertip) poses
# Mean (estimate - demo grasp) over 29 zone demos (2026-11-18, dry-run replay): subtracted from each estimate.
# After this the OVERHEAD estimate is within 5 mm of the demo grasp along the closing axis in 29/29 demos
# (median 1.9 mm); the WRIST estimate only in 72% (hand-eye scatter), so the grasp position comes from the
# overhead and the wrist gives the cube rotation + a cross-check.
OVERHEAD_BIAS_MM = np.array([-1.8, -3.4])
WRIST_BIAS_MM = np.array([-5.0, -0.7])

# HSV ranges (OpenCV H 0-180) of the cube colours, as in place_cube.py
HUE = {"red": [(0, 8), (168, 180)], "green": [(40, 90)], "blue": [(95, 120)]}
# Saturation / value floors. The demos' cubes had S > 110, but on 2026-11-18 the green cube read H 83, S 50-100 in the
# overhead view: low floors + the cube shape / position filters (overhead_cubes) instead of strict colour.
S_MIN, V_MIN = 55, 40

DECLS = [
    {"name": "get_overhead_image",
     "description": "Returns the current 640x480 image from the fixed overhead camera (table, markers, bowl, arm). "
                    "Call it at the start, before retries, and after placing to verify.",
     "parameters": {"type": "object", "properties": {}}},
    {"name": "move_above",
     "description": "Moves the open gripper above the cube that you pointed at in the OVERHEAD image, so the cube is "
                    "near the centre of the wrist camera. Returns the WRIST camera image. Blocking.",
     "parameters": {"type": "object", "properties": {
         "y": {"type": "integer", "description": "Cube centre in the OVERHEAD image, y, normalised 0-1000"},
         "x": {"type": "integer", "description": "Cube centre in the OVERHEAD image, x, normalised 0-1000"}},
         "required": ["y", "x"]}},
    {"name": "grasp_and_lift",
     "description": "Turns the gripper to angle_deg, descends onto the cube that you pointed at in the WRIST image, "
                    "closes the gripper and lifts. Then returns to the hover position and returns a WRIST image of the "
                    "spot where the cube was, plus cube_still_on_table (colour check). Blocking.",
     "parameters": {"type": "object", "properties": {
         "y": {"type": "integer", "description": "Cube centre in the WRIST image, y, normalised 0-1000"},
         "x": {"type": "integer", "description": "Cube centre in the WRIST image, x, normalised 0-1000"},
         "angle_deg": {"type": "number", "description": "Cube rotation in the WRIST image, -45 to 45 degrees, "
                                                        "counter-clockwise positive, 0 = edges aligned with the image axes"}},
         "required": ["y", "x", "angle_deg"]}},
    {"name": "place_in_bowl",
     "description": "Carries the held cube above the bowl, lowers it and opens the gripper. Only call it after "
                    "confirming the cube is held. Blocking.",
     "parameters": {"type": "object", "properties": {}}},
    {"name": "go_home",
     "description": "Moves the arm to the HOME pose, clear of the table. Safe to call any time. Blocking.",
     "parameters": {"type": "object", "properties": {}}},
]
MOTIONS = {"move_above", "grasp_and_lift", "place_in_bowl", "go_home"}


# ---------------------------------------------------------------------------------------------------- vision
def colour_mask(rgb, colour):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = (hsv[..., i].astype(int) for i in range(3))
    m = np.zeros(h.shape, bool)
    for lo, hi in HUE[colour]:
        m |= (h >= lo) & (h <= hi)
    m &= (s > S_MIN) & (v > V_MIN)
    return cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def snap_blob(rgb, colour, uv, radius, area):
    """Nearest `colour` blob (area range) to pixel uv within radius -> (centre, minAreaRect angle deg, area) or None."""
    n, lab, st, cen = cv2.connectedComponentsWithStats(colour_mask(rgb, colour))
    best = None
    for i in range(1, n):
        if not area[0] <= st[i, 4] <= area[1]:
            continue
        d = float(np.hypot(*(cen[i] - uv)))
        if d <= radius and (best is None or d < best[0]):
            best = (d, i)
    if best is None:
        return None
    i = best[1]
    pts = np.column_stack(np.nonzero(lab == i))[:, ::-1].astype(np.float32)
    (_, _), (_, _), ang = cv2.minAreaRect(pts)
    return cen[i], float(ang), int(st[i, 4])


def norm_to_px(y, x):
    return np.array([float(x) / 1000 * W, float(y) / 1000 * H])


def colour_of(task):
    t = task.lower()
    return next((c for c in HUE if c in t), None)


# ---------------------------------------------------------------------------------------------------- geometry
class Geometry:
    def __init__(self, overhead_home_rgb):
        z = np.load(DATA / ".place_cube_cache.npz")  # cube001 demos: cube centre (table px) <-> grasp TCP (mm)
        self.T2R, _ = cv2.findHomography(z["tab"], z["grasp"], cv2.RANSAC, 15)
        self.warp = TableWarp.from_image(overhead_home_rgb)  # once, arm at HOME: markers unobstructed

    def overhead_to_robot(self, uv):
        t = self.warp.points(np.float32(uv).reshape(1, 2))
        return cv2.perspectiveTransform(t.reshape(-1, 1, 2), self.T2R).reshape(2)

    @staticmethod
    def wrist_ray(uv, p_mm, rpy, tcp_z):
        """Camera centre and ray direction (base frame, mm) of a wrist pixel, TCP pose p_mm / rpy."""
        R = Rot.from_euler("xyz", rpy)
        Rc = R * Rot.from_quat(CAM_QUAT)
        fx, fy, cx, cy = CAM_K
        cam = np.asarray(p_mm, float) + R.apply(CAM_T_MM) - R.apply([0.0, 0.0, tcp_z])
        return cam, Rc.apply([(uv[0] - cx) / fx, (uv[1] - cy) / fy, 1.0])

    def wrist_to_robot(self, uv, p_mm, rpy, tcp_z, plane_z):
        cam, ray = self.wrist_ray(uv, p_mm, rpy, tcp_z)
        return cam + ray * (plane_z - cam[2]) / ray[2]


def flange_y_angle(rpy):
    """Direction (deg, base xy) the Gripper Lite fingers close along (= flange Y axis)."""
    v = Rot.from_euler("xyz", rpy).apply([0.0, 1.0, 0.0])
    return math.degrees(math.atan2(v[1], v[0]))


def wrap(a, period):
    return (a + period / 2) % period - period / 2


# ---------------------------------------------------------------------------------------------------- robot backends
class RealArm:
    def __init__(self, ip):
        from lerobot_robot_lite6 import Lite6, Lite6Config

        self.robot = Lite6(Lite6Config(id="tataro_lite6", ip=ip, servo_on_connect=False, go_home_on_connect=True))
        print("Connecting Lite6 (arm goes HOME) + cameras ...")
        self.robot.connect()
        self.arm = self.robot._arm
        self._ready()
        self.tcp_z = float(self.arm.tcp_offset[2])
        if abs(self.tcp_z - TCP_Z_EXPECTED) > 2.0:
            self.close()
            raise SystemExit(f"Controller TCP offset z = {self.tcp_z:.1f} mm, expected {TCP_Z_EXPECTED} (fingertips). "
                             "Poses would be flange poses and the gripper would hit the table. Set the TCP offset in "
                             "UFactory Studio (same as for recording) and retry.")
        time.sleep(1.0)  # let the cameras settle

    def _ready(self):
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.3)

    def _check(self):
        if self.arm.error_code:
            raise RuntimeError(f"controller error C{self.arm.error_code}")

    def image(self, cam):
        return np.asarray(self.robot.cameras[cam].async_read()).copy()

    def pose(self):
        code, p = self.arm.get_position(is_radian=True)
        if code != 0:
            raise RuntimeError(f"get_position failed ({code})")
        return np.array(p[:3], float), list(p[3:6])

    def move(self, xyz, rpy, speed):
        code = self.arm.set_position(*xyz, *rpy, speed=speed, mvacc=1000, is_radian=True, wait=True)
        self._check()
        if code != 0:
            raise RuntimeError(f"set_position({xyz[0]:.0f},{xyz[1]:.0f},{xyz[2]:.0f}) failed ({code})")

    def move_around(self, xyz, rpy, speed, r_min):
        cur, _ = self.pose()
        for wp in around_base(cur, xyz, r_min=r_min):
            self.move(wp, rpy, speed)

    def gripper(self, closed, wait):
        from record_traj import set_gripper

        if not set_gripper(self.arm, closed, verbose=True):
            raise RuntimeError("gripper command failed")
        time.sleep(wait)
        if not closed:
            self.arm.stop_lite6_gripper()

    def home(self):
        self.robot._move_home_blocking()
        self._ready()

    def stop(self):
        try:
            self.arm.set_state(4)
            time.sleep(0.3)
            self._ready()
        except Exception:  # noqa: BLE001
            pass

    def close(self):
        try:
            self._ready()
            self.arm.open_lite6_gripper()
            time.sleep(1.0)
            self.arm.stop_lite6_gripper()
        except Exception:  # noqa: BLE001
            pass
        self.robot.disconnect()


class DryRun:
    """Replays a recorded zone demo instead of the arm: overhead = first frame, wrist = demo frame at its pregrasp
    pause, pose = the demo's recorded TCP pose there. Motions are only printed."""

    def __init__(self, episode, dataset="record_zone001_20261110"):
        from recolor_dataset import Src

        src = Src(DATA / dataset)
        rows = src.rows(episode)
        self.task = src.task(episode)
        st = np.stack(rows["observation.state"].values)
        act = np.stack(rows.action.values)
        c = int(np.argmax(act[:, 6] > 0.5))
        self.demo_grasp = act[c, :3] * 1000
        z = st[:c, 8] * 1000
        moving = np.r_[np.inf, np.linalg.norm(np.diff(st[:c, 6:9], axis=0), axis=1) * 1000]
        cand = np.where((np.abs(z - (self.demo_grasp[2] + 150)) < 5) & (moving < 0.5))[0]
        k = int(cand[len(cand) // 2]) if len(cand) else int(np.argmin(np.abs(z - (self.demo_grasp[2] + 150))))
        n_end = len(rows)
        fr_w = src.frames(episode, "wrist", n_end)
        fr_o = src.frames(episode, "overhead", n_end)
        self.frames = {"overhead": fr_o[0], "overhead_end": fr_o[-1], "wrist_hover": fr_w[k],
                       "wrist_after": fr_w[min(n_end - 1, c + 90)]}
        self.hover_p = st[k, 6:9] * 1000
        self.hover_rpy = list(R_from_rel_rotvec(st[k, 9:12]).as_euler("xyz"))
        self.home_rpy = list(R_from_rel_rotvec(st[0, 9:12]).as_euler("xyz"))
        self.tcp_z = TCP_Z_EXPECTED
        self.state = "home"
        self._p, self._rpy = st[0, 6:9] * 1000, self.home_rpy
        print(f"DRY RUN: zone demo {episode} ('{self.task}'), demo grasp at ({self.demo_grasp[0]:.0f}, "
              f"{self.demo_grasp[1]:.0f}, {self.demo_grasp[2]:.0f}) mm; demo hover frame {k} at "
              f"({self.hover_p[0]:.0f}, {self.hover_p[1]:.0f}, {self.hover_p[2]:.0f})")

    def image(self, cam):
        if cam == "overhead":
            return self.frames["overhead_end" if self.state == "placed" else "overhead"]
        return self.frames["wrist_after" if self.state == "lifted" else "wrist_hover"]

    def pose(self):
        return np.array(self._p, float), list(self._rpy)

    def move(self, xyz, rpy, speed):
        print(f"    [dry] move to ({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}) mm, yaw "
              f"{math.degrees(rpy[2]):.1f} deg @ {speed:.0f} mm/s")
        self._p, self._rpy = np.array(xyz, float), list(rpy)

    def move_around(self, xyz, rpy, speed, r_min):
        self.move(xyz, rpy, speed)

    def gripper(self, closed, wait):
        print(f"    [dry] gripper {'CLOSE' if closed else 'OPEN'}")

    def home(self):
        print("    [dry] HOME")
        self._p, self._rpy = np.array([200.0, 0, 250.0]), self.home_rpy

    def stop(self):
        pass

    def close(self):
        pass


# ---------------------------------------------------------------------------------------------------- skills
class Skills:
    def __init__(self, hw, args, colour, log):
        self.hw, self.a, self.colour, self.log = hw, args, colour, log
        self.dry = isinstance(hw, DryRun)
        self.home_p, self.home_rpy = hw.pose()
        self.geo = Geometry(hw.image("overhead"))
        self.last_overhead = None
        self.hover = None  # (p, rpy, cube estimate from overhead, wrist image)
        self.grasped = None  # (p, rpy, wrist pixel of the cube)
        self.state = "home"
        self.debug_dir = None  # set by main(): extra overhead snapshots during place_in_bowl

    def _debug_snap(self, tag):
        """Save an overhead frame (where is the held cube?) - the 2026-11-18 run lost the cube between grasp and bowl."""
        if self.debug_dir is None or self.dry:
            return
        try:
            time.sleep(self.a.settle)
            img = self.hw.image("overhead")
            cv2.imwrite(str(self.debug_dir / f"place_{tag}.jpg"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            self.log({"debug_snap": f"place_{tag}.jpg"})
        except Exception as e:  # noqa: BLE001  diagnostics must never stop the run
            print(f"    (debug snapshot {tag} failed: {e})")

    def _confirm(self, what):
        print(f"\n>>> {what}")
        if self.dry or self.a.no_confirm:
            return True
        return input("    Enter = execute, n = cancel: ").strip().lower() != "n"

    def _cancel(self):
        return {"status": "error", "error": "cancelled by operator"}, None

    # -- functions Gemini calls; each returns (response dict, image or None)
    def bowl_px(self):
        """Bowl centre in the raw overhead image (robot mm -> table px -> raw px)."""
        t = cv2.perspectiveTransform(np.float32(self.a.bin).reshape(1, 1, 2), np.linalg.inv(self.geo.T2R))
        return cv2.perspectiveTransform(t, np.linalg.inv(self.geo.warp.H)).reshape(2)

    def bowl_changed(self, img):
        """Did the bowl's appearance change since the first overhead image? (colour can't tell: the bowl is teal
        like the green cube.) True if > 2.5 % of the bowl pixels changed by > 35 grey levels."""
        c = self.bowl_px()
        yy, xx = np.mgrid[:H, :W]
        m = (xx - c[0]) ** 2 + (yy - c[1]) ** 2 <= 26 ** 2
        g0 = cv2.GaussianBlur(cv2.cvtColor(self.bowl_ref, cv2.COLOR_RGB2GRAY), (5, 5), 0).astype(int)
        g1 = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), (5, 5), 0).astype(int)
        frac = float((np.abs(g1 - g0)[m] > 35).mean())
        return frac > 0.025, frac

    def get_overhead_image(self):
        img = self.hw.image("overhead")
        self.last_overhead = img
        if getattr(self, "bowl_ref", None) is None:
            self.bowl_ref = img
            return {"status": "ok", "image": "overhead image attached"}, img
        changed, frac = self.bowl_changed(img)
        self.log({"bowl_changed": changed, "bowl_changed_fraction": round(frac, 4)})
        return {"status": "ok", "image": "overhead image attached",
                "bowl_changed_since_start": changed}, img

    def move_above(self, y, x):
        if self.last_overhead is None:
            return {"status": "error", "error": "no overhead image yet; call get_overhead_image first"}, None
        uv = norm_to_px(y, x)
        snap = snap_blob(self.last_overhead, self.colour, uv, radius=30, area=(15, 1500)) if self.colour else None
        uv_used = snap[0] if snap else uv
        g = self.geo.overhead_to_robot(uv_used) - OVERHEAD_BIAS_MM
        gz = self.a.grasp_z
        note = (f"snapped {np.hypot(*(snap[0] - uv)):.0f} px to the {self.colour} blob" if snap
                else f"no {self.colour} blob within 30 px of the point; using the point as given")
        print(f"    overhead point px ({uv[0]:.0f}, {uv[1]:.0f}) -> {note} -> cube at ({g[0]:.1f}, {g[1]:.1f}) mm")
        if self.dry:
            print(f"    [dry] demo grasp was ({self.hw.demo_grasp[0]:.1f}, {self.hw.demo_grasp[1]:.1f}) -> "
                  f"overhead estimate off by {np.hypot(*(g - self.hw.demo_grasp[:2])):.1f} mm")
        r = math.hypot(*g)
        if r < self.a.min_radius or not (-120 <= g[0] <= 450 and -370 <= g[1] <= 370):
            return {"status": "error", "error": f"cube at ({g[0]:.0f}, {g[1]:.0f}) mm is outside the reachable/safe "
                                                f"area (>= {self.a.min_radius:.0f} mm from the base)"}, None
        plan = None
        for flip in (0.0, math.pi):  # as record_handguided: the camera side must not point at the base
            rpy = [self.home_rpy[0], self.home_rpy[1], wrap(self.home_rpy[2] + flip, 2 * math.pi)]
            px, py, pz, _ = camera_centred_pregrasp((g[0], g[1], gz), rpy, self.a.pregrasp_dz, self.hw.tcp_z)
            if math.hypot(px, py) >= self.a.min_radius:
                plan = ((px, py, pz), rpy)
                break
        if plan is None:
            return {"status": "error", "error": "no safe hover pose for this cube (too close to the base)"}, None
        (px, py, pz), rpy = plan
        if not self._confirm(f"move_above: hover at ({px:.0f}, {py:.0f}, {pz:.0f}) mm, yaw {math.degrees(rpy[2]):.0f} "
                             f"deg (cube estimate ({g[0]:.0f}, {g[1]:.0f}))"):
            return self._cancel()
        if self.dry:
            print(f"    [dry] demo hover was ({self.hw.hover_p[0]:.0f}, {self.hw.hover_p[1]:.0f}, "
                  f"{self.hw.hover_p[2]:.0f}); the dry run continues from the DEMO's hover pose")
            self.hw.move(self.hw.hover_p, self.hw.hover_rpy, self.a.speed)
        else:
            self.hw.move_around((px, py, pz), rpy, self.a.speed, self.a.min_radius)
            time.sleep(self.a.settle)
        p, rpy_now = self.hw.pose()
        img = self.hw.image("wrist")
        self.hover = (p, rpy_now, g, img)
        self.state = "hover"
        self.log({"move_above": {"overhead_px": uv.tolist(), "snapped": snap is not None, "cube_mm": g.tolist(),
                                 "hover": p.tolist()}})
        return {"status": "ok", "image": "wrist image attached (taken at the hover position)"}, img

    def grasp_and_lift(self, y, x, angle_deg):
        if self.state != "hover" or self.hover is None:
            return {"status": "error", "error": "call move_above first (not at a hover position)"}, None
        p, rpy, g_over, img = self.hover
        uv = norm_to_px(y, x)
        snap = snap_blob(img, self.colour, uv, radius=90, area=(300, 60000)) if self.colour else None
        uv_used = snap[0] if snap else uv
        top = self.a.cube_top_z
        g_w = self.geo.wrist_to_robot(uv_used, p, rpy, self.hw.tcp_z, top)[:2]
        g = g_w - WRIST_BIAS_MM
        # edge direction: blob rectangle if found, else Gemini's angle (CCW in the image, y points down)
        a_img = snap[1] if snap else -float(angle_deg)
        d = np.array([math.cos(math.radians(a_img)), math.sin(math.radians(a_img))]) * 40.0
        e = self.geo.wrist_to_robot(uv_used + d, p, rpy, self.hw.tcp_z, top)[:2] - g_w
        edge = math.degrees(math.atan2(e[1], e[0]))
        delta = wrap(edge - flange_y_angle(rpy), 90.0)  # fingers close across faces: align with an edge (mod 90)
        R_new = Rot.from_euler("z", delta, degrees=True) * Rot.from_euler("xyz", rpy)
        rpy_g = list(R_new.as_euler("xyz"))
        note = (f"snapped {np.hypot(*(snap[0] - uv)):.0f} px to the {self.colour} blob (area {snap[2]} px, rect angle "
                f"{snap[1]:.0f})" if snap else f"no {self.colour} blob near the point; using it as given")
        off = float(np.hypot(*(g - g_over)))
        print(f"    wrist point px ({uv[0]:.0f}, {uv[1]:.0f}) -> {note} -> cube at ({g[0]:.1f}, {g[1]:.1f}) mm "
              f"({off:.0f} mm from the overhead estimate); edge {edge:.0f} deg -> rotate gripper {delta:+.0f} deg")
        if self.dry:
            for n_, v_ in (("overhead", g_over), ("wrist", g)):
                print(f"    [dry] demo grasp ({self.hw.demo_grasp[0]:.1f}, {self.hw.demo_grasp[1]:.1f}) -> {n_} estimate "
                      f"off by {np.hypot(*(v_ - self.hw.demo_grasp[:2])):.1f} mm")
        g = {"overhead": g_over, "wrist": g, "average": (g_over + g) / 2}[self.a.position]
        if off > self.a.max_disagree:
            return {"status": "error", "error": f"wrist and overhead positions disagree by {off:.0f} mm "
                                                f"(> {self.a.max_disagree:.0f}); wrong cube or bad point"}, None
        gz = self.a.grasp_z
        if not self._confirm(f"grasp_and_lift: cube ({g[0]:.0f}, {g[1]:.0f}) mm, gripper yaw "
                             f"{math.degrees(rpy_g[2]):.0f} deg: above at z {gz + 40:.0f} -> down to z {gz:.0f} -> close "
                             f"-> up to z {self.a.travel_z:.0f} -> back to hover to check"):
            return self._cancel()
        self.hw.move((g[0], g[1], gz + 40), rpy_g, self.a.slow)
        self.hw.move((g[0], g[1], gz), rpy_g, self.a.slow)
        self.hw.gripper(True, self.a.grip_wait)
        self.hw.move((g[0], g[1], self.a.travel_z), rpy_g, self.a.slow)
        self.state = "lifted"
        if isinstance(self.hw, DryRun):
            self.hw.state = "lifted"
        # the wrist camera can't see a held cube: look at the spot where it was instead
        self.hw.move((p[0], p[1], max(p[2], self.a.travel_z)), rpy, self.a.slow)
        if not self.dry:
            time.sleep(self.a.settle)
        after = self.hw.image("wrist")
        if not self.dry:
            # re-project the cube spot into the image from this (possibly higher) pose
            p2, rpy2 = self.hw.pose()
            R = Rot.from_euler("xyz", rpy2)
            Rc = R * Rot.from_quat(CAM_QUAT)
            cam = p2 + R.apply(CAM_T_MM) - R.apply([0.0, 0.0, self.hw.tcp_z])
            v = Rc.inv().apply(np.r_[g, top] - cam)
            fx, fy, cx, cy = CAM_K
            spot = np.array([fx * v[0] / v[2] + cx, fy * v[1] / v[2] + cy])
        else:
            spot = uv_used
        still = snap_blob(after, self.colour, spot, radius=90, area=(300, 60000)) if self.colour else None
        self.grasped = (g, rpy_g)
        self.log({"grasp_and_lift": {"wrist_px": uv.tolist(), "snapped": snap is not None, "cube_mm": g.tolist(),
                                     "yaw_deg": math.degrees(rpy_g[2]), "still_on_table": still is not None}})
        return {"status": "ok", "cube_still_on_table": still is not None,
                "image": "wrist image of the spot where the cube was, attached"}, after

    def place_in_bowl(self):
        if self.state != "lifted":
            return {"status": "error", "error": "nothing lifted; call grasp_and_lift first"}, None
        bx, by = self.a.bin
        _, rpy = self.hw.pose()
        if not self._confirm(f"place_in_bowl: travel at z {self.a.travel_z:.0f} around the base to ({bx:.0f}, "
                             f"{by:.0f}) -> down to z {self.a.bin_z:.0f} -> open -> up"):
            return self._cancel()
        self.hw.move_around((bx, by, self.a.travel_z), rpy, self.a.speed, self.a.min_radius)
        self._debug_snap("1_above_bowl")  # cube should be visible between the fingers here
        self.hw.move((bx, by, self.a.bin_z), rpy, self.a.slow)
        self._debug_snap("2_before_open")
        self.hw.gripper(False, self.a.grip_wait)
        self.hw.move((bx, by, self.a.travel_z), rpy, self.a.speed)
        self._debug_snap("3_after_open")
        self.state = "placed"
        if isinstance(self.hw, DryRun):
            self.hw.state = "placed"
        return {"status": "ok"}, None

    def go_home(self):
        if not self._confirm("go_home: lift to travel height if low, then joint move to HOME"):
            return self._cancel()
        p, rpy = self.hw.pose()
        if p[2] < self.a.travel_z - 1:
            self.hw.move((p[0], p[1], self.a.travel_z), rpy, self.a.slow)
        self.hw.home()
        held = self.state == "lifted"
        if not held:
            self.hw.gripper(False, self.a.grip_wait)
        if self.state != "placed":
            self.state = "home"
        self.hover = None
        return {"status": "ok", "gripper": "closed (still holding?)" if held else "open"}, None


# ---------------------------------------------------------------------------------------------------- --local planner
BOWL_RADIUS_MM = 75.0  # a cube this close to the bowl centre counts as "in the bowl"


def overhead_cubes(sk, img, colour):
    """`colour` blobs that look like a 3 cm cube and lie in the pick area -> list of (pixel uv, robot mm).

    In the overhead view a cube is ~10-16 px wide (area ~80-250 px), square and solid. Desk clutter beside the table
    and stains are rejected by shape and by position (robot mm: the reachable pick area or the bowl)."""
    n, _, st, cen = cv2.connectedComponentsWithStats(colour_mask(img, colour))
    bowl = np.array(sk.a.bin)
    out = []
    for i in range(1, n):
        x, y, w, h, area = st[i]
        if not (40 <= area <= 450 and max(w, h) / max(1, min(w, h)) <= 1.7 and area / (w * h) >= 0.55):
            continue
        mm = sk.geo.overhead_to_robot(cen[i]) - OVERHEAD_BIAS_MM
        in_pick_area = 100 <= mm[0] <= 450 and abs(mm[1]) <= 340 and 150 <= np.hypot(*mm) <= 460
        if in_pick_area and np.hypot(*(mm - bowl)) > BOWL_RADIUS_MM:  # the teal bowl rim can look green
            out.append((cen[i], mm))
    return out


def run_local(sk, args, colour, save, log):
    """No AI model: find the cube by colour, same functions and safety checks as the Gemini run."""
    if not colour:
        raise SystemExit("--local needs a colour in the task (red / green / blue).")
    bowl = np.array(args.bin)

    def step(name, *a):
        print(f"\n[local -> {name}{tuple(round(float(v)) for v in a) if a else '()'}]")
        res, img = getattr(sk, name)(*a)
        print(f"    -> {res}")
        log({"call": name, "args": [float(v) for v in a], **res})
        if img is not None:
            save(name, img)
        return res, img

    for attempt in range(1, 4):
        _, img = step("get_overhead_image")
        cubes = overhead_cubes(sk, img, colour)
        on_table = cubes
        print(f"    {len(cubes)} {colour} cube(s) found in the pick area")
        if not on_table:
            return (f"no {colour} cube found in the pick area. If there is one: it may be next to / under the gripper "
                    "or in the arm's shadow at HOME (x < ~240 mm near y = 0), or too pale in this light; move it further "
                    "out (or use the Gemini mode, which found such a cube on 2026-11-18)")
        uv, mm = min(on_table, key=lambda c: np.hypot(*c[1]))  # closest to the robot base, as in the prompt
        res, wimg = step("move_above", uv[1] / H * 1000, uv[0] / W * 1000)
        if res["status"] != "ok":
            step("go_home")
            return f"move_above failed: {res.get('error')}"
        w = snap_blob(wimg, colour, np.array([W / 2, H / 2]), radius=250, area=(300, 60000))
        if w is None:
            print(f"    no {colour} cube in the wrist image: retry from HOME")
            step("go_home")
            continue
        res, _ = step("grasp_and_lift", w[0][1] / H * 1000, w[0][0] / W * 1000, 0.0)
        if res["status"] != "ok":
            step("go_home")
            return f"grasp_and_lift failed: {res.get('error')}"
        if res["cube_still_on_table"]:
            print(f"    attempt {attempt}: cube still on the table -> missed, retry")
            step("go_home")
            continue
        res, _ = step("place_in_bowl")
        if res["status"] != "ok":
            step("go_home")
            return f"place_in_bowl failed: {res.get('error')}"
        step("go_home")
        _, img = step("get_overhead_image")
        changed, frac = sk.bowl_changed(img)
        return (f"SUCCESS: the bowl changed after placing ({frac * 100:.0f}% of its pixels) - the cube is in it "
                f"(attempt {attempt})" if changed else
                f"FAILED to place: the bowl looks the same as at the start ({frac * 100:.1f}% changed). The cube left "
                "the table but is not in the bowl: is it still in the gripper, or did it fall during the move?")
    return "gave up after 3 attempts"


# ---------------------------------------------------------------------------------------------------- main loop
def jpeg(rgb):
    ok, b = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
    return b.tobytes()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--task", default=None, help='e.g. "Put the green cube in the bowl." (dry run: the demo\'s colour)')
    p.add_argument("--dry-run", type=int, default=None, metavar="EP", help="no arm: replay zone demo episode EP")
    p.add_argument("--local", action="store_true",
                   help="no Gemini / no API: find the cube by colour and run the fixed sequence (same safety checks)")
    p.add_argument("--model", default=MODEL)
    p.add_argument("--prompt", default=str(HERE / "gemini_system_prompt.txt"))
    p.add_argument("--max-turns", type=int, default=25)
    p.add_argument("--ip", default="192.168.1.150")
    p.add_argument("--no-confirm", action="store_true", help="do not wait for Enter before each motion")
    p.add_argument("--grasp-z", type=float, default=14.0, help="TCP z at the grasp [mm] (demo median 13.7)")
    p.add_argument("--cube-top-z", type=float, default=25.0, help="z of the cube's top face [mm] (wrist ray plane)")
    p.add_argument("--pregrasp-dz", type=float, default=150.0, help="hover height above the grasp [mm] (demos: 150)")
    p.add_argument("--travel-z", type=float, default=200.0)
    p.add_argument("--bin", type=float, nargs=2, default=[128.0, -270.0])
    p.add_argument("--bin-z", type=float, default=60.0)
    p.add_argument("--speed", type=float, default=100.0, help="travel speed [mm/s]")
    p.add_argument("--slow", type=float, default=40.0, help="speed near the cube / bowl [mm/s]")
    p.add_argument("--grip-wait", type=float, default=1.0)
    p.add_argument("--settle", type=float, default=0.8, help="s to wait before grabbing a wrist image")
    p.add_argument("--min-radius", type=float, default=150.0, help="min distance from the base axis [mm]")
    p.add_argument("--position", default="overhead", choices=["overhead", "wrist", "average"],
                   help="which cube estimate the grasp uses (overhead = most precise on the demos)")
    p.add_argument("--max-disagree", type=float, default=25.0,
                   help="refuse a grasp if wrist and overhead cube positions differ more than this [mm]")
    args = p.parse_args()

    if not args.local and not os.environ.get("GEMINI_API_KEY"):
        raise SystemExit("Set GEMINI_API_KEY in this terminal first (export GEMINI_API_KEY=...), or use --local.")

    out = Path.home() / "lerobot_outputs" / "gemini_runs" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    events = []

    def log(ev):
        events.append({"t": round(time.time(), 2), **ev})
        (out / "log.json").write_text(json.dumps(events, indent=1, default=str))

    hw = DryRun(args.dry_run) if args.dry_run is not None else RealArm(args.ip)
    task = args.task or (f"Put the {colour_of(hw.task)} cube in the bowl." if args.dry_run is not None else None)
    if not task:
        hw.close()
        raise SystemExit('Give --task, e.g. --task "Put the green cube in the bowl."')
    colour = colour_of(task)
    print(f"Task: {task}   (colour for blob snapping: {colour or 'none - points used as given'})")
    sk = Skills(hw, args, colour, log)
    sk.debug_dir = out
    if args.local:
        n_saved = [0]

        def save(name, img):
            n_saved[0] += 1
            cv2.imwrite(str(out / f"{n_saved[0]:02d}_{name}.jpg"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        try:
            result = run_local(sk, args, colour, save, log)
            print(f"\n=== {result} ===")
            log({"result_local": result})
        except KeyboardInterrupt:
            print("\nCtrl+C: stopping the arm.")
            hw.stop()
        except Exception as e:  # noqa: BLE001
            print(f"\n[!] {e}")
            hw.stop()
            if not isinstance(hw, DryRun) and input("    Enter = go HOME now, n = leave it: ").strip().lower() != "n":
                try:
                    hw.home()
                except Exception as e2:  # noqa: BLE001
                    print(f"    go HOME failed: {e2}")
        finally:
            hw.close()
            print(f"\nRun saved in {out}")
        return
    from google import genai
    from google.genai import types

    client = genai.Client(http_options={"api_version": "v1beta"})  # preview models are only on v1beta
    config = types.GenerateContentConfig(
        system_instruction=Path(args.prompt).read_text(),
        tools=[types.Tool(function_declarations=DECLS)],
        automatic_function_calling=types.AutomaticFunctionCallingConfig(disable=True),
        temperature=0.5,
    )
    # first overhead image goes with the task: saves one request (free tier = 20 requests/day for this model)
    res0, img0 = sk.get_overhead_image()
    n_img = 1
    cv2.imwrite(str(out / "01_overhead_start.jpg"), cv2.cvtColor(img0, cv2.COLOR_RGB2BGR))
    contents = [types.Content(role="user", parts=[
        types.Part.from_text(text=task),
        types.Part.from_text(text="Overhead image (taken just now, arm at HOME), same as get_overhead_image() returns:"),
        types.Part.from_bytes(data=jpeg(img0), mime_type="image/jpeg")])]
    log({"task": task, "model": args.model, "dry_run": args.dry_run})
    try:
        for turn in range(args.max_turns):
            try:
                resp = client.models.generate_content(model=args.model, contents=contents, config=config)
            except Exception as e:  # noqa: BLE001  quota / network: don't leave the arm hovering
                print(f"\n[!] Gemini request failed: {str(e)[:300]}")
                log({"gemini_error": str(e)[:1000]})
                if sk.state not in ("home", "placed") and not isinstance(hw, DryRun):
                    if input("    Arm is not at HOME. Enter = go HOME now, n = leave it: ").strip().lower() != "n":
                        sk.a.no_confirm = True
                        sk.go_home()
                break
            cand = resp.candidates[0].content
            contents.append(cand)
            text = "".join(pt.text for pt in (cand.parts or []) if getattr(pt, "text", None) and not getattr(pt, "thought", False))
            if text.strip():
                print(f"\n[Gemini] {text.strip()}")
                log({"gemini": text.strip()})
            calls = resp.function_calls or []
            if not calls:
                break
            parts, motion_done = [], False
            for fc in calls:
                fargs = dict(fc.args or {})
                print(f"\n[Gemini -> {fc.name}({', '.join(f'{k}={v}' for k, v in fargs.items())})]")
                log({"call": fc.name, "args": fargs})
                if fc.name in MOTIONS and motion_done:
                    res, img = {"status": "error", "error": "only one motion function per turn"}, None
                elif fc.name not in {d["name"] for d in DECLS}:
                    res, img = {"status": "error", "error": f"unknown function {fc.name}"}, None
                else:
                    try:
                        res, img = getattr(sk, fc.name)(**fargs)
                    except Exception as e:  # noqa: BLE001  controller error etc.: stop the arm, tell Gemini
                        hw.stop()
                        res, img = {"status": "error", "error": str(e)}, None
                        print(f"    [!] {e}")
                motion_done |= fc.name in MOTIONS
                print(f"    -> {res}")
                log({"result": fc.name, **res})
                parts.append(types.Part.from_function_response(name=fc.name, response=res))
                if img is not None:
                    n_img += 1
                    f = out / f"{n_img:02d}_{fc.name}.jpg"
                    cv2.imwrite(str(f), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    cam = "Overhead" if fc.name == "get_overhead_image" else "Wrist"
                    parts += [types.Part.from_text(text=f"{cam} image returned by {fc.name}:"),
                              types.Part.from_bytes(data=jpeg(img), mime_type="image/jpeg")]
            contents.append(types.Content(role="user", parts=parts))
        else:
            print(f"\nStopped after {args.max_turns} turns.")
    except KeyboardInterrupt:
        print("\nCtrl+C: stopping the arm.")
        hw.stop()
    finally:
        hw.close()
        print(f"\nRun saved in {out}")


if __name__ == "__main__":
    main()
