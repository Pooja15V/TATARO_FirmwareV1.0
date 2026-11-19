#!/usr/bin/env python
"""Record pick-and-place episodes: YOU show the grasp pose by hand, the ARM does the recorded motion.

Per episode:
  1. arm goes HOME, gripper opens
  2. put the cube anywhere reachable (new spot + new rotation every episode = useful data)
  3. arm goes compliant (manual mode): drag the gripper down around the cube, fingers either
     side of it, at the height you want it gripped, then press Enter
  4. arm lifts straight up, goes HOME (not recorded), you confirm the cube colour
  5. RECORDED: HOME -> pregrasp 150 mm up, offset so the cube is centred in the wrist camera (1 s pause) -> over cube -> down -> close -> lift -> above bin -> down -> open -> up
No hands in the images, every episode has a different cube position, ~15 s each.

    conda activate lerobot
    python record_handguided.py                       # new dataset tataro/record_cube001_20261106
    python record_handguided.py --resume --num-episodes 10

Multi-cube scenes (--multi, for language-conditioned policies such as SmolVLA): put 2-3 cubes of different
colours on the table, type which colours are there, the script names the TARGET (the colour with the fewest
episodes so far) and you hand-guide the gripper to THAT cube. The other cubes stay in view as distractors.

    python record_handguided.py --multi --num-episodes 10 --display     # new dataset tataro/record_multicube001_<date>
    python record_handguided.py --multi --resume --repo-id tataro/record_multicube001_<date> --num-episodes 10

Keys during the recorded motion: Right arrow = stop early, Left arrow = discard, Esc = stop.
Ctrl+C anywhere = stop, unfinished episode is discarded. KEEP THE E-STOP IN REACH.
action(t) = measured TCP pose + gripper at t + --lookahead frames (same as record_traj.py).
"""

import argparse
import math
import random
import re
import time
from collections import deque
from pathlib import Path

import numpy as np

from lerobot.datasets import LeRobotDataset
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame, combine_feature_dicts, hw_to_dataset_features
from lerobot_robot_lite6 import Lite6, Lite6Config

from record_traj import COLORS, DEFAULT_TASK, Keys, detect_cube_color, set_gripper

# Wrist D435i: hand-eye link_eef -> camera_color_optical (xarm_ros2/Calibration/tataro_handeye_11A.calib)
# and colour intrinsics at 640x480 (read from the camera 2026-11-06).
CAM_T_MM = np.array([64.42, -29.66, 27.05])
CAM_QUAT = [0.008566553471248385, -0.0026546086904837285, 0.7144087940688149, 0.6996710957095956]
CAM_K = (605.41, 605.25, 321.59, 246.56)  # fx, fy, cx, cy


def camera_centred_pregrasp(grasp_mm, rpy, dz, tcp_z, pixel=(320.0, 240.0), max_reach=400.0):
    """TCP xy (same orientation, z = grasp + dz) from which the wrist camera sees the grasp point at `pixel`.

    The camera sits ~65 mm beside the flange, so straight above the cube the cube is out of view.
    Returns (x, y, z, predicted pixel of the cube).
    """
    from scipy.spatial.transform import Rotation as Rot

    R = Rot.from_euler("xyz", rpy)
    Rc = R * Rot.from_quat(CAM_QUAT)
    fx, fy, cx, cy = CAM_K
    cube = np.asarray(grasp_mm, dtype=float)
    ray = Rc.apply([(pixel[0] - cx) / fx, (pixel[1] - cy) / fy, 1.0])  # camera -> cube direction (base frame)
    cam_from_tcp = R.apply(CAM_T_MM) - R.apply([0.0, 0.0, tcp_z])     # camera position - TCP position
    z = cube[2] + dz
    s = (cube[2] - (z + cam_from_tcp[2])) / ray[2]
    p = cube - s * ray - cam_from_tcp
    p[2] = z
    max_reach = max(max_reach, math.hypot(cube[0], cube[1]))  # never further out than the cube itself
    r = math.hypot(p[0], p[1])
    if r > max_reach:  # keep it reachable: pull the xy offset back towards the cube
        k = 1.0
        for _ in range(30):
            k *= 0.9
            q = cube + k * (p - cube)
            if math.hypot(q[0], q[1]) <= max_reach:
                break
        p = np.array([q[0], q[1], z])
    v = Rc.inv().apply(cube - (p + cam_from_tcp))
    return p[0], p[1], p[2], (fx * v[0] / v[2] + cx, fy * v[1] / v[2] + cy)


def around_base(p0, p1, r_min=200.0, step_deg=15.0):
    """Waypoints from p0 to p1 (mm) that swing around the base instead of cutting across it.

    A straight Cartesian line between two points on opposite sides of the robot passes over the base,
    where the Lite6 folds into itself (self-collision). Interpolate in polar coordinates (angle, radius,
    z) and keep the radius >= r_min. Returns the points after p0 (the last one is p1).
    """
    r0, r1 = math.hypot(p0[0], p0[1]), math.hypot(p1[0], p1[1])
    a0, a1 = math.atan2(p0[1], p0[0]), math.atan2(p1[1], p1[0])
    da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
    n = max(1, int(math.ceil(abs(math.degrees(da)) / step_deg)))
    if n == 1 and min(r0, r1) >= r_min:
        return [tuple(p1)]
    pts = []
    for i in range(1, n + 1):
        t = i / n
        r = max(r_min, r0 + (r1 - r0) * t) if i < n else r1
        a = a0 + da * t
        pts.append((r * math.cos(a), r * math.sin(a), p0[2] + (p1[2] - p0[2]) * t))
    if r0 < r_min:  # start close to the base: first move straight out
        pts.insert(0, (r_min * math.cos(a0), r_min * math.sin(a0), p0[2]))
    pts[-1] = tuple(p1)
    return pts


def make_zone_overlay(overhead_rgb, zone):
    """Returns f(overhead) -> copy with the zone square and the cube position drawn (for the live view only).

    Uses the table markers (detected once, now) and the demo-fitted table->robot map from place_cube.py.
    Returns None (with a message) if the markers aren't visible.
    """
    import cv2

    from lerobot_robot_lite6.table_warp import MarkersNotFound, TableWarp
    from place_cube import DATA as REF, blob, demo_table

    try:
        warp = TableWarp.from_image(overhead_rgb)
    except MarkersNotFound as e:
        print(f"[!] zone overlay disabled: {e}")
        return None
    _, tab, grasp = demo_table(REF.parent / ".place_cube_cache.npz")
    t2r, _ = cv2.findHomography(tab, grasp, cv2.RANSAC, 15)
    r2raw = np.linalg.inv(warp.H) @ np.linalg.inv(t2r)  # robot mm -> raw overhead px
    zx, zy, zh = zone
    sq = np.float32([[zx - zh, zy - zh], [zx + zh, zy - zh], [zx + zh, zy + zh], [zx - zh, zy + zh]])
    poly = cv2.perspectiveTransform(sq.reshape(-1, 1, 2), r2raw).astype(np.int32)

    def draw(img):
        vis = np.ascontiguousarray(img).copy()
        cv2.polylines(vis, [poly], True, (255, 0, 255), 2)
        b = blob(img)
        if b is not None:
            xy = cv2.perspectiveTransform(warp.points(b).reshape(1, 1, 2), t2r)[0, 0]
            inside = abs(xy[0] - zx) <= zh and abs(xy[1] - zy) <= zh
            col = (0, 255, 0) if inside else (255, 0, 0)
            cv2.drawMarker(vis, (int(b[0]), int(b[1])), col, cv2.MARKER_CROSS, 18, 2)
            cv2.putText(vis, f"cube ({xy[0]:.0f}, {xy[1]:.0f}) mm  {'INSIDE' if inside else 'OUTSIDE'} zone",
                        (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        return vis

    print(f"Zone overlay: see 'zone_view' in the Rerun viewer (magenta square = zone)")
    return draw


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", default="192.168.1.150")
    p.add_argument("--task", default=DEFAULT_TASK, help="{color} is replaced by the cube colour")
    p.add_argument("--color", default="ask", choices=["ask", "red", "green", "blue"])
    p.add_argument("--repo-id", default="tataro/record_cube001_20261106")
    p.add_argument("--root", default=None, help="default: ~/lerobot_data/<repo-id>")
    p.add_argument("--resume", action="store_true", help="append to an existing dataset")
    p.add_argument("--num-episodes", type=int, default=1)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--lookahead", type=int, default=3)
    p.add_argument("--bin", type=float, nargs=2, default=[128.0, -270.0], metavar=("X", "Y"), help="bin centre [mm]")
    p.add_argument("--bin-z", type=float, default=60.0, help="release height [mm] (demos released at ~28)")
    p.add_argument("--travel-z", type=float, default=200.0, help="height for moving between cube and bin [mm]")
    p.add_argument("--pregrasp-dz", type=float, default=150.0,
                   help="pregrasp height above the grasp [mm]: the wrist camera sees the cube from here")
    p.add_argument("--pregrasp-pause", type=float, default=1.0, help="s to hold still at the pregrasp pose")
    p.add_argument("--pregrasp-pixel", type=float, nargs=2, default=[320.0, 240.0], metavar=("U", "V"),
                   help="where the cube should appear in the 640x480 wrist image at the pregrasp")
    p.add_argument("--no-centre", action="store_true", help="pregrasp straight above the cube (cube out of view)")
    p.add_argument("--speed", type=float, default=150.0, help="travel speed [mm/s]")
    p.add_argument("--slow", type=float, default=50.0, help="speed close to the cube / bin [mm/s]")
    p.add_argument("--grip-wait", type=float, default=1.0, help="s to wait for the gripper to close/open")
    p.add_argument("--pre", type=float, default=1.0, help="s recorded at HOME before moving")
    p.add_argument("--post", type=float, default=1.0, help="s recorded at the end")
    p.add_argument("--min-z", type=float, default=0.0, help="refuse grasp poses lower than this [mm]")
    p.add_argument("--min-radius", type=float, default=200.0,
                   help="transfers stay at least this far from the base axis [mm] (avoids self-collision)")
    p.add_argument("--min-pregrasp-radius", type=float, default=150.0,
                   help="pregrasp must be at least this far from the base axis [mm] (C22 at 87 mm)")
    p.add_argument("--min-grasp-radius", type=float, default=150.0,
                   help="refuse cubes closer than this to the base axis [mm]")
    p.add_argument("--no-cameras", action="store_true")
    p.add_argument("--display", action="store_true",
                   help="live Rerun view: cameras + state/action (~10 Hz between episodes, 30 fps while recording)")
    p.add_argument("--zone", type=float, nargs=3, default=None, metavar=("CX", "CY", "HALF"),
                   help="only accept grasps inside the square centre (CX, CY) +- HALF mm, e.g. 280 -60 125")
    p.add_argument("--multi", action="store_true",
                   help="multi-cube scenes: the script picks the target colour, you guide the gripper to that cube")
    args = p.parse_args()
    if args.multi and args.repo_id == p.get_default("repo_id"):
        args.repo_id = f"tataro/record_multicube001_{time.strftime('%Y%m%d')}"

    cfg =Lite6Config(id="tataro_lite6", ip=args.ip, servo_on_connect=False, go_home_on_connect=False)
    if args.no_cameras:
        cfg.cameras = {}
    robot = Lite6(cfg)

    root = Path(args.root or Path.home() / "lerobot_data" / args.repo_id)
    if args.resume:
        dataset = LeRobotDataset.resume(args.repo_id, root=root, image_writer_threads=4)
    else:
        if root.exists() and not (root / "data").exists():  # leftover of a run that saved nothing
            import shutil

            shutil.rmtree(root)
        if root.exists():
            raise SystemExit(f"{root} already exists. Use --resume to add episodes, or pick a new --repo-id.")
        features = combine_feature_dicts(
            hw_to_dataset_features(robot.action_features, ACTION),
            hw_to_dataset_features(robot.observation_features, OBS_STR),
        )
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id, fps=args.fps, features=features, root=root,
            robot_type=robot.name, use_videos=True, image_writer_threads=4,
        )

    print("Connecting Lite6 + cameras ...")
    try:
        robot.connect()
    except ConnectionError as e:  # D435i sometimes stays "busy" after a replug/crash: reset and retry once
        print(f"[!] {e}\n    hardware-resetting the RealSense cameras and retrying ...")
        try:
            robot.disconnect()
        except Exception:  # noqa: BLE001
            pass
        import pyrealsense2 as rs

        for d in rs.context().query_devices():
            d.hardware_reset()
        time.sleep(6)
        robot = Lite6(cfg)
        robot.connect()
    arm = robot._arm
    keys = Keys()
    import threading

    obs_lock = threading.Lock()  # get_observation from the recording loop and the live-view thread
    live = {"run": True, "recording": False}
    if args.display:
        from lerobot.utils.rerun_visualization import init_rerun, log_rerun_data

        init_rerun(session_name="tataro_handguided")

        zone_overlay = None
        if args.zone and "overhead" in robot.cameras:
            zone_overlay = make_zone_overlay(robot.get_observation()["overhead"], args.zone)

        def live_view():  # between episodes: keep the viewer updating while you place / guide
            import rerun as rr

            while live["run"]:
                if not live["recording"]:
                    try:
                        with obs_lock:
                            o = robot.get_observation()
                        log_rerun_data(observation=o)
                        if zone_overlay is not None:  # display only: the recorded images stay clean
                            # JPEG: ~10x less viewer memory than a raw image stream
                            rr.log("zone_view", rr.Image(zone_overlay(o["overhead"])).compress(jpeg_quality=70))
                    except Exception:  # noqa: BLE001
                        pass
                time.sleep(0.1)

        threading.Thread(target=live_view, daemon=True).start()
    dt = 1.0 / args.fps
    action_keys = list(robot.action_features)

    def check_error():
        if arm.error_code:
            raise RuntimeError(f"controller error C{arm.error_code}: check the arm/scene, then restart")

    def ready_position_mode():
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.3)

    def grip(closed: bool):
        set_gripper(arm, closed)
        time.sleep(args.grip_wait)
        if not closed:
            arm.stop_lite6_gripper()

    def blocking_move(x, y, z, rpy, speed):
        code = arm.set_position(x, y, z, *rpy, speed=speed, mvacc=1000, is_radian=True, wait=True)
        check_error()
        if code != 0:
            raise RuntimeError(f"set_position failed ({code})")

    # episodes per target colour (balances the --multi targets; includes episodes already in the dataset)
    counts = {c: 0 for c in COLORS.values()}
    try:
        for t in dataset.meta.episodes["tasks"]:
            for c in counts:
                if re.search(rf"\b{c}\b", t[0] if isinstance(t, (list, tuple)) else str(t)):
                    counts[c] += 1
    except Exception:  # noqa: BLE001  (new dataset: no episodes yet)
        pass
    present = list(COLORS.values())  # --multi: colours on the table, remembered between episodes
    if args.multi:
        print(f"Multi-cube mode. Episodes per target so far: {counts}")

    if args.zone:
        zx, zy, zh = args.zone
        print(f"Zone: grasps only inside x {zx - zh:.0f}..{zx + zh:.0f}, y {zy - zh:.0f}..{zy + zh:.0f} mm "
              "(run place_cube.py --zone to see it on the table)")

    recorded = 0
    try:
        while recorded < args.num_episodes and not keys.stop:
            check_error()
            # ---------- setup (not recorded) ----------
            grip(False)
            ready_position_mode()
            print("\nMoving HOME ...")
            robot._move_home_blocking()
            ready_position_mode()
            robot._gripper_target = 0.0
            if args.multi:
                while True:
                    ans = input("Arrange 2-3 cubes of DIFFERENT colours (new spots/rotations, >= 6 cm apart, "
                                "put the last picked one back).\nColours on the table r/g/b "
                                f"[Enter = {''.join(c[0] for c in present)}]: ").strip().lower()
                    sel = [COLORS[ch] for ch in dict.fromkeys(ans) if ch in COLORS] if ans else present
                    if len(sel) >= 2:
                        break
                    print("  need at least 2 different colours on the table")
                present = sel
                target = min(present, key=lambda c: (counts[c], random.random()))
                print(f"\n    >>>>  TARGET: the {target.upper()} cube  <<<<\n")
                input(f"Press Enter to go compliant, then guide the gripper to the {target.upper()} cube ...")
            else:
                input("Put the cube on the table (new spot / rotation), then press Enter to go compliant ...")
            arm.set_mode(2)  # manual (hand-guiding) mode
            arm.set_state(0)
            time.sleep(0.3)
            print("Arm is COMPLIANT. Guide the open gripper down around the cube "
                  "(fingers either side, at the grip height).")
            input("Press Enter when the grasp pose is right ...")
            code, pose = arm.get_position(is_radian=True)
            ready_position_mode()
            check_error()
            if code != 0:
                raise RuntimeError(f"get_position failed ({code})")
            gx, gy, gz = pose[:3]
            rpy = list(pose[3:6])
            print(f"Grasp pose: x {gx:.1f} y {gy:.1f} z {gz:.1f} mm, rpy "
                  f"{', '.join(f'{math.degrees(a):.1f}' for a in rpy)} deg")
            if gz < args.min_z:
                print(f"[!] grasp z {gz:.1f} < --min-z {args.min_z}: too low, skipping this one")
                continue
            # lift straight up away from the cube (orientation as guided), then HOME
            blocking_move(gx, gy, max(gz + 60.0, args.travel_z - 50), rpy, args.slow)
            robot._move_home_blocking()
            ready_position_mode()
            if args.zone:
                zx, zy, zh = args.zone
                if abs(gx - zx) > zh or abs(gy - zy) > zh:
                    print(f"[!] grasp ({gx:.0f}, {gy:.0f}) is outside the zone x {zx - zh:.0f}..{zx + zh:.0f}, "
                          f"y {zy - zh:.0f}..{zy + zh:.0f} mm. Put the cube inside it and redo.")
                    continue
            if math.hypot(gx, gy) < args.min_grasp_radius:
                print(f"[!] cube is {math.hypot(gx, gy):.0f} mm from the base axis (< {args.min_grasp_radius:.0f}): "
                      "too close, the arm would fold into itself. Move the cube further out and redo.")
                continue
            # The Gripper Lite is symmetric: grasp yaw and yaw+180 are the same grasp. The camera-centred
            # pregrasp is offset ~100 mm to the camera side; if that points at the base (C22 self-collision
            # at 87 mm on 2026-11-06), use the flipped yaw so the offset points outward instead.
            bx, by = args.bin
            plans, reasons = [], []
            for flip in (0.0, math.pi):
                r_ = [rpy[0], rpy[1], (rpy[2] + flip + math.pi) % (2 * math.pi) - math.pi]
                if args.no_centre:
                    pre = (gx, gy, gz + args.pregrasp_dz, None)
                else:
                    pre = camera_centred_pregrasp((gx, gy, gz), r_, args.pregrasp_dz, arm.tcp_offset[2],
                                                  tuple(args.pregrasp_pixel))
                r_pre = math.hypot(pre[0], pre[1])
                tag = f"yaw {math.degrees(r_[2]):.0f}"
                if r_pre < args.min_pregrasp_radius:
                    reasons.append(f"{tag}: pregrasp only {r_pre:.0f} mm from the base")
                    continue
                plans.append((r_pre, r_, pre))
            if not plans:
                print("[!] no safe pregrasp for this grasp:\n    " + "\n    ".join(reasons)
                      + "\n    Place the cube somewhere else (or rotate it) and redo.")
                continue
            r_pre, rpy, (px, py, pz, uv) = plans[0]  # prefer the yaw you guided
            print(f"Pregrasp: x {px:.1f} y {py:.1f} z {pz:.1f} mm, {r_pre:.0f} mm from base, yaw "
                  f"{math.degrees(rpy[2]):.0f} deg" + (f" (cube expected at wrist pixel {uv[0]:.0f},{uv[1]:.0f})" if uv else "")
                  + ("" if not reasons else f"  [yaw flipped: {reasons[0]}]"))
            color = target if args.multi else args.color
            if color == "ask":
                det = detect_cube_color(robot.cameras["wrist"].async_read()) if "wrist" in robot.cameras else None
                while True:
                    ans = input(f"Cube colour r/g/b [Enter = {det or 'not detected'}]: ").strip().lower()
                    color = COLORS.get(ans[:1]) if ans else det
                    if color:
                        break
            task = args.task.format(color=color)
            print(f"Task: {task}")

            # ---------- recorded motion ----------
            keys.exit_early = keys.rerecord = False
            buf: deque = deque()
            n_frames = 0
            next_t = [time.perf_counter()]
            t_ep = time.monotonic()

            def tick():
                nonlocal n_frames
                with obs_lock:
                    obs = robot.get_observation()
                buf.append((obs, build_dataset_frame(dataset.features, obs, prefix=OBS_STR)))
                if len(buf) > args.lookahead:
                    _, of = buf.popleft()
                    act = {k: obs[k] for k in action_keys}
                    dataset.add_frame({**of, **build_dataset_frame(dataset.features, act, prefix=ACTION), "task": task})
                    n_frames += 1
                    if args.display:
                        log_rerun_data(observation=obs, action=act)
                next_t[0] += dt
                s = next_t[0] - time.perf_counter()
                if s > 0:
                    time.sleep(s)
                elif s < -0.1:
                    next_t[0] = time.perf_counter()
                return obs

            def wait_s(sec):
                t_end = time.monotonic() + sec
                while time.monotonic() < t_end and not keys.exit_early:
                    tick()

            def move(x, y, z, speed, timeout=20.0):
                """Linear move while recording; returns when within 1.5 mm of the target and stopped."""
                if keys.exit_early:
                    return
                code = arm.set_position(x, y, z, *rpy, speed=speed, mvacc=1000, is_radian=True, wait=False)
                if code != 0:
                    raise RuntimeError(f"set_position({x:.0f},{y:.0f},{z:.0f}) failed ({code}, C{arm.error_code})")
                t0 = time.monotonic()
                while not keys.exit_early:
                    obs = tick()
                    check_error()
                    d = math.dist((obs["ee.x"] * 1000, obs["ee.y"] * 1000, obs["ee.z"] * 1000), (x, y, z))
                    if d < 1.5 and arm.state != 1:
                        return
                    if time.monotonic() - t0 > timeout:
                        raise RuntimeError(f"move to ({x:.0f},{y:.0f},{z:.0f}) timed out, {d:.1f} mm away")

            def move_around(x, y, z, speed, blend=30.0):
                """Like move(), but swings around the base (blended waypoints) instead of a straight line."""
                if keys.exit_early:
                    return
                code, cur = arm.get_position()
                pts = around_base(cur[:3], (x, y, z), r_min=args.min_radius)
                for wx, wy, wz in pts[:-1]:
                    code = arm.set_position(wx, wy, wz, *rpy, speed=speed, mvacc=1000, radius=blend,
                                            is_radian=True, wait=False)
                    if code != 0:
                        raise RuntimeError(f"set_position via ({wx:.0f},{wy:.0f},{wz:.0f}) failed ({code}, C{arm.error_code})")
                length = sum(math.dist(a, b) for a, b in zip([tuple(cur[:3])] + pts[:-1], pts))
                move(x, y, z, speed, timeout=length / speed * 2 + 10)

            def gripper(closed):
                if keys.exit_early:
                    return
                set_gripper(arm, closed, verbose=True)
                wait_s(args.grip_wait)

            print(f"=== Episode {recorded + 1}/{args.num_episodes} (dataset has {dataset.num_episodes}) "
                  f"- RECORDING. ->/<-/Esc ===")
            live["recording"] = True
            wait_s(args.pre)
            move_around(px, py, pz, args.speed)  # pregrasp: cube in the middle of the wrist image
            wait_s(args.pregrasp_pause)
            move(gx, gy, gz + 40.0, args.slow)  # over the cube
            move(gx, gy, gz, args.slow)
            gripper(True)
            move(gx, gy, args.travel_z, args.slow)
            move_around(bx, by, args.travel_z, args.speed)  # around the base, not over it
            move(bx, by, args.bin_z, args.slow)
            gripper(False)
            move(bx, by, args.travel_z, args.speed)
            wait_s(args.post)
            last = buf[-1][0] if buf else None
            while buf:
                _, of = buf.popleft()
                act = {k: last[k] for k in action_keys}
                dataset.add_frame({**of, **build_dataset_frame(dataset.features, act, prefix=ACTION), "task": task})
                n_frames += 1
            wall = time.monotonic() - t_ep
            live["recording"] = False
            if keys.exit_early:  # arrow/Esc pressed mid-motion: stop the arm
                arm.set_state(4)
                time.sleep(0.3)
                ready_position_mode()
            released = robot.read_gripper_state() < 0.5
            if keys.rerecord or not released:
                print("Discarded" + (" (Left arrow)." if keys.rerecord else ": gripper still closed."))
                dataset.clear_episode_buffer()
            else:
                dataset.save_episode()
                recorded += 1
                counts[color] += 1
                print(f"Saved episode: {n_frames} frames ({n_frames / args.fps:.1f}s, {n_frames / wall:.1f} fps). "
                      f"Dataset now has {dataset.num_episodes}.")
    except KeyboardInterrupt:
        print("\nInterrupted - discarding the unfinished episode.")
        dataset.clear_episode_buffer()
        try:
            arm.set_state(4)
            time.sleep(0.3)
            ready_position_mode()
        except Exception:  # noqa: BLE001
            pass
    finally:
        live["run"] = False
        time.sleep(0.2)
        try:
            ready_position_mode()
            grip(False)
        except Exception:  # noqa: BLE001
            pass
        robot.disconnect()
        if keys.listener:
            keys.listener.stop()
        dataset.finalize()
        print(f"Dataset at {root}  ({dataset.num_episodes} episodes)")


if __name__ == "__main__":
    main()
