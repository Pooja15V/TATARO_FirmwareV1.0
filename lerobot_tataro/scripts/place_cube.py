#!/usr/bin/env python
"""Live overhead view with the cube's robot (x, y): place a cube at a recorded demo's spot, or inside a zone.

Positions are measured in TABLE coordinates (markers 51/52/53, lerobot_robot_lite6.table_warp), then mapped to robot
mm with a map fitted on the demos' grasps, so a bumped overhead camera doesn't fool it.
The arm is not moved or connected.

    conda activate lerobot
    python place_cube.py --episode 45                 # put the cube where demo episode 45 had it
    python place_cube.py --zone 280 -60 125           # show the recording zone, read the cube position
    python place_cube.py --list                       # demo cube positions (robot mm)
The view opens in a Rerun window (the terminal also prints the cube position). Ctrl+C = quit.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from recolor_dataset import Src  # noqa: E402

from lerobot_robot_lite6.table_warp import TableWarp  # noqa: E402

DATA = HERE.parent / "datasets" / "tataro" / "record_cube001_20261106"


def blob(img):
    """Centre (raw overhead px) of the biggest red/green/blue cube-sized blob on the table, or None."""
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    h, s, v = (hsv[..., i].astype(int) for i in range(3))
    m = (s > 110) & (v > 45) & (((h >= 95) & (h <= 120)) | (h <= 8) | (h >= 168) | ((h >= 40) & (h <= 90)))
    roi = np.zeros_like(m)
    roi[25:375, 135:500] = True  # table
    roi[250:, 190:340] = False  # arm at HOME
    roi[280:360, 365:450] = False  # bowl
    m = cv2.morphologyEx((m & roi).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats(m)
    c = [(st[i, 4], cen[i]) for i in range(1, n) if 25 < st[i, 4] < 1200]
    return max(c, key=lambda t: t[0])[1] if c else None


def demo_table(cache: Path):
    """Per demo: cube position in table px and grasp (robot mm)."""
    if cache.exists():
        z = np.load(cache)
        if "tab" in z:
            return z["eps"], z["tab"], z["grasp"]
    src = Src(DATA)
    eps, tab, grasp = [], [], []
    for e in sorted(int(x) for x in src.eps.episode_index):
        img = src.frames(e, "overhead", 1)[0]
        b = blob(img)
        if b is None:
            continue
        a = np.stack(src.rows(e).action.values)
        eps.append(e)
        tab.append(TableWarp.from_image(img).points(b)[0])
        grasp.append(a[int(np.argmax(a[:, 6] > 0.5)), :2] * 1000)
    eps, tab, grasp = np.array(eps), np.array(tab, np.float32), np.array(grasp, np.float32)
    np.savez(cache, eps=eps, tab=tab, grasp=grasp)
    return eps, tab, grasp


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--episode", type=int, default=None, help="target = cube position of this demo episode")
    p.add_argument("--zone", type=float, nargs=3, default=None, metavar=("CX", "CY", "HALF"))
    p.add_argument("--list", action="store_true")
    args = p.parse_args()

    eps, tab, grasp = demo_table(DATA.parent / ".place_cube_cache.npz")
    T2R, inl = cv2.findHomography(tab, grasp, cv2.RANSAC, 15)  # table px -> robot mm
    err = np.linalg.norm(cv2.perspectiveTransform(tab[None], T2R)[0] - grasp, axis=1)
    R2T = np.linalg.inv(T2R)
    if args.list:
        for e, g in zip(eps, grasp):
            print(f"episode {e:2d}: cube at robot ({g[0]:4.0f}, {g[1]:4.0f}) mm")
        return
    print(f"table->robot map from {int(inl.sum())}/{len(eps)} demos, median error {np.median(err):.0f} mm")
    tgt_xy = None
    if args.episode is not None:
        if args.episode not in eps:
            raise SystemExit(f"episode {args.episode}: cube not found in its overhead image; pick one from --list")
        tgt_xy = grasp[int(np.where(eps == args.episode)[0][0])]
        print(f"target: demo episode {args.episode}, cube at robot ({tgt_xy[0]:.0f}, {tgt_xy[1]:.0f}) mm")

    import rerun as rr  # the lerobot env has headless OpenCV (no cv2.imshow), so show it in Rerun
    from lerobot.cameras.utils import make_cameras_from_configs
    from lerobot_robot_lite6 import Lite6Config

    rr.init("place_cube", spawn=True)
    cam = make_cameras_from_configs({"overhead": Lite6Config(id="x").cameras["overhead"]})["overhead"]
    cam.connect()
    time.sleep(1.0)
    warp = TableWarp.from_image(np.asarray(cam.read()))  # once: the arm/hand may cover markers later
    raw_of = np.linalg.inv(warp.H)

    def robot_to_raw(xy):
        t = cv2.perspectiveTransform(np.float32(xy).reshape(-1, 1, 2), R2T)
        return cv2.perspectiveTransform(t, raw_of).reshape(-1, 2)

    print("Rerun viewer opened. Ctrl+C to quit.")
    last = ""
    try:
        while True:
            img = np.asarray(cam.read()).copy()
            b = blob(img)
            vis = img.copy()  # RGB
            if args.zone:
                zx, zy, zh = args.zone
                q = robot_to_raw([[zx - zh, zy - zh], [zx + zh, zy - zh], [zx + zh, zy + zh], [zx - zh, zy + zh]])
                cv2.polylines(vis, [q.astype(np.int32)], True, (255, 0, 255), 2)
            if tgt_xy is not None:
                c = robot_to_raw([tgt_xy])[0]
                cv2.circle(vis, (int(c[0]), int(c[1])), 10, (255, 255, 0), 2)
            if b is None:
                msg, col = "cube not found on the table", (255, 0, 0)
            else:
                xy = cv2.perspectiveTransform(warp.points(b).reshape(1, 1, 2), T2R)[0, 0]
                msg, col = f"cube at robot ({xy[0]:.0f}, {xy[1]:.0f}) mm", (0, 255, 0)
                if args.zone:
                    inside = abs(xy[0] - zx) <= zh and abs(xy[1] - zy) <= zh
                    msg += "  INSIDE zone" if inside else "  OUTSIDE zone"
                    col = (0, 255, 0) if inside else (255, 0, 0)
                if tgt_xy is not None:
                    d = float(np.linalg.norm(xy - tgt_xy))
                    dx, dy = tgt_xy - xy
                    col = (0, 255, 0) if d < 10 else (255, 200, 0) if d < 25 else (255, 0, 0)
                    msg += (f", off by {d:.0f} mm: move {dx:+.0f} in x (+ = away from base), "
                            f"{dy:+.0f} in y (- = towards the bowl)" + ("  OK" if d < 10 else ""))
                cv2.drawMarker(vis, (int(b[0]), int(b[1])), col, cv2.MARKER_CROSS, 18, 2)
            cv2.putText(vis, msg.split(",")[0], (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
            rr.log("overhead", rr.Image(vis))
            if msg != last:
                print("\r" + msg.ljust(120), end="", flush=True)
                last = msg
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        cam.disconnect()


if __name__ == "__main__":
    main()
