#!/usr/bin/env python
"""Score a run_policy.py output folder: per episode, cube start position (overhead colour detection),
where the gripper closed and opened, grasp offset, and where the cube ends up (bowl or not).

    python score_policy_run.py ~/lerobot_outputs/policy_runs/20261123_162549
Assumes a table-view overhead video (the size it was saved at is rescaled to 336 px table coords).
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_grasp_vs_detector import HUE, table_to_robot  # noqa: E402

T2R = table_to_robot()
BOWL = lambda xy: xy[0] < 200 and xy[1] < -200  # noqa: E731  bowl at ~(125, -270) robot mm


def blobs(bgr):
    """All cube-coloured blobs -> [(colour, area, robot_xy)]; frame rescaled to 336 table px."""
    k = 336 / bgr.shape[1]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = (hsv[..., i].astype(int) for i in range(3))
    out = []
    for col, f in HUE.items():
        m = ((s > 40) & (v > 45) & f(h)).astype(np.uint8)
        m[:, : int(18 / k)] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, _, st, cen = cv2.connectedComponentsWithStats(m)
        for i in range(1, n):
            if 40 < st[i, 4] * k * k < 900:
                xy = cv2.perspectiveTransform(np.float32(cen[i] * k).reshape(1, 1, 2), T2R)[0, 0]
                out.append((col, int(st[i, 4] * k * k), xy))
    return out


def frame(path, i):
    c = cv2.VideoCapture(str(path))
    n = int(c.get(cv2.CAP_PROP_FRAME_COUNT))
    c.set(cv2.CAP_PROP_POS_FRAMES, min(max(i, 0), n - 1))
    ok, f = c.read()
    return f, n


def main():
    run = Path(sys.argv[1]).expanduser()
    for csv in sorted(run.glob("ep*.csv"), key=lambda p: int(p.stem[2:])):
        ep = csv.stem
        d = pd.read_csv(csv)
        g = d["act.gripper.pos"].values
        close = np.where(g > 0.5)[0]
        print(f"\n== {ep}: {d.t.iloc[-1]:.1f} s, {len(d)} steps")
        f0, n = frame(run / f"{ep}_overhead.mp4", 2)
        start = [b for b in blobs(f0) if not BOWL(b[2])]
        for col, a, xy in start:
            print(f"   cube at start: {col} ({xy[0]:.0f}, {xy[1]:.0f}) mm  [blob {a}px]")
        if len(close) == 0:
            print("   gripper never closed")
        else:
            i = close[0]
            gx, gy, gz = (d[f"obs.ee.{k}"].values[i] * 1000 for k in "xyz")
            print(f"   gripper closed at t={d.t.values[i]:.1f}s  TCP ({gx:.0f}, {gy:.0f}, z {gz:.0f}) mm")
            if start:
                cx, cy = start[0][2]
                print(f"   grasp offset vs cube: dx {gx - cx:+.0f}  dy {gy - cy:+.0f}  "
                      f"|d| {np.hypot(gx - cx, gy - cy):.0f} mm")
            opn = np.where(g[i:] < 0.5)[0]
            if len(opn):
                j = i + opn[0]
                print(f"   gripper opened at t={d.t.values[j]:.1f}s  TCP ({d['obs.ee.x'].values[j]*1000:.0f}, "
                      f"{d['obs.ee.y'].values[j]*1000:.0f}, z {d['obs.ee.z'].values[j]*1000:.0f}) mm")
            else:
                print("   gripper still closed at the end")
        fe, _ = frame(run / f"{ep}_overhead.mp4", n - 2)
        end = blobs(fe)
        txt = ", ".join(f"{c} ({xy[0]:.0f},{xy[1]:.0f}){' BOWL' if BOWL(xy) else ''}" for c, _, xy in end) or "none"
        print(f"   cubes visible at end: {txt}")


if __name__ == "__main__":
    main()
