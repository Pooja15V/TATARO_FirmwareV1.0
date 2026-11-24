#!/usr/bin/env python
"""Replay a run_policy.py run with the policy's target drawn on the overhead table view. No arm needed.

For each episode of a run folder:
  yellow +        = cube found by cube_detector.py on the first frame (arm at HOME)
  blue trail      = where the policy sent the TCP so far (x, y of its actions)
  blue ring       = the policy's current target; turns into a solid RED dot while it commands the gripper closed
  white ring      = where the arm actually is (measured TCP)
Writes ep<N>_annotated.mp4 and ep<N>_grasp.png (frame at the first close) into the run folder, and prints how far
the gripper closed from the cube.

    python replay_run.py                       # latest run in ~/lerobot_outputs/policy_runs
    python replay_run.py --run 20261124_153122 --colour green
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

import cube_detector as cd

RUNS = Path.home() / "lerobot_outputs" / "policy_runs"
S = 2  # draw at 672 px


def px(xy_mm):
    return tuple(int(v * S) for v in cd.to_table([xy_mm])[0])


def text(im, lines, colours=None):
    for i, t in enumerate(lines):
        y = 26 + 26 * i
        c = (colours or {}).get(i, (255, 255, 255))
        cv2.putText(im, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(im, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 1)


def episode(run, ep, colour):
    d = pd.read_csv(run / f"ep{ep}.csv")
    cap = cv2.VideoCapture(str(run / f"ep{ep}_overhead.mp4"))
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(f)
    if not frames or frames[0].shape[0] != frames[0].shape[1]:
        print(f"ep{ep}: overhead video is not the square table view, skipped")
        return None
    n = min(len(frames), len(d))
    _, cube = cd.detect(cv2.cvtColor(frames[0], cv2.COLOR_BGR2RGB), colour)
    act = d[["act.ee.x", "act.ee.y"]].to_numpy() * 1000
    obs = d[["obs.ee.x", "obs.ee.y", "obs.ee.z"]].to_numpy() * 1000
    grip = d["act.gripper.pos"].to_numpy() > 0.5
    first_close = int(np.argmax(grip)) if grip.any() else None
    h = frames[0].shape[0] * S
    vw = cv2.VideoWriter(str(run / f"ep{ep}_annotated.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30, (h, h))
    trail = [px(a) for a in act[:n]]
    err_close = None if (cube is None or first_close is None) else float(np.linalg.norm(obs[first_close, :2] - cube))
    for i in range(n):
        im = cv2.resize(frames[i], None, fx=S, fy=S, interpolation=cv2.INTER_CUBIC)
        if i > 1:
            cv2.polylines(im, [np.array(trail[: i + 1], np.int32)], False, (255, 160, 0), 1)
        if cube is not None:
            cv2.drawMarker(im, px(cube), (0, 255, 255), cv2.MARKER_CROSS, 28, 2)
        cv2.circle(im, px(obs[i, :2]), 6, (255, 255, 255), 1)
        if grip[i]:
            cv2.circle(im, trail[i], 8, (0, 0, 255), -1)
        else:
            cv2.circle(im, trail[i], 9, (255, 160, 0), 2)
        d_cube = None if cube is None else float(np.linalg.norm(act[i] - cube))
        lines = [f"ep{ep} {colour}  t={d.t.iloc[i]:5.1f}s  z={obs[i, 2]:4.0f} mm  grip={'CLOSE' if grip[i] else 'open'}",
                 "policy target -> cube: " + ("-" if d_cube is None else f"{d_cube:.0f} mm")]
        if first_close is not None and i >= first_close and err_close is not None:
            lines.append(f"closed {err_close:.0f} mm from the cube")
        text(im, lines, {2: (0, 255, 0) if (err_close or 99) < 10 else (0, 0, 255)})
        vw.write(im)
        if i == first_close:
            cv2.imwrite(str(run / f"ep{ep}_grasp.png"), im)
    vw.release()
    r = {"ep": ep, "cube": None if cube is None else cube.round(1).tolist(),
         "close_at": None if first_close is None else obs[first_close].round(1).tolist(),
         "close_err_mm": None if err_close is None else round(err_close, 1), "t_close": None
         if first_close is None else round(float(d.t.iloc[first_close]), 1)}
    print(f"ep{ep}: cube {r['cube']}  gripper closed at {r['close_at']} (t={r['t_close']} s)  "
          f"-> {r['close_err_mm']} mm from the cube")
    return r


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run", default=None, help="run folder name or path (default: latest)")
    p.add_argument("--colour", default=None, help="cube colour (default: from outcomes.json, else green)")
    a = p.parse_args()
    run = Path(a.run) if a.run and Path(a.run).is_dir() else (
        RUNS / a.run if a.run else max((q for q in RUNS.iterdir() if q.is_dir()), key=lambda q: q.name))
    oc = {}
    if (run / "outcomes.json").exists():
        oc = {o["ep"]: o["colour"] for o in json.loads((run / "outcomes.json").read_text())}
    print(f"run {run}")
    for f in sorted(run.glob("ep*.csv"), key=lambda q: int(q.stem[2:])):
        ep = int(f.stem[2:])
        episode(run, ep, a.colour or oc.get(ep, "green"))
    print(f"annotated videos + grasp images in {run}")


if __name__ == "__main__":
    main()
