#!/usr/bin/env python
"""Cube detector for the 336x336 TABLE VIEW (overhead warped on markers 51/52/53), arm at HOME.

HSV ranges are calibrated from the training data itself: for each demo episode the grasp xy tells where the cube
was, so its first frame is sampled there (table view is paler than the raw D455 view, so the CV-pipeline
COLOR_RANGES do not transfer). Robot mm <-> table px uses the homography place_cube.py fits on the demos.

    python cube_detector.py --calibrate          # (re)build cube_hsv.json from the training split
    python cube_detector.py --test               # detection rate / error on the val split
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "datasets" / "tataro"
CAL = DATA / "cube_hsv.json"
COLOURS = ("red", "green", "blue")
SIZE = 336


def table_to_robot():
    z = np.load(DATA / ".place_cube_cache.npz")
    H, _ = cv2.findHomography(z["tab"], z["grasp"], cv2.RANSAC, 15)
    return H


T2R = table_to_robot()
R2T = np.linalg.inv(T2R)


def to_robot(px):
    return cv2.perspectiveTransform(np.float32(px).reshape(-1, 1, 2), T2R).reshape(-1, 2)


def to_table(xy_mm):
    return cv2.perspectiveTransform(np.float32(xy_mm).reshape(-1, 1, 2), R2T).reshape(-1, 2)


def roi_mask(shape=(SIZE, SIZE)):
    m = np.zeros(shape, bool)
    m[8:-8, 18:-8] = True  # left strip = desk clutter beyond the table edge
    m[220:, 255:] = False  # bowl / bin
    return m


def load_cal():
    return json.loads(CAL.read_text())


def colour_mask(hsv, rng):
    h, s, v = (hsv[..., i].astype(int) for i in range(3))
    hm = np.zeros_like(h, bool)
    for lo, hi in rng["h"]:
        hm |= (h >= lo) & (h <= hi)
    return hm & (s >= rng["s"][0]) & (v >= rng["v"][0]) & (v <= rng["v"][1])


BG = DATA / "table_background.png"  # empty table with the arm at HOME (median of demo first frames)
_bg = None


def background():
    global _bg
    if _bg is None and BG.exists():
        _bg = cv2.cvtColor(cv2.imread(str(BG)), cv2.COLOR_BGR2RGB).astype(np.int16)
    return _bg


def changed(img, bg=None, thr=35):
    """Pixels that differ from the empty-table background (kills permanent stains/marks on the table)."""
    bg = background() if bg is None else bg
    if bg is None:
        return np.ones(img.shape[:2], bool)
    d = np.abs(img.astype(np.int16) - bg).sum(axis=2)
    return d > thr


def detect(table_rgb, colour, cal=None, roi=None, bg=None):
    """(table px centre, robot mm) of the biggest cube-sized blob of `colour`, or (None, None).
    bg: empty-table reference (e.g. the HOME frame before the cube was placed); default = table_background.png."""
    cal = cal or load_cal()
    k = SIZE / table_rgb.shape[1]
    img = table_rgb if k == 1 else cv2.resize(table_rgb, (SIZE, SIZE), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    m = colour_mask(hsv, cal[colour]) & (roi_mask() if roi is None else roi) & changed(img, bg)
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats(m)
    c = [(st[i, 4], cen[i]) for i in range(1, n) if 30 < st[i, 4] < 900]
    if not c:
        return None, None
    px = max(c, key=lambda t: t[0])[1]
    return px, to_robot(px)[0]


def bowl_roi(shape=(SIZE, SIZE)):
    """Inside of the bowl / bin (bottom-right of the table view), the place target."""
    m = np.zeros(shape, bool)
    m[222:330, 257:330] = True
    return m


def new_object(before, after, roi, thr=70, area=(120, 1500)):
    """Robot mm of the biggest new cube-sized object (any colour) inside roi, from before/after table views."""
    d = np.abs(after.astype(np.int16) - before.astype(np.int16)).sum(axis=2) > thr
    m = cv2.morphologyEx((d & roi).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats(m)
    c = [(st[i, 4], cen[i]) for i in range(1, n) if area[0] < st[i, 4] < area[1]]
    return None if not c else to_robot(max(c, key=lambda t: t[0])[1])[0]


def check_episode(before, after, colour, cal=None, near_mm=25.0):
    """Grasp-success check from two HOME table views (336x336 RGB), before and after a policy episode.

    success  = a new `colour` blob appeared inside the bowl
    missed   = the cube is still where it was (within near_mm)
    moved    = the cube is on the table somewhere else (pushed, or dropped on the way)
    unknown  = the cube is not visible on the table or in the bowl (still in the gripper, off the table, hidden)
    Returns (outcome, info dict with robot-mm positions)."""
    cal = cal or load_cal()
    _, pre = detect(before, colour, cal)
    _, post = detect(after, colour, cal)
    _, bowl = detect(after, colour, cal, roi=bowl_roi(), bg=before.astype(np.int16))  # new in the bowl only
    if bowl is None:  # cube lit so it looks grey/white: accept any new cube-sized object in the bowl
        bowl = new_object(before, after, bowl_roi())
    info = {"before": None if pre is None else pre.round(1).tolist(),
            "after": None if post is None else post.round(1).tolist(),
            "bowl": None if bowl is None else bowl.round(1).tolist()}
    if bowl is not None:
        out = "success"
    elif post is not None and pre is not None and np.linalg.norm(post - pre) < near_mm:
        out = "missed"
    elif post is not None:
        out = "moved"
    else:
        out = "unknown"
    return out, info


def draw_check(before, after, colour, outcome, info, title=""):
    """Side-by-side BGR image of the before/after table views with the detections marked."""
    tiles = []
    for img, keys, name in ((before, ["before"], "before"), (after, ["after", "bowl"], "after")):
        v = cv2.resize(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        for k in keys:
            if info.get(k) is not None:
                px = tuple(int(c * 2) for c in to_table([info[k]])[0])
                cv2.drawMarker(v, px, (0, 255, 255) if k != "bowl" else (0, 255, 0), cv2.MARKER_CROSS, 30, 3)
        cv2.putText(v, name, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(v, name, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        tiles.append(v)
    im = np.hstack(tiles)
    col = {"success": (0, 200, 0), "missed": (0, 0, 255), "moved": (0, 140, 255)}.get(outcome, (200, 200, 200))
    t = f"{title} {colour}: {outcome.upper()}"
    cv2.putText(im, t, (10, im.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 5)
    cv2.putText(im, t, (10, im.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
    return im


def _first_frames(root, repo_id):
    from lerobot.datasets import LeRobotDataset

    ds = LeRobotDataset(repo_id, root=root)
    epi = np.array(ds.hf_dataset["episode_index"])
    import torch

    act = torch.stack(list(ds.hf_dataset["action"])).numpy()
    for e in range(ds.num_episodes):
        fr = np.where(epi == e)[0]
        g = np.where(act[fr, 6] > 0.5)[0]
        if len(g) == 0:
            continue
        f = ds[int(fr[0])]
        img = (f["observation.images.overhead"].permute(1, 2, 0).numpy() * 255).clip(0, 255).astype(np.uint8)
        colour = next((c for c in COLOURS if c in f["task"]), None)
        yield e, colour, img, act[fr[g[0]], :2] * 1000


def calibrate():
    split = DATA / "record_tataro_v2_table_split"
    samples = {c: [] for c in COLOURS}
    firsts = []
    for e, colour, img, grasp in _first_frames(split / "train", "tataro/record_tataro_v2_table_train"):
        firsts.append(img)
        if colour is None:
            continue
        x, y = to_table([grasp])[0].round().astype(int)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        box = hsv[max(y - 7, 0): y + 8, max(x - 7, 0): x + 8].reshape(-1, 3).astype(int)
        box = box[box[:, 1] > 35]  # drop table pixels (sat ~3)
        if len(box) >= 6:
            samples[colour].append(box)
    # Hue bands from the sampled histograms (2026-11-23, 218 train eps): red 160-179 + 0-10 (plus ~15% stray
    # 90-110 px at the edges of recoloured copies), green 50-110 peaking 60-90 (teal in the pale table view),
    # blue 90-110. Blind percentiles let the strays widen red over blue, so take each colour's main band only
    # and split green/blue at 92.
    BANDS = {"red": [[0, 10], [158, 179]], "green": [[50, 92]], "blue": [[92, 112]]}
    cal = {}
    for c in COLOURS:
        p = np.concatenate(samples[c])
        hr = BANDS[c]
        inband = np.zeros(len(p), bool)
        for lo, hi in hr:
            inband |= (p[:, 0] >= lo) & (p[:, 0] <= hi)
        p = p[inband]
        s_lo = int(max(np.percentile(p[:, 1], 5) - 5, 30))
        v_lo, v_hi = np.percentile(p[:, 2], [3, 99])
        cal[c] = {"h": hr, "s": [s_lo, 255], "v": [int(v_lo) - 10, min(int(v_hi) + 10, 255)],
                  "episodes": len(samples[c])}
        print(c, cal[c])
    CAL.write_text(json.dumps(cal, indent=1))
    # Cube positions vary between demos, so the per-pixel median of the first frames is the empty table
    # (with its permanent stains and the arm at HOME).
    bg = np.median(np.stack(firsts), axis=0).astype(np.uint8)
    cv2.imwrite(str(BG), cv2.cvtColor(bg, cv2.COLOR_RGB2BGR))
    print(f"saved {CAL} and {BG} (median of {len(firsts)} first frames)")


def test():
    split = DATA / "record_tataro_v2_table_split"
    cal = load_cal()
    errs, miss = [], []
    for e, colour, img, grasp in _first_frames(split / "val", "tataro/record_tataro_v2_table_val"):
        _, xy = detect(img, colour, cal)
        if xy is None:
            miss.append(e)
            continue
        errs.append(np.linalg.norm(xy - grasp))
        print(f"ep {e:3d} {colour:5s} det ({xy[0]:4.0f},{xy[1]:4.0f}) demo grasp ({grasp[0]:4.0f},{grasp[1]:4.0f}) "
              f"|d| {errs[-1]:5.1f} mm")
    print(f"\ndetected {len(errs)}/{len(errs) + len(miss)}, median |det-demo grasp| {np.median(errs):.1f} mm, "
          f"max {np.max(errs):.1f}; missed {miss}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calibrate", action="store_true")
    p.add_argument("--test", action="store_true")
    a = p.parse_args()
    if a.calibrate:
        calibrate()
    if a.test:
        test()
