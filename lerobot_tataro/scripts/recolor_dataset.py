#!/usr/bin/env python
"""Synthetic colour variants: recolour the BLUE cube in recorded episodes to red / green.

Copies every blue-cube episode of a dataset into a NEW dataset with the cube's pixels recoloured in
both cameras (state + actions unchanged, task text changed to the new colour). The real dataset is
never modified. Colour targets were measured on the real red/green cubes (wrist pregrasp frames,
2026-11-06), so the synthetic cubes match their hue, saturation and darkness.

    conda activate lerobot
    python recolor_dataset.py --preview                  # writes preview images only, no dataset
    python recolor_dataset.py                            # full run -> <src>_synth_recolor
    python recolor_dataset.py --colors red --episodes 0 1 2

Method: a strict mask (saturated blue) finds the cube, a dilated + softer mask catches its edges,
blue things that are in the same overhead pixels in most episodes (cables/electronics on the left)
and the bottom strip of the overhead view (people standing at the table) are left alone.
"""

import argparse
import glob
import re
from pathlib import Path

import av
import cv2
import numpy as np
import pandas as pd

PROJECT_DATA = Path(__file__).resolve().parent.parent / "datasets"

# measured on the real cubes: (hue [OpenCV 0-180], saturation scale, value scale) per camera, relative to blue
TARGETS = {
    "red": {"wrist": (2.0, 139.3 / 176.6, 101.3 / 122.9), "overhead": (176.0, 140.0 / 167.0, 136.7 / 128.2)},
    # overhead green stats were polluted by background; use the wrist ratios for both cameras
    "green": {"wrist": (73.0, 134.1 / 176.6, 58.8 / 122.9), "overhead": (66.0, 134.1 / 176.6, 58.8 / 122.9)},
}
CAMS = ("wrist", "overhead")


def cube_mask(rgb, cam, static=None):
    """Float mask 0..1 of the blue cube (soft edges).

    Measured (wrist, 2026-11-06): cube hue 98-102, at least part of it S >= 125 even when small/far;
    its lit face is paler (S 70-130). Bowl: hue 88-98 (median 95), S 57-120. So: seed = hue >= 98 & S >= 125,
    then grow a limited distance (scaled to the seed size) into hue >= 97 & S >= 65 pixels.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = (hsv[..., i].astype(np.int16) for i in range(3))
    seed = ((h >= 98) & (h <= 116) & (s >= 125) & (v > 40)).astype(np.uint8)
    if cam == "overhead":
        seed[430:] = 0  # people at the table edge
        if static is not None:
            seed[static] = 0
    seed = cv2.morphologyEx(seed, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(seed)
    allowed = ((h >= 97) & (h <= 118) & (s >= 65) & (v > 35)).astype(np.uint8)
    min_area = 12 if cam == "overhead" else 40
    out = np.zeros(h.shape, np.uint8)
    for i in range(1, n):
        area = st[i, cv2.CC_STAT_AREA]
        if area < min_area:
            continue
        comp = (lab == i).astype(np.uint8)
        steps = int(np.clip(1.2 * np.sqrt(area), 8, 40))  # geodesic growth, ~seed size (pale lit face)
        k = np.ones((3, 3), np.uint8)
        for _ in range(steps):
            nxt = cv2.dilate(comp, k) & (allowed | comp)
            if (nxt == comp).all():
                break
            comp = nxt
        out |= comp
    if not out.any():
        return None
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))  # glare / texture holes
    return cv2.GaussianBlur(out.astype(np.float32), (5, 5), 0)


def recolor(rgb, mask, target):
    if mask is None:
        return rgb
    hue, s_scale, v_scale = target
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV).astype(np.float32)
    dh = hsv[..., 0] - 98.5  # keep the cube's own small hue variation
    hsv[..., 0] = np.mod(hue + dh, 180.0)
    hsv[..., 1] = np.clip(hsv[..., 1] * s_scale, 0, 255)
    hsv[..., 2] = np.clip(hsv[..., 2] * v_scale, 0, 255)
    new = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2RGB).astype(np.float32)
    a = mask[..., None]
    return (rgb.astype(np.float32) * (1 - a) + new * a).astype(np.uint8)


class Src:
    def __init__(self, root: Path):
        self.root = root
        self.eps = pd.concat([pd.read_parquet(f) for f in sorted(glob.glob(str(root / "meta/episodes/*/*.parquet")))])
        self.data = pd.concat([pd.read_parquet(f) for f in sorted(glob.glob(str(root / "data/*/*.parquet")))])
        tasks = pd.read_parquet(root / "meta/tasks.parquet")
        self.task_of = {int(i): t for t, i in zip(tasks.index, tasks["task_index"])}

    def rows(self, e):
        return self.data[self.data.episode_index == e].sort_values("frame_index")

    def task(self, e):
        return self.task_of[int(self.rows(e).task_index.iloc[0])]

    def frames(self, e, cam, n):
        row = self.eps[self.eps.episode_index == e].iloc[0]
        key = f"videos/observation.images.{cam}"
        path = self.root / f"videos/observation.images.{cam}/chunk-{int(row[key + '/chunk_index']):03d}/file-{int(row[key + '/file_index']):03d}.mp4"
        t0 = float(row[key + "/from_timestamp"])
        c = av.open(str(path))
        s = c.streams.video[0]
        c.seek(int(max(0.0, t0 - 1.0) / s.time_base), stream=s, backward=True)
        out = []
        for f in c.decode(s):
            if f.time is None or f.time < t0 - 1e-3:
                continue
            out.append(f.to_ndarray(format="rgb24"))
            if len(out) == n:
                break
        c.close()
        if len(out) != n:
            raise RuntimeError(f"episode {e} {cam}: decoded {len(out)} frames, expected {n}")
        return out


def static_blue(src: Src, eps):
    """Overhead pixels that are saturated-blue in most episodes' first frame = scene objects, not the cube."""
    acc = None
    for e in eps:
        img = src.frames(e, "overhead", 1)[0]
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        m = (hsv[..., 0] >= 88) & (hsv[..., 0] <= 125) & (hsv[..., 1] > 90)
        acc = m.astype(np.float32) if acc is None else acc + m
    static = acc / len(eps) > 0.5
    return cv2.dilate(static.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--src-root", default=str(PROJECT_DATA / "tataro/record_cube001_20261106"))
    p.add_argument("--src-repo-id", default="tataro/record_cube001_20261106")
    p.add_argument("--out-root", default=None, help="default: <src-root>_synth_recolor")
    p.add_argument("--colors", nargs="+", default=["red", "green"], choices=list(TARGETS))
    p.add_argument("--episodes", type=int, nargs="+", default=None, help="default: every blue-cube episode")
    p.add_argument("--preview", action="store_true", help="only write preview images (real vs synthetic)")
    args = p.parse_args()

    src_root = Path(args.src_root)
    src = Src(src_root)
    all_eps = sorted(int(e) for e in src.eps.episode_index)
    blue = [e for e in all_eps if re.search(r"\bblue\b", src.task(e))]
    eps = args.episodes if args.episodes is not None else blue
    print(f"source {src_root}: {len(all_eps)} episodes, {len(blue)} blue; recolouring {len(eps)} -> {args.colors}")
    static = static_blue(src, blue[:: max(1, len(blue) // 15)])

    if args.preview:
        out = src_root.parent / (src_root.name + "_recolor_preview")
        out.mkdir(exist_ok=True)
        for e in eps[:6]:
            rows = src.rows(e)
            g = np.stack(rows["observation.state"].values)[:, 12]
            c = int(np.where(np.diff(g) > 0)[0][0])
            ks = [5, c // 2, c - 15, len(rows) - 1]
            for cam in CAMS:
                fr = src.frames(e, cam, len(rows))
                tiles = []
                for k in ks:
                    m = cube_mask(fr[k], cam, static)
                    col = [fr[k]] + [recolor(fr[k], m, TARGETS[cl][cam]) for cl in args.colors]
                    tiles.append(np.vstack(col))
                cv2.imwrite(str(out / f"ep{e:02d}_{cam}.jpg"),
                            cv2.cvtColor(cv2.resize(np.hstack(tiles), None, fx=0.5, fy=0.5), cv2.COLOR_RGB2BGR))
        print(f"preview images (rows: original, {', '.join(args.colors)}) in {out}")
        return

    from lerobot.datasets import LeRobotDataset

    info = __import__("json").load(open(src_root / "meta/info.json"))
    features = {k: v for k, v in info["features"].items()
                if k not in ("timestamp", "frame_index", "episode_index", "index", "task_index")}
    out_root = Path(args.out_root or str(src_root) + "_synth_recolor")
    if out_root.exists():
        raise SystemExit(f"{out_root} exists; delete it or pass --out-root")
    ds = LeRobotDataset.create(repo_id=args.src_repo_id + "_synth_recolor", fps=info["fps"], features=features,
                               root=out_root, robot_type=info.get("robot_type"), use_videos=True,
                               image_writer_threads=8)
    try:
        for i, e in enumerate(eps):
            rows = src.rows(e)
            frames = {cam: src.frames(e, cam, len(rows)) for cam in CAMS}
            masks = {cam: [cube_mask(f, cam, static) for f in frames[cam]] for cam in CAMS}
            for cl in args.colors:
                task = re.sub(r"\bblue\b", cl, src.task(e))
                for j, (_, r) in enumerate(rows.iterrows()):
                    fr = {"observation.state": np.asarray(r["observation.state"], dtype=np.float32),
                          "action": np.asarray(r["action"], dtype=np.float32), "task": task}
                    for cam in CAMS:
                        fr[f"observation.images.{cam}"] = recolor(frames[cam][j], masks[cam][j], TARGETS[cl][cam])
                    ds.add_frame(fr)
                ds.save_episode()
                print(f"[{i + 1}/{len(eps)}] source ep {e} -> {cl}: {len(rows)} frames  "
                      f"(synthetic dataset now {ds.num_episodes} episodes)", flush=True)
    finally:
        ds.finalize()
    print(f"done: {out_root}  ({ds.num_episodes} episodes)")


if __name__ == "__main__":
    main()
