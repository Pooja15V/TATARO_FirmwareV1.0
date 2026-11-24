#!/usr/bin/env python
"""Finish the recolouring of synthetic red/green episodes in a table-view dataset split.

recolor_dataset.py only seeded its cube mask on strongly saturated blue (S >= 125). The pale/overexposed blue cube
(S ~60 in the table view) got no mask, so 2026-11-23 checks found red/green-labelled episodes whose overhead (and in
a few, wrist) still shows a BLUE cube, fully or partly. This script recolours the LEFTOVER blue cube pixels of every
red/green episode that still has them (already recoloured pixels are not blue, so they stay as they are).

  overhead (336 table view): blue-ish (H 96-118, S >= 28) AND different from the empty-table background
                             (table_background.png, so the static bowl / scene never match), cube-sized blobs
  wrist (640x480):           seed H 98-116 & S >= 80, grown into H >= 97 & S >= 45 (bowl is H 88-98)
  colour: target hue (+ the cube's own hue variation), S and V scaled by measured real-cube ratios, so a pale
          blue cube becomes an equally pale red/green.

Output: <dst>/<split> = original split with the affected episodes replaced (original split untouched).

    conda activate lerobot_smolvla
    python fix_recolor.py --split val --preview          # before/after images of affected episodes only
    python fix_recolor.py --split train                  # build record_tataro_v2fix_split/train
"""
import argparse
import json
import re
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cube_detector as cd  # noqa: E402
from recolor_dataset import Src  # noqa: E402

DATA = cd.DATA
# (hue, S scale, V scale) relative to the blue cube. Overhead: measured in the table view 2026-11-23
# (median H/S/V red 174/143/146, green 71.5/122/73, blue 98/172/148). Wrist: recolor_dataset.TARGETS.
TARGETS = {
    "red": {"overhead": (174.0, 143 / 172, 146 / 148), "wrist": (2.0, 139.3 / 176.6, 101.3 / 122.9)},
    "green": {"overhead": (71.5, 122 / 172, 73 / 148), "wrist": (73.0, 134.1 / 176.6, 58.8 / 122.9)},
}
BLUE_H = 99.0


def mask_overhead(rgb, bg, col):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = (hsv[..., i].astype(np.int16) for i in range(3))
    region = np.ones(h.shape, bool)
    region[:, :18] = False
    region[215:, 250:] = False  # bowl: its rim differs from the background with lighting and is light blue
    # lit faces: blue-ish and bright (V >= 90; the cable and the arm's bluish shadow are V 40-60)
    m = (h >= 97) & (h <= 118) & (s >= 28) & (v >= 90) & cd.changed(rgb, bg, thr=30) & region
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    keep = np.isin(lab, [i for i in range(1, n) if 10 <= st[i, cv2.CC_STAT_AREA] <= 1500]).astype(np.uint8)
    # shaded faces (dark teal, H 92-118, V ~50): only where they touch the cube's lit blue or already
    # recoloured pixels, grown a few px (the black cable is unsaturated, so it never qualifies)
    # also the purple fringe (H 118-165) the first recolour's soft edge left between the red and teal faces
    dark = ((h >= (92 if col == "red" else 86)) & (h <= (165 if col == "red" else 118)) & (s >= 40) & (v >= 25) & region).astype(np.uint8)
    done = (((h <= 8) | (h >= 165)) if col == "red" else ((h >= 50) & (h <= 92))) & (s >= 60) & region
    seed = cv2.morphologyEx(done.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8)) | keep
    grown = seed.copy()
    for _ in range(8):
        nxt = cv2.dilate(grown, np.ones((3, 3), np.uint8)) & (dark | grown)
        if (nxt == grown).all():
            break
        grown = nxt
    keep = keep | (grown & dark)
    if keep.sum() < 10:
        return None
    keep = cv2.dilate(keep, np.ones((3, 3), np.uint8)) & (s >= 18)  # pale edges
    return cv2.GaussianBlur(keep.astype(np.float32), (3, 3), 0)


def mask_wrist(rgb, col):
    """Leftover blue cube pixels in a wrist frame of a `col`-labelled episode.

    Bowl rim is H 88-98 / S ~77, the blue cube H 98-102, so only H >= 99 is ever recoloured. Two seeds:
    a fully un-recoloured cube (strong blue, S >= 110) and the cube's already recoloured pixels (red/green),
    grown a few px into pale blue -> catches glare patches the first recolour left on the cube's lit face."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = (hsv[..., i].astype(np.int16) for i in range(3))
    blue = ((h >= 99) & (h <= 118) & (s >= 45) & (v > 35)).astype(np.uint8)
    strong = ((h >= 99) & (h <= 116) & (s >= 110) & (v > 40)).astype(np.uint8)
    done = (((h <= 8) | (h >= 165)) if col == "red" else ((h >= 55) & (h <= 90))) & (s >= 90) & (v > 30)
    out = np.zeros(h.shape, np.uint8)
    for seed, min_area, steps in ((strong, 60, 30), (done.astype(np.uint8), 200, 8)):
        seed = cv2.morphologyEx(seed, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, _ = cv2.connectedComponentsWithStats(seed)
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] < min_area:
                continue
            comp = (lab == i).astype(np.uint8)
            for _ in range(steps):
                nxt = cv2.dilate(comp, np.ones((3, 3), np.uint8)) & (blue | comp)
                if (nxt == comp).all():
                    break
                comp = nxt
            out |= comp & blue
    if out.sum() < 30:
        return None
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return cv2.GaussianBlur(out.astype(np.float32), (5, 5), 0)


def recolor(rgb, mask, target):
    if mask is None:
        return rgb
    hue, s_scale, v_scale = target
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV).astype(np.float32)
    hsv[..., 0] = np.mod(hue + np.clip(hsv[..., 0] - BLUE_H, -12, 12), 180.0)  # clip: fringe px stay near target
    hsv[..., 1] = np.clip(hsv[..., 1] * s_scale, 0, 255)
    hsv[..., 2] = np.clip(hsv[..., 2] * v_scale, 0, 255)
    new = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2RGB).astype(np.float32)
    h0 = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)[..., 0].astype(np.float32)
    dist = np.abs(np.mod(h0 - hue + 90.0, 180.0) - 90.0)
    a = (mask * (dist > 20))[..., None]  # never re-apply to pixels that already have the target colour
    return (rgb.astype(np.float32) * (1 - a) + new * a).astype(np.uint8)


def affected(src, e, n, bg, col):
    """(overhead_frames_with_blue, wrist_frames_with_blue) over every 10th frame."""
    ov = src.frames(e, "overhead", n)
    wr = src.frames(e, "wrist", n)
    idx = range(0, n, 10)
    a = sum(mask_overhead(ov[i], bg, col) is not None for i in idx)
    b = sum((m := mask_wrist(wr[i], col)) is not None and m.sum() > 150 for i in idx)
    return a, b, ov, wr


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--split", required=True, choices=["train", "val"])
    p.add_argument("--src", default="record_tataro_v2_table_split")
    p.add_argument("--dst", default="record_tataro_v2fix_split")
    p.add_argument("--preview", action="store_true")
    p.add_argument("--min-frames", type=int, default=3, help="sampled frames with leftover blue to count as affected")
    a = p.parse_args()

    src_root = DATA / a.src / a.split
    src = Src(src_root)
    bg = cd.background()
    eps = sorted(int(e) for e in src.eps.episode_index)
    todo = []
    for e in eps:
        task = src.task(e)
        col = next((c for c in ("red", "green") if re.search(rf"\b{c}\b", task)), None)
        if col is None:
            continue
        n = len(src.rows(e))
        fo, fw, ov, wr = affected(src, e, n, bg, col)
        if fo >= a.min_frames or fw >= a.min_frames:
            todo.append((e, col, fo, fw))
            print(f"ep {e:3d} {col:5s}: leftover blue in {fo} overhead / {fw} wrist of {len(range(0, n, 10))} "
                  f"sampled frames", flush=True)
    print(f"{a.split}: {len(todo)} of {len(eps)} episodes need fixing", flush=True)

    if a.preview:
        out = DATA / a.dst / f"preview_{a.split}"
        out.mkdir(parents=True, exist_ok=True)
        for e, col, _, _ in todo:
            n = len(src.rows(e))
            ov, wr = src.frames(e, "overhead", n), src.frames(e, "wrist", n)
            ks = [0, n // 3, n // 2]
            rows = []
            for k in ks:
                o2 = recolor(ov[k], mask_overhead(ov[k], bg, col), TARGETS[col]["overhead"])
                w2 = recolor(wr[k], mask_wrist(wr[k], col), TARGETS[col]["wrist"])
                ws = lambda im: cv2.resize(im, (448, 336))  # noqa: E731
                rows.append(np.hstack([ov[k], o2, ws(wr[k]), ws(w2)]))
            cv2.imwrite(str(out / f"ep{e:03d}_{col}.jpg"), cv2.cvtColor(np.vstack(rows), cv2.COLOR_RGB2BGR))
        print(f"previews (overhead before|after, wrist before|after) in {out}")
        return

    from lerobot.datasets import LeRobotDataset
    from lerobot.datasets.dataset_tools import delete_episodes, merge_datasets

    work = DATA / a.dst / f"_work_{a.split}"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    info = json.load(open(src_root / "meta/info.json"))
    features = {k: v for k, v in info["features"].items()
                if k not in ("timestamp", "frame_index", "episode_index", "index", "task_index")}
    fixed = LeRobotDataset.create(repo_id=f"tataro/v2fix_{a.split}_fixed", fps=info["fps"], features=features,
                                  root=work / "fixed", robot_type=info.get("robot_type"), use_videos=True,
                                  image_writer_threads=8)
    try:
        for i, (e, col, _, _) in enumerate(todo):
            rows = src.rows(e)
            n = len(rows)
            ov, wr = src.frames(e, "overhead", n), src.frames(e, "wrist", n)
            task = src.task(e)
            for j, (_, r) in enumerate(rows.iterrows()):
                fixed.add_frame({
                    "observation.state": np.asarray(r["observation.state"], dtype=np.float32),
                    "action": np.asarray(r["action"], dtype=np.float32), "task": task,
                    "observation.images.overhead": recolor(ov[j], mask_overhead(ov[j], bg, col), TARGETS[col]["overhead"]),
                    "observation.images.wrist": recolor(wr[j], mask_wrist(wr[j], col), TARGETS[col]["wrist"]),
                })
            fixed.save_episode()
            print(f"[{i + 1}/{len(todo)}] ep {e} ({col}) recoloured, {n} frames", flush=True)
    finally:
        fixed.finalize()

    orig = LeRobotDataset(f"tataro/record_tataro_v2_table_{a.split}", root=src_root)
    kept = delete_episodes(orig, episode_indices=[t[0] for t in todo], output_dir=work / "kept",
                           repo_id=f"tataro/v2fix_{a.split}_kept")
    fixed = LeRobotDataset(f"tataro/v2fix_{a.split}_fixed", root=work / "fixed")
    out = DATA / a.dst / a.split
    if out.exists():
        shutil.rmtree(out)
    merged = merge_datasets([kept, fixed], output_repo_id=f"tataro/record_tataro_v2fix_{a.split}", output_dir=out,
                            concatenate_videos=False)
    metas = sorted((out / "meta/episodes").rglob("*.parquet"))  # merge_datasets meta file_index bug (see build_all)
    if len(metas) == 1:
        m = pd.read_parquet(metas[0])
        m["meta/episodes/chunk_index"] = 0
        m["meta/episodes/file_index"] = 0
        m.to_parquet(metas[0], index=False)
    print(f"done: {out}  {merged.num_episodes} episodes, {merged.num_frames} frames "
          f"(source {orig.num_episodes} eps, {orig.num_frames} frames)")


if __name__ == "__main__":
    main()
