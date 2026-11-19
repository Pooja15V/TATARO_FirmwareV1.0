#!/usr/bin/env python
"""Make a "vision-driven" copy of a LeRobot dataset (v3 training set).

Two changes, nothing else (videos are hard-linked, not re-encoded):
  1. observation.state keeps only gripper.pos (stored twice: LeRobot turns a shape-(1,) feature into a scalar). With the arm pose in the input, ACT learned "keep going from
     where the arm is" and overshot the cube on the real arm (2026-11-11). Without it, position must come from the
     cameras (overhead table view shows the arm, wrist view shows the cube).
  2. The idle wait at HOME at the start of each episode (1-4.4 s, depends on when Enter was pressed) is cut to
     --lead frames before the arm starts moving, so the same HOME image no longer maps to random start times.

    python make_v3_dataset.py --src record_tataro_v2_table_split/train --dst record_tataro_v3_split/train
    python make_v3_dataset.py --src record_tataro_v2_table_split/val   --dst record_tataro_v3_split/val
"""

import argparse
import json
import os
import shutil
from pathlib import Path

import numpy as np
import pandas as pd

from lerobot.datasets.compute_stats import aggregate_stats, get_feature_stats
from lerobot.datasets.utils import serialize_dict

DATA = Path(__file__).resolve().parent.parent / "datasets" / "tataro"
NUMERIC = ["action", "observation.state", "timestamp", "frame_index", "episode_index", "index", "task_index"]
STAT_KEYS = ["min", "max", "mean", "std", "count", "q01", "q10", "q50", "q90", "q99"]


def stats_of(arr):
    arr = np.asarray(arr)
    return get_feature_stats(arr, axis=0, keepdims=arr.ndim == 1)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--src", required=True, help="dataset folder under datasets/tataro (or a path)")
    p.add_argument("--dst", required=True)
    p.add_argument("--lead", type=int, default=6, help="frames kept before the arm starts moving (6 = 0.2 s)")
    p.add_argument("--move-mm", type=float, default=3.0, help="TCP target change that counts as 'moving'")
    args = p.parse_args()
    src = Path(args.src) if Path(args.src).is_absolute() else DATA / args.src
    dst = Path(args.dst) if Path(args.dst).is_absolute() else DATA / args.dst
    if dst.exists():
        raise SystemExit(f"{dst} exists; delete it first")

    info = json.loads((src / "meta/info.json").read_text())
    fps = info["fps"]
    names = info["features"]["observation.state"]["names"]
    gi = names.index("gripper.pos")

    # ---- data: trim + reduce state
    files = sorted(src.glob("data/*/*.parquet"))
    parts = []
    for f in files:
        d = pd.read_parquet(f)
        d["_file"] = str(f.relative_to(src))
        parts.append(d)
    data = pd.concat(parts).sort_values(["episode_index", "frame_index"]).reset_index(drop=True)
    keep = np.zeros(len(data), bool)
    cut = {}
    for e, g in data.groupby("episode_index", sort=True):
        a = np.stack(g.action.values)
        moving = np.linalg.norm(a[:, :3] - a[0, :3], axis=1) * 1000 > args.move_mm
        k = max(0, int(np.argmax(moving)) - args.lead) if moving.any() else 0
        cut[int(e)] = k
        keep[g.index[k:]] = True
    data = data[keep].reset_index(drop=True)
    data["frame_index"] = data.groupby("episode_index").cumcount().astype(np.int64)
    data["timestamp"] = (data["frame_index"] / fps).astype(np.float32)
    data["index"] = np.arange(len(data), dtype=np.int64)
    data["observation.state"] = [np.asarray(s, np.float32)[[gi, gi]] for s in data["observation.state"]]  # LeRobot turns shape (1,) into a scalar

    for rel, g in data.groupby("_file"):
        out = dst / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        g.drop(columns="_file").to_parquet(out, index=False)

    # ---- per-episode meta
    eps = pd.concat(pd.read_parquet(f) for f in sorted(src.glob("meta/episodes/*/*.parquet")))
    eps = eps.sort_values("episode_index").reset_index(drop=True)
    vkeys = [k for k, v in info["features"].items() if v["dtype"] == "video"]
    ep_stats_all = []
    starts = data.groupby("episode_index")["index"].agg(["min", "max"])
    for i, row in eps.iterrows():
        e = int(row.episode_index)
        g = data[data.episode_index == e]
        n = len(g)
        eps.at[i, "length"] = n
        eps.at[i, "dataset_from_index"] = int(starts.loc[e, "min"])
        eps.at[i, "dataset_to_index"] = int(starts.loc[e, "max"]) + 1
        for vk in vkeys:
            eps.at[i, f"videos/{vk}/from_timestamp"] = float(row[f"videos/{vk}/from_timestamp"]) + cut[e] / fps
        st = {}
        for ft in NUMERIC:
            arr = np.stack(g[ft].values) if ft in ("action", "observation.state") else g[ft].to_numpy()
            st[ft] = stats_of(arr)
        ep_stats_all.append(st)
    # write stats columns (object columns: assign whole column at once to allow shape changes)
    for ft in NUMERIC:
        for k in STAT_KEYS:
            col = f"stats/{ft}/{k}"
            if col in eps.columns:
                eps[col] = pd.Series([np.asarray(st[ft][k]) for st in ep_stats_all], dtype=object)
    for vk in vkeys:  # image stats kept (cut frames are near-identical HOME frames); only the count changes
        eps[f"stats/{vk}/count"] = pd.Series([np.array([int(n)]) for n in eps["length"]], dtype=object)
    for f in sorted(src.glob("meta/episodes/*/*.parquet")):
        rel = f.relative_to(src)
        ci, fi = int(f.parent.name.split("-")[1]), int(f.stem.split("-")[1])
        sub = eps[(eps["meta/episodes/chunk_index"] == ci) & (eps["meta/episodes/file_index"] == fi)]
        (dst / rel).parent.mkdir(parents=True, exist_ok=True)
        sub.to_parquet(dst / rel, index=False)

    # ---- global meta
    shutil.copy(src / "meta/tasks.parquet", dst / "meta/tasks.parquet")
    agg = serialize_dict(aggregate_stats(ep_stats_all))
    old = json.loads((src / "meta/stats.json").read_text())
    for vk in vkeys:
        agg[vk] = old[vk]
    (dst / "meta/stats.json").write_text(json.dumps(agg, indent=4))
    info["features"]["observation.state"]["shape"] = [2]
    info["features"]["observation.state"]["names"] = ["gripper.pos", "gripper.pos.copy"]
    info["total_frames"] = int(len(data))
    (dst / "meta/info.json").write_text(json.dumps(info, indent=4))

    # ---- videos: hard links (same timestamps; episodes just start later via from_timestamp)
    for f in src.glob("videos/**/*.mp4"):
        out = dst / f.relative_to(src)
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.link(f, out)
        except OSError:
            shutil.copy(f, out)

    c = np.array(list(cut.values())) / fps
    print(f"{dst.name}: {len(eps)} episodes, {len(data)} frames (was {int(keep.size)}); "
          f"cut {c.min():.1f}-{c.max():.1f} s per episode (median {np.median(c):.1f}); state = [gripper.pos, gripper.pos]")


if __name__ == "__main__":
    main()
