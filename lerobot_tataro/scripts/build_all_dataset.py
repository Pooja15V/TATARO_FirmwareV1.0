#!/usr/bin/env python
"""Build one dataset with real + synthetic red/green/blue episodes.

1. recolour the blue-cube episodes of the real dataset to red and green, in parts of --part-size source
   episodes (each part is its own finalized dataset, so an interruption only loses the current part;
   re-running skips finished parts)
2. merge real + all parts into  <real>_all

    conda activate lerobot
    nohup python build_all_dataset.py > ../datasets/tataro/build_all.log 2>&1 &
"""

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pandas as pd

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "datasets" / "tataro"


def finished(root: Path, expected: int) -> bool:
    try:
        info = json.load(open(root / "meta/info.json"))
        return info["total_episodes"] == expected and any((root / "meta/episodes").rglob("*.parquet"))
    except Exception:  # noqa: BLE001
        return False


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--real", default="record_cube001_20261106")
    p.add_argument("--part-size", type=int, default=10)
    p.add_argument("--colors", nargs="+", default=["red", "green"])
    args = p.parse_args()

    real = DATA / args.real
    tasks = pd.read_parquet(real / "meta/tasks.parquet")
    task_of = {int(i): t for t, i in zip(tasks.index, tasks["task_index"])}
    data = pd.concat([pd.read_parquet(f) for f in sorted(real.glob("data/*/*.parquet"))])
    first = data.groupby("episode_index").task_index.first()
    blue = [int(e) for e, t in first.items() if re.search(r"\bblue\b", task_of[int(t)])]
    print(f"{real.name}: {data.episode_index.nunique()} episodes, {len(blue)} blue", flush=True)

    parts = []
    for k in range(0, len(blue), args.part_size):
        eps = blue[k:k + args.part_size]
        out = DATA / f"{args.real}_synth_part{k // args.part_size:02d}"
        parts.append(out)
        if finished(out, len(eps) * len(args.colors)):
            print(f"{out.name}: already done", flush=True)
            continue
        if out.exists():
            shutil.rmtree(out)
        print(f"{out.name}: recolouring source episodes {eps[0]}..{eps[-1]}", flush=True)
        subprocess.run([sys.executable, str(HERE / "recolor_dataset.py"), "--src-root", str(real),
                        "--out-root", str(out), "--episodes", *map(str, eps), "--colors", *args.colors],
                       check=True)
        if not finished(out, len(eps) * len(args.colors)):
            raise SystemExit(f"{out.name} incomplete")

    from lerobot.datasets import LeRobotDataset
    from lerobot.datasets.dataset_tools import merge_datasets

    out = DATA / f"{args.real}_all"
    if out.exists():
        shutil.rmtree(out)
    srcs = [LeRobotDataset(f"tataro/{args.real}", root=real)]
    srcs += [LeRobotDataset(f"tataro/{d.name}", root=d) for d in parts]
    print(f"merging {len(srcs)} datasets ({sum(d.num_episodes for d in srcs)} episodes) -> {out.name}", flush=True)
    merged = merge_datasets(srcs, output_repo_id=f"tataro/{out.name}", output_dir=out, concatenate_videos=False)
    # merge_datasets writes all episode metadata into one file but keeps the sources' meta file_index
    # (0..N), so split_dataset etc. look for meta/episodes/.../file-001.parquet. Point them all at file 0.
    metas = sorted((out / "meta/episodes").rglob("*.parquet"))
    if len(metas) == 1:
        e = pd.read_parquet(metas[0])
        e["meta/episodes/chunk_index"] = 0
        e["meta/episodes/file_index"] = 0
        e.to_parquet(metas[0], index=False)
    print(f"done: {out}  {merged.num_episodes} episodes, {merged.num_frames} frames", flush=True)


if __name__ == "__main__":
    main()
