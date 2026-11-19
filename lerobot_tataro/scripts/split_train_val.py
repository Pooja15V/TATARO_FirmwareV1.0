#!/usr/bin/env python
"""Train/val split that keeps each real episode together with its synthetic recoloured copies.

The synthetic red/green episodes have exactly the same actions as their blue source. Putting the blue
source in train and its red copy in val would leak the trajectory into validation. So episodes with
identical actions form one group, and whole groups go to train or val. Groups are spread over the
table/colours by picking every k-th group (deterministic, --seed shuffles instead).

    conda activate lerobot
    python split_train_val.py                     # 10% val -> datasets/tataro/<name>_split/{train,val}
    python split_train_val.py --val-frac 0.15 --dry-run

Note: lerobot-train has no validation loop; use the val split with eval_offline.py
(python eval_offline.py --ckpt ... --repo-id tataro/<name>_val --root .../val --episodes ...).
"""

import argparse
import random
import re
from pathlib import Path

import numpy as np
import pandas as pd

DATA = Path(__file__).resolve().parent.parent / "datasets" / "tataro"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", default="record_cube001_20261106_all")
    p.add_argument("--val-frac", type=float, default=0.10)
    p.add_argument("--seed", type=int, default=None, help="random groups instead of every k-th")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    root = DATA / args.name
    data = pd.concat([pd.read_parquet(f) for f in sorted(root.glob("data/*/*.parquet"))])
    tasks = pd.read_parquet(root / "meta/tasks.parquet")
    task_of = {int(i): t for t, i in zip(tasks.index, tasks["task_index"])}

    groups: dict[bytes, list[int]] = {}
    colour = {}
    for e, d in data.groupby("episode_index"):
        a = np.stack(d.sort_values("frame_index")["action"].values).astype(np.float32)
        groups.setdefault(np.round(a, 5).tobytes(), []).append(int(e))
        m = re.search(r"\b(red|green|blue)\b", task_of[int(d.task_index.iloc[0])])
        colour[int(e)] = m.group(1) if m else "?"
    gl = sorted(groups.values(), key=min)
    n_eps = sum(len(g) for g in gl)
    print(f"{args.name}: {n_eps} episodes in {len(gl)} groups "
          f"({sum(len(g) > 1 for g in gl)} with synthetic copies)")

    n_val_groups = max(1, round(len(gl) * args.val_frac))
    if args.seed is not None:
        val_groups = random.Random(args.seed).sample(range(len(gl)), n_val_groups)
    else:
        step = len(gl) / n_val_groups
        val_groups = [int(step * i + step / 2) for i in range(n_val_groups)]
    val = sorted(e for i in val_groups for e in gl[i])
    train = sorted(e for i, g in enumerate(gl) if i not in val_groups for e in g)

    def count(eps):
        c = {}
        for e in eps:
            c[colour[e]] = c.get(colour[e], 0) + 1
        return c

    print(f"train {len(train)} episodes {count(train)}\nval   {len(val)} episodes {count(val)}  -> {val}")
    if args.dry_run:
        return

    from lerobot.datasets import LeRobotDataset
    from lerobot.datasets.dataset_tools import split_dataset

    out = DATA / f"{args.name}_split"
    ds = LeRobotDataset(f"tataro/{args.name}", root=root)
    res = split_dataset(ds, splits={"train": train, "val": val}, output_dir=out)
    for k, d in res.items():
        print(f"{k}: {d.num_episodes} episodes, {d.num_frames} frames -> {d.root}")


if __name__ == "__main__":
    main()
