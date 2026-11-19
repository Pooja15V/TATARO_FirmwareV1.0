#!/usr/bin/env python
"""Copy a dataset with the overhead camera replaced by the marker-registered top-down TABLE view.

The table warp (lerobot_robot_lite6.table_warp) is detected on the first frame of every episode (markers 51/52/53)
and applied to all its frames, so episodes recorded before/after the overhead camera was bumped line up.
Everything else (wrist images, state, actions, tasks) is copied unchanged. The source is never modified.

    conda activate lerobot
    python convert_table_view.py --src record_cube001_20261106_all                  # -> ..._all_table
    python convert_table_view.py --src record_zone001_20261110 --src-root ~/lerobot_data/tataro/record_zone001_20261110
    python convert_table_view.py --src record_cube001_20261106 --preview            # sample images only
"""

import argparse
import json
import re
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from recolor_dataset import Src  # noqa: E402

from lerobot_robot_lite6.table_warp import OUT_PX, MarkersNotFound, TableWarp  # noqa: E402

DATA = HERE.parent / "datasets" / "tataro"
CAM = "observation.images.overhead"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--src", required=True, help="dataset name (tataro/<name>)")
    p.add_argument("--src-root", default=None, help=f"default: {DATA}/<src>")
    p.add_argument("--out-root", default=None, help="default: next to the source, <src>_table")
    p.add_argument("--episodes", type=int, nargs="+", default=None)
    p.add_argument("--preview", action="store_true", help="write a few warped frames and stop")
    args = p.parse_args()

    src_root = Path(args.src_root).expanduser() if args.src_root else DATA / args.src
    src = Src(src_root)
    eps = args.episodes or sorted(int(e) for e in src.eps.episode_index)

    if args.preview:
        out = src_root.parent / f"{args.src}_table_preview"
        out.mkdir(exist_ok=True)
        for e in eps[:: max(1, len(eps) // 6)][:6]:
            raw = src.frames(e, "overhead", 1)[0]
            w = TableWarp.from_image(raw)
            cv2.imwrite(str(out / f"ep{e:03d}.jpg"), cv2.cvtColor(np.hstack([cv2.resize(raw, (448, 336)), w(raw)]),
                                                               cv2.COLOR_RGB2BGR))
        print(f"preview in {out}")
        return

    from lerobot.datasets import LeRobotDataset

    info = json.load(open(src_root / "meta/info.json"))
    features = {k: v for k, v in info["features"].items()
                if k not in ("timestamp", "frame_index", "episode_index", "index", "task_index")}
    features[CAM] = {**features[CAM], "shape": [OUT_PX, OUT_PX, 3]}
    features[CAM].pop("info", None)  # video codec info is regenerated
    out_root = Path(args.out_root).expanduser() if args.out_root else src_root.parent / f"{src_root.name}_table"
    if out_root.exists():
        raise SystemExit(f"{out_root} exists; delete it or pass --out-root")
    ds = LeRobotDataset.create(repo_id=f"tataro/{out_root.name}", fps=info["fps"], features=features, root=out_root,
                               robot_type=info.get("robot_type"), use_videos=True, image_writer_threads=8)
    warp = None
    try:
        for i, e in enumerate(eps):
            rows = src.rows(e)
            over = src.frames(e, "overhead", len(rows))
            wrist = src.frames(e, "wrist", len(rows))
            try:
                warp = TableWarp.from_image(over[0])
            except MarkersNotFound as ex:
                if warp is None:
                    raise
                print(f"   episode {e}: {ex}; reusing the previous episode's warp")
            for j, (_, r) in enumerate(rows.iterrows()):
                ds.add_frame({"observation.state": np.asarray(r["observation.state"], np.float32),
                              "action": np.asarray(r["action"], np.float32),
                              "observation.images.wrist": wrist[j], CAM: warp(over[j]),
                              "task": src.task(e)})
            ds.save_episode()
            colour = (re.search(r"\b(red|green|blue)\b", src.task(e)) or [None, "?"])[1]
            print(f"[{i + 1}/{len(eps)}] episode {e} ({colour}): {len(rows)} frames", flush=True)
    finally:
        ds.finalize()
    print(f"done: {out_root}  ({ds.num_episodes} episodes)")


if __name__ == "__main__":
    main()
