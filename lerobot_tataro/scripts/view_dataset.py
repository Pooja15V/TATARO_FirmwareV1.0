#!/usr/bin/env python
"""Show a whole LeRobot dataset (all episodes, back to back) in one Rerun window.

lerobot-dataset-viz shows a single episode. This streams the dataset's mp4 files to Rerun as they are (no decoding),
so all episodes load in seconds; scrub the "dataset_time" timeline to move through them.

    conda activate lerobot
    python view_dataset.py                                  # record_tataro_v2_table (243 episodes)
    python view_dataset.py --dataset record_zone001_20261110
    python view_dataset.py --dataset record_cube001_20261106_all --save ~/Desktop/all.rrd   # file instead of window

Panels: overhead + wrist video, TCP target x/y/z [mm] + gripper, episode number, task text per episode, and a map of
every episode's grasp point (robot mm; green/red/blue = cube colour, the current episode is the big dot).
"""

import argparse
import glob
from pathlib import Path

import numpy as np
import pandas as pd

DATASETS = Path(__file__).resolve().parent.parent / "datasets" / "tataro"
RGB = {"red": (230, 60, 60), "green": (60, 190, 80), "blue": (70, 110, 240)}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset", default="record_tataro_v2_table", help="folder in datasets/tataro (or a path)")
    p.add_argument("--save", default=None, help="write an .rrd file instead of opening the viewer")
    args = p.parse_args()

    root = Path(args.dataset).expanduser()
    if not root.exists():
        root = DATASETS / args.dataset
    import json

    info = json.loads((root / "meta" / "info.json").read_text())
    fps = info["fps"]
    cams = [k for k, v in info["features"].items() if v["dtype"] == "video"]
    eps = pd.concat(pd.read_parquet(f) for f in sorted(glob.glob(str(root / "meta/episodes/**/*.parquet"), recursive=True)))
    eps = eps.sort_values("episode_index").reset_index(drop=True)
    data = pd.concat(pd.read_parquet(f, columns=["episode_index", "frame_index", "action"])
                     for f in sorted(glob.glob(str(root / "data/**/*.parquet"), recursive=True)))
    data = data.sort_values(["episode_index", "frame_index"]).reset_index(drop=True)
    act = np.stack(data.action.values)

    import rerun as rr
    import rerun.blueprint as rrb

    rr.init(f"dataset {root.name}", spawn=args.save is None)
    if args.save:
        rr.save(str(Path(args.save).expanduser()))

    short = {c: c.split(".")[-1] for c in cams}
    # 1) the video files, once each (static)
    for c in cams:
        for f in sorted((root / "videos" / c).rglob("*.mp4")):
            rr.log(f"files/{short[c]}/{f.parent.name}/{f.stem}", rr.AssetVideo(path=f), static=True)

    # 2) per-frame columns
    ep_of = data.episode_index.to_numpy()
    starts = {int(e): i for i, e in enumerate(eps.episode_index)}
    t_global = np.arange(len(data)) / fps  # episodes back to back
    times = [rr.TimeColumn("dataset_time", duration=t_global)]
    for c in cams:
        fi = eps[f"videos/{c}/file_index"].to_numpy()[[starts[int(e)] for e in ep_of]]
        ci = eps[f"videos/{c}/chunk_index"].to_numpy()[[starts[int(e)] for e in ep_of]]
        t0 = eps[f"videos/{c}/from_timestamp"].to_numpy()[[starts[int(e)] for e in ep_of]]
        ts_ns = np.round((t0 + data.frame_index.to_numpy() / fps) * 1e9).astype(np.int64)
        refs = [f"files/{short[c]}/chunk-{a:03d}/file-{b:03d}" for a, b in zip(ci, fi)]
        rr.send_columns(f"camera/{short[c]}", indexes=times,
                        columns=rr.VideoFrameReference.columns(timestamp=ts_ns, video_reference=refs))
    for j, n in enumerate(["x", "y", "z"]):
        rr.send_columns(f"action/tcp_mm/{n}", indexes=times, columns=rr.Scalars.columns(scalars=act[:, j] * 1000))
    rr.send_columns("action/gripper", indexes=times, columns=rr.Scalars.columns(scalars=act[:, 6]))
    rr.send_columns("episode", indexes=times, columns=rr.Scalars.columns(scalars=ep_of.astype(float)))

    # 3) per-episode: task text + grasp map
    colour = [next((k for k in RGB if k in t[0]), "blue") for t in eps.tasks]
    grasp = []
    for e in eps.episode_index:
        a = act[ep_of == e]
        grasp.append(a[int(np.argmax(a[:, 6] > 0.5)), :2] * 1000)
    grasp = np.array(grasp)
    pts = np.c_[grasp[:, 1] * -1, grasp[:, 0] * -1]  # draw robot +x up, +y left (as seen from the overhead camera)
    rr.log("grasp_map/all", rr.Points2D(pts, colors=[RGB[c] for c in colour], radii=3,
                                         labels=[str(int(e)) for e in eps.episode_index]), static=True)
    rr.log("grasp_map/base", rr.Points2D([[0, 0]], colors=[(120, 120, 120)], radii=8, labels=["robot base"]),
           static=True)
    first = np.array([starts[int(e)] for e in eps.episode_index])
    first_row = np.searchsorted(ep_of, eps.episode_index.to_numpy())
    ep_times = [rr.TimeColumn("dataset_time", duration=t_global[first_row])]
    rr.send_columns("task", indexes=ep_times, columns=rr.TextLog.columns(
        text=[f"episode {int(e)}: {t[0]}  ({int(n)} frames, grasp at x {g[0]:.0f}, y {g[1]:.0f} mm)"
              for e, t, n, g in zip(eps.episode_index, eps.tasks, eps.length, grasp)]))
    rr.send_columns("grasp_map/current", indexes=ep_times, columns=[
        *rr.Points2D.columns(positions=pts[first], radii=np.full(len(first), 9.0)),
    ])

    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(
        rrb.Vertical(rrb.Horizontal(*[rrb.Spatial2DView(origin=f"camera/{short[c]}", name=short[c]) for c in cams]),
                     rrb.TimeSeriesView(origin="action/tcp_mm", name="TCP target [mm]"),
                     rrb.TimeSeriesView(origin="action/gripper", name="gripper (1 = closed)"),
                     row_shares=[3, 2, 1]),
        rrb.Vertical(rrb.Spatial2DView(origin="grasp_map", name="grasp points (robot mm, base at 0)"),
                     rrb.TextLogView(origin="task", name="episodes"),
                     rrb.TimeSeriesView(origin="episode", name="episode #"), row_shares=[3, 2, 1]),
        column_shares=[3, 2])))
    print(f"{root.name}: {len(eps)} episodes, {len(data)} frames ({len(data) / fps / 60:.0f} min) "
          + (f"saved to {args.save}" if args.save else "sent to the Rerun viewer"))


if __name__ == "__main__":
    main()
