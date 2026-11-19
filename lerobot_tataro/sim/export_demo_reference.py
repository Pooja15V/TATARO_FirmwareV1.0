#!/usr/bin/env python
"""Export what the sim comparison needs from the REAL recordings (run in the lerobot env: the videos are AV1).

    conda activate lerobot
    python export_demo_reference.py        # -> sim/demo_reference.npz, then run compare_sim_demos.py (system python3)

Saves: every real demo's cube position (robot mm, = TCP x/y when the gripper closed), its colour and dataset,
the bowl release point, one raw 640x480 overhead frame per recording day, the ArUco marker centres/corners found
in it, and the table-px -> robot-mm map (fitted on the demos, same as place_cube.py).
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "scripts"))
from recolor_dataset import Src  # noqa: E402

from lerobot_robot_lite6.table_warp import DICT, TableWarp  # noqa: E402

DS = HERE.parent / "datasets" / "tataro"
V2 = DS / "record_tataro_v2_table"  # 0-49 zone, 50-126 real cube001, 127-242 recoloured copies (same positions)
RAW = {"zone": DS / "record_zone001_20261110", "cube001": DS / "record_cube001_20261106"}


def main():
    eps = pd.concat(pd.read_parquet(f) for f in sorted(V2.glob("meta/episodes/*/*.parquet"))).sort_values("episode_index")
    data = pd.concat(pd.read_parquet(f, columns=["episode_index", "action"]) for f in sorted(V2.glob("data/*/*.parquet")))
    grasp, release, colour, src = [], [], [], []
    syn_grasp, syn_colour = [], []  # recoloured (synthetic) copies, episodes 127+
    for e, g in data.groupby("episode_index"):
        a = np.stack(g.action.values)
        if e > 126:
            c = int(np.argmax(a[:, 6] > 0.5))
            t = eps.loc[eps.episode_index == e, "tasks"].iloc[0][0]
            syn_grasp.append(a[c, :3] * 1000)
            syn_colour.append(next((k for k in ("red", "green", "blue") if k in t), "?"))
            continue
        c = int(np.argmax(a[:, 6] > 0.5))
        o = c + int(np.argmax(a[c:, 6] < 0.5))
        grasp.append(a[c, :3] * 1000)
        release.append(a[o, :3] * 1000)
        t = eps.loc[eps.episode_index == e, "tasks"].iloc[0][0]
        colour.append(next((k for k in ("red", "green", "blue") if k in t), "?"))
        src.append("zone" if e < 50 else "cube001")

    z = np.load(DS / ".place_cube_cache.npz")
    T2R, _ = cv2.findHomography(z["tab"], z["grasp"], cv2.RANSAC, 15)

    out = dict(grasp=np.array(grasp), release=np.array(release), colour=np.array(colour), src=np.array(src),
               T2R=T2R, syn_grasp=np.array(syn_grasp), syn_colour=np.array(syn_colour))
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), cv2.aruco.DetectorParameters())
    for name, root in RAW.items():
        img = Src(root).frames(0, "overhead", 1)[0]
        corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
        found = {int(i): corners[k][0] for k, i in enumerate(ids.ravel())}
        warp = TableWarp.from_image(img)
        out[f"img_{name}"] = img
        out[f"H_{name}"] = warp.H
        for mid in (51, 52, 53):
            out[f"m{mid}_{name}"] = found[mid]  # 4 corners, raw px
        print(f"{name}: overhead frame {img.shape}, markers {sorted(found)}")
    np.savez_compressed(HERE / "demo_reference.npz", **out)
    print(f"{len(grasp)} real demos + {len(syn_grasp)} recoloured copies -> {HERE / 'demo_reference.npz'}")


if __name__ == "__main__":
    main()
