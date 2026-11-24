#!/usr/bin/env python
"""Policy grasp point vs detected cube position, offline (no arm, no cameras).

For each episode of a table-view dataset (overhead already warped to the 336x336 table view):
  det    = cube found by colour in the FIRST overhead frame (task colour), table px -> robot mm with the
           same homography place_cube.py uses (fitted on the original demos' grasps)
  demo   = where the demonstrator actually closed the gripper
  policy = where the policy's predicted chunk closes the gripper, from frames T s before the real grasp
           (teacher-forced: recorded images + state, like eval_grasp_point.py)
Prints |policy - det|, |policy - demo| and |det - demo| (the last one = detector + map error).

    conda activate lerobot_smolvla
    python eval_grasp_vs_detector.py --ckpt ~/lerobot_outputs/smolvla_tataro_v2/checkpoints/020000/pretrained_model
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
DATA = HERE.parent / "datasets" / "tataro"

HUE = {"red": lambda h: (h <= 8) | (h >= 168), "green": lambda h: (h >= 40) & (h <= 90),
       "blue": lambda h: (h >= 95) & (h <= 120)}


def table_to_robot():
    z = np.load(DATA / ".place_cube_cache.npz")
    H, _ = cv2.findHomography(z["tab"], z["grasp"], cv2.RANSAC, 15)
    return H


def detect(table_rgb: np.ndarray, colour: str):
    """Centre (table px) of the biggest cube-sized blob of `colour` in a 336x336 table view, or None."""
    hsv = cv2.cvtColor(table_rgb, cv2.COLOR_RGB2HSV)
    h, s, v = (hsv[..., i].astype(int) for i in range(3))
    # Cubes are pale in the warped table view (sat ~60-90, table ~3), so the raw-view threshold 110 misses them.
    m = (s > 40) & (v > 45) & HUE[colour](h)
    roi = np.zeros_like(m)
    roi[8:-8, 18:-8] = True  # left strip = desk clutter beyond the table edge (red/green junk)
    roi[220:, 255:] = False  # bowl / bin (bottom right)
    m = (m & roi).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats(m)
    c = [(st[i, 4], cen[i]) for i in range(1, n) if 40 < st[i, 4] < 900]  # 35 mm cube ~ 14 px -> ~200 px^2
    return max(c, key=lambda t: t[0])[1] if c else None


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ckpt", required=True)
    p.add_argument("--repo-id", default="tataro/record_tataro_v2_table_val")
    p.add_argument("--root", default=str(DATA / "record_tataro_v2_table_split" / "val"))
    p.add_argument("--episodes", type=int, nargs="*", default=None)
    p.add_argument("--before", type=float, nargs="+", default=[0.5, 1.0, 1.5])
    p.add_argument("--samples", type=int, default=3)
    p.add_argument("--save-crops", default=None, help="folder: save first overhead frame with detection drawn")
    a = p.parse_args()

    from lerobot.datasets import LeRobotDataset
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    torch.manual_seed(0)
    ckpt = Path(a.ckpt).expanduser()
    ptype = json.loads((ckpt / "config.json").read_text())["type"]
    pol = get_policy_class(ptype).from_pretrained(ckpt).to("cuda").eval()
    pre, post = make_pre_post_processors(pol.config, pretrained_path=ckpt)
    ds = LeRobotDataset(a.repo_id, root=a.root)
    T2R = table_to_robot()
    epi = np.array(ds.hf_dataset["episode_index"])
    act = torch.stack(list(ds.hf_dataset["action"])).numpy()
    eps = a.episodes if a.episodes else list(range(ds.num_episodes))
    C = pol.config.chunk_size
    if a.save_crops:
        Path(a.save_crops).mkdir(parents=True, exist_ok=True)

    rows = []
    with torch.inference_mode():
        for e in eps:
            fr = np.where(epi == e)[0]
            g = np.where(act[fr, 6] > 0.5)[0]
            if len(g) == 0:
                continue
            gi = fr[g[0]]
            demo = act[gi, :2] * 1000
            f0 = ds[int(fr[0])]
            colour = next(c for c in HUE if c in f0["task"])
            img = (f0["observation.images.overhead"].permute(1, 2, 0).numpy() * 255).clip(0, 255).astype(np.uint8)
            b = detect(img, colour)
            if b is None:
                print(f"ep {e:3d} {colour:5s}: cube not detected")
                continue
            det = cv2.perspectiveTransform(np.float32(b).reshape(1, 1, 2), T2R)[0, 0]
            if a.save_crops:
                vis = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                cv2.drawMarker(vis, (int(b[0]), int(b[1])), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
                cv2.imwrite(str(Path(a.save_crops) / f"ep{e:03d}.png"), vis)
            r = {"ep": e, "det_demo": float(np.linalg.norm(det - demo))}
            for T in a.before:
                f = ds[int(gi - round(T * ds.fps))]
                obs = {k: v.unsqueeze(0) for k, v in f.items() if k.startswith("observation.")}
                obs["task"] = [f["task"]]
                pts = []
                for _ in range(a.samples):
                    c = post(pol.predict_action_chunk(pre(dict(obs)))).squeeze(0).float().cpu().numpy()
                    cl = np.where(c[:, 6] > 0.5)[0]
                    pts.append(c[cl[0] if len(cl) else min(round(T * ds.fps), C - 1), :2] * 1000)
                pred = np.median(pts, axis=0)
                r[T] = (float(np.linalg.norm(pred - det)), float(np.linalg.norm(pred - demo)))
            rows.append(r)
            print(f"ep {e:3d} {colour:5s} det=({det[0]:4.0f},{det[1]:4.0f}) demo=({demo[0]:4.0f},{demo[1]:4.0f}) "
                  f"|det-demo| {r['det_demo']:5.1f}  " + "  ".join(
                      f"@{T}s |pol-det| {r[T][0]:5.1f} |pol-demo| {r[T][1]:5.1f}" for T in a.before), flush=True)

    if rows:
        print(f"\n{len(rows)} episodes, medians [mm]: |det-demo| {np.median([r['det_demo'] for r in rows]):.1f}")
        for T in a.before:
            print(f"  @{T}s  |policy-detector| {np.median([r[T][0] for r in rows]):.1f}   "
                  f"|policy-demo| {np.median([r[T][1] for r in rows]):.1f}")


if __name__ == "__main__":
    main()
