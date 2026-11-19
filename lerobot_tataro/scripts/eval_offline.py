#!/usr/bin/env python
"""Offline check of a trained policy: no robot needed.

Feeds the recorded camera images + state of dataset episodes to the policy, frame by frame, and
compares its predicted actions with what the arm actually did. Saves a plot per episode.

    python eval_offline.py --ckpt ~/lerobot_outputs/<run>/checkpoints/last/pretrained_model
    python eval_offline.py --ckpt ... --episodes 0 6 11 15
    python eval_offline.py --ckpt ... --repo-id tataro/record_cube001_20261106_all_val \
        --root ../datasets/tataro/record_cube001_20261106_all_split/val    # held-out split
    python eval_offline.py --ckpt ... --live      # watch it frame by frame in a Rerun window
"""

import argparse
from pathlib import Path

import numpy as np
import torch

from lerobot.datasets import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors

DATASETS = Path(__file__).resolve().parent.parent / "datasets"
NAMES = ["x [m]", "y [m]", "z [m]", "wx [rad]", "wy [rad]", "wz [rad]", "gripper"]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ckpt", required=True, help="…/checkpoints/<step>/pretrained_model")
    p.add_argument("--repo-id", default="tataro/record_cube001_20261106_all")
    p.add_argument("--root", default=None, help="default: lerobot_tataro/datasets/<repo-id>")
    p.add_argument("--episodes", type=int, nargs="+", default=None, help="default: one per taught pose")
    p.add_argument("--out", default=None, help="plot folder (default: next to the checkpoint)")
    p.add_argument("--live", action="store_true",
                   help="stream cameras, predicted vs recorded actions and the 3D TCP paths to a Rerun viewer")
    args = p.parse_args()

    ckpt = Path(args.ckpt).expanduser()
    root = Path(args.root).expanduser() if args.root else DATASETS / args.repo_id
    if not (ckpt / "model.safetensors").exists():
        raise SystemExit(f"No checkpoint at {ckpt} (not saved yet?)")
    out = Path(args.out or ckpt.parent / "offline_eval")
    out.mkdir(parents=True, exist_ok=True)
    dev = "cuda" if torch.cuda.is_available() else "cpu"

    policy = ACTPolicy.from_pretrained(ckpt).to(dev).eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=ckpt)
    ds = LeRobotDataset(args.repo_id, root=root)
    ep_idx = np.array(ds.hf_dataset["episode_index"])
    eps = args.episodes
    if eps is None:  # first episode of each distinct pick position
        eps, seen = [], []
        st = torch.stack(list(ds.hf_dataset["observation.state"])).numpy()
        for e in range(ds.num_episodes):
            s = st[ep_idx == e]
            close = np.where(np.diff(s[:, 12]) > 0)[0]
            if len(close) == 0:
                continue
            xy = s[close[0], 6:8]
            if all(np.linalg.norm(xy - q) > 0.03 for q in seen):
                seen.append(xy)
                eps.append(e)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    print(f"checkpoint {ckpt}\nepisodes {eps}\n")
    if args.live:
        import rerun as rr

        rr.init(f"eval {ckpt.parent.parent.parent.name} on {root.name}", spawn=True)
        rr.log("tcp_3d/base", rr.Points3D([[0, 0, 0]], radii=12, colors=[[0, 0, 0]], labels=["base"]), static=True)
        for j, n in enumerate(NAMES):
            rr.log(f"actions/{j}_{n.split()[0]}/recorded", rr.SeriesLines(colors=[[150, 150, 150]], widths=2.5,
                                                                         names="recorded"), static=True)
            rr.log(f"actions/{j}_{n.split()[0]}/policy", rr.SeriesLines(colors=[[220, 50, 50]], widths=1.5,
                                                                       names="policy"), static=True)
        step = 0
    for e in eps:
        frames = np.where(ep_idx == e)[0]
        policy.reset()
        pred, true = [], []
        with torch.inference_mode():
            for i in frames:
                f = ds[int(i)]
                obs = {k: v.unsqueeze(0) for k, v in f.items() if k.startswith("observation.")}
                obs["task"] = [f["task"]]
                a = post(policy.select_action(pre(obs)))
                pred.append(a.squeeze(0).cpu().numpy())
                true.append(f["action"].numpy())
                if args.live:
                    rr.set_time("frame", sequence=step)
                    rr.set_time("episode_time", duration=float(f["frame_index"]) / ds.fps)
                    step += 1
                    for k, v in f.items():
                        if k.startswith("observation.images."):
                            img = (v.permute(1, 2, 0).numpy() * 255).clip(0, 255).astype(np.uint8)
                            rr.log(f"camera/{k.split('.')[-1]}", rr.Image(img).compress(jpeg_quality=80))
                    for j, n in enumerate(NAMES):
                        rr.log(f"actions/{j}_{n.split()[0]}/recorded", rr.Scalars(float(true[-1][j])))
                        rr.log(f"actions/{j}_{n.split()[0]}/policy", rr.Scalars(float(pred[-1][j])))
                    err = float(np.linalg.norm(pred[-1][:3] - true[-1][:3]) * 1000)
                    rr.log("tcp_error_mm", rr.Scalars(err))
                    P, T = np.array(pred)[:, :3] * 1000, np.array(true)[:, :3] * 1000
                    rr.log("tcp_3d/recorded", rr.LineStrips3D([T], colors=[[150, 150, 150]], radii=1.5))
                    rr.log("tcp_3d/policy", rr.LineStrips3D([P], colors=[[220, 50, 50]], radii=1.5))
                    grip = lambda g: [[30, 160, 30]] if g > 0.5 else [[60, 60, 220]]  # noqa: E731
                    rr.log("tcp_3d/policy_now", rr.Points3D([P[-1]], radii=6, colors=grip(pred[-1][6])))
                    rr.log("tcp_3d/recorded_now", rr.Points3D([T[-1]], radii=6, colors=grip(true[-1][6])))
                    rr.log("info", rr.TextLog(f"episode {e}  frame {int(f['frame_index'])}  {f['task']}  "
                                              f"TCP error {err:.1f} mm"))
        pred, true = np.array(pred), np.array(true)
        pos_err = np.linalg.norm(pred[:, :3] - true[:, :3], axis=1) * 1000
        grip_acc = ((pred[:, 6] > 0.5) == (true[:, 6] > 0.5)).mean()
        print(f"episode {e:2d} ({len(frames)} frames, {ds[int(frames[0])]['task']}):\n"
              f"   TCP position error  mean {pos_err.mean():5.1f} mm   95% {np.percentile(pos_err, 95):5.1f} mm   "
              f"max {pos_err.max():5.1f} mm\n   gripper open/close correct {grip_acc:.1%}")

        t = np.arange(len(frames)) / ds.fps
        fig, axs = plt.subplots(7, 1, figsize=(10, 13), sharex=True)
        for j, ax in enumerate(axs):
            ax.plot(t, true[:, j], label="recorded (demo)", lw=2, color="#888")
            ax.plot(t, pred[:, j], label="policy", lw=1.2, color="#d33")
            ax.set_ylabel(NAMES[j])
        axs[0].legend(loc="upper right")
        axs[0].set_title(f"{ckpt.parent.name}: episode {e} - mean TCP error {pos_err.mean():.1f} mm, "
                         f"gripper {grip_acc:.0%}")
        axs[-1].set_xlabel("time [s]")
        fig.tight_layout()
        fig.savefig(out / f"episode_{e:02d}.png", dpi=80)
        plt.close(fig)
    print(f"\nplots in {out}")


if __name__ == "__main__":
    main()
