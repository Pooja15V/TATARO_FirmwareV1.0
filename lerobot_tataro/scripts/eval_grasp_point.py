"""Grasp-point check for a chunked policy (SmolVLA/ACT) on a LeRobot split, no robot.

For each episode: real grasp = first frame where action gripper > 0.5. From obs at T seconds before it,
predict one chunk; predicted grasp xy = chunk action where the chunk first closes the gripper
(or at the real-grasp offset if it never closes). Teacher-forced state (recorded), like the ACT v2 test.
"""
import argparse, json, time
from pathlib import Path
import numpy as np, torch
from lerobot.datasets import LeRobotDataset
from lerobot.policies.factory import get_policy_class, make_pre_post_processors

p = argparse.ArgumentParser()
p.add_argument("--ckpt", required=True)
p.add_argument("--root", required=True)
p.add_argument("--repo-id", required=True)
p.add_argument("--episodes", type=int, nargs="*", default=None)
p.add_argument("--before", type=float, nargs="+", default=[0.5, 1.0, 1.5])
p.add_argument("--samples", type=int, default=3)
p.add_argument("--swap", action="store_true", help="also predict with a different colour prompt")
a = p.parse_args()

torch.manual_seed(0)
ckpt = Path(a.ckpt).expanduser()
ptype = json.loads((ckpt / "config.json").read_text())["type"]
pol = get_policy_class(ptype).from_pretrained(ckpt).to("cuda").eval()
pre, post = make_pre_post_processors(pol.config, pretrained_path=ckpt)
ds = LeRobotDataset(a.repo_id, root=a.root)
epi = np.array(ds.hf_dataset["episode_index"])
act = torch.stack(list(ds.hf_dataset["action"])).numpy()
eps = a.episodes if a.episodes else list(range(ds.num_episodes))
C = pol.config.chunk_size
ZONE, R = np.array([0.28, -0.06]), 0.125
COL = ["red", "green", "blue"]

def chunk(obs):
    t = time.perf_counter()
    c = pol.predict_action_chunk(pre(obs))
    c = post(c).squeeze(0).float().cpu().numpy()
    torch.cuda.synchronize()
    return c, time.perf_counter() - t

rows, times = [], []
with torch.inference_mode():
    for e in eps:
        fr = np.where(epi == e)[0]
        g = np.where(act[fr, 6] > 0.5)[0]
        if len(g) == 0:
            continue
        gi = fr[g[0]]
        gxy = act[gi, :2]
        inzone = bool(np.all(np.abs(gxy - ZONE) <= R))  # square, as record_handguided.py --zone
        r = {"ep": e, "zone": inzone, "g": gxy}
        for T in a.before:
            k = int(round(T * ds.fps))
            f = ds[int(gi - k)]
            obs = {kk: v.unsqueeze(0) for kk, v in f.items() if kk.startswith("observation.")}
            obs["task"] = [f["task"]]
            errs = []
            for s in range(a.samples):
                c, dt = chunk(dict(obs))
                times.append(dt)
                cl = np.where(c[:, 6] > 0.5)[0]
                j = cl[0] if len(cl) else min(k, C - 1)
                errs.append(np.linalg.norm(c[j, :2] - gxy) * 1000)
            r[T] = (np.median(errs), np.std(errs), len(cl) > 0)
            if a.swap and T == a.before[0]:
                cur = next(cc for cc in COL if cc in f["task"])
                other = COL[(COL.index(cur) + 1) % 3]
                o2 = dict(obs); o2["task"] = [f["task"].replace(cur, other)]
                c2, _ = chunk(o2)
                r["swap"] = np.abs(c2[:, :3] - c[:, :3]).max() * 1000
        rows.append(r)
        print(f"ep {e:3d} {'zone' if inzone else 'out '} grasp=({gxy[0]*1000:.0f},{gxy[1]*1000:.0f}) "
              + " ".join(f"@{T}s {r[T][0]:5.1f}mm(sd{r[T][1]:.1f}{'' if r[T][2] else ',noclose'})" for T in a.before)
              + (f"  swap-diff {r['swap']:.1f}mm" if 'swap' in r else ""), flush=True)

print(f"\nchunk inference: median {np.median(times)*1000:.0f} ms, max {np.max(times)*1000:.0f} ms (chunk {C})")
for name, sel in [("zone", True), ("out-of-zone", False)]:
    rs = [r for r in rows if r["zone"] == sel]
    if rs:
        print(f"{name} ({len(rs)} eps): " + "  ".join(
            f"@{T}s median {np.median([r[T][0] for r in rs]):.1f} max {np.max([r[T][0] for r in rs]):.1f} mm"
            for T in a.before))
