#!/usr/bin/env python
"""Show where the policy would grasp vs the detected cube, on the overhead table view. No arm.

For each episode of a table-view split: the first overhead frame (arm at HOME) with
  yellow cross  = cube found by cube_detector.py (task colour; falls back to any cube colour -> "LABEL?")
  white circle  = where the demonstrator closed the gripper
  coloured line = the policy's predicted 50-step chunk from the recorded frame T s before the real grasp
                  (blue 1.5 s, green 1.0 s, red 0.5 s); the filled dot = where that chunk closes the gripper
Errors are printed and written on each tile. Output: one PNG per episode + a contact sheet.

    conda activate lerobot_smolvla
    python viz_policy_grasp.py --ckpt ~/lerobot_outputs/smolvla_tataro_v2/checkpoints/020000/pretrained_model
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import torch

import cube_detector as cd

BEFORE = {1.5: (255, 120, 0), 1.0: (0, 200, 0), 0.5: (0, 0, 255)}  # BGR
UP = 2  # draw at 2x (672 px)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ckpt", required=True)
    p.add_argument("--split", default="val", choices=["train", "val"])
    p.add_argument("--episodes", type=int, nargs="*", default=None)
    p.add_argument("--out", default=None, help="default: <ckpt>/../grasp_viz_<split>")
    p.add_argument("--data", default="record_tataro_v2_table",
                   help="dataset family under datasets/tataro: <data>_split/<split>, repo tataro/<data>_<split>")
    a = p.parse_args()

    from lerobot.datasets import LeRobotDataset
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    torch.manual_seed(0)
    ckpt = Path(a.ckpt).expanduser()
    out = Path(a.out) if a.out else ckpt.parent / f"grasp_viz_{a.split}"
    out.mkdir(parents=True, exist_ok=True)
    ptype = json.loads((ckpt / "config.json").read_text())["type"]
    pol = get_policy_class(ptype).from_pretrained(ckpt).to("cuda").eval()
    pre, post = make_pre_post_processors(pol.config, pretrained_path=ckpt)
    ds = LeRobotDataset(f"tataro/{a.data}_{a.split}", root=cd.DATA / f"{a.data}_split" / a.split)
    epi = np.array(ds.hf_dataset["episode_index"])
    act = torch.stack(list(ds.hf_dataset["action"])).numpy()
    cal = cd.load_cal()
    tiles, res = [], []

    def tp(xy_mm):  # robot mm -> drawing px
        return tuple(int(v) for v in (cd.to_table([xy_mm])[0] * UP))

    with torch.inference_mode():
        for e in (a.episodes if a.episodes else range(ds.num_episodes)):
            fr = np.where(epi == e)[0]
            g = np.where(act[fr, 6] > 0.5)[0]
            if len(g) == 0:
                continue
            gi = fr[g[0]]
            demo = act[gi, :2] * 1000
            f0 = ds[int(fr[0])]
            colour = next(c for c in cd.COLOURS if c in f0["task"])
            img = (f0["observation.images.overhead"].permute(1, 2, 0).numpy() * 255).clip(0, 255).astype(np.uint8)
            _, det = cd.detect(img, colour, cal)
            note = ""
            if det is None:  # single-cube scenes: any cube colour, flagged
                for c in cd.COLOURS:
                    _, det = cd.detect(img, c, cal)
                    if det is not None:
                        note = f"LABEL? cube looks {c}"
                        break
            vis = cv2.resize(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), None, fx=UP, fy=UP, interpolation=cv2.INTER_CUBIC)
            cv2.circle(vis, tp(demo), 14, (255, 255, 255), 2)
            if det is not None:
                cv2.drawMarker(vis, tp(det), (0, 255, 255), cv2.MARKER_CROSS, 26, 2)
            r = {"ep": e, "colour": colour, "note": note,
                 "det": None if det is None else det.round(1).tolist(), "demo": demo.round(1).tolist()}
            lines = []
            for T, col in BEFORE.items():
                k = int(round(T * ds.fps))
                f = ds[int(gi - k)]
                obs = {kk: v.unsqueeze(0) for kk, v in f.items() if kk.startswith("observation.")}
                obs["task"] = [f["task"]]
                c = post(pol.predict_action_chunk(pre(obs))).squeeze(0).float().cpu().numpy()
                cl = np.where(c[:, 6] > 0.5)[0]
                j = cl[0] if len(cl) else min(k, len(c) - 1)
                pred = c[j, :2] * 1000
                pts = np.array([tp(q) for q in c[:, :2] * 1000], np.int32)
                cv2.polylines(vis, [pts], False, col, 2)
                cv2.circle(vis, tp(pred), 7, col, -1)
                e_det = None if det is None else float(np.linalg.norm(pred - det))
                e_demo = float(np.linalg.norm(pred - demo))
                r[f"{T}s"] = {"pred": pred.round(1).tolist(), "err_det": e_det, "err_demo": round(e_demo, 1),
                              "closes": bool(len(cl))}
                lines.append(f"{T}s: vs cube {'-' if e_det is None else f'{e_det:.0f}'} mm, vs demo {e_demo:.0f} mm"
                             + ("" if len(cl) else " (no close)"))
            res.append(r)
            head = f"ep{e} '{colour}'" + (f"  {note}" if note else "")
            for i, t in enumerate([head] + lines):
                cv2.putText(vis, t, (10, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                cv2.putText(vis, t, (10, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255) if note and i == 0 else (255, 255, 255), 1)
            cv2.imwrite(str(out / f"ep{e:03d}.png"), vis)
            tiles.append(vis)
            print(f"ep {e:3d} {colour:5s} " + " | ".join(lines) + (f"  [{note}]" if note else ""), flush=True)

    (out / "results.json").write_text(json.dumps(res, indent=1))
    cols = 5
    th = [cv2.resize(t, (336, 336)) for t in tiles]
    th += [np.zeros_like(th[0])] * (-len(th) % cols)
    sheet = np.vstack([np.hstack(th[i:i + cols]) for i in range(0, len(th), cols)])
    cv2.imwrite(str(out / "contact_sheet.png"), sheet)
    for T in BEFORE:
        ed = [r[f"{T}s"]["err_det"] for r in res if r[f"{T}s"]["err_det"] is not None and not r["note"]]
        print(f"@{T}s: |policy - detected cube| median {np.median(ed):.1f} mm, max {np.max(ed):.1f} "
              f"({len(ed)} eps with label-consistent detection)")
    print(f"images in {out}")


if __name__ == "__main__":
    main()
