#!/usr/bin/env python
"""Run a trained policy (e.g. ACT) on the TATARO Lite6.

lerobot-rollout only keeps robot features named *.pos / *.vel, so it drops this plugin's TCP pose
(ee.x ... ee.wz). This script uses exactly the features the policy was trained with.

    conda activate lerobot
    python run_policy.py --ckpt ~/lerobot_outputs/act_lite6_cube_traj/checkpoints/010000/pretrained_model
    python run_policy.py --ckpt ... --episodes 3 --duration 70

The arm goes HOME, then the policy drives it for --duration seconds per episode.
Keys: Right arrow = end episode early, Esc = stop. KEEP THE E-STOP IN REACH.
Every episode is saved as an mp4 per camera + a CSV of state/actions in --out (for showing results).
"""

import argparse
import csv
import time
from pathlib import Path

import numpy as np
import torch

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import make_robot_action, prepare_observation_for_inference
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame, combine_feature_dicts, hw_to_dataset_features
from lerobot_robot_lite6 import Lite6, Lite6Config


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ckpt", required=True)
    p.add_argument("--task", default="Pick up the red cube and place it in the bin")
    p.add_argument("--episodes", type=int, default=1)
    p.add_argument("--duration", type=float, default=70.0)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--max-speed", type=float, default=0.4, help="TCP speed limit [m/s] (demos peak ~0.55)")
    p.add_argument("--ip", default="192.168.1.150")
    p.add_argument("--out", default=str(Path.home() / "lerobot_outputs" / "policy_runs"))
    p.add_argument("--n-action-steps", type=int, default=None,
                   help="actions executed per predicted chunk before re-planning (ACT trained with 100 = every 3.3 s; "
                        "smaller re-plans more often from fresh images, e.g. 30 = every 1 s)")
    p.add_argument("--temporal-ensemble", type=float, default=None, metavar="COEFF",
                   help="ACT temporal ensembling (e.g. 0.01 as in the ACT paper): re-plan every step and average the "
                        "overlapping chunks; overrides --n-action-steps")
    args = p.parse_args()

    ckpt = Path(args.ckpt).expanduser()
    if not (ckpt / "model.safetensors").exists():
        raise SystemExit(f"No checkpoint at {ckpt}")
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    policy = ACTPolicy.from_pretrained(ckpt).to(dev).eval()
    if args.n_action_steps:
        if not 1 <= args.n_action_steps <= policy.config.chunk_size:
            raise SystemExit(f"--n-action-steps must be 1..{policy.config.chunk_size}")
        policy.config.n_action_steps = args.n_action_steps  # read by reset()/select_action()
    if args.temporal_ensemble is not None:
        # ACT paper's temporal ensembling: predict a chunk every step and average all chunks' actions for "now"
        # (older predictions weigh more). Damps the drift where each re-plan from a slightly-ahead arm pose aims
        # further ahead.
        from lerobot.policies.act.modeling_act import ACTTemporalEnsembler

        policy.config.temporal_ensemble_coeff = args.temporal_ensemble
        policy.config.n_action_steps = 1
        policy.temporal_ensembler = ACTTemporalEnsembler(args.temporal_ensemble, policy.config.chunk_size)
    print(f"ACT: chunk {policy.config.chunk_size}, re-plan every {policy.config.n_action_steps} steps")
    pre, post = make_pre_post_processors(policy.config, pretrained_path=ckpt)

    robot = Lite6(Lite6Config(id="tataro_lite6", ip=args.ip, max_lin_speed_m_s=args.max_speed))
    features = combine_feature_dicts(
        hw_to_dataset_features(robot.action_features, ACTION),
        hw_to_dataset_features(robot.observation_features, OBS_STR),
    )
    # State layout: 13 = joints + TCP pose + gripper (v1/v2); 2 = gripper only, stored twice (v3,
    # make_v3_dataset.py: no arm pose, so the policy has to locate the cube from the cameras).
    st = policy.config.input_features.get("observation.state")
    if st is not None and tuple(st.shape) == (2,):
        features["observation.state"] = {**features["observation.state"], "shape": (2,),
                                         "names": ["gripper.pos", "gripper.pos"]}
        print("State input: gripper only (vision-driven policy)")
    if st is not None and tuple(features["observation.state"]["shape"]) != tuple(st.shape):
        raise SystemExit(f"observation.state: robot gives {features['observation.state']['shape']}, "
                         f"policy expects {st.shape}")

    from pynput import keyboard

    ev = {"next": False, "stop": False}

    def on_press(key):
        if key == keyboard.Key.right:
            ev["next"] = True
        elif key == keyboard.Key.esc:
            ev["stop"] = ev["next"] = True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    out_root = Path(args.out) / time.strftime("%Y%m%d_%H%M%S")
    out_root.mkdir(parents=True, exist_ok=True)
    import cv2

    print("Connecting Lite6 (arm goes HOME) ...")
    robot.connect()
    dt = 1.0 / args.fps

    # Policies trained on the table view (convert_table_view.py) expect a square overhead image: warp the live
    # overhead the same way, with the table markers detected once now (arm at HOME, markers unobstructed).
    warp = None
    ov = policy.config.input_features.get("observation.images.overhead")
    if ov is not None and tuple(ov.shape)[-2:] != (480, 640):
        from lerobot_robot_lite6.table_warp import TableWarp

        warp = TableWarp.from_image(robot.get_observation()["overhead"], size=int(ov.shape[-1]))
        print(f"Table view: overhead warped to {warp.size}x{warp.size} on markers 51/52/53")
    try:
        for ep in range(args.episodes):
            if ev["stop"]:
                break
            if ep > 0:
                robot.go_home()
                input("Reset the cube, then press Enter ...")
            policy.reset()
            ev["next"] = False
            writers = {}
            rows = []
            print(f"\n=== Episode {ep + 1}/{args.episodes}: policy running for {args.duration:.0f}s "
                  f"(Right arrow = end, Esc = stop) ===")
            t0 = time.perf_counter()
            next_t = t0
            n = 0
            try:
                while time.perf_counter() - t0 < args.duration and not ev["next"]:
                    obs = robot.get_observation()
                    if warp is not None:
                        obs["overhead"] = warp(obs["overhead"])
                    frame = build_dataset_frame(features, obs, prefix=OBS_STR)
                    with torch.inference_mode():
                        batch = prepare_observation_for_inference(frame, dev, args.task, robot.name)
                        a = post(policy.select_action(pre(batch)))
                    action = make_robot_action(a, features)
                    sent = robot.send_action(action)
                    for cam in robot.cameras:
                        img = obs[cam]
                        if cam not in writers:
                            h, w = img.shape[:2]
                            writers[cam] = cv2.VideoWriter(str(out_root / f"ep{ep}_{cam}.mp4"),
                                                           cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
                        writers[cam].write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    rows.append({"t": round(time.perf_counter() - t0, 3),
                                 **{f"obs.{k}": obs[k] for k in action},
                                 **{f"act.{k}": float(v) for k, v in action.items()},
                                 **{f"sent.{k}": float(v) for k, v in sent.items()}})
                    n += 1
                    if n % 30 == 0:
                        print(f"\r t={time.perf_counter() - t0:5.1f}s TCP=({obs['ee.x']:.3f},{obs['ee.y']:.3f},"
                              f"{obs['ee.z']:.3f}) grip={'CLOSED' if obs['gripper.pos'] > 0.5 else 'open  '}",
                              end="", flush=True)
                    next_t += dt
                    time.sleep(max(0.0, next_t - time.perf_counter()))
            finally:  # also on Ctrl+C / E-stop / controller error, so the run can be analysed
                for w in writers.values():
                    w.release()
                if rows:
                    with open(out_root / f"ep{ep}.csv", "w", newline="") as f:
                        wr = csv.DictWriter(f, fieldnames=list(rows[0]))
                        wr.writeheader()
                        wr.writerows(rows)
            el = time.perf_counter() - t0
            print(f"\nEpisode {ep + 1} done: {n} steps in {el:.1f}s ({n / el:.1f} Hz)")
    except KeyboardInterrupt:
        print("\nStopped (Ctrl+C).")
    finally:
        try:
            robot.go_home()
        except Exception as e:  # noqa: BLE001
            print(f"go_home failed: {e}")
        robot.disconnect()
        listener.stop()
        print(f"Videos + CSV in {out_root}")


if __name__ == "__main__":
    main()
