#!/usr/bin/env python
"""Run a trained policy (ACT or SmolVLA) on the TATARO Lite6.

lerobot-rollout only keeps robot features named *.pos / *.vel, so it drops this plugin's TCP pose
(ee.x ... ee.wz). This script uses exactly the features the policy was trained with.

    conda activate lerobot
    python run_policy.py --ckpt ~/lerobot_outputs/act_lite6_cube_traj/checkpoints/010000/pretrained_model
    python run_policy.py --ckpt ... --episodes 3 --duration 70

    conda activate lerobot_smolvla        # SmolVLA needs transformers (not in the lerobot env)
    python run_policy.py --ckpt ~/lerobot_outputs/smolvla_tataro_v2/checkpoints/last/pretrained_model \
        --task "Pick up the blue cube and place it in the bin"

SmolVLA is language-conditioned: --task picks the cube colour. Use the exact training wording
("Pick up the red|green|blue cube and place it in the bin"). Its cameras were renamed for training
(overhead -> camera1, wrist -> camera2, see --rename-map); this script applies the same mapping.

The arm goes HOME, then the policy drives it for --duration seconds per episode.
Keys: Right arrow = end episode early, Esc = stop. KEEP THE E-STOP IN REACH.

Placement view (default; --no-place for the old terminal prompt): before each episode a live overhead window
marks the cube positions used in the demos (grey), the current target (white ring, 'n'/'p' to cycle; nearest the
central zone first) and the detected cube of the task colour (yellow cross + distance to target). Place the cube
on the target, press Enter/Space in the window to run. 'r'/'g'/'b' change the colour, 'q'/Esc quit. Tested targets
are coloured by outcome (green success, red missed, orange moved/unknown).
Every episode is saved as an mp4 per camera + a CSV of state/actions in --out (for showing results).

Outcome check (--no-check to skip): the overhead table view at HOME before and after each episode is compared
by cube_detector.check_episode -> success (cube of the task colour now in the bowl) / missed (still in place) /
moved (elsewhere on the table) / unknown (not visible). Saved as ep<N>_check.png + outcomes.json.
"""

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
import torch

from lerobot.policies.factory import get_policy_class, make_pre_post_processors
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
    p.add_argument("--no-check", action="store_true", help="skip the before/after overhead outcome check")
    p.add_argument("--no-view", action="store_true", help="no live camera window")
    p.add_argument("--guard-mm", type=float, default=20.0,
                   help="descent guard: below --guard-z the policy target must be within this many mm (xy) of the "
                        "detected cube, else z is held at --guard-z (a fingertip landing on the cube trips C31); "
                        "0 = off")
    p.add_argument("--guard-z", type=float, default=40.0, help="descent guard height [mm]")
    p.add_argument("--targets", default=None, metavar="X,Y;X,Y",
                   help="placement targets in robot mm instead of the demo positions, e.g. '258,-33;268,35' (demos)")
    p.add_argument("--no-place", action="store_true",
                   help="no live placement view before each episode (use the terminal Enter prompt instead)")
    p.add_argument("--out", default=str(Path.home() / "lerobot_outputs" / "policy_runs"))
    p.add_argument("--n-action-steps", type=int, default=None,
                   help="actions executed per predicted chunk before re-planning (ACT trained with 100 = every 3.3 s; "
                        "smaller re-plans more often from fresh images, e.g. 30 = every 1 s)")
    p.add_argument("--temporal-ensemble", type=float, default=None, metavar="COEFF",
                   help="ACT temporal ensembling (e.g. 0.01 as in the ACT paper): re-plan every step and average the "
                        "overlapping chunks; overrides --n-action-steps")
    p.add_argument("--rename-map", default=None,
                   help="JSON {robot obs key: policy key}. Default: for SmolVLA the training mapping "
                        "overhead->camera1, wrist->camera2; none for ACT")
    args = p.parse_args()

    ckpt = Path(args.ckpt).expanduser()
    if not (ckpt / "model.safetensors").exists():
        raise SystemExit(f"No checkpoint at {ckpt}")
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ptype = json.loads((ckpt / "config.json").read_text())["type"]
    policy = get_policy_class(ptype).from_pretrained(ckpt).to(dev).eval()
    if args.rename_map is not None:
        rename_map = json.loads(args.rename_map)
    elif ptype == "smolvla":
        rename_map = {"observation.images.overhead": "observation.images.camera1",
                      "observation.images.wrist": "observation.images.camera2"}
    else:
        rename_map = {}
    missing = [k for k in rename_map.values() if k not in policy.config.input_features]
    if missing:
        raise SystemExit(f"--rename-map targets {missing} are not policy inputs "
                         f"({list(policy.config.input_features)})")
    if args.temporal_ensemble is not None and ptype != "act":
        raise SystemExit("--temporal-ensemble is ACT-only")
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
    print(f"{ptype}: chunk {policy.config.chunk_size}, re-plan every {policy.config.n_action_steps} steps"
          + (f", task '{args.task}'" if ptype == "smolvla" else ""))
    pre, post = make_pre_post_processors(policy.config, pretrained_path=ckpt)

    robot = Lite6(Lite6Config(id="tataro_lite6", ip=args.ip, max_lin_speed_m_s=args.max_speed))
    features = combine_feature_dicts(
        hw_to_dataset_features(robot.action_features, ACTION),
        hw_to_dataset_features(robot.observation_features, OBS_STR),
    )
    # State layout: 13 = joints + TCP pose + gripper (v1/v2); 2 = gripper only, stored twice (v3,
    # make_v3_dataset.py: no arm pose, so the policy has to locate the cube from the cameras).
    st = policy.config.input_features.get("observation.state")
    st_shape = tuple(st.shape) if st is not None else None
    # Fine-tuning from a pretrained VLA (e.g. --policy.path=lerobot/smolvla_base) keeps the BASE model's
    # input_features in config.json (smolvla_base: state (6,)), while the model pads any state to max_state_dim.
    # The state size actually trained on is in the saved normalizer stats.
    for f in sorted(ckpt.glob("policy_preprocessor_step_*_normalizer_processor.safetensors")):
        from safetensors.torch import load_file

        stats = load_file(str(f))
        if "observation.state.mean" in stats:
            st_shape = tuple(stats["observation.state.mean"].shape)
    if st_shape == (2,):
        features["observation.state"] = {**features["observation.state"], "shape": (2,),
                                         "names": ["gripper.pos", "gripper.pos"]}
        print("State input: gripper only (vision-driven policy)")
    if st_shape is not None and tuple(features["observation.state"]["shape"]) != st_shape:
        raise SystemExit(f"observation.state: robot gives {features['observation.state']['shape']}, "
                         f"policy expects {st_shape}")

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
    ov = policy.config.input_features.get(rename_map.get("observation.images.overhead", "observation.images.overhead"))
    ov_size = int(ov.shape[-1]) if ov is not None else None
    # As with the state: a VLA fine-tuned from a base model keeps the BASE camera shapes in config.json
    # (smolvla_base: 256x256), so take the overhead size from the training dataset's info.json.
    try:
        tc = json.loads((ckpt / "train_config.json").read_text())
        info = json.loads((Path(tc["dataset"]["root"]) / "meta" / "info.json").read_text())
        ov_size = int(info["features"]["observation.images.overhead"]["shape"][1])
    except (OSError, KeyError, TypeError, ValueError):
        pass
    if ov_size is not None and ov_size != 640:
        from lerobot_robot_lite6.table_warp import TableWarp

        warp = TableWarp.from_image(robot.get_observation()["overhead"], size=ov_size)
        print(f"Table view: overhead warped to {warp.size}x{warp.size} on markers 51/52/53")
    # Outcome check works on the 336 px table view regardless of what the policy was trained on.
    check_warp = None
    if not args.no_check:
        try:
            import cube_detector as cd

            if warp is not None and warp.size == cd.SIZE:
                check_warp = warp
            else:
                from lerobot_robot_lite6.table_warp import TableWarp

                check_warp = TableWarp.from_image(robot.get_observation()["overhead"], size=cd.SIZE)
        except Exception as e:  # noqa: BLE001
            print(f"[check] outcome check disabled: {e}")

    def home_view():
        """Table view at HOME (a few frames first so the camera buffer holds a post-motion image)."""
        for _ in range(5):
            o = robot.get_observation()
            time.sleep(0.1)
        return check_warp(o["overhead"])

    VIEW_H = 480
    win = "TATARO policy - overhead table view | wrist"  # ASCII only (non-ASCII titles render black here)

    trail = []  # policy targets of the current episode (table-view px)

    def show(obs, action=None, t=None, cube=None, note=""):
        """Live window: overhead table view (as the policy sees it) | wrist camera, with the policy target."""
        if args.no_view:
            return
        ov = cv2.cvtColor(obs["overhead"], cv2.COLOR_RGB2BGR)
        k = VIEW_H / ov.shape[0]
        ov = cv2.resize(ov, (int(ov.shape[1] * k), VIEW_H))
        tables = check_warp is not None and ov.shape[0] == ov.shape[1]  # table view -> robot mm overlay works
        dist = None
        if tables:
            s = VIEW_H / cd.SIZE
            if cube is not None:
                px = tuple(int(v * s) for v in cd.to_table([cube])[0])
                cv2.drawMarker(ov, px, (0, 255, 255), cv2.MARKER_CROSS, 28, 2)
            if action is not None:
                axy = [action["ee.x"] * 1000, action["ee.y"] * 1000]
                px = tuple(int(v * s) for v in cd.to_table([axy])[0])
                trail.append(px)
                if len(trail) > 1:
                    cv2.polylines(ov, [np.array(trail, np.int32)], False, (255, 160, 0), 1)
                if cube is not None:
                    dist = float(np.linalg.norm(np.subtract(axy, cube)))
                closed = action.get("gripper.pos", 0) > 0.5
                cv2.circle(ov, px, 9, (0, 0, 255) if closed else (255, 160, 0), 2 if not closed else -1)
        wr = obs.get("wrist")
        tiles = [ov]
        if wr is not None:
            wr = cv2.cvtColor(wr, cv2.COLOR_RGB2BGR)
            tiles.append(cv2.resize(wr, (int(wr.shape[1] * VIEW_H / wr.shape[0]), VIEW_H)))
        im = np.hstack(tiles)
        lines = [args.task]
        if t is not None:
            lines.append(f"t={t:5.1f}s  TCP=({obs['ee.x'] * 1000:.0f},{obs['ee.y'] * 1000:.0f},{obs['ee.z'] * 1000:.0f}) mm"
                         f"  grip={'CLOSED' if obs['gripper.pos'] > 0.5 else 'open'}")
        if dist is not None:
            lines.append(f"policy target -> cube: {dist:.0f} mm")
        if note:
            lines.append(note)
        for i, txt in enumerate(lines):
            y = 24 + 24 * i
            cv2.putText(im, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(im, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imshow(win, im)
        cv2.waitKey(1)

    # Demo cube positions (= demo grasp xy, one per distinct real recording position) as placement targets.
    targets = None
    if not args.no_place and not args.no_view and check_warp is not None:
        g = np.load(cd.DATA / ".place_cube_cache.npz")["grasp"]
        uniq = []
        for q in g:
            if all(np.linalg.norm(q - u) > 15 for u in uniq):
                uniq.append(np.asarray(q, float))
        targets = sorted(uniq, key=lambda q: np.linalg.norm(q - np.array([280.0, -60.0])))
        if args.targets:
            targets = [np.array([float(v) for v in t.split(",")]) for t in args.targets.split(";") if t.strip()]
            print(f"Placement view: {len(targets)} custom targets " + ", ".join(f"({t[0]:.0f}, {t[1]:.0f})" for t in targets))
        else:
            print(f"Placement view: {len(targets)} demo cube positions as targets")
    place = {"i": 0, "tested": {}}
    OUT_COL = {"success": (0, 200, 0), "missed": (0, 0, 255)}

    def set_colour(c):
        old = next((x for x in ("red", "green", "blue") if x in args.task), None)
        if old and c != old:
            args.task = args.task.replace(old, c)
            print(f"Task: '{args.task}'")

    def place_cube():
        """Live placement window (arm at HOME). True = run the episode, False = stop."""
        S = 2  # draw at 672 px
        while True:
            o = robot.get_observation()
            tv = check_warp(o["overhead"])
            colour = next((c for c in cd.COLOURS if c in args.task), None)
            _, cube = cd.detect(tv, colour) if colour else (None, None)
            im = cv2.resize(cv2.cvtColor(tv, cv2.COLOR_RGB2BGR), None, fx=S, fy=S, interpolation=cv2.INTER_CUBIC)

            def px(xy):
                return tuple(int(v * S) for v in cd.to_table([xy])[0])

            for i, q in enumerate(targets):
                res = place["tested"].get(i)
                cv2.circle(im, px(q), 5, OUT_COL.get(res, (0, 140, 255)) if res else (160, 160, 160), -1)
            tq = targets[place["i"]]
            r = int(np.linalg.norm(np.subtract(px(tq + [12.5, 0]), px(tq))))  # 25 mm cube outline
            cv2.circle(im, px(tq), max(r, 8), (255, 255, 255), 2)
            cv2.drawMarker(im, px(tq), (255, 255, 255), cv2.MARKER_TILTED_CROSS, 10, 1)
            lines = [f"{args.task}",
                     f"target {place['i'] + 1}/{len(targets)} at ({tq[0]:.0f}, {tq[1]:.0f}) mm"
                     + (f"  [last: {place['tested'][place['i']]}]" if place["i"] in place["tested"] else "")]
            ok = False
            if cube is not None:
                cv2.drawMarker(im, px(cube), (0, 255, 255), cv2.MARKER_CROSS, 26, 2)
                d = float(np.linalg.norm(cube - tq))
                ok = d < 10
                cv2.line(im, px(cube), px(tq), (0, 255, 0) if ok else (0, 255, 255), 1)
                dx, dy = tq - cube
                lines.append(f"{colour} cube at ({cube[0]:.0f}, {cube[1]:.0f})  off by {d:.0f} mm"
                             f"  (move x {dx:+.0f}, y {dy:+.0f})" + ("  OK" if ok else ""))
            else:
                lines.append(f"{colour} cube NOT seen")
            lines.append("Enter/Space = run   n/p = next/prev target   r/g/b = colour   q/Esc = quit")
            for i, txt in enumerate(lines):
                y = 26 + 26 * i
                c = (0, 255, 0) if (i == 2 and ok) else (255, 255, 255)
                cv2.putText(im, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                cv2.putText(im, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 1)
            cv2.imshow(win, im)
            k = cv2.waitKey(30) & 0xFF
            if k in (13, 10, 32):
                return True
            if k in (27, ord("q")) or ev["stop"]:
                return False
            if k == ord("n"):
                place["i"] = (place["i"] + 1) % len(targets)
            elif k == ord("p"):
                place["i"] = (place["i"] - 1) % len(targets)
            elif k in (ord("r"), ord("g"), ord("b")):
                set_colour({"r": "red", "g": "green", "b": "blue"}[chr(k)])

    outcomes = []
    try:
        for ep in range(args.episodes):
            if ev["stop"]:
                break
            if ep > 0:
                robot.go_home()
            if targets is not None:
                print(f"\nPlace the cube on the target in the window, then press Enter there (q = quit).")
                if not place_cube():
                    print("Stopped from the placement view.")
                    break
            elif ep > 0:
                ans = input("Reset the cube, then press Enter (or type red/green/blue to change the colour) ... ")
                ans = ans.strip().lower()
                if ans in ("red", "green", "blue"):
                    set_colour(ans)
            before, cube_xy = None, None
            if check_warp is not None:
                before = home_view()
                colour = next((c for c in cd.COLOURS if c in args.task), None)
                _, cube_xy = cd.detect(before, colour) if colour else (None, None)
                print(f"[check] {colour} cube " + ("NOT seen on the table (check cube / colour word)" if cube_xy is None
                                                    else f"seen at ({cube_xy[0]:.0f}, {cube_xy[1]:.0f}) mm"))
            policy.reset()
            trail.clear()
            ev["next"] = False
            writers = {}
            rows = []
            print(f"\n=== Episode {ep + 1}/{args.episodes}: policy running for {args.duration:.0f}s "
                  f"(Right arrow = end, Esc = stop) ===")
            t0 = time.perf_counter()
            next_t = t0
            n = 0
            prev_wz = None
            last_sent, frozen_since = None, None
            guard_since, guarded = None, None
            try:
                while time.perf_counter() - t0 < args.duration and not ev["next"]:
                    obs = robot.get_observation()
                    # Yaw unwrap: the arm reports wz in (-pi, pi]. When the tool yaw passes +-180 deg the state
                    # jumps -3.14 -> +3.14, the policy (no episode crosses +-pi in training) answers with a ~180 deg
                    # turn, the driver rejects it as a rotation jump and holds the target for the rest of the
                    # episode (2026-11-23 run 162549 ep0). Keep the yaw fed to the policy continuous instead.
                    if prev_wz is not None:
                        obs["ee.wz"] = prev_wz + (obs["ee.wz"] - prev_wz + np.pi) % (2 * np.pi) - np.pi
                    prev_wz = obs["ee.wz"]
                    if warp is not None:
                        obs["overhead"] = warp(obs["overhead"])
                    frame = build_dataset_frame(features, obs, prefix=OBS_STR)
                    frame = {rename_map.get(k, k): v for k, v in frame.items()}
                    with torch.inference_mode():
                        batch = prepare_observation_for_inference(frame, dev, args.task, robot.name)
                        a = post(policy.select_action(pre(batch)))
                    action = make_robot_action(a, features)
                    # Descent guard (test harness only): the gripper must be over the cube before going low.
                    if args.guard_mm > 0 and cube_xy is not None:
                        off = float(np.hypot(action["ee.x"] * 1000 - cube_xy[0], action["ee.y"] * 1000 - cube_xy[1]))
                        low = action["ee.z"] * 1000 < args.guard_z
                        near_cube = np.hypot(obs["ee.x"] * 1000 - cube_xy[0], obs["ee.y"] * 1000 - cube_xy[1]) < 80
                        # Open or closed: closed fingers pressing on the cube trip C31 too (run 162532).
                        if low and near_cube and off > args.guard_mm:
                            action["ee.z"] = args.guard_z / 1000
                            guard_since = guard_since or time.perf_counter()
                            closing = action["gripper.pos"] > 0.5
                            if closing or time.perf_counter() - guard_since > 2.0:
                                guarded = round(off, 1)
                                why = "closed the gripper there" if closing else "did not correct within 2 s"
                                print(f"\n[guard] policy wants to descend {off:.0f} mm from the cube "
                                      f"(limit {args.guard_mm:.0f}), held at z {args.guard_z:.0f} mm and {why} -> "
                                      f"ending the episode (would have missed / hit the cube).")
                                action["gripper.pos"] = 0.0
                                robot.send_action(action)
                                break
                        else:
                            guard_since = None
                    sent = robot.send_action(action)
                    # Watchdog: the driver holds its previous target when it rejects one (jump/tilt limits) and
                    # may then keep rejecting everything, leaving the arm frozen. End the episode if so.
                    sp = np.array([sent[k] for k in ("ee.x", "ee.y", "ee.z")])
                    ap = np.array([action[k] for k in ("ee.x", "ee.y", "ee.z")])
                    now = time.perf_counter()
                    if last_sent is not None and np.allclose(sp, last_sent) and np.linalg.norm(ap - sp) > 0.03:
                        frozen_since = frozen_since or now
                        if now - frozen_since > 1.5:
                            print(f"\n[!] Driver has rejected the policy's targets for 1.5 s (arm holding at "
                                  f"{sp * 1000} mm, policy wants {ap * 1000} mm; see the [Lite6 SAFETY] "
                                  f"warning above). Ending this episode.")
                            break
                    else:
                        frozen_since = None
                    last_sent = sp
                    show(obs, action, time.perf_counter() - t0, cube_xy)
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
            if before is not None and colour is not None:
                robot.go_home()
                after = home_view()
                res, info = cd.check_episode(before, after, colour)
                cv2.imwrite(str(out_root / f"ep{ep}_check.png"),
                            cd.draw_check(before, after, colour, res, info, f"ep{ep}"))
                tgt = None
                if targets is not None:
                    place["tested"][place["i"]] = res
                    tgt = targets[place["i"]].round(1).tolist()
                outcomes.append({"ep": ep, "task": args.task, "colour": colour, "outcome": res,
                                 "duration_s": round(el, 1), "target": tgt, "guarded_off_mm": guarded, **info})
                (out_root / "outcomes.json").write_text(json.dumps(outcomes, indent=1))
                print(f"[check] episode {ep + 1}: {res.upper()}  {info}")
                if not args.no_view:
                    cv2.imshow(win, cd.draw_check(before, after, colour, res, info, f"ep{ep}"))
                    cv2.waitKey(1)
    except KeyboardInterrupt:
        print("\nStopped (Ctrl+C).")
    finally:
        try:
            robot.go_home()
        except Exception as e:  # noqa: BLE001
            print(f"go_home failed: {e}")
        robot.disconnect()
        listener.stop()
        cv2.destroyAllWindows()
        if outcomes:
            from collections import Counter

            c = Counter(o["outcome"] for o in outcomes)
            print(f"\n=== Outcomes: {len(outcomes)} episodes: " + ", ".join(f"{k} {v}" for k, v in c.most_common())
                  + f"  (success rate {c['success'] / len(outcomes):.0%}) ===")
            for o in outcomes:
                print(f"  ep{o['ep']} {o['colour']:5s} {o['outcome']}")
        print(f"Videos + CSV in {out_root}")


if __name__ == "__main__":
    main()
