#!/usr/bin/env python
"""Record LeRobot episodes by replaying trajectories saved on the Lite6 controller.

Teach a trajectory by hand-guiding the arm (UFactory Studio > Trajectory Recording) and save it
with a name. This script then replays it with the cameras running, so your hands are not in the
images. The arm does the motion and the script records joints, TCP pose, gripper and both cameras.

    conda activate lerobot
    python record_traj.py --list                                   # trajectories on the controller
    python record_traj.py --traj Scanning --task "Scan the table" --repo-id tataro/lite6_scan
    python record_traj.py --traj Scanning Pick2 Place2 --resume --gripper "close@46.3,open@59.1"
    python record_traj.py --traj Scanning PickRed --resume          # played back to back = 1 episode

Each episode: arm goes HOME -> recording starts -> trajectories play -> recording stops
-> next episode: put the cube at the taught spot, type its colour (r/g/b, Enter = detected).

Gripper: a .traj stores joints only. Drive the gripper during playback with the keys
    c = close   o = open
or give a fixed schedule (seconds from the start of playback) with --gripper. Key presses are
printed at the end in --gripper format, so the timing can be reused for the next episodes.
Other keys: Right arrow = stop this episode early, Left arrow = discard it, Esc = stop and save.

action(t) = measured TCP pose + gripper at t + --lookahead frames (what a policy has to command
through the plugin's servo filter to follow the same path).
"""

import argparse
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

from lerobot.datasets import LeRobotDataset
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame, combine_feature_dicts, hw_to_dataset_features
from lerobot_robot_lite6 import Lite6, Lite6Config
from lerobot_robot_lite6.pose_utils import EE_KEYS

DEFAULT_TASK = "Pick up the {color} cube and place it in the bin"
COLORS = {"r": "red", "g": "green", "b": "blue"}


def detect_cube_color(rgb: np.ndarray) -> str | None:
    """Dominant saturated red/green/blue in an RGB image (wrist camera at HOME sees the table)."""
    import cv2

    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h = hsv[..., 0][(hsv[..., 1] > 110) & (hsv[..., 2] > 50)]
    counts = {"red": int(((h < 10) | (h > 165)).sum()), "green": int(((h > 40) & (h < 85)).sum()),
              "blue": int(((h > 95) & (h < 130)).sum())}
    best = max(counts, key=counts.get)
    return best if counts[best] > 300 else None


def parse_gripper_schedule(s: str | None) -> list[tuple[float, float]]:
    if not s:
        return []
    out = []
    for item in s.split(","):
        what, at = item.strip().split("@")
        out.append((float(at), 1.0 if what.strip().lower().startswith("c") else 0.0))
    return sorted(out)


class Keys:
    def __init__(self):
        self.exit_early = self.rerecord = self.stop = False
        self.gripper_cmd: float | None = None  # pending command from c / o
        self._lock = threading.Lock()
        self.listener = None
        try:
            from pynput import keyboard

            def on_press(key):
                with self._lock:
                    if key == keyboard.Key.right:
                        self.exit_early = True
                    elif key == keyboard.Key.left:
                        self.rerecord = self.exit_early = True
                    elif key == keyboard.Key.esc:
                        self.stop = self.exit_early = True
                    elif getattr(key, "char", None) in ("c", "C"):
                        self.gripper_cmd = 1.0
                    elif getattr(key, "char", None) in ("o", "O"):
                        self.gripper_cmd = 0.0

            self.listener = keyboard.Listener(on_press=on_press)
            self.listener.start()
        except Exception as e:  # noqa: BLE001
            print(f"[warn] keyboard listener unavailable ({e}); use --gripper and Ctrl+C")

    def take_gripper(self):
        with self._lock:
            g, self.gripper_cmd = self.gripper_cmd, None
            return g


def set_gripper(arm, closed: bool, verbose: bool = False) -> bool:
    """Drive the Gripper Lite, verify the tool outputs and retry. Returns True on success."""
    want = [0, 1] if closed else [1, 0]
    for attempt in range(3):
        code = (arm.close_lite6_gripper if closed else arm.open_lite6_gripper)(sync=False)
        c2, out = arm.get_tgpio_output_digital()
        if code == 0 and c2 == 0 and list(out[:2]) == want:
            if verbose:
                print(f"    gripper {'CLOSE' if closed else 'OPEN'} ok", flush=True)
            return True
        time.sleep(0.05)
    print(f"    [!] gripper {'CLOSE' if closed else 'OPEN'} FAILED (code {code}, outputs {out[:2]}, "
          f"controller error {arm.error_code})", flush=True)
    return False


def wait_playback_done(arm, started_at: float, timeout: float, keys: Keys, tick) -> bool:
    """Call tick() until the controller finishes the motion. Returns False if aborted."""
    seen_moving = False
    idle_since = None
    while True:
        tick()
        if keys.exit_early:
            arm.set_state(4)  # stop motion
            time.sleep(0.3)
            arm.set_state(0)
            return False
        if arm.error_code:
            raise RuntimeError(f"controller error C{arm.error_code} during playback")
        st = arm.state
        if st == 1 and not seen_moving:
            print(f"    moving after {time.monotonic() - started_at:.1f}s", flush=True)
        if st == 1:
            seen_moving, idle_since = True, None
        else:
            idle_since = idle_since or time.monotonic()
            # done once it moved and has been still for 0.5 s (or never started within 5 s)
            if (seen_moving and time.monotonic() - idle_since > 0.5) or (
                not seen_moving and time.monotonic() - started_at > 5.0
            ):
                print(f"    finished after {time.monotonic() - started_at:.1f}s (moved={seen_moving}, state {st})", flush=True)
                return seen_moving
        if arm.error_code:
            raise RuntimeError(f"controller error {arm.error_code} during playback")
        if time.monotonic() - started_at > timeout:
            print("[warn] playback timeout")
            return False


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", default="192.168.1.150")
    p.add_argument("--list", action="store_true", help="list trajectories on the controller and exit")
    p.add_argument("--traj", nargs="+", help="trajectory name(s) on the controller, played in order")
    p.add_argument("--task", default=DEFAULT_TASK, help="{color} is replaced by the cube colour")
    p.add_argument("--color", default="ask", choices=["ask", "red", "green", "blue"],
                   help="cube colour; 'ask' = prompt before each episode (default = detected from wrist camera)")
    p.add_argument("--repo-id", default="tataro/lite6_cube_traj")
    p.add_argument("--root", default=None, help="default: ~/lerobot_data/<repo-id>")
    p.add_argument("--resume", action="store_true", help="append to an existing dataset")
    p.add_argument("--num-episodes", type=int, default=1)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--lookahead", type=int, default=3, help="action = state this many frames ahead")
    p.add_argument("--speed", type=int, default=1, choices=[1, 2, 4], help="playback speed multiplier")
    p.add_argument("--gripper", default=None, help='schedule, e.g. "close@14.2,open@31" (s from playback start)')
    p.add_argument("--pre", type=float, default=1.0, help="seconds recorded at HOME before playback")
    p.add_argument("--post", type=float, default=3.0, help="seconds recorded after playback ends")
    p.add_argument("--keep-unreleased", action="store_true", help="keep episodes that end with the gripper closed")
    p.add_argument("--no-home", action="store_true", help="don't move HOME before each episode")
    p.add_argument("--no-cameras", action="store_true")
    p.add_argument("--display", action="store_true", help="show cameras/state in Rerun")
    args = p.parse_args()

    cfg = Lite6Config(id="tataro_lite6", ip=args.ip, servo_on_connect=False, go_home_on_connect=False)
    if args.no_cameras:
        cfg.cameras = {}
    robot = Lite6(cfg)

    if args.list:
        from xarm.wrapper import XArmAPI

        arm = XArmAPI(args.ip)
        code, trajs = arm.get_trajectories()
        for t in trajs:
            print(f"  {t['name']:30s} {t['duration']:7.1f} s")
        arm.disconnect()
        return
    if not args.traj:
        raise SystemExit("--traj NAME [NAME ...] required (see --list)")

    root = Path(args.root or Path.home() / "lerobot_data" / args.repo_id)
    if args.resume:
        dataset = LeRobotDataset.resume(args.repo_id, root=root, image_writer_threads=4)
    else:
        if root.exists():
            raise SystemExit(f"{root} already exists. Use --resume to add episodes, or pick a new --repo-id.")
        features = combine_feature_dicts(
            hw_to_dataset_features(robot.action_features, ACTION),
            hw_to_dataset_features(robot.observation_features, OBS_STR),
        )
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id, fps=args.fps, features=features, root=root,
            robot_type=robot.name, use_videos=True, image_writer_threads=4,
        )

    print("Connecting Lite6 + cameras ...")
    robot.connect()
    arm = robot._arm
    code, trajs = arm.get_trajectories()
    known = {t["name"]: t["duration"] for t in trajs}
    missing = [t for t in args.traj if t not in known]
    if missing:
        robot.disconnect()
        raise SystemExit(f"Not on the controller: {missing}. Available: {list(known)}")
    total_dur = sum(known[t] for t in args.traj)

    keys = Keys()
    if args.display:
        from lerobot.utils.rerun_visualization import init_rerun, log_rerun_data

        init_rerun(session_name="tataro_lite6_traj")
    schedule = parse_gripper_schedule(args.gripper)
    dt = 1.0 / args.fps
    action_keys = list(robot.action_features)

    recorded = 0
    try:
        while recorded < args.num_episodes and not keys.stop:
            if arm.error_code:
                raise RuntimeError(f"controller error C{arm.error_code}: check the arm/scene, then restart the script")
            # open BEFORE homing so a held cube is never carried around
            set_gripper(arm, False)
            time.sleep(1.0)
            arm.stop_lite6_gripper()
            if not args.no_home:
                print("Moving HOME ...")
                robot._move_home_blocking()
            arm.set_mode(0)
            arm.set_state(0)
            robot._gripper_target = 0.0
            # cube colour -> task text
            color = args.color
            if color == "ask":
                det = detect_cube_color(robot.cameras["wrist"].async_read()) if "wrist" in robot.cameras else None
                while True:
                    ans = input(f"Place the cube at the taught spot. Colour r/g/b [Enter = {det or 'not detected'}]: ").strip().lower()
                    color = COLORS.get(ans[:1]) if ans else det
                    if color:
                        break
            task = args.task.format(color=color)
            print(f"Task: {task}")
            keys.exit_early = keys.rerecord = False
            keys.take_gripper()

            buf: deque = deque()  # (obs, observation_frame) waiting for their lookahead action
            n_frames = 0
            pressed: list[str] = []
            sched = list(schedule)
            stats = {"late": 0, "ticks": 0}
            prof = {"obs": 0.0, "write": 0.0, "tick": 0.0}
            t_ep = time.monotonic()
            t_play = [None]  # playback start (monotonic)
            next_t = [time.perf_counter()]

            def tick():
                nonlocal n_frames
                t_tick = time.perf_counter()
                now = time.monotonic()
                # gripper: schedule or keys
                if t_play[0] is not None:
                    while sched and now - t_play[0] >= sched[0][0]:
                        set_gripper(arm, sched.pop(0)[1] >= 0.5, verbose=True)
                g = keys.take_gripper()
                if g is not None:
                    set_gripper(arm, g >= 0.5, verbose=True)
                    if t_play[0] is not None:
                        pressed.append(f"{'close' if g else 'open'}@{now - t_play[0]:.1f}")
                t_a = time.perf_counter()
                obs = robot.get_observation()
                prof["obs"] += time.perf_counter() - t_a
                obs_frame = build_dataset_frame(dataset.features, obs, prefix=OBS_STR)
                buf.append((obs, obs_frame))
                if len(buf) > args.lookahead:
                    o, of = buf.popleft()
                    act = {k: obs[k] for k in action_keys}
                    dataset.add_frame({**of, **build_dataset_frame(dataset.features, act, prefix=ACTION), "task": task})
                    n_frames += 1
                    prof["write"] += time.perf_counter() - t_a
                    if args.display:
                        log_rerun_data(observation=o, action=act)
                # pace
                prof["tick"] += time.perf_counter() - t_tick
                next_t[0] += dt
                sleep = next_t[0] - time.perf_counter()
                stats["ticks"] += 1
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    stats["late"] += 1
                    if sleep < -0.1:
                        next_t[0] = time.perf_counter()

            print(f"\n=== Episode {recorded + 1}/{args.num_episodes} (dataset has {dataset.num_episodes}) "
                  f"- recording, playing {args.traj}. c/o gripper, ->/<-/Esc ===")
            t_end = time.monotonic() + args.pre
            while time.monotonic() < t_end and not keys.exit_early:
                tick()
            ok = True
            for name in args.traj:
                if keys.exit_early:
                    break
                # Right after a playback ends the controller rejects loads for a moment (-1): wait and retry.
                for attempt in range(5):
                    arm.set_mode(0)
                    arm.set_state(0)
                    t_w = time.monotonic() + 1.0
                    while time.monotonic() < t_w:
                        tick()
                    code = arm.load_trajectory(name, wait=True)
                    if code == 0:
                        break
                    print(f"    load {name} returned {code}, retrying ({attempt + 1}/5)", flush=True)
                if code != 0:
                    raise RuntimeError(f"load_trajectory({name}) failed: {code}")
                arm.set_mode(0)
                arm.set_state(0)
                code = arm.playback_trajectory(times=1, wait=False, double_speed=args.speed)
                if code != 0:
                    raise RuntimeError(f"playback_trajectory({name}) failed: {code}")
                start = time.monotonic()
                if t_play[0] is None:
                    t_play[0] = start
                print(f"  playing {name} (playback code {code}, state {arm.state}) ...", flush=True)
                ok = wait_playback_done(arm, start, known[name] / args.speed * 1.5 + 30, keys, tick) and ok
            t_end = time.monotonic() + args.post
            while time.monotonic() < t_end and not keys.exit_early:
                tick()
            # flush lookahead buffer: remaining frames get the final state as action
            last = buf[-1][0] if buf else None
            while buf:
                o, of = buf.popleft()
                act = {k: last[k] for k in action_keys}
                dataset.add_frame({**of, **build_dataset_frame(dataset.features, act, prefix=ACTION), "task": task})
                n_frames += 1

            wall = time.monotonic() - t_ep
            late = max(0.0, 1.0 - stats["ticks"] / (wall * args.fps))  # fraction of frames missing vs fps
            nt = max(1, stats["ticks"])
            print(f"  wall {wall:.1f}s, {nt / wall:.1f} fps, {nt} ticks; per tick ms: obs {prof['obs'] / nt * 1e3:.1f}, "
                  f"obs+write {prof['write'] / nt * 1e3:.1f}, whole tick {prof['tick'] / nt * 1e3:.1f}")
            released = robot.read_gripper_state() < 0.5
            if not released and not args.keep_unreleased:
                print("Discarded: the gripper never opened, so the cube was not released "
                      "(--keep-unreleased to keep such episodes).")
                dataset.clear_episode_buffer()
                keys.rerecord = False
                set_gripper(arm, False)  # drop the cube now, before homing
                time.sleep(1.0)
                arm.stop_lite6_gripper()
            elif keys.rerecord:
                print("Discarded (Left arrow).")
                dataset.clear_episode_buffer()
                keys.rerecord = False
            else:
                dataset.save_episode()
                recorded += 1
                print(f"Saved episode: {n_frames} frames ({n_frames / args.fps:.1f}s), dropped {late:.1%}"
                      f"{'' if ok else '  [playback was cut short]'}. Dataset now has {dataset.num_episodes}.")
            if pressed:
                print(f'  gripper keys this episode:  --gripper "{",".join(pressed)}"')
            if late > 0.05:
                print("  [warn] >5% frames dropped vs --fps; close other programs using CPU/cameras.")

            if recorded < args.num_episodes and not keys.stop:
                print("Reset the scene for the next episode.")
    except KeyboardInterrupt:
        print("\nInterrupted - discarding the unfinished episode.")
        dataset.clear_episode_buffer()
        try:
            arm.set_state(4)
            time.sleep(0.3)
            arm.set_state(0)
        except Exception:  # noqa: BLE001
            pass
    finally:
        robot.disconnect()
        if keys.listener:
            keys.listener.stop()
        dataset.finalize()
        print(f"Dataset at {root}  ({dataset.num_episodes} episodes)")


if __name__ == "__main__":
    main()
