#!/usr/bin/env python
"""Record LeRobot demonstrations on the TATARO Lite6 with Android-phone teleop.

    conda activate lerobot
    python record.py --num-episodes 10 --display
    python record.py --num-episodes 10 --resume          # add more episodes to the same dataset

Each episode: arm goes HOME -> you teleoperate the task (recorded) -> reset phase (not
recorded, teleop still live) to put the cube back.
Keyboard (laptop window focus not needed):
    Right arrow  end the current episode / reset phase early
    Left arrow   discard and re-record the current episode
    Esc          stop and save
Data goes to ~/lerobot_data/<repo-id>  (nothing is uploaded unless --push-to-hub).
"""

import argparse
from pathlib import Path

from tataro_common import add_common_args, make_phone, make_processors, make_robot

from lerobot.datasets import LeRobotDataset
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import combine_feature_dicts, hw_to_dataset_features
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.rerun_visualization import init_rerun
from lerobot.utils.utils import log_say

DEFAULT_TASK = "Pick up the cube and place it in the bin"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("--repo-id", default="tataro/lite6_pick_place_bin")
    p.add_argument("--root", default=None, help="default: ~/lerobot_data/<repo-id>")
    p.add_argument("--task", default=DEFAULT_TASK)
    p.add_argument("--num-episodes", type=int, default=10)
    p.add_argument("--episode-time", type=float, default=60.0, help="max seconds per episode")
    p.add_argument("--reset-time", type=float, default=20.0, help="max seconds for reset phase")
    p.add_argument("--resume", action="store_true", help="append to an existing dataset")
    p.add_argument("--push-to-hub", action="store_true")
    args = p.parse_args()
    root = Path(args.root or Path.home() / "lerobot_data" / args.repo_id)

    robot = make_robot(args)
    phone = make_phone()
    teleop_proc, robot_proc, obs_proc = make_processors(args)

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
            repo_id=args.repo_id,
            fps=args.fps,
            features=features,
            root=root,
            robot_type=robot.name,
            use_videos=True,
            image_writer_threads=4,
        )

    robot.connect()
    phone.connect()  # blocks until the phone calibrates (touch-and-move on the WebXR page)
    listener, events = init_keyboard_listener()
    if args.display:
        init_rerun(session_name="tataro_lite6_record")

    loop_kw = dict(
        robot=robot,
        events=events,
        fps=args.fps,
        teleop_action_processor=teleop_proc,
        robot_action_processor=robot_proc,
        robot_observation_processor=obs_proc,
        teleop=phone,
        single_task=args.task,
        display_data=args.display,
    )

    recorded = 0
    try:
        while recorded < args.num_episodes and not events["stop_recording"]:
            robot.go_home()
            teleop_proc.reset()
            log_say(f"Recording episode {dataset.num_episodes + 1}")
            print(f"\n=== Episode {recorded + 1}/{args.num_episodes} (dataset total {dataset.num_episodes}) — GO ===")
            record_loop(dataset=dataset, control_time_s=args.episode_time, **loop_kw)

            if events["rerecord_episode"]:
                log_say("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            recorded += 1
            print(f"Saved. Dataset now has {dataset.num_episodes} episodes.")

            if not events["stop_recording"] and recorded < args.num_episodes:
                log_say("Reset the environment")
                print(f"--- Reset phase (not recorded, max {args.reset_time:.0f}s, Right arrow to skip) ---")
                record_loop(control_time_s=args.reset_time, **loop_kw)
                events["exit_early"] = False
    finally:
        log_say("Stop recording")
        robot.disconnect()
        phone.disconnect()
        listener.stop()
        dataset.finalize()
        print(f"Dataset at {root}  ({dataset.num_episodes} episodes)")
        if args.push_to_hub:
            dataset.push_to_hub()


if __name__ == "__main__":
    main()
