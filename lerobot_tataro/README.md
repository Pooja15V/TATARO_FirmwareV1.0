# TATARO Lite6 × LeRobot

LeRobot plugin + scripts for the UFactory Lite6 (192.168.1.150) with Gripper Lite, wrist D435i
(`wrist`, 141722076951) and overhead D455 (`overhead`, 318122301267), teleoperated with an Android phone.

```
lerobot_robot_lite6/      pip-installed plugin  ->  --robot.type=lite6
  config_lite6.py         IP, home pose, speed limits, workspace box, cameras
  lite6.py                Robot class (Cartesian servo thread, safety clamps, gripper)
  phone_processor.py      phone clutch -> absolute TCP target
  pose_utils.py           pose conversions (orientation relative to "tool down")
scripts/
  teleoperate.py          drive the arm with the phone, nothing recorded
  record.py               record episodes into a LeRobot dataset (phone teleop)
  record_traj.py          record by replaying trajectories taught in UFactory Studio
  record_handguided.py    record pick-and-place: you hand-guide the grasp, the arm does the recorded motion
  recolor_dataset.py      synthetic red/green copies of blue-cube episodes
  build_all_dataset.py    recolour in parts + merge into record_cube001_20261106_all
  split_train_val.py      train/val split that keeps each episode with its synthetic copies
  eval_offline.py         compare a checkpoint's actions with recorded episodes (no robot)
  run_policy.py           run a trained policy on the arm (+ saves videos/CSV; auto-warps the overhead for table-view policies)
  place_cube.py           live overhead view: cube position in robot mm, zone square, demo-episode targets
  convert_table_view.py   copy a dataset with the overhead replaced by the 336x336 table view
  view_dataset.py         show a whole dataset (all episodes back to back) in one Rerun window
  make_v3_dataset.py      v3 training copy: state = gripper only, idle HOME start trimmed (videos hard-linked)
lerobot_robot_lite6/table_warp.py   ArUco table-view warp (markers 52/51/53)
datasets/                 git repo -> github.com/Pooja15V/TATARO_LeRobot_Dataset (Git LFS)
  tataro/                 the recordings + the training sets
```

Everything runs in the `lerobot` conda env: `conda activate lerobot`.

## Data format
| key | content |
|---|---|
| `action` (7) | absolute TCP target `ee.x, ee.y, ee.z` [m], `ee.wx, ee.wy, ee.wz` [rotvec rad, relative to gripper-pointing-down], `gripper.pos` (0 open / 1 closed) |
| `observation.state` (13) | `joint1..6.pos` [rad], measured `ee.*`, `gripper.pos` |
| `observation.images.wrist / .overhead` | 640×480 RGB @ 30 fps (AV1 video) |

The controller's TCP offset (currently 83.6 mm) applies, so `ee.*` is the fingertip point.

## 1. Teleop test (no recording)
Keep the E-stop in reach. The arm moves to HOME on start: joints `[-0.3,-11.8,47,0,58.8,-0.2]`°, TCP ≈ (200, 0, 250) mm.
```bash
cd lerobot_tataro/scripts
python teleoperate.py --no-cameras --max-speed 0.08     # first time: slow, arm only
python teleoperate.py --display                          # with cameras in Rerun
```
1. Connect the phone to the same network as the laptop. Open the printed `https://<ip>:4443` URL in Chrome and accept the certificate warning. Then tap **Start**.
2. **Calibrate the heading:** hold the phone flat, screen up, with its top edge pointing at robot +X (away from the base, toward the work area). Press **Move** once and release. That direction becomes robot +X, whatever the AR session's own orientation is.
3. Hold **Move** and the arm follows. Release and the arm holds still (clutch: re-grip and continue).
4. **A** opens the gripper, **B** closes it (or use the gripper toggle button).

Verify the axes without the arm: `python teleoperate.py --check-axes` (robot not connected). Push forward, left and up, and it should print +X, +Y and +Z.
`--no-rotation` gives translation only. `--scale 0.5` gives finer motion.

The phone teleop is our own `Lite6WebXRPhone` (reads the raw WebXR pose). LeRobot's built-in `Phone` uses the teleop package's accumulated pose, which is in the AR session's arbitrary-yaw frame, so +X pointed in a random direction.

## 2. Record demonstrations
```bash
python record.py --num-episodes 10 --display               # creates ~/lerobot_data/tataro/lite6_pick_place_bin
python record.py --num-episodes 10 --resume                # add more later
```
Each episode, the arm goes HOME, then you do the task (recorded), then a reset phase (not recorded) where you put the cube back.
Keys: **→** ends the episode early, **←** re-records it, **Esc** stops and saves.

Tips: 50+ consistent demos for ACT. Vary the cube position across the table. Finish each episode once the cube is in the bin and press **→** (don't idle).

## 2b. Record with a hand-guided grasp (current method)
```bash
python record_handguided.py --resume --num-episodes 10 --display   # dataset tataro/record_cube001_20261106
```
Per episode: the arm goes HOME. You place the cube, then hand-guide the open gripper around it (manual mode) and press Enter. The arm then does the recorded motion with no hands in view:
HOME → pregrasp 150 mm above the cube, offset so the cube is centred in the wrist camera (1 s hold) → descend → close → lift → around the base → over the bowl (128, −270) → release at 60 mm → up.
Long moves swing around the base (≥ 200 mm from the base axis) to avoid self-collision.
If the camera offset would put the pregrasp closer than 150 mm to the base, the gripper yaw is flipped by 180°. The Gripper Lite is symmetric, so the grasp is unchanged.
Cubes closer than 150 mm to the base are refused.
The cube colour is asked per episode (Enter = detected) and goes into the task text.

**Multi-cube scenes (pick by colour, for SmolVLA):**
```bash
python record_handguided.py --multi --num-episodes 10 --display   # new dataset tataro/record_multicube001_<date>
python record_handguided.py --multi --resume --repo-id tataro/record_multicube001_<date> --num-episodes 10
```
Put 2–3 cubes of different colours on the table, at least 6 cm apart, and type which colours are there (e.g. `rgb`, Enter = same as last time).
The script then prints **TARGET: the RED cube**. It picks the colour with the fewest episodes so far, so the colours stay balanced. Hand-guide the gripper to that cube, and the rest is the same as above.
Between episodes, put the picked cube back and move all the cubes to new spots, so the target isn't always in the same place relative to the others.
Don't recolour these episodes with `recolor_dataset.py`: it would recolour every blue cube in the scene.

## 2c. Zone demos + table view (fix for the 2026-11-10 failed arm tests)
The first ACT policy missed every real pick by 6–16 cm. There were three causes:
- **Too few demos for the area:** 77 demos spread over the whole table.
- **Background clutter:** the policy picked up on items on the left desk.
- **Moved camera:** the overhead camera had shifted about 10 px since the recording.

The fix:
```bash
python place_cube.py --zone 280 -60 125      # Rerun view: magenta square = zone, cube position in robot mm
python record_handguided.py --repo-id tataro/record_zone001_20261110 --zone 280 -60 125 --num-episodes 10 --display
python record_handguided.py --repo-id tataro/record_zone001_20261110 --zone 280 -60 125 --resume --num-episodes 10
python convert_table_view.py --src record_zone001_20261110 --src-root ~/lerobot_data/tataro/record_zone001_20261110
```
- `--zone CX CY HALF` refuses grasps outside the square (here x 155–405, y −185…65 mm; 28 of the old demos are inside it). Vary the cube's position and rotation within the zone.
- `convert_table_view.py` replaces the overhead image with a 336×336 top-down view of just the table, registered on markers 51/52/53 (`lerobot_robot_lite6/table_warp.py`). It is the same idea as `xarm_ros2/src/computer_vision/TATARO_CV_TableCropping_with_ARUCO_7x7.py`, but 2-D only, with the markers detected once per episode.
- `run_policy.py` applies the same warp live (detected once at start-up) when the policy was trained on the table view.
- Keep the left desk as it is during recording and testing.

## Datasets (`datasets/tataro/`)
To browse one: `python view_dataset.py` (default: v2), or `python view_dataset.py --dataset record_zone001_20261110`.

| dataset | episodes | frames | content |
|---|---|---|---|
| `record_cube001_20261106` | 77 | 53,297 | **original recording**: hand-guided pick → bowl on the real arm, 2026-11-06. 58 blue / 10 red / 9 green, cube all over the table |
| `record_cube001_20261106_all` | 193 | 132,727 | v1 training set: real 77 + synthetic 116 (blue 58 real; red 10 real + 58 synthetic; green 9 real + 58 synthetic) |
| `record_zone001_20261110` | 50 | 33,925 | **zone recording**, 2026-11-10: cube inside x 155–405, y −185…65 mm only. 24 green / 21 red / 5 blue |
| **`record_tataro_v2_table`** | 243 | 166,652 | **train on this (v2)**: zone (eps 0–49) + `_all` (eps 50–242), overhead = 336×336 table view |

All are LeRobot v3.0, 30 fps, 20–28 s per episode.
- In `_all`, episodes 0–76 are the real ones in their original order, and each synthetic episode has exactly the same actions as its blue source.
- The intermediate table-view sets (`record_cube001_20261106_all_table`, `record_zone001_20261110_table`) and the `_split` folders are only local. Rebuild them with `convert_table_view.py`, `merge_datasets` (then apply the meta file_index fix from `build_all_dataset.py`) and `split_train_val.py`.

`datasets/` is its own git repo, pushed to [Pooja15V/TATARO_LeRobot_Dataset](https://github.com/Pooja15V/TATARO_LeRobot_Dataset). Videos and parquet files are in Git LFS, and `.gitignore` admits only the four folders above.
```bash
git clone git@github.com:Pooja15V/TATARO_LeRobot_Dataset.git datasets   # needs git-lfs installed
cd datasets && git lfs push origin main && git push                       # upload LFS first; a plain push of GBs times out
```
The older `lite6_cube_traj` (16 replayed Studio trajectories) and `lite6_cube_traj_failures` (5) are no longer in the project. They are still in `~/lerobot_data/tataro/` and in the repo's git history.

Rebuild the combined dataset with `python build_all_dataset.py`. It writes the synthetic parts `record_cube001_20261106_synth_part00..05` next to the original (skipping parts already finished), then merges them into `_all`. The parts are not kept in the repo, so delete them after the merge.
`recolor_dataset.py --preview` writes real-vs-synthetic comparison images. The colour targets were measured on the real red and green cubes.
The mask leaves out the bowl (paler cyan, hue < 98), the fixed blue objects in the overhead view, and the bottom strip of the overhead image (people).
Known artefacts: a thin cyan rim on some cube edges, and occasionally a small tinted patch of bowl next to the cube.
ACT ignores the task text, so for ACT the synthetic data teaches it to grab a cube of any colour, not a chosen colour.

## 3. Train (ACT, RTX 4060)
Free the GPU first (kill Jupyter and Isaac).

**v2 (current): table view.** 218 train / 25 val episodes, about 5 step/s, so 60k steps take about 3.3 h:
```bash
lerobot-train \
  --dataset.repo_id=tataro/record_tataro_v2_table_train \
  --dataset.root=$PWD/../datasets/tataro/record_tataro_v2_table_split/train \
  --dataset.image_transforms.enable=true \
  "--dataset.image_transforms.tfs={brightness 0.8-1.2, contrast 0.8-1.2, saturation 0.7-1.3, sharpness 0.5-1.5}" \
  --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --output_dir=$HOME/lerobot_outputs/act_tataro_v2_table --job_name=act_tataro_v2_table \
  --batch_size=8 --steps=60000 --save_freq=10000 --num_workers=4 --trackio.enable=false
```
The `tfs` value above is shorthand. The real argument is JSON, for example `{"brightness":{"weight":1.0,"type":"ColorJitter","kwargs":{"brightness":[0.8,1.2]}}, ...}` with `SharpnessJitter` for sharpness.
Augmentation is colour only. Don't use LeRobot's default random shift: about 4 cm of image shift conflicts with absolute-position actions. Hue is also left out, to keep the cube colours.

Pause and resume:
- `kill -STOP <pid> $(pgrep -P <pid>)` freezes the run and `kill -CONT` continues it. A reboot loses a frozen run.
- To restart from the last checkpoint (every 10k steps): `--resume=true --config_path=$HOME/lerobot_outputs/act_tataro_v2_table/checkpoints/last/pretrained_model/train_config.json`.

v2 result (2026-11-10), offline HOME-frame test on cube positions not used in training, inside the zone:
- error 4 mm after 1 s, median 37 mm after 3.3 s;
- v1 was about 80 mm;
- episodes used in training: about 13 mm.

**v3 (2026-11-11): vision-driven.** On the arm, v2 overshot the cube by about 45 mm. It leaned on the arm pose in `observation.state`: from a slightly-ahead pose, each re-plan aimed further ahead. With the demo's arm pose, the live images alone aimed within 5 mm.
v3 is the same episodes and split as v2, with two changes:
- `observation.state` = gripper only. It is stored twice, because LeRobot turns a 1-value feature into a scalar.
- The idle wait at HOME (1–4 s) is cut to 0.2 s before the arm starts moving.
```bash
python make_v3_dataset.py --src record_tataro_v2_table_split/train --dst record_tataro_v3_split/train
python make_v3_dataset.py --src record_tataro_v2_table_split/val   --dst record_tataro_v3_split/val
lerobot-train --dataset.repo_id=tataro/record_tataro_v3_train --dataset.root=$PWD/../datasets/tataro/record_tataro_v3_split/train \
  (same image_transforms / ACT / batch / steps as v2) --output_dir=$HOME/lerobot_outputs/act_tataro_v3_vision --job_name=act_tataro_v3_vision
```
`run_policy.py` detects the 2-value state and feeds only the gripper.

**v1 (old):** whole table, raw overhead:
```bash
python split_train_val.py        # -> datasets/tataro/record_cube001_20261106_all_split/{train,val}
lerobot-train \
  --dataset.repo_id=tataro/record_cube001_20261106_all_train \
  --dataset.root=$PWD/../datasets/tataro/record_cube001_20261106_all_split/train \
  --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --output_dir=$HOME/lerobot_outputs/act_lite6_cube_all --job_name=act_lite6_cube_all \
  --batch_size=8 --steps=60000 --save_freq=10000 --num_workers=4 --trackio.enable=false
```
To train on all 193 episodes, use `--dataset.repo_id=tataro/record_cube001_20261106_all --dataset.root=$PWD/../datasets/tataro/record_cube001_20261106_all`.
Resume with `--resume=true --config_path=$HOME/lerobot_outputs/act_lite6_cube_all/checkpoints/last/pretrained_model/train_config.json`.

## 4. Run the policy on the arm
LeRobot 0.6: `lerobot-record` no longer takes `--policy.path`. Also, `lerobot-rollout` keeps only `*.pos`/`*.vel` robot features, so it drops this plugin's `ee.*` pose.
Use `run_policy.py`, which feeds the policy exactly the features it was trained with:
```bash
python run_policy.py --ckpt ~/lerobot_outputs/<run>/checkpoints/last/pretrained_model --episodes 3 --duration 40 --n-action-steps 30
python eval_offline.py --ckpt ...    # offline check against recorded episodes first (no robot; default dataset: _all)
```
`--n-action-steps 30` re-plans from fresh camera images every 1 s instead of every 3.3 s (ACT's chunk of 100). `--temporal-ensemble 0.01` instead predicts every step and averages the overlapping chunks, as in the ACT paper. This did not fix v2's overshoot.
The CSV is also saved when a run is stopped with Ctrl+C, the E-stop or a controller error.
Each episode is saved as an mp4 per camera plus a CSV in `~/lerobot_outputs/policy_runs/<time>/`.
The same safety clamps (workspace, jump, tilt, speed) apply to policy actions.

## Safety limits (config_lite6.py, overridable with `--robot.<field>=` in lerobot-* CLIs)
- Workspace x −0.12 to 0.45, y −0.37 to 0.37, z 0.005 to 0.40 m (widened 2026-11-10 to cover the hand-guided data). **Check z_min against your table height.**
- Targets closer than 150 mm to the base axis (in xy) are pushed radially outward, to avoid self-collision (`min_base_radius_m`).
- TCP speed 0.15 m/s and 90 °/s. Targets jumping more than 150 mm or 60° are rejected. Tool tilt is capped at 60° from vertical.
- Collision sensitivity 3. Any controller error stops streaming and raises, so clear it and restart.
- Homing from the folded zero pose raises J3/J5 first (J2 from zero once tripped C22).

## Gemini Robotics-ER 2 pick-and-place (`scripts/gemini_pick.py`)

Gemini plans and points; the script turns its points into robot mm and moves the arm (no learned policy).
System prompt: `scripts/gemini_system_prompt.txt` (edit freely). Needs `google-genai` (installed in the `lerobot`
env, 3.3.0) and `GEMINI_API_KEY` in the terminal. Model `gemini-robotics-er-2-preview` (v1beta API).

```bash
python scripts/gemini_pick.py --dry-run 36                                    # no arm: replays zone demo 36
python scripts/gemini_pick.py --task "Put the green cube in the bowl."        # real arm, Enter before every move
```

- Grasp position = overhead point -> snapped to the colour blob -> TableWarp -> table->robot map, minus a fixed
  bias (on 29 zone demos: within 5 mm of the demo grasp along the closing axis in 29/29, median 1.9 mm).
- Wrist camera (camera-centred hover as in `record_handguided.py`): cube rotation, cross-check (refuses if it
  disagrees with the overhead by > 25 mm), and the grasp check (it can't see a held cube, so it looks at the
  spot where the cube was).
- Refuses to start unless the controller TCP offset is 83.6 mm (fingertip poses, as in the dataset).
- Every run is saved to `~/lerobot_outputs/gemini_runs/<time>/` (images + log.json).
- `--local`: no Gemini / no API key. Finds the cube by colour (cube-shaped blob in the pick area, nearest to the
  base), then the same move_above -> grasp_and_lift -> check -> place_in_bowl -> go_home -> verify sequence with
  the same safety checks (up to 3 attempts). Dry-run on 6 zone demos: right cube in 5/5 visible cases (0.8-4.4 mm
  from the demo grasp); a cube under the gripper / arm shadow at HOME (x < ~240 mm near y = 0) is not found.
