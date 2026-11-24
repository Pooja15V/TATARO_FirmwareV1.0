#!/usr/bin/env bash
# Fix the leftover-blue recolouring in the v2 table splits, then train SmolVLA on the fixed train split.
# Same recipe as smolvla_tataro_v2 (2026-11-23): smolvla_base, batch 16, 20k steps, colour-only augmentation.
#   setsid nohup bash train_smolvla_v2fix.sh > ~/lerobot_outputs/smolvla_tataro_v2fix.log 2>&1 < /dev/null &
set -euo pipefail
export PYTHONPATH=
PY=~/miniconda3/envs/lerobot_smolvla/bin/python
HERE="$(cd "$(dirname "$0")" && pwd)"
DATA="$HERE/../datasets/tataro/record_tataro_v2fix_split"
cd "$HERE"

for split in val train; do
  if [ -f "$DATA/$split/meta/info.json" ]; then echo "== $split already built"; continue; fi
  echo "== fixing $split ($(date))"
  $PY fix_recolor.py --split "$split"
done

echo "== training ($(date))"
TFS='{"brightness":{"weight":1.0,"type":"ColorJitter","kwargs":{"brightness":[0.8,1.2]}},"contrast":{"weight":1.0,"type":"ColorJitter","kwargs":{"contrast":[0.8,1.2]}},"saturation":{"weight":1.0,"type":"ColorJitter","kwargs":{"saturation":[0.7,1.3]}},"sharpness":{"weight":1.0,"type":"SharpnessJitter","kwargs":{"sharpness":[0.5,1.5]}}}'
RM='{"observation.images.overhead":"observation.images.camera1","observation.images.wrist":"observation.images.camera2"}'
~/miniconda3/envs/lerobot_smolvla/bin/lerobot-train --policy.path=lerobot/smolvla_base --policy.push_to_hub=false \
  "--rename_map=$RM" --policy.scheduler_decay_steps=20000 \
  --dataset.repo_id=tataro/record_tataro_v2fix_train --dataset.root="$DATA/train" \
  --dataset.image_transforms.enable=true "--dataset.image_transforms.tfs=$TFS" \
  --output_dir="$HOME/lerobot_outputs/smolvla_tataro_v2fix" --job_name=smolvla_tataro_v2fix \
  --batch_size=16 --steps=20000 --save_freq=5000 --log_freq=200 --num_workers=4
echo "== done ($(date))"
