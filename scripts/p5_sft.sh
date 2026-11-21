#!/usr/bin/env bash
# Phase 5.5 QLoRA SFT, detached in container gurdy3-sft. Start ONLY when brain/data/full/status.txt says DONE and
# `docker ps` shows no other GPU job. Extra args go to brain/train/sft.py (e.g. --epochs 1, --out brain/runs/sft_v1b).
set -euo pipefail
cd "$(dirname "$0")/.."
docker ps --format '{{.Names}}' | grep -q . && { echo "other containers running:"; docker ps --format '{{.Names}}'; exit 1; }
grep -q '^DONE' brain/data/full/status.txt || { echo "datagen not DONE:"; cat brain/data/full/status.txt; exit 1; }
docker run -d --name gurdy3-sft --gpus all --memory 6g --shm-size 1g \
  -e PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True -e HF_HUB_OFFLINE=1 \
  -v "$PWD":/home/ubuntu/gurdy2 -v "$HOME/hf_models":/hf:ro \
  --entrypoint bash gurdy2/brain-train:cuda -c \
  'python3 -u brain/train/sft.py --data brain/data/full --model /hf/Qwen3-1.7B --out brain/runs/sft_v1 "$@" 2>&1 | tee -a brain/runs/sft_v1.log' _ "$@"
echo started; echo "monitor: cat brain/runs/sft_v1/status.txt ; docker logs --tail 5 gurdy3-sft"
