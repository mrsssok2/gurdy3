#!/usr/bin/env bash
# Phase 5 SFT v2 (OOD fixes): base Qwen3-1.7B -> LoRA, ~400 steps, data brain/data/v2, saves every 50 steps.
# Same settings as sft_v1 (r 32 / alpha 64, lr 2e-4 cosine, effective batch 16, gradient checkpointing, max-len 3328), only the data differs.
# ONE command:   scripts/p5_sft_v2.sh
# Re-running the same command RESUMES from brain/runs/sft_v2 (state.pt every 50 steps). Extra args are passed to brain/train/sft.py.
# Monitor: cat brain/runs/sft_v2/status.txt ; docker logs --tail 5 gurdy3-sft2          Stop: docker rm -f gurdy3-sft2 (resumable)
# Expected: ~60 s/step (v1 measured 8-9 h / 525 steps incl. a slow start) -> roughly 6.5-7.5 h; 3.3 GiB VRAM, <= 6 GiB RAM.
set -euo pipefail
cd "$(dirname "$0")/.."
OLL=http://127.0.0.1:11435
[ -s brain/data/v2/train_all.jsonl ] || { echo "missing brain/data/v2 (python3 -m brain.data_gen.generate_v2 / merge_v2)"; exit 1; }
# training needs the GPU alone: unload models held by this project's Ollama (keep_alive 30 min), then check the free VRAM
for m in $(curl -s -m 3 $OLL/api/ps 2>/dev/null | grep -o '"name":"[^"]*"' | cut -d'"' -f4); do
  curl -s -m 10 $OLL/api/generate -d "{\"model\":\"$m\",\"keep_alive\":0}" >/dev/null && echo "unloaded $m from Ollama"
done
sleep 3
USED=$(/usr/lib/wsl/lib/nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
[ "$USED" -lt 2500 ] || { echo "GPU already uses $USED MiB (need < 2500): stop other GPU jobs (Ollama: pkill -f 'ollama serve'; docker ps; Windows apps)"; exit 1; }
if docker ps --format '{{.Names}}' | grep -qv '^gurdy3-sft2$' && [ -n "$(docker ps --format '{{.Names}}' | grep -v '^gurdy3-sft2$')" ]; then
  echo "other containers running:"; docker ps --format '{{.Names}}'; exit 1; fi
docker ps --format '{{.Names}}' | grep -qx gurdy3-sft2 && { echo "gurdy3-sft2 already running"; exit 0; }
docker rm -f gurdy3-sft2 >/dev/null 2>&1 || true
mkdir -p brain/runs
docker run -d --name gurdy3-sft2 --gpus all --memory 6g --shm-size 1g \
  -e PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True -e HF_HUB_OFFLINE=1 \
  -v "$PWD":/home/ubuntu/gurdy2 -v "$HOME/hf_models":/hf:ro \
  --entrypoint bash gurdy2/brain-train:cuda -c \
  'python3 -u brain/train/sft.py --data brain/data/v2 --train-file train_all.jsonl --val-file val_all.jsonl --model /hf/Qwen3-1.7B \
     --out brain/runs/sft_v2 --epochs 1 --max-steps 400 --save-every 50 "$@" 2>&1 | tee -a brain/runs/sft_v2.log' _ "$@"
echo "started gurdy3-sft2"; echo "monitor: cat brain/runs/sft_v2/status.txt ; docker logs --tail 5 gurdy3-sft2"
