#!/usr/bin/env bash
# Build an Ollama model from a LoRA run: merge adapter -> f16 GGUF -> Q4_K_M -> `ollama create` with the training TEMPLATE.
#   scripts/build_gguf.sh <run_dir> <step|final|best> <ollama_name> [--dry-run]
#   e.g. scripts/build_gguf.sh brain/runs/sft_v1 400 gurdy-sft-400
# Everything runs on the CPU (no GPU): ~4 GB RAM peak per stage, ~10 min. Needs the project's Ollama server running
# for the last step (it listens on 127.0.0.1:11435, see README); the GPU is not used by any step here.
# Pinned: llama.cpp tag b12494 (source tarball sha256 below). Merge + convert run in gurdy2/brain-train:cuda (CPU),
# llama-quantize runs in gurdy2/gguf:b12494 (built from the same tarball by docker/gguf/Dockerfile, built on first use).
# <step>: the adapter in <run_dir> is the LAST saved checkpoint; the number must equal the step stored in
#   <run_dir>/state.pt (otherwise the script stops: it never builds a different step than you asked for).
#   `final` / `best` use <run_dir>/final or <run_dir>/best.
# Env: HF_MODELS (default ~/hf_models, must hold the base model dir), GGUF_WORK (default ~/gguf_build),
#      OLLAMA_BIN (~/ollama/bin/ollama), OLLAMA_HOST (127.0.0.1:11435), BASE_MODEL (Qwen3-1.7B).
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
[ $# -ge 3 ] || { sed -n 2,4p "$0"; exit 2; }
RUN=$(realpath "$1"); STEP=$2; NAME=$3; DRY=0; [ "${4:-}" = "--dry-run" ] && DRY=1
TAG=b12494; SHA=b40d84985b98330ca2d51c6eef3ead6e7ec1aae2af97f7452af708e5900e2fcb
HF_MODELS=${HF_MODELS:-$HOME/hf_models}; BASE_MODEL=${BASE_MODEL:-Qwen3-1.7B}
WORK=${GGUF_WORK:-$HOME/gguf_build}/$NAME
OLLAMA_BIN=${OLLAMA_BIN:-$HOME/ollama/bin/ollama}; export OLLAMA_HOST=${OLLAMA_HOST:-127.0.0.1:11435}
TRAIN_IMG=gurdy2/brain-train:cuda; Q_IMG=gurdy2/gguf:$TAG
MODELFILE=$PWD/brain/tools/Modelfile.sft
step() { echo; echo "== $*"; }
run() { if [ $DRY = 1 ]; then echo "[dry-run] $*"; else "$@"; fi; }

step "checks"
# Free disk: the WSL virtual disk lives on the Windows C: drive and grows into it; a full C: crashes WSL ("Bus error", 2026-11-18).
MIN_FREE_GB=${MIN_FREE_GB:-20}
free_gb() { df -BG --output=avail "$1" 2>/dev/null | tail -1 | tr -dc '0-9'; }
for d in "$HOME" /mnt/c; do
  [ -d "$d" ] || { echo "note: $d not mounted, free-space check skipped for it"; continue; }
  f=$(free_gb "$d")
  if [ -n "$f" ] && [ "$f" -lt "$MIN_FREE_GB" ]; then
    echo "refusing to start: only ${f} GB free on $d (need >= ${MIN_FREE_GB} GB; the build writes ~10 GB and WSL's disk lives on C:). Free space, or MIN_FREE_GB=<n> to override."; exit 1
  fi
done
[ -d "$RUN" ] || { echo "no such run dir: $RUN"; exit 1; }
[ -f "$MODELFILE" ] || { echo "missing $MODELFILE"; exit 1; }
[ -d "$HF_MODELS/$BASE_MODEL" ] || { echo "base model not found: $HF_MODELS/$BASE_MODEL"; exit 1; }
docker image inspect $TRAIN_IMG >/dev/null 2>&1 || { echo "missing image $TRAIN_IMG (docker compose build brain_train / see docker/brain_train)"; exit 1; }
docker ps --format '{{.Names}}' | grep -q . && echo "note: containers running: $(docker ps --format '{{.Names}}' | tr '\n' ' ') (CPU/RAM are shared; ~4 GB RAM needed)"
case "$STEP" in
  final|best) AD=$RUN/$STEP ;;
  *[!0-9]*|'') echo "step must be a number, 'final' or 'best'"; exit 2 ;;
  *) AD=$RUN
     SAVED=$(docker run --rm -e CUDA_VISIBLE_DEVICES= -v "$RUN":/run:ro --entrypoint python3 $TRAIN_IMG -c \
       "import torch;print(torch.load('/run/state.pt',map_location='cpu',weights_only=True)['step'])")
     [ "$SAVED" = "$STEP" ] || { echo "the adapter in $RUN is step $SAVED, not $STEP (only the last checkpoint is kept; use $SAVED, final or best)"; exit 1; } ;;
esac
[ -f "$AD/adapter_model.safetensors" ] && [ -f "$AD/adapter_config.json" ] || { echo "no adapter in $AD"; exit 1; }
echo "adapter: $AD  (step $STEP)  base: $HF_MODELS/$BASE_MODEL  work: $WORK"

step "1/5 llama.cpp $TAG source (sha256 checked)"
mkdir -p "$WORK"
TARBALL=$WORK/llama-$TAG.tar.gz
[ -f "$TARBALL" ] || { [ -f "$HOME/p5_gguf/llama.tar.gz" ] && cp "$HOME/p5_gguf/llama.tar.gz" "$TARBALL" || true; }
[ -f "$TARBALL" ] || run curl -fsSL -o "$TARBALL" https://github.com/ggml-org/llama.cpp/archive/refs/tags/$TAG.tar.gz
if [ $DRY = 0 ] || [ -f "$TARBALL" ]; then
  echo "$SHA  $TARBALL" | sha256sum -c - || { echo "tarball hash mismatch, deleting"; rm -f "$TARBALL"; exit 1; }
fi
[ $DRY = 1 ] || { [ -f "$WORK/llama.cpp/convert_hf_to_gguf.py" ] || { mkdir -p "$WORK/llama.cpp" && tar xzf "$TARBALL" -C "$WORK/llama.cpp" --strip-components=1; }; }

step "2/5 merge LoRA into the fp16 base (CPU)"
M=$WORK/merged
if [ -f "$M/model.safetensors" ] || [ -f "$M/model.safetensors.index.json" ]; then echo "already merged: $M"; else
  run mkdir -p "$M"
  run docker run --rm --memory 6g -e CUDA_VISIBLE_DEVICES= -e HF_HUB_OFFLINE=1 -v "$AD":/adapter:ro -v "$HF_MODELS":/hf:ro \
    -v "$M":/out --entrypoint python3 $TRAIN_IMG -c "
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import PeftModel
m = AutoModelForCausalLM.from_pretrained('/hf/$BASE_MODEL', torch_dtype=torch.float16)
m = PeftModel.from_pretrained(m, '/adapter').merge_and_unload()
m.save_pretrained('/out', safe_serialization=True)
AutoTokenizer.from_pretrained('/hf/$BASE_MODEL').save_pretrained('/out')
print('merged')"
fi

step "3/5 convert to f16 GGUF (convert_hf_to_gguf.py from $TAG, CPU)"
F16=$WORK/$NAME-f16.gguf
if [ -s "$F16" ]; then echo "already converted: $F16"; else
  run docker run --rm --memory 6g -e CUDA_VISIBLE_DEVICES= -v "$WORK":/work --entrypoint bash $TRAIN_IMG -c "
set -e; cd /work
pip install -q --target /work/pylib pyyaml tqdm >/dev/null 2>&1 || true      # gguf-py comes from the pinned source tree
PYTHONPATH=/work/pylib:/work/llama.cpp/gguf-py python3 llama.cpp/convert_hf_to_gguf.py merged --outtype f16 --outfile $NAME-f16.gguf"
fi

step "4/5 quantise to Q4_K_M (llama-quantize built from $TAG)"
Q=$WORK/$NAME-q4km.gguf
if [ -s "$Q" ]; then echo "already quantised: $Q"; else
  docker image inspect $Q_IMG >/dev/null 2>&1 || run docker build -t $Q_IMG -f docker/gguf/Dockerfile docker/gguf
  run docker run --rm --memory 6g -v "$WORK":/work $Q_IMG /work/$NAME-f16.gguf /work/$NAME-q4km.gguf Q4_K_M
fi

step "5/5 register in Ollama as '$NAME' ($OLLAMA_HOST)"
[ -x "$OLLAMA_BIN" ] || { echo "ollama binary not found: $OLLAMA_BIN (set OLLAMA_BIN)"; exit 1; }
# the Modelfile carries the exact training TEMPLATE (system + user + empty think block); only the FROM path changes
sed "s|^FROM .*|FROM ./$NAME-q4km.gguf|" "$MODELFILE" > "$WORK/Modelfile"
if [ $DRY = 1 ]; then echo "[dry-run] (cd $WORK && $OLLAMA_BIN create $NAME -f Modelfile)"; else
  curl -fs -m 3 "http://$OLLAMA_HOST/api/version" >/dev/null || { echo "Ollama is not running on $OLLAMA_HOST. Start it:
  OLLAMA_HOST=$OLLAMA_HOST OLLAMA_MODELS=~/ollama_models LD_LIBRARY_PATH=/usr/lib/wsl/lib $OLLAMA_BIN serve   (then re-run this command; finished stages are skipped)"; exit 1; }
  (cd "$WORK" && "$OLLAMA_BIN" create "$NAME" -f Modelfile)
  echo "done: $NAME. Check:  curl -s http://$OLLAMA_HOST/api/tags | grep -o '\"name\":\"$NAME[^\"]*'"
fi
