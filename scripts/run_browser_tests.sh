#!/usr/bin/env bash
# Browser tests of the static demo (Connect to my robot + in-browser voice wiring) in headless Chromium, through Docker.
#   scripts/run_browser_tests.sh            builds the static site, starts two mock robot servers, runs web/tests/browser/test_demo_browser.py
# Needs the images gurdy2/brain:cpu and a Playwright image with the python package (built here from mcr.microsoft.com/playwright/python, ~1.5 GB, removed again with REMOVE_IMAGE=true).
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
OUT=$(mktemp -d); trap 'docker rm -f gb_robot >/dev/null 2>&1 || true; [ "${REMOVE_IMAGE:-false}" = true ] && docker image rm pwshot mcr.microsoft.com/playwright/python:v1.49.0-noble >/dev/null 2>&1 || true; rm -rf "$OUT"' EXIT
if ! docker image inspect pwshot >/dev/null 2>&1; then
  printf 'FROM mcr.microsoft.com/playwright/python:v1.49.0-noble\nRUN pip install --retries 10 -q playwright==1.49.0\n' > "$OUT/Dockerfile"; docker build -q -t pwshot "$OUT"
fi
python3 scripts/build_static_demo.py --out "$OUT/site" >/dev/null
python3 - "$OUT" <<'PY'
import math, struct, sys, wave
out = sys.argv[1]
for name, amp in (("mic_loud.wav", 0.12), ("mic_silent.wav", 0.0)):                       # 16 kHz mono, 6 s (the browser's fake microphone loops it)
    w = wave.open(f"{out}/{name}", "wb"); w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
    w.writeframes(b"".join(struct.pack("<h", int(amp * 32767 * math.sin(2 * math.pi * 220 * i / 16000))) for i in range(16000 * 6))); w.close()
PY
export GURDY_WEB_TOKEN=browser-test-token
docker rm -f gb_robot >/dev/null 2>&1 || true
docker run -d --name gb_robot -e GURDY_WEB_TOKEN -v "$PWD":/w -w /w -e PYTHONPATH=/w/gurdy_core:/w gurdy2/brain:cpu sh -c \
  "python3 -m web.api --mock --bind 0.0.0.0 --port 8090 --allowed-origin http://localhost:8000 & python3 -m web.api --mock --bind 0.0.0.0 --port 8091 --allowed-origin https://other.example & wait" >/dev/null
sleep 6
docker run --rm --network container:gb_robot -v "$PWD/web/tests/browser":/t -v "$OUT":/o -w /t \
  -e SITE_URL=http://localhost:8000 -e ROBOT_URL=http://localhost:8090 -e ROBOT_TOKEN="$GURDY_WEB_TOKEN" -e ROBOT_OTHER_ORIGIN_URL=http://localhost:8091 \
  -e MIC_WAV=/o/mic_loud.wav -e MIC_WAV_SILENT=/o/mic_silent.wav -e SHOT_DIR=/o pwshot bash -c "cd /o/site && (python3 -m http.server 8000 >/dev/null 2>&1 &) ; sleep 1; cd /t && python3 test_demo_browser.py"
