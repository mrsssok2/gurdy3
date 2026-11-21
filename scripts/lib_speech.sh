# Source me (cwd = repo root). start_speech_service -> starts the speech service container `gurdy3_speech` (Whisper on the GPU when possible + Piper)
# on the compose network and waits until it answers; sets GURDY_SPEECH_URL. The dashboard server reaches it by container name.
start_speech_service() {
  source scripts/lib_voice.sh
  local svc; svc=$(voice_service); echo "starting the speech service ($svc: Whisper + Piper) ..."
  docker rm -f gurdy3_speech >/dev/null 2>&1 || true
  docker compose run -d -T --name gurdy3_speech "$svc" python3 -m brain.voice.speech_service --port 8770 >/dev/null
  for _ in $(seq 120); do
    docker exec gurdy3_speech python3 -c "import urllib.request;urllib.request.urlopen('http://127.0.0.1:8770/health',timeout=1)" >/dev/null 2>&1 && break
    docker ps -q -f name=gurdy3_speech | grep -q . || { echo "the speech service exited:"; docker logs gurdy3_speech 2>&1 | tail -15; return 1; }
    sleep 1
  done
  export GURDY_SPEECH_URL=http://gurdy3_speech:8770
}
