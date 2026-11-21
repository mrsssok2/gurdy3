#!/usr/bin/env bash
# Mission control on the MOCK world (the Gazebo flat_lab layout, keyword planner). Usage: scripts/start_web.sh [lan] [https] [voice]
#   (no arg) reachable from this PC only: http://localhost:8080   (the microphone works on localhost)
#   lan      reachable from the Wi-Fi / hotspot (phone) -> also needs the Windows firewall rule, docs/phase7_phone.md
#   https    also https on :8443 with a local-CA certificate (the phone's microphone needs https; docs/phase7_voice.md)
#   tailscale  private remote access from anywhere via Tailscale Serve (docs/phase7_remote.md)
#   voice    start the speech service (Whisper on the GPU + Piper) -> hold-to-talk mic button and spoken replies
# Token: $GURDY_WEB_TOKEN if set, else a random one generated here and printed (never written to a file).
set -euo pipefail
cd "$(dirname "$0")/.."
LAN=false; HTTPS=false; VOICE=false; TAIL=false
for a in "$@"; do case "$a" in lan) LAN=true ;; https) HTTPS=true ;; voice) VOICE=true ;; tailscale) TAIL=true ;; *) echo "unknown argument: $a (lan | https | voice | tailscale)"; exit 2 ;; esac; done
export GURDY_WEB_TOKEN="${GURDY_WEB_TOKEN:-$(python3 -c 'import secrets;print(secrets.token_urlsafe(24))')}"
$LAN && export WEB_PUBLISH=0.0.0.0 || export WEB_PUBLISH=127.0.0.1
if $TAIL; then source scripts/lib_tailscale.sh; ts_check || exit 1; fi
EXTRA=(); PORTS=(-p "$WEB_PUBLISH:8080:8080")
docker rm -f gurdy3-web gurdy3_speech >/dev/null 2>&1 || true
trap 'docker rm -f gurdy3_speech >/dev/null 2>&1 || true; { $TAIL && ts_stop; } || true' EXIT
if $HTTPS; then
  [ -f "$HOME/.gurdy/tls/gurdy.crt" ] || { echo "no certificate yet: run scripts/make_web_cert.sh <the laptop's Wi-Fi / hotspot IP> first (docs/phase7_voice.md)"; exit 1; }
  export GURDY_TLS_PORT=8443 GURDY_TLS_CERT=/tls/gurdy.crt GURDY_TLS_KEY=/tls/gurdy.key GURDY_TLS_CA=/tls/ca.crt
  EXTRA+=(-v "$HOME/.gurdy/tls:/tls:ro"); PORTS+=(-p "$WEB_PUBLISH:8443:8443")
fi
if $VOICE; then source scripts/lib_speech.sh; start_speech_service; fi
echo "open:   http://localhost:8080/#token=$GURDY_WEB_TOKEN"
$LAN && echo "phone:  http://<PC Wi-Fi IPv4>:8080/#token=$GURDY_WEB_TOKEN"
$HTTPS && echo "https:  https://localhost:8443/#token=$GURDY_WEB_TOKEN   (phone: https://<PC IPv4>:8443/#token=...)"
if $TAIL; then ts_start 8080 || exit 1; echo "anywhere: ${TS_URL}#token=$GURDY_WEB_TOKEN   (private: Tailscale must be ON on the phone)"; fi
echo "stop:   Ctrl+C (the containers are removed)"
docker compose run --rm --name gurdy3-web "${EXTRA[@]}" "${PORTS[@]}" web
