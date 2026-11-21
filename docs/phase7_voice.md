# Voice in the dashboard (hold-to-talk) — laptop and phone

How it works: the browser records while you hold the button (MediaRecorder, WebM/Opus on Chrome, MP4 on Safari) and sends the clip to the dashboard server,
which sends it to the **speech service** (`brain/voice/speech_service.py`: Whisper on the GPU, Piper). The transcript goes through the same rules as the
microphone chat: vocabulary prompt (your place names / objects), the expected-answer hint while the robot waits for yes / no, the hallucination filter
("Thank you." on silence is dropped), and the **stop fast path** (a stop phrase fires the e-stop before the agent is asked). Your words appear as your chat
bubble with Whisper's confidence; Gurdy's replies are spoken (Piper) until you press the speaker icon to mute. Speaking over the robot silences it.
Voice is **off in `--demo` mode** (no GPU on a free host).

A microphone needs a *secure context*: `http://localhost` counts, `http://192.168...` does not. So: laptop = plain localhost, phone = https (below).

## Laptop (Windows browser, same PC as WSL)
```bash
scripts/start_web.sh voice                 # mock world (flat_lab layout), keyword planner, Whisper GPU + Piper
# or the real robot in Gazebo (about 2 minutes, GPU: LLM + Whisper = ~3.3 GiB):
scripts/start_web_robot.sh voice
```
Open the URL it prints (`http://localhost:8080/#token=...`). Hold the round **mic button** next to Send (or hold **Space** when no text field is focused), speak,
release. The browser asks once for the microphone: Allow. Ctrl+C stops everything (the speech container is removed too).

## Phone over the laptop's hotspot (https)
Windows 11 hotspot gives the laptop `192.168.137.1`; on a normal Wi-Fi use the address from `ipconfig` (Wi-Fi adapter, IPv4). Use that address below.

1. **Certificate** (WSL, once; repeat with the new address if it changes, the phone keeps trusting the same CA):
   ```bash
   scripts/make_web_cert.sh 192.168.137.1            # add more addresses separated by spaces, e.g. 192.168.1.23
   ```
2. **Firewall** (PowerShell as Administrator, once; use the same `-Profile` you used for port 8080 in `docs/phase7_phone.md`; hotspot networks are often "Public", then use `-Profile Any`):
   ```powershell
   New-NetFirewallRule -DisplayName "Gurdy web 8443" -Direction Inbound -Protocol TCP -LocalPort 8443 -Action Allow -Profile Private
   ```
3. **Start** (WSL): `scripts/start_web.sh lan https voice` (mock) or `scripts/start_web_robot.sh lan https voice` (real robot).
4. **Trust the certificate on Android** (once): on the phone open `http://192.168.137.1:8080/ca.crt` (downloads `gurdy-ca.crt`; the file is the PUBLIC CA certificate), then
   Settings → Security (and privacy) → More security settings → Encryption & credentials → **Install a certificate → CA certificate → Install anyway** → pick the file.
   (Samsung: Settings → Biometrics and security → Other security settings → Install from device storage → CA certificate. A screen lock must be set.)
5. Open `https://192.168.137.1:8443/#token=<token>` in Chrome (padlock, no warning), allow the microphone, **hold the big "Hold to talk" button** (Map and Chat tabs), speak, release.
   Without step 4 Chrome shows "Your connection is not private": Advanced → Proceed also works for many phones, but trusting the CA is the reliable way.
   iPhone: open the same `.crt` in Safari, install the profile (Settings → Profile Downloaded), then Settings → General → About → Certificate Trust Settings → enable "Gurdy local CA".

Safety of this setup: the CA private key `~/.gurdy/tls/ca.key` never leaves the laptop (mode 600) and is not in git. Anyone holding it could issue certificates your phone would trust, so remove the CA from the phone when you stop using this
(Android: Settings → Encryption & credentials → User credentials → Gurdy local CA → Remove) and delete `~/.gurdy/tls` if you like. The access token still protects every API call, and the E-STOP / Release buttons work without the microphone.
Plain `http://` on the LAN keeps working for everything except the microphone. **Tailscale Serve (`docs/phase7_remote.md`) gives a real https certificate and works from anywhere**, then none of the CA steps are needed.

## If something is off
- No mic button: the page found no speech service. `docker logs gurdy3_speech` (it needs the `voice_gpu` image + `~/p5_models`, `~/p5_pkgs`, `~/p5_tts`); the button appears once `GET /api/health` says `"voice": true`.
- "The microphone needs a secure page": you opened `http://<IP>:8080` on the phone; use the `https://…:8443` address.
- "Too short": hold the button while speaking (the clip must be at least 0.4 s). "Sorry, I didn't catch that": silence or noise was filtered out.
- The first request after start is slower (model warm-up); after that Whisper takes about 0.3 s on the GPU.
