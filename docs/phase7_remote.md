# Private remote access with Tailscale (phone, anywhere, real https)

Goal: open the **real-robot** dashboard (with voice) on your Android phone from anywhere, including mobile data, with a valid https certificate and no self-signed certs.
It stays **private to your tailnet** (your own devices). Nothing is exposed to the public internet, and the access token is still required.

Status: written from Tailscale's documentation (`tailscale serve --bg --https=443 http://127.0.0.1:8080`, `serve status`, `serve ... off`, `serve reset`; Serve is tailnet-only, Funnel is the public one).
**Not tested with a real Tailscale account yet** (the helper script is tested against a stub). If a step or message differs on your screen, tell me and I fix this file.

## Why Tailscale runs on Windows (not inside WSL or Docker)
Docker Desktop already publishes the dashboard port on Windows (`127.0.0.1:8080`). The Tailscale app on Windows then only has to forward `https://<your-pc>.<tailnet>.ts.net` to that local port.
Reasons this is the best fit:
- **One Tailscale node = your PC.** Inside WSL or a container you would need a second node with its own login / auth key, `tailscaled` running under WSL (systemd, restarts when WSL restarts) or a sidecar container that needs `/dev/net/tun`. More moving parts, more things to break.
- **Survives WSL / Docker restarts.** The Windows service keeps running; only the app behind it comes and goes.
- **No Windows firewall rule, no certificates to make, no `lan` mode:** the traffic arrives through Tailscale's own interface, and the dashboard stays bound to `127.0.0.1` (not reachable from your Wi-Fi at all).
- The certificate is a real one for `*.ts.net`, issued automatically, so Android Chrome shows a normal padlock and the **microphone works** (https is a secure context).

## Step by step (in this order)

### 1. Make a Tailscale account (free personal plan is enough)
Open https://login.tailscale.com, sign in with Google / Microsoft / GitHub / Apple. The first login creates your private network (your "tailnet"). Use the **same login** on every device below.

### 2. Install Tailscale on Windows
1. Download the installer from https://tailscale.com/download/windows, run it, accept the prompts.
2. A Tailscale icon appears in the system tray (bottom right, maybe under the `^`). Click it → **Log in** → your browser opens → approve with the same account. The tray icon turns "connected".
3. Check in **PowerShell**:
   ```powershell
   tailscale version        # 1.52 or newer is needed for `serve --bg`
   tailscale status         # shows this PC and its 100.x.y.z address
   ```
   If `tailscale` is "not recognized", close and reopen PowerShell (the installer adds it to PATH).
4. Optional but sensible: give the PC a short neutral name (the name appears in the public URL and in public certificate logs): Tailscale admin console → Machines → your PC → ⋯ → Edit machine name.

### 3. Switch on MagicDNS and HTTPS certificates (once, in the admin console)
Open https://login.tailscale.com/admin/dns :
- **MagicDNS**: must be on (it is on by default for new tailnets).
- **HTTPS Certificates** → **Enable HTTPS…** and confirm.
  (Honest note: Tailscale asks you to accept that your machine names are published in public certificate-transparency logs. The names are visible, nothing else is. This is why a neutral machine name is nice.)
If you skip this, the first `tailscale serve` prints a link asking for it; the start script shows that message.

### 4. Install Tailscale on your Android phone
1. Play Store → **Tailscale** → install → open → **Get started / Log in** with the **same account**.
2. Accept the "VPN connection request" (Tailscale uses Android's VPN slot; only one VPN can be active at a time, so turn other VPN apps off).
3. The app shows your devices; your Windows PC should be listed. The toggle at the top must be **on** whenever you want to reach the robot.

### 5. Start the robot with remote access (WSL)
Check `docker ps` is empty, then:
```bash
scripts/start_web_robot.sh tailscale voice
```
(`voice` adds hold-to-talk and spoken replies; leave it out for text only. Add `MODEL=keyword` in front if you do not want the LLM.)
The script checks that Tailscale is running and logged in (before the 2-minute start-up), starts the robot stack, then runs
`tailscale.exe serve --bg --https=443 http://127.0.0.1:8080` through Windows and prints, besides the localhost URL:
```
ANYWHERE (private, Tailscale on the phone must be ON):  https://<your-pc>.<tailnet>.ts.net/#token=<token>
```
The mock world works the same way: `scripts/start_web.sh tailscale voice`.

What the script does for you / the manual equivalent in **Windows PowerShell**:
```powershell
tailscale serve --bg --https=443 http://127.0.0.1:8080     # start serving (tailnet only)
tailscale serve status                                       # what is served, and who can reach it
tailscale serve --https=443 off                              # stop serving this one entry
tailscale serve reset                                        # remove ALL serve settings (if you want a clean slate)
```

### 6. Open it on the phone
1. Tailscale app: switch **on** (works on Wi-Fi or mobile data, any country).
2. Chrome → paste the full `https://<your-pc>.<tailnet>.ts.net/#token=<token>` line printed by the script. Padlock, no warnings.
3. Allow the microphone when asked, then hold the big **Hold to talk** button. (Add to Home Screen works too and now gets the full installable app with offline start, because the page is real https.)
The token is in the part after `#`: it is never sent to the server in the URL and the page removes it from the address bar after saving it. It stays required for every action.

### 7. Check that it is private
- **From a device that is not on your tailnet** (e.g. turn Tailscale **off** on the phone, mobile data on) the same URL must **not** load (it will not even resolve or connect). That is the real test.
- In PowerShell:
  ```powershell
  tailscale serve status       # must say the service is available "within your tailnet" (tailnet only)
  tailscale funnel status      # must say no funnel / nothing is shared publicly
  ```
  Never run `tailscale funnel`: that one is the public-internet option.
- Admin console → Machines: only your own devices are listed. Do not share the PC node with other people or add other users to the tailnet unless you mean to: everyone on the tailnet can open the page (the token still stops them from controlling the robot).
- The dashboard itself listens only on `127.0.0.1` in this mode (no `lan`), so nothing on your home Wi-Fi can reach it either.

### 8. Turn it off
- Normal: press **Ctrl+C** in the terminal running the script. It stops the robot containers and runs `tailscale serve --https=443 off` for you.
- Manual (PowerShell): `tailscale serve --https=443 off`, check `tailscale serve status` shows nothing.
- To go fully dark: tray icon → **Disconnect** (or the toggle in the phone app). The PC then disappears from your tailnet until you connect again.

## Good to know
- **The PC must stay awake and online** (Windows sleep, a closed lid or a Wi-Fi drop cuts the connection). The robot sim is only as available as the laptop.
- **Mobile data use:** I have not measured it. The map / state stream is small; the camera panel on the phone is a closed row by default (nothing is fetched until you open it); voice sends a few seconds of audio per command and returns a short spoken reply.
- Tailscale keys expire after about 180 days by default: if the PC suddenly shows "needs login", log in again from the tray icon (or disable key expiry for the PC in the admin console).
- Serve adds identity headers (who is connecting) to the request; the dashboard does not use them. Authentication is the shared token only.
- The access token is generated at every start (or set `GURDY_WEB_TOKEN`); a new start means a new URL fragment. Nothing is written to disk.

## If something does not work
| Symptom | Check |
|---|---|
| Script: "Tailscale not found" | Install on Windows (step 2). The script looks for `tailscale.exe` (WSL interop) and `C:\Program Files\Tailscale`. |
| Script: "not connected (NeedsLogin…)" | Tray icon → Log in / Connect. |
| `serve` error mentioning HTTPS / certificates | Step 3: enable HTTPS Certificates, run the script again. |
| Every command says "My language model is not running right now." | The agent container cannot reach Ollama. `scripts/start_web_robot.sh` now tests this itself (it tries every address a container could use and runs one real prompt inside the agent container before printing READY, otherwise it stops with the reason). Run it again; if it says no container can reach Ollama, check `ss -ltn | grep 11435` (must show `*:11435`). |
| Phone: page does not load | Tailscale on in the phone app? Other VPN off? PowerShell `tailscale serve status` shows the entry? Robot script still running? |
| Phone: loads but "Invalid token" / asks for the token | Use the exact URL of this run (the token changes every start). |
| Page loads, no mic button | Voice was not started: use `… tailscale voice`; `docker logs gurdy3_speech`. |
| Certificate warning | Should not happen with serve. If it does, you opened the `100.x.y.z` address or `http://`: use the `https://….ts.net` name. |
