# Public demo: a static page that runs the simulation in the browser

**Why this changed:** Hugging Face now needs a PRO subscription for Docker Spaces on free CPU (static Spaces stay free). So the public demo no longer needs a server at all: the dashboard plus the **real Python simulation** (mock `flat_lab` world, agent with safety / grounding / executor, keyword planner with the smart phrasings) run inside the visitor's browser. It is published for free on **GitHub Pages** (`https://mrsssok2.github.io/gurdy3/`) and can also be uploaded as a **static Hugging Face Space**.

What a visitor gets: the same dashboard (map, planned path, chat with suggestion chips, status, alerts, demo banner, E-STOP), no mic, no token, no server, nothing sent anywhere. Every visitor automatically has a private robot (it lives in their own tab); a reload starts a new one.

## The two options I compared

| | **A. Pyodide (chosen)**: the repo's Python runs in the browser | **B. JavaScript port** |
|---|---|---|
| Fidelity | The same files: agent, safety checker, grounding, executor, mock world, keyword parser, phrasing, telemetry. | A second implementation of ~4,000 lines of Python (agent ~2,200, mock world ~950, tools ~400, parsers) that must be kept in sync by hand; parity could only be spot-checked on a fixed command list. |
| First load | **5.6 MiB** from the jsDelivr CDN (Pyodide runtime, brotli) + 0.23 MiB our site. Measured cold: **8-11 s** to a working page (desktop-class, Docker Chromium), **2.7-3.4 s** with the runtime cached; on a throttled phone profile (4x slower CPU, 1.6 Mbit/s, 150 ms latency) cold **35 s**. | Instant (< 0.2 MB). |
| Risk | `langgraph` cannot be installed in Pyodide (checked in headless Chromium with micropip: it fails because its dependencies orjson / ormsgpack / xxhash / uuid_utils have no Pyodide wheels; pydantic 2.10 and pyyaml do load). Solved with a 40-line stand-in for the two things `brain/agent/graph.py` uses (`StateGraph`, `END`). | Drift between the JS and Python behaviour over time. |
| Proof of equivalence | `web/tests/test_static_demo.py` runs 30 commands on the repo with the **real langgraph** and on the unzipped build with **only the stand-in**: identical replies, announcements and robot poses. | A parity test would have to be written for every behaviour. |

Pyodide wins on fidelity and upkeep; its only cost is the one-time download. The page shows a loading screen ("Starting the simulated robot… everything runs in your browser") during it.

## How it works
`scripts/build_static_demo.py` copies `web/app` (the normal dashboard, unchanged apart from one line: no service worker in the static build), adds `static_backend.js` (answers the dashboard's `fetch("/api/…")` and `WebSocket("/ws")` calls locally), `demo_worker.js` (a Web Worker that loads Pyodide from the CDN, installs `pyyaml`, unpacks `gurdy_demo.zip` and runs `web/static_demo/session.py`) and `gurdy_demo.zip` (88 KiB of the repo's Python: no server, ROS, LLM client or audio code). The simulation advances 3x faster than wall time, like the server demo.
Pyodide version pinned: 0.27.7 (Python 3.12.7). It is loaded from `cdn.jsdelivr.net`; to host it yourself pass `--pyodide-base`.

Measured 2026-11-19 in headless Chromium: first visit loads **5.6 MiB (CDN) + 0.23 MiB**; chat "enter room B", "yes" walks the robot to room B, E-STOP works, no mic button, demo banner shown, a second browser context starts with an empty chat and a robot at the origin, a reload resets the world, no JavaScript errors, works from a sub-path (`/gurdy3/`) like GitHub Pages. Not tested on a real phone or on Safari/Firefox.

**Demo world (2026-11-26): lab4.** The public demo runs `brain.mock.gazebo_lab4()` (4 rooms A-D, 4 doorways, 10 objects, built from `maps/lab4.places.yaml` + `docs/results/data/lab4/lab4_gt.json`; walls checked against `scripts/gen_world_lab4.py` in `brain/tests/test_lab4_world.py`). The map panel draws these walls / rooms (the generator geometry the Gazebo world and `lab4_map` come from, not the SLAM pgm). `flat_lab()` / `gazebo_flat_lab()` are unchanged for the eval and test suites.

**Timeline tab in the demo (2026-11-26).** The in-browser world has no past, so `web/api/sample_history.py` seeds a short INVENTED earlier patrol (11 events in the lab4 rooms: toolbox moved in room D, visits to rooms A-D, battery alert, no-go set / cleared around the fire extinguisher, blocked C-A path + failed task with a cause link) into each visitor's in-memory incident graph. Each event carries `sample: true` and the panel tags it "sample"; "since you started me" / "away" windows exclude it, "today" includes it. Live events from the visitor's own session are added after it. Test: `web/tests/test_static_demo.py::test_timeline_has_tagged_sample_history`.

## Step by step: GitHub Pages (your repo `mrsssok2/gurdy3`)
1. **The repo must be public** (Pages is free for public repositories; for a private repo it needs a paid plan).
2. **Push** the commits (`git push`). The workflow `.github/workflows/pages.yml` runs on every push to `main` that touches the demo files.
3. In the repo on github.com: **Settings → Pages → Build and deployment → Source: "GitHub Actions"** (not "Deploy from a branch"). Do this once; it needs no token.
4. Open the **Actions** tab → workflow **"Publish static demo"**. If it did not start (for example because the push happened before step 3), click **Run workflow** (branch `main`). It has three jobs: `test` (runs the demo tests), `build`, `deploy`; all must be green (about 2-3 minutes).
5. The deploy job prints the page URL; it is **https://mrsssok2.github.io/gurdy3/** (give it a minute after the job finishes).
6. **Check it:** open the URL (laptop and phone): loading screen for a few seconds, then the dashboard with the banner "Demo: simulated robot"; type "go to room B", then "yes"; the robot walks. In the browser's developer tools (F12 → Network / Console) there must be no red 404 / CORS errors. `curl -I https://mrsssok2.github.io/gurdy3/` answers 200.
7. If a job fails: open it and read the red step. Typical causes: Pages not set to "GitHub Actions" (step 3); the repo is private; a test fails (the log shows which).

**Updating later:** change the code, `git push`; the workflow rebuilds and republishes by itself. Nothing else to do.

## Alternative: a static Hugging Face Space (free)
1. Create the account at https://huggingface.co/join and confirm the e-mail.
2. Token: https://huggingface.co/settings/tokens → **Create new token** → type **Write** → copy it once. Keep it out of the repo and the chat; you type it at a silent prompt below.
3. Tool (once): `python3 -m venv ~/.venvs/hf && ~/.venvs/hf/bin/pip install huggingface_hub`
4. Look at what will be uploaded (optional): `python3 scripts/demo_space.py stage` → `build/demo_space` (22 files, ~290 KiB, git-ignored).
5. Deploy:
   ```bash
   read -s HF_TOKEN; export HF_TOKEN        # paste the token, press Enter (nothing is shown or saved)
   ~/.venvs/hf/bin/python scripts/demo_space.py deploy
   unset HF_TOKEN
   ```
   It creates the public **static** Space `USER/gurdy-demo` if it does not exist, uploads, waits for RUNNING and fetches the page. It prints `LIVE: https://USER-gurdy-demo.static.hf.space/`.
6. Check it like step 6 above. To update: run the deploy command again (clean snapshot; old versions stay in the Space's Files → History). To stop it: Space → Settings → Pause / delete.
The Space page embeds the demo in an iframe; Web Workers and the CDN load work there. If something looks off in the iframe, open the direct `*.static.hf.space` URL.

## Limits and honest notes
- First visit: about 6 MB and several seconds (35 s on a slow phone profile). Later visits are quick because the browser caches the runtime.
- Needs a recent browser with WebAssembly and Web Workers. If `cdn.jsdelivr.net` is blocked (some company networks), the page shows an error message instead of the simulator.
- Same planner as before (keyword parser, no LLM), no voice. The world is the Gazebo `flat_lab` layout, not Gazebo itself.
- The per-visitor server mode (`python -m web.api --demo`, with sessions, caps and rate limits; tests in `web/tests/test_demo_sessions.py`) still exists for self-hosting on a machine you control; it is no longer what gets deployed.
- Safety: the page contains no robot connection, token, secret or backend address. Everything it can do is run a simulation inside the visitor's own browser.


---

# Two extra features of the static demo (2026-11-19)

## 1. "Connect to my robot" (the same page talks to your real robot)
By default everyone gets the in-browser simulation. A small **Connect to my robot** button (**My robot** on phones) opens a dialog: paste your robot address (for example `https://mrsssok.tail4d1190.ts.net/#token=...`, the token can also go in its own field) and press Connect.
- The page checks the robot first (`/api/health`, then an authenticated `/api/state`). Only if both pass does it reload in **real-robot mode**: a red banner **"Connected to REAL robot"** with a **Back to demo** button; REST, WebSocket and the robot's own GPU voice (when started with `voice`) now go to the robot instead of the in-browser Python. The simulator is not downloaded or started in this mode.
- If the robot cannot be reached the dialog says **"Your robot isn't running or this device isn't on your Tailscale network."** (plus a hint about the allowed-origins setting: a browser cannot tell "down" from "blocked by CORS"), and the page stays in demo mode. A wrong token says "not accepted", a non-https address is refused before anything is sent.
- Storage: the robot URL is remembered in `localStorage`; the **token is kept in `sessionStorage` only** (this tab; forgotten when it is closed, so a new visit asks again). If the robot disappears while the tab is reloaded, the page falls back to the demo with a message. While connected, a lost connection turns the banner amber ("Your robot isn't running or …, retrying") until it recovers.

**Robot side (security).** The robot server answers cross-origin calls only for web pages on an allow-list (default `https://mrsssok2.github.io`):
- `Access-Control-Allow-Origin` is the exact listed origin, never `*`; preflights (`OPTIONS`) need no token but need a listed origin (and answer the Private Network Access header for a public page reaching a tailnet address); any other browser origin gets **403** with no CORS headers, also on POST and on the **WebSocket handshake** (Origin is checked there too); the page the robot serves itself (any address it is reached by) is always allowed; requests without an Origin (curl, scripts) are unaffected.
- **The access token stays required for everything.** CORS only lets that page *try*; a listed page without the token gets 401.
- Configure with `GURDY_ALLOWED_ORIGINS=https://a.example,https://b.example` (or `--allowed-origin URL`, repeatable; `none` = only the server's own pages; a wildcard is rejected). `scripts/start_web_robot.sh` passes the variable on; for a Hugging Face static Space copy add `https://USER-gurdy-demo.static.hf.space`.
Tests: `web/tests/test_origin.py` (9 tests: default, lookalikes, no wildcard, preflight, token still required, WebSocket origin, own pages, env / list, parsing) and the browser tests below.

## 2. Voice in the demo (Whisper in the visitor's browser)
Hold the mic button (or hold Space on a laptop) in the demo. **Nothing is downloaded until the first press**; then a progress card shows the speech model downloading (once; the browser keeps it). Whisper runs in a Web Worker through transformers.js 3.8.1 (`whisper-base.en` by default, 8-bit weights on WASM; WebGPU is tried first when the browser has it; `?whisper=tiny.en` selects the smaller, faster-to-download model). **Audio never leaves the device**; only the model files are fetched. Gurdy's replies are spoken with the browser's own voices (`speechSynthesis`); the speaker icon mutes, and pressing the mic silences the robot.
The same rules as the real microphone pipeline decide what happens to the text (the Python in the browser is the repo's code):
- **vocabulary prompt** (`brain/voice/vocab.py`: place names, objects, colours, commands) is given to Whisper as its prompt; while the agent waits for a yes / no (or a room letter) the **expected answers** replace it (`answer_hint`);
- **stop fast path**: a stop phrase (including mishearings like "Stalk") fires the e-stop before anything else;
- **yes / no confirmations** go through the agent's confirmation gate as usual;
- **hallucination filter** (`brain/voice/transcript_filter.py`, shared with the server pipeline): silence / noise (RMS below 0.004), repeated chunks, and - for this small browser model, which gives no no-speech probability - stock silence phrases ("Thank you.", "you", ...) on a quiet clip are dropped as "Sorry, I didn't catch that."
Differences to the server pipeline (honest): no Whisper confidence or no-speech probability from the browser model (the bubble says "voice" without a percentage), no voice-activity filter, and I did not measure whether the prompt improves accuracy in the browser model (it is applied and does not break decoding: transcripts contain no prompt echo).

**Measured** (Chromium in Docker on this PC, WASM, whisper-tiny.en; the container needed `--disable-http2`, because Chromium 131's HTTP/2 got HTTP 405 from huggingface.co here, curl and Python were fine, desktop Chrome is the normal case for transformers.js demos):
| | Desktop profile |
|---|---|
| Model download (first mic press only) | about **50 MB**: 40.8 MB weights (encoder 10.1 + decoder 30.7), ~5 MB tokenizer / config, ~4 MB ONNX Runtime from the CDN |
| Time to a ready model on this connection | 62 s (network-bound) |
| Transcription of a ~2.4 s clip (model warm, single-threaded WASM) | 2.7-3.2 s |
| Throttled phone profile (1.6 Mbit/s, 150 ms latency) | model ready after **231 s** (the download is what costs: ~50 MB at 0.2 MB/s); the CPU slowdown (4x) did not reach the worker in this setup, so a phone's real transcription time is unmeasured (this PC: 1.6-3.2 s per clip; expect slower) |

Found while measuring: the tiny model once printed "Go to Room B." ten times for a 2.6 s clip. The shared repetition filter only caught 1-3 word loops; it now catches loops of up to 6 words (such a transcript is dropped as "didn't catch that" instead of being executed).
Data use: the first mic press costs about 50 MB (do it on Wi-Fi); after that nothing is downloaded again.

**Hugging Face / GitHub Pages note:** the model comes straight from huggingface.co and the code from jsdelivr, so no extra files live in the repo or the Pages site.

## Browser tests (not part of CI: they need Chromium)
`scripts/run_browser_tests.sh` builds the site, starts two mock robot servers and runs `web/tests/browser/test_demo_browser.py` in headless Chromium: 31 checks, all passing on 2026-11-19 - default demo with the Connect button; unreachable robot keeps demo mode; wrong token; page not on the robot's allow-list; garbage / plain-http addresses; successful connect (banner, chat and E-STOP reach the robot, no simulator started); reload keeps the session; Back to demo; URL remembered but never the token; robot gone on reload falls back; phone layout; voice with a **stub** Whisper (lazy model: not requested before the first press, progress card, transcript bubble marked "voice", hint while a question is open, spoken replies, "Yes." by voice, "Stalk" stops, mute, barge-in) and the silent-microphone filter; the robot's own page does not get the browser mic.
Run once with `REMOVE_IMAGE=true scripts/run_browser_tests.sh` to delete the ~1.5 GB Playwright image afterwards.

## Your steps to try both
1. **Push** (`git push`); the Pages workflow rebuilds and publishes (Actions tab → "Publish static demo" green). Open https://mrsssok2.github.io/gurdy3/ (hard-refresh once so the new files load).
2. **Voice in the demo:** press and hold the mic button, allow the microphone, wait for the progress card (about 50 MB, once), say "go to room B", release; then hold again and say "yes". Gurdy answers aloud (speaker icon = mute).
3. **Connect to my robot:** on the robot PC, `scripts/start_web_robot.sh tailscale voice` (the default allow-list already contains `https://mrsssok2.github.io`; for another page set `GURDY_ALLOWED_ORIGINS`). On the phone / laptop with Tailscale ON open the demo page, tap **Connect to my robot**, paste `https://<your-pc>.<tailnet>.ts.net/#token=<token>` (the line the script prints), Connect. Banner "Connected to REAL robot"; type or hold-to-talk (the robot's GPU Whisper). **Back to demo** returns to the simulation.
4. Check the failure path: turn Tailscale off on the phone and try Connect: "Your robot isn't running or this device isn't on your Tailscale network."


## Polish (2026-11-19, later)
- **Model:** the demo now uses `whisper-base.en` because `tiny.en` heard "go to room B" as "Go to Ruby." and once looped. Measured here: **base.en first download ~85 MB** (76.9 MB weights = encoder 23.2 + decoder 53.7, plus ~8 MB tokenizer / runtime) vs tiny.en ~50 MB; ready in 87 s vs 62 s on this connection (network-bound; ~7 minutes on the 1.6 Mbit/s profile); transcription 3.2-4.5 s vs 2.7-3.2 s per ~2.6 s clip (single-threaded WASM). On the test clip base.en returned "Go to room B. Go to room B." (no loop). Use `?whisper=tiny.en` on a slow connection.
- **Room-name mishearings:** `fix_room_mishearings` in `brain/voice/transcript_filter.py` turns "Go to Ruby." / "roomby" / "room bee" into "room B" (and "room sea" into "room C", a lone "Ruby" into the answer "B"), only after a movement word or as the whole answer. It is applied to speech transcripts only (the browser session and the server's speech service), never to typed text or the evaluation sets. Tests in `brain/tests/test_transcript_filter.py`, `web/tests/test_static_demo.py`.
- **Top bar:** on windows 900-1280 px wide it drops the sub-title, the "connected" word and the chip captions instead of overlapping (checked at 900 / 1000 / 1100 / 1280 px: no overlap, no overflow).
- **E-STOP messages:** on the real robot "Emergency stop engaged / released." is now shown once (the robot announces it; the page no longer adds a second line). The mock / demo still shows "Emergency stop sent."
