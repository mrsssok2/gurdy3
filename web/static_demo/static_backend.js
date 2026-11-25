// Static demo transport. The dashboard (app.js) talks to a server through fetch("/api/...") and a WebSocket; here both are answered locally by
// the Python simulation running in a Web Worker (demo_worker.js). app.js is unchanged: same UI, same code paths, no server, one private world per tab.
(() => {
  "use strict";
  window.GURDY_STATIC_DEMO = true;
  const t0 = performance.now();
  const pending = new Map();
  let worker = null, nextId = 1, readyResolve, readyReject;
  const ready = new Promise((res, rej) => { readyResolve = res; readyReject = rej; });

  // ---- loading screen (first visit downloads ~6 MB of Python runtime; the browser caches it afterwards)
  const modeReady = window.GURDY_MODE_READY || Promise.resolve();          // remote_connect.js decides demo vs real robot first
  const ov = document.createElement("div");
  ov.id = "bootOverlay";
  ov.setAttribute("role", "status");
  ov.innerHTML = '<div class="bo-card"><div class="bo-logo"></div><h2>Starting the simulated robot…</h2><p id="boText">Downloading the Python runtime</p>' +
    '<div class="bo-bar"><i></i></div><p class="bo-note">Everything runs in your browser. Nothing is sent to a server. First visit downloads about 6 MB.</p></div>';
  const css = document.createElement("style");
  css.textContent = "#bootOverlay{position:fixed;inset:0;z-index:50;display:grid;place-items:center;background:#0E1116;color:#E6EDF3;font:15px/1.5 Inter,system-ui,sans-serif;transition:opacity .35s}" +
    "#bootOverlay.done{opacity:0;pointer-events:none}.bo-card{max-width:380px;padding:24px;text-align:center}.bo-logo{width:18px;height:18px;margin:0 auto 14px;border-radius:6px;background:#2DD4BF;box-shadow:0 0 18px #2DD4BF99}" +
    "#bootOverlay h2{margin:0 0 6px;font:700 20px 'Space Grotesk',Inter,sans-serif}#bootOverlay p{margin:6px 0;color:#8B98A5}.bo-bar{height:6px;margin:16px 0;border-radius:99px;background:#1C232C;overflow:hidden}" +
    ".bo-bar i{display:block;width:40%;height:100%;border-radius:99px;background:#2DD4BF;animation:bo 1.2s ease-in-out infinite}@keyframes bo{0%{margin-left:-40%}100%{margin-left:100%}}.bo-note{font-size:13px}.bo-err{color:#FFB4B7!important}";
  document.head.appendChild(css);
  const input = document.getElementById("chatInput"); if (input) input.maxLength = 200;          // same message cap as the server demo
  const bootText = (t, err) => { const el = document.getElementById("boText"); if (el) { el.textContent = t; el.classList.toggle("bo-err", !!err); } };

  const onWorkerMessage = (ev) => {
    const m = ev.data;
    if (m.type === "progress") bootText(m.text + "…");
    else if (m.type === "ready") { window.__demoReadyMs = Math.round(performance.now() - t0); ov.classList.add("done"); setTimeout(() => ov.remove(), 500); readyResolve(); }
    else if (m.type === "fatal") { bootText("The simulator could not start: " + m.error + " (a recent browser with WebAssembly is needed)", true); readyReject(new Error(m.error)); }
    else if (pending.has(m.id)) { const p = pending.get(m.id); pending.delete(m.id); m.error ? p.rej(new Error(m.error)) : p.res(m.result); }
  };
  const onWorkerError = (e) => { bootText("The simulator could not start: " + (e.message || "worker error"), true); readyReject(new Error(e.message || "worker error")); };
  // the Python simulation is only started in DEMO mode (in robot mode nothing is downloaded or run here)
  modeReady.then(() => {
    if (window.GURDY_REMOTE) return;
    document.body.appendChild(ov);
    worker = new Worker("demo_worker.js");
    worker.onmessage = onWorkerMessage; worker.onerror = onWorkerError;
  });
  const rpc = (op, ...args) => ready.then(() => new Promise((res, rej) => { const id = nextId++; pending.set(id, { res, rej }); worker.postMessage({ id, op, args }); }));
  const sockets = new Set();                                                 // open fake WebSockets (REST-initiated chat must reach them too)
  const pushAll = (items, state) => { for (const s of sockets) { for (const it of items) s._emit({ type: "chat", ...it }); if (state) s._emit({ type: "state", ...state }); } };

  // ---- fetch("/api/...")
  const realFetch = window.fetch.bind(window);
  const json = (obj, status = 200) => new Response(JSON.stringify(obj), { status, headers: { "Content-Type": "application/json" } });
  window.fetch = async (input, init = {}) => {
    const url = new URL(typeof input === "string" ? input : input.url, location.href);
    const i = url.pathname.indexOf("/api/");
    if (i < 0 || url.origin !== location.origin) return realFetch(input, init);       // other origins (the real robot) go straight to the network
    const route = url.pathname.slice(i), method = (init.method || "GET").toUpperCase();
    if (route === "/api/health") return json({ ok: true, version: "static", demo: true, voice: false });
    if (route === "/api/map") return json((await rpc("hello")).map);
    if (route === "/api/state") return json(await rpc("state"));
    if (route === "/api/camera") return json({ available: false });
    if (route === "/api/timeline" || route === "/api/changes") return json(await rpc("incidents", route.slice(5), Object.fromEntries(url.searchParams)));
    if (route === "/api/messages") return json({ messages: await rpc("chat_since", Number(url.searchParams.get("since") || 0)) });
    if (route === "/api/estop/release") return json({ ok: false });
    if (route === "/api/voice" || route === "/api/tts") return json({ error: "voice is not available in the demo" }, 503);
    if (route === "/api/voice_context") return json(await rpc("voice_context"));            // vocabulary prompt + expected answers for the in-browser Whisper
    if (method === "POST" && route === "/api/voice_text") {
      let b = {}; try { b = JSON.parse(init.body || "{}"); } catch { /* */ }
      const r = await rpc("voice", b.text, b.rms, b.confidence);
      pushAll(r.items, r.state);
      return json({ heard: r.heard, rejected: r.rejected || null, stop: !!r.stop, reply: r.reply || null });
    }
    if (method === "POST" && route === "/api/estop") { await rpc("estop"); return json({ ok: true }); }
    if (method === "POST" && route === "/api/say") {
      let text; try { text = JSON.parse(init.body || "{}").text; } catch { /* bad body */ }
      const r = await rpc("say", text);
      return r.error ? json({ error: r.error }, 400) : json({ reply: r.reply.text, seq: r.reply.seq });
    }
    return json({ error: "unknown route" }, 404);
  };

  // ---- WebSocket("/ws")
  const RealWS = window.WebSocket;
  class LocalWS {
    constructor() {
      sockets.add(this);
      this.readyState = 0; this.onopen = this.onmessage = this.onclose = this.onerror = null; this._timer = 0; this._busy = false;
      ready.then(async () => {
        const hello = await rpc("hello");
        if (this.readyState === 3) return;
        this.readyState = 1; this.onopen && this.onopen({});
        this._emit(hello); this._emit({ type: "state", ...(await rpc("state")) });
        this._timer = setInterval(() => this._tick(), 500);
      }).catch(() => { this.readyState = 3; this.onerror && this.onerror({}); this.onclose && this.onclose({}); });
    }
    _emit(obj) { if (this.readyState === 1 && this.onmessage) this.onmessage({ data: JSON.stringify(obj) }); }
    async _tick() {
      if (this._busy || this.readyState !== 1) return;
      this._busy = true;
      try { const r = await rpc("tick"); this._emit({ type: "state", ...r.state }); for (const c of r.chat) this._emit({ type: "chat", ...c }); }
      finally { this._busy = false; }
    }
    async send(data) {
      let cmd; try { cmd = JSON.parse(data); } catch { return this._emit({ type: "error", error: "expected a JSON object with 'op'" }); }
      if (cmd.op === "say") {
        const r = await rpc("say", cmd.text);
        if (r.error) return this._emit({ type: "error", error: "bad 'text'" });
        this._emit({ type: "ack", op: "say" }); this._emit({ type: "chat", ...r.user }); this._emit({ type: "chat", ...r.reply }); this._emit({ type: "state", ...r.state });
      } else if (cmd.op === "estop") {
        const r = await rpc("estop"); this._emit({ type: "chat", ...r.item }); this._emit({ type: "ack", op: "estop" }); this._emit({ type: "state", ...r.state });
      } else if (cmd.op === "release") this._emit({ type: "ack", op: "release", ok: false });
      else this._emit({ type: "error", error: "unknown op" });
    }
    close() { sockets.delete(this); if (this.readyState === 3) return; this.readyState = 3; clearInterval(this._timer); this.onclose && this.onclose({}); }
  }
  function WS(url, ...rest) {
    const u = new URL(url, location.href);
    return u.host === location.host && u.pathname.endsWith("/ws") ? new LocalWS() : new RealWS(url, ...rest);       // a wss:// address on another host = the real robot
  }
  Object.assign(WS, { CONNECTING: 0, OPEN: 1, CLOSING: 2, CLOSED: 3 });
  window.WebSocket = WS;
})();
