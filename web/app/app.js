import { makeView, fmtTime, fmtNum, batteryClass, legSummary, newAlerts, wsUrl, remoteWsUrl, backoff, prettyName, colorHex, objectLabel, placeLabels, angDiff } from "./lib.js";

const $ = (id) => document.getElementById(id);
const C = { bg: "#0B0E13", room: "#121821", wall: "#3A4552", obstacle: "#4B5563", zone: "#D93036", accent: "#2DD4BF", text: "#E6EDF3", muted: "#8B98A5" };

const S = { demo: false, token: "", ws: null, attempt: 0, timer: null, map: null, state: null, lastSeq: 0, seen: new Set(), history: [], tab: "map", disp: null, voice: false, muted: false, voiceUsed: false, live: false };

// Public demo: every browser session has its own random 128-bit id (sessionStorage, so each tab / visit is a separate simulated robot);
// the server keys the world, agent and chat on it. Not secret-critical elsewhere (private mode ignores it), never shown or shared.
function newSid() { const b = new Uint8Array(16); (window.crypto || window.msCrypto).getRandomValues(b); return [...b].map((x) => x.toString(16).padStart(2, "0")).join(""); }
let memSid = "";
function getSid() {
  try { let s = sessionStorage.getItem("gurdy_sid"); if (!s) { s = newSid(); sessionStorage.setItem("gurdy_sid", s); } return s; }
  catch { return memSid || (memSid = newSid()); }                // storage blocked (private mode, partitioned iframe): the id lives as long as the page
}
function resetSid() { memSid = ""; try { sessionStorage.removeItem("gurdy_sid"); } catch { /* fine */ } }

// "Connect to my robot" (static demo only): when window.GURDY_REMOTE is set the page talks to that robot server instead of the in-browser simulation.
const baseUrl = () => (window.GURDY_REMOTE ? window.GURDY_REMOTE.base : "");
const remoteStatus = (kind) => { if (window.GURDY_REMOTE_UI && window.GURDY_REMOTE) window.GURDY_REMOTE_UI.status(kind); };

function store(k, v) { try { localStorage.setItem(k, v); } catch { /* private mode: fine */ } }
function load(k) { try { return localStorage.getItem(k) || ""; } catch { return ""; } }

// ---------------------------------------------------------------- connection
function setConn(kind, text) { $("conn").className = "conn " + kind; $("connText").textContent = text; }

async function api(path, opts = {}) {
  const auth = S.demo ? {} : { Authorization: "Bearer " + S.token };      // the public demo has no token
  const r = await fetch(baseUrl() + path, { ...opts, headers: { ...auth, "X-Session": getSid(), "Content-Type": "application/json", ...(opts.headers || {}) } });
  if (r.status === 401) throw Object.assign(new Error("unauthorized"), { code: 401 });
  if (!r.ok) { const j = await r.json().catch(() => ({})); throw Object.assign(new Error(j.error || r.statusText), { code: r.status, busy: !!j.busy }); }
  return r.json();
}

async function connect() {
  clearTimeout(S.timer);
  await (window.GURDY_MODE_READY || Promise.resolve());          // static demo: remote_connect.js has decided demo vs real robot
  if (window.GURDY_REMOTE) S.token = window.GURDY_REMOTE.token;
  try {                                                // public, needs no token: tells us whether this is the demo server
    const hl = await (await fetch(baseUrl() + "/api/health")).json();
    S.demo = !!hl.demo; S.voice = !!hl.voice && !S.demo;
    S.browserVoice = S.demo && !!window.GURDY_STATIC_DEMO && !window.GURDY_REMOTE && !!window.GURDY_BROWSER_VOICE && window.GURDY_BROWSER_VOICE.supported;
    $("demoBanner").hidden = !S.demo;
    setupVoice();
  } catch { return retry("server not reachable"); }
  if (!S.token && !S.demo) return askToken();
  try {
    S.map = await api("/api/map");                     // also validates the token (a WebSocket cannot report a 401 to the page) and opens the demo session
    $("busyBanner").hidden = true;
  } catch (e) {
    if (e.code === 401) { remoteStatus("token"); return askToken("That token was not accepted."); }
    if (e.code === 503 && e.busy || e.code === 429) {                       // demo full / too many sessions: friendly message, try again later
      $("busyBanner").textContent = e.busy ? "The demo is busy right now. Trying again in a minute…" : e.message;
      $("busyBanner").hidden = false; setConn("off", "busy");
      clearTimeout(S.timer); S.timer = setTimeout(connect, 20000); return;
    }
    return retry("server not reachable");
  }
  const ws = new WebSocket(window.GURDY_REMOTE ? remoteWsUrl(window.GURDY_REMOTE.base, S.token, getSid()) : wsUrl(location, S.demo ? "" : S.token, getSid()));
  S.ws = ws;
  ws.onopen = () => { S.attempt = 0; setConn("on", "connected"); remoteStatus("ok"); };
  ws.onmessage = (ev) => { try { onMessage(JSON.parse(ev.data)); } catch { /* ignore malformed */ } };
  ws.onclose = () => { if (S.ws === ws) { S.ws = null; if (S.expired) { S.expired = false; S.attempt = 0; setConn("off", "starting a new session"); S.timer = setTimeout(connect, 1500); } else retry("disconnected"); } };
  ws.onerror = () => ws.close();
}

function retry(text) {
  setConn("off", text);
  remoteStatus("unreachable");
  S.timer = setTimeout(connect, backoff(S.attempt++));
}

function onMessage(m) {
  if (m.type === "hello") { S.live = false; S.map = m.map; S.lastSeq = 0; S.seen.clear(); S.history = []; $("chatList").replaceChildren(); m.chat.forEach(addChat); S.live = true; renderChips(); }   // also sent after a demo reset
  else if (m.type === "state") onState(m);
  else if (m.type === "session_expired") { resetSid(); S.expired = true; addChat({ seq: -1, role: "system", text: m.message || "This demo session timed out." }); }
  else if (m.type === "chat") addChat(m);
  else if (m.type === "error") addChat({ seq: -1, role: "system", text: m.error });
}

function send(obj) {
  if (S.ws && S.ws.readyState === WebSocket.OPEN) { S.ws.send(JSON.stringify(obj)); return true; }
  return false;
}

// ---------------------------------------------------------------- e-stop and chat
async function estop() {
  stopSpeaking();
  if (send({ op: "estop" })) return;
  try { await api("/api/estop", { method: "POST", body: "{}" }); } catch (e) { addChat({ seq: -1, role: "system", text: "E-stop failed to send: " + e.message }); }
}

async function releaseEstop() {
  if (send({ op: "release" })) return;
  try { await api("/api/estop/release", { method: "POST", body: "{}" }); } catch (e) { addChat({ seq: -1, role: "system", text: "Release failed: " + e.message }); }
}

async function say(text) {
  if (send({ op: "say", text })) return;
  try { await api("/api/say", { method: "POST", body: JSON.stringify({ text }) }); await pollChat(); } catch (e) { addChat({ seq: -1, role: "system", text: "Not sent: " + e.message }); }
}

async function pollChat() {
  const r = await api("/api/messages?since=" + S.lastSeq);
  r.messages.forEach(addChat);
}

function addChat(m) {
  if (m.seq > 0) { if (m.seq <= S.lastSeq) return; S.lastSeq = m.seq; }
  const el = document.createElement("div");
  el.className = "msg " + (["user", "robot", "system"].includes(m.role) ? m.role : "robot");
  el.textContent = m.text;                              // textContent only: robot / alert text is never parsed as HTML
  if (m.voice) { const meta = document.createElement("span"); meta.className = "meta"; meta.textContent = "voice" + (m.confidence != null ? ` · ${Math.round(m.confidence * 100)} % sure` : ""); el.appendChild(meta); }
  const list = $("chatList");
  const atEnd = list.scrollHeight - list.scrollTop - list.clientHeight < 80;
  list.appendChild(el);
  while (list.childElementCount > 300) list.firstChild.remove();
  if (atEnd || m.role === "user") list.scrollTop = list.scrollHeight;
  renderChips();
  if (S.live && m.role === "robot" && S.voiceUsed && !S.muted) speak(m.text);
}

// ---------------------------------------------------------------- suggestion chips (empty chat only)
const SUGGESTIONS = ["Go to room B", "Where are you?", "Find the yellow box", "Go home"];
function renderChips() {
  const box = $("chips"), empty = $("chatList").childElementCount === 0;
  if (!empty) { box.replaceChildren(); return; }
  if (box.childElementCount) return;
  box.replaceChildren(...SUGGESTIONS.map((t) => { const b = document.createElement("button"); b.type = "button"; b.className = "chip-btn"; b.textContent = t; b.addEventListener("click", () => say(t)); return b; }));
}

// ---------------------------------------------------------------- state -> UI
function onState(st) {
  S.state = st;
  if (st.connected === false) { setConn("off", st.error ? "robot source error" : "no robot"); return; }
  setConn(st.stale ? "stale" : "on", st.stale ? "busy" : "connected");
  $("staleBanner").hidden = !st.stale;
  $("estopBanner").hidden = !st.estop;
  const p = st.pose || {};
  $("px").textContent = fmtNum(p.x, 2, " m"); $("py").textContent = fmtNum(p.y, 2, " m");
  $("ph").textContent = fmtNum(p.heading_deg, 0, "°"); $("pplace").textContent = p.place || "–";
  $("sMode").textContent = st.mode || "–";
  $("sTask").textContent = st.task ? prettyName(st.task.tool) : "none";
  $("sSpeed").textContent = fmtNum(st.speed_mps, 2); $("sSpeedUnit").textContent = st.speed_mps == null ? "" : "m/s";
  $("sTime").textContent = fmtTime(st.sim_time_s);
  const b = st.battery_pct;
  $("sBat").textContent = fmtNum(b, 0, " %");
  $("sBatBar").style.width = Math.max(0, Math.min(100, b || 0)) + "%";
  $("sBatBar").className = batteryClass(b);
  $("sSim").hidden = !st.battery_simulated;
  $("sLegSummary").textContent = legSummary(st.legs);
  $("tBat").textContent = fmtNum(b, 0, "%"); $("tBatBar").style.width = Math.max(0, Math.min(100, b || 0)) + "%"; $("tBatBar").className = batteryClass(b); $("tSim").hidden = !st.battery_simulated;
  $("tLegText").textContent = legSummary(st.legs);
  renderLegs(st.legs || {});
  renderObjects(st.objects || []);
  renderAlerts(st.alerts || []);
  setCameraAvailable(!!st.camera_available);
  kick();
}

function renderLegs(legs) {
  const box = $("legs");
  box.replaceChildren(...Object.entries(legs).map(([name, state]) => {
    const d = document.createElement("div"); d.className = "leg " + state;
    const dot = document.createElement("span"); dot.className = "dot";
    const n = document.createElement("span"); n.textContent = prettyName(name);
    const s = document.createElement("span"); s.className = "st"; s.textContent = state;
    d.append(dot, n, s); return d;
  }));
  $("tLegs").replaceChildren(...Object.entries(legs).map(([name, state]) => {
    const d = document.createElement("span"); d.className = "dot " + state; d.title = `${prettyName(name)}: ${state}`; return d;
  }));
}

function renderObjects(objs) {
  $("sObjCount").textContent = String(objs.length);
  const ul = $("objList");
  if (!objs.length) { ul.replaceChildren(emptyLi("Nothing seen yet.")); return; }
  ul.replaceChildren(...objs.map((o) => {
    const li = document.createElement("li");
    const sw = document.createElement("span"); sw.className = "sw"; sw.style.background = colorHex(o.color);
    const t = document.createElement("span"); t.textContent = `${objectLabel(o)}${o.place ? " · " + o.place : ""}`;
    const w = document.createElement("span"); w.className = "when"; w.textContent = o.stale ? "stale" : fmtTime(o.age_s) + " ago";
    li.append(sw, t, w); return li;
  }));
}

function emptyLi(text) { const li = document.createElement("li"); li.className = "empty"; li.textContent = text; return li; }

function renderAlerts(active) {
  for (const t of [...S.seen]) if (!active.includes(t)) S.seen.delete(t);       // a cleared alert may fire again later
  S.history.unshift(...newAlerts(active, S.seen, new Date()));
  if (S.history.length > 50) S.history.length = 50;
  $("aCount").textContent = String(active.length);
  const badge = $("alertBadge"); badge.hidden = active.length === 0; badge.textContent = String(active.length);
  const item = (a, when) => {
    const li = document.createElement("li"); if (/fail|fell|fall|critical|stuck/i.test(a.text)) li.className = "crit";
    const t = document.createElement("span"); t.textContent = a.text;
    const w = document.createElement("span"); w.className = "when"; w.textContent = when;
    li.append(t, w); return li;
  };
  const hhmm = (d) => d.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
  $("alertNow").replaceChildren(...(active.length ? active.map((t) => item({ text: t }, "now")) : [emptyLi("No active alerts.")]));
  const old = S.history.filter((a) => !active.includes(a.text));
  $("alertHist").replaceChildren(...(old.length ? old.map((a) => item(a, hhmm(a.at))) : [emptyLi("Nothing earlier this session.")]));
}

// ---------------------------------------------------------------- map
const FONT = "13px Inter, system-ui, sans-serif";

/** Small shape per object class, centred on (x, y), in the object's own colour. `s` = half size in px. */
function drawObject(g, o, x, y, s) {
  const col = colorHex(o.color);
  g.save();
  g.globalAlpha = o.stale ? 0.45 : 1;
  g.fillStyle = col; g.strokeStyle = "#0B0E13"; g.lineWidth = 1.5;
  switch (o.class) {
    case "barrel": g.beginPath(); g.arc(x, y, s, 0, 6.2832); g.fill(); g.stroke(); g.beginPath(); g.arc(x, y, s * 0.5, 0, 6.2832); g.strokeStyle = "#0B0E1388"; g.stroke(); break;
    case "traffic_cone": g.beginPath(); g.moveTo(x, y - s * 1.1); g.lineTo(x + s, y + s * 0.8); g.lineTo(x - s, y + s * 0.8); g.closePath(); g.fill(); g.stroke(); break;
    case "fire_extinguisher": g.beginPath(); g.roundRect(x - s * 0.55, y - s, s * 1.1, s * 2, s * 0.4); g.fill(); g.stroke(); break;
    case "toolbox": g.beginPath(); g.roundRect(x - s * 1.2, y - s * 0.65, s * 2.4, s * 1.3, 3); g.fill(); g.stroke(); break;
    case "mannequin": g.beginPath(); g.arc(x, y - s * 0.55, s * 0.5, 0, 6.2832); g.fill(); g.stroke(); g.beginPath(); g.roundRect(x - s * 0.7, y, s * 1.4, s * 1.1, 4); g.fill(); g.stroke(); break;
    case "chair": g.beginPath(); g.roundRect(x - s * 0.8, y - s * 0.2, s * 1.6, s * 1.0, 3); g.fill(); g.stroke(); g.fillRect(x - s * 0.8, y - s * 1.0, s * 1.6, s * 0.4); break;
    case "box": default: g.beginPath(); g.roundRect(x - s, y - s, s * 2, s * 2, 3); g.fill(); g.stroke();
  }
  if (o.stale) { g.setLineDash([3, 3]); g.strokeStyle = col; g.strokeRect(x - s - 3, y - s - 3, 2 * s + 6, 2 * s + 6); }
  g.restore();
}

function drawRobot(g, x, y, th, R) {
  g.save();
  g.translate(x, y);
  // soft glow ring
  const grad = g.createRadialGradient(0, 0, R * 0.4, 0, 0, R * 3.2);
  grad.addColorStop(0, "#2DD4BF66"); grad.addColorStop(1, "#2DD4BF00");
  g.fillStyle = grad; g.beginPath(); g.arc(0, 0, R * 3.2, 0, 6.2832); g.fill();
  g.rotate(th);
  g.shadowColor = "#2DD4BF"; g.shadowBlur = 14;
  g.fillStyle = "#2DD4BF"; g.strokeStyle = "#062B27"; g.lineWidth = 2;
  g.beginPath(); g.moveTo(R * 1.5, 0); g.lineTo(-R * 0.95, R * 0.95); g.lineTo(-R * 0.45, 0); g.lineTo(-R * 0.95, -R * 0.95); g.closePath(); g.fill();
  g.shadowBlur = 0; g.stroke();
  g.restore();
}

function drawMap() {
  S.raf = 0;
  const cv = $("map"), dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (!w || !h || !S.map) return;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const g = cv.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.fillStyle = C.bg; g.fillRect(0, 0, w, h);
  const pad = Math.max(14, Math.min(w, h) * 0.04);
  const v = makeView(S.map.bounds, w, h, pad, true);
  const rect = v.rect;
  g.font = FONT; g.textBaseline = "middle";
  const measure = (t) => g.measureText(t).width;
  const LH = 17;
  const taken = [], labels = [];
  for (const [name, r] of Object.entries(S.map.rooms || {})) {          // floor + room titles (placed first, so they win their corner)
    g.fillStyle = C.room; g.fillRect(...rect(r));
    g.strokeStyle = "#1B2430"; g.lineWidth = 1; g.strokeRect(...rect(r));
    const t = prettyName(name).toUpperCase(), rr = rect(r), box = { x: rr[0] + 10, y: rr[1] + 8, w: measure(t) + 2, h: LH };
    taken.push(box); labels.push({ t, box, room: true });
  }
  g.fillStyle = C.wall; for (const r of S.map.walls || []) g.fillRect(...rect(r));
  g.fillStyle = C.obstacle; for (const r of S.map.obstacles || []) g.fillRect(...rect(r));
  const st = S.state || {};
  for (const z of st.zones || []) {                                   // no-go zones
    const zp = v.P(z.x, z.y); g.beginPath(); g.arc(zp.x, zp.y, z.r * v.s, 0, 2 * Math.PI);
    g.fillStyle = "#D9303622"; g.fill(); g.setLineDash([6, 4]); g.strokeStyle = C.zone; g.lineWidth = 1.5; g.stroke(); g.setLineDash([]);
  }
  // trail: decimated server side (>= 12 cm apart, last 12 m); drawn as a smooth curve that fades towards the old end, ending at the robot
  const D = S.disp;
  const tr = (st.trail || []).map((p) => v.P(p[0], p[1]));
  if (D) tr.push(v.P(D.x, D.y));
  if (tr.length > 1) {
    g.lineWidth = 3; g.lineCap = "round"; g.lineJoin = "round";
    const CH = Math.min(8, tr.length - 1), per = (tr.length - 1) / CH;
    for (let c = 0; c < CH; c++) {
      const i0 = Math.floor(c * per), i1 = Math.min(tr.length - 1, Math.ceil((c + 1) * per));
      g.beginPath(); g.moveTo(tr[i0].x, tr[i0].y);
      for (let i = i0 + 1; i < i1; i++) { const mx = (tr[i].x + tr[i + 1].x) / 2, my = (tr[i].y + tr[i + 1].y) / 2; g.quadraticCurveTo(tr[i].x, tr[i].y, mx, my); }
      g.lineTo(tr[i1].x, tr[i1].y);
      g.strokeStyle = `rgba(45,212,191,${0.18 + 0.6 * ((c + 1) / CH)})`; g.stroke();
    }
  }
  const path = st.path || [];
  if (path.length > 1) {
    const p0 = D ? v.P(D.x, D.y) : v.P(...path[0]);
    g.beginPath(); g.moveTo(p0.x, p0.y);
    for (let i = 1; i < path.length; i++) { const q = v.P(...path[i]); g.lineTo(q.x, q.y); }
    g.setLineDash([8, 6]); g.lineWidth = 2.5; g.strokeStyle = "#FACC15CC"; g.stroke(); g.setLineDash([]);
    const e = v.P(...path[path.length - 1]);
    g.beginPath(); g.arc(e.x, e.y, 9, 0, 6.2832); g.strokeStyle = "#FACC15"; g.lineWidth = 2; g.stroke();
    g.beginPath(); g.arc(e.x, e.y, 3, 0, 6.2832); g.fillStyle = "#FACC15"; g.fill();
  }
  for (const m of st.moves || []) drawMove(g, v, m);                                // objects the robot saw MOVE: old spot -> new spot
  // markers first, then labels laid out so none overlap and all stay inside the map
  const items = [];
  for (const [name, p] of Object.entries(S.map.places || {})) {
    if (name === "doorway") continue;
    const pp = v.P(p.x, p.y);
    g.fillStyle = C.muted; g.beginPath(); g.arc(pp.x, pp.y, 3, 0, 6.2832); g.fill();
    items.push({ text: prettyName(name), ax: pp.x, ay: pp.y, r: 3, kind: "place" });          // room centres too (the corner titles stay)
  }
  const sz = Math.max(6, Math.min(11, 0.16 * v.s));
  for (const o of st.objects || []) {
    const op = v.P(o.x, o.y);
    drawObject(g, o, op.x, op.y, sz);
    items.push({ text: objectLabel(o), ax: op.x, ay: op.y, r: sz, kind: "obj" });
  }
  const RR = Math.max(13, 0.2 * v.s), rp = D ? v.P(D.x, D.y) : null;
  if (D) items.push({ text: "Gurdy", ax: rp.x, ay: rp.y, r: RR * 1.9, kind: "robot" });
  for (const l of labels) { g.fillStyle = "#6B7886"; g.textAlign = "left"; g.font = "600 12px 'Space Grotesk', Inter, sans-serif"; g.fillText(l.t, l.box.x, l.box.y + LH / 2); }
  g.font = FONT;
  const markers = items.map((i) => { const r = i.kind === "robot" ? RR * 1.6 : i.r; return { x: i.ax - r, y: i.ay - r, w: 2 * r, h: 2 * r }; });
  for (const l of placeLabels(items, w, h, measure, LH, taken.concat(markers))) {
    const kind = items.find((i) => i.text === l.text && i.ax === l.ax)?.kind;
    g.textAlign = "left";
    if (l.leader) {                                                 // label moved away from a crowded marker: thin line back to it
      g.beginPath(); g.moveTo(l.ax, l.ay); g.lineTo(Math.min(Math.max(l.ax, l.x), l.x + l.w), Math.min(Math.max(l.ay, l.y), l.y + l.h));
      g.strokeStyle = "#8B98A588"; g.lineWidth = 1; g.stroke();
    }
    g.fillStyle = "#0B0E13BB"; g.beginPath(); g.roundRect(l.x - 3, l.y - 1, l.w + 6, l.h + 2, 5); g.fill();
    g.fillStyle = kind === "robot" ? C.accent : kind === "obj" ? C.text : C.muted;
    g.fillText(l.text, l.x, l.y + l.h / 2);
  }
  if (D) drawRobot(g, rp.x, rp.y, v.ang(D.h), RR);
}

/** Arrow from where an object used to be to where it is now (Incident Graph: object_moved), with a hollow ghost at the old spot. */
function drawMove(g, v, m) {
  if (!m.from || !m.to) return;
  const a = v.P(m.from.x, m.from.y), b = v.P(m.to.x, m.to.y), L = Math.hypot(b.x - a.x, b.y - a.y);
  if (L < 6) return;
  const col = "#FB923C", ux = (b.x - a.x) / L, uy = (b.y - a.y) / L, head = Math.min(12, L * 0.4), tip = { x: b.x - ux * 9, y: b.y - uy * 9 };
  g.save();
  g.strokeStyle = col; g.fillStyle = col; g.lineWidth = 2; g.setLineDash([6, 4]);
  g.beginPath(); g.arc(a.x, a.y, 7, 0, 6.2832); g.stroke();                       // ghost of the old position
  g.beginPath(); g.moveTo(a.x + ux * 8, a.y + uy * 8); g.lineTo(tip.x, tip.y); g.stroke();
  g.setLineDash([]);
  g.beginPath(); g.moveTo(tip.x + ux * head * 0.8, tip.y + uy * head * 0.8); g.lineTo(tip.x - uy * head * 0.45, tip.y + ux * head * 0.45); g.lineTo(tip.x + uy * head * 0.45, tip.y - ux * head * 0.45); g.closePath(); g.fill();
  g.restore();
}

/** Smooth motion: state arrives at 2 Hz, the marker eases towards the newest pose every animation frame. */
function frame() {
  const st = S.state;
  if (st && st.pose) {
    const T = { x: st.pose.x, y: st.pose.y, h: (st.pose.heading_deg || 0) * Math.PI / 180 };
    const D = S.disp || (S.disp = { ...T });
    const dx = T.x - D.x, dy = T.y - D.y, dh = angDiff(D.h, T.h);
    if (Math.hypot(dx, dy) > 3) { D.x = T.x; D.y = T.y; } else { D.x += dx * 0.18; D.y += dy * 0.18; }
    D.h += dh * 0.2;
    S.settled = Math.hypot(dx, dy) < 0.002 && Math.abs(dh) < 0.002;
  }
  drawMap();
  S.raf = S.settled && !S.dirty ? 0 : requestAnimationFrame(frame);
  S.dirty = false;
}
function kick() { S.dirty = true; S.settled = false; if (S.tab === "map" || isDesktop()) { if (!S.raf) S.raf = requestAnimationFrame(frame); } }
const isDesktop = () => window.matchMedia("(min-width: 900px)").matches;

// ---------------------------------------------------------------- camera (real robot: JPEG + detector boxes, ~2 Hz while the panel is visible)
// Phone: a collapsible row under the map (closed by default). Laptop: a docked card in the right column. Never drawn over the map.
const cam = { busy: false, img: new Image(), open: false, available: false };
function placeCam() {
  const box = $("camBox"), slot = isDesktop() ? $("camSlotDesk") : $("camSlotPhone");
  if (box.parentElement !== slot) slot.appendChild(box);
  box.classList.toggle("open", isDesktop() || cam.open);
  $("camHead").setAttribute("aria-expanded", String(isDesktop() || cam.open));
}
function setCameraAvailable(v) { cam.available = v; $("camBox").hidden = !v; $("app").toggleAttribute("data-cam", v); if (v) placeCam(); }
$("camHead").addEventListener("click", () => { if (isDesktop()) return; cam.open = !cam.open; placeCam(); if (cam.open) pollCamera(); kick(); });
async function pollCamera() {
  const visible = isDesktop() || cam.open;
  if (cam.busy || document.hidden || !cam.available || !visible) return;
  cam.busy = true;
  try {
    const c = await api("/api/camera");
    if (!c.available) return;
    await new Promise((res, rej) => { cam.img.onload = res; cam.img.onerror = rej; cam.img.src = "data:image/jpeg;base64," + c.jpeg_b64; });
    const cv = $("camCanvas"), g = cv.getContext("2d");
    if (cv.width !== c.w || cv.height !== c.h) { cv.width = c.w; cv.height = c.h; }
    g.drawImage(cam.img, 0, 0, c.w, c.h);
    const sx = c.w / (c.boxes_w || c.w), sy = c.h / (c.boxes_h || c.h);
    g.font = "600 13px Inter, system-ui, sans-serif"; g.textBaseline = "top"; g.lineWidth = 2;
    for (const b of c.boxes || []) {
      const [x1, y1, x2, y2] = b.box, label = `${prettyName(b.name)} ${Math.round(b.score * 100)}%`;
      g.strokeStyle = "#2DD4BF"; g.strokeRect(x1 * sx, y1 * sy, (x2 - x1) * sx, (y2 - y1) * sy);
      const tw = g.measureText(label).width + 8, ly = Math.max(0, y1 * sy - 17);
      g.fillStyle = "#2DD4BF"; g.fillRect(x1 * sx, ly, tw, 17);
      g.fillStyle = "#062B27"; g.fillText(label, x1 * sx + 4, ly + 2);
    }
    const n = (c.boxes || []).length;
    $("camInfo").textContent = n ? `${n} detected` : "nothing detected";
    $("camList").replaceChildren(...(c.boxes || []).map((b) => { const s = document.createElement("span"); s.textContent = `${prettyName(b.name)} ${Math.round(b.score * 100)}%`; return s; }));
  } catch { /* a failed poll just leaves the last picture */ } finally { cam.busy = false; }
}
setInterval(pollCamera, 500);
new ResizeObserver(() => $("app").style.setProperty("--camh", $("camBox").offsetHeight + "px")).observe($("camBox"));
window.matchMedia("(min-width: 900px)").addEventListener("change", () => { placeCam(); kick(); });

// ---------------------------------------------------------------- voice: hold-to-talk (MediaRecorder -> /api/voice) and spoken replies (/api/tts)
const mic = { stream: null, rec: null, chunks: [], t0: 0, busy: false, idle: 0, down: false };
const toastEl = () => $("toast");
function toast(text, ms = 4000) { const t = toastEl(); t.textContent = text; t.hidden = false; clearTimeout(toast.k); toast.k = setTimeout(() => { t.hidden = true; }, ms); }

function setupVoice() {
  S.muted = load("gurdy_muted") === "1";
  const on = S.voice || S.browserVoice;
  for (const id of ["micBtn", "micFab"]) $(id).hidden = !on;
  $("muteBtn").hidden = !on;
  $("muteBtn").setAttribute("aria-pressed", String(S.muted));
}

function micSupport() {
  if (!window.isSecureContext || !navigator.mediaDevices || !window.MediaRecorder)
    return "The microphone needs a secure page: open localhost:8080 on this computer, or the https address on the phone (docs/phase7_voice.md).";
  return "";
}

function recorderType() {
  for (const t of ["audio/webm;codecs=opus", "audio/webm", "audio/mp4", "audio/ogg;codecs=opus"]) if (MediaRecorder.isTypeSupported(t)) return t;
  return "";
}

function unlockAudio() {
  try { if (!S.ac) S.ac = new (window.AudioContext || window.webkitAudioContext)(); if (S.ac.state === "suspended") S.ac.resume(); } catch { /* no audio output */ }
}

async function micStart() {
  if (mic.busy || mic.rec) return;
  const why = micSupport();
  if (why) { toast(why, 7000); return; }
  mic.down = true;
  stopSpeaking(); unlockAudio();                                  // barge-in: talking over the robot silences it
  if (S.browserVoice && window.GURDY_BROWSER_VOICE.state === "idle") {          // lazy: the speech model is only downloaded when the visitor first presses the mic
    window.GURDY_BROWSER_VOICE.onProgress(showModelProgress);
    window.GURDY_BROWSER_VOICE.ensureLoaded().catch(() => {});
  }
  try {
    if (!mic.stream) mic.stream = await navigator.mediaDevices.getUserMedia({ audio: { channelCount: 1, echoCancellation: true, noiseSuppression: true } });
  } catch (e) { mic.down = false; toast(e && e.name === "NotAllowedError" ? "Microphone permission was denied. Allow it in the browser's site settings." : "No microphone available: " + (e && e.message || e), 7000); return; }
  clearTimeout(mic.idle);
  if (!mic.down) { micIdleRelease(); return; }                    // the button was released while the permission prompt was open
  const type = recorderType();
  mic.chunks = [];
  mic.rec = new MediaRecorder(mic.stream, type ? { mimeType: type } : undefined);
  mic.rec.ondataavailable = (e) => { if (e.data && e.data.size) mic.chunks.push(e.data); };
  mic.rec.onstop = micSend;
  mic.t0 = performance.now();
  mic.rec.start();
  setMicState("rec");
}

function micStop() {
  mic.down = false;
  if (mic.rec && mic.rec.state === "recording") setTimeout(() => { if (mic.rec && mic.rec.state === "recording") mic.rec.stop(); }, 250);   // the last word is often still being said
}

function micIdleRelease() { clearTimeout(mic.idle); mic.idle = setTimeout(() => { if (mic.stream && !mic.rec) { mic.stream.getTracks().forEach((t) => t.stop()); mic.stream = null; } }, 20000); }

function setMicState(s) {
  for (const id of ["micBtn", "micFab"]) { const b = $(id); b.classList.toggle("rec", s === "rec"); b.classList.toggle("busy", s === "busy"); }
  $("micFabText").textContent = s === "rec" ? "Listening…" : s === "busy" ? "Thinking…" : "Hold to talk";
}

async function micSend() {
  const dur = (performance.now() - mic.t0) / 1000, blob = new Blob(mic.chunks, { type: (mic.rec && mic.rec.mimeType) || "audio/webm" });
  mic.rec = null; micIdleRelease();
  if (dur < 0.4 || blob.size < 300) { setMicState("idle"); toast("Too short. Hold the button while you speak."); return; }
  mic.busy = true; setMicState("busy");
  S.voiceUsed = true;
  if (S.browserVoice) return browserVoiceSend(blob);
  try {
    const r = await fetch(baseUrl() + "/api/voice", { method: "POST", headers: S.demo ? {} : { Authorization: "Bearer " + S.token, "Content-Type": blob.type || "application/octet-stream" }, body: blob });
    if (r.status === 401) throw new Error("not authorised");
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(j.error || r.statusText);
    if (!S.ws) await pollChat();                                  // without a WebSocket the transcript / reply arrive by polling
  } catch (e) { addChat({ seq: -1, role: "system", text: "Voice failed: " + e.message }); }
  finally { mic.busy = false; setMicState("idle"); }
}

// Static demo: the browser's own Whisper (transformers.js, downloaded on the first press) transcribes on this device; the text goes to the in-browser agent through
// the same rules as the real pipeline (vocabulary prompt, yes/no hint, hallucination filter, stop fast path - web/static_demo/session.py). Audio never leaves the device.
async function browserVoiceSend(blob) {
  const BV = window.GURDY_BROWSER_VOICE;
  try {
    const ctx = await api("/api/voice_context");
    const tr = await BV.transcribe(blob, ctx);                                  // waits for the model download on the very first use
    window.__lastVoice = { text: tr.text, seconds: tr.seconds, device: tr.device, rms: tr.rms, audioSeconds: tr.audioSeconds, usedPrompt: tr.usedPrompt };
    await api("/api/voice_text", { method: "POST", body: JSON.stringify({ text: tr.text, rms: tr.rms }) });
    if (!S.ws) await pollChat();
  } catch (e) { addChat({ seq: -1, role: "system", text: "Voice failed: " + e.message }); }
  finally { mic.busy = false; setMicState("idle"); }
}
function showModelProgress(p) {
  const box = $("voiceLoad");
  if (p.state === "ready" || p.state === "idle") { box.hidden = true; return; }
  box.hidden = false;
  const BV = window.GURDY_BROWSER_VOICE, mb = (x) => (x / 1048576).toFixed(0);
  $("vlBar").style.width = (p.pct || 3) + "%";
  $("vlText").textContent = p.error ? "Speech model failed: " + p.error : p.total ? `Downloading the speech model (${BV.model}) ${mb(p.loaded)} of ${mb(p.total)} MB, once; it stays in your browser` : `Preparing the speech model (${BV.model}, about ${BV.approxMb} MB)…`;
}

// replies: one queue, one at a time; a new recording / the e-stop clears it
const tts = { queue: [], playing: null, gen: 0 };
async function speak(text) {
  if (S.browserVoice) { if (!S.muted) window.GURDY_BROWSER_VOICE.speak(text); return; }       // the browser's own voices (speechSynthesis)
  if (!S.ac || !S.voice) return;
  const gen = tts.gen;
  try {
    const r = await fetch(baseUrl() + "/api/tts", { method: "POST", headers: { Authorization: "Bearer " + S.token, "Content-Type": "application/json" }, body: JSON.stringify({ text }) });
    if (!r.ok) return;
    const buf = await S.ac.decodeAudioData(await r.arrayBuffer());
    if (gen !== tts.gen) return;                                  // cancelled while synthesizing
    tts.queue.push(buf); playNext();
  } catch { /* spoken replies are a convenience: the text is already in the chat */ }
}
function playNext() {
  if (tts.playing || !tts.queue.length || S.muted) return;
  const src = S.ac.createBufferSource(); src.buffer = tts.queue.shift(); src.connect(S.ac.destination);
  tts.playing = src; src.onended = () => { tts.playing = null; playNext(); }; src.start();
}
function stopSpeaking() { if (window.GURDY_BROWSER_VOICE) window.GURDY_BROWSER_VOICE.stopSpeaking(); tts.gen++; tts.queue = []; if (tts.playing) { try { tts.playing.onended = null; tts.playing.stop(); } catch { /* already stopped */ } tts.playing = null; } }

function wireVoiceButtons() {
  for (const id of ["micBtn", "micFab"]) {
    const b = $(id);
    b.addEventListener("pointerdown", (e) => { e.preventDefault(); try { b.setPointerCapture(e.pointerId); } catch { /* old browser */ } micStart(); });
    for (const ev of ["pointerup", "pointercancel"]) b.addEventListener(ev, (e) => { e.preventDefault(); micStop(); });
    b.addEventListener("contextmenu", (e) => e.preventDefault());              // long-press must not open the browser menu
  }
  $("muteBtn").addEventListener("click", () => { S.muted = !S.muted; store("gurdy_muted", S.muted ? "1" : "0"); $("muteBtn").setAttribute("aria-pressed", String(S.muted)); if (S.muted) stopSpeaking(); });
  // laptop: hold Space (outside text fields) to talk
  addEventListener("keydown", (e) => { if (e.code === "Space" && !e.repeat && S.voice && isDesktop() && !/INPUT|TEXTAREA|BUTTON/.test(document.activeElement.tagName)) { e.preventDefault(); micStart(); } });
  addEventListener("keyup", (e) => { if (e.code === "Space" && mic.down) micStop(); });
}
wireVoiceButtons();

// ---------------------------------------------------------------- UI wiring
// ---------------------------------------------------------------- timeline (Incident Graph)
const tlQuery = () => {
  const q = new URLSearchParams({ since: $("tlSince").value });
  if ($("tlRoom").value) q.set("room", $("tlRoom").value);
  const o = $("tlObj").value; if (o) { const [cls, col] = o.split("|"); q.set("class", cls); if (col) q.set("color", col); }
  return q;
};
const objName = (o) => [o.color, String(o.class || "").replace(/_/g, " ")].filter(Boolean).join(" ");

function fillTlFilters() {
  const rooms = Object.keys((S.map && S.map.rooms) || {});
  const have = (sel, v) => [...sel.options].some((o) => o.value === v);
  for (const r of rooms) if (!have($("tlRoom"), r)) $("tlRoom").add(new Option(prettyName(r), r));
  for (const o of (S.state && S.state.objects) || []) { const v = o.class + "|" + (o.color || ""); if (!have($("tlObj"), v)) $("tlObj").add(new Option(objName(o), v)); }
}

async function loadTimeline() {
  if (!S.map) return;
  fillTlFilters();
  let r; try { r = await api("/api/timeline?" + tlQuery()); } catch { return; }
  const ul = $("tlList");
  if (!r.available) { $("tlCount").textContent = "–"; return ul.replaceChildren(emptyLi("This robot keeps no timeline in this session.")); }
  if (r.busy || r.error) return;
  const d = r.data;
  $("tlCount").textContent = d.total + (d.truncated ? " (latest " + d.events.length + ")" : "");
  ul.replaceChildren(...(d.events.length ? d.events.slice().reverse().map((e) => {
    const li = document.createElement("li"); li.className = e.type;
    const t = document.createElement("span"); t.textContent = e.text;
    const w = document.createElement("span"); w.className = "when"; w.textContent = e.clock; w.title = e.ago_text;
    if (e.sample) { const g = document.createElement("span"); g.className = "sample-tag"; g.textContent = "sample"; g.title = "Invented demo history, not something this robot did"; li.append(t, g, w); return li; }
    li.append(t, w); return li;
  }) : [emptyLi("Nothing recorded for this filter " + d.since_text + ".")]));
}

async function showChanged() {
  const q = tlQuery(); q.delete("class"); q.delete("color");
  let r; try { r = await api("/api/changes?" + q); } catch (e) { return; }
  const a = $("tlAnswer"); a.hidden = false;
  a.textContent = r.available ? (r.text || r.error || "Robot is busy, try again.") : "This robot keeps no timeline in this session.";
}

setInterval(() => { if (!document.hidden && S.map && (S.tab === "timeline" || isDesktop())) loadTimeline(); }, 4000);
["tlSince", "tlRoom", "tlObj"].forEach((id) => $(id).addEventListener("change", () => { $("tlAnswer").hidden = true; loadTimeline(); }));
$("tlChanged").addEventListener("click", showChanged);

function showTab(name) {
  S.tab = name; $("app").dataset.tab = name;
  for (const v of document.querySelectorAll(".view")) v.classList.toggle("active", v.id === "view-" + name);
  for (const t of document.querySelectorAll(".tab")) t.classList.toggle("active", t.dataset.view === name);
  if (name === "map") kick();
  if (name === "chat") { const l = $("chatList"); l.scrollTop = l.scrollHeight; }
  if (name === "timeline") loadTimeline();
}

function askToken(err) {
  const d = $("login");
  $("loginErr").hidden = !err; $("loginErr").textContent = err || "";
  if (!d.open) d.showModal();
}

document.querySelectorAll(".tab").forEach((t) => t.addEventListener("click", () => showTab(t.dataset.view)));
$("estop").addEventListener("click", estop);
$("release").addEventListener("click", releaseEstop);
$("chatForm").addEventListener("submit", (e) => { e.preventDefault(); const i = $("chatInput"), t = i.value.trim(); if (t) { i.value = ""; say(t); } });
$("loginForm").addEventListener("submit", () => {
  S.token = $("tokenInput").value.trim(); $("tokenInput").value = "";
  if (window.GURDY_REMOTE) { window.GURDY_REMOTE.token = S.token; try { sessionStorage.setItem("gurdy_remote", JSON.stringify(window.GURDY_REMOTE)); } catch { /* */ } }
  else store("gurdy_token", S.token);                                           // never persist a robot token typed in the static demo
  connect();
});
window.addEventListener("resize", kick);
new ResizeObserver(kick).observe($("map"));

const fromUrl = new URLSearchParams(location.hash.slice(1)).get("token");      // http://host:8080/#token=...  (a fragment never reaches server logs)
if (fromUrl && !window.GURDY_STATIC_DEMO) { store("gurdy_token", fromUrl); history.replaceState(null, "", location.pathname); }
S.token = load("gurdy_token");
connect();

// Offline start: the service worker caches the app shell only (never /api or the WebSocket). It needs a secure context
// (https or localhost); over plain http on the LAN the browser simply refuses and the page works online as before.
if ("serviceWorker" in navigator && !window.GURDY_STATIC_DEMO) navigator.serviceWorker.register("sw.js").catch(() => {});      // the static demo (GitHub Pages / static Space) ships no worker file
