// "Connect to my robot" for the static demo. Default = the in-browser simulation. This file adds a small button + dialog; after a successful check the page is reloaded in
// ROBOT mode: app.js then talks to the robot server (REST + WebSocket + its GPU voice) through window.GURDY_REMOTE.base, with a clear banner and "Back to demo".
// Storage: the robot URL is remembered in localStorage; the TOKEN lives in sessionStorage only (this tab, forgotten when it is closed). Nothing here weakens the
// robot's authentication: the token is still sent on every request, and the robot decides which web pages may call it (CORS / Origin allow-list on the server).
(() => {
  "use strict";
  const KEY_URL = "gurdy_robot_url", KEY_SESSION = "gurdy_remote";
  const MSG = {
    unreachable: "Your robot isn't running or this device isn't on your Tailscale network.",
    unreachableHint: " If it is running and you are on Tailscale, its allowed-origins list (GURDY_ALLOWED_ORIGINS) must include this page's address: a browser cannot tell the two cases apart.",
    token: "That token was not accepted by the robot.",
    origin: "The robot refused this web page (its allowed-origins list does not include it).",
    notrobot: "That address answered, but it is not a Gurdy robot server.",
    notready: "The robot server is up but its simulation is not ready yet. Try again in a minute.",
    badurl: "Enter the robot address, as printed by the robot start script (https, or http://localhost).",
  };
  const ls = { get: (k) => { try { return localStorage.getItem(k) || ""; } catch { return ""; } }, set: (k, v) => { try { localStorage.setItem(k, v); } catch { /* private mode */ } } };
  const ss = { get: (k) => { try { return sessionStorage.getItem(k) || ""; } catch { return ""; } }, set: (k, v) => { try { sessionStorage.setItem(k, v); } catch { /* */ } }, del: (k) => { try { sessionStorage.removeItem(k); } catch { /* */ } } };

  /** "https://<robot-address>/#token=<token>" | "<robot-address>" -> {base, token}; base = origin only. Throws on anything that is not https (or http on localhost). */
  function parseRobotUrl(text) {
    let raw = String(text || "").trim();
    if (!raw) throw new Error("badurl");
    if (!/^[a-z]+:\/\//i.test(raw)) raw = "https://" + raw;
    let u; try { u = new URL(raw); } catch { throw new Error("badurl"); }
    if (!/^([a-z0-9-]+\.)*[a-z0-9-]+$|^\[[0-9a-f:]+\]$/i.test(u.hostname)) throw new Error("badurl");     // a real host name or IP, nothing odd
    const local = ["localhost", "127.0.0.1", "[::1]"].includes(u.hostname);
    if (!(u.protocol === "https:" || (u.protocol === "http:" && local))) throw new Error("badurl");
    const token = new URLSearchParams(u.hash.replace(/^#/, "")).get("token") || u.searchParams.get("token") || "";
    return { base: u.origin, token };
  }

  /** Check a robot: health (public) then an authenticated call. -> {ok:true} | {ok:false, why:"unreachable"|"origin"|"notrobot"|"token"|"notready"} */
  async function probe(base, token, timeoutMs = 7000) {
    const get = async (path, headers) => {
      const ctl = new AbortController(), t = setTimeout(() => ctl.abort(), timeoutMs);
      try { return await fetch(base + path, { headers, cache: "no-store", signal: ctl.signal }); } finally { clearTimeout(t); }
    };
    let h;
    try { h = await get("/api/health"); } catch { return { ok: false, why: "unreachable" }; }       // network error, DNS, timeout, blocked by CORS: all look the same to a page
    if (h.status === 403) return { ok: false, why: "origin" };
    let body = null; try { body = await h.json(); } catch { /* not JSON */ }
    if (!h.ok || !body || body.ok !== true || body.demo === true) return { ok: false, why: "notrobot" };
    let s;
    try { s = await get("/api/state", { Authorization: "Bearer " + token }); } catch { return { ok: false, why: "unreachable" }; }
    if (s.status === 401) return { ok: false, why: "token" };
    if (s.status === 403) return { ok: false, why: "origin" };
    if (!s.ok) return { ok: false, why: "notready" };
    const st = await s.json().catch(() => ({}));
    return st.connected === false ? { ok: false, why: "notready" } : { ok: true, voice: !!body.voice };
  }

  const toast = (text) => { const t = document.createElement("div"); t.className = "remote-toast"; t.textContent = text; document.body.appendChild(t); setTimeout(() => t.remove(), 9000); };
  const backToDemo = () => { ss.del(KEY_SESSION); location.reload(); };

  // ---- decide the mode BEFORE app.js connects (app.js awaits this promise)
  let realBar = null;
  const stored = (() => { try { return JSON.parse(ss.get(KEY_SESSION) || "null"); } catch { return null; } })();
  window.GURDY_MODE_READY = stored && stored.base ? probe(stored.base, stored.token).then((r) => {
    if (r.ok) { window.GURDY_REMOTE = stored; return; }
    ss.del(KEY_SESSION);                                                             // not reachable now: stay in demo mode, tell the visitor
    const say = () => toast(MSG[r.why] + " Back in demo mode.");
    document.readyState === "loading" ? document.addEventListener("DOMContentLoaded", say) : say();          // the probe may finish after the page has loaded
  }) : Promise.resolve();

  // ---- UI
  function ui() {
    const banners = document.querySelector(".banners");
    if (window.GURDY_MODE_READY) window.GURDY_MODE_READY.then(() => {
      if (window.GURDY_REMOTE) {
        realBar = document.createElement("div");
        realBar.className = "banner real";
        realBar.innerHTML = '<span id="realText"><b>Connected to REAL robot</b> <span class="host"></span></span><button type="button" class="relbtn" id="backDemo">Back to demo</button>';
        realBar.querySelector(".host").textContent = new URL(window.GURDY_REMOTE.base).host;
        (banners || document.body).prepend(realBar);
        realBar.querySelector("#backDemo").addEventListener("click", backToDemo);
        document.getElementById("demoBanner") && (document.getElementById("demoBanner").hidden = true);
      } else {
        const btn = document.createElement("button");
        btn.type = "button"; btn.className = "remote-btn"; btn.id = "remoteBtn"; btn.innerHTML = '<span class="long">Connect to my robot</span><span class="short">My robot</span>';
        const top = document.querySelector(".top"), estop = document.getElementById("estop");
        top && top.insertBefore(btn, estop);
        btn.addEventListener("click", openDialog);
      }
    });
  }

  let dlg = null;
  function openDialog() {
    if (!dlg) {
      dlg = document.createElement("dialog");
      dlg.id = "remoteDialog";
      dlg.innerHTML = '<form method="dialog" id="remoteForm"><h2>Connect to my robot</h2>' +
        '<p>Paste your robot address and access token. Works only from a device on your Tailscale network. The address is remembered; the token is kept for this tab only.</p>' +
        '<label for="robotUrl">Robot address</label><input id="robotUrl" type="text" inputmode="url" autocomplete="off" spellcheck="false" placeholder="https://<robot address>">' +
        '<label for="robotToken">Access token <span class="opt">(if not in the address)</span></label><input id="robotToken" type="password" autocomplete="off">' +
        '<p class="err" id="robotErr" hidden></p><div class="row2"><button type="button" class="relbtn" id="robotCancel">Cancel</button><button type="submit" class="send" id="robotGo">Connect</button></div></form>';
      document.body.appendChild(dlg);
      dlg.querySelector("#robotCancel").addEventListener("click", () => dlg.close());
      dlg.querySelector("#remoteForm").addEventListener("submit", (e) => { e.preventDefault(); submit(); });
    }
    dlg.querySelector("#robotUrl").value = ls.get(KEY_URL);
    dlg.querySelector("#robotToken").value = "";
    err("");
    dlg.showModal();
  }
  function err(text) { const e = dlg.querySelector("#robotErr"); e.textContent = text; e.hidden = !text; }

  async function submit() {
    const go = dlg.querySelector("#robotGo");
    let parsed;
    try { parsed = parseRobotUrl(dlg.querySelector("#robotUrl").value); } catch { return err(MSG.badurl); }
    const token = dlg.querySelector("#robotToken").value.trim() || parsed.token;
    if (!token) return err("Enter the access token.");
    go.disabled = true; go.textContent = "Checking…"; err("");
    const r = await probe(parsed.base, token);
    go.disabled = false; go.textContent = "Connect";
    if (!r.ok) return err(MSG[r.why] + (r.why === "unreachable" ? MSG.unreachableHint : ""));      // stay in demo mode
    ls.set(KEY_URL, parsed.base);                                                       // the URL is remembered, the token is not
    ss.set(KEY_SESSION, JSON.stringify({ base: parsed.base, token }));
    location.reload();
  }

  // ---- status hook for app.js (connection lost / recovered while in robot mode)
  window.GURDY_REMOTE_UI = {
    status(kind) {
      if (!realBar) return;
      const el = realBar.querySelector("#realText");
      if (kind === "ok") el.innerHTML = '<b>Connected to REAL robot</b> <span class="host"></span>';
      else el.innerHTML = "<b>" + (kind === "token" ? MSG.token : MSG.unreachable) + "</b> Retrying… <span class=\"host\"></span>";
      el.querySelector(".host").textContent = window.GURDY_REMOTE ? new URL(window.GURDY_REMOTE.base).host : "";
      realBar.classList.toggle("lost", kind !== "ok");
    },
  };
  window.GURDY_REMOTE_TEST = { parseRobotUrl, probe, MSG };          // exposed for the browser tests

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", ui); else ui();
})();
