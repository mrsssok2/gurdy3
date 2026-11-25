"""Phase 7 step 1 tests. Run: python3 -m unittest web.tests.test_api_server   (stdlib only; real MockBackend, fake agent host; no Docker/ROS/LLM)."""
import base64
import http.client
import json
import os
import socket
import threading
import time
import unittest
from io import BytesIO

from brain.mock import MockBackend
from brain.tools import ToolLayer
from web.api import GurdyWebServer, MockTelemetry, ws

TOKEN = "test-token-123"


class FakeAgent:
    def __init__(self):
        self.layer = ToolLayer(MockBackend())


class FakeHost:
    """Same surface as AgentHost used by the server: agent.layer.backend, lock, say, estop, on_message."""

    def __init__(self):
        self.agent, self.lock, self.listeners = FakeAgent(), threading.Lock(), []
        self.estops, self.say_delay = 0, 0.0

    def say(self, text, conf=None, started_ago=None):
        with self.lock:
            time.sleep(self.say_delay)
            if "walk" in text:
                self.agent.layer.backend.walk("forward", 1.0)
            return f"ok: {text}"

    def estop(self):                                    # like AgentHost.estop: no lock
        self.estops += 1
        self.agent.layer.call("stop")

    def on_message(self, fn):
        self.listeners.append(fn)


def request(srv, method, path, body=None, token=TOKEN, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", srv.port, timeout=5)
    h = dict(headers or {})
    if token:
        h["Authorization"] = f"Bearer {token}"
    data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body))
    c.request(method, path, data, h)
    r = c.getresponse()
    raw = r.read()
    c.close()
    try:
        return r.status, json.loads(raw)
    except ValueError:
        return r.status, raw


class WsClient:
    """Tiny blocking WebSocket test client."""

    def __init__(self, srv, token=TOKEN, path=None):
        self.s = socket.create_connection(("127.0.0.1", srv.port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall((f"GET {path or '/ws?token=' + str(token)} HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        self.f = self.s.makefile("rb")
        self.status = int(self.f.readline().split()[1])
        self.headers = {}
        while (line := self.f.readline()) not in (b"\r\n", b""):
            k, _, v = line.decode().partition(":")
            self.headers[k.strip().lower()] = v.strip()
        self.key = key

    def send(self, obj):
        self.s.sendall(ws.encode_frame(ws.OP_TEXT, json.dumps(obj).encode(), os.urandom(4)))

    def recv(self):
        def read_unmasked():
            b0, b1 = self.f.read(2)
            n = b1 & 0x7F
            if n == 126:
                n = int.from_bytes(self.f.read(2), "big")
            elif n == 127:
                n = int.from_bytes(self.f.read(8), "big")
            return b0 & 0x0F, self.f.read(n)
        return read_unmasked()

    def recv_json(self, type_=None, limit=20):
        for _ in range(limit):
            op, data = self.recv()
            if op == ws.OP_TEXT:
                o = json.loads(data)
                if type_ is None or o.get("type") == type_:
                    return o
        raise AssertionError(f"no {type_} message")

    def close(self):
        self.f.close()
        self.s.close()


class Base(unittest.TestCase):
    def setUp(self):
        self.host = FakeHost()
        self.srv = GurdyWebServer(self.host, token=TOKEN, port=0, state_s=0.05).start()

    def tearDown(self):
        self.srv.close()


class RestTests(Base):
    def test_health_is_public(self):
        self.assertEqual(request(self.srv, "GET", "/api/health", token=None), (200, {"ok": True, "version": "0.1", "demo": False, "voice": False}))

    def test_auth_required_everywhere(self):
        for m, p in (("GET", "/api/state"), ("GET", "/api/map"), ("GET", "/api/messages"), ("POST", "/api/say"), ("POST", "/api/estop")):
            self.assertEqual(request(self.srv, m, p, {"text": "x"} if m == "POST" else None, token=None)[0], 401, p)
            self.assertEqual(request(self.srv, m, p, {"text": "x"} if m == "POST" else None, token="wrong")[0], 401, p)
        self.assertEqual(self.host.estops, 0)               # an unauthenticated stop did not reach the robot
        self.assertEqual(request(self.srv, "GET", "/api/state?token=" + TOKEN, token=None)[0], 200)

    def test_state_matches_backend(self):
        s, st = request(self.srv, "GET", "/api/state")
        self.assertEqual(s, 200)
        b = self.host.agent.layer.backend
        self.assertEqual((st["pose"]["x"], st["pose"]["y"]), (round(b.x, 2), round(b.y, 2)))
        self.assertEqual((st["mode"], st["battery_pct"], st["estop"], st["stale"], st["source"]), ("idle", 100.0, False, False, "mock"))
        self.assertEqual(sorted(st["legs"].values()), ["ok"] * 6)
        self.assertTrue(st["battery_simulated"])

    def test_state_follows_motion_and_trail(self):
        b = self.host.agent.layer.backend
        request(self.srv, "GET", "/api/state")
        b.walk("forward", 1.0)
        b.advance(30)
        st = request(self.srv, "GET", "/api/state")[1]
        self.assertGreater(st["pose"]["x"], 0.5)
        self.assertGreaterEqual(len(st["trail"]), 2)
        self.assertEqual(st["trail"][-1], [st["pose"]["x"], st["pose"]["y"]])

    def test_map_geometry(self):
        s, m = request(self.srv, "GET", "/api/map")
        self.assertEqual(s, 200)
        self.assertEqual(sorted(m["rooms"]), ["room A", "room B"])
        self.assertEqual(len(m["walls"]), 6)
        self.assertIn("home", m["places"])
        self.assertEqual(set(m["bounds"]), {"x0", "y0", "x1", "y1"})

    def test_say_and_message_log(self):
        s, r = request(self.srv, "POST", "/api/say", {"text": "  hello robot "})
        self.assertEqual((s, r["reply"]), (200, "ok: hello robot"))
        msgs = request(self.srv, "GET", "/api/messages")[1]
        self.assertEqual([(m["role"], m["text"]) for m in msgs["messages"]], [("user", "hello robot"), ("robot", "ok: hello robot")])
        self.assertEqual(request(self.srv, "GET", f"/api/messages?since={msgs['seq']}")[1]["messages"], [])
        self.assertEqual(request(self.srv, "GET", "/api/messages?since=abc")[0], 400)

    def test_pump_messages_reach_chat(self):
        for fn in self.host.listeners:
            fn("Battery is low.")
        m = request(self.srv, "GET", "/api/messages")[1]["messages"]
        self.assertEqual((m[-1]["role"], m[-1]["text"]), ("robot", "Battery is low."))

    def test_say_validation(self):
        for body in ({}, {"text": ""}, {"text": "   "}, {"text": 5}, {"text": "x" * 501}, [1]):
            self.assertEqual(request(self.srv, "POST", "/api/say", body)[0], 400, body)
        self.assertEqual(request(self.srv, "POST", "/api/say", b"not json")[0], 400)
        self.assertEqual(request(self.srv, "POST", "/api/say", b"x" * 70000)[0], 413)
        self.assertEqual(request(self.srv, "GET", "/api/messages")[1]["messages"], [])

    def test_estop_stops_robot_and_is_logged(self):
        b = self.host.agent.layer.backend
        b.walk("forward", 2.0)
        self.assertEqual(request(self.srv, "GET", "/api/state")[1]["mode"], "moving")
        self.assertEqual(request(self.srv, "POST", "/api/estop", {})[0], 200)
        self.assertEqual(self.host.estops, 1)
        st = request(self.srv, "GET", "/api/state")[1]
        self.assertNotEqual(st["mode"], "moving")
        self.assertIn("Emergency stop", request(self.srv, "GET", "/api/messages")[1]["messages"][-1]["text"])

    def test_unknown_routes(self):
        self.assertEqual(request(self.srv, "GET", "/api/nope")[0], 404)
        self.assertEqual(request(self.srv, "POST", "/api/nope", {})[0], 404)
        self.assertEqual(request(self.srv, "POST", "/other", {})[0], 404)


class SlowTurnTests(Base):
    def test_state_stale_not_blocked_and_estop_not_queued(self):
        self.srv.telemetry.lock_timeout_s = 0.05
        request(self.srv, "GET", "/api/state")                       # prime the cache
        self.host.say_delay = 1.0
        t = threading.Thread(target=request, args=(self.srv, "POST", "/api/say", {"text": "walk"}))
        t.start()
        time.sleep(0.2)                                              # the turn holds host.lock now
        t0 = time.monotonic()
        st = request(self.srv, "GET", "/api/state")[1]
        self.assertTrue(st["stale"])
        self.assertEqual(st["pose"]["place"], "room A")
        self.assertLess(time.monotonic() - t0, 0.6)
        t0 = time.monotonic()
        self.assertEqual(request(self.srv, "POST", "/api/estop", {})[0], 200)
        self.assertLess(time.monotonic() - t0, 0.5)                  # did not wait for the 1 s turn
        t.join()


class StaticTests(Base):
    def test_index_and_traversal(self):
        s, body = request(self.srv, "GET", "/", token=None)
        self.assertEqual(s, 200)
        self.assertIn(b"Gurdy", body)
        for p in ("/../server.py", "/..%2fserver.py", "/%2e%2e/api/server.py", "/missing.js", "/api"):
            self.assertIn(request(self.srv, "GET", p, token=None)[0], (404, 401), p)
        c = http.client.HTTPConnection("127.0.0.1", self.srv.port)
        c.request("GET", "/../../../etc/passwd")                    # raw path, not normalised by the client library
        self.assertEqual(c.getresponse().status, 404)


class DashboardAssets(Base):
    """No browser / node here, so check structure: served, linked files exist, tabs + style tokens from the spec, DOM ids used by app.js exist."""

    def test_assets_served_and_consistent(self):
        import re
        from pathlib import Path
        app = Path(__file__).resolve().parents[1] / "app"
        html = (app / "index.html").read_text()
        css = (app / "style.css").read_text()
        js = (app / "app.js").read_text()
        for f in ("style.css", "app.js", "lib.js"):
            st, body = request(self.srv, "GET", "/" + f, token=None)
            self.assertEqual(st, 200, f)
        self.assertEqual([t.lower() for t in re.findall(r'class="tab[^"]*" data-view="(\w+)"', html)], ["map", "chat", "status", "timeline", "alerts"])
        for color in ("#0E1116", "#161B22", "#2DD4BF", "#D93036"):
            self.assertIn(color.lower(), css.lower())
        for font in ("Inter", "Space Grotesk", "JetBrains Mono"):
            self.assertIn(font, css)
        self.assertIn("estop", html)
        for rel in re.findall(r"url\((fonts/[^)]+)\)", css):                      # bundled fonts exist, are served as woff2, licences present
            self.assertTrue((app / rel).exists(), rel)
            st, data = request(self.srv, "GET", "/" + rel, token=None)
            self.assertEqual((st, data[:4]), (200, b"wOF2"), rel)
        self.assertEqual(len(re.findall(r"@font-face", css)), 5)
        for lic in ("OFL-Inter.txt", "OFL-SpaceGrotesk.txt", "OFL-JetBrainsMono.txt"):
            self.assertIn("SIL Open Font License", (app / "fonts" / lic).read_text())
        self.assertNotIn("http://", html + css + js.replace("http://host", ""))      # offline: no CDN / external URL
        self.assertNotIn("https://", html + css + js)
        self.assertNotIn("innerHTML", js)                                          # robot text must never be parsed as HTML
        ids = set(re.findall(r'id="(\w+)"', html))
        used = set(re.findall(r'\$\("(\w+)"\)', js))
        self.assertEqual(sorted(used - ids), [])
        for name in re.findall(r'import \{[^}]*\} from "\./(\w+)\.js"', js):
            self.assertTrue((app / (name + ".js")).exists())
        exported = set(re.findall(r"export function (\w+)", (app / "lib.js").read_text()))
        imported = set(re.findall(r"\w+", re.search(r"import \{([^}]*)\}", js).group(1)))
        self.assertEqual(sorted(imported - exported), [])


class PwaAssets(Base):
    """Manifest, icons, service worker, meta tags. The worker itself cannot run here (no browser): its rules are checked structurally."""

    APP = None

    def setUp(self):
        super().setUp()
        from pathlib import Path
        self.app = Path(__file__).resolve().parents[1] / "app"

    @staticmethod
    def png_size(data):
        import struct
        assert data[:8] == b"\x89PNG\r\n\x1a\n", "not a PNG"
        return struct.unpack(">II", data[16:24])

    def test_manifest(self):
        st, m = request(self.srv, "GET", "/manifest.json", token=None)
        self.assertEqual(st, 200)
        self.assertEqual((m["name"], m["short_name"], m["display"], m["orientation"]), ("Gurdy", "Gurdy", "standalone", "portrait"))
        self.assertEqual((m["theme_color"], m["background_color"]), ("#0E1116", "#0E1116"))
        self.assertEqual((m["start_url"], m["scope"]), ("./", "./"))
        purposes = {(i["sizes"], i["purpose"]) for i in m["icons"]}
        self.assertEqual(purposes, {("192x192", "any"), ("512x512", "any"), ("512x512", "maskable")})
        for icon in m["icons"]:
            st, data = request(self.srv, "GET", "/" + icon["src"], token=None)
            self.assertEqual(st, 200)
            w, h = self.png_size(data)
            self.assertEqual(f"{w}x{h}", icon["sizes"], icon["src"])

    def test_apple_icon_and_meta(self):
        html = (self.app / "index.html").read_text()
        for needle in ('rel="manifest"', 'rel="apple-touch-icon"', 'name="apple-mobile-web-app-capable"', 'name="mobile-web-app-capable"',
                       'name="apple-mobile-web-app-title"', 'name="theme-color" content="#0E1116"', "viewport-fit=cover"):
            self.assertIn(needle, html)
        self.assertEqual(self.png_size((self.app / "icons" / "apple-touch-icon.png").read_bytes()), (180, 180))

    def test_icons_are_dark_with_teal_art(self):
        import zlib
        data = (self.app / "icons" / "icon-maskable-512.png").read_bytes()
        idat = b""
        i = 8
        while i < len(data):
            n = int.from_bytes(data[i:i + 4], "big")
            if data[i + 4:i + 8] == b"IDAT":
                idat += data[i + 8:i + 8 + n]
            i += 12 + n
        raw = zlib.decompress(idat)
        stride = 512 * 4 + 1
        px = lambda x, y: tuple(raw[y * stride + 1 + x * 4: y * stride + 1 + x * 4 + 4])
        self.assertEqual(px(0, 0), (0x0E, 0x11, 0x16, 255))                 # maskable: full-bleed dark, opaque corner
        self.assertEqual(px(256, 256), (0x2D, 0xD4, 0xBF, 255))             # body centre is teal

    def test_service_worker_scope_and_rules(self):
        import re
        st, sw = request(self.srv, "GET", "/sw.js", token=None)
        self.assertEqual(st, 200)
        sw = sw.decode()
        shell = re.findall(r'"([^"]+)"', re.search(r"const SHELL = \[(.*?)\];", sw, re.S).group(1))
        for f in shell:                                                      # every precached file exists (a missing one would fail install)
            if f != "./":
                self.assertTrue((self.app / f).is_file(), f)
        # everything the page loads is in the shell
        html, css = (self.app / "index.html").read_text(), (self.app / "style.css").read_text()
        needed = set(re.findall(r'(?:href|src)="([^"#]+)"', html)) | set(re.findall(r"url\((fonts/[^)]+)\)", css)) | {"lib.js"}
        needed = {n for n in needed if not n.startswith("http")}
        self.assertEqual(sorted(needed - set(shell)), [])
        # robot data is never handled or stored
        self.assertIn('url.pathname.startsWith("/api/")', sw)
        self.assertIn('url.pathname === "/ws"', sw)
        self.assertIn('req.method !== "GET"', sw)
        self.assertIn("url.origin !== self.location.origin", sw)
        self.assertNotRegex(sw, r"cache\.put\([^)]*api")
        self.assertIn("serviceWorker", (self.app / "app.js").read_text())

    def test_sw_is_not_http_cached(self):
        c = http.client.HTTPConnection("127.0.0.1", self.srv.port)
        c.request("GET", "/sw.js")
        r = c.getresponse()
        r.read()
        self.assertEqual(r.getheader("Cache-Control"), "no-cache")      # a new worker is picked up on the next visit


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class DemoBase(unittest.TestCase):
    def setUp(self):
        self.host, self.clock = FakeHost(), Clock()
        self.made = []
        self.srv = None

    def make(self, **kw):
        def factory():
            a = FakeAgent()
            self.made.append(a)
            return a
        args = dict(port=0, state_s=0.05, demo=True, clock=self.clock)
        args.update(kw)
        self.srv = GurdyWebServer(self.host, **args).start()
        return self.srv

    def tearDown(self):
        if self.srv:
            self.srv.close()


class DemoMode(DemoBase):
    def test_no_token_needed_and_flagged_as_demo(self):
        srv = self.make()
        self.assertEqual(request(srv, "GET", "/api/health", token=None), (200, {"ok": True, "version": "0.1", "demo": True, "voice": False}))
        st, state = request(srv, "GET", "/api/state", token=None)
        self.assertEqual((st, state["demo"], state["source"]), (200, True, "mock"))
        self.assertEqual(request(srv, "GET", "/api/map", token="anything")[0], 200)
        c = WsClient(srv, path="/ws")
        self.assertEqual(c.status, 101)
        self.assertTrue(c.recv_json("hello")["demo"])
        c.close()

    def test_private_mode_still_needs_token_and_is_not_demo(self):
        srv = GurdyWebServer(self.host, token=TOKEN, port=0).start()
        try:
            self.assertEqual(request(srv, "GET", "/api/state", token=None)[0], 401)
            self.assertFalse(request(srv, "GET", "/api/health", token=None)[1]["demo"])
            self.assertNotIn("demo", request(srv, "GET", "/api/state")[1])
        finally:
            srv.close()

    def test_refuses_anything_but_the_mock_backend(self):
        class RealishBackend:
            pass

        class RealishHost(FakeHost):
            def __init__(self):
                super().__init__()
                self.agent.layer.backend = RealishBackend()

        class RemoteLike:                                   # like RemoteHost: no .agent at all
            lock = threading.Lock()
            def say(self, *a, **k): return ""
            def estop(self): pass

        class OtherTelemetry:
            def snapshot(self): return {}
            def map(self): return {}

        with self.assertRaises(ValueError):
            GurdyWebServer(RealishHost(), port=0, demo=True)
        with self.assertRaises(ValueError):
            GurdyWebServer(RemoteLike(), telemetry=OtherTelemetry(), port=0, demo=True)
        with self.assertRaises(ValueError):
            GurdyWebServer(FakeHost(), telemetry=OtherTelemetry(), port=0, demo=True)
        GurdyWebServer(FakeHost(), port=0, demo=True).httpd.server_close()        # the mock itself is fine

    def test_message_length_cap(self):
        srv = self.make()
        self.assertEqual(request(srv, "POST", "/api/say", {"text": "x" * 201}, token=None)[0], 400)
        self.assertEqual(request(srv, "POST", "/api/say", {"text": "x" * 200}, token=None)[0], 200)
        self.assertEqual(request(srv, "POST", "/api/say", b"x" * 5000, token=None)[0], 413)         # body cap 4 KB
        c = WsClient(srv, path="/ws")
        c.send({"op": "say", "text": "y" * 201})
        self.assertEqual(c.recv_json("error")["error"], "bad 'text'")
        c.close()

    def test_say_rate_limit_per_ip_with_retry_after(self):
        srv = self.make()
        codes = [request(srv, "POST", "/api/say", {"text": "hi"}, token=None)[0] for _ in range(7)]
        self.assertEqual(codes, [200] * 5 + [429] * 2)
        c = http.client.HTTPConnection("127.0.0.1", srv.port)
        c.request("POST", "/api/say", json.dumps({"text": "hi"}))
        r = c.getresponse()
        r.read()
        self.assertEqual(r.status, 429)
        self.assertGreaterEqual(int(r.getheader("Retry-After")), 1)
        self.clock.t += 4.1                                              # one token refilled
        self.assertEqual(request(srv, "POST", "/api/say", {"text": "hi"}, token=None)[0], 200)
        self.assertEqual(request(srv, "POST", "/api/say", {"text": "hi"}, token=None)[0], 429)

    def test_proxy_header_only_trusted_when_configured(self):
        srv = self.make()                                               # not trusted: header ignored, all requests are one client
        for i in range(5):
            self.assertEqual(request(srv, "POST", "/api/say", {"text": "hi"}, token=None, headers={"X-Forwarded-For": f"9.9.9.{i}"})[0], 200)
        self.assertEqual(request(srv, "POST", "/api/say", {"text": "hi"}, token=None, headers={"X-Forwarded-For": "9.9.9.99"})[0], 429)
        srv.close()
        srv = self.make(trust_proxy=True)                               # trusted: the proxy-appended (last) entry is the client
        for i in range(5):
            self.assertEqual(request(srv, "POST", "/api/say", {"text": "hi"}, token=None, headers={"X-Forwarded-For": "1.1.1.1, 2.2.2.2"})[0], 200)
        self.assertEqual(request(srv, "POST", "/api/say", {"text": "hi"}, token=None, headers={"X-Forwarded-For": "6.6.6.6, 2.2.2.2"})[0], 429)
        self.assertEqual(request(srv, "POST", "/api/say", {"text": "hi"}, token=None, headers={"X-Forwarded-For": "1.1.1.1, 3.3.3.3"})[0], 200)

    def test_estop_and_general_limits(self):
        srv = self.make()
        codes = [request(srv, "POST", "/api/estop", {}, token=None)[0] for _ in range(12)]
        self.assertEqual(codes, [200] * 10 + [429] * 2)
        self.assertEqual(self.host.estops, 10)                            # limited requests never reached the robot
        srv2 = self.make(clock=Clock())
        codes = [request(srv2, "GET", "/api/state", token=None)[0] for _ in range(62)]
        self.assertEqual((codes.count(200), codes.count(429)), (60, 2))

    def test_ws_say_limit_and_per_ip_connection_cap(self):
        srv = self.make()
        c = WsClient(srv, path="/ws")
        for _ in range(5):
            c.send({"op": "say", "text": "hi"})
            c.recv_json("ack")
        c.send({"op": "say", "text": "hi"})
        self.assertIn("slow down", c.recv_json("error")["error"])
        more = [WsClient(srv, path="/ws") for _ in range(2)]
        fourth = WsClient(srv, path="/ws")
        self.assertEqual(fourth.status, 503)                              # 3 per IP
        for x in [c, fourth, *more]:
            x.close()
        for _ in range(100):
            if srv.n_clients == 0:
                break
            time.sleep(0.05)
        again = WsClient(srv, path="/ws")                                 # slots are released
        self.assertEqual(again.status, 101)
        again.close()

    def test_cli_rejects_llm_in_demo_and_needs_a_mode(self):
        import subprocess
        import sys
        for args in (["--demo", "--ollama", "x"], []):
            r = subprocess.run([sys.executable, "-m", "web.api", *args], capture_output=True, text=True, timeout=30)
            self.assertEqual(r.returncode, 2, args)

    def test_demo_banner_in_page(self):
        from pathlib import Path
        app = Path(__file__).resolve().parents[1] / "app"
        self.assertIn('id="demoBanner"', (app / "index.html").read_text())
        self.assertIn("Demo: simulated robot", (app / "index.html").read_text())
        self.assertIn("S.demo", (app / "app.js").read_text())


class RateLimiterUnit(unittest.TestCase):
    def test_bucket_and_memory_bound(self):
        from web.api.limits import RateLimiter
        clk = Clock()
        rl = RateLimiter(2, 10.0, clock=clk, max_keys=5)
        self.assertEqual([rl.allow("a")[0] for _ in range(3)], [True, True, False])
        ok, wait = rl.allow("a")
        self.assertFalse(ok)
        self.assertAlmostEqual(wait, 10.0, delta=0.01)
        self.assertTrue(rl.allow("b")[0])                                 # other keys independent
        clk.t += 10
        self.assertTrue(rl.allow("a")[0])
        for i in range(50):                                               # many addresses: dictionary stays bounded
            rl.allow(f"ip{i}")
        self.assertLessEqual(len(rl._b), 6)


class WebSocketFrames(unittest.TestCase):
    def test_accept_key_rfc_example(self):
        self.assertEqual(ws.accept_key("dGhlIHNhbXBsZSBub25jZQ=="), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=")

    def test_roundtrip_lengths(self):
        for n in (0, 5, 125, 126, 4000, 70000):
            payload = os.urandom(n)
            fin, op, data = ws.read_frame(BytesIO(ws.encode_frame(ws.OP_BIN, payload, b"\x01\x02\x03\x04")), max_len=1 << 20)
            self.assertEqual((fin, op, data), (True, ws.OP_BIN, payload))

    def test_rejects_unmasked_oversized_and_reserved(self):
        with self.assertRaises(ws.WsError):
            ws.read_frame(BytesIO(ws.encode_frame(ws.OP_TEXT, b"hi")))
        with self.assertRaises(ws.WsError) as cm:
            ws.read_frame(BytesIO(ws.encode_frame(ws.OP_TEXT, b"x" * 200, b"abcd")), max_len=100)
        self.assertEqual(cm.exception.code, 1009)
        with self.assertRaises(ws.WsError):
            ws.read_frame(BytesIO(bytes([0xC1, 0x80]) + b"abcd"))

    def test_fragments_and_ping(self):
        m = b"abcd"
        part1 = bytes([0x01, 0x82]) + m + bytes(b ^ m[i % 4] for i, b in enumerate(b"he"))              # text, fin=0
        ping = ws.encode_frame(ws.OP_PING, b"p", m)
        part2 = bytes([0x80, 0x83]) + m + bytes(b ^ m[i % 4] for i, b in enumerate(b"llo"))             # continuation, fin
        sent = []
        self.assertEqual(ws.read_message(BytesIO(part1 + ping + part2), sent.append), (ws.OP_TEXT, b"hello"))
        self.assertEqual(sent, [ws.encode_frame(ws.OP_PONG, b"p")])
        self.assertIsNone(ws.read_message(BytesIO(ws.encode_frame(ws.OP_CLOSE, b"", m)), sent.append))


class WebSocketServer(Base):
    def test_requires_token(self):
        for path in ("/ws", "/ws?token=bad"):
            c = WsClient(self.srv, path=path)
            self.assertEqual(c.status, 401)
            c.close()

    def test_handshake_hello_and_state_stream(self):
        c = WsClient(self.srv)
        self.assertEqual(c.status, 101)
        self.assertEqual(c.headers["sec-websocket-accept"], ws.accept_key(c.key))
        hello = c.recv_json("hello")
        self.assertEqual(sorted(hello["map"]["rooms"]), ["room A", "room B"])
        s1 = c.recv_json("state")
        s2 = c.recv_json("state")
        self.assertEqual(s1["pose"]["place"], "room A")
        self.assertIsNotNone(s2)
        c.close()

    def test_say_over_ws_broadcasts_chat_to_all_clients(self):
        a, b = WsClient(self.srv), WsClient(self.srv)
        a.send({"op": "say", "text": "walk"})
        for c in (a, b):
            texts = []
            while len(texts) < 2:
                o = c.recv_json("chat")
                texts.append((o["role"], o["text"]))
            self.assertEqual(texts, [("user", "walk"), ("robot", "ok: walk")])
        self.assertGreater(self.host.agent.layer.backend.status().data["speed_mps"] + len(self.host.agent.layer.backend.events), 0)
        a.close(); b.close()

    def test_estop_over_ws_and_bad_commands(self):
        c = WsClient(self.srv)
        c.send({"op": "estop"})
        self.assertEqual(c.recv_json("ack")["op"], "estop")
        self.assertEqual(self.host.estops, 1)
        c.send({"op": "fly"})
        self.assertIn("unknown op", c.recv_json("error")["error"])
        c.s.sendall(ws.encode_frame(ws.OP_TEXT, b"not json", b"abcd"))
        self.assertIn("expected", c.recv_json("error")["error"])
        c.send({"op": "say", "text": ""})
        self.assertEqual(c.recv_json("error")["error"], "bad 'text'")
        c.close()

    def test_client_limit_and_cleanup(self):
        cs = [WsClient(self.srv) for _ in range(8)]
        extra = WsClient(self.srv)
        self.assertEqual(extra.status, 503)
        extra.close()
        for c in cs:
            c.close()
        for _ in range(100):                                         # the server notices closed sockets
            if self.srv.n_clients == 0:
                break
            time.sleep(0.05)
        self.assertEqual(self.srv.n_clients, 0)


class TokenSource(unittest.TestCase):
    def test_env_token_random_fallback_and_telemetry_error(self):
        os.environ["GURDY_WEB_TOKEN"] = "from-env"
        try:
            a = GurdyWebServer(FakeHost(), port=0)
            self.assertEqual(a.token, "from-env")
            a.httpd.server_close()
        finally:
            del os.environ["GURDY_WEB_TOKEN"]
        b = GurdyWebServer(FakeHost(), port=0)
        c = GurdyWebServer(FakeHost(), port=0)
        self.assertGreaterEqual(len(b.token), 16)
        self.assertNotEqual(b.token, c.token)
        self.assertFalse(b.check_token(None) or b.check_token("") or b.check_token("ü"))

        class Broken:
            def snapshot(self): raise RuntimeError("boom")
            def map(self): return {}
        b.telemetry = Broken()
        self.assertEqual(b.state(b.default), {"connected": False, "error": "RuntimeError: boom"})
        b.httpd.server_close(); c.httpd.server_close()


if __name__ == "__main__":
    unittest.main()


class PlannedPathTest(unittest.TestCase):
    def test_snapshot_has_planned_path_while_walking_and_none_when_idle(self):
        from brain.mock.backend import MockBackend
        from web.api.telemetry import MockTelemetry

        class Host:
            lock = None
            agent = type("A", (), {"layer": type("L", (), {"backend": MockBackend()})()})()

        t = MockTelemetry(Host())
        self.assertEqual(t.snapshot()["path"], [])
        self.assertTrue(Host.agent.layer.backend.go_to(place="room B").ok)
        path = t.snapshot()["path"]
        self.assertGreaterEqual(len(path), 2)
        self.assertEqual(path[-1], [4.5, 0.0])
