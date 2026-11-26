"""Public demo: every browser session gets its own world, agent and chat (nothing shared), freed when idle, capped when full."""
import http.client
import json
import threading
import time
import unittest

from web.api.demo import build_host
from web.api.server import GurdyWebServer
from web.tests.test_api_server import Clock, WsClient, request


def sid(n: int) -> str:
    return f"{n:032x}"


class Demo:
    def __init__(self, **kw):
        self.clock = Clock()
        self.built = []

        def factory():
            h = build_host()
            self.built.append(h)
            return h
        args = dict(port=0, state_s=0.05, demo=True, host_factory=factory, clock=self.clock, trust_proxy=True)
        args.update(kw)
        self.srv = GurdyWebServer(None, **args).start()

    def req(self, method, path, n, body=None, ip="1.1.1.1"):
        return request(self.srv, method, path, body, token=None, headers={"X-Session": sid(n), "X-Forwarded-For": ip})

    def say(self, n, text, ip="1.1.1.1"):
        return self.req("POST", "/api/say", n, {"text": text}, ip)

    def ws(self, n, ip="1.1.1.1"):
        return WsClient(self.srv, path=f"/ws?sid={sid(n)}")


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.d = Demo()

    def tearDown(self):
        self.d.srv.close()

    def test_two_sessions_do_not_see_each_other(self):
        d = self.d
        self.assertEqual(d.say(1, "go to room B", ip="1.1.1.1")[0], 200)
        d.say(1, "yes", ip="1.1.1.1")
        a_msgs = d.req("GET", "/api/messages?since=0", 1)[1]["messages"]
        b_msgs = d.req("GET", "/api/messages?since=0", 2, ip="2.2.2.2")[1]["messages"]
        self.assertGreaterEqual(len(a_msgs), 4)
        self.assertEqual(b_msgs, [])                                           # B sees nothing of A's conversation
        d.say(2, "where are you?", ip="2.2.2.2")
        a_texts = [m["text"] for m in d.req("GET", "/api/messages?since=0", 1)[1]["messages"]]
        self.assertNotIn("where are you?", a_texts)                            # and A sees nothing of B's
        self.assertEqual(len(d.built), 2)
        self.assertIsNot(d.built[0].agent.layer.backend, d.built[1].agent.layer.backend)

    def test_robots_are_independent_worlds(self):
        d = self.d
        d.say(1, "go to room B", ip="1.1.1.1"); d.say(1, "yes", ip="1.1.1.1")
        d.built[0].agent.layer.backend.advance(60)                              # A's robot walks (virtual clock)
        a = d.req("GET", "/api/state", 1)[1]
        b = d.req("GET", "/api/state", 2, ip="2.2.2.2")[1]
        self.assertGreater(a["pose"]["x"], 1.0)
        self.assertEqual((b["pose"]["x"], b["mode"]), (0.0, "idle"))           # B's robot never moved
        self.assertEqual(b["trail"], [[0.0, 0.0]])

    def test_websocket_chat_and_state_stay_in_their_session(self):
        d = self.d
        a, b = d.ws(1), d.ws(2)
        self.assertEqual((a.status, b.status), (101, 101))
        a.recv_json("hello"); b.recv_json("hello")
        a.send({"op": "say", "text": "where are you?"})
        self.assertEqual(a.recv_json("chat")["role"], "user")
        got = []
        b.s.settimeout(1.0)
        try:
            for _ in range(8):
                op, data = b.recv()
                o = json.loads(data)
                if o.get("type") == "chat" and o.get("role") != "robot":          # B's own robot may announce what it sees at the start (lab4: a cone)
                    got.append(o)
        except (OSError, TimeoutError):
            pass
        self.assertEqual(got, [], "B received a chat message of A")
        a.close(); b.close()

    def test_new_visitor_hello_has_an_empty_chat(self):
        d = self.d
        d.say(1, "hello there")
        c = d.ws(2)
        self.assertEqual(c.recv_json("hello")["chat"], [])
        c.close()

    def test_session_id_is_required_and_validated(self):
        d = self.d
        self.assertEqual(request(d.srv, "GET", "/api/state", token=None)[0], 400)
        for bad in ("x", "z" * 32, "A" * 32, "1" * 31, "1" * 33, "../" * 8):
            self.assertEqual(request(d.srv, "GET", "/api/state", token=None, headers={"X-Session": bad})[0], 400, bad)
        self.assertEqual(request(d.srv, "GET", "/api/health", token=None)[0], 200)      # no session needed
        self.assertEqual(WsClient(d.srv, path="/ws").status, 400)
        self.assertEqual(len(d.built), 0)                                                # nothing was created for bad ids

    def test_static_page_needs_no_session(self):
        c = http.client.HTTPConnection("127.0.0.1", self.d.srv.port)
        c.request("GET", "/")
        self.assertEqual(c.getresponse().status, 200)
        self.assertEqual(len(self.d.built), 0)

    def test_rate_limits_are_per_ip_not_per_session(self):
        d = self.d
        codes = [d.say(i % 3 + 1, "hi", ip="5.5.5.5")[0] for i in range(7)]          # three session ids, ONE address
        self.assertEqual(codes, [200] * 5 + [429] * 2)
        self.assertEqual(d.say(9, "hi", ip="6.6.6.6")[0], 200)                         # another address has its own bucket

    def test_message_cap_and_no_voice_in_the_demo(self):
        d = self.d
        self.assertEqual(d.say(1, "x" * 201)[0], 400)
        self.assertEqual(d.req("POST", "/api/voice", 1, b"abc")[0], 503)
        self.assertFalse(request(d.srv, "GET", "/api/health", token=None)[1]["voice"])


class LifecycleTests(unittest.TestCase):
    def test_cap_busy_message_and_slot_reuse_after_idle(self):
        d = Demo(max_sessions=2, session_idle_s=600)
        try:
            self.assertEqual(d.req("GET", "/api/state", 1, ip="1.1.1.1")[0], 200)
            self.assertEqual(d.req("GET", "/api/state", 2, ip="2.2.2.2")[0], 200)
            conn = http.client.HTTPConnection("127.0.0.1", d.srv.port)
            conn.request("GET", "/api/state", headers={"X-Session": sid(3), "X-Forwarded-For": "3.3.3.3"})
            r = conn.getresponse()
            body = json.loads(r.read())
            self.assertEqual((r.status, body["busy"]), (503, True))
            self.assertIn("busy", body["error"])
            self.assertEqual(r.getheader("Retry-After"), "60")
            self.assertEqual(WsClient(d.srv, path=f"/ws?sid={sid(3)}").status, 503)
            self.assertEqual(d.req("GET", "/api/state", 1, ip="1.1.1.1")[0], 200)        # members are unaffected
            d.clock.t += 601
            self.assertEqual(d.srv.reap(), 2)                                              # both idle: slots are free again
            self.assertEqual(d.req("GET", "/api/state", 3, ip="3.3.3.3")[0], 200)
        finally:
            d.srv.close()

    def test_idle_sessions_are_freed_and_their_clients_told(self):
        d = Demo(max_sessions=3, session_idle_s=600)
        try:
            d.say(1, "hello", ip="1.1.1.1")
            d.say(2, "hello", ip="2.2.2.2")
            c = d.ws(2)
            c.recv_json("hello")
            d.clock.t += 400
            d.req("GET", "/api/state", 1, ip="1.1.1.1")                                    # session 1 stays alive
            d.clock.t += 400                                                               # session 2 idle for 800 s, session 1 for 400 s
            self.assertEqual(d.srv.reap(), 1)
            self.assertEqual(d.srv.n_sessions, 1)
            self.assertEqual(c.recv_json("session_expired")["type"], "session_expired")
            self.assertTrue(d.built[1]._stop.is_set())                                     # the freed session's pump thread was stopped
            self.assertFalse(d.built[0]._stop.is_set())
            self.assertEqual(d.req("GET", "/api/state", 3, ip="3.3.3.3")[0], 200)          # the freed slot is usable again
            fresh = d.req("GET", "/api/messages?since=0", 2, ip="2.2.2.2")[1]["messages"]
            self.assertEqual(fresh, [])                                                    # the old id starts a brand-new session, no old chat
            c.close()
        finally:
            d.srv.close()

    def test_websocket_commands_count_as_activity(self):
        d = Demo(session_idle_s=600)
        try:
            c = d.ws(1)
            c.recv_json("hello")
            d.clock.t += 500
            c.send({"op": "say", "text": "hi"})
            c.recv_json("ack")
            d.clock.t += 500
            self.assertEqual(d.srv.reap(), 0)                                              # 500 s since the command, not 1000
            d.clock.t += 200
            self.assertEqual(d.srv.reap(), 1)
            c.close()
        finally:
            d.srv.close()

    def test_one_address_cannot_fill_the_slots(self):
        d = Demo(max_sessions=30)
        try:
            codes = [d.req("GET", "/api/state", n, ip="8.8.8.8")[0] for n in range(1, 9)]
            self.assertEqual(codes[:4], [200] * 4)                                         # creation burst: 4, then one per 30 s
            self.assertEqual(set(codes[4:]), {429})
            d.clock.t += 31
            self.assertEqual(d.req("GET", "/api/state", 20, ip="8.8.8.8")[0], 200)
            self.assertEqual(d.req("GET", "/api/state", 1, ip="8.8.8.8")[0], 200)          # known sessions keep working
            self.assertEqual(d.srv.n_sessions, 5)
            for n in range(5):                                                             # hard cap of 6 sessions per address
                d.clock.t += 31
                d.req("GET", "/api/state", 100 + n, ip="8.8.8.8")
            self.assertLessEqual(d.srv.n_sessions, 6)
        finally:
            d.srv.close()

    def test_freeing_sessions_releases_threads(self):
        d = Demo(max_sessions=10, session_idle_s=10)
        try:
            base = threading.active_count()
            for n in range(1, 5):
                d.req("GET", "/api/state", n, ip=f"9.9.9.{n}")
            self.assertGreater(threading.active_count(), base)
            d.clock.t += 20
            self.assertEqual(d.srv.reap(), 4)
            for _ in range(60):
                if threading.active_count() <= base:
                    break
                time.sleep(0.1)
            self.assertLessEqual(threading.active_count(), base)
        finally:
            d.srv.close()

    def test_host_factory_is_demo_only_and_mock_only(self):
        with self.assertRaises(ValueError):
            GurdyWebServer(None, port=0, host_factory=build_host)                          # not demo
        class Bad:
            class agent:
                class layer:
                    backend = object()
            def on_message(self, fn): pass
        d = Demo(host_factory=lambda: Bad())
        try:
            with self.assertRaises(ValueError):
                d.srv.session_for(sid(1), "1.1.1.1")                                       # a non-mock host never gets a session
        finally:
            d.srv.close()


if __name__ == "__main__":
    unittest.main()
