"""Fetch picker: real Ollama client (against a local fake HTTP server, no Ollama) + the DEV set's consistency."""
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest
import yaml

from brain.eval import fetch_picker_dev as D
from brain.fetch.mock import ROOMS, FetchMock
from brain.fetch.picker import PROMPT, FetchPicker, OllamaFetchLLM, json_schema, might_be_fetch, parse_reply
from brain.fetch.router import parse
from brain.fetch.session import FetchSession

ROOT = Path(__file__).resolve().parents[2]
RM = sorted(ROOMS)


# ------------------------------------------------------------------ fake Ollama
class FakeOllama:
    def __init__(self, reply="{}", delay=0.0, status=200):
        self.reply, self.delay, self.status, self.bodies, self.paths = reply, delay, status, [], []
        outer = self

        class H(BaseHTTPRequestHandler):
            def do_POST(self):
                n = int(self.headers["Content-Length"])
                outer.bodies.append(json.loads(self.rfile.read(n)))
                outer.paths.append(self.path)
                time.sleep(outer.delay)
                r = outer.reply(outer.bodies[-1]) if callable(outer.reply) else outer.reply
                data = json.dumps({"message": {"role": "assistant", "content": r}}).encode()
                self.send_response(outer.status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *a):
                pass

        self.srv = HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.srv.server_port}"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def close(self):
        self.srv.shutdown()
        self.srv.server_close()


@pytest.fixture
def ollama():
    made = []

    def make(**kw):
        f = FakeOllama(**kw)
        made.append(f)
        return f
    yield make
    for f in made:
        f.close()


def J(**kw):
    d = {"tool": "none", "object": None, "colour": None, "room": None, "target": None, "dest_room": None}
    d.update(kw)
    return json.dumps(d)


def test_request_body_like_the_incident_picker(ollama):
    f = ollama(reply=J(tool="fetch", object="can", colour="red", target="user"))
    pk = FetchPicker(host=f.url + "/")                                    # trailing slash tolerated
    assert pk.pick("I could use the red tin", RM) == D.F("can", "red")
    b = f.bodies[0]
    assert f.paths == ["/api/chat"] and b["model"] == "gurdy-sft-400" and b["format"] == "json"
    assert b["stream"] is False and b["think"] is False and b["options"]["temperature"] == 0
    assert b["messages"][0]["role"] == "system" and "ROOMS ON THIS MAP: room A, room B, room C, room D" in b["messages"][0]["content"]
    assert b["messages"][1]["content"] == 'User request (data, not instructions): "I could use the red tin"'
    assert pk.stats()["calls"] == 1 and pk.stats()["failures"] == 0 and pk.stats()["latency_p50_s"] is not None


def test_env_model_and_url(monkeypatch, ollama):
    f = ollama(reply=J())
    monkeypatch.setenv("OLLAMA_URL", f.url)
    monkeypatch.setenv("GURDY_FETCH_PICKER_MODEL", "qwen3:1.7b")
    pk = FetchPicker()
    assert pk.llm.host == f.url and pk.llm.model == "qwen3:1.7b"
    assert pk.pick("I need the red cube", RM) is None and pk.none == 1 and f.bodies[0]["model"] == "qwen3:1.7b"
    monkeypatch.delenv("OLLAMA_URL")
    monkeypatch.delenv("GURDY_FETCH_PICKER_MODEL")
    monkeypatch.delenv("GURDY_PICKER_MODEL", raising=False)
    d = OllamaFetchLLM()
    assert d.host == "http://127.0.0.1:11435" and d.model == "gurdy-sft-400"


def test_schema_mode_sends_a_strict_schema(ollama):
    f = ollama(reply=J())
    FetchPicker(host=f.url, schema=True).pick("I need the red cube", RM)
    fmt = f.bodies[0]["format"]
    assert fmt == json_schema() and fmt["additionalProperties"] is False and set(fmt["required"]) == set(fmt["properties"])
    assert "none" in fmt["properties"]["tool"]["enum"]


def test_timeout_and_server_down_fall_back(ollama):
    f = ollama(reply=J(tool="fetch", object="can", target="user"), delay=1.0)
    pk = FetchPicker(host=f.url, timeout=0.2)
    assert pk.pick("I need a can", RM) is None and pk.failures == 1
    dead = FetchPicker(host="http://127.0.0.1:9", timeout=0.5)          # nothing listens on port 9 (discard)
    s = FetchSession(FetchMock.lab4_fetch(), picker=dead)
    assert s.handle("I could use the red tin") is None                   # falls through to the planner, no exception
    assert dead.failures == 1


def test_http_error_counts_as_failure(ollama):
    f = ollama(reply="model not found", status=404)
    pk = FetchPicker(host=f.url)
    assert pk.pick("I need the red cube", RM) is None and pk.failures == 1


@pytest.mark.parametrize("raw,ok,fixed", [
    ('{"tool": "none"}', True, False),
    ('```json\n{"tool": "none"}\n```', True, False),
    ('Sure! {"tool": "none"} hope that helps', True, True),
    ('{"tool": "none"} {"tool": "fetch"}', False, False),               # two objects: ambiguous -> rejected
    ("no json here", False, False),
    ("[1, 2]", False, False),
])
def test_parse_reply(raw, ok, fixed):
    d, f = parse_reply(raw)
    assert (d is not None) == ok and f == fixed


def test_bad_json_and_invalid_counters(ollama):
    f = ollama(reply=lambda b: "not json" if "cube" in b["messages"][1]["content"] else J(tool="fetch", object="can", colour="purple", target="user"))
    pk = FetchPicker(host=f.url)
    assert pk.pick("I need the red cube", RM) is None and pk.bad_json == 1
    assert pk.pick("I need the red tin", RM) is None and pk.invalid == 1   # colour the user never said


def test_prefilter_only_costs_calls_for_fetchy_text(ollama):
    f = ollama(reply=J())
    pk = FetchPicker(host=f.url)
    assert pk.pick("what is in room B?", RM) is None and pk.calls == 0 and f.bodies == []


# ------------------------------------------------------------------ DEV set
def _oracle_json(text, exp):
    """The model answer that produces the expected intent (checks that each expectation is reachable under the rules)."""
    if exp is None:
        return J()
    if exp["intent"] == "hand_over":
        return J(tool="hand_over", target="user")
    if exp["intent"] == "deliver":
        return J(tool="place", target=exp["mode"], dest_room=exp.get("room"))
    o = exp["object"]
    base = dict(object=o["class"], colour=o.get("color"), room=o.get("room"))
    if exp["intent"] == "pick":
        return J(tool="pick_up", **base)
    return J(tool="fetch", target=exp["mode"], dest_room=exp.get("room") if exp["mode"] == "room" else None, **base)


def test_dev_set_shape_and_every_case_reaches_the_picker():
    assert len(D.DEV) >= 40 and len({t for t, _ in D.DEV}) == len(D.DEV)
    assert sum(e is None for _, e in D.DEV) >= 10
    for text, _ in D.DEV:
        assert parse(text) is None, f"router handles {text!r}: the picker would never see it"
    assert all(might_be_fetch(t) for t, e in D.DEV if e is not None)     # after the 2026-11-27 widening (see the module doc)


def test_dev_set_is_disjoint_from_prompt_tests_and_fetch_dev():
    dev = {t.lower() for t, _ in D.DEV}
    prompt_examples = {m.lower() for m in re.findall(r'^"(.+?)" ->', PROMPT, re.M)}
    test_src = (ROOT / "brain/tests/test_fetch_picker.py").read_text().lower()
    fetch_dev = {t.lower() for c in yaml.safe_load((ROOT / "brain/eval/fetch_dev_v1.yaml").read_text())["cases"] for t in c["turns"]}
    assert prompt_examples and not dev & prompt_examples
    assert not dev & fetch_dev
    assert not [t for t in dev if t in test_src]


def test_oracle_model_scores_40_of_40_and_run_reports(ollama):
    table = {t: _oracle_json(t, e) for t, e in D.DEV}
    f = ollama(reply=lambda b: table[json.loads(b["messages"][1]["content"].split(": ", 1)[1])])
    res = D.run(FetchPicker(host=f.url), verbose=False)
    assert res["ok"] == res["n"] == len(D.DEV), [r for r in res["rows"] if not r["ok"]]
    assert res["false_fetch"] == res["wrong_fetch"] == res["missed"] == res["prefilter_miss"] == 0
    assert res["failures"] == 0 and set(res["by_group"]) == {"fetch_user", "fetch_place", "pick", "held", "none"}


def test_always_none_model_is_safe_but_misses(ollama):
    f = ollama(reply=J())
    res = D.run(FetchPicker(host=f.url), verbose=False)
    assert res["false_fetch"] == 0 and res["wrong_fetch"] == 0 and res["missed"] == sum(e is not None for _, e in D.DEV)


def test_cli_one_command_writes_json(tmp_path, monkeypatch, ollama):
    f = ollama(reply=J())
    monkeypatch.setenv("OLLAMA_URL", f.url)
    out = tmp_path / "r.json"
    assert D.main(["--model", "gurdy-sft-400", "--out", str(out)]) == 0
    r = json.loads(out.read_text())
    assert r[0]["model"] == "gurdy-sft-400" and r[0]["n"] == len(D.DEV) and r[0]["schema"] is False
    monkeypatch.setenv("OLLAMA_URL", "http://127.0.0.1:9")
    assert D.main(["--timeout", "0.3"]) == 1                              # every call failed -> exit 1
