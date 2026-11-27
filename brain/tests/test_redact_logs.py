"""scripts/redact_logs.py: no token / secret may reach a saved log (overnight_1127, fetch retests)."""
import importlib.util
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("redact_logs", ROOT / "scripts/redact_logs.py")
R = importlib.util.module_from_spec(spec)
spec.loader.exec_module(R)

TOK = "SDvFCMmHtKoacwbR8ydIT1gFO-b-ItuX"          # shape of a start_web_robot.sh token (secrets.token_urlsafe(24))

LEAKS = [   # real line formats from today's logs + common secret shapes
    f"READY. Open in the laptop browser:  http://localhost:8080/#token={TOK}",
    f"access token: {TOK}",
    f"curl -H 'Authorization: Bearer {TOK}' localhost:8080/api/state",
    f"ws://localhost:8080/ws?sid=abc&token={TOK}",
    f"GURDY_WEB_TOKEN={TOK} FETCH=true scripts/start_web_robot.sh",
    "HF_TOKEN: hf_abcdefghijklmnopqrstuvwxyz0123",
    "export GNANI_API_KEY='k3y-0123456789abcdef'",
    "push with ghp_0123456789abcdefghijABCDEFGHIJ0123",
]
SAFE = [
    "[detector_node-25] [INFO] detector det_v3_s.onnx on CPUExecutionProvider, input (256, 320)",
    "epoch 104/150  mAP50 0.962  mAP50-95 0.900",
    "prompt is 2550 tokens; max_tokens: 512; token budget ok",
    "{\"reply\": \"I picked up the blue can in room A and I'm holding it with the arm folded.\", \"seq\": 2}",
    "[bt_navigator-11] [INFO] Begin navigating from current location (-0.99, 0.36) to (0.00, 0.00)",
]


def test_every_leak_is_removed_and_safe_lines_are_untouched():
    for line in LEAKS:
        out, n = R.redact_text(line, values=[])
        assert n >= 1 and R.MASK in out, line
        assert TOK not in out and "hf_abc" not in out and "k3y-0123" not in out and "ghp_0123" not in out, out
    for line in SAFE:
        assert R.redact_text(line, values=[]) == (line, 0)


def test_exact_secret_values_from_the_environment():
    env = {"GURDY_WEB_TOKEN": TOK, "REDACT_VALUES": "my-other-secret-123,short"}
    vals = R.secret_values(env)
    assert TOK in vals and "my-other-secret-123" in vals and "short" not in vals     # < 6 chars never (would eat words)
    out, n = R.redact_text(f"some tool printed {TOK} and my-other-secret-123 bare", vals)
    assert TOK not in out and "my-other-secret-123" not in out and n == 2


def test_copy_tree_redacts_every_file_including_binary_and_nested(tmp_path):
    src, dst = tmp_path / "raw", tmp_path / "saved"
    (src / "chat").mkdir(parents=True)
    (src / "stack.out").write_text(LEAKS[0] + "\n" + SAFE[1] + "\n")
    (src / "chat" / "launch.log").write_text("ok\n" + LEAKS[1] + "\n")
    (src / "chat" / "blob.bin").write_bytes(b"\x00\xff\x10" + TOK.encode() + b"\x00\x01")
    n = R.copy_tree(src, dst, values=[TOK])
    assert n >= 3
    for f in dst.rglob("*"):
        if f.is_file():
            assert TOK.encode() not in f.read_bytes(), f
    assert (dst / "stack.out").read_text().splitlines()[1] == SAFE[1]
    assert (dst / "chat" / "blob.bin").read_bytes().startswith(b"\x00\xff\x10")


def test_filter_mode_and_value_only_from_env(tmp_path):
    env = {**os.environ, "GURDY_WEB_TOKEN": TOK}
    p = subprocess.run([sys.executable, str(ROOT / "scripts/redact_logs.py"), "filter"], input=f"x {TOK} y\nplain\n",
                       capture_output=True, text=True, env=env)
    assert p.returncode == 0 and TOK not in p.stdout and p.stdout.endswith("plain\n") and "1 replacement" in p.stderr
