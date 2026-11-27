#!/usr/bin/env python3
"""Remove tokens / secrets from logs before they are saved in the repo (overnight runs, retests).

    python3 scripts/redact_logs.py copy SRC_DIR DST_DIR      # copy a log tree, every file redacted (DST gets only redacted files)
    python3 scripts/redact_logs.py inplace DIR_OR_FILE       # redact in place
    some_command | python3 scripts/redact_logs.py filter     # stream filter (stdin -> stdout)

What is replaced by `<redacted>`:
  * the exact values of the secret environment variables (SECRET_ENV below) and of REDACT_VALUES (comma / newline separated):
    values are passed by ENVIRONMENT, never on the command line (that would show them in `ps`);
  * token-like patterns: `#token=` / `?token=` / `&token=` in URLs, `access token: X`, `Authorization: Bearer X` / `Bearer X`,
    `NAME=value` / `NAME: value` for upper-case names ending in TOKEN / SECRET / PASSWORD / API_KEY / APIKEY,
    Hugging Face (`hf_...`), OpenAI-style (`sk-...`), GitHub (`ghp_...`, `github_pat_...`) and Slack (`xox?-...`) tokens.
Files are processed byte-transparently (latin-1 round trip), so binary files are copied unchanged apart from a matched secret.
Exit status 0; the number of replacements is printed to stderr.
"""
import os
import re
import shutil
import sys
from pathlib import Path

MASK = "<redacted>"
SECRET_ENV = ("GURDY_WEB_TOKEN", "HF_TOKEN", "HUGGING_FACE_HUB_TOKEN", "GNANI_API_KEY", "TS_AUTHKEY", "GITHUB_TOKEN", "GH_TOKEN",
              "OPENAI_API_KEY", "ANTHROPIC_API_KEY")
PATTERNS = [
    (re.compile(r"([#?&]token=)[^\s&\"'<>]+"), r"\1" + MASK),
    (re.compile(r"(?i)(access token:\s*)\S+"), r"\1" + MASK),
    (re.compile(r"(?i)(\bBearer\s+)[A-Za-z0-9._~+/=-]{6,}"), r"\1" + MASK),
    (re.compile(r"(\b[A-Z0-9_]*(?:TOKEN|SECRET|PASSWORD|API_KEY|APIKEY)\b[\"']?\s*[=:]\s*[\"']?)(?!<redacted>)[^\s\"',;]+"), r"\1" + MASK),
    (re.compile(r"\bhf_[A-Za-z0-9]{20,}"), MASK),
    (re.compile(r"\bsk-[A-Za-z0-9_-]{20,}"), MASK),
    (re.compile(r"\b(?:ghp|gho|ghu|ghs)_[A-Za-z0-9]{20,}|\bgithub_pat_[A-Za-z0-9_]{20,}"), MASK),
    (re.compile(r"\bxox[abprs]-[A-Za-z0-9-]{10,}"), MASK),
]


def secret_values(env=None) -> list:
    env = os.environ if env is None else env
    vals = [env.get(k, "") for k in SECRET_ENV]
    vals += re.split(r"[,\n]", env.get("REDACT_VALUES", ""))
    return sorted({v.strip() for v in vals if len(v.strip()) >= 6}, key=len, reverse=True)   # longest first; tiny values never


def redact_text(s: str, values=None) -> tuple:
    """-> (redacted text, number of replacements)."""
    n = 0
    for v in (secret_values() if values is None else values):
        if v in s:
            n += s.count(v)
            s = s.replace(v, MASK)
    for rx, rep in PATTERNS:
        s, k = rx.subn(rep, s)
        n += k
    return s, n


def redact_file(src: Path, dst: Path, values) -> int:
    t, n = redact_text(src.read_bytes().decode("latin-1"), values)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(t.encode("latin-1"))
    shutil.copymode(src, dst)
    return n


def copy_tree(src: Path, dst: Path, values=None) -> int:
    values = secret_values() if values is None else values
    if src.is_file():
        return redact_file(src, dst, values)
    n = 0
    for f in sorted(src.rglob("*")):
        if f.is_file() and not f.is_symlink():
            n += redact_file(f, dst / f.relative_to(src), values)
    return n


def inplace(p: Path, values=None) -> int:
    values = secret_values() if values is None else values
    files = [p] if p.is_file() else [f for f in sorted(p.rglob("*")) if f.is_file() and not f.is_symlink()]
    return sum(redact_file(f, f, values) for f in files)


def main(argv):
    if len(argv) >= 1 and argv[0] == "filter":
        vals, n = secret_values(), 0
        for line in sys.stdin.buffer:
            t, k = redact_text(line.decode("latin-1"), vals)
            n += k
            sys.stdout.buffer.write(t.encode("latin-1"))
            sys.stdout.flush()
    elif len(argv) == 3 and argv[0] == "copy":
        n = copy_tree(Path(argv[1]), Path(argv[2]))
    elif len(argv) == 2 and argv[0] == "inplace":
        n = inplace(Path(argv[1]))
    else:
        print(__doc__, file=sys.stderr)
        return 2
    print(f"redact_logs: {n} replacement(s)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
