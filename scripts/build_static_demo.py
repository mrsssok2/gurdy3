#!/usr/bin/env python3
"""Build the STATIC public demo: the dashboard (web/app) + a Web Worker that runs the real Python simulation in Pyodide. No server needed, so it can
live on GitHub Pages or a static Hugging Face Space.

  python3 scripts/build_static_demo.py [--out build/static_demo] [--pyodide-base URL]

Output folder: index.html (+ the dashboard's css / js / fonts / icons), static_backend.js (answers the dashboard's /api and /ws calls locally),
remote_connect.js ("Connect to my robot"), browser_voice.js + whisper_worker.js (hold-to-talk with Whisper in the browser, loaded lazily), demo_worker.js, demo_config.js (where Pyodide is loaded from), gurdy_demo.zip (the Python code: agent, mock flat_lab world, keyword planner, telemetry,
a small `langgraph` stand-in because the real one cannot be installed in Pyodide), .nojekyll. The Pyodide runtime itself (~6 MB) comes from the jsDelivr CDN.
"""
from __future__ import annotations

import argparse
import shutil
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PYODIDE_VERSION = "0.27.7"                      # Python 3.12.7, same minor version as the development images
PYODIDE_BASE = f"https://cdn.jsdelivr.net/pyodide/v{PYODIDE_VERSION}/full/"

PY_FILES = [
    "brain/__init__.py", "brain/eval/__init__.py", "brain/eval/keyword_parser.py", "brain/data_gen/__init__.py", "brain/data_gen/lexicon.py",
    "brain/voice/__init__.py", "brain/voice/host.py", "brain/voice/vocab.py", "brain/voice/transcript_filter.py",
    "gurdy_core/gurdy_core/__init__.py", "gurdy_core/gurdy_core/vision/__init__.py", "gurdy_core/gurdy_core/vision/classes.py",
    "web/__init__.py", "web/api/demo.py", "web/api/telemetry.py", "web/api/incidents_api.py", "web/api/sample_history.py", "web/static_demo/__init__.py", "web/static_demo/session.py",
    "scripts/gen_world_flat_lab.py", "maps/flat_lab.places.yaml",
]
PY_DIRS = {"brain/incidents": ("*.py",), "brain/agent": ("*.py", "*.yaml"), "brain/mock": ("*.py",), "brain/tools": ("*.py",)}
PY_SKIP = {"ollama_llm.py", "cli.py"}            # the LLM client and the terminal chat are not part of the keyword-planner demo
WEB_API_INIT = '"""Trimmed for the in-browser demo: only the telemetry (the HTTP server is not part of it)."""\nfrom .telemetry import MockTelemetry, Telemetry\n\n__all__ = ["MockTelemetry", "Telemetry"]\n'
SCRIPTS = ("remote_connect.js", "browser_voice.js", "static_backend.js")      # classic scripts injected before app.js
APP_SKIP = {"sw.js"}                              # no service worker in the static demo (it would cache a shell that lacks the worker files)


def build_zip(path: Path) -> int:
    n = 0
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        def add(src: Path, arc: str) -> None:
            nonlocal n
            z.write(src, arc)
            n += 1
        for f in PY_FILES:
            add(ROOT / f, f)
        for d, pats in PY_DIRS.items():
            for pat in pats:
                for src in sorted((ROOT / d).glob(pat)):
                    if src.name not in PY_SKIP:
                        add(src, str(src.relative_to(ROOT)))
        for src in sorted((ROOT / "web/static_demo/pylib").rglob("*.py")):
            add(src, str(src.relative_to(ROOT / "web/static_demo")))          # -> pylib/langgraph/...
        z.writestr("web/api/__init__.py", WEB_API_INIT)
        n += 1
    return n


def build(out: Path, pyodide_base: str = PYODIDE_BASE) -> Path:
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    for src in (ROOT / "web/app").iterdir():
        if src.name in APP_SKIP:
            continue
        if src.is_dir():
            shutil.copytree(src, out / src.name)
        else:
            shutil.copy2(src, out / src.name)
    html = (out / "index.html").read_text()
    marker = '<script type="module" src="app.js"></script>'
    if marker not in html:
        sys.exit("web/app/index.html changed: the app.js script tag was not found")
    scripts = "".join(f'<script src="{f}"></script>\n' for f in SCRIPTS)                  # order matters: mode decision, voice, transport, then the app
    (out / "index.html").write_text(html.replace(marker, scripts + marker, 1))
    for f in (*SCRIPTS, "demo_worker.js", "whisper_worker.js"):
        shutil.copy2(ROOT / "web/static_demo" / f, out / f)
    (out / "demo_config.js").write_text(f'self.PYODIDE_BASE = "{pyodide_base}";\n')
    build_zip(out / "gurdy_demo.zip")
    (out / ".nojekyll").write_text("")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(ROOT / "build" / "static_demo"))
    ap.add_argument("--pyodide-base", default=PYODIDE_BASE, help="where the Pyodide runtime is loaded from (default: jsDelivr, pinned version)")
    a = ap.parse_args()
    out = build(Path(a.out), a.pyodide_base)
    files = [p for p in out.rglob("*") if p.is_file()]
    total = sum(p.stat().st_size for p in files)
    print(f"built {len(files)} files, {total / 1024:.0f} KiB in {out}  (python zip: {(out / 'gurdy_demo.zip').stat().st_size / 1024:.0f} KiB)")


if __name__ == "__main__":
    main()
