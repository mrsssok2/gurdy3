#!/bin/bash
# Create (or re-create with --force) the host test venv for gurdy3 (no Docker). Shared by all checkouts / worktrees.
#   scripts/make_host_venv.sh [--force] [--lock]      --lock: exact versions of requirements/host-tests.lock
# Then, from any checkout:  ~/projects/.venv-gurdy3/bin/python -m pytest -q gurdy_core/tests   (see CLAUDE.md "Host tests")
set -euo pipefail
VENV=${GURDY3_VENV:-$HOME/projects/.venv-gurdy3}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
REQ=$ROOT/requirements/host-tests.txt
for a in "$@"; do
  case $a in --force) rm -rf "$VENV" ;; --lock) REQ=$ROOT/requirements/host-tests.lock ;; *) echo "unknown arg $a"; exit 2 ;; esac
done
if [ -x "$VENV/bin/python" ]; then echo "exists: $VENV (use --force to rebuild)"; exit 0; fi
python3 -m venv "$VENV"
"$VENV/bin/python" -m pip install -q --upgrade pip
"$VENV/bin/python" -m pip install -q -r "$REQ"
"$VENV/bin/python" -c "import numpy, pytest, yaml, langgraph; print('ok', numpy.__version__)"
echo "venv ready: $VENV"
