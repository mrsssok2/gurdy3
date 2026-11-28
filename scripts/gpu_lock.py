"""Python side of scripts/gpu_lock.sh (same lock file, same protocol): `acquire(owner)` blocks until THIS process holds the
shared GPU lock (~/projects/.gpu_lock); it is released when the process exits (the holder `flock -o LOCK tail --pid=PID`
watches the pid). Nested / DRY_RUN rules as in gpu_lock.sh."""
import os
import subprocess
import sys
import time
from pathlib import Path

HOLDER = ('printf "owner=%s\\npid=%s\\nstart=%s\\nhost=%s\\n" "$1" "$2" "$(date -u +%FT%TZ)" "$(hostname)" > "$3.info.tmp" && '
          'mv "$3.info.tmp" "$3.info"; tail --pid="$2" -f /dev/null; rm -f "$3.info"')


def lock_path() -> Path:
    return Path(os.environ.get("GPU_LOCK", str(Path.home() / "projects/.gpu_lock")))


def acquire(owner: str, timeout: float = 0.0) -> bool:
    if os.environ.get("DRY_RUN") == "1":
        return True
    lock, me = lock_path(), os.getpid()
    info = Path(str(lock) + ".info")
    anc = os.environ.get("GPU_LOCK_OWNER_PID")
    if anc and info.exists() and f"pid={anc}" in info.read_text().split() and Path(f"/proc/{anc}").exists():
        return True                                                    # held by the batch that started us
    lock.parent.mkdir(parents=True, exist_ok=True)
    lock.touch()
    holder = subprocess.Popen(["flock", "-o", str(lock), "sh", "-c", HOLDER, "_", owner, str(me), str(lock)],
                              stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    t0, said = time.time(), False
    while not (info.exists() and f"pid={me}" in info.read_text().split()):
        if holder.poll() is not None:
            print("gpu_lock: holder process died", file=sys.stderr)
            return False
        if timeout and time.time() - t0 >= timeout:
            holder.kill()
            print(f"gpu_lock: busy after {timeout:.0f} s: giving up", file=sys.stderr)
            return False
        if not said and time.time() - t0 > 0.2:
            print(f"gpu_lock: {owner} waiting ({info.read_text().strip() if info.exists() else 'held'})", file=sys.stderr)
            said = True
        time.sleep(0.2)
    os.environ["GPU_LOCK_OWNER_PID"] = str(me)
    print(f"gpu_lock: acquired by {owner} (pid {me}) after {time.time() - t0:.0f} s", file=sys.stderr)
    return True
