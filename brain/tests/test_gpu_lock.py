"""scripts/gpu_lock.sh / gpu_lock.py: one shared lock for all projects (2026-11-28: two sessions' per-try Gazebo batches
interleaved in the gaps between stacks). Every test uses its own lock file (GPU_LOCK) -- never the real ~/projects/.gpu_lock."""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SH = str(ROOT / "scripts/gpu_lock.sh")


@pytest.fixture
def env(tmp_path):
    e = {k: v for k, v in os.environ.items() if not k.startswith("GPU_LOCK")}
    e.pop("DRY_RUN", None)
    e["GPU_LOCK"] = str(tmp_path / "lock")
    return e


def bg(env, owner, cmd, log):
    return subprocess.Popen(["bash", SH, "run", owner, "--", "bash", "-c", cmd], env=env, stdout=open(log, "a"),
                            stderr=subprocess.DEVNULL, start_new_session=True)


def status(env):
    return subprocess.run(["bash", SH, "status"], env=env, capture_output=True, text=True).stdout.strip()


def wait_for(pred, t=10.0):
    t0 = time.time()
    while time.time() - t0 < t:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_second_batch_waits_for_the_first_whole_batch(env, tmp_path):
    log = tmp_path / "order.log"
    a = bg(env, "A", "echo A-start; sleep 1.5; echo A-end", log)
    assert wait_for(lambda: "held by A" in status(env))
    b = bg(env, "B", "echo B-start; echo B-end", log)
    a.wait(10); b.wait(10)
    assert log.read_text().split() == ["A-start", "A-end", "B-start", "B-end"]
    assert wait_for(lambda: status(env) == "free")


def test_status_names_owner_pid_and_start(env, tmp_path):
    a = bg(env, "gurdy3:overnight", "sleep 2", tmp_path / "x")
    assert wait_for(lambda: "held by gurdy3:overnight (pid " in status(env))
    assert "since 20" in status(env)
    a.wait(10)


def test_killed_owner_releases_the_lock(env, tmp_path):
    """kill -9 of the batch (its children keep running): the lock is free again within ~2 s, never stale."""
    a = bg(env, "A", "sleep 30", tmp_path / "x")
    assert wait_for(lambda: "held by A" in status(env))
    os.killpg(a.pid, signal.SIGKILL)
    t0 = time.time()
    r = subprocess.run(["bash", SH, "run", "B", "--", "true"], env=env, capture_output=True, text=True, timeout=20)
    assert r.returncode == 0 and time.time() - t0 < 5


def test_nested_scripts_of_one_batch_do_not_deadlock(env, tmp_path):
    inner = tmp_path / "inner.sh"
    inner.write_text(f"source {SH}; gpu_lock_acquire inner || exit 1; echo inner-ran\n")
    r = subprocess.run(["bash", SH, "run", "outer", "--", "bash", str(inner)], env=env, capture_output=True, text=True, timeout=20)
    assert r.returncode == 0 and "inner-ran" in r.stdout


def test_timeout_gives_up_while_held(env, tmp_path):
    a = bg(env, "A", "sleep 4", tmp_path / "x")
    assert wait_for(lambda: "held by A" in status(env))
    e = dict(env, GPU_LOCK_TIMEOUT="1")
    r = subprocess.run(["bash", SH, "run", "B", "--", "echo", "ran"], env=e, capture_output=True, text=True, timeout=20)
    assert r.returncode == 1 and "ran" not in r.stdout and "giving up" in r.stderr
    a.wait(10)


def test_python_and_bash_share_the_lock(env, tmp_path):
    code = (f"import sys,time; sys.path.insert(0,{str(ROOT / 'scripts')!r}); import gpu_lock; "
            "assert gpu_lock.acquire('py'); print('py-start', flush=True); time.sleep(1.5); print('py-end', flush=True)")
    log = tmp_path / "o.log"
    p = subprocess.Popen([sys.executable, "-c", code], env=env, stdout=open(log, "a"), stderr=subprocess.DEVNULL)
    assert wait_for(lambda: "held by py" in status(env))
    b = bg(env, "B", "echo B", log)
    p.wait(10); b.wait(10)
    assert log.read_text().split() == ["py-start", "py-end", "B"]


def test_stale_info_of_a_dead_pid_is_cleared(env, tmp_path):
    Path(env["GPU_LOCK"] + ".info").write_text("owner=ghost\npid=999999\nstart=x\n")
    assert status(env) == "free (cleared stale info of dead pid 999999)"
    assert not Path(env["GPU_LOCK"] + ".info").exists()


def test_dry_run_never_takes_the_lock(env, tmp_path):
    a = bg(env, "A", "sleep 3", tmp_path / "x")
    assert wait_for(lambda: "held by A" in status(env))
    t0 = time.time()
    r = subprocess.run(["bash", SH, "run", "dry", "--", "true"], env=dict(env, DRY_RUN="1"), timeout=10)
    assert r.returncode == 0 and time.time() - t0 < 1.0
    a.wait(10)


def test_every_docker_script_takes_the_lock():
    """Host scripts take it themselves; scripts that run INSIDE a container say so in their usage line (the container is
    started with `scripts/gpu_lock.sh run ...`). New scripts that start Docker without either fail here."""
    import re
    in_container_or_attach = {"arm_pose.sh"}                         # docker exec into an already running (locked) container
    missing = []
    for f in sorted((ROOT / "scripts").glob("*.sh")):
        s = f.read_text()
        if f.name.startswith("lib_") or f.name == "gpu_lock.sh" or f.name in in_container_or_attach:
            continue
        if re.search(r"docker (compose|run|exec)", s) or f.name.startswith("overnight_"):
            if "gpu_lock_acquire" not in s and "gpu_lock.sh run" not in s:
                missing.append(f.name)
    assert missing == []
