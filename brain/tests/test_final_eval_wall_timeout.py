"""scripts/p5_final_eval.sh with a STUB docker / curl (no Docker, no Gazebo, no Ollama): a hanging container hits the wall-clock limit, is killed
and retried, and the mission ends as a FAILED "wall_timeout" record that the report understands. Takes ~20 s."""
import json
import os
import shutil
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def stub(dirpath: Path, name: str, body: str) -> None:
    f = dirpath / name
    f.write_text("#!/bin/bash\n" + body)
    f.chmod(f.stat().st_mode | stat.S_IEXEC)


class WallTimeout(unittest.TestCase):
    def run_script(self, compose_body: str, attempts: int = 2):
        tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, tmp, True)
        run = ROOT / "brain" / "runs" / "final_eval" / f"_selftest_{os.getpid()}"
        self.addCleanup(shutil.rmtree, run, True)
        bindir = tmp / "bin"
        bindir.mkdir()
        stub(bindir, "docker", f'''case "$1" in
  ps) exit 0 ;;
  rm) exit 0 ;;
  run) echo ok ;;          # the address probe (scripts/lib_ollama.sh) runs a throwaway container: it "reaches" Ollama
  image) exit 0 ;;
  compose) {compose_body} ;;
esac''')
        stub(bindir, "curl", 'echo \'{"models":[{"name":"gurdy-sft-400"}]}\'')
        env = {**os.environ, "PATH": f"{bindir}:{os.environ['PATH']}", "ONLY": "nogo_refuse", "ATTEMPTS": str(attempts), "WALL_MIN": "3",
               "WALL_FACTOR": "0", "RUN_DIR": str(run), "DOC": str(tmp / "report.md")}
        r = subprocess.run(["bash", str(ROOT / "scripts" / "p5_final_eval.sh")], cwd=ROOT, env=env, capture_output=True, text=True, timeout=120)
        return r, run, tmp

    def test_hanging_container_is_killed_and_mission_recorded_as_wall_timeout(self):
        r, run, tmp = self.run_script("sleep 100")
        log = (run / "run.log").read_text()
        self.assertEqual(log.count("hit the wall limit (3s)"), 2, r.stdout + r.stderr)
        rec = json.loads((run / "missions" / "nogo_refuse.json").read_text())
        self.assertEqual(rec["status"], "wall_timeout")
        self.assertIn("wall timeout: 2 of 2 attempts exceeded 3 s wall", rec["error"])
        attempts = json.loads((run / "attempts" / "nogo_refuse.json").read_text())
        self.assertEqual(attempts["attempts"], 2)
        self.assertTrue((run / "logs" / "nogo_refuse.attempt1.stack").is_dir())      # per-attempt stack log dir exists on the host
        self.assertIn("WALL TIMEOUT", (run / "report.md").read_text())
        self.assertTrue((tmp / "report.md").exists())                                 # DOC override: the real docs/results file is not touched
        self.assertTrue((run / "status.txt").read_text().startswith("DONE"))

    def test_quick_container_is_not_affected(self):
        ok = json.dumps({"mission": "nogo_refuse", "status": "ok", "poses": [], "messages": [], "turns": []})
        # the stub "container" writes a valid result file at $OUT (passed as -e OUT=...) and exits at once
        body = 'for a in "$@"; do case "$a" in OUT=*) o="${a#OUT=}" ;; esac; done; echo \'%s\' > "$o"; exit 0' % ok
        r, run, _ = self.run_script(body)
        log = (run / "run.log").read_text()
        self.assertNotIn("wall limit", log.replace("wall limit 3s)", ""))             # only the "(limit ..., wall limit 3s)" announcement
        self.assertEqual(json.loads((run / "missions" / "nogo_refuse.json").read_text())["status"], "ok")
        self.assertIn("attempt 1 finished rc=0 result=ok", log)
        self.assertNotIn("attempt 2", log)


class Verdict(unittest.TestCase):
    def test_wall_timeout_is_a_failure_not_an_infrastructure_failure(self):
        from brain.eval.final_checks import evaluate
        v = evaluate({"mission": "nogo_refuse", "status": "wall_timeout", "poses": [], "messages": [], "turns": []})
        self.assertEqual((v["success"], v["timed_out"], v["infra_failure"]), (False, True, False))


if __name__ == "__main__":
    unittest.main()
