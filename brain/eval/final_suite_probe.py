"""Stop-latency probe (NOT a held-out suite): one mission in lab4 that walks and stops 8 times, at different gait phases, so one stack gives 8 measured stops.
  SUITE=final_suite_probe RUN_DIR=brain/runs/stop_probe/<name> scripts/p5_final_eval.sh
Each stop: wall seconds (and sim seconds) from the word 'stop' until the measured speed is < 0.02 m/s (3 consecutive samples), as in the suites."""
from __future__ import annotations

from .final_suite_lab4 import WORLD, MAP_ID, MAP_NAME, PLACES_FILE, PLACES, resolve_region, mission  # noqa: F401

_STEPS = []
for i, (dest, wait) in enumerate([("room D", 9), ("room C", 11), ("room B", 8), ("room D", 12), ("room C", 9.5), ("room B", 10.5), ("room D", 8.5), ("room C", 11.5)]):
    _STEPS += [("say", f"go to {dest}"), ("wait", wait), ("stop", ["stop", "stop right there", "hold it", "freeze"][i % 4]), ("wait", 4)]
SUITE = [mission("stop_probe", "8 walk-and-stop cycles", "stop", 500, _STEPS, [("stop_latency", 1.0)])]
BY_ID = {m["id"]: m for m in SUITE}


def check_ids() -> list[str]:
    return [m["id"] for m in SUITE]
