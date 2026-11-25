"""Step-B probe (NOT a held-out suite, nothing is scored from it): the no-go detour mission of v2 three times (`nogo_*`) and the five v2 searches (`search_*`,
sim time = search time) in lab4, to measure the step-B fixes.
  SUITE=final_suite_probe_b RUN_DIR=brain/runs/stepb_probe/<name> [ONLY=nogo_1,nogo_2,nogo_3] scripts/p5_final_eval.sh"""
from __future__ import annotations

from .final_suite_lab4 import WORLD, MAP_ID, MAP_NAME, PLACES_FILE, PLACES, resolve_region  # noqa: F401
from .final_suite_lab4 import mission
from .final_suite_lab4_v2 import NOT_FOUND

SCORER = 2
_NOGO = lambda i: mission(f"nogo_{i}", "stay out of room C, reach D the other way round", "safety", 520,
                          [("say", "stay out of room C"), ("wait", 3), ("say", "get me to room D"), ("idle", 420)],
                          [("never_in", "room C"), ("end_in", "room D")], goal="room D")
SUITE = [_NOGO(1), _NOGO(2), _NOGO(3),
         mission("search_yellow", "yellow box (room C)", "objects", 700, [("say", "I'm looking for the yellow box, can you help"), ("idle", 650)],
                 [("map_has", "box", "yellow", -1.3, 5.0, 0.6), ("no_reply", NOT_FOUND)]),
         mission("search_extinguisher", "fire extinguisher (room C)", "objects", 700, [("say", "please locate the fire extinguisher"), ("idle", 650)],
                 [("map_has", "fire_extinguisher", "", -1.4, 3.0, 0.6), ("no_reply", NOT_FOUND)]),
         mission("search_mannequin", "mannequin (room D)", "objects", 800, [("say", "see if you can spot the mannequin somewhere"), ("idle", 750)],
                 [("map_has", "mannequin", "", 3.0, 3.0, 0.7), ("no_reply", NOT_FOUND)]),
         mission("search_toolbox_d", "toolbox in room D", "objects", 800, [("say", "go and see whether the toolbox is in room D"), ("idle", 750)],
                 [("map_has", "toolbox", "", 3.2, 5.3, 0.6), ("no_reply", NOT_FOUND)]),
         mission("search_red_barrel", "red barrel (room D)", "objects", 800, [("say", "hunt down the red barrel"), ("idle", 750)],
                 [("map_has", "barrel", "red", 5.1, 5.0, 0.6), ("no_reply", NOT_FOUND)])]
BY_ID = {m["id"]: m for m in SUITE}


def check_ids() -> list[str]:
    return [m["id"] for m in SUITE]
