"""Which final-eval mission suite is active: env GURDY_FINAL_SUITE = `final_suite` (Gurdy 2.0, flat_lab, default) or `final_suite_lab4` (Gurdy 3.0 held-out, lab4)."""
import importlib
import os


def suite_module():
    return importlib.import_module("." + os.environ.get("GURDY_FINAL_SUITE", "final_suite"), __package__)
