"""Object data the fetch rules need (stdlib copy of gurdy_core.arm.PICKABLE / EXISTING and vision/small_objects.py;
parity is tested in brain/tests/test_fetch.py when numpy is available)."""
from __future__ import annotations

MAX_PAYLOAD_KG = 0.08          # config/arm.yaml payload.max_kg (elbow torque limit, Phase 9a)
MAX_GRIP_WIDTH_M = 0.07        # gripper opening 0.08 - 1 cm margin
CARRY_SPEED_MPS = 0.08         # walking speed limit while carrying (Phase 9a plan section 7)

PICKABLE = {   # class: (mass kg, grip width m, words people use)
    "small_box": (0.05, 0.05, ("cube", "block", "small box", "little box", "small_box")),
    "mini_toolbox": (0.08, 0.02, ("mini toolbox", "small toolbox", "little toolbox", "mini_toolbox", "tool case")),
    "can": (0.06, 0.066, ("can", "tin", "soda can")),
    "mini_cone": (0.03, 0.045, ("mini cone", "small cone", "little cone", "mini_cone")),
    "mini_extinguisher": (0.08, 0.05, ("mini extinguisher", "small extinguisher", "little extinguisher", "mini_extinguisher")),
}
TOO_BIG = {    # class: (narrowest side m, estimated real mass kg, words)
    "box": (0.30, 1.0, ("box", "crate", "carton", "cardboard box")),
    "toolbox": (0.20, 3.0, ("toolbox",)),
    "fire_extinguisher": (0.16, 3.0, ("fire extinguisher", "extinguisher")),
    "traffic_cone": (0.36, 1.0, ("traffic cone", "cone")),
    "barrel": (0.56, 10.0, ("barrel", "drum")),
    "chair": (0.42, 4.0, ("chair",)),
}
PEOPLE_WORDS = ("person", "people", "human", "man", "woman", "child", "kid", "baby", "boy", "girl", "mannequin", "dummy",
                "me", "him", "her", "them", "you", "yourself", "somebody", "someone", "anyone", "user", "operator")
COLOURS = ("red", "blue", "green", "yellow", "orange", "purple", "pink", "silver", "black", "white", "grey", "gray")


def label(cls: str) -> str:
    return {"small_box": "cube", "mini_toolbox": "mini toolbox", "mini_cone": "mini cone",
            "mini_extinguisher": "mini extinguisher"}.get(cls, cls.replace("_", " "))
