"""Object classes of the inspection-robot detector (Phase 4): ids, names, English aliases, size priors."""
from __future__ import annotations

import re

CLASSES = ("box", "barrel", "fire_extinguisher", "traffic_cone", "toolbox", "mannequin", "chair")
CLASS_ID = {n: i for i, n in enumerate(CLASSES)}
# Gazebo label-system id = class id + 1 (label 0 is "background / unlabelled" in the boundingbox camera).
GZ_LABEL_OFFSET = 1

# find_object("red box") style lookup: English words -> class
ALIASES = {
    "box": ("box", "boxes", "carton", "cardboard", "crate"),
    "barrel": ("barrel", "drum"),
    "fire_extinguisher": ("extinguisher", "fire"),
    "traffic_cone": ("cone", "traffic"),
    "toolbox": ("toolbox", "tool"),
    "mannequin": ("mannequin", "person", "human", "dummy"),
    "chair": ("chair",),
}
# approximate real size (w, d, h) metres, the radius used to push a front-surface depth point to the object centre
SIZE_PRIOR = {
    "box": (0.40, 0.40, 0.35), "barrel": (0.55, 0.55, 0.90), "fire_extinguisher": (0.16, 0.16, 0.50),
    "traffic_cone": (0.30, 0.30, 0.70), "toolbox": (0.45, 0.20, 0.20), "mannequin": (0.45, 0.25, 1.70),
    "chair": (0.45, 0.45, 0.90),
}
COLORS = {  # RGB 0..1 for RViz markers
    "box": (0.65, 0.45, 0.20), "barrel": (0.10, 0.30, 0.85), "fire_extinguisher": (0.90, 0.10, 0.10),
    "traffic_cone": (1.00, 0.45, 0.00), "toolbox": (0.95, 0.80, 0.10), "mannequin": (0.80, 0.30, 0.80),
    "chair": (0.20, 0.70, 0.45),
}


def radius_prior(name: str) -> float:
    """Half of the smaller horizontal extent: how far behind the visible surface the object centre is."""
    w, d, _ = SIZE_PRIOR[name]
    return 0.5 * min(w, d)


def resolve_class(text: str) -> str | None:
    """'find the red box' -> 'box'; 'fire extinguisher' -> 'fire_extinguisher'; unknown -> None."""
    words = re.findall(r"[a-z]+", text.lower())
    hits = [c for c, al in ALIASES.items() if any(w in al for w in words)]
    return hits[0] if len(hits) == 1 else None


# ---------------------------------------------------------------- Phase 9b: small pickable objects (NOT in CLASSES yet)
# The deployed detector (det_v2_s) has the 7 classes above. The v3 detector will have CLASSES_V3: the old ids 0-6 are
# unchanged, the small classes are appended (7-11), so old labels / datasets stay valid. Switch the runtime to
# CLASSES_V3 / SMALL_ALIASES only together with a v3 model (the ONNX output width = 4 + number of classes).
SMALL_CLASSES = ("small_box", "mini_toolbox", "can", "mini_cone", "mini_extinguisher")
CLASSES_V3 = CLASSES + SMALL_CLASSES
CLASS_ID_V3 = {n: i for i, n in enumerate(CLASSES_V3)}
SMALL_ALIASES = {
    "small_box": ("cube", "block", "small box", "little box"),
    "mini_toolbox": ("small toolbox", "mini toolbox", "little toolbox", "tool case"),
    "can": ("can", "tin", "soda"),
    "mini_cone": ("small cone", "mini cone", "little cone"),
    "mini_extinguisher": ("small extinguisher", "mini extinguisher", "little extinguisher"),
}
SIZE_PRIOR_SMALL = {   # (w, d, h) m, = vision/small_objects.py CATALOG (tested)
    "small_box": (0.05, 0.05, 0.05), "mini_toolbox": (0.12, 0.05, 0.095), "can": (0.066, 0.066, 0.12),
    "mini_cone": (0.08, 0.08, 0.10), "mini_extinguisher": (0.05, 0.05, 0.15),
}
