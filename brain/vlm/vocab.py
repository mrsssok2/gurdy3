"""Words <-> detector classes for scene questions and VLM answers. A VLM uses free words ("crate", "bin", "ball", "table"):
each maps to a detector class (then it can be checked) or to None (outside what the detector knows: never confirmable)."""
from __future__ import annotations

import re

CLASSES = ("box", "barrel", "fire_extinguisher", "traffic_cone", "toolbox", "mannequin", "chair",
           "small_box", "mini_toolbox", "can", "mini_cone", "mini_extinguisher")
WORDS = {
    "box": "box", "boxes": "box", "crate": "box", "crates": "box", "carton": "box", "cardboard box": "box",
    "barrel": "barrel", "barrels": "barrel", "drum": "barrel", "drums": "barrel", "oil drum": "barrel",
    "fire extinguisher": "fire_extinguisher", "extinguisher": "fire_extinguisher", "extinguishers": "fire_extinguisher",
    "traffic cone": "traffic_cone", "cone": "traffic_cone", "cones": "traffic_cone", "pylon": "traffic_cone",
    "toolbox": "toolbox", "tool box": "toolbox", "toolboxes": "toolbox",
    "mannequin": "mannequin", "dummy": "mannequin", "mannequins": "mannequin",
    "chair": "chair", "chairs": "chair", "stool": "chair",
    "cube": "small_box", "cubes": "small_box", "small box": "small_box", "block": "small_box",
    "mini toolbox": "mini_toolbox", "small toolbox": "mini_toolbox",
    "can": "can", "cans": "can", "tin": "can", "soda can": "can",
    "mini cone": "mini_cone", "small cone": "mini_cone",
    "mini extinguisher": "mini_extinguisher", "small extinguisher": "mini_extinguisher",
}
# classes a VLM is likely to confuse: a fact of the sibling class supports the word (size is hard to judge from one image)
SIBLINGS = {"small_box": "box", "box": "small_box", "mini_toolbox": "toolbox", "toolbox": "mini_toolbox", "mini_cone": "traffic_cone",
            "traffic_cone": "mini_cone", "mini_extinguisher": "fire_extinguisher", "fire_extinguisher": "mini_extinguisher"}
COLOURS = {"red", "orange", "yellow", "green", "blue", "purple", "pink", "brown", "black", "white", "gray", "grey", "silver"}
SAME_COLOUR = {"grey": "gray", "silver": "gray"}
NOT_OBJECTS = {"floor", "ground", "wall", "walls", "ceiling", "room", "light", "shadow", "shadows", "background", "image", "picture",
               "scene", "view", "camera", "nothing", "space", "area", "corner", "path", "way", "doorway", "door"}


def colour(word) -> str:
    w = (word or "").strip().lower()
    return SAME_COLOUR.get(w, w) if w in COLOURS else ""


def to_class(name: str):
    """'red crate' -> ('box', 'red'); 'green ball' -> (None, 'green') with the noun kept by the caller; longest phrase first."""
    t = re.sub(r"[^a-z ]+", " ", (name or "").lower()).split()
    col = next((colour(w) for w in t if colour(w)), "")
    s = " ".join(w for w in t if not colour(w))
    for k in sorted(WORDS, key=len, reverse=True):
        if re.search(rf"\b{re.escape(k)}\b", s):
            return WORDS[k], col
    return None, col


def noun(name: str) -> str:
    """The object word without colour / articles: 'a large green ball' -> 'ball'."""
    t = [w for w in re.sub(r"[^a-z ]+", " ", (name or "").lower()).split()
         if not colour(w) and w not in ("a", "an", "the", "some", "large", "small", "big", "little", "tall", "short")]
    return t[-1] if t else ""


def say(cls: str) -> str:
    return {"small_box": "cube", "mini_toolbox": "small toolbox", "mini_cone": "small cone", "mini_extinguisher": "small extinguisher",
            "fire_extinguisher": "fire extinguisher", "traffic_cone": "traffic cone"}.get(cls, cls)
