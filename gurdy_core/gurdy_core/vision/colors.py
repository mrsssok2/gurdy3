"""Colour naming for a detected object crop (for 'red box'). English names."""
from __future__ import annotations

import colorsys

import numpy as np

NAMES = ("red", "orange", "yellow", "green", "blue", "purple", "pink", "brown", "black", "white", "gray")


SAT_MIN, VAL_MIN, SAT_FRAC_MIN = 0.30, 0.20, 0.15      # a pixel is "coloured" above these; colour named if >= 15 % are


def _hue_name(h_deg: float, v: float) -> str:
    if h_deg < 15 or h_deg >= 340:
        return "red"
    if h_deg < 40:
        return "brown" if v < 0.6 else "orange"
    if h_deg < 70:
        return "yellow"
    if h_deg < 170:
        return "green"
    if h_deg < 260:
        return "blue"
    return "purple" if h_deg < 310 else "pink"


def color_name_v1(rgb_crop: np.ndarray) -> str:
    """Phase 4 rule (until 2026-11-28): median RGB of the crop -> HSV. Names a thin object's crop after its background /
    white band (mini cones: white 85 %, gray 12 % on synth_v3_body test). Kept for comparison (vision/color_eval.py)."""
    px = np.asarray(rgb_crop, np.float64).reshape(-1, 3)
    if not len(px):
        return "gray"
    r, g, b = (np.median(px, 0) / 255.0).tolist()
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    if v < 0.18:
        return "black"
    if s < 0.18:
        return "white" if v > 0.75 else "gray"
    return _hue_name(h * 360, v)


def _hsv(px: np.ndarray):
    """Vectorised RGB (0-1) -> H (deg), S, V."""
    mx, mn = px.max(1), px.min(1)
    d = mx - mn
    s = np.where(mx > 0, d / np.maximum(mx, 1e-9), 0.0)
    r, g, b = px[:, 0], px[:, 1], px[:, 2]
    dd = np.maximum(d, 1e-9)
    h = np.where(mx == r, ((g - b) / dd) % 6, np.where(mx == g, (b - r) / dd + 2, (r - g) / dd + 4)) * 60.0
    return np.where(d > 0, h, 0.0), s, mx


def color_name(rgb_crop: np.ndarray) -> str:
    """Name of the object's colour in a crop: the majority hue name of the SATURATED pixels (s >= 0.30, v >= 0.20) when they
    are >= 15 % of the crop -- white bands, specular highlights and grey floors do not count; otherwise the object is
    genuinely white / gray / black (or unsaturated) and the old median rule names it (color_name_v1)."""
    px = np.asarray(rgb_crop, np.float64).reshape(-1, 3) / 255.0
    if not len(px):
        return "gray"
    h, s, v = _hsv(px)
    col = (s >= SAT_MIN) & (v >= VAL_MIN)
    if col.mean() >= SAT_FRAC_MIN:
        names = [_hue_name(float(a), float(b)) for a, b in zip(h[col], v[col])]
        vals, counts = np.unique(names, return_counts=True)
        return str(vals[int(np.argmax(counts))])
    return color_name_v1(rgb_crop)


def resolve_color(text: str) -> str | None:
    words = text.lower().split()
    for en in NAMES:
        if en in words:
            return en
    return None
