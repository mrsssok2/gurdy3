"""Scan inter-arrival statistics and the safety-shield stale-scan timeout rule (no ROS, stdlib only).

Decision rule (docs/results/phase9f_small_items.md):
    timeout = clamp(margin x p99(gap), floor, cap)        margin 1.2, floor 0.5 s (today's default), cap 1.0 s
The cap is the Nav2 collision monitor's source_timeout (nav2_params.yaml: 1.0 s): the shield must not be laxer than
the layer before it. If p99 x margin exceeds the cap, the timeout stays at the cap and the cause (late scans, e.g.
detector CPU load) must be fixed instead; the result says so (`capped`).
"""
from __future__ import annotations

import math

FLOOR_S, CAP_S, MARGIN, PCT = 0.5, 1.0, 1.2, 99.0


def gaps(times) -> list:
    t = sorted(float(x) for x in times)
    return [b - a for a, b in zip(t, t[1:])]


def percentile(v, p):
    v = sorted(v)
    if not v:
        return math.nan
    k = (len(v) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def stats(g, thresholds=(0.3, 0.5, 1.0)) -> dict:
    if not g:
        return {"n": 0}
    out = {"n": len(g), "mean": sum(g) / len(g), "p50": percentile(g, 50), "p95": percentile(g, 95),
           "p99": percentile(g, 99), "max": max(g)}
    for th in thresholds:
        out[f"n_over_{th}"] = sum(x > th for x in g)
    return out


def choose_timeout(g, pct=PCT, margin=MARGIN, floor=FLOOR_S, cap=CAP_S) -> dict:
    """-> {timeout, p, raw, capped, reason}"""
    if len(g) < 100:
        return {"timeout": floor, "p": None, "raw": None, "capped": False,
                "reason": f"only {len(g)} gaps: keep the default {floor} s, record longer (>= 100 gaps, ideally 10 min under load)"}
    p = percentile(g, pct)
    raw = margin * p
    t = min(max(raw, floor), cap)
    capped = raw > cap
    reason = (f"p{pct:g} gap {p:.3f} s x {margin} = {raw:.3f} s"
              + (f" > cap {cap} s: timeout stays {cap} s; FIX THE LATE SCANS (load) instead of raising it" if capped else
                 f" -> {t:.2f} s" + (" (floor)" if raw < floor else "")))
    return {"timeout": round(t, 3), "p": p, "raw": raw, "capped": capped, "reason": reason}


def validate_timeout(value: float, cap=CAP_S, minimum=0.05) -> tuple:
    """Parameter check for the node: -> (used value, warning or None)."""
    v = float(value)
    if not math.isfinite(v) or v < minimum:
        return FLOOR_S, f"scan_timeout {value!r} invalid: using {FLOOR_S} s"
    if v > cap:
        return cap, f"scan_timeout {v} s above the cap {cap} s (Nav2 collision monitor source_timeout): using {cap} s"
    return v, None
