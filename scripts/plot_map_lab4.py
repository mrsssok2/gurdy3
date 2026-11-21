#!/usr/bin/env python3
"""PNG (pure stdlib) of a saved map with the lab4 walls (red) and objects (blue) overlaid:  python3 scripts/plot_map_lab4.py maps/lab4_map.yaml OUT.png
Also prints: share of true wall length covered by occupied cells (<= 1 cell away) and occupied cells farther than 0.15 m from any wall / object (spurious)."""
import math
import struct
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_world_lab4 as g  # noqa: E402


def read_pgm(p):
    d = open(p, "rb").read()
    _, wh, _, body = d.split(b"\n", 3)
    w, h = map(int, wh.split())
    return w, h, body


def main():
    yml, out = sys.argv[1], sys.argv[2]
    meta = {k.strip(): v.strip() for k, v in (l.split(":", 1) for l in open(yml) if ":" in l)}
    res = float(meta["resolution"]); ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    w, h, px = read_pgm(str(Path(yml).with_name(meta["image"])))
    occ = {(c, h - 1 - r) for r in range(h) for c in range(w) if px[r * w + c] < 50}   # (col, row from bottom)
    segs = []
    for name, (cx, cy), (sx, sy) in g.walls():
        if sx >= sy:
            segs.append((cx - sx / 2, cy, cx + sx / 2, cy))
        else:
            segs.append((cx, cy - sy / 2, cx, cy + sy / 2))
    pts = []
    for s in segs:
        n = max(2, int(math.hypot(s[2] - s[0], s[3] - s[1]) / 0.05))
        pts += [(s[0] + (s[2] - s[0]) * i / n, s[1] + (s[3] - s[1]) * i / n) for i in range(n + 1)]
    cells = lambda x, y: (int((x - ox) / res), int((y - oy) / res))
    cov = 0
    for x, y in pts:
        c, r = cells(x, y)
        if any((c + dc, r + dr) in occ for dc in (-2, -1, 0, 1, 2) for dr in (-2, -1, 0, 1, 2)):
            cov += 1
    obj = [(o[2], o[3]) for o in g.OBJECTS]
    spurious = 0
    for c, r in occ:
        x, y = ox + (c + 0.5) * res, oy + (r + 0.5) * res
        if min(math.hypot(x - a, y - b) for a, b in pts + obj) > 0.25:
            spurious += 1
    print(f"wall coverage {cov / len(pts):.1%} ({cov}/{len(pts)} wall samples have an occupied cell within 2 cells); occupied cells > 0.25 m from any wall/object: {spurious} of {len(occ)}")
    S = 4
    W, H = w * S, h * S
    img = bytearray([200] * (W * H * 3))

    def put(x, y, col):
        if 0 <= x < W and 0 <= y < H:
            i = (y * W + x) * 3
            img[i:i + 3] = bytes(col)
    for r in range(h):
        for c in range(w):
            v = px[r * w + c]
            col = (20, 20, 20) if v < 50 else ((255, 255, 255) if v > 240 else (150, 150, 150))
            for dy in range(S):
                for dx in range(S):
                    put(c * S + dx, r * S + dy, col)
    for x, y in pts:
        put(int((x - ox) / res * S), H - 1 - int((y - oy) / res * S), (230, 30, 30))
    for x, y in obj:
        cx, cy = int((x - ox) / res * S), H - 1 - int((y - oy) / res * S)
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                put(cx + dx, cy + dy, (30, 60, 230))
    raw = b"".join(b"\x00" + bytes(img[y * W * 3:(y + 1) * W * 3]) for y in range(H))
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    open(out, "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


main()
