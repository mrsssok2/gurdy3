import json, math, sys
from collections import Counter
import numpy as np
D = sys.argv[1]; T = np.array([-0.8, 0.7])
rows = [json.loads(l) for l in open(D + "/chat/sightings.jsonl")]
print("sightings", len(rows), "t", rows[0]["t"], "-", rows[-1]["t"])
near = [r for r in rows if math.hypot(r["p"][0] - T[0], r["p"][1] - T[1]) < 0.4]
print("within 0.4 m of the cube:", len(near), Counter((r["cls"], r["color"]) for r in near).most_common())
for r in near[:40]:
    print("  t %.1f %-14s %-6s score %.2f range %.2f p (%.2f, %.2f) box %s trunc %s" % (r["t"], r["cls"], r["color"], r["score"], r["range"], r["p"][0], r["p"][1], [round(v) for v in r["box"]], r["truncated"]))
sb = [r for r in rows if r["cls"] == "small_box"]
print("all small_box sightings:", len(sb), Counter((r["color"], round(r["p"][0], 1), round(r["p"][1], 1)) for r in sb).most_common(8))
# camera poses: was the cube in the field of view (60 deg hfov, 4:3, pitched) in frames that had ANY detection?
def R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
inview = []
seen_t = set()
for r in rows:
    if r["t"] in seen_t: continue
    seen_t.add(r["t"])
    c = r["cam"]; t = np.array(c[:3]); Rm = R(c[3:])
    pc = Rm.T @ (np.array([T[0], T[1], -0.085]) - t)       # optical frame: z forward, x right, y down
    if pc[2] > 0.1:
        u = 160 + 233.9 * 0 + (pc[0] / pc[2]) * 277.1; v = 120 + (pc[1] / pc[2]) * 277.1
        if 0 <= u < 320 and 0 <= v < 240:
            inview.append((r["t"], float(np.linalg.norm(pc)), u, v))
print("frames (with any detection) where the cube centre projects into the image:", len(inview))
for x in inview[:30]:
    print("  t %.1f dist %.2f m  pixel (%.0f, %.0f)" % x)
