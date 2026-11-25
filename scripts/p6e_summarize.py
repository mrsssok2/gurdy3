"""Mean / range of N replay runs per (suite, picker) from docs/results/phase6e/<tag>/*.json (scores recomputed with the scorer, nothing re-scored from frozen docs)."""
import glob, json, re, sys, collections
from brain.eval import incident_score as SC
tag = sys.argv[1]
by = collections.defaultdict(list)
for f in sorted(glob.glob(f"docs/results/phase6e/{tag}/*.json")):
    m = re.match(r"(.*)_(on|off)_(\d)\.json", f.split("/")[-1])
    SC.use_suite(m[1]); out = json.load(open(f)); s = SC.score(out)
    by[(m[1], m[2])].append((s["answers_ok"], s["routed"], s["content_ok"], s["wrong_claims"], s, f))
for (suite, pk), runs in sorted(by.items()):
    ok = [r[0] for r in runs]
    print(f"{suite:24s} picker {pk:3s} n={len(runs)} ok {ok} mean {sum(ok)/len(ok):.1f}/20 range {min(ok)}-{max(ok)} | routed {[r[1] for r in runs]} content {[r[2] for r in runs]} wrong claims {[r[3] for r in runs]}")
