"""Dev-only: summarise failing seeds of an eval frames dir (local, reads traces)."""
import glob
import json
import sys

for d in sys.argv[1:]:
    for f in sorted(glob.glob(d + "/seed*_trace.json")):
        r = json.load(open(f))
        if r["success"]:
            continue
        n = [t for t in r["trace"] if t["call"] == "note"]
        print(d.split("/")[-1], f.split("/")[-1][:9], r.get("error"),
              [t["belt_speed"].get("v") for t in n if "belt_speed" in t],
              [t for t in n if "why" in t or "t_hit" in t][:2])
