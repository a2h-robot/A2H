"""Dev-only: test pipeline.speed_from_frames on dumped frames. arg: frame stride."""
import importlib.util
import sys
from pathlib import Path
import numpy as np
here = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("pl", str(here.parent / "pipeline.py"))
P = importlib.util.module_from_spec(spec)
spec.loader.exec_module(P)
step = int(sys.argv[1]) if len(sys.argv) > 1 else 1
verbose = len(sys.argv) > 2
for f in sorted((here / "out").glob("frames_*.npz")):
    d = np.load(f)
    fr, t = d["frames"][::step], d["t"][::step]
    dr = int(d["dir"])
    v, info = P.speed_from_frames(fr, t, 0.44, ref=(-dr * 0.10, -dr * 0.22))
    print(f.name, "true", round(float(d["v"]) * int(d["dir"]), 4), "est", {k: v for k, v in info.items() if k != "rows_detail"}, flush=True)
    if verbose:
        for row in info.get("rows_detail", [])[::3]:
            print("   ", row)
