import importlib.util, json
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("pl", HERE.parent / "pipeline.py"); pl = importlib.util.module_from_spec(spec); spec.loader.exec_module(pl)
d = json.load(open(HERE / "probe1_10000.json"))
tr = np.array([r[:7] for r in d["traj"]], float)
hw = d["ramp_half_width"]; lo, hi = 0.08 - (hw - 0.022), 0.08 + (hw - 0.022)
w = tr[(tr[:, 0] >= 2.0) & (tr[:, 0] <= 4.5)]
print(np.round(w[:, 1:3], 3).tolist())
for k in [25, 15, 8]:
    xs, ys = w[-k:, 1], w[-k:, 2]
    print(k, pl.fit_lane(xs, ys, lo, hi, xs[-1]), pl.fit_recent(xs, ys, lo, hi, xs[-1]))
