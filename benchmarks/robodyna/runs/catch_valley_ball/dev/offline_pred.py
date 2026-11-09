"""Offline check of the landing predictor on recorded true trajectories (probe1_*.json) with
simulated perception noise. python offline_pred.py"""
import glob
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("pl", HERE.parent / "pipeline.py")
pl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pl)

SIG = 0.01 / np.sqrt(pl.N_AVG)
times = [float(a) for a in sys.argv[1:]] or [1.4, 3.0, 4.0, 4.5]
rng = np.random.default_rng(0)
for f in sorted(glob.glob(str(HERE / "probe1_*.json"))):
    d = json.load(open(f))
    tr = np.array([r[:7] for r in d["traj"]], dtype=float)
    side = d["side"]
    rl = d["red_line_x"]
    hw = d["ramp_half_width"]
    cy = 0.08
    land_t = [r for r in d["traj"] if r[7] == "released" and (r[1] - rl) * side > 0.08 and r[3] < 0.80]
    true = land_t[0][2] if land_t else None
    lo, hi = cy - (hw - 0.022), cy + (hw - 0.022)
    x_exit = rl - side * pl.EXIT_BEFORE_LINE
    x_land = rl + side * pl.LAND_PAST_LINE
    out = []
    for T in times:
        win = tr[(tr[:, 0] >= max(0.25, T - 2.5)) & (tr[:, 0] <= T)]
        win = win[::1]
        nr = win[:, 1:3] + rng.normal(0, SIG, (len(win), 2))
        nd = win[:, 4:6] + rng.normal(0, SIG, (len(win), 2))
        R = np.column_stack([win[:, 0], nr, win[:, 3]])
        D = np.column_stack([win[:, 0], nd, win[:, 6]])
        yl, ydl = pl.predict_landing(R, D, lo, hi, x_exit, x_land)
        out.append((round(yl, 3), round(ydl, 3)))
    print(d["seed"], "true", true, "env", d["landing"][1], "pred@", dict(zip(times, out)),
          "true exit y", d["ball_exit"][1])
