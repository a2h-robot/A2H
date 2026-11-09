"""Dev-only diagnostic (simulator internals): run pipeline.py on seeds and print the task's latches
over time next to the pipeline's notes.  python diag.py [variation.yaml|-] seed ..."""
import importlib.util
import json
import os
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

args = sys.argv[1:]
var = None
if args and (args[0].endswith(".yaml") or args[0] == "-"):
    var = C.load_variation(str(BENCH.parents[1] / args[0])) if args[0] != "-" else None
    args = args[1:]
seeds = [int(s) for s in args] or C.task_seeds("place_block_belt")
spec = importlib.util.spec_from_file_location("pl", str(Path(__file__).resolve().parents[1] / "pipeline.py"))
P = importlib.util.module_from_spec(spec)
spec.loader.exec_module(P)
g = gah_env.GahEnv("place_block_belt")
for seed in seeds:
    g.reset(seed, var)
    e = g.env
    api = RobotAPI(g, noise_seed=10000 + seed)
    ev = []
    st = {"rel": False, "belt": False, "drop": False, "contact": False, "kin": False}

    def cb(n, e=e, ev=ev, st=st):
        t = g.sim_time
        for key, val in (("contact", e._belt_contact_latched), ("rel", e._released), ("belt", e._block_kinematic),
                         ("drop", e._block_dropped)):
            if val and not st[key]:
                st[key] = True
                ev.append((round(t, 3), key, np.round(e.block.get_pose().p, 3).tolist(),
                           round(float(e.bowl.get_pose().p[1]), 3)))
    g.step_callbacks.append(cb)
    vid = []
    if os.environ.get("DIAG_VIDEO"):
        g.step_callbacks.append(lambda n, vid=vid: vid.append((round(g.sim_time, 1), api.camera_frame())) if n % 50 == 0 else None)
    err = None
    try:
        P.run(api)
    except Exception as ex:
        import traceback
        err = traceback.format_exc(limit=5)
    t_ret = g.sim_time
    g.finish()
    g.step_callbacks.clear()
    print(f"\n=== seed {seed} mirrored={e.belt_mirrored} v_belt={e.belt_speed:.4f} v_bowl={e.bowl_move_speed:.3f} "
          f"clear=[{e.clear_y_lo:.3f},{e.clear_y_hi:.3f}] success={g.success()} at "
          f"{None if g.success_step is None else round(g.success_step / 250, 2)} ret={t_ret:.2f} end={g.sim_time:.2f}")
    print("   flags", {k: e.info.get(k) for k in ("placed_before_line", "avoided_blocker", "hit_blocker")},
          "contact", e._belt_contact_x, e._belt_contact_y, "block_end", np.round(e.block.get_pose().p, 3).tolist(),
          "bowl_end", np.round(e.bowl.get_pose().p, 3).tolist(), "tilt", round(e.max_tilt_deg, 1))
    for x in ev:
        print("   ev", x)
    for tr in api.trace:
        if tr["call"] == "note" or (tr["call"] in ("move_to", "gripper") and not tr.get("ok", True)):
            print("   ", json.dumps(tr, default=str))
        elif tr["call"] == "gripper":
            print("    ", json.dumps(tr))
    if err:
        print(err)
    if vid:
        import imageio
        out = Path(__file__).resolve().parent / "out"
        out.mkdir(exist_ok=True)
        imageio.mimsave(out / f"diag_{seed}.mp4", [f for _, f in vid], fps=5)
    sys.stdout.flush()
