"""DEV ONLY (simulator internals): per seed, the shelf layout, descent plan and the marble's true
trajectory with the arms idle. Never used by pipeline.py."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

idx = [int(a) for a in sys.argv[1:] if a != "-v"] or list(range(10))
seeds = C.task_seeds("catch_shelf_marble")
g = gah_env.GahEnv("catch_shelf_marble")
for k in idx:
    s = seeds[k]
    g.reset(s)
    e = g.env
    api = RobotAPI(g, noise_seed=0, pos_noise=0, rot_noise_deg=0)
    print(f"\n=== idx {k} seed {s} n={e.n_shelves} osc={e.osc_shelf_idx} period={e.osc_period} roll={e.roll_speed}")
    print("  centers", np.round(e.shelf_centers_x, 3).tolist(), "z", np.round(e.shelf_z, 3).tolist(),
          "ang", np.round(e.shelf_angle_deg, 1).tolist())
    print("  belt", round(e.belt_x_min, 3), round(e.belt_x_max, 3), "bowl range", round(e.bowl_x_min, 3),
          round(e.bowl_x_max, 3), "bowl0", np.round(e.bowl.get_pose().p, 3).tolist(), "target",
          round(e.target_catch_x, 3), "inner_r", getattr(e, "_bowl_inner_r", None), "h", getattr(e, "_bowl_height", None))
    print("  keys", {sd: np.round(e.keys[sd].get_pose().p, 3).tolist() for sd in e.keys}, "top", e.key_top_z,
          "state", e._marble_state, "total_steps", getattr(e, "total_marble_steps", None))
    for leg in e.descent_legs:
        print("   leg", {kk: (np.round(v, 3).tolist() if isinstance(v, (list, tuple, np.ndarray)) else v)
                         for kk, v in leg.items() if kk != "locals"})
    for nm in api.objects() + api.parts():
        st = api.detect(nm)
        print("  det", nm, np.round(st.position, 3).tolist(), np.round(st.bbox_extents, 3).tolist())
    print("  ee L", np.round(api.ee_pose("left"), 3).tolist(), "tcp L", np.round(api.tcp_pose("left"), 3).tolist())
    print("  ee R", np.round(api.ee_pose("right"), 3).tolist(), "tcp R", np.round(api.tcp_pose("right"), 3).tolist())
    for i in range(400):
        g.tick(25)
        p = np.round(e.ball.get_pose().p, 3).tolist()
        if (i % 4 == 0 and "-v" in sys.argv) or e._marble_state != "descending":
            print(f"   t={g.sim_time:5.2f} ball={p} st={e._marble_state} res={e._marble_result} leg={e._leg_idx}")
        if e._marble_state in ("resolved", "landed"):
            break
    print("  target", round(e.target_catch_x, 3), "fail", g.fail_reason())
    g.close()
