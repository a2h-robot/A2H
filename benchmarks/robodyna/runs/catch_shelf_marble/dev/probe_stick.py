"""DEV ONLY (simulator internals): v3's stick in the bowl: where is it at reset, and does a sideways
sweep of a closed gripper knock it out? Never used by pipeline.py."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

var = C.load_variation(str(BENCH / "runs/catch_shelf_marble/variations/v3.yaml"))
g = gah_env.GahEnv("catch_shelf_marble")
e = g.env
for idx in (0, 3):
    g.reset(C.task_seeds("catch_shelf_marble")[idx], var)
    api = RobotAPI(g, noise_seed=0, pos_noise=0, rot_noise_deg=0)
    st = api.detect("stick")
    print("objects", api.objects(), "stick", np.round(st.position, 3).tolist(), np.round(st.bbox_extents, 3).tolist(),
          "bowl", np.round(e.bowl.get_pose().p, 3).tolist())
    for nm, o in [(o.name, o) for o in api.get_obstacles()]:
        print("  obstacle", nm, np.round(o.center, 3).tolist(), np.round(o.extents, 3).tolist())
    q = api.ee_pose("left")[3:]
    bx = float(e.bowl.get_pose().p[0])
    t0 = api.time()
    api.close_gripper("left")
    for tag, y, z in (("behind", -0.09, 0.985), ("down", -0.09, 0.975), ("sweep", 0.07, 0.975), ("up", 0.07, 1.1)):
        ok = api.move_to([bx, y, z, *q], "left")
        p = e.stick.get_pose().p if hasattr(e, "stick") else g.extra["stick"].actor.get_pose().p
        print(f"  {tag:7s} ok={ok} t={api.time() - t0:.2f} ee={np.round(api.ee_pose('left')[:3], 3).tolist()} "
              f"stick={np.round(p, 3).tolist()}", flush=True)
    api.wait(1.0)
    p = g.extra["stick"].actor.get_pose().p
    print("  after 1 s stick", np.round(p, 3).tolist(), "bowl", np.round(e.bowl.get_pose().p, 3).tolist())
    g.close()
