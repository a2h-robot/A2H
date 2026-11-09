"""DEV ONLY (simulator internals): with a variation's added obstacles, try approaches to the left key
(direct, and from above) and print where the gripper ends up, what it touches and whether the key
fires. Never used by pipeline.py.  python .../dev/probe_bar.py variations/vK.yaml"""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

var = C.load_variation(str(BENCH / "runs/catch_shelf_marble" / sys.argv[1]))
g = gah_env.GahEnv("catch_shelf_marble")
e = g.env

for mode in ("direct", "direct_avoid", "above"):
    g.reset(C.task_seeds("catch_shelf_marble")[0], var)
    api = RobotAPI(g, noise_seed=0, pos_noise=0, rot_noise_deg=0)
    api.close_gripper("left")
    kx, ky = e.key_xy["left"]
    top = e._reactive_buttons.tops_z[0]
    q = api.ee_pose("left")[3:]
    obs = api.get_obstacles(exclude=("red ball", "bowl"))
    av = obs if mode != "direct" else None
    steps = [0.55, 0.18, 0.15] if mode == "above" else [0.18, 0.15]
    for dz in steps:
        tgt = [kx, ky, top + dz, *q]
        ok = api.move_to(tgt, "left", avoid=av)
        ee = api.ee_pose("left")
        touch = sorted({(c.bodies[0].entity.get_name(), c.bodies[1].entity.get_name()) for c in e.scene.get_contacts()
                        if "obstacle" in (c.bodies[0].entity.get_name() + c.bodies[1].entity.get_name())
                        or "bar" in (c.bodies[0].entity.get_name() + c.bodies[1].entity.get_name())})
        print(f"{mode:12s} dz={dz} ok={ok} err={np.linalg.norm(np.array(ee[:3]) - tgt[:3]):.4f} "
              f"ee={np.round(ee[:3], 3).tolist()} held={e._key_pressed['left']} touch={touch}", flush=True)
    g.close()
