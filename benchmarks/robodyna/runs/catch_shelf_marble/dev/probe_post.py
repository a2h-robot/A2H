"""DEV ONLY (simulator internals): with v2's post, try hover poses for the left arm and print where
the gripper ends up and what it touches. Never used by pipeline.py."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

var = C.load_variation(str(BENCH / "runs/catch_shelf_marble/variations/v2.yaml"))
g = gah_env.GahEnv("catch_shelf_marble")
e = g.env
top = None


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


for yaw, ox, oy in [(0, 0, 0), (np.pi / 2, 0, 0), (-np.pi / 2, 0, 0), (0, 0.02, 0), (np.pi / 2, 0.02, 0), (0, 0.035, 0)]:
    g.reset(C.task_seeds("catch_shelf_marble")[0], var)
    api = RobotAPI(g, noise_seed=0, pos_noise=0, rot_noise_deg=0)
    api.close_gripper("left")
    kx, ky = e.key_xy["left"]
    top = e._reactive_buttons.tops_z[0]
    q = qmul([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)], api.ee_pose("left")[3:])
    obs = api.get_obstacles(exclude=("red ball", "bowl"))
    for dz in (0.18, 0.15):
        tgt = [kx + ox, ky + oy, top + dz, *q]
        ok = api.move_to(tgt, "left", avoid=obs)
        ee = api.ee_pose("left")
        touch = sorted({(c.bodies[0].entity.get_name(), c.bodies[1].entity.get_name()) for c in e.scene.get_contacts()
                        if "post" in (c.bodies[0].entity.get_name() + c.bodies[1].entity.get_name())})
        print(f"yaw={yaw:+.2f} off=({ox},{oy}) dz={dz} ok={ok} err={np.linalg.norm(np.array(ee[:3]) - tgt[:3]):.4f} "
              f"ee={np.round(ee[:3], 3).tolist()} depth={np.round(e._reactive_buttons.visual_depth, 4).tolist()} "
              f"touch={touch}", flush=True)
    g.close()
