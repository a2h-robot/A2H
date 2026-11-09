"""DEV ONLY (simulator internals): world AABBs of the left arm's links at home, mid-way and at the
key hover / press poses, to place obstacles. Never used by pipeline.py."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
import scene_edits  # noqa
from api import RobotAPI  # noqa

g = gah_env.GahEnv("catch_shelf_marble")
g.reset(C.task_seeds("catch_shelf_marble")[0])
e = g.env
api = RobotAPI(g, noise_seed=0, pos_noise=0, rot_noise_deg=0)


def dump(tag, arm="left"):
    ent = e.robot.left_entity if arm == "left" else e.robot.right_entity
    print(f"--- {tag} ee={np.round(api.ee_pose(arm)[:3], 3).tolist()}")
    for link in ent.get_links():
        try:
            lo, hi = scene_edits.world_aabb(link.entity if hasattr(link, "entity") else link)
        except Exception as ex:
            continue
        if np.all(np.isfinite(lo)):
            print(f"   {link.get_name():32s} lo={np.round(lo, 3).tolist()} hi={np.round(hi, 3).tolist()}")


dump("home")
api.close_gripper("left")
dump("home closed")
kx, ky = e.key_xy["left"]
top = e._reactive_buttons.tops_z[0]
q = api.ee_pose("left")[3:]
api.move_to([kx, ky, top + 0.18, *q], "left")
dump("hover")
api.move_to([kx, ky, top + 0.15, *q], "left")
dump("press")
g.close()
