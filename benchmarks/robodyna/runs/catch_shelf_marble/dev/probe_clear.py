"""DEV ONLY (simulator internals for readout): run the pipeline's _clear_bowl on v4 and print the
true canister pose and the gripper after every move. Never used by pipeline.py."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

P = C._load_pipeline(str(BENCH / "runs/catch_shelf_marble/pipeline.py"))
var = C.load_variation(str(BENCH / "runs/catch_shelf_marble" / (sys.argv[1] if len(sys.argv) > 1 else "variations/v4.yaml")))
g = gah_env.GahEnv("catch_shelf_marble")
e = g.env
g.reset(C.task_seeds("catch_shelf_marble")[0], var)
api = RobotAPI(g, noise_seed=0)
can = list(g.extra.values())[0]
orig = api.move_to


def move_to(pose, arm="right", *a, **k):
    ok = orig(pose, arm, *a, **k)
    p = can.actor.get_pose()
    print(f"t={api.time():5.2f} {arm} ok={ok} tgt={np.round(pose[:3], 3).tolist()} ee={np.round(api.ee_pose(arm)[:3], 3).tolist()} "
          f"can p={np.round(p.p, 3).tolist()} q={np.round(p.q, 2).tolist()}", flush=True)
    touch = sorted({(c.bodies[0].entity.get_name(), c.bodies[1].entity.get_name()) for c in e.scene.get_contacts()})
    print("   contacts", [t for t in touch if "canister" in t[0] + t[1] or "finger" in t[0] + t[1]])
    return ok


api.move_to = move_to
home_q = {arm: api.ee_pose(arm)[3:] for arm in ("left", "right")}
bowl = [api.detect("bowl") for _ in range(10)]
rim_z = float(np.mean([b.bbox_center[2] + min(b.bbox_extents) / 2 for b in bowl]))
print("rim", rim_z, "in bowl", P._in_bowl(api, rim_z))
arm = P._clear_bowl(api, home_q, rim_z)
print("clear ->", arm, "in bowl now", P._in_bowl(api, rim_z), "t", api.time())
if arm:
    kx, ky = e.key_xy[arm]
    top = e._reactive_buttons.tops_z[0]
    for dz in (0.18, 0.15):
        api.move_to([kx, ky, top + dz, *home_q[arm]], arm)
        api.wait(0.1)
        print("open-gripper press dz", dz, "held", e._key_pressed, "opening", round(api.gripper_opening(arm), 2))
g.close()
