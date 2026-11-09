"""DEV ONLY (simulator internals for readout): does pressing a key through the API move the bowl,
at what gripper height, and how long do small moves take. Never used by pipeline.py."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

seeds = C.task_seeds("catch_shelf_marble")
g = gah_env.GahEnv("catch_shelf_marble")
g.reset(seeds[int(sys.argv[1]) if len(sys.argv) > 1 else 3])
e = g.env
from envs._GLOBAL_CONFIGS import GRASP_DIRECTION_DIC  # noqa
print("GRASP_DIRECTION_DIC", {k: np.round(v, 4).tolist() for k, v in GRASP_DIRECTION_DIC.items()})
api = RobotAPI(g, noise_seed=0, pos_noise=0, rot_noise_deg=0)
bank = e._reactive_buttons
print("trigger_depth", bank.trigger_depth, "max_depth", bank.max_depth, "tops", bank.tops_z, "xy_tol", bank.xy_tol)
td = GRASP_DIRECTION_DIC["top_down"]


def st(tag):
    print(f"{tag:>28s} t={api.time():6.2f} eeL={np.round(api.ee_pose('left')[:3], 4).tolist()} "
          f"tcpL={np.round(api.tcp_pose('left')[:3], 4).tolist()} depth={np.round(bank.visual_depth, 4).tolist()} "
          f"held={e._key_pressed} bowl={round(float(e.bowl.get_pose().p[0]), 4)}", flush=True)


st("start")
t = api.time(); api.close_gripper("left"); print("close took", round(api.time() - t, 2))
st("closed")
kx, ky = e.key_xy["left"]
top = bank.tops_z[0]
for dz in (0.25, 0.20):
    t = api.time()
    ok = api.move_to([kx, ky, top + dz, *td], "left")
    st(f"hover ee=top+{dz} ok={ok} dt={api.time() - t:.2f}")
for dz in (0.19, 0.18, 0.175, 0.17, 0.165, 0.16, 0.155, 0.15, 0.145):
    t = api.time()
    ok = api.move_to([kx, ky, top + dz, *td], "left")
    st(f"ee=top+{dz} ok={ok} dt={api.time() - t:.2f}")
    api.wait(0.1)
    st("  +0.1s")
api.wait(0.5)
st("held 0.5 s")
for dz in (0.17, 0.2):
    t = api.time()
    ok = api.move_to([kx, ky, top + dz, *td], "left")
    st(f"up ee=top+{dz} ok={ok} dt={api.time() - t:.2f}")
    api.wait(0.1)
    st("  +0.1s")
# small press cycles: hover at top+0.175, press to top+0.15, release
for hover, press in ((0.175, 0.15), (0.17, 0.155)):
    api.move_to([kx, ky, top + hover, *td], "left"); api.wait(0.2)
    b0 = float(e.bowl.get_pose().p[0]); t0 = api.time()
    api.move_to([kx, ky, top + press, *td], "left"); t1 = api.time(); b1 = float(e.bowl.get_pose().p[0])
    api.move_to([kx, ky, top + hover, *td], "left"); t2 = api.time(); b2 = float(e.bowl.get_pose().p[0])
    api.wait(0.2); b3 = float(e.bowl.get_pose().p[0])
    print(f"cycle {hover}->{press}: down {t1 - t0:.2f}s db={b1 - b0:.4f}; up {t2 - t1:.2f}s db={b2 - b1:.4f}; after {b3 - b2:.4f}")
    st("after cycle")
# move_by small
api.move_to([kx, ky, top + 0.175, *td], "left"); api.wait(0.2)
for d in (-0.005, -0.01, -0.02, -0.03):
    t = api.time(); ok = api.move_by("left", dz=d); dt = api.time() - t
    t = api.time(); api.move_by("left", dz=-d); dt2 = api.time() - t
    print(f"move_by {d}: {dt:.2f}s back {dt2:.2f}s ok={ok}")
st("end")
for tr in api.trace[-6:]:
    print("trace", tr)
g.close()
