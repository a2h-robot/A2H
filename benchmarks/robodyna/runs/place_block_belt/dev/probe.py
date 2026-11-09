"""Dev-only probe (uses simulator internals; never imported by pipeline.py).
Per seed: layout facts, perceived objects/parts, ee/tcp poses, grasp candidates."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

seeds = [int(s) for s in sys.argv[1:]] or C.task_seeds("place_block_belt")
g = gah_env.GahEnv("place_block_belt")
for seed in seeds:
    g.reset(seed)
    e = g.env
    api = RobotAPI(g, noise_seed=0, pos_noise=0.0, rot_noise_deg=0.0)
    print(f"\n=== seed {seed} mirrored={e.belt_mirrored} dir={e.belt_dir} belt_speed={e.belt_speed:.3f} "
          f"bowl_speed={e.bowl_move_speed:.3f} bowl_dir={e._bowl_move_dir} belt_y={e.belt_y} "
          f"blocker y={e.blocker_y:.3f} hy={e.blocker_half_y:.3f} x={e.blocker_x:.3f} hx={e.blocker_half_x:.3f} "
          f"clear=[{e.clear_y_lo:.3f},{e.clear_y_hi:.3f}] line_x={e.place_line_x:.3f} drop_x={e._belt_drop_x:.3f} "
          f"bowl_x={e._bowl_base_x:.3f} block_half_h={e.block_half_h:.4f} surf={e.belt_surface_z:.4f} t={g.sim_time:.2f}",
          flush=True)
    for name in api.objects() + api.parts():
        s = api.detect(name)
        print(f"  {name:13s} pos={np.round(s.position,3).tolist()} q={np.round(s.quaternion,3).tolist()} "
              f"ext={np.round(s.bbox_extents,3).tolist()} c={np.round(s.bbox_center,3).tolist()} pts={len(s.points)}")
    for arm in ("left", "right"):
        print(f"  {arm} ee={np.round(api.ee_pose(arm),3).tolist()} tcp={np.round(api.tcp_pose(arm),3).tolist()}")
    arm = "left" if e.belt_dir > 0 else "right"
    for c in api.grasp_candidates("block", arm, strategy="top"):
        print("  cand", c["contact_point"], np.round(c["pose"], 3).tolist(), round(c["score"], 3))
    s = api.detect("block")
    for k, v in s.points.items():
        print("  pt", k, np.round(v, 3).tolist())
    tr = api.track("bowl", 1.0, hz=10)
    print("  bowl y track", [round(float(x.position[1]), 3) for x in tr], flush=True)
