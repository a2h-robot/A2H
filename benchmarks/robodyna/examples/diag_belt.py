"""Diagnostic: replay belt_smoke_pipeline on one seed and print place_block_belt's own latches over time."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(BENCH), str(BENCH / "tools"), str(BENCH / "examples")]
import numpy as np  # noqa: E402
import gah_env  # noqa: E402
from api import RobotAPI  # noqa: E402
import belt_smoke_pipeline as P  # noqa: E402

seed = int(sys.argv[1]) if len(sys.argv) > 1 else 10001
g = gah_env.GahEnv("place_block_belt")
g.reset(seed)
e = g.env
api = RobotAPI(g, noise_seed=0)
keys = ("_released", "_belt_active", "placed_on_belt", "dropped_at_start_left", "hit_blocker", "in_bowl",
        "_runs_without_expert")
def snap(tag):
    flags = {k: (getattr(e, k)() if callable(getattr(e, k, None)) else getattr(e, k, None)) for k in keys}
    print(f"{tag:>14s} t={g.sim_time:5.2f} block={np.round(e.block.get_pose().p, 3).tolist()} "
          f"bowl={np.round(e.bowl.get_pose().p, 3).tolist()} {flags} gate={dict(e._eval_gate_state)}", flush=True)
snap("start")
orig_place = api.place
def place(*a, **k):
    r = orig_place(*a, **k); snap("after place"); return r
api.place = place
P.run(api)
snap("after run")
for i in range(10):
    g.tick(250); snap(f"+{i + 1}s")
for t in api.trace: print("   trace", t)
print("success", g.success(), "reason", g.fail_reason(), "| info", {k: v for k, v in e.info.items() if not isinstance(v, (dict, list))})
