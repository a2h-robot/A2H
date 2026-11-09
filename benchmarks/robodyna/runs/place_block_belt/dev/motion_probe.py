"""Dev-only probe: does an object_motion edit move the block? Prints the true block pose over time."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
var = C.load_variation(str(BENCH.parents[1] / sys.argv[1]))
import gah_env  # noqa
from api import RobotAPI  # noqa

g = gah_env.GahEnv("place_block_belt")
for seed in [int(s) for s in sys.argv[2:]]:
    g.reset(seed, var)
    api = RobotAPI(g, noise_seed=0)
    for k in range(12):
        p = g.env.block.get_pose().p
        print(f"seed {seed} t={api.time():5.2f} block={np.round(p, 3).tolist()}", flush=True)
        api.wait(1.0)
