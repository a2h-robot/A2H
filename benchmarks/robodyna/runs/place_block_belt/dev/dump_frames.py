"""Dev-only: dump 13 head frames 0.25 s apart (arms still) to out/frames_<seed>.npz with ground truth."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

out = Path(__file__).resolve().parent / "out"
out.mkdir(exist_ok=True)
g = gah_env.GahEnv("place_block_belt")
import _common as C  # noqa
var = C.load_variation(str(BENCH.parents[1] / sys.argv[1])) if sys.argv[1].endswith(".yaml") else None
tag = Path(sys.argv[1]).stem + "_" if var else ""
for seed in [int(s) for s in sys.argv[1:] if not s.endswith(".yaml")]:
    g.reset(seed, var)
    e = g.env
    api = RobotAPI(g, noise_seed=0)
    fr, ts = [], []
    for k in range(27):
        fr.append(api.camera_frame("head"))
        ts.append(api.time())
        api.wait(0.15)
    np.savez_compressed(out / f"frames_{tag}{seed}.npz", frames=np.stack(fr), t=np.array(ts), v=e.belt_speed,
                        dir=e.belt_dir)
    print(seed, e.belt_speed, e.belt_dir)
