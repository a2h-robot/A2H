"""Dev-only: save head-camera frames 0.2 s apart and print band x positions (internals) for comparison."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa
from PIL import Image  # noqa

seed = int(sys.argv[1]) if len(sys.argv) > 1 else 10000
out = Path(__file__).resolve().parent / "out"
out.mkdir(exist_ok=True)
g = gah_env.GahEnv("place_block_belt")
g.reset(seed)
e = g.env
api = RobotAPI(g, noise_seed=0)
print("belt_speed", e.belt_speed, "dir", e.belt_dir)
for i in range(4):
    img = api.camera_frame("head")
    print(i, img.shape, "t", api.time(), "bands", [round(float(b.get_pose().p[0]), 3) for b in e._belt_bands])
    Image.fromarray(img).save(out / f"head_{seed}_{i}.png")
    api.wait(0.2)
for v in ("front",):
    try:
        Image.fromarray(api.camera_frame(v)).save(out / f"{v}_{seed}.png")
    except Exception as ex:
        print(v, ex)
cams = e.cameras
print([k for k in cams.get_rgb()])
