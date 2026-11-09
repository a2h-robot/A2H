"""Dev-only: how low can each top-down orientation reach over the block, and what opening results."""
import importlib.util
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

spec = importlib.util.spec_from_file_location("pl", str(Path(__file__).resolve().parents[1] / "pipeline.py"))
P = importlib.util.module_from_spec(spec)
spec.loader.exec_module(P)
g = gah_env.GahEnv("place_block_belt")
for seed in [int(s) for s in sys.argv[1:]]:
    for yaw_extra in (0.0, np.pi):
        for target in (0.795, 0.785):
            g.reset(seed)
            api = RobotAPI(g, noise_seed=0, pos_noise=0.0, rot_noise_deg=0.0)
            b = api.detect("block")
            arm = "left" if b.position[0] < 0 else "right"
            A = P.Arm(api, arm)
            q = A.q_yaw(yaw_extra)
            bc = b.bbox_center
            ok1 = A.move_tcp([bc[0], bc[1], 0.90], q)
            ok2 = A.move_tcp([bc[0], bc[1], target], q, hold_orientation=True, speed="slow")
            r = A.tcp()
            api.close_gripper(arm)
            op = api.gripper_opening(arm)
            ok3 = A.move_tcp([r[0], r[1], r[2] + 0.15], q, hold_orientation=True)
            bz = api.detect("block").bbox_center[2]
            print(f"seed {seed} arm {arm} yaw+{yaw_extra:.2f} target {target} ok {ok1} {ok2} {ok3} reached z {r[2]:.4f} "
                  f"opening {op:.3f} block z after lift {bz:.3f}", flush=True)
