"""Diagnostic probe: can the arm push the box? (uses internals for logging only)."""
import json
from pathlib import Path
import numpy as np


def run(api):
    env = api._env
    rig = env._get_rigid(env.bowl)
    log = []

    def snap(tag):
        log.append({"tag": tag, "t": round(api.time(), 3), "kin": bool(rig.kinematic),
                    "box": np.round(env.bowl.get_pose().p, 4).tolist(),
                    "tcp": np.round(api.tcp_pose(arm), 4).tolist(), "ee": np.round(api.ee_pose(arm), 4).tolist()})

    side = float(env.side)
    arm = "right" if side > 0 else "left"
    box = api.detect("blue box").position
    q = api.ee_pose(arm)[3:]
    snap("start")
    api.close_gripper(arm)
    snap("closed")
    ok1 = api.move_to([box[0], box[1] - 0.10, 0.95, *q], arm)
    snap(f"above ok={ok1}")
    ok2 = api.move_to([box[0], box[1] - 0.10, 0.89, *q], arm)
    snap(f"down ok={ok2}")
    for i in range(4):
        ok = api.move_by(arm, dy=0.05)
        snap(f"push{i} ok={ok}")
    ok = api.move_by(arm, dz=0.1)
    snap(f"up ok={ok}")
    d = Path(__file__).resolve().parent
    (d / f"probe2_{api._genv.seed}.json").write_text(json.dumps(log, indent=1))
