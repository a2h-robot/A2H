"""DIAGNOSTIC (internals for logging only): x-push with the rotated open gripper, one finger on the
box's x face; contact offset and time per step for 3, 6 and 10 cm steps."""
import importlib.util
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent


def run(api):
    spec = importlib.util.spec_from_file_location("pl", HERE.parent / "pipeline.py")
    pl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pl)
    env = api._env
    side = float(env.side)
    arm = "right" if side > 0 else "left"
    q = pl.qmul(api.ee_pose(arm)[3:], [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])
    hand = pl.Hand(api, arm, q)
    log = []
    b = np.asarray(env.bowl.get_pose().p)
    sx = side

    def snap(tag, t0):
        bb = np.asarray(env.bowl.get_pose().p)
        log.append({"tag": tag, "dt": round(api.time() - t0, 3), "t": round(api.time(), 2),
                    "box": np.round(bb, 4).tolist(), "tcp": np.round(hand.tcp(), 4).tolist(),
                    "box_minus_tcp": np.round(bb[:2] - hand.tcp()[:2], 4).tolist()})

    t = api.time()
    hand.to([b[0] - sx * 0.12, b[1] - 0.06, pl.HOVER_Z])
    snap("hover", t)
    t = api.time()
    hand.to([b[0] - sx * 0.12, b[1] - 0.06, pl.PUSH_Z])
    snap("down", t)
    for step, n in ((0.03, 4), (0.06, 4), (0.10, 2)):
        for i in range(n):
            c = hand.tcp()
            t = api.time()
            hand.to([c[0] + sx * step, b[1] - 0.06, pl.PUSH_Z], straight=True)
            snap(f"s{step}", t)
    (HERE / f"probe5_{api._genv.seed}.json").write_text(json.dumps(log, indent=1))
