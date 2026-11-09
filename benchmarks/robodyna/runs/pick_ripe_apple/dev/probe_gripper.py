"""DEV ONLY (simulator internals): finger gap vs commanded / measured opening, and close timing (ur5-wsg).
Writes dev/out/gripper_<seed>.json."""
import json
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent / "out"


def run(api):
    genv = api._genv
    env = genv.env
    OUT.mkdir(exist_ok=True)
    names = list(env.robot.gripper_name)
    links = {l.get_name(): l for l in env.robot.right_entity.get_links()}
    fing = [n for n in names if n in links]
    rows = []

    def snap(tag):
        ps = {n: np.asarray(links[n].get_entity_pose().p).tolist() for n in fing}
        rows.append({"tag": tag, "t": api.time(), "opening": api.gripper_opening("right"), "links": ps})

    snap("start")
    for pos in (1.0, 0.9, 0.8, 0.7, 0.65, 0.6, 0.55, 0.5, 0.4, 0.3, 0.0, 1.0):
        t0 = api.time()
        api.close_gripper("right", pos)
        snap(f"cmd{pos}")
        rows[-1]["dur"] = api.time() - t0
    # timing of a full close sampled by world callbacks
    trace = []
    genv.step_callbacks.append(lambda n: trace.append((genv.sim_time, api.gripper_opening("right"))) if n % 5 == 0 else None)
    api.close_gripper("right", 0.0)
    genv.step_callbacks.clear()
    (OUT / f"gripper_{genv.seed}.json").write_text(json.dumps(
        {"fingers": fing, "all_gripper_names": names, "rows": rows, "close_trace": trace}, indent=0, default=str))
