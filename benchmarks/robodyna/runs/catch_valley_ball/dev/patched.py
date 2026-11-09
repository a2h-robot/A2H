"""DIAGNOSTIC ONLY (not a scored run): hand the box to PhysX the way RoboDyna's interactive mode does
(setup_demo -> _enable_box_physics), then run the real pipeline. Shows what the pipeline would do if
the eval path did not leave the box kinematic (see backend_issues.md)."""
import importlib.util
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent


def run(api):
    env = api._env
    env._enable_box_physics()
    env._push_active = True
    spec = importlib.util.spec_from_file_location("cvb_pipeline", HERE.parent / "pipeline.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    traj = []
    info = {"seed": api._genv.seed}

    def dump():
        (HERE / f"patched_{api._genv.seed}.json").write_text(json.dumps({**info, "traj": traj}, default=str))

    def cb(n):
        if n % 10 == 0:
            traj.append([n, *np.round(env.ball.get_pose().p, 3).tolist(), *np.round(env.bowl.get_pose().p, 3).tolist(),
                         *np.round(env.distractor.get_pose().p, 3).tolist(), *np.round(env.bowl.get_pose().q, 3).tolist()])
        if n % 250 == 0 and info:
            dump()
    api._genv.step_callbacks.append(cb)
    err = None
    try:
        mod.run(api)
    except Exception as e:
        err = repr(e)
        raise
    finally:
        info.update({
            "seed": api._genv.seed, "err": err, "debug": mod.DEBUG, "landing": np.round(env.landing, 3).tolist(),
            "distractor_landing": None if env.distractor_landing is None else np.round(env.distractor_landing, 3).tolist(),
            "box_end_run": np.round(env.bowl.get_pose().p, 3).tolist(), "t_end_run": api.time(),
            "red_line_x": float(env.red_line_x)})
        dump()
