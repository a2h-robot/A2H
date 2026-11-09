"""DEV ONLY (uses simulator internals; never imported by pipeline.py).
Runs pipeline.py (or the file in GAH_PROBE_PIPELINE) and logs ground truth every 25 physics steps to
runs/pick_ripe_apple/dev/out/<tag>_<seed>.json.
eval.py --task pick_ripe_apple --pipeline benchmarks/robodyna/runs/pick_ripe_apple/dev/probe_gt.py
"""
import importlib.util
import json
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"


def _load(path):
    spec = importlib.util.spec_from_file_location("probe_target", path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def run(api):
    genv = api._genv
    env = genv.env
    OUT.mkdir(exist_ok=True)
    tag = os.environ.get("GAH_PROBE_TAG", "probe")
    target = os.environ.get("GAH_PROBE_PIPELINE", str(HERE.parent / "pipeline.py"))
    mod = _load(target)
    rows = []

    def rigid_info(r):
        try:
            return {"grav_off": bool(r.get_disable_gravity()), "kin": bool(r.get_kinematic()),
                    "v": np.round(r.get_linear_velocity(), 3).tolist()}
        except Exception as e:
            return str(e)

    events = []

    def contacts():
        out = []
        an = env.apple.get_name()
        for c in env.scene.get_contacts():
            a, b = c.bodies[0].entity.name, c.bodies[1].entity.name
            if an in (a, b):
                o = b if a == an else a
                out.append([o, [np.round(pt.position, 3).tolist() for pt in c.points][:2]])
        return out

    def cb(n):
        if n % 5 == 0 and len(events) < 400 and (env._apple_attached or len(events) < 400):
            cs = contacts()
            if cs and any(x[0] in env.robot.gripper_name for x in cs):
                events.append({"t": round(genv.sim_time, 3), "att": bool(env._apple_attached),
                               "openL": round(api.gripper_opening("left"), 3),
                               "openR": round(api.gripper_opening("right"), 3), "contacts": cs[:4]})
        if n % 25:
            return
        row = {"t": round(genv.sim_time, 2), "r": round(float(env.ripeness), 3),
               "rs": round(float(getattr(env, "spoiled_ripeness", -1)), 3),
               "att": bool(env._apple_attached), "satt": bool(getattr(env, "_spoiled_attached", False)),
               "r_grasp": env.r_grasp, "apple": np.round(env._apple_eval_p(env.apple), 3).tolist(),
               "basket": np.round(env.basket.get_pose().p, 3).tolist(),
               "held": bool(env._apple_held_by_gripper(env.apple)),
               "rigid": rigid_info(env._apple_rigid),
               "tcpL": np.round(env.robot.get_left_tcp_pose()[:3], 3).tolist(),
               "tcpR": np.round(env.robot.get_right_tcp_pose()[:3], 3).tolist(),
               "fail": str(getattr(env, "_last_fail_reason", ""))}
        extra_names = set(genv.extra) | {getattr(getattr(v, "actor", v), "name", k) for k, v in genv.extra.items()}
        pc = set()
        for c in env.scene.get_contacts():
            a, b = c.bodies[0].entity.name, c.bodies[1].entity.name
            if a in extra_names or b in extra_names:
                pc.add(f"{a}|{b}")
        row["extra_contacts"] = sorted(pc)
        if env.spoiled_apple is not None:
            row["spoiled"] = np.round(env._apple_eval_p(env.spoiled_apple), 3).tolist()
        rows.append(row)

    genv.step_callbacks.append(cb)
    head = {"seed": genv.seed, "apple_side": env.apple_side, "red_window": env.red_window,
            "red_tol": env.red_tol, "ripen_steps": env.ripen_steps, "basket_speed": env.basket_speed,
            "basket_dir": env._basket_move_dir, "basket_x_bounds": [env.basket_x_min, env.basket_x_max],
            "basket_half_xy": list(env.basket_half_xy), "basket_top_z": env.basket_top_z,
            "basket_base_z": env.basket_base_z, "basket_id": env.basket_id, "basket_y": env.basket_y,
            "high_branch_side": env.high_branch_side,
            "good_center": np.round(env._apple_grasp_center(env.apple), 4).tolist(),
            "hang": {str(k): np.round(v.p, 4).tolist() for k, v in env._hang_poses.items()},
            "detect": {}}
    for nm in api.objects() + api.parts():
        s = api.detect(nm)
        head["detect"][nm] = {"pos": np.round(s.position, 3).tolist(), "ext": np.round(s.bbox_extents, 3).tolist(),
                              "c": np.round(s.bbox_center, 3).tolist(),
                              "rgb": None if s.rgb is None else np.round(s.rgb, 2).tolist(),
                              "points": list(s.points)}
    err = None
    try:
        mod.run(api)
    except Exception as e:
        err = f"{type(e).__name__}: {e}"
        raise
    finally:
        # 3 s more of world time, still logged
        try:
            api.wait(3.0)
        except Exception:
            pass
        head["pipeline_log"] = getattr(mod, "LOG", None)
        head["err"] = err
        head["rows"] = rows
        head["contact_events"] = events
        head["final_success"] = genv.success()
        (OUT / f"{tag}_{genv.seed}.json").write_text(json.dumps(head, indent=0, default=str))
