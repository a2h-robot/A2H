"""Diagnostic probe (uses internals; not a pipeline)."""
import json
from pathlib import Path
import numpy as np


def run(api):
    env = api._env
    out = {"seed": api._genv.seed}
    rig = env._get_rigid(env.bowl)
    out["box_kinematic"] = bool(rig.kinematic) if rig is not None else None
    for k in ["side", "mirrored", "red_line_x", "ramp_exit_x", "ramp_center_y", "ramp_half_width", "roll_time",
              "drop_time", "launch_speed", "up_angle", "bowl_inner_radius", "bowl_outer_radius", "wall_bounce_enabled",
              "enable_distractor"]:
        v = getattr(env, k, None)
        out[k] = float(v) if v is not None else None
    for k in ["landing", "ball_exit", "ball_start", "distractor_landing", "distractor_exit", "release_velocity"]:
        v = getattr(env, k, None)
        out[k] = None if v is None else np.round(np.asarray(v, float), 4).tolist()
    out["objects"] = api.objects()
    out["parts"] = api.parts()
    det = {}
    for n in api.objects() + api.parts():
        s = api.detect(n)
        det[n] = {"p": np.round(s.position, 3).tolist(), "ext": np.round(s.bbox_extents, 3).tolist(),
                  "c": np.round(s.bbox_center, 3).tolist()}
    out["det"] = det
    out["ee"] = {a: np.round(api.ee_pose(a), 3).tolist() for a in ("left", "right")}
    out["tcp"] = {a: np.round(api.tcp_pose(a), 3).tolist() for a in ("left", "right")}
    traj = []
    for i in range(90):
        traj.append([round(api.time(), 3), *np.round(env.ball.get_pose().p, 4).tolist(),
                     *np.round(env.distractor.get_pose().p, 4).tolist(), env._ball_phase])
        api.wait(0.1)
    out["traj"] = traj
    d = Path("benchmarks/robodyna/runs/catch_valley_ball/dev")
    d = Path(__file__).resolve().parent
    (d / f"probe1_{api._genv.seed}.json").write_text(json.dumps(out, indent=1, default=str))
