"""Dev diagnostic (simulator internals; never used by pipeline.py).
Runs a pipeline in-process on a few seeds and logs ground truth around the release:
python benchmarks/robodyna/runs/drop_ball_hole/dev/probe.py PIPELINE [seed_index ...] [--var YAML]
"""
import importlib.util
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("CUDA_VISIBLE_DEVICES", "0")
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa: E402

import _common as C  # noqa: E402
import gah_env  # noqa: E402
from api import RobotAPI  # noqa: E402

args = sys.argv[1:]
var = None
if "--var" in args:
    i = args.index("--var")
    var = C.load_variation(args[i + 1])
    del args[i:i + 2]
pipe_path = str(Path(args[0]).resolve())
idxs = [int(a) for a in args[1:] if not a.startswith("--")] or [0]
seeds = C.task_seeds("drop_ball_hole")
genv = gah_env.GahEnv("drop_ball_hole")
for k in idxs:
    seed = seeds[k]
    genv.reset(seed, var)
    env = genv.env
    api = RobotAPI(genv, noise_seed=10_000 + seed)
    spec = importlib.util.spec_from_file_location(f"p{k}", pipe_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    log = []
    robot = env.robot

    contacts = {}

    def cb(n):
        if "--contacts" in sys.argv and n % 10 == 0:
            for c in env.scene.get_contacts():
                a, b2 = c.bodies[0].entity.get_name(), c.bodies[1].entity.get_name()
                if "wall" in a or "wall" in b2 or "obstacle" in a or "obstacle" in b2:
                    if "bucket" in a or "bucket" in b2:
                        continue
                    key = tuple(sorted((a, b2)))
                    contacts.setdefault(key, []).append(round(n / 250, 2))
        bp = np.asarray(env.ball.get_pose().p)
        hole = env.cap_center[:2] + env._cap_rot2() @ env._cap_hole_local_xy
        try:
            held = len(env.get_gripper_actor_contact_position(env.ball.get_name())) > 0
        except Exception:
            held = None
        log.append(dict(t=n / 250, b=bp.round(4).tolist(), hole=hole.round(4).tolist(),
                        d=float(np.linalg.norm(bp[:2] - hole)), held=held, rel=bool(env.ball_released),
                        stuck=bool(env.ball_stuck_on_platform), inbox=bool(env._ball_in_box())))
    genv.step_callbacks.append(cb)
    err = None
    try:
        mod.run(api)
    except Exception as e:
        import traceback
        err = traceback.format_exc()
    genv.step_callbacks.clear()
    genv.finish()
    print(f"\n==== idx {k} seed {seed} success={genv.success()} fail='{genv.fail_reason()}' err={err}")
    print("true omega rad/s", env.spin_omega * 2.5, "ball_side", env.ball_side, "hole local", env._cap_hole_local_xy,
          "r_hole", env.cap_hole_radius, "cap_z", env.cap_z, "decoy local", env._dummy_hole_local_xy)
    print("DEBUG", json.dumps(getattr(mod, "DEBUG", {}), default=str))
    # first release / stuck / inbox
    for key in ("rel", "stuck", "inbox"):
        e = next((r for r in log if r[key]), None)
        print(key, None if e is None else e)
    tc = mod.DEBUG.get("t_cmd_actual")
    if tc is not None:
        seg = [r for r in log if tc - 0.005 <= r["t"] <= tc + 1.5]
        r0 = seg[0]
        free = next((r for r in seg if r["held"] is False and r["t"] > tc + 0.2), None)
        top = env.cap_z + 0.006 + env.ball_radius
        cross = next((r for r in seg if r["b"][2] < top), None)
        ee = np.asarray(robot.get_left_ee_pose() if env.ball_side == "left" else robot.get_right_ee_pose())
        print(f"  at cmd: b={r0['b']} d={r0['d']:.4f}; free at dt={free and round(free['t']-tc,3)} b={free and free['b']}")
        if cross:
            print(f"  plane cross dt={cross['t']-tc:.3f} b={cross['b']} hole={cross['hole']} d={cross['d']:.4f}"
                  f" shift_xy={np.round(np.subtract(cross['b'][:2], r0['b'][:2]),4).tolist()}")
        print("  ee quat at end", np.round(ee[3:], 3).tolist())
        if not genv.success() or "--steps" in sys.argv:
            for r in seg[::5]:
                if r["t"] - tc < 0.45:
                    continue
                print(f"   s dt={r['t']-tc:.3f} b={np.round(r['b'],4).tolist()} hole={r['hole']} d={r['d']:.4f} held={r['held']} stuck={r['stuck']} inbox={r['inbox']}")
    if "--trace" in sys.argv or not genv.success():
        for e in api.trace:
            print("   TR", e)
        for r in log[::125]:
            print(f"   t={r['t']:.2f} b={r['b']} held={r['held']} rel={r['rel']} stuck={r['stuck']}")
    for k, v in contacts.items():
        print("  CONTACT", k, "n=", len(v), "t", v[0], "-", v[-1])
    if "--fingers" in sys.argv:
        import transforms3d as t3d
        arm_ent = robot.left_entity if env.ball_side == "left" else robot.right_entity
        ee = np.asarray(robot.get_left_ee_pose() if env.ball_side == "left" else robot.get_right_ee_pose())
        tcp = np.asarray(robot.get_left_tcp_pose() if env.ball_side == "left" else robot.get_right_tcp_pose())
        R = t3d.quaternions.quat2mat(ee[3:])
        print("  ee", np.round(ee, 3).tolist(), "tcp", np.round(tcp, 3).tolist())
        for l in arm_ent.get_links():
            n = l.get_name()
            if any(k in n.lower() for k in ("finger", "pad", "tip", "gripper")):
                p = np.asarray(l.entity.get_pose().p)
                print("   link", n, np.round(p, 3).tolist(), "in ee frame", np.round(R.T @ (p - ee[:3]), 3).tolist())
    # gripper links lowest z at the end of descent
    sys.stdout.flush()
    genv.close()
