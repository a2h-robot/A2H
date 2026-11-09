"""DIAGNOSTIC: box physics enabled as in interactive mode; step-push and log fingers/box."""
import json
from pathlib import Path
import numpy as np


def run(api):
    env = api._env
    rig = env._get_rigid(env.bowl)
    groups0 = [list(s.get_collision_groups()) for s in rig.get_collision_shapes()][:1]
    env._enable_box_physics()
    env._push_active = True
    side = float(env.side)
    arm = "right" if side > 0 else "left"
    log = []

    def snap(tag):
        log.append({"tag": tag, "t": round(api.time(), 2), "kin": bool(rig.kinematic),
                    "box": np.round(env.bowl.get_pose().p, 4).tolist(),
                    "tcp": np.round(api.tcp_pose(arm), 4).tolist()[:3], "ee": np.round(api.ee_pose(arm), 4).tolist()[:3],
                    "drop": round(env._finger_drop(arm), 4), "open": round(api.gripper_opening(arm), 3)})

    box = env.bowl.get_pose().p
    q0 = np.asarray(api.ee_pose(arm)[3:])
    r = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0])
    w1, x1, y1, z1 = q0
    w2, x2, y2, z2 = r
    q = [w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
         w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2]
    snap("start")
    try:
        groups = [list(s.get_collision_groups()) for s in rig.get_collision_shapes()][:1]
        lk = [l for l in (env.robot.right_entity if arm == "right" else env.robot.left_entity).get_links()
              if "finger" in l.get_name().lower()]
        fg = [[list(s.get_collision_groups()) for s in l.get_collision_shapes()][:1] for l in lk]
        log.append({"box_groups_orig": groups0, "box_groups_now": groups, "finger_groups": fg,
                    "finger_links": [l.get_name() for l in lk]})
    except Exception as e:
        log.append({"groups_err": repr(e)})
    api.move_to([box[0], box[1] - 0.13, 0.955, *q], arm)
    snap("hover")
    api.move_to([box[0], box[1] - 0.13, 0.92, *q], arm)
    snap("down")
    for i in range(6):
        api.move_to([box[0], box[1] - 0.13 + 0.03 * (i + 1), 0.92, *q], arm, hold_orientation=True)
        snap(f"push{i}")
    d = Path(__file__).resolve().parent
    (d / f"probe4_{api._genv.seed}.json").write_text(json.dumps(log, indent=1))
