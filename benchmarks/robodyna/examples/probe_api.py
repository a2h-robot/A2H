"""Backend probe used for SANITY.md: per task, the named objects and parts, the camera, gripper
calibration on ur5-wsg and the planner-world frame. One GPU.
python benchmarks/robodyna/examples/probe_api.py [task ...]"""
import sys
import time
from pathlib import Path

BENCH = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa: E402

import _common as C  # noqa: E402
import gah_env  # noqa: E402
from api import Box, RobotAPI  # noqa: E402

tasks = sys.argv[1:] or list(C.GAH_TASKS)
for task in tasks:
    print(f"\n===== {task} ({gah_env.SCENARIO})", flush=True)
    genv = gah_env.GahEnv(task)
    seed = C.task_seeds(task)[0]
    t = time.time()
    genv.reset(seed)
    print("reset", round(time.time() - t, 1), "s  seed", seed)
    api = RobotAPI(genv, noise_seed=0, pos_noise=0.0, rot_noise_deg=0.0)
    for name in api.objects() + api.parts():
        s = api.detect(name)
        print(f"  {name:16s} pos={np.round(s.position, 3).tolist()} ext={np.round(s.bbox_extents, 3).tolist()} "
              f"rgb={None if s.rgb is None else np.round(s.rgb, 2).tolist()} points={list(s.points)[:4]}")
    img = api.camera_frame()
    print("  frame", img.shape, img.dtype, round(float(img.mean()), 1))
    for arm in ("left", "right"):
        print(f"  {arm} ee", np.round(api.ee_pose(arm), 3).tolist())
    api.close_gripper("right")
    print("  closed-empty opening", round(api.gripper_opening("right"), 4), "holding", api.gripper_holding("right"))
    api.open_gripper("right")
    print("  open opening", round(api.gripper_opening("right"), 4))
    ee = api.ee_pose("right")
    tgt = list(ee)
    tgt[2] -= 0.08
    api._set_world("right", [Box("probe", np.array(tgt[:3]), np.array([1., 0, 0, 0]), np.array([0.08, 0.08, 0.08]))])
    print("  plan into box ->", "BLOCKED" if api._plan("right", tgt) is None else "planned (box ignored!)")
    api._set_world("right", [Box("probe", np.array([0.0, 0.3, 1.3]), np.array([1., 0, 0, 0]), np.array([0.05] * 3))])
    print("  plan with far box ->", "ok" if api._plan("right", tgt) is not None else "FAIL")
    api._set_world("right", [])
    print("  sim time", round(api.time(), 2), "s; success latched:", genv.success(), "| fail reason:", genv.fail_reason())
    genv.close()
