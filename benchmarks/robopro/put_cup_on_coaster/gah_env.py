"""RoboPRO task construction for the GAH backend.

make_task(task) builds a RoboPRO bench task object once per worker process (its curobo planners are
expensive to warm up); reset(env, seed, variation) then rebuilds the scene of one seed with RoboPRO's
own setup_demo(), applies the scene edits and lets everything settle.

RoboPRO = a RoboTwin 2.0 fork (customized_robotwin/) plus a benchmark of household scenes (benchmark/).
ROBOPRO_ROOT must point at customized_robotwin/ and BENCH_ROOT at benchmark/ (benchmarks/robopro/env.sh
sets both). RoboPRO uses paths relative to customized_robotwin/, so the worker process chdirs there.
"""

from __future__ import annotations

import importlib
import os
import sys

import numpy as np
import yaml

ROOT = os.environ.get("ROBOPRO_ROOT")
# task -> (RoboPRO scene, bench task config). The clean config has no clutter (obstacle_density 0).
TASK_INFO = {"put_cup_on_coaster": ("study", "bench_demo_study_clean")}
TASKS = list(TASK_INFO)

# Per-episode budget of simulation steps (250 Hz physics), counted by api.RobotAPI.
STEP_LIMIT = int(os.environ.get("GAH_STEP_LIMIT", 30000))

TABLE_Z = 0.74

# Names of each task's main object (the grasp that counts for the success check).
MAIN_OBJECTS = {"cup"}


class StepLimitExceeded(Exception):
    pass


def chdir_root():
    if not ROOT:
        raise RuntimeError("ROBOPRO_ROOT is not set; source benchmarks/robopro/env.sh first")
    os.environ.setdefault("BENCH_ROOT", os.path.join(os.path.dirname(ROOT), "benchmark"))
    os.environ["ROBOTWIN_BENCH_TASK"] = "bench"
    if os.getcwd() != ROOT:
        os.chdir(ROOT)
    for p in (ROOT, os.path.dirname(ROOT)):
        if p not in sys.path:
            sys.path.insert(0, p)
    from script.bench_script.setup_paths import setup_paths
    setup_paths()


def _patch_curobo_cache(n_obb: int = 128, n_mesh: int = 100):
    """Give every curobo MotionGen a collision cache large enough for api.py's obstacle boxes.
    RoboPRO tasks ask for tiny caches (put_cup_on_coaster: 3 boxes), and the planner is built once,
    with the first episode's cache size."""
    from curobo.wrap.reacher.motion_gen import MotionGenConfig
    if getattr(MotionGenConfig, "_gah_patched", False):
        return
    raw = MotionGenConfig.__dict__["load_from_robot_config"]
    is_static = isinstance(raw, staticmethod)
    orig = raw.__func__

    def load(*a, **kw):
        cache = dict(kw.get("collision_cache") or {})
        cache["obb"] = max(int(cache.get("obb", 0)), n_obb)
        cache["mesh"] = max(int(cache.get("mesh", 0)), n_mesh)
        kw["collision_cache"] = cache
        return orig(*a, **kw)

    if is_static:
        MotionGenConfig.load_from_robot_config = staticmethod(load)
    else:
        MotionGenConfig.load_from_robot_config = classmethod(lambda cls, *a, **kw: load(cls, *a, **kw))
    MotionGenConfig._gah_patched = True


def _patch_render_pools():
    """RoboPRO builds a new SAPIEN renderer on every reset with 50000-entry material/texture pools that are
    never freed, so a persistent worker leaks ~1 GB of GPU memory per episode (same leak as RoboTwin,
    see the RoboTwin backend's ISSUES). Cap the pools; the task scenes use far fewer."""
    import sapien.render as sr
    if getattr(sr, "_gah_pool_patch", False):
        return
    orig = sr.set_global_config
    cap_m = int(os.environ.get("RT_MAX_MATERIALS", 4000))
    cap_t = int(os.environ.get("RT_MAX_TEXTURES", 4000))

    def capped(*a, **kw):
        if "max_num_materials" in kw:
            kw["max_num_materials"] = min(int(kw["max_num_materials"]), cap_m)
        if "max_num_textures" in kw:
            kw["max_num_textures"] = min(int(kw["max_num_textures"]), cap_t)
        return orig(*a, **kw)

    sr.set_global_config = capped
    sr._gah_pool_patch = True


def task_module(task: str):
    chdir_root()
    scene, _ = TASK_INFO[task]
    return importlib.import_module(f"bench_envs.{scene}.{task}")


def task_args(task: str) -> dict:
    chdir_root()
    from envs._GLOBAL_CONFIGS import CONFIGS_PATH
    _, config = TASK_INFO[task]
    with open(os.path.join(os.environ["BENCH_ROOT"], "bench_task_config", f"{config}.yml"), "r", encoding="utf-8") as f:
        args = yaml.safe_load(f)
    with open(os.path.join(CONFIGS_PATH, "_embodiment_config.yml"), "r", encoding="utf-8") as f:
        embodiments = yaml.safe_load(f)
    with open(os.path.join(CONFIGS_PATH, "_camera_config.yml"), "r", encoding="utf-8") as f:
        cameras = yaml.safe_load(f)
    robot_file = embodiments[args["embodiment"][0]]["file_path"]
    with open(os.path.join(robot_file, "config.yml"), "r", encoding="utf-8") as f:
        emb_cfg = yaml.safe_load(f)
    head = args["camera"]["head_camera_type"]
    data_type = dict(args.get("data_type") or {})
    data_type["proximity"] = False               # per-step clearance tracking is eval instrumentation
    args.update(
        task_name=task, task_config=config,
        left_robot_file=robot_file, right_robot_file=robot_file, dual_arm_embodied=True,
        left_embodiment_config=emb_cfg, right_embodiment_config=emb_cfg,
        embodiment_name=str(args["embodiment"][0]),
        head_camera_h=cameras[head]["h"], head_camera_w=cameras[head]["w"],
        render_freq=0, save_data=False, need_plan=True, collect_data=False, eval_mode=False,
        eval_video_log=False, enable_collision_metrics=False, data_type=data_type,
        save_freq=None,  # no rendering inside RoboPRO's own motions
    )
    return args


def make_task(task: str):
    chdir_root()
    _patch_curobo_cache()
    _patch_render_pools()
    mod = task_module(task)
    env = getattr(mod, task)()
    env._gah_task = task
    env._gah_args = task_args(task)
    env._gah_episodes = 0
    return env


def reset(env, seed: int, variation: dict | None = None):
    """Set up one episode: RoboPRO scene for `seed`, then the variation's scene edits.
    Raises envs.utils.create_actor.UnStableError if the seed's scene is unstable."""
    import scene_edits
    if env._gah_episodes and env._gah_episodes % 5 == 0:
        from sapien.render import clear_cache
        clear_cache()
    env._gah_episodes += 1
    env.setup_demo(now_ep_num=0, seed=seed, is_test=True, **env._gah_args)
    env.plan_success = True
    edits = (variation or {}).get("edits") or {}
    state = scene_edits.apply(env, edits, rng=np.random.default_rng(seed))
    env._gah_edit_state = state
    return env


def expert(env) -> bool:
    """RoboPRO's scripted expert (play_once) on the current scene; True if it solves the task.
    Used only to pick solvable seeds (tools/pool.py seeds), never by pipelines."""
    env.play_once()
    return bool(env.plan_success and env.check_success())


def success(env, api) -> bool:
    """The task's own check_success() (RoboPRO's code, unchanged and hashed)."""
    return bool(env.check_success())


def diagnostics(env) -> dict:
    """Ground-truth end state for the evaluation record (for the developer, like a camera frame;
    never visible to the pipeline)."""
    out = {}
    try:
        if env._gah_task == "put_cup_on_coaster":
            cup = np.array(env.target_obj.get_pose().p, float)
            coaster = np.array(env.des_obj.get_pose().p, float)
            out = {"cup_model": f"{env.target_name}/{env.target_id}", "scene_id": int(env.scene_id),
                   "expert_arm": str(env.side_to_place),
                   "cup_xyz": np.round(cup, 3).tolist(), "coaster_xyz": np.round(coaster, 3).tolist(),
                   "dxy": np.round(cup[:2] - coaster[:2], 3).tolist(),
                   "grippers_open": [bool(env.robot.is_left_gripper_open()), bool(env.robot.is_right_gripper_open())]}
    except Exception as e:
        out = {"error": repr(e)}
    return out


# --------------------------------------------------------------------------- #
# Task spec: language names -> simulator bodies, parts, regions (used by api.RobotAPI)
# --------------------------------------------------------------------------- #
def _entity(x):
    return x.actor if hasattr(x, "actor") else x


def _rigid(entity):
    import sapien
    return entity.find_component_by_type(sapien.physx.PhysxRigidBaseComponent)


def _scaled(m, scale):
    m = np.array(m, dtype=np.float64)
    m[:3, 3] *= np.array(scale, dtype=np.float64)
    return m


# furniture of the study scene (static); the planner sees them per convex piece
FURNITURE = {"bookcase": "bookcase", "wooden_box": "box"}


def task_spec(env) -> dict:
    """bodies: [(owner, rigid component, group)]; aliases: {name: owner}; parts / regions:
    {name: () -> (world sapien.Pose of the box centre, full extents in that frame)};
    actors: {name: () -> (actor, frame(name) -> 4x4 world, [(frame name, local 4x4 contact matrix)])};
    fine: owners the planner sees per convex piece instead of as one box."""
    import sapien
    spec = {"bodies": [], "aliases": {}, "parts": {}, "regions": {}, "actors": {}, "fine": ["table"],
            "part_owner": {}}
    known = set()
    for _, comp in (getattr(env, "_gah_edit_state", None) or {}).get("added", []):
        known.add(comp.entity.per_scene_id)          # added bodies are registered by api.RobotAPI itself

    def add(owner, ent, group=None):
        comp = _rigid(_entity(ent))
        if comp is None or not comp.get_collision_shapes():
            return
        spec["bodies"].append((owner, comp, group or owner))
        known.add(comp.entity.per_scene_id)

    add("table", env.table)
    add("wall", env.wall)
    for owner, attr in FURNITURE.items():
        if hasattr(env, attr):
            add(owner, getattr(env, attr))
            spec["fine"].append(owner)
    spec["aliases"].update({"014_bookcase": "bookcase", "042_wooden_box": "wooden_box", "box": "wooden_box"})

    if env._gah_task == "put_cup_on_coaster":
        add("cup", env.target_obj)
        add("coaster", env.des_obj)
        spec["aliases"].update({"021_cup": "cup", "019_coaster": "coaster", "object": "cup"})

        def cup_frame(_):
            return env.target_obj.actor.get_pose().to_transformation_matrix()

        spec["actors"]["cup"] = lambda: (env.target_obj, cup_frame,
                                         [("root", _scaled(m, env.target_obj.config["scale"]))
                                          for m in env.target_obj.config.get("contact_points_pose", [])])

        from api import _shape_local_boxes, _union_box

        def coaster_top():
            """The space just above the coaster: its footprint, from its top face up 12 cm."""
            comp = _rigid(env.des_obj.actor)
            lp, hs = _union_box(_shape_local_boxes(comp))
            T = comp.entity.get_pose() * lp
            R = T.to_transformation_matrix()[:3, :3]
            ext = np.abs(R) @ (2 * hs)                     # world-axis extents of the coaster
            c = np.array(T.p, float)
            c[2] += ext[2] / 2 + 0.06
            return sapien.Pose(c), np.array([ext[0], ext[1], 0.12])

        spec["regions"]["coaster_top"] = coaster_top
        spec["part_owner"]["coaster_top"] = "coaster"

    # every other named rigid body in the scene (RoboPRO clutter, if a config adds any)
    robot_ids = set()
    for ent in (env.robot.left_entity, env.robot.right_entity):
        for link in ent.get_links():
            robot_ids.add(link.entity.per_scene_id)
    n = 0
    for ent in env.scene.get_entities():
        if ent.per_scene_id in known or ent.per_scene_id in robot_ids:
            continue
        name = ent.get_name()
        if not name or name in ("ground", "floor") or name.startswith(("camera", "light")):
            continue
        comp = _rigid(ent)
        if comp is None or not comp.get_collision_shapes():
            continue
        if isinstance(comp, sapien.physx.PhysxArticulationLinkComponent):
            continue
        owner = f"clutter_{n}"
        n += 1
        spec["bodies"].append((owner, comp, owner))
        spec["aliases"][name] = owner
        known.add(ent.per_scene_id)
    return spec


def close(env):
    try:
        env.close_env()
    except Exception:
        pass
