"""RoboPRO task construction for the GAH backend.

make_task(task) builds a RoboPRO bench task object once (the curobo planners inside it are expensive to
warm up); reset(env, seed, variation) then re-creates the scene for one episode, applies the scene edits
and lets everything settle. The same object is reused across episodes in a worker process.

RoboPRO (github.com/EAI-RSM/RoboPRO) is a RoboTwin 2.0 fork: `customized_robotwin/` holds the simulator
and robot (aloha-agilex, curobo planner), `benchmark/` holds the bench tasks (bench_envs/<scene>/<task>.py)
and their configs (bench_task_config/). env.sh sets ROBOTWIN_ROOT (= customized_robotwin) and BENCH_ROOT.
RoboPRO uses paths relative to ROBOTWIN_ROOT, so the worker process chdirs there.
"""

from __future__ import annotations

import importlib
import os
import sys

import numpy as np
import yaml

ROOT = os.environ.get("ROBOTWIN_ROOT")
BENCH_ROOT = os.environ.get("BENCH_ROOT")
SCENES = ("kitchens", "kitchenl", "office", "study")
TASKS = ["put_spoon_on_plate_ks"]
TASK_CONFIG = {"put_spoon_on_plate_ks": "bench_demo_kitchens_clean"}   # clean scene: no clutter, fixed lights

# Per-episode budget of simulation steps (250 Hz physics), counted by api.RobotAPI.
# The expert of put_spoon_on_plate_ks needs ~1.5k steps; this leaves room for re-detection and retries.
STEP_LIMIT = int(os.environ.get("GAH_STEP_LIMIT", 30000))

TABLE_Z = 0.74

# Names of each task's main object (the grasp that counts for the success check).
MAIN_OBJECTS = {"spoon"}

# Fixed furniture: never treated as touchable just because the arm touches it (api.FIXED_SCENE).
FIXED_SCENE = {"table", "wall", "sink", "dishrack", "microwave"}


class StepLimitExceeded(Exception):
    pass


def chdir_root():
    if not ROOT or not BENCH_ROOT:
        raise RuntimeError("ROBOTWIN_ROOT / BENCH_ROOT are not set; source benchmarks/robopro/env.sh first")
    if os.getcwd() != ROOT:
        os.chdir(ROOT)
    for p in (BENCH_ROOT, ROOT):
        if p not in sys.path:
            sys.path.insert(0, p)
    os.environ.setdefault("ROBOTWIN_BENCH_TASK", "bench")


def _patch_curobo_cache(n_obb: int = 64):
    """Give every curobo MotionGen a collision cache big enough for the scene boxes api.py adds.
    RoboPRO tasks pass their own cache (e.g. {"mesh": 100, "obb": 3}), so raise the box cache, not just default
    it. api.py plans against boxes only, so the mesh cache (a large GPU buffer per MotionGen, four per task)
    is cut to GAH_CUROBO_MESH (default 2) unless RoboPRO's own planner world is needed (pool.py seeds)."""
    from curobo.wrap.reacher.motion_gen import MotionGenConfig
    if getattr(MotionGenConfig, "_gah_patched", False):
        return
    raw = MotionGenConfig.__dict__["load_from_robot_config"]
    is_static = isinstance(raw, staticmethod)
    orig = raw.__func__

    def load(*a, **kw):
        cc = dict(kw.get("collision_cache") or {})
        cc["obb"] = max(int(cc.get("obb", 0)), n_obb)
        cc["mesh"] = min(int(cc.get("mesh", 1) or 1), int(os.environ.get("GAH_CUROBO_MESH", 2)))
        kw["collision_cache"] = cc
        return orig(*a, **kw)

    if is_static:
        MotionGenConfig.load_from_robot_config = staticmethod(load)
    else:
        MotionGenConfig.load_from_robot_config = classmethod(lambda cls, *a, **kw: load(cls, *a, **kw))
    MotionGenConfig._gah_patched = True


def _patch_render_pools():
    """RoboPRO builds a new SAPIEN renderer on every reset with 50000-entry material/texture pools that are
    never freed, so a long-lived worker leaks VRAM every episode. The scenes need far fewer entries."""
    import sapien.render as sr
    if getattr(sr, "_gah_patched", False):
        return
    orig = sr.set_global_config
    cap = {"max_num_materials": int(os.environ.get("RT_MAX_MATERIALS", 4000)),
           "max_num_textures": int(os.environ.get("RT_MAX_TEXTURES", 4000))}

    def capped(*a, **kw):
        for k, v in cap.items():
            if k in kw:
                kw[k] = min(int(kw[k]), v)
        return orig(*a, **kw)

    sr.set_global_config = capped
    sr._gah_patched = True


def task_module(task: str):
    chdir_root()
    for name in [f"bench_envs.{task}"] + [f"bench_envs.{s}.{task}" for s in SCENES]:
        try:
            return importlib.import_module(name)
        except ModuleNotFoundError as e:
            if not str(e.name).startswith("bench_envs"):
                raise
    raise ModuleNotFoundError(f"no RoboPRO bench task {task!r}")


def task_args(task: str, config: str | None = None) -> dict:
    chdir_root()
    config = config or TASK_CONFIG[task]
    from envs._GLOBAL_CONFIGS import CONFIGS_PATH
    with open(os.path.join(BENCH_ROOT, "bench_task_config", f"{config}.yml"), "r", encoding="utf-8") as f:
        args = yaml.safe_load(f)
    with open(os.path.join(CONFIGS_PATH, "_embodiment_config.yml"), "r", encoding="utf-8") as f:
        embodiments = yaml.safe_load(f)
    with open(os.path.join(CONFIGS_PATH, "_camera_config.yml"), "r", encoding="utf-8") as f:
        cameras = yaml.safe_load(f)
    robot_file = embodiments[args["embodiment"][0]]["file_path"]
    with open(os.path.join(robot_file, "config.yml"), "r", encoding="utf-8") as f:
        emb_cfg = yaml.safe_load(f)
    head = args["camera"]["head_camera_type"]
    args.update(
        task_name=task, task_config=config,
        left_robot_file=robot_file, right_robot_file=robot_file, dual_arm_embodied=True,
        left_embodiment_config=emb_cfg, right_embodiment_config=emb_cfg,
        embodiment_name=str(args["embodiment"][0]),
        head_camera_h=cameras[head]["h"], head_camera_w=cameras[head]["w"],
        render_freq=0, save_data=False, need_plan=True, collect_data=False, eval_mode=False,
        eval_video_log=False, enable_collision_metrics=False,
        save_freq=None,  # no rendering inside RoboPRO's own motions
    )
    return args


def make_task(task: str):
    chdir_root()
    _patch_curobo_cache()
    _patch_render_pools()
    mod = task_module(task)
    env = getattr(mod, task)()
    # RoboPRO pushes its scene meshes into curobo on every reset (update_world). api.py replaces the planner
    # world with boxes before every plan, so episodes skip that; RoboPRO's own expert (seed picking) needs it.
    robopro_update_world = env.update_world

    def update_world(*a, **kw):
        if getattr(env, "_gah_robopro_world", False):
            return robopro_update_world(*a, **kw)
        return None

    env.update_world = update_world
    env._gah_robopro_world = False
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


def expert_solves(env, seed: int) -> bool:
    """RoboPRO's own scripted solution (play_once) on the unedited scene of `seed` (used to pick seeds).
    Needs RoboPRO's planner world, i.e. a task object built with GAH_CUROBO_MESH=100."""
    env._gah_robopro_world = True
    try:
        env.setup_demo(now_ep_num=0, seed=seed, is_test=True, **env._gah_args)
        env.play_once()
        return bool(env.plan_success and env.check_success())
    finally:
        env._gah_robopro_world = False


def success(env, api) -> bool:
    """The task's own check_success() (unchanged, hashed in tools/INTEGRITY.json)."""
    return bool(env.check_success())


def diagnostics(env) -> dict:
    """Ground-truth end state for the evaluation record (for the developer, like a camera frame;
    never visible to the pipeline)."""
    out = {}
    try:
        if env._gah_task == "put_spoon_on_plate_ks":
            s = np.asarray(env.target_obj.get_pose().p, float)
            p = np.asarray(env.des_obj.get_pose().p, float)
            out = {"scene_id": int(env.scene_id), "spoon_xyz": np.round(s, 3).tolist(),
                   "plate_xyz": np.round(p, 3).tolist(), "xy_err": round(float(np.linalg.norm(s[:2] - p[:2])), 3),
                   "grippers_open": [bool(env.robot.is_left_gripper_open()), bool(env.robot.is_right_gripper_open())]}
    except Exception as e:
        out = {"error": repr(e)}
    return out


# --------------------------------------------------------------------------- #
# Task spec: language names -> simulator bodies, parts, regions (used by api.RobotAPI and scene_edits)
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


def task_objects(env) -> dict:
    """Task objects by language name -> RoboTwin Actor (spoon is dynamic, plate is static)."""
    if env._gah_task == "put_spoon_on_plate_ks":
        return {"spoon": env.target_obj, "plate": env.des_obj}
    return {}


def set_task_object(env, name: str, actor):
    """Replace a task object (scene_edits swap_object / object_scale): the task's success check reads
    env.target_obj (spoon) and env.des_obj (plate) live."""
    if env._gah_task == "put_spoon_on_plate_ks":
        if name == "spoon":
            env.target_obj = actor
        elif name == "plate":
            env.des_obj = actor
            env.des_obj_pose = env.des_obj.get_pose().p.tolist() + [0, 0, 0, 1]
            env.des_obj_pose[2] += 0.02


def named_entities(env) -> dict:
    """Language name -> ("actor", entity) | ("articulation", PhysxArticulation) for scene_edits."""
    out = {n: ("actor", _entity(a)) for n, a in task_objects(env).items()}
    for n in ("table", "wall", "dishrack"):
        if hasattr(env, n):
            out[n] = ("actor", _entity(getattr(env, n)))
    if hasattr(env, "microwave"):
        out["microwave"] = ("articulation", env.microwave.actor)
    return out


def aliases(env) -> dict:
    out = {}
    for n, a in task_objects(env).items():
        model = _entity(a).get_name()           # e.g. 134_spoon
        out[model] = n
        out[model.split("_", 1)[-1]] = n
    return {k: v for k, v in out.items() if k != v}


def task_spec(env) -> dict:
    """bodies: [(owner, rigid component, group)]; aliases: {name: owner}; parts / regions:
    {name: () -> (world sapien.Pose of the box centre, full extents in that frame)};
    actors: {name: () -> (actor, frame(name) -> 4x4 world, [(frame name, local 4x4 contact matrix)])};
    fine: owners the planner sees per collision piece instead of as one box."""
    spec = {"bodies": [], "aliases": {}, "parts": {}, "regions": {}, "actors": {}, "part_owner": {},
            "fine": ["table", "sink"], "model_frame": {}}
    spec["bodies"].append(("table", _rigid(_entity(env.table)), "table"))
    spec["bodies"].append(("wall", _rigid(_entity(env.wall)), "wall"))
    # sink basin + chrome rims (separate static actors), dish rack, microwave links
    names = {"sink", "sink_rim_front", "sink_rim_back", "sink_rim_left", "sink_rim_right"}
    for e in env.scene.get_entities():
        if e.get_name() in names:
            comp = _rigid(e)
            if comp is not None and comp.get_collision_shapes():
                spec["bodies"].append(("sink", comp, "sink"))
    if hasattr(env, "dishrack"):
        spec["bodies"].append(("dishrack", _rigid(_entity(env.dishrack)), "dishrack"))
    if hasattr(env, "microwave"):
        for link in env.microwave.actor.get_links():
            if link.get_collision_shapes():
                spec["bodies"].append(("microwave", link, "microwave"))

    for name, act in task_objects(env).items():
        spec["bodies"].append((name, _rigid(_entity(act)), name))
        spec["model_frame"][name] = lambda a=act: _entity(a).get_pose()

        def frame(_, a=act):
            return _entity(a).get_pose().to_transformation_matrix()

        def provider(a=act, f=frame):
            return a, f, [("root", _scaled(m, a.config["scale"])) for m in a.config.get("contact_points_pose", [])]

        if act.config.get("contact_points_pose") and name in MAIN_OBJECTS:
            spec["actors"][name] = provider
    spec["aliases"].update(aliases(env))
    return spec


def close(env):
    try:
        env.close_env()
    except Exception:
        pass
