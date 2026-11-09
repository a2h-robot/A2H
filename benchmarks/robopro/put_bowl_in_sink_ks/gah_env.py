"""RoboPRO task construction for the GAH backend.

RoboPRO is a RoboTwin 2.0 fork (same aloha-agilex robot and curobo planner) with kitchen, office and study
benchmark scenes: task classes in <bench>/bench_envs/<scene>/<task>.py, configs in <bench>/bench_task_config.
make_task(task) builds a task object once per worker (the curobo planners take ~20 s to warm up);
reset(env, seed, variation) re-creates the scene for one episode, applies the scene edits and lets
everything settle. The same object is reused across episodes in a worker process.

ROBOTWIN_ROOT must point at RoboPRO's customized_robotwin/ and BENCH_ROOT at its benchmark/ (the install's
env.sh sets both). RoboPRO uses paths relative to ROBOTWIN_ROOT, so the worker process chdirs there.

Backend fixes to RoboPRO's scene setup (the task's success check is untouched; see ISSUES.md):
- pick_boxdrink_from_basket spawns the box drink at the basket origin, 2.2 cm inside the basket floor.
  PhysX on aarch64 ejects it (it lands on the floor or table); the backend lifts the spawn by SPAWN_LIFT.
- RoboPRO sizes the renderer pools at 50000 materials/textures and builds a new renderer on every reset,
  which leaks GPU memory per episode; the pools are capped at GAH_RT_MAX.
- The task asks curobo for a 3-cuboid collision cache; the API's planner world needs more (GAH_CUROBO_OBB).
"""

from __future__ import annotations

import importlib
import os
import sys

import numpy as np
import yaml

ROOT = os.environ.get("ROBOTWIN_ROOT")
BENCH_ROOT = os.environ.get("BENCH_ROOT")

# task -> (RoboPRO scene family, benchmark task config)
TASKS = {"pick_boxdrink_from_basket": ("kitchenl", "bench_demo_kitchenl_clean"),
         "place_bowl_in_dishrack_ks": ("kitchens", "bench_demo_kitchens_clean"),
         "put_bowl_in_sink_ks": ("kitchens", "bench_demo_kitchens_clean")}
BOWL_TASKS = {"place_bowl_in_dishrack_ks", "put_bowl_in_sink_ks"}

# Per-episode budget of simulation steps (250 Hz physics), counted by api.RobotAPI.
# The expert of pick_boxdrink_from_basket needs ~3-5k steps; this leaves room for re-detection and retries.
STEP_LIMIT = int(os.environ.get("GAH_STEP_LIMIT", 30000))

# Names of each task's main object (the grasp that counts for the success check).
MAIN_OBJECTS = {"boxdrink", "bowl"}

# Spawn lift [m] applied to the task object before the first physics step (see module docstring).
SPAWN_LIFT = {"pick_boxdrink_from_basket": 0.03}

# Container geometry for the planner: a basket is a floor slab plus four walls, not one solid box.
BASKET_FLOOR = 0.025       # m, measured: the floor top is 2.2 cm above the basket's lowest point
BASKET_WALL = 0.015        # m


class StepLimitExceeded(Exception):
    pass


def chdir_root():
    if not ROOT or not BENCH_ROOT:
        raise RuntimeError("ROBOTWIN_ROOT / BENCH_ROOT are not set; source the RoboPRO env.sh first")
    if os.getcwd() != ROOT:
        os.chdir(ROOT)
    for p in (os.path.join(ROOT, "description/utils"), BENCH_ROOT, ROOT):
        if p not in sys.path:
            sys.path.insert(0, p)
    os.environ.setdefault("ROBOTWIN_BENCH_TASK", "bench")


def _patch_render_pools(n: int = int(os.environ.get("GAH_RT_MAX", 4000))):
    import sapien.render as R
    if getattr(R, "_gah_patched", False):
        return
    orig = R.set_global_config

    def capped(*a, **kw):
        kw["max_num_materials"] = min(int(kw.get("max_num_materials", n)), n)
        kw["max_num_textures"] = min(int(kw.get("max_num_textures", n)), n)
        return orig(*a, **kw)

    R.set_global_config = capped
    R._gah_patched = True


def _patch_curobo_cache(n_obb: int = int(os.environ.get("GAH_CUROBO_OBB", 96))):
    """Give every curobo MotionGen a collision cache large enough for the API's planner world."""
    from curobo.wrap.reacher.motion_gen import MotionGenConfig
    if getattr(MotionGenConfig, "_gah_patched", False):
        return
    raw = MotionGenConfig.__dict__["load_from_robot_config"]
    is_static = isinstance(raw, staticmethod)
    orig = raw.__func__

    def load(*a, **kw):
        cache = dict(kw.get("collision_cache") or {})
        cache["obb"] = max(int(cache.get("obb", 0)), n_obb)
        kw["collision_cache"] = cache
        return orig(*a, **kw)

    if is_static:
        MotionGenConfig.load_from_robot_config = staticmethod(load)
    else:
        MotionGenConfig.load_from_robot_config = classmethod(lambda cls, *a, **kw: load(cls, *a, **kw))
    MotionGenConfig._gah_patched = True


def task_args(task: str) -> dict:
    chdir_root()
    from envs import CONFIGS_PATH
    _, config = TASKS[task]
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
        task_name=task, task_config=config, policy_name="gah",
        left_robot_file=robot_file, right_robot_file=robot_file, dual_arm_embodied=True,
        left_embodiment_config=emb_cfg, right_embodiment_config=emb_cfg,
        embodiment_name=str(args["embodiment"][0]),
        head_camera_h=cameras[head]["h"], head_camera_w=cameras[head]["w"],
        render_freq=0, save_data=False, need_plan=True, collect_data=False, eval_mode=False,
        save_freq=None, eval_video_log=False, enable_collision_metrics=False,
    )
    args["data_type"] = dict(args.get("data_type") or {}, proximity=False)   # no per-step clearance tracking
    return args


def _task_class(task):
    scene, _ = TASKS[task]
    cls = getattr(importlib.import_module(f"bench_envs.{scene}.{task}"), task)
    if task in SPAWN_LIFT and not getattr(cls, "_gah_spawn_fixed", False):
        lift = SPAWN_LIFT[task]
        orig_load = cls.load_actors

        def load_actors(self):
            import sapien
            orig_load(self)
            obj = main_actor(self)
            p = obj.get_pose()
            obj.actor.set_pose(sapien.Pose(np.asarray(p.p, float) + [0.0, 0.0, lift], p.q))

        cls.load_actors = load_actors
        cls._gah_spawn_fixed = True
    return cls


def make_task(task: str):
    if task not in TASKS:
        raise ValueError(f"unknown task {task!r}; known: {sorted(TASKS)}")
    chdir_root()
    _patch_render_pools()
    _patch_curobo_cache()
    env = _task_class(task)()
    env._gah_task = task
    env._gah_args = task_args(task)
    env._gah_episodes = 0
    return env


def main_actor(env):
    """The task object (a RoboTwin Actor) whose grasp and end pose the success check reads."""
    if env._gah_task == "pick_boxdrink_from_basket":
        return env.boxdrink
    if env._gah_task in BOWL_TASKS:
        return env.target_obj
    raise ValueError("no main actor for this task")


def table_top(env) -> float:
    from bench_envs.utils.scene_gen_utils import get_actor_boundingbox
    return float(get_actor_boundingbox(env.table)[1][2])


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
    env._gah_table_z = table_top(env)
    edits = (variation or {}).get("edits") or {}
    env._gah_edit_state = scene_edits.apply(env, edits, rng=np.random.default_rng(seed))
    if env._gah_task == "put_bowl_in_sink_ks":
        # check_success compares the bowl with its start position, which only the scripted expert
        # (play_once) records; record it here, at the start of every episode, as play_once does
        env._target_start_p = np.asarray(env.target_obj.get_pose().p, dtype=float).copy()
    return env


def success(env, api) -> bool:
    """The task's own check_success()."""
    return bool(env.check_success())


def expert(env) -> bool:
    """Run RoboPRO's scripted expert (play_once) on the current scene; True if the task succeeds."""
    try:
        env.play_once()
    except Exception:
        return False
    return bool(env.plan_success) and bool(env.check_success())


def diagnostics(env) -> dict:
    """Ground-truth end state for the evaluation record (for the developer, like a camera frame;
    never visible to the pipeline)."""
    out = {}
    try:
        if env._gah_task == "pick_boxdrink_from_basket":
            b = env.boxdrink.get_pose()
            up = np.asarray(b.to_transformation_matrix())[:3, 1]      # the box's long axis (model y)
            out = {"scene_id": int(getattr(env, "scene_id", -1)),
                   "box_xyz": np.round(b.p, 3).tolist(),
                   "box_upright_cos": round(float(up[2]), 3),
                   "box_above_table_m": round(float(b.p[2] - env._gah_table_z), 3),
                   "box_in_basket_xy": bool(env._is_boxdrink_inside_basket()),
                   "basket_xyz": np.round(env.basket_right.get_pose().p, 3).tolist()}
        elif env._gah_task == "place_bowl_in_dishrack_ks":
            b = env.target_obj.get_pose()
            up = np.asarray(b.to_transformation_matrix())[:3, :3] @ BOWL_UP
            c = np.asarray(env.dishrack_center, float)
            out = {"scene_id": int(getattr(env, "scene_id", -1)),
                   "bowl_xyz": np.round(b.p, 3).tolist(),
                   "bowl_upright_cos": round(float(up[2]), 3),
                   "bowl_axis": np.round(up, 3).tolist(),
                   "bowl_minus_rack_xyz": np.round(np.asarray(b.p) - c, 3).tolist(),
                   "rack_center": np.round(c, 3).tolist()}
        elif env._gah_task == "put_bowl_in_sink_ks":
            b = env.target_obj.get_pose()
            up = np.asarray(b.to_transformation_matrix())[:3, :3] @ BOWL_UP
            sp = np.asarray(env.sink.get_pose().p, float)
            sg = env.kitchens_info["sink_geom"]
            d = np.asarray(b.p) - sp
            start = getattr(env, "_target_start_p", None)
            out = {"scene_id": int(getattr(env, "scene_id", -1)),
                   "bowl_xyz": np.round(b.p, 3).tolist(),
                   "bowl_upright_cos": round(float(up[2]), 3),
                   "bowl_minus_sink_xyz": np.round(d, 3).tolist(),
                   "in_sink_xy": bool(abs(d[0]) < sg["hole_hx"] and abs(d[1]) < sg["hole_hy"]),
                   "below_rim": bool(d[2] < 0.02),
                   "moved_m": None if start is None else round(float(np.linalg.norm(np.asarray(b.p) - start)), 3)}
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


def _mesh_world_aabb(comp):
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    T = comp.entity.get_pose()
    for s in comp.get_collision_shapes():
        v = np.asarray(s.get_vertices(), float) * np.asarray(s.get_scale(), float)
        M = (T * s.get_local_pose()).to_transformation_matrix()
        w = v @ M[:3, :3].T + M[:3, 3]
        lo, hi = np.minimum(lo, w.min(0)), np.maximum(hi, w.max(0))
    return lo, hi


def _container_boxes(comp, floor=BASKET_FLOOR, wall=BASKET_WALL):
    """[(pose in the body frame, half size)] of a floor slab and four walls (world-axis aligned) that
    replace the solid bounding box of a static container mesh, so the planner can reach inside."""
    import sapien
    lo, hi = _mesh_world_aabb(comp)
    boxes_w = [
        (lo[0], hi[0], lo[1], hi[1], lo[2], lo[2] + floor),        # floor
        (lo[0], lo[0] + wall, lo[1], hi[1], lo[2], hi[2]),          # -x wall
        (hi[0] - wall, hi[0], lo[1], hi[1], lo[2], hi[2]),          # +x wall
        (lo[0], hi[0], lo[1], lo[1] + wall, lo[2], hi[2]),          # -y wall
        (lo[0], hi[0], hi[1] - wall, hi[1], lo[2], hi[2]),          # +y wall
    ]
    Tinv = comp.entity.get_pose().inv()
    out = []
    for x0, x1, y0, y1, z0, z1 in boxes_w:
        c = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        out.append((Tinv * sapien.Pose(c), np.array([(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2])))
    return out


def _container_interior(comp, floor=BASKET_FLOOR, wall=BASKET_WALL):
    import sapien
    lo, hi = _mesh_world_aabb(comp)
    lo_i = lo + [wall, wall, floor]
    hi_i = hi - [wall, wall, 0.0]
    return sapien.Pose((lo_i + hi_i) / 2), hi_i - lo_i


FURNITURE = (("microwave_left", "microwave"), ("fridge_left", "fridge"), ("cabinet", "cabinet"),
             ("microwave", "microwave"))

# The bowl's opening axis in its model frame (002_bowl is modelled y-up).
BOWL_UP = np.array([0.0, 1.0, 0.0])


def named_entities(env) -> dict:
    """Canonical name -> ("actor", entity) | ("articulation", PhysxArticulation) for scene edits."""
    out = {"table": ("actor", _entity(env.table)), "wall": ("actor", _entity(env.wall))}
    if env._gah_task == "pick_boxdrink_from_basket":
        out["boxdrink"] = ("actor", env.boxdrink.actor)
        out["basket"] = ("actor", _entity(env.basket_right))
    if env._gah_task in BOWL_TASKS:
        out["bowl"] = ("actor", env.target_obj.actor)
        out["dishrack"] = ("actor", _entity(env.dishrack))
        out["sink"] = ("actor", _entity(env.sink))
    for attr, name in FURNITURE:
        a = getattr(env, attr, None)
        if a is not None:
            out[name] = ("articulation", _entity(a))
    return out


ALIASES = {"pick_boxdrink_from_basket": {
    "boxdrink": ["box_drink", "box drink", "drink", "drink box", "juice box", "task_boxdrink", "068_boxdrink",
                 "object"],
    "basket": ["basket_right", "tabletrashbin", "063_tabletrashbin", "bin"],
}, **{t: {
    "bowl": ["002_bowl", "object", "dish"],
    "dishrack": ["dish_rack", "dish rack", "rack", "135_dish-rack", "drying rack"],
    "utensil_holder": ["utensil holder", "utensil cup", "cutlery holder", "cutlery cup"],
    "sink": ["basin", "sink basin", "kitchen sink"],
} for t in BOWL_TASKS}}


def aliases(env) -> dict:
    out = {}
    for canon, names in ALIASES.get(env._gah_task, {}).items():
        for n in names:
            out[n] = canon
    return out


def task_spec(env) -> dict:
    """bodies: [(owner, rigid component, group)]; aliases: {name: owner}; parts / regions:
    {name: () -> (world sapien.Pose of the box centre, full extents in that frame)};
    actors: {name: () -> (actor, frame(name) -> 4x4 world, [(frame name, local 4x4 contact matrix)])};
    fine: owners the planner sees per box instead of as one box; local_boxes: owner -> [(component,
    [(pose in the body frame, half size)])] replacing the collision-shape boxes (containers)."""
    spec = {"bodies": [], "aliases": aliases(env), "parts": {}, "regions": {}, "actors": {}, "fine": ["table"],
            "part_owner": {}, "local_boxes": {}}
    spec["bodies"].append(("table", _rigid(_entity(env.table)), "table"))
    spec["bodies"].append(("wall", _rigid(_entity(env.wall)), "wall"))

    # furniture articulations (fridge, microwave, wall cabinet): one box per link
    robot_ids = {l.entity.per_scene_id for e in (env.robot.left_entity, env.robot.right_entity) for l in e.get_links()}
    names = {}
    for attr, name in FURNITURE:
        a = getattr(env, attr, None)
        if a is not None:
            names[_entity(a).get_links()[0].entity.per_scene_id] = name
    for art in env.scene.get_all_articulations():
        links = art.get_links()
        if not links or links[0].entity.per_scene_id in robot_ids:
            continue
        owner = names.get(links[0].entity.per_scene_id, art.get_name())
        for link in links:
            if link.get_collision_shapes():
                spec["bodies"].append((owner, link, owner))

    # other static scene bodies (e.g. the filler panel behind the wall cabinet)
    skip = {"ground", "table", "wall", "task_boxdrink", "basket_right", "002_bowl", "dishrack", "135_dish-rack"}
    for ent in env.scene.get_all_actors():
        comp = _rigid(ent)
        if ent.get_name() in skip or comp is None or not comp.get_collision_shapes():
            continue
        if ent.get_name().startswith(("distractor", "obstacle")) or getattr(ent, "_gah_added", False):
            continue
        spec["bodies"].append((ent.get_name(), comp, ent.get_name()))

    if env._gah_task in BOWL_TASKS:
        return _bowl_spec(env, spec)
    if env._gah_task != "pick_boxdrink_from_basket":
        return spec

    spec["bodies"].append(("boxdrink", _rigid(env.boxdrink.actor), "boxdrink"))

    def box_frame(_):
        return env.boxdrink.actor.get_pose().to_transformation_matrix()

    spec["actors"]["boxdrink"] = lambda: (env.boxdrink, box_frame,
                                          [("root", _scaled(m, env.boxdrink.config["scale"]))
                                           for m in env.boxdrink.config.get("contact_points_pose", [])])

    bas = _rigid(_entity(env.basket_right))
    spec["bodies"].append(("basket", bas, "basket"))
    spec["local_boxes"]["basket"] = [(bas, _container_boxes(bas))]
    spec["fine"].append("basket")
    spec["regions"]["basket_interior"] = lambda: _container_interior(bas)
    spec["part_owner"]["basket_interior"] = "basket"
    return spec


def _bowl_spec(env, spec):
    """Kitchen-S bowl tasks: the bowl (graspable, contact points from its model data), the dish rack (static
    triangle mesh; the planner sees a floor slab, four walls and the utensil holder) and the sink basin (the
    planner sees its floor and walls, not one solid box)."""
    bowl = env.target_obj
    spec["bodies"].append(("bowl", _rigid(bowl.actor), "bowl"))

    def bowl_frame(_):
        return bowl.actor.get_pose().to_transformation_matrix()

    spec["actors"]["bowl"] = lambda: (bowl, bowl_frame,
                                      [("root", _scaled(m, bowl.config["scale"]))
                                       for m in bowl.config.get("contact_points_pose", [])])
    rack = _rigid(_entity(env.dishrack))
    spec["bodies"].append(("dishrack", rack, "dishrack"))
    spec["local_boxes"]["dishrack"] = [(rack, _rack_boxes(env, rack))]
    spec["fine"].append("dishrack")

    def cup():
        import sapien
        lo, _ = _mesh_world_aabb(rack)
        c = np.asarray(env.dishrack_center, float)
        (x0, x1), (y0, y1), top = RACK_CUP
        return (sapien.Pose([c[0] + (x0 + x1) / 2, c[1] + (y0 + y1) / 2, lo[2] + top / 2]),
                np.array([x1 - x0, y1 - y0, top]))

    spec["parts"]["utensil_holder"] = cup
    spec["part_owner"]["utensil_holder"] = "dishrack"
    spec["fine"].append("sink")
    return spec


# Dish rack (135_dish-rack, as scaled by the kitchens scene), measured from its collision mesh relative to
# env.dishrack_center: a wire basket 30 x 24 x 16 cm whose floor wires rise to 4.9 cm at the middle, with a
# utensil cup inside its -x end.
RACK_FLOOR = 0.05          # m above the counter (the highest floor wire)
RACK_WALL = 0.012          # m
RACK_CUP = ((-0.135, -0.095), (-0.047, 0.047), 0.146)   # x, y ranges relative to the centre, top above the counter


def _rack_boxes(env, comp):
    """Planner boxes (body frame) for the dish rack: floor slab, four walls and the utensil cup, instead of
    its solid bounding box, so the gripper can reach into it."""
    import sapien
    out = _container_boxes(comp, floor=RACK_FLOOR, wall=RACK_WALL)
    lo, _ = _mesh_world_aabb(comp)
    c = np.asarray(env.dishrack_center, float)
    (x0, x1), (y0, y1), top = RACK_CUP
    centre = np.array([c[0] + (x0 + x1) / 2, c[1] + (y0 + y1) / 2, lo[2] + top / 2])
    out.append((comp.entity.get_pose().inv() * sapien.Pose(centre),
                np.array([(x1 - x0) / 2, (y1 - y0) / 2, top / 2])))
    return out


def close(env):
    try:
        env.close_env()
    except Exception:
        pass
