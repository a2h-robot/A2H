"""RoboDyna environment wrapper for GAH (person E).

RoboDyna is a RoboTwin 2.0 derivative with the dual-UR5 `ur5-wsg` embodiment (SAPIEN 3, cuRobo).
Every GAH training task runs in one fixed Base-suite condition, SCENARIO (Opt 1+2), with the same
feature flags RoboDyna's GUI, collector and policy evaluator apply (task_config/scenario_overrides.py).

One `GahEnv` lives per worker process and per task; the robot and its cuRobo planners survive
across episodes, so only the first reset pays the planner warmup.

`reset(seed, variation)` sets the episode up exactly as RoboDyna's policy evaluator does
(eval_mode, is_test). The variation's scene edits (scene_edits.py) run inside the task's own
`load_actors`, so RoboDyna's stability check (UnStableError) also covers the edited scene.
RoboDyna arms its scene dynamics (belts, rolling balls, ripening fruit, oscillating parts) at the
end of setup_demo, so the world is live from the pipeline's first step, as it is for a policy.
This needs RoboDyna main at 954af13e or later, which has the expert-free eval gates
(`_sync_eval_gates`, merged from dualmode-eval): before that, several tasks only advanced their
state inside the scripted expert, so no other program could succeed.

Every physics step goes through a hook that counts it against the budget, drives object_motion
edits and, like RoboDyna's evaluator (`take_action`), checks the task's own `check_success()`
after the step: the first success is latched. After `run(api)` returns, the world keeps running
with the arms holding still for up to POST_RUN_STEPS (a released block still rides the belt).
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path

import scene_edits

ROBODYNA_ROOT = os.environ.get("ROBODYNA_ROOT", str(Path.home() / "RoboDyna"))   # RoboDyna main >= 5303d554
SCENARIO = os.environ.get("GAH_SCENARIO", "opt1+2")
TASK_CONFIG = "demo_dynamic"     # RoboDyna's production profile
TIMESTEP = 1 / 250               # RoboDyna's physics step
MAX_STEPS = {"default": 15000}   # 60 s of sim time per episode (= demo_dynamic eval_step_limit)
POST_RUN_STEPS = 2500            # up to 10 s of world time after run(api) returns, arms holding
OBB_CACHE = 48                   # cuRobo cuboid slots per arm (table + perceived obstacles)

# Named things per task. objects: movable task objects (detect, grasp, scene edits);
# parts: fixed or task-animated parts (detect only). Name -> attribute on the task, or
# "attr[key]" for one entry of a dict attribute. Names describe what is visible, never the answer:
# pick_ripe_apple exposes "left apple" / "right apple", not which one is spoiled, and
# drop_ball_hole exposes its holes as "hole A" / "hole B" in a per-seed random order (see HOLES).
TASKS = {
    "catch_valley_ball": {
        "goal": "Push the blue box in front of the ramp behind the red line to catch the red ball.",
        "objects": {"red ball": "ball", "blue box": "bowl", "black ball": "distractor"},
        "parts": {"red line": "red_line"},
    },
    "pick_ripe_apple": {
        "goal": "Pick the good apple and place it in the basket while ignoring spoiled fruit.",
        "objects": {"left apple": "apples[-1]", "right apple": "apples[1]", "basket": "basket"},
        "parts": {"tree": "tree"},
    },
    "drop_ball_hole": {
        "goal": "Guide the ball through the target hole of the rotating sorter into the container.",
        "objects": {"ball": "ball"},
        "parts": {"sorter": "cap_entity", "container floor": "bucket_floor"},
    },
    "place_block_belt": {
        "goal": "Place the block onto the belt so it rides upright into the exit bowl.",
        "objects": {"block": "block", "bowl": "bowl", "blocker": "blocker"},
        "parts": {"belt": "belt", "place line": "place_line", "belt support": "support_box"},
    },
    "catch_shelf_marble": {
        "goal": "Move the bowl to left and right by pressing the corresponding keys to catch the red ball.",
        "objects": {"red ball": "ball", "bowl": "bowl"},
        "parts": {"belt": "belt", "left key": "keys[left]", "right key": "keys[right]"},
    },
}


HOLES = {"drop_ball_hole": ("hole A", "hole B")}   # target and (Opt 2) decoy hole, shuffled per seed


class StepLimitExceeded(Exception):
    pass


class VirtualPart:
    """A part with no body of its own (a hole cut in a moving platform): pose and size only."""

    def __init__(self, name, pose_fn, extents, rgb=None):
        self._name, self._pose_fn = name, pose_fn
        self.gah_extents = list(map(float, extents))
        self.gah_rgb = rgb

    def get_pose(self):
        return self._pose_fn()

    def get_name(self):
        return self._name

    def get_components(self):
        return []


def _resolve(env, spec):
    """Task attribute "attr", or "attr[key]" for one entry of a dict attribute (numeric keys match by value)."""
    if "[" not in spec:
        return getattr(env, spec, None)
    attr, key = spec[:-1].split("[", 1)
    d = getattr(env, attr, None)
    if not isinstance(d, dict):
        return None
    for k, v in d.items():
        if str(k) == key:
            return v
        try:
            if float(k) == float(key):
                return v
        except (TypeError, ValueError):
            pass
    return None


def _hole_parts(env, seed):
    """drop_ball_hole: the target hole and, with Opt 2, the decoy hole, as moving VirtualParts.
    The decoy is cut smaller than the ball; names do not say which hole is which."""
    import numpy as np
    import sapien
    holes = [("target", env._cap_hole_local_xy, float(env.cap_hole_radius))]
    if getattr(env, "add_dummy_hole", False) and getattr(env, "_dummy_hole_local_xy", None) is not None:
        holes.append(("decoy", env._dummy_hole_local_xy, float(env.dummy_hole_radius)))
    order = np.random.default_rng(seed).permutation(len(holes))
    out = {}
    for name, k in zip(HOLES["drop_ball_hole"], order):
        _, local, r = holes[k]
        local = np.asarray(local, dtype=np.float64).copy()

        def pose(local=local):
            cap = env.cap_entity.get_pose()
            xy = np.asarray(env.cap_center[:2]) + env._cap_rot2() @ local
            return sapien.Pose([float(xy[0]), float(xy[1]), float(cap.p[2])], cap.q)

        out[name] = VirtualPart(name, pose, [2 * r, 2 * r, 0.0])
    return out


def _setup_paths():
    os.chdir(ROBODYNA_ROOT)
    for p in (ROBODYNA_ROOT, os.path.join(ROBODYNA_ROOT, "script"), os.path.join(ROBODYNA_ROOT, "script", "bench_script")):
        if p not in sys.path:
            sys.path.insert(0, p)
    os.environ.setdefault("VK_ICD_FILENAMES", "/usr/share/vulkan/icd.d/nvidia_icd.json")
    os.environ.pop("DISPLAY", None)


_PATCHED = False


def _patch_curobo_cache():
    """RoboDyna builds cuRobo with a world of one cuboid (the table). update_world() cannot grow the
    collision cache after warmup (CUDA graphs fix its size), so reserve OBB_CACHE slots up front."""
    global _PATCHED
    if _PATCHED:
        return
    from curobo.wrap.reacher.motion_gen import MotionGenConfig
    orig = MotionGenConfig.load_from_robot_config

    def load_from_robot_config(*a, **kw):
        kw.setdefault("collision_cache", {"obb": OBB_CACHE})
        return orig(*a, **kw)

    MotionGenConfig.load_from_robot_config = staticmethod(load_from_robot_config)
    _PATCHED = True


def load_args(task: str) -> dict:
    """RoboDyna's episode arguments for `task` in SCENARIO, as its policy evaluator builds them."""
    from script.bench_script.record_demo import build_args
    from task_config.scenario_overrides import apply_base_scenario
    tmp = os.environ.get("GAH_TMP", str(Path.home() / "gah_tmp"))
    args = build_args(task, TASK_CONFIG, os.path.join(tmp, task), option=None, task_arg_overrides=[])
    args.update(collect_data=False, save_data=False, eval_video_log=False, need_plan=True, render_freq=0,
                episode_num=1, check_render_success=False, export_lerobot=False, eval_mode=True)
    apply_base_scenario(args, task, SCENARIO)
    return args


def _entity(val):
    """RoboDyna keeps some task objects as envs.utils.Actor and some as raw sapien.Entity."""
    return getattr(val, "actor", val)


class GahEnv:
    def __init__(self, task: str):
        if task not in TASKS:
            raise KeyError(f"unknown task {task!r}; configured: {sorted(TASKS)}")
        _setup_paths()
        _patch_curobo_cache()
        self.task = task
        self.args = load_args(task)
        mod = importlib.import_module(f"envs.{task}")
        self.task_module = mod
        self.cls = getattr(mod, task)
        self.env = self.cls()
        self.max_steps = MAX_STEPS.get(task, MAX_STEPS["default"])
        self.n_resets = 0
        self.steps = 0
        self.edits = {}
        self.extra = {}            # name -> Actor added by scene edits
        self.motion = []           # active object_motion specs
        self.seed = None
        self.succeeded = False     # latched: check_success() was true after some physics step
        self.success_step = None
        self.step_callbacks = []   # fn(steps) after every counted step (video capture)
        self.virtual = {}          # name -> VirtualPart (e.g. holes)

    # ------------------------------------------------------------------ objects
    def task_objects(self) -> dict:
        out = {}
        for name, spec in TASKS[self.task]["objects"].items():
            val = _resolve(self.env, spec)
            if val is not None:
                out[name] = val
        return out

    def parts(self) -> dict:
        out = {}
        for name, spec in TASKS[self.task].get("parts", {}).items():
            val = _resolve(self.env, spec)
            if val is not None:
                out[name] = val
        out.update(self.virtual)
        return out

    def objects(self) -> dict:
        """Every movable named object: task objects first, then objects added by scene edits."""
        out = self.task_objects()
        out.update(self.extra)
        return out

    def entity(self, name):
        """sapien.Entity of an object or part."""
        val = {**self.parts(), **self.objects()}.get(name)
        return None if val is None else _entity(val)

    # ------------------------------------------------------------------ episode
    def reset(self, seed: int, variation: dict | None = None):
        self.edits = scene_edits.normalize((variation or {}).get("edits", {}))
        self.extra = {}
        self.motion = []
        self.steps = 0
        self.seed = seed
        self.succeeded = False
        self.success_step = None
        self.step_callbacks = []
        env = self.env
        cls_load = self.cls.load_actors
        restore = scene_edits.install_creation_hooks(self)

        def load_actors():
            cls_load(env)
            scene_edits.apply_at_load(self, self.edits)

        env.load_actors = load_actors
        try:
            env.setup_demo(now_ep_num=self.n_resets, seed=seed, is_test=True, **self.args)
        finally:
            del env.load_actors
            restore()
        self.n_resets += 1
        self.virtual = _hole_parts(env, seed) if self.task in HOLES else {}
        env.plan_success = True
        scene = env.scene
        raw_step = scene.step

        def step():
            self.steps += 1
            if self.steps > self.max_steps:
                raise StepLimitExceeded(f"{self.max_steps} physics steps")
            if self.motion:
                scene_edits.on_step(self)
            raw_step()
            self._latch_success()
            for cb in self.step_callbacks:
                cb(self.steps)

        scene.step = step
        self._raw_step = raw_step

    def _latch_success(self):
        if self.succeeded:
            return
        try:
            ok = bool(self.env.check_success())
        except Exception:
            ok = False
        if ok:
            self.succeeded, self.success_step = True, self.steps

    def tick(self, n: int):
        """Advance the world n counted physics steps the way RoboDyna does (kinematic tasks and the
        expert-free gates, then physics, then transient checks), with the arms holding their targets."""
        robot = self.env.robot
        for _ in range(int(n)):
            robot._entity_qf(robot.left_entity)
            robot._entity_qf(robot.right_entity)
            self.env._tick_kinematic_tasks()
            self.env.scene.step()
            if not self.env.transient_event:
                self.env._update_transient_checks()

    @property
    def sim_time(self) -> float:
        return self.steps * TIMESTEP

    def finish(self, n: int = POST_RUN_STEPS):
        """After run(api): keep the world running, arms holding, until success or n steps or the budget."""
        for _ in range(int(n)):
            if self.succeeded or self.steps >= self.max_steps:
                break
            self.tick(1)

    def success(self) -> bool:
        self._latch_success()
        return self.succeeded

    def fail_reason(self) -> str:
        return "" if self.succeeded else str(getattr(self.env, "_last_fail_reason", "") or "")

    def close(self):
        try:
            self.env.close_env(clear_cache=(self.n_resets % 5 == 0))
        except Exception:
            pass
