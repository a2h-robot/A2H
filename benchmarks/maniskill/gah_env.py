"""ManiSkill3 task environments with GAH scene edits baked in.

`make_env(task, variation)` returns a single-env ManiSkill3 environment whose
class is a subclass of the original task. The variation's edits are applied
inside the env's own build / reset hooks, so every `env.reset(seed=...)`
(including the ones done by the official motion-planning solutions) sees the
edited scene. The task's success check (`evaluate`) is never overridden.
"""

from __future__ import annotations

import copy

import mani_skill.envs  # noqa: F401  (registers the tasks)
from mani_skill.utils.registration import REGISTERED_ENVS

import scene_edits

TASKS = [
    "PegInsertionSide-v1",
    "PlugCharger-v1",
    "PullCubeTool-v1",
    "LiftPegUpright-v1",
    "PlaceSphere-v1",
]

# Language-level object name -> attribute on the env.
TASK_OBJECTS = {
    "PegInsertionSide-v1": {"peg": "peg", "box_with_hole": "box"},
    "PlugCharger-v1": {"charger": "charger", "receptacle": "receptacle"},
    "PullCubeTool-v1": {"cube": "cube", "l_shape_tool": "l_shape_tool"},
    "LiftPegUpright-v1": {"peg": "peg"},
    "PlaceSphere-v1": {"sphere": "obj", "bin": "bin"},
}

DEFAULT_MAX_STEPS = 2000  # 100 s of simulated time at 20 Hz control


class StepLimitExceeded(RuntimeError):
    pass


def make_env(task: str, variation: dict | None = None, max_steps: int = DEFAULT_MAX_STEPS):
    if task not in TASKS:
        raise ValueError(f"unknown task {task}; choose from {TASKS}")
    edits = scene_edits.normalize((variation or {}).get("edits", {}))
    base_cls = REGISTERED_ENVS[task].cls

    class GAHEnv(base_cls):
        def __init__(self, *args, **kwargs):
            self.gah_task = task
            self.gah_edits = copy.deepcopy(edits)
            self.gah_extra = {}
            self.gah_max_steps = max_steps
            self.gah_steps = 0
            super().__init__(*args, **kwargs)

        @property
        def _default_human_render_camera_configs(self):
            cfg = super()._default_human_render_camera_configs
            for spec in self.gah_edits.get("camera_pose", []):
                cfg = scene_edits.shift_camera(cfg, spec)
            return cfg

        def _load_lighting(self, options):
            if self.gah_edits.get("lighting"):
                scene_edits.load_lighting(self, self.gah_edits["lighting"][-1])
            else:
                super()._load_lighting(options)

        def _load_scene(self, options):
            super()._load_scene(options)
            scene_edits.build_extra_actors(self, self.gah_edits)

        def _initialize_episode(self, env_idx, options):
            super()._initialize_episode(env_idx, options)
            self.gah_steps = 0
            scene_edits.apply_episode_edits(self, self.gah_edits)

        def step(self, action):
            self.gah_steps += 1
            if self.gah_steps > self.gah_max_steps:
                raise StepLimitExceeded(f"episode exceeded {self.gah_max_steps} control steps")
            scene_edits.apply_motion(self, self.gah_steps * self.control_timestep)
            return super().step(action)

        def gah_actor(self, name: str):
            """Actor for a language-level object name (task objects and added actors)."""
            attr = TASK_OBJECTS[task].get(name)
            if attr is not None:
                return getattr(self, attr)
            if name in self.gah_extra:
                return self.gah_extra[name]
            raise KeyError(f"unknown object '{name}'; known: {self.gah_object_names()}")

        def gah_object_names(self):
            return list(TASK_OBJECTS[task]) + list(self.gah_extra)

        def gah_success(self) -> bool:
            return bool(self.evaluate()["success"].cpu().numpy().reshape(-1)[0])

    GAHEnv.__name__ = f"GAH_{base_cls.__name__}"
    return GAHEnv(
        num_envs=1,
        obs_mode="none",
        control_mode="pd_joint_pos",
        render_mode="rgb_array",
        sim_backend="physx_cpu",
    )
