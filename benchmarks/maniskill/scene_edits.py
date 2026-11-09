"""Scene edits for the ManiSkill3 backend (see docs/api_spec.md, "Scene edits").

Supported keys: object_pose, object_color, lighting, add_distractor,
add_obstacle, object_motion, camera_pose, dynamics.
Unsupported: object_scale, swap_object (see SCENE_EDITS.md).

Every key maps to one spec dict or a list of spec dicts.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import sapien
import torch
from transforms3d.euler import euler2quat
from transforms3d.quaternions import qmult

SUPPORTED = {
    "object_pose", "object_color", "lighting", "add_distractor", "add_obstacle",
    "object_motion", "camera_pose", "dynamics",
}
UNSUPPORTED = {
    "object_scale": "collision geometry of task objects is fixed when the task builds its scene",
    "swap_object": "task objects and their success checks are defined by the task class",
}


class UnsupportedEdit(ValueError):
    pass


def normalize(edits: dict) -> dict:
    out = {}
    for key, value in (edits or {}).items():
        if key in UNSUPPORTED:
            raise UnsupportedEdit(f"{key}: {UNSUPPORTED[key]}")
        if key not in SUPPORTED:
            raise UnsupportedEdit(f"unknown edit key '{key}'")
        specs = value if isinstance(value, list) else [value]
        out[key] = [dict(s) for s in specs]
    return out


def _quat_from_rpy(rpy):
    return np.array(euler2quat(*rpy), dtype=np.float64)  # wxyz, static xyz


def _np(x):
    return x.cpu().numpy().reshape(-1) if torch.is_tensor(x) else np.asarray(x, dtype=np.float64).reshape(-1)


def _set_actor_pose(actor, p, q):
    actor.set_pose(sapien.Pose(np.asarray(p, dtype=np.float32), np.asarray(q, dtype=np.float32)))


def _materials(entity):
    rb = entity.find_component_by_type(sapien.render.RenderBodyComponent)
    if rb is None:
        return []
    mats = []
    for shape in rb.render_shapes:
        parts = shape.get_parts() if hasattr(shape, "get_parts") else []
        if parts:
            mats.extend(part.material for part in parts)
        elif getattr(shape, "material", None) is not None:
            mats.append(shape.material)
    return mats


# --------------------------------------------------------------------------- #
# Build time (inside _load_scene / _load_lighting / camera config)
# --------------------------------------------------------------------------- #
def build_extra_actors(env, edits: dict):
    for kind, dynamic in (("add_obstacle", False), ("add_distractor", True)):
        for i, spec in enumerate(edits.get(kind, [])):
            name = spec.get("name") or f"{'obstacle' if kind == 'add_obstacle' else 'distractor'}_{i}"
            shape = spec.get("shape", spec.get("asset", "box"))
            size = np.asarray(spec.get("size", [0.04, 0.04, 0.04]), dtype=np.float64)
            color = list(spec.get("rgb", spec.get("color", [0.5, 0.5, 0.5]))) + [1.0]
            builder = env.scene.create_actor_builder()
            material = sapien.render.RenderMaterial(base_color=color)
            if shape == "box":
                half = (size / 2).tolist()
                builder.add_box_collision(half_size=half)
                builder.add_box_visual(half_size=half, material=material)
            elif shape == "sphere":
                r = float(size.reshape(-1)[0]) / 2
                builder.add_sphere_collision(radius=r)
                builder.add_sphere_visual(radius=r, material=material)
            elif shape == "cylinder":  # size = [diameter, length], axis along x
                r, half_len = float(size[0]) / 2, float(size[1]) / 2
                builder.add_cylinder_collision(radius=r, half_length=half_len)
                builder.add_cylinder_visual(radius=r, half_length=half_len, material=material)
            else:
                raise UnsupportedEdit(f"{kind}: unknown shape '{shape}' (box | sphere | cylinder)")
            p, q = _spec_pose(spec)
            builder.initial_pose = sapien.Pose(p.astype(np.float32), q.astype(np.float32))
            if dynamic:
                actor = builder.build(name=f"gah_{name}")
                if "mass" in spec:
                    for entity in actor._objs:
                        body = entity.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
                        body.mass = float(spec["mass"])
            else:
                actor = builder.build_kinematic(name=f"gah_{name}")
            env.gah_extra[name] = actor
            spec["_name"] = name


def _spec_pose(spec):
    pose = spec.get("pose", [0.0, 0.3, 0.05])
    pose = np.asarray(pose, dtype=np.float64).reshape(-1)
    p = pose[:3]
    if pose.size == 7:
        q = pose[3:]
    elif "rpy" in spec:
        q = _quat_from_rpy(spec["rpy"])
    else:
        q = np.array([1.0, 0, 0, 0])
    return p, q


def load_lighting(env, spec):
    intensity = float(spec.get("intensity", 1.0))
    color = np.asarray(spec.get("color", spec.get("rgb", [1.0, 1.0, 1.0])), dtype=np.float64)
    shadow = env.enable_shadow
    env.scene.set_ambient_light((0.3 * intensity * color).tolist())
    env.scene.add_directional_light([1, 1, -1], (intensity * color).tolist(), shadow=shadow,
                                    shadow_scale=5, shadow_map_size=2048)
    env.scene.add_directional_light([0, 0, -1], (intensity * color).tolist())


def shift_camera(cfg, spec):
    if isinstance(cfg, (list, tuple)):
        return [shift_camera(c, spec) for c in cfg]
    pose = cfg.pose
    sp = pose.sp if hasattr(pose, "sp") else pose
    p = np.asarray(sp.p, dtype=np.float64) + np.asarray(spec.get("delta_xyz", [0, 0, 0]), dtype=np.float64)
    q = qmult(_quat_from_rpy(spec.get("delta_rpy", [0, 0, 0])), np.asarray(sp.q, dtype=np.float64))
    return dataclasses.replace(cfg, pose=sapien.Pose(p.astype(np.float32), q.astype(np.float32)))


# --------------------------------------------------------------------------- #
# Episode time (inside _initialize_episode / step)
# --------------------------------------------------------------------------- #
def apply_episode_edits(env, edits: dict):
    for kind in ("add_obstacle", "add_distractor"):
        for spec in edits.get(kind, []):
            p, q = _spec_pose(spec)
            actor = env.gah_extra[spec["_name"]]
            _set_actor_pose(actor, p, q)
            if kind == "add_distractor":
                actor.set_linear_velocity(torch.zeros((1, 3)))
                actor.set_angular_velocity(torch.zeros((1, 3)))

    for spec in edits.get("object_pose", []):
        actor = env.gah_actor(spec["name"])
        p, q = _np(actor.pose.p), _np(actor.pose.q)
        if "xyz" in spec:
            p = np.asarray(spec["xyz"], dtype=np.float64)
        if "delta_xyz" in spec:
            p = p + np.asarray(spec["delta_xyz"], dtype=np.float64)
        if "rpy" in spec:
            q = _quat_from_rpy(spec["rpy"])
        if "delta_rpy" in spec:
            q = qmult(_quat_from_rpy(spec["delta_rpy"]), q)
        _set_actor_pose(actor, p, q)
        if env.gah_task == "PlugCharger-v1" and spec["name"] == "receptacle":
            # The task stores its goal relative to the receptacle at reset time.
            env.goal_pose = env.receptacle.pose * sapien.Pose(q=euler2quat(0, 0, np.pi))

    for spec in edits.get("object_color", []):
        actor = env.gah_actor(spec["name"])
        rgba = list(spec.get("rgb", spec.get("color"))) + [1.0]
        for entity in actor._objs:
            for mat in _materials(entity):
                mat.base_color = rgba[:4]

    for spec in edits.get("dynamics", []):
        actor = env.gah_actor(spec["name"])
        for entity in actor._objs:
            body = entity.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
            if body is None:
                continue
            if "mass" in spec:
                body.mass = float(spec["mass"])
            if "friction" in spec:
                f = float(spec["friction"])
                for shape in body.collision_shapes:
                    old = shape.physical_material
                    shape.set_physical_material(sapien.physx.PhysxMaterial(f, f, old.restitution))

    env.gah_motion_state = [dict(spec, _started=False) for spec in edits.get("object_motion", [])]


def apply_motion(env, t: float):
    """object_motion: {name, velocity [m/s], start_time [s], duration [s] (default 2),
    mode: constant (default, velocity re-applied every step) | impulse (applied once)}."""
    for spec in getattr(env, "gah_motion_state", []):
        start = float(spec.get("start_time", 0.0))
        dur = float(spec.get("duration", 2.0))
        mode = spec.get("mode", "constant")
        if t < start:
            continue
        if mode == "impulse" and spec["_started"]:
            continue
        if mode == "constant" and t > start + dur:
            continue
        actor = env.gah_actor(spec["name"])
        vel = torch.tensor([spec["velocity"]], dtype=torch.float32)
        actor.set_linear_velocity(vel)
        spec["_started"] = True


def scene_validity(env, settle_steps: int = 10, max_drift: float = 0.03, reach: float = 0.85):
    """Cheap physical sanity check after reset: objects in reach, above the table, and not
    exploding out of interpenetration. Returns (ok, reason)."""
    moving = {s["name"] for s in getattr(env, "gah_motion_state", [])}
    base = _np(env.agent.robot.pose.p)
    start = {}
    for name in env.gah_object_names():
        p = _np(env.gah_actor(name).pose.p)
        start[name] = p
        if name in env.gah_extra:
            continue
        if np.linalg.norm(p[:2] - base[:2]) > reach:
            return False, f"{name} is out of reach ({np.linalg.norm(p[:2] - base[:2]):.2f} m from the base)"
        if p[2] < -0.01:
            return False, f"{name} is below the table (z={p[2]:.3f})"
    qpos = env.agent.robot.get_qpos().cpu().numpy()[0]
    hold = np.concatenate([qpos[:7], [1.0]])
    saved = getattr(env, "gah_motion_state", [])
    env.gah_motion_state = []  # check the static scene only
    try:
        for _ in range(settle_steps):
            env.step(hold)
    finally:
        env.gah_motion_state = saved
    for name, p0 in start.items():
        if name in moving:
            continue
        drift = np.linalg.norm(_np(env.gah_actor(name).pose.p) - p0)
        if drift > max_drift:
            return False, f"{name} moved {drift:.3f} m while settling (interpenetration or unsupported)"
    return True, "ok"
