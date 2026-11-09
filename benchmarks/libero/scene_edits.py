"""Scene edits for the LIBERO-Goal backend (see docs/api_spec.md, "Scene edits").

All ten keys are supported. Every key maps to one spec dict or a list of spec dicts.
Edits are applied at three points (see gah_env.GAHEnv):
  1. BDDL, before the env is built:        swap_object
  2. MJCF, before the model is compiled:   add_obstacle, add_distractor, object_scale
  3. MuJoCo model/data, after the init state is loaded:
                                           object_pose, object_color, lighting, camera_pose,
                                           dynamics, object_motion (every physics substep)
The task's BDDL goal (its success check) is never changed.
"""

from __future__ import annotations

import re
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

SUPPORTED = {
    "object_pose", "object_color", "object_scale", "swap_object", "lighting",
    "add_distractor", "add_obstacle", "object_motion", "camera_pose", "dynamics",
}
CAMERAS = {"head": "agentview", "wrist": "robot0_eye_in_hand"}
TABLE_TOP_Z = 0.90


class UnsupportedEdit(ValueError):
    pass


def normalize(edits: dict | None) -> dict:
    out = {}
    for key, value in (edits or {}).items():
        if key not in SUPPORTED:
            raise UnsupportedEdit(f"unknown edit key '{key}'; supported: {sorted(SUPPORTED)}")
        specs = value if isinstance(value, list) else [value]
        if not all(isinstance(s, dict) for s in specs):
            raise UnsupportedEdit(f"{key}: each spec must be a mapping")
        out[key] = [dict(s) for s in specs]
    for spec in out.get("object_scale", []):
        s = float(spec.get("scale", 1.0))
        if not 0.5 <= s <= 1.5:
            raise UnsupportedEdit(f"object_scale: scale {s} outside [0.5, 1.5]")
    return out


def quat_wxyz_from_rpy(rpy):
    x, y, z, w = Rotation.from_euler("xyz", rpy).as_quat()
    return np.array([w, x, y, z])


def _rot(q_wxyz):
    w, x, y, z = q_wxyz
    return Rotation.from_quat([x, y, z, w])


def _wxyz(rot: Rotation):
    x, y, z, w = rot.as_quat()
    return np.array([w, x, y, z])


# --------------------------------------------------------------------------- #
# 1. BDDL: swap_object
# --------------------------------------------------------------------------- #
def _section(text, name):
    m = re.search(r"\(:" + name + r"\b", text)
    if not m:
        return None
    depth, i = 0, m.start()
    for j in range(i, len(text)):
        depth += text[j] == "("
        depth -= text[j] == ")"
        if depth == 0:
            return text[i:j + 1]
    return None


def swapped_bddl(bddl_path: str, specs: list[dict]) -> str:
    """Write a copy of the BDDL file with object categories swapped; return its path.
    Only movable objects (the :objects section) can be swapped. Instance names stay the same,
    so the goal (success check) is textually identical."""
    from libero.libero.envs.base_object import OBJECTS_DICT
    text = Path(bddl_path).read_text()
    objects = _section(text, "objects")
    new_objects = objects
    for spec in specs:
        name, new_cat = spec["name"], spec["with"]
        if new_cat not in OBJECTS_DICT:
            raise UnsupportedEdit(f"swap_object: unknown LIBERO object category '{new_cat}'")
        pat = re.compile(r"(\b" + re.escape(name) + r"\s+-\s+)([A-Za-z0-9_]+)")
        if not pat.search(new_objects):
            raise UnsupportedEdit(f"swap_object: '{name}' is not a movable object of this task")
        new_objects = pat.sub(lambda m: m.group(1) + new_cat, new_objects, count=1)
    new_text = text.replace(objects, new_objects)
    assert _section(new_text, "goal") == _section(text, "goal")
    tmp = Path(tempfile.mkdtemp(prefix="gah_bddl_")) / Path(bddl_path).name
    tmp.write_text(new_text)
    return str(tmp)


# --------------------------------------------------------------------------- #
# 2. MJCF (called from the problem class's _load_model, before compilation)
# --------------------------------------------------------------------------- #
def _primitive(name, spec, fixed):
    from robosuite.models.objects import BallObject, BoxObject, CylinderObject
    shape = spec.get("shape", "box")
    size = np.asarray(spec.get("size", [0.04, 0.04, 0.04]), dtype=float).reshape(-1)
    rgba = list(spec.get("rgb", [0.55, 0.35, 0.2] if fixed else [0.2, 0.5, 0.8]))[:3] + [1.0]
    kw = dict(name=name, rgba=rgba, joints=None if fixed else "default", obj_type="all",
              friction=(1.0, 0.005, 0.0001))
    if not fixed and "mass" in spec:
        vol = {"box": np.prod(size), "sphere": np.pi / 6 * size[0] ** 3,
               "cylinder": np.pi * (size[0] / 2) ** 2 * (size[1] if size.size > 1 else size[0])}[shape]
        kw["density"] = float(spec["mass"]) / max(vol, 1e-6)
    if shape == "box":
        return BoxObject(size=(size / 2).tolist(), **kw)
    if shape == "sphere":
        return BallObject(size=[size[0] / 2], **kw)
    if shape == "cylinder":  # size = [diameter, length]; axis along z
        return CylinderObject(size=[size[0] / 2, (size[1] if size.size > 1 else size[0]) / 2], **kw)
    raise UnsupportedEdit(f"shape must be box | sphere | cylinder, got '{shape}'")


def spec_pose(spec):
    pose = np.asarray(spec.get("pose", [0.0, 0.0, TABLE_TOP_Z + 0.05]), dtype=float).reshape(-1)
    p = pose[:3]
    if pose.size == 7:
        q = pose[3:] / np.linalg.norm(pose[3:])
    else:
        q = quat_wxyz_from_rpy(spec.get("rpy", [0.0, 0.0, 0.0]))
    return p, q


def edit_model(env, edits: dict):
    """env: the LIBERO problem env (robosuite), inside _load_model after super()._load_model()."""
    from robosuite.utils.mjcf_utils import array_to_string
    env.gah_added = {}
    for kind, fixed in (("add_obstacle", True), ("add_distractor", False)):
        for i, spec in enumerate(edits.get(kind, [])):
            name = spec.get("name") or f"{'obstacle' if fixed else 'distractor'}_{i}"
            if "asset" in spec and spec["asset"] not in ("box", "sphere", "cylinder"):
                from libero.libero.envs.objects import get_object_fn
                obj = get_object_fn(spec["asset"])(name=name) if not fixed else \
                    get_object_fn(spec["asset"])(name=name, joints=None)
            else:
                obj = _primitive(name, {**spec, "shape": spec.get("shape", spec.get("asset", "box"))}, fixed)
            p, q = spec_pose(spec)
            body = obj.get_obj()
            if fixed:
                body.set("pos", array_to_string(p))
                body.set("quat", array_to_string(q))
            env.model.merge_objects([obj])
            env.gah_added[name] = {"root_body": obj.root_body, "fixed": fixed, "pose": (p, q), "kind": kind}
    for spec in edits.get("object_scale", []):
        _scale_object_xml(env, spec["name"], float(spec["scale"]))


def _scale_vec(s, factor):
    return " ".join(f"{float(v) * factor:.6g}" for v in s.split())


def _scale_object_xml(env, name, scale):
    obj = env.objects_dict.get(name) or env.fixtures_dict.get(name)
    if obj is None:
        raise UnsupportedEdit(f"object_scale: unknown object '{name}'")
    root = None
    for b in env.model.worldbody.iter("body"):
        if b.get("name") == obj.root_body:
            root = b
            break
    if root is None:
        raise UnsupportedEdit(f"object_scale: body for '{name}' not found")
    meshes = {m.get("name"): m for m in env.model.asset.iter("mesh")}
    scaled_meshes = set()
    for el in root.iter():
        if el is not root and el.get("pos") is not None:
            el.set("pos", _scale_vec(el.get("pos"), scale))
        if el.tag in ("geom", "site") and el.get("size") is not None and el.get("type") != "mesh":
            el.set("size", _scale_vec(el.get("size"), scale))
        if el.tag == "geom" and el.get("mesh") and el.get("mesh") not in scaled_meshes:
            m = meshes[el.get("mesh")]
            m.set("scale", _scale_vec(m.get("scale", "1 1 1"), scale))
            scaled_meshes.add(el.get("mesh"))
        if el.tag == "inertial" and el.get("mass") is not None:
            el.set("mass", f"{float(el.get('mass')) * scale ** 3:.6g}")
    env.gah_scaled = getattr(env, "gah_scaled", {}) | {name: scale}


# --------------------------------------------------------------------------- #
# 3. Model / data edits after the init state is loaded
# --------------------------------------------------------------------------- #
def apply_initial_placement(genv):
    """Right after the init state is loaded (before the scene settles): place added free
    objects, and lift resized / swapped objects so that their lowest point is on the table."""
    import mujoco
    m, d = genv.model, genv.data
    edits = genv.edits
    for name, info in genv.added.items():
        if not info["fixed"]:
            genv.set_free_pose(name, *info["pose"])
    lift = {s["name"] for s in edits.get("object_scale", [])} | {s["name"] for s in edits.get("swap_object", [])}
    mujoco.mj_forward(m, d)
    for name in lift:
        if genv.is_free(name):
            p, q = genv.true_pose(name)
            low = genv.lowest_point(name)
            genv.set_free_pose(name, p + [0, 0, TABLE_TOP_Z + 0.002 - low], q)
            mujoco.mj_forward(m, d)


def apply_episode_edits(genv):
    """After the base scene has settled: poses, appearance, camera and dynamics."""
    import mujoco
    m, d = genv.model, genv.data
    edits = genv.edits

    for spec in edits.get("object_pose", []):
        name = genv.resolve(spec["name"])
        p, q = genv.true_pose(name)
        if "xyz" in spec:
            p = np.asarray(spec["xyz"], float)
        if "delta_xyz" in spec:
            p = p + np.asarray(spec["delta_xyz"], float)
        if "rpy" in spec:
            q = quat_wxyz_from_rpy(spec["rpy"])
        if "delta_rpy" in spec:
            q = _wxyz(Rotation.from_euler("xyz", spec["delta_rpy"]) * _rot(q))
        if genv.is_free(name):
            genv.set_free_pose(name, p, q)
        else:
            b = genv.root_body(name)
            parent = m.body_parentid[b]
            if parent != 0:
                raise UnsupportedEdit(f"object_pose: '{name}' is not a top-level body")
            m.body_pos[b] = p
            m.body_quat[b] = q
        mujoco.mj_forward(m, d)

    for spec in edits.get("object_color", []):
        name = genv.resolve(spec["name"])
        rgba = list(spec["rgb"])[:3] + [1.0]
        for g in genv.geoms_of(name, collision=False):
            m.geom_matid[g] = -1
            m.geom_rgba[g] = rgba

    for spec in edits.get("lighting", []):
        intensity = float(spec.get("intensity", 1.0))
        color = np.asarray(spec.get("color", [1.0, 1.0, 1.0]), float)[:3]
        for i in range(m.nlight):
            m.light_diffuse[i] = np.clip(genv.base_light["diffuse"][i] * intensity * color, 0, 1)
            m.light_specular[i] = np.clip(genv.base_light["specular"][i] * intensity * color, 0, 1)
            m.light_ambient[i] = np.clip(genv.base_light["ambient"][i] * intensity * color, 0, 1)

    for spec in edits.get("camera_pose", []):
        cam = CAMERAS.get(spec.get("view", "head"), spec.get("view"))
        c = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam)
        if c < 0:
            raise UnsupportedEdit(f"camera_pose: unknown view '{spec.get('view')}'")
        m.cam_pos[c] = m.cam_pos[c] + np.asarray(spec.get("delta_xyz", [0, 0, 0]), float)
        dr = Rotation.from_euler("xyz", spec.get("delta_rpy", [0, 0, 0]))
        m.cam_quat[c] = _wxyz(dr * _rot(m.cam_quat[c]))

    for spec in edits.get("dynamics", []):
        name = genv.resolve(spec["name"])
        if "mass" in spec:
            bodies = genv.bodies_of(name)
            total = sum(m.body_mass[b] for b in bodies)
            ratio = float(spec["mass"]) / max(total, 1e-9)
            for b in bodies:
                m.body_mass[b] *= ratio
                m.body_inertia[b] *= ratio
        if "friction" in spec:
            for g in genv.geoms_of(name, collision=True):
                m.geom_friction[g, 0] = float(spec["friction"])
        if "damping" in spec:  # extension: joint damping of an articulated object (e.g. a stiff drawer)
            for j in genv.joints_of(name):
                m.dof_damping[m.jnt_dofadr[j]] = float(spec["damping"])
    if edits.get("dynamics"):  # mj_setConst overwrites qpos/qvel with qpos0; keep the episode state
        qpos, qvel, t = d.qpos.copy(), d.qvel.copy(), d.time
        mujoco.mj_setConst(m, d)
        d.qpos[:], d.qvel[:], d.time = qpos, qvel, t
    mujoco.mj_forward(m, d)


def apply_motion(genv, t: float):
    """Called before every physics substep (500 Hz) with the simulated time since the pipeline started."""
    for i, spec in enumerate(genv.edits.get("object_motion", [])):
        name = genv.resolve(spec["name"])
        if not genv.is_free(name):
            raise UnsupportedEdit(f"object_motion: '{name}' is fixed in place")
        start = float(spec.get("start_time", 0.0))
        dur = float(spec.get("duration", 2.0))
        mode = spec.get("mode", "constant")
        v = np.asarray(spec.get("velocity", [0, 0, 0]), float)
        if mode == "impulse":
            if t >= start and not genv.motion_fired.get(i):
                genv.set_free_velocity(name, v)
                genv.motion_fired[i] = True
        elif start <= t <= start + dur:
            genv.set_free_velocity(name, v, keep_z=v[2] == 0)
