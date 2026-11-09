"""Scene edits for the RoboPRO backend (docs/api_spec.md, Scene edits). See SCENE_EDITS.md.

apply(env, edits, rng) runs after RoboPRO has built and settled the scene of one seed. It edits the
scene with SAPIEN calls, lets it settle again and returns a state dict for api.RobotAPI:
  {"added": [(owner, rigid component)], "motions": [Motion], "drift": {owner: metres}, "log": [...]}.
Ported from the RoboTwin 2.0 backend (same SAPIEN calls); only the name resolution is RoboPRO-specific.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import sapien
import transforms3d as t3d

KEYS = ["object_pose", "object_color", "object_scale", "swap_object", "lighting", "add_distractor",
        "add_obstacle", "object_motion", "camera_pose", "dynamics"]
SETTLE_STEPS = 250          # 1 s at 250 Hz after the edits
TABLE_Z = 0.74


class UnsupportedEdit(Exception):
    pass


def normalize(edits: dict) -> dict:
    """{key: [spec, ...]}; raises UnsupportedEdit for unknown keys or malformed specs."""
    out = {}
    for k, v in (edits or {}).items():
        if k not in KEYS:
            raise UnsupportedEdit(f"unknown edit key {k!r}; supported: {KEYS}")
        specs = v if isinstance(v, list) else [v]
        for s in specs:
            if not isinstance(s, dict):
                raise UnsupportedEdit(f"{k}: each spec must be a mapping, got {s!r}")
            need = {"object_pose": ["name"], "object_color": ["name", "rgb"], "object_scale": ["name", "scale"],
                    "swap_object": ["name", "with"], "lighting": [], "add_distractor": ["shape", "size", "pose"],
                    "add_obstacle": ["shape", "size", "pose"], "object_motion": ["name", "velocity"],
                    "camera_pose": [], "dynamics": ["name"]}[k]
            miss = [f for f in need if f not in s]
            if miss:
                raise UnsupportedEdit(f"{k}: missing fields {miss}")
        out[k] = specs
    return out


# --------------------------------------------------------------------------- #
# name resolution
# --------------------------------------------------------------------------- #
def _rigid(entity):
    return entity.find_component_by_type(sapien.physx.PhysxRigidBaseComponent)


# task objects that may be moved, recoloured, re-scaled or swapped: name -> env attribute (RoboPRO Actor)
TASK_OBJECTS = {"put_cup_on_coaster": {"cup": "target_obj", "coaster": "des_obj"}}
ALIASES = {"021_cup": "cup", "019_coaster": "coaster", "object": "cup",
           "014_bookcase": "bookcase", "042_wooden_box": "wooden_box", "box": "wooden_box"}
FURNITURE = {"bookcase": "bookcase", "wooden_box": "box"}     # static study-scene furniture


def _task_objects(env) -> dict:
    return TASK_OBJECTS.get(getattr(env, "_gah_task", ""), {})


def _resolve(env, name, added):
    """-> ("actor", entity). Names: task objects (cup, coaster), furniture (bookcase, wooden_box), table,
    wall, and bodies added by this variation."""
    if name in added:
        return "actor", added[name]
    name = ALIASES.get(name, name)
    objs = _task_objects(env)
    if name in objs:
        return "actor", getattr(env, objs[name]).actor
    if name in FURNITURE and hasattr(env, FURNITURE[name]):
        ent = getattr(env, FURNITURE[name])
        return "actor", ent.actor if hasattr(ent, "actor") else ent
    if name in ("table", "wall"):
        ent = getattr(env, name)
        return "actor", ent.actor if hasattr(ent, "actor") else ent
    raise UnsupportedEdit(f"unknown object name {name!r}; known: {sorted(objs) + sorted(FURNITURE) + ['table', 'wall']}")


def _pose_from(spec_pose, rpy=None) -> sapien.Pose:
    a = np.asarray(spec_pose, float).reshape(-1)
    q = a[3:7] if a.size == 7 else np.array([1.0, 0, 0, 0])
    if rpy is not None:
        q = t3d.quaternions.qmult(t3d.euler.euler2quat(*rpy, "sxyz"), q)
    return sapien.Pose(a[:3], q / np.linalg.norm(q))


# --------------------------------------------------------------------------- #
# motions (applied by api.RobotAPI._step every simulation step)
# --------------------------------------------------------------------------- #
class Motion:
    def __init__(self, kind, target, velocity, start_time=0.0, duration=2.0, mode="constant"):
        self.kind, self.target = kind, target
        self.v = np.asarray(velocity, float)
        self.t0, self.dur, self.mode = float(start_time), float(duration), mode
        self.fired = False

    def apply(self, t):
        if t < self.t0 or (self.mode == "constant" and t > self.t0 + self.dur):
            return
        if self.mode == "impulse":
            if self.fired:
                return
            self.fired = True
        if self.kind == "articulation":
            pose = self.target.get_root_pose()
            self.target.set_root_pose(sapien.Pose(pose.p + self.v / 250.0, pose.q))
        else:
            comp = self.target.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
            if comp is not None:
                comp.set_linear_velocity(self.v)


# --------------------------------------------------------------------------- #
# object creation
# --------------------------------------------------------------------------- #
def _build_model(scene, modelname, model_id, scale_mult, pose, is_static=False):
    """RoboTwin's create_actor with an extra scale factor. Returns (Actor, entity)."""
    from envs.utils.actor_utils import Actor
    from envs.utils.create_actor import get_glb_or_obj_file
    modeldir = Path("assets/objects") / modelname
    if not modeldir.exists():
        cands = [d.name for d in Path("assets/objects").iterdir() if d.name.split("_", 1)[-1] == modelname]
        if not cands:
            raise UnsupportedEdit(f"no asset named {modelname!r} in assets/objects")
        modeldir = Path("assets/objects") / cands[0]
        modelname = cands[0]
    if model_id is None:
        ids = sorted(int(p.stem.replace("model_data", "")) for p in modeldir.glob("model_data*.json")
                     if p.stem.replace("model_data", "").isdigit())
        model_id = ids[0] if ids else None
    jf = modeldir / (f"model_data{model_id}.json" if model_id is not None else "model_data.json")
    data = json.loads(jf.read_text()) if jf.exists() else {"scale": [1, 1, 1]}
    scale = np.array(data.get("scale", [1, 1, 1]), float) * float(scale_mult)
    data["scale"] = scale.tolist()
    col = get_glb_or_obj_file(modeldir / "collision", model_id) if (modeldir / "collision").exists() else ""
    if col == "" or not Path(col).exists():
        col = get_glb_or_obj_file(modeldir, model_id)
    vis = get_glb_or_obj_file(modeldir / "visual", model_id) if (modeldir / "visual").exists() else ""
    if vis == "" or not Path(vis).exists():
        vis = get_glb_or_obj_file(modeldir, model_id)
    b = scene.create_actor_builder()
    b.set_physx_body_type("static" if is_static else "dynamic")
    b.add_multiple_convex_collisions_from_file(filename=str(col), scale=scale.tolist())
    b.add_visual_from_file(filename=str(vis), scale=scale.tolist())
    ent = b.build(name=modelname)
    ent.set_name(modelname)
    ent.set_pose(pose)
    return Actor(ent, data), ent


def _primitive(scene, shape, size, pose, rgb, is_static, name, mass):
    from envs.utils.create_actor import create_box, create_cylinder, create_sphere
    size = np.asarray(size, float).reshape(-1)
    color = list(rgb) if rgb is not None else [0.6, 0.4, 0.2]
    if shape == "box":
        hs = size / 2 if size.size == 3 else np.full(3, size[0] / 2)
        act = create_box(scene, pose, half_size=hs.tolist(), color=color, is_static=is_static, name=name)
    elif shape == "sphere":
        act = create_sphere(scene, pose, radius=float(size[0]) / 2, color=color, is_static=is_static, name=name)
    elif shape == "cylinder":
        d, L = (float(size[0]), float(size[1])) if size.size >= 2 else (float(size[0]), float(size[0]))
        act = create_cylinder(scene, pose, radius=d / 2, half_length=L / 2, color=color, is_static=is_static, name=name)
    elif shape.startswith("asset:"):
        act, _ = _build_model(scene, shape.split(":", 1)[1], None, float(size[0]) if size.size == 1 else 1.0,
                              pose, is_static=is_static)
        act.actor.set_name(name)
    else:
        raise UnsupportedEdit(f"unknown shape {shape!r}: box | sphere | cylinder | asset:<model>")
    ent = act.actor if hasattr(act, "actor") else act
    ent.set_name(name)
    if not is_static:
        comp = ent.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        comp.mass = float(mass)
    return ent


def _set_color(entity, rgb):
    rb = entity.find_component_by_type(sapien.render.RenderBodyComponent)
    if rb is None:
        return
    for shape in rb.render_shapes:
        parts = shape.parts if hasattr(shape, "parts") else [shape]
        for part in parts:
            mat = part.material
            mat.base_color = [*map(float, rgb[:3]), 1.0]
            try:
                mat.base_color_texture = None
            except Exception:
                pass


def _set_friction(scene, comp, friction):
    mat = scene.create_physical_material(float(friction), float(friction), 0.0)
    for s in comp.get_collision_shapes():
        s.set_physical_material(mat)


# --------------------------------------------------------------------------- #
def apply(env, edits: dict, rng=None) -> dict:
    edits = normalize(edits)
    scene = env.scene
    state = {"added": [], "motions": [], "drift": {}, "log": []}
    added = {}

    def note(msg):
        state["log"].append(msg)

    objs = _task_objects(env)
    for spec in edits.get("swap_object", []) + edits.get("object_scale", []):
        if ALIASES.get(spec["name"], spec["name"]) not in objs:
            raise UnsupportedEdit(f"swap_object/object_scale only support the task objects {sorted(objs)}, "
                                  f"not {spec['name']!r}")

    # 1. replace / rescale a task object (the success check reads the env attribute, so it follows)
    for key in ("swap_object", "object_scale"):
        for spec in edits.get(key, []):
            attr = objs[ALIASES.get(spec["name"], spec["name"])]
            old_act = getattr(env, attr)
            old = old_act.actor
            pose = old.get_pose()
            cur_model = old.get_name()
            cur_id = getattr(env, "target_id" if attr == "target_obj" else "des_obj_id", None)
            model = spec["with"] if key == "swap_object" else cur_model
            mid = spec.get("model_id") if key == "swap_object" else cur_id
            mult = float(spec.get("scale", 1.0))
            mass = float(old.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass) \
                if old.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent) is not None else 0.1
            scene.remove_entity(old)
            p = np.array(pose.p, float)
            p[2] += 0.01 * mult                     # drop it from just above the table
            act, ent = _build_model(scene, model, mid, mult, sapien.Pose(p, pose.q))
            act.set_mass(mass)
            setattr(env, attr, act)
            if attr == "target_obj":
                env.target_name, env.target_id = ent.get_name(), mid
            else:
                env.des_obj_id = mid
            note(f"{key}: {spec['name']} -> {model}/{mid} x{mult}")

    # 2. poses
    for spec in edits.get("object_pose", []):
        _, h = _resolve(env, spec["name"], added)
        pose = h.get_pose()
        p, q = np.array(pose.p, float), np.array(pose.q, float)
        if "xyz" in spec:
            xyz = np.asarray(spec["xyz"], float)
            p[:len(xyz)] = xyz
        if "delta_xyz" in spec:
            p += np.asarray(spec["delta_xyz"], float)
        if "rpy" in spec:
            q = t3d.euler.euler2quat(*spec["rpy"], "sxyz")
        if "delta_rpy" in spec:
            q = t3d.quaternions.qmult(t3d.euler.euler2quat(*spec["delta_rpy"], "sxyz"), q)
        h.set_pose(sapien.Pose(p, q))

    # 3. added objects
    for key, static in (("add_distractor", False), ("add_obstacle", True)):
        for i, spec in enumerate(edits.get(key, [])):
            name = spec.get("name") or f"{'distractor' if not static else 'obstacle'}_{i}"
            pose = _pose_from(spec["pose"], spec.get("rpy"))
            ent = _primitive(scene, spec["shape"], spec["size"], pose, spec.get("rgb"), static, name,
                             spec.get("mass", 0.1))
            added[name] = ent
            state["added"].append((name, _rigid(ent)))

    # 4. appearance
    for spec in edits.get("object_color", []):
        _, h = _resolve(env, spec["name"], added)
        _set_color(h, spec["rgb"])
    for spec in edits.get("lighting", []):
        k = float(spec.get("intensity", 1.0))
        col = np.asarray(spec.get("color", [1, 1, 1]), float)
        scene.set_ambient_light((np.asarray(scene.ambient_light, float) * k * col).tolist())
        for l in list(getattr(env, "direction_light_lst", [])) + list(getattr(env, "point_light_lst", [])):
            l.set_color((np.asarray(l.get_color(), float) * k * col).tolist())
    for spec in edits.get("camera_pose", []):
        if spec.get("view", "head") != "head":
            raise UnsupportedEdit("camera_pose: only view=head is a fixed camera (wrist cameras ride on the arms)")
        cams = env.cameras
        cam = cams.static_camera_list[cams.static_camera_name.index("head_camera")]
        pose = cam.entity.get_pose()
        dq = t3d.euler.euler2quat(*spec.get("delta_rpy", [0, 0, 0]), "sxyz")
        cam.entity.set_pose(sapien.Pose(pose.p + np.asarray(spec.get("delta_xyz", [0, 0, 0]), float),
                                        t3d.quaternions.qmult(dq, pose.q)))

    # 5. dynamics
    for spec in edits.get("dynamics", []):
        _, h = _resolve(env, spec["name"], added)
        if "damping" in spec:
            raise UnsupportedEdit("dynamics: damping applies to joints; this scene has no articulated objects")
        comp = h.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        if comp is None and "mass" in spec:
            raise UnsupportedEdit(f"dynamics: {spec['name']!r} is static")
        if "mass" in spec:
            comp.mass = float(spec["mass"])
        if "friction" in spec:
            _set_friction(scene, _rigid(h), spec["friction"])

    # 6. motions (start at simulated time start_time after the pipeline starts)
    for spec in edits.get("object_motion", []):
        kind, h = _resolve(env, spec["name"], added)
        if h.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent) is None:
            raise UnsupportedEdit(f"object_motion: {spec['name']!r} is static; use add_distractor")
        state["motions"].append(Motion(kind, h, spec["velocity"], spec.get("start_time", 0.0),
                                       spec.get("duration", 2.0), spec.get("mode", "constant")))

    # 7. settle and measure drift (interpenetration shows up as a jump)
    tracked = {n: getattr(env, a).actor for n, a in objs.items()}
    tracked.update({n: e for n, e in added.items()})
    before = {n: np.array(e.get_pose().p, float) for n, e in tracked.items()}
    if edits:
        for _ in range(SETTLE_STEPS):
            scene.step()
    for n, e in tracked.items():
        state["drift"][n] = float(np.linalg.norm(np.array(e.get_pose().p, float) - before[n]))
    state["edits"] = edits
    return state


def scene_validity(env, state) -> tuple[bool, str]:
    """Reject scenes a real setup could not have: task objects off or under the table, out of reach, or
    bodies that jumped while settling (interpenetration)."""
    for name, attr in _task_objects(env).items():
        p = np.array(getattr(env, attr).actor.get_pose().p, float)
        if p[2] < TABLE_Z - 0.01:
            return False, f"{name} below the table top (z={p[2]:.3f})"
        if not (-0.55 <= p[0] <= 0.55 and -0.33 <= p[1] <= 0.33):
            return False, f"{name} off the table or out of reach at {np.round(p, 3).tolist()}"
    for name, d in state.get("drift", {}).items():
        if d > 0.03:
            return False, f"{name} moved {d:.3f} m while settling (interpenetration or unsupported)"
    for name, comp in state.get("added", []):
        z = comp.entity.get_pose().p[2]
        if z < TABLE_Z - 0.05:
            return False, f"{name} fell off the table"
    return True, "ok"
