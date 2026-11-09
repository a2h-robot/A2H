"""Scene edits for the RoboPRO backend (docs/api_spec.md, Scene edits). See SCENE_EDITS.md.

apply(env, edits, rng) runs after RoboPRO has built and settled the scene of one seed. It edits the
scene with SAPIEN calls, lets it settle again and returns a state dict for api.RobotAPI:
  {"added": [(owner, rigid component)], "motions": [Motion], "drift": {owner: metres}, "log": [...]}.
Everything here uses plain SAPIEN entities/components, so it also works for other SAPIEN scenes
(ManiSkill3) given a name -> entity resolver.
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


def _canon(env, name):
    import gah_env
    return gah_env.aliases(env).get(name, name)


def _main_name(env):
    import gah_env
    return next(iter(gah_env.MAIN_OBJECTS & set(gah_env.named_entities(env))), None)


def _resolve(env, name, added):
    """-> ("actor", entity) | ("articulation", PhysxArticulation)."""
    import gah_env
    if name in added:
        return "actor", added[name]
    named = gah_env.named_entities(env)
    canon = _canon(env, name)
    if canon in named:
        return named[canon]
    raise UnsupportedEdit(f"unknown object name {name!r}; known: {sorted(named) + sorted(added)}")


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
def _asset_dir(modelname):
    from envs._GLOBAL_CONFIGS import ASSETS_PATH
    root = Path(ASSETS_PATH) / "objects"
    if (root / modelname).exists():
        return modelname
    cands = sorted(d.name for d in root.iterdir() if d.name.split("_", 1)[-1] == modelname)
    if not cands:
        raise UnsupportedEdit(f"no asset named {modelname!r} in the RoboPRO assets")
    return cands[0]


def _build_model(env, modelname, model_id, scale_mult, pose, is_static=False):
    """RoboPRO's create_actor (convex collisions) with an extra scale factor on the model's own scale.
    Returns (Actor, entity)."""
    from envs.utils.create_actor import create_actor
    modelname = _asset_dir(modelname)
    if model_id is None:
        from envs._GLOBAL_CONFIGS import ASSETS_PATH
        d = Path(ASSETS_PATH) / "objects" / modelname
        ids = sorted(int(p.stem.replace("model_data", "")) for p in d.glob("model_data*.json")
                     if p.stem.replace("model_data", "").isdigit())
        model_id = ids[0] if ids else 0
    scale = float(env._get_asset_model_scale_create_actor(modelname, int(model_id))) * float(scale_mult)
    act = create_actor(scene=env.scene, pose=pose, modelname=modelname, model_id=int(model_id),
                       is_static=is_static, convex=True, scale=scale)
    if act is None:
        raise UnsupportedEdit(f"could not build {modelname} model {model_id}")
    if isinstance(act.config, dict):
        act.config["scale"] = [scale] * 3
    return act, act.actor


def _cylinder(scene, pose, radius, half_length, color, is_static, name):
    """A cylinder (axis along its local x), static or dynamic. RoboPRO's create_cylinder is dynamic only."""
    ent = sapien.Entity()
    ent.set_name(name)
    body = sapien.physx.PhysxRigidStaticComponent() if is_static else sapien.physx.PhysxRigidDynamicComponent()
    body.attach(sapien.physx.PhysxCollisionShapeCylinder(radius=radius, half_length=half_length,
                                                        material=scene.default_physical_material))
    render = sapien.render.RenderBodyComponent()
    render.attach(sapien.render.RenderShapeCylinder(radius=radius, half_length=half_length,
                                                   material=sapien.render.RenderMaterial(base_color=[*color[:3], 1])))
    ent.add_component(body)
    ent.add_component(render)
    ent.set_pose(pose)
    scene.add_entity(ent)
    return ent


def _primitive(env, shape, size, pose, rgb, is_static, name, mass):
    scene = env.scene
    from envs.utils.create_actor import create_box, create_sphere
    size = np.asarray(size, float).reshape(-1)
    color = list(rgb) if rgb is not None else [0.6, 0.4, 0.2]
    if shape == "box":
        hs = size / 2 if size.size == 3 else np.full(3, size[0] / 2)
        act = create_box(scene, pose, half_size=hs.tolist(), color=color, is_static=is_static, name=name)
    elif shape == "sphere":
        act = create_sphere(scene, pose, radius=float(size[0]) / 2, color=color, is_static=is_static, name=name)
    elif shape == "cylinder":
        d, L = (float(size[0]), float(size[1])) if size.size >= 2 else (float(size[0]), float(size[0]))
        act = _cylinder(scene, pose, d / 2, L / 2, color, is_static, name)
    elif shape.startswith("asset:"):
        act, _ = _build_model(env, shape.split(":", 1)[1], None, float(size[0]) if size.size == 1 else 1.0,
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
def _inside_xy(entity, container) -> bool:
    """True if `entity`'s origin lies over `container`'s world bounding box (x, y)."""
    import gah_env
    lo, hi = gah_env._mesh_world_aabb(_rigid(container))
    p = np.asarray(entity.get_pose().p, float)
    return bool(np.all(lo[:2] <= p[:2]) and np.all(p[:2] <= hi[:2]))


def apply(env, edits: dict, rng=None) -> dict:
    import gah_env
    edits = normalize(edits)
    scene = env.scene
    state = {"added": [], "motions": [], "drift": {}, "log": []}
    added = {}
    main = _main_name(env)

    def note(msg):
        state["log"].append(msg)

    for spec in edits.get("swap_object", []) + edits.get("object_scale", []):
        if _canon(env, spec["name"]) != main:
            raise UnsupportedEdit(f"swap_object/object_scale only support the task object ({main}), "
                                  f"not {spec['name']!r}")

    # 1. replace / rescale the task object (same pose; its bottom stays where it was)
    for key in ("swap_object", "object_scale"):
        for spec in edits.get(key, []):
            old = gah_env.main_actor(env)
            pose = old.get_pose()
            model = spec["with"] if key == "swap_object" else env.boxdrink_modelname
            mid = spec.get("model_id") if key == "swap_object" else env.boxdrink_model_id
            mult = float(spec.get("scale", 1.0)) * (float(env.boxdrink_scale) if key == "object_scale" else 1.0)
            mass = float(old.actor.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass)
            scene.remove_entity(old.actor)
            act, ent = _build_model(env, model, mid, mult, sapien.Pose(np.asarray(pose.p, float) + [0, 0, 0.005],
                                                                     pose.q))
            act.set_mass(mass)
            ent.set_name("task_boxdrink")
            env.boxdrink = act
            env.boxdrink_modelname, env.boxdrink_model_id = ent.get_name(), mid
            env._ensure_boxdrink_grasp_metadata()
            note(f"{key}: {model} x{mult}")

    # 2. poses. Moving a container carries what lies in it.
    for spec in edits.get("object_pose", []):
        kind, h = _resolve(env, spec["name"], added)
        if kind == "articulation":
            raise UnsupportedEdit(f"object_pose: {spec['name']!r} is fixed furniture; move the basket or an object")
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
        new = sapien.Pose(p, q)
        if _canon(env, spec["name"]) == "basket" and main is not None:
            obj = gah_env.main_actor(env).actor
            if _inside_xy(obj, h):
                rel = pose.inv() * obj.get_pose()
                h.set_pose(new)
                obj.set_pose(new * rel)
                note("object_pose: basket moved with its contents")
                continue
        h.set_pose(new)

    # 3. added objects. pose: [x, y, z(, qw, qx, qy, qz)] in the world, or {relative_to: <name>,
    # offset: [dx, dy, dz], z?} = the named object's origin plus a world-frame offset (layouts differ per
    # seed), with an optional absolute height z
    for key, static in (("add_distractor", False), ("add_obstacle", True)):
        for i, spec in enumerate(edits.get(key, [])):
            name = spec.get("name") or f"{'distractor' if not static else 'obstacle'}_{i}"
            if isinstance(spec["pose"], dict):
                ref = spec["pose"].get("relative_to")
                if ref is None:
                    raise UnsupportedEdit(f"{key}: a pose mapping needs relative_to (and offset)")
                kind, h = _resolve(env, ref, added)
                base = h.get_root_pose() if kind == "articulation" else h.get_pose()
                p = np.asarray(base.p, float) + np.asarray(spec["pose"].get("offset", [0, 0, 0]), float)
                if "z" in spec["pose"]:                  # absolute height, e.g. standing on the table
                    p[2] = float(spec["pose"]["z"])
                pose = _pose_from(p, spec.get("rpy"))
            else:
                pose = _pose_from(spec["pose"], spec.get("rpy"))
            ent = _primitive(env, spec["shape"], spec["size"], pose, spec.get("rgb"), static, name,
                             spec.get("mass", 0.1))
            added[name] = ent
            state["added"].append((name, _rigid(ent)))

    # 4. appearance
    for spec in edits.get("object_color", []):
        kind, h = _resolve(env, spec["name"], added)
        if kind == "articulation":
            for l in h.get_links():
                _set_color(l.entity, spec["rgb"])
        else:
            _set_color(h, spec["rgb"])
    for spec in edits.get("lighting", []):
        k = float(spec.get("intensity", 1.0))
        col = np.asarray(spec.get("color", [1, 1, 1]), float)
        scene.set_ambient_light((np.asarray(scene.ambient_light, float) * k * col).tolist())
        for l in list(getattr(env, "direction_light_lst", [])) + list(getattr(env, "point_light_lst", [])):
            l.set_color((np.asarray(l.get_color(), float) * k * col).tolist())
    for spec in edits.get("camera_pose", []):
        cams = env.cameras
        view = spec.get("view", "head")
        if f"{view}_camera" not in cams.static_camera_name:
            raise UnsupportedEdit(f"camera_pose: view must be a fixed camera "
                                  f"{[n.replace('_camera', '') for n in cams.static_camera_name]} "
                                  f"(wrist cameras ride on the arms)")
        cam = cams.static_camera_list[cams.static_camera_name.index(f"{view}_camera")]
        pose = cam.entity.get_pose()
        dq = t3d.euler.euler2quat(*spec.get("delta_rpy", [0, 0, 0]), "sxyz")
        cam.entity.set_pose(sapien.Pose(pose.p + np.asarray(spec.get("delta_xyz", [0, 0, 0]), float),
                                        t3d.quaternions.qmult(dq, pose.q)))

    # 5. dynamics
    for spec in edits.get("dynamics", []):
        kind, h = _resolve(env, spec["name"], added)
        if kind == "articulation":
            raise UnsupportedEdit("dynamics: name an object (mass, friction), not fixed furniture")
        comp = h.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        if comp is None and "mass" in spec:
            raise UnsupportedEdit(f"dynamics: {spec['name']!r} is static (no mass)")
        if "mass" in spec:
            comp.mass = float(spec["mass"])
        if "friction" in spec:
            _set_friction(scene, _rigid(h), spec["friction"])

    # 6. motions (start at simulated time start_time after the pipeline starts)
    for spec in edits.get("object_motion", []):
        kind, h = _resolve(env, spec["name"], added)
        if kind == "articulation" or h.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent) is None:
            raise UnsupportedEdit(f"object_motion: {spec['name']!r} is static; move a free object or add_distractor")
        state["motions"].append(Motion(kind, h, spec["velocity"], spec.get("start_time", 0.0),
                                       spec.get("duration", 2.0), spec.get("mode", "constant")))

    # 7. settle and measure drift (interpenetration shows up as a jump)
    tracked = {main: gah_env.main_actor(env).actor} if main else {}
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
    """Reject scenes a real setup could not have: objects off or under the table, out of reach, or
    objects that jumped while settling (interpenetration)."""
    import gah_env
    table_z = float(env._gah_table_z)
    main = _main_name(env)
    if main:
        p = np.array(gah_env.main_actor(env).actor.get_pose().p, float)
        if p[2] < table_z - 0.01:
            return False, f"{main} below the table top (z={p[2]:.3f})"
        if not (-0.55 <= p[0] <= 0.55 and -0.33 <= p[1] <= 0.33):
            return False, f"{main} off the table or out of reach at {np.round(p, 3).tolist()}"
    if hasattr(env, "basket_right"):
        c = np.array(env.basket_right.get_pose().p, float)
        if not (-0.55 <= c[0] <= 0.55 and -0.33 <= c[1] <= 0.33) or c[2] < table_z - 0.01:
            return False, f"basket off the table or out of reach at {np.round(c, 3).tolist()}"
    for name, d in state.get("drift", {}).items():
        if d > 0.03:
            return False, f"{name} moved {d:.3f} m while settling (interpenetration or unsupported)"
    for name, comp in state.get("added", []):
        z = comp.entity.get_pose().p[2]
        if z < table_z - 0.05:
            return False, f"{name} fell off the table"
    return True, "ok"
