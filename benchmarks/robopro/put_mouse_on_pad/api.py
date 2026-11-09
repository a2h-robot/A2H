"""Primitive API for the RoboPRO backend (RoboTwin 2.0 fork; aloha-agilex, two 6-DoF arms).

Implements docs/api_spec.md. Pipelines define `run(api)` and use only this object and the pose helpers
in this module. The robot base is at y = -0.65 and faces +y; the table top is at z = 0.74;
x > 0 is the robot's right side.
Ported from the RoboTwin 2.0 GAH backend (benchmarks/robotwin on the b-robotwin-* branches, backend v2).

Conventions
- Positions are world xyz in metres; quaternions are wxyz (SAPIEN order).
- A gripper pose is the pose of the grasp point (TCP, the point between the fingertips).
  Its x axis is the approach direction (from the wrist towards the fingertips); the fingers close
  along its y axis; z = x cross y. `gripper_quat(approach, closing)` builds such a quaternion.
- Every function that moves the robot advances the simulation. Perception does not.

Perception is not ground truth: detect(), get_obstacles() and track() return the simulator pose plus
fresh zero-mean Gaussian noise (default 1 cm, 3 deg), independently on every call.
Proprioception (tcp_pose, gripper_opening, gripper_holding) is exact, as on a real robot.

Motion planning is curobo (collision-aware). The planner world is the true scene geometry (convex
pieces as boxes) minus the robot, minus what the arm holds or touches, minus `allow_contact`.
Safety limits are enforced here: a motion stops when the robot pushes a non-allowed object or the
table with more than FORCE_LIMIT newtons, and gripper targets must lie in WORKSPACE.

Extensions beyond docs/api_spec.md (same as the RoboTwin backend, see ISSUES.md):
  objects(), regions(), tcp_pose(), gripper_opening(), move_by(), home(), sim_time(), last_error,
  move_to(linear=, allow_contact=), grasp(pre_dis=, depth=, contact_point=), place(target is the
  OBJECT position, yaw=, pre_dis=, release=, retreat=), pose helpers (pose_mul, pose_inv,
  rpy_to_quat, quat_to_rpy, gripper_quat, quat_rotate).
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import sapien
import transforms3d as t3d

import gah_env

FORCE_LIMIT = float(os.environ.get("GAH_FORCE_LIMIT", 40.0))   # N
WORKSPACE = {"x": (-0.70, 0.70), "y": (-0.50, 0.45), "z": (0.74, 1.40)}  # gripper (TCP) target bounds
SPEED_REPEAT = {"slow": 2, "normal": 1}   # sim steps per trajectory point (planner runs at 250 Hz)
TCP_OFFSET = 0.12                          # m between RoboTwin's end-effector frame and the TCP
MIN_TCP_ABOVE_TABLE = 0.012                # grasp points are raised to at least this height
FINGER_CLEARANCE = 0.004                   # lowest finger point above the table top at a grasp


def finger_corners(opening: float) -> np.ndarray:
    """Corners (TCP frame) of the two finger boxes of an aloha gripper at an opening in [0, 1]:
    fingertips reach 3.6 cm past the TCP; the inner gap is 9 cm * opening."""
    out = []
    for sgn in (1, -1):
        y_in = 0.045 * opening
        for x in (-0.051, 0.036):
            for y in (y_in, y_in + 0.037):
                for z in (-0.031, 0.031):
                    out.append([x, sgn * y, z])
    return np.array(out)
DT = 1.0 / 250.0
ARMS = ("left", "right")
# never treated as touchable just because the arm touches them (study and office furniture; the office drawer
# slides, so it is not here)
FIXED_SCENE = {"table", "wall", "bookcase", "wooden_box", "shelf", "cabinet", "file_holder"}


@dataclass
class ObjectState:
    name: str
    position: np.ndarray      # (3,) world xyz of the bounding-box centre [m]
    quaternion: np.ndarray    # (4,) wxyz
    bbox_extents: np.ndarray  # (3,) full extents of the oriented bounding box, in the object frame [m]
    confidence: float = 1.0


@dataclass
class Box:
    name: str
    center: np.ndarray        # (3,)
    quaternion: np.ndarray    # (4,) wxyz
    extents: np.ndarray       # (3,) full extents [m]


# --------------------------------------------------------------------------- #
# Pose helpers (poses are (p, q) tuples, q in wxyz)
# --------------------------------------------------------------------------- #
def _pose7(pose) -> np.ndarray:
    if isinstance(pose, sapien.Pose):
        return np.r_[pose.p, pose.q].astype(np.float64)
    if isinstance(pose, ObjectState):
        return np.r_[pose.position, pose.quaternion].astype(np.float64)
    if isinstance(pose, Box):
        return np.r_[pose.center, pose.quaternion].astype(np.float64)
    if isinstance(pose, dict):
        return np.r_[pose["p"], pose.get("q", [1, 0, 0, 0])].astype(np.float64)
    if isinstance(pose, tuple) and len(pose) == 2:
        return np.r_[np.asarray(pose[0], float), np.asarray(pose[1], float)]
    arr = np.asarray(pose, dtype=np.float64)
    if arr.shape == (4, 4):
        return np.r_[arr[:3, 3], t3d.quaternions.mat2quat(arr[:3, :3])]
    if arr.size == 7:
        return arr.reshape(7)
    if arr.size == 3:
        return np.r_[arr.reshape(3), [1.0, 0, 0, 0]]
    raise ValueError(f"cannot interpret pose {pose!r}")


def _mat(pose) -> np.ndarray:
    p7 = _pose7(pose)
    T = np.eye(4)
    q = p7[3:] / np.linalg.norm(p7[3:])
    T[:3, :3] = t3d.quaternions.quat2mat(q)
    T[:3, 3] = p7[:3]
    return T


def _tuple(T) -> tuple:
    return np.asarray(T[:3, 3], float).copy(), np.asarray(t3d.quaternions.mat2quat(T[:3, :3]), float)


def pose_mul(a, b):
    """a * b (apply b in a's frame)."""
    return _tuple(_mat(a) @ _mat(b))


def pose_inv(a):
    return _tuple(np.linalg.inv(_mat(a)))


def rpy_to_quat(roll, pitch, yaw):
    """Static-frame xyz Euler angles [rad] -> quaternion wxyz."""
    return np.array(t3d.euler.euler2quat(roll, pitch, yaw, "sxyz"))


def quat_to_rpy(q):
    return np.array(t3d.euler.quat2euler(np.asarray(q, float), "sxyz"))


def quat_rotate(q, v):
    """Rotate vector v by quaternion q (wxyz)."""
    return t3d.quaternions.quat2mat(np.asarray(q, float)) @ np.asarray(v, float)


def gripper_quat(approach, closing=None):
    """Quaternion of a gripper whose fingers point along `approach` and close along `closing`.
    If `closing` is None, a horizontal closing axis is chosen."""
    x = np.asarray(approach, float)
    x = x / np.linalg.norm(x)
    if closing is None:
        closing = np.cross([0, 0, 1.0], x)
        if np.linalg.norm(closing) < 1e-6:          # vertical approach: close along world x
            closing = np.array([1.0, 0, 0])
    y = np.asarray(closing, float)
    y = y - x * np.dot(x, y)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    return np.array(t3d.quaternions.mat2quat(np.stack([x, y, z], axis=1)))


def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


# --------------------------------------------------------------------------- #
# Collision geometry of scene bodies (local boxes per collision shape)
# --------------------------------------------------------------------------- #
def _shape_local_boxes(comp):
    """[(sapien.Pose in the body frame, half_size)] for each collision shape of a rigid component."""
    out = []
    for s in comp.get_collision_shapes():
        lp = s.get_local_pose()
        name = type(s).__name__
        if name.endswith("Box"):
            out.append((lp, np.asarray(s.half_size, float)))
        elif name.endswith("ConvexMesh"):
            v = np.asarray(s.get_vertices(), float) * np.asarray(s.get_scale(), float)
            lo, hi = v.min(0), v.max(0)
            out.append((lp * sapien.Pose((lo + hi) / 2), (hi - lo) / 2))
        elif name.endswith("Sphere"):
            out.append((lp, np.full(3, float(s.radius))))
        elif name.endswith("Cylinder") or name.endswith("Capsule"):
            r, h = float(s.radius), float(s.half_length)
            out.append((lp, np.array([h + (r if name.endswith("Capsule") else 0), r, r])))
        elif name.endswith("TriangleMesh"):
            # one box per connected part: a non-convex mesh (the office wall shelf: two boards and two side
            # panels) would otherwise become one solid box that fills the space between the boards
            v = np.asarray(s.get_vertices(), float) * np.asarray(s.get_scale(), float)
            for idx in _mesh_parts(np.asarray(s.get_triangles()), len(v)):
                lo, hi = v[idx].min(0), v[idx].max(0)
                out.append((lp * sapien.Pose((lo + hi) / 2), (hi - lo) / 2))
    return out


def _mesh_parts(tris, n_vertices):
    """Vertex index arrays of the connected parts of a triangle mesh."""
    parent = np.arange(n_vertices)

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b, c in tris:
        ra, rb, rc = find(a), find(b), find(c)
        parent[rb] = ra
        parent[find(rc)] = ra
    roots = np.array([find(i) for i in range(n_vertices)])
    used = np.unique(np.asarray(tris).reshape(-1))
    return [used[roots[used] == r] for r in np.unique(roots[used])]


def _union_box(boxes):
    """One box (pose in the body frame, half size) that contains all local boxes."""
    corners = []
    for lp, hs in boxes:
        T = lp.to_transformation_matrix()
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    corners.append(T[:3, :3] @ (hs * [sx, sy, sz]) + T[:3, 3])
    corners = np.array(corners)
    lo, hi = corners.min(0), corners.max(0)
    return sapien.Pose((lo + hi) / 2), (hi - lo) / 2


def _traced(fn):
    """Record public API calls in api.trace (the execution log a real robot run would give)."""
    import functools

    @functools.wraps(fn)
    def wrapper(self, *a, **kw):
        depth = self._trace_depth
        self._trace_depth += 1
        try:
            out = fn(self, *a, **kw)
        finally:
            self._trace_depth -= 1
        if depth == 0:
            args = ", ".join([_short(x) for x in a] + [f"{k}={_short(v)}" for k, v in kw.items()])
            res = _short(out)
            err = f"  [{self.last_error}]" if out is False and self.last_error else ""
            self.trace.append(f"t={self.sim_time():.1f}s {fn.__name__}({args}) -> {res}{err}")
        return out
    return wrapper


def _short(x):
    if isinstance(x, ObjectState):
        return f"ObjectState({x.name}, p={np.round(x.position, 3).tolist()})"
    if isinstance(x, (list, tuple)) and len(x) > 4:
        return f"[{len(x)} items]"
    if isinstance(x, np.ndarray):
        return str(np.round(x, 3).tolist())
    if isinstance(x, tuple):
        return "(" + ", ".join(_short(v) for v in x) + ")"
    s = repr(x)
    return s if len(s) < 60 else s[:57] + "..."


class RobotAPI:
    """Primitive API (docs/api_spec.md) on a RoboPRO task object prepared by gah_env.reset()."""

    def __init__(self, env, noise_seed: int = 0, pos_noise: float | None = None,
                 rot_noise_deg: float | None = None, step_limit: int | None = None):
        self.env = env
        self.robot = env.robot
        self.scene = env.scene
        self.rng = np.random.default_rng(noise_seed)
        self.pos_noise = float(os.environ.get("GAH_POS_NOISE", 0.01) if pos_noise is None else pos_noise)
        self.rot_noise = np.deg2rad(float(os.environ.get("GAH_ROT_NOISE_DEG", 3.0)
                                          if rot_noise_deg is None else rot_noise_deg))
        self.step_limit = step_limit or gah_env.STEP_LIMIT
        self.steps = 0
        self.last_error = ""
        self.object_arm = None                 # arm that last grasped the task object (success check)
        self._held = {"left": None, "right": None}
        self.trace = []                        # execution log of the public calls
        self._trace_depth = 0
        self._state = getattr(env, "_gah_edit_state", {}) or {}
        self._motions = list(self._state.get("motions", []))
        self._build_registry()

    # ================================================================== registry
    def _build_registry(self):
        env = self.env
        robot_links = set()
        for ent in {id(self.robot.left_entity): self.robot.left_entity,
                    id(self.robot.right_entity): self.robot.right_entity}.values():
            for link in ent.get_links():
                robot_links.add(link.entity.per_scene_id)
        self._robot_ids = robot_links
        self._finger_ids = {}
        for arm in ARMS:
            grip = self.robot.left_gripper if arm == "left" else self.robot.right_gripper
            self._finger_ids[arm] = {g[0].child_link.entity.per_scene_id for g in grip}
        # owner name of every non-robot body, and the rigid components that belong to it
        self._owner_of = {}                      # per_scene_id -> owner name
        self._bodies = {}                        # owner -> list of rigid components
        self._groups = {}                        # group name -> set of owners (e.g. cabinet -> all links)
        self._aliases = {}                       # language name -> canonical name
        spec = gah_env.task_spec(env)
        self._group_of = {}                      # owner -> group (e.g. drawer -> cabinet)
        for owner, comp, group in spec["bodies"]:
            self._group_of[owner] = group
            self._owner_of[comp.entity.per_scene_id] = owner
            self._bodies.setdefault(owner, []).append(comp)
            self._groups.setdefault(group, set()).add(owner)
            self._groups.setdefault(owner, set()).add(owner)
        for alias, canon in spec["aliases"].items():
            self._aliases[alias] = canon
        self._parts = spec["parts"]              # name -> callable() -> (sapien.Pose, extents)
        self._regions = spec["regions"]          # name -> callable() -> (sapien.Pose, extents)
        self._actors = spec["actors"]            # owner -> RoboTwin Actor (for contact points)
        self._fine = set(spec["fine"])           # owners planned against per convex piece
        self._part_owner = spec.get("part_owner", {})   # part / region -> body it belongs to
        for owner, comp in self._state.get("added", []):
            self._owner_of[comp.entity.per_scene_id] = owner
            self._bodies.setdefault(owner, []).append(comp)
            self._groups.setdefault(owner, set()).add(owner)
        self._local_boxes = {o: [(c, _shape_local_boxes(c)) for c in comps] for o, comps in self._bodies.items()}
        self._static = {o for o, comps in self._bodies.items()
                        if all(isinstance(c, sapien.physx.PhysxRigidStaticComponent) for c in comps)}

    def _canon(self, name):
        return self._aliases.get(name, name)

    def objects(self) -> list[str]:
        """Names detect() accepts: scene objects, parts and regions."""
        names = set(self._bodies) | set(self._parts) | set(self._regions) | set(self._aliases)
        return sorted(n for n in names if n not in ("table", "wall"))

    def regions(self) -> list[str]:
        return sorted(self._regions)

    # ================================================================== simulation
    def sim_time(self) -> float:
        return self.steps * DT

    def _step(self, n: int = 1):
        for _ in range(n):
            if self.steps >= self.step_limit:
                raise gah_env.StepLimitExceeded(f"{self.steps} simulation steps")
            t = self.steps * DT
            for m in self._motions:
                m.apply(t)
            self.scene.step()
            self.steps += 1

    def _hold(self, arm=None, n: int = 10):
        for a in (ARMS if arm is None else (arm,)):
            ent = self.robot.left_entity if a == "left" else self.robot.right_entity
            joints = self.robot.left_arm_joints if a == "left" else self.robot.right_arm_joints
            q = ent.get_qpos()
            act = ent.get_active_joints()
            for j in joints:
                j.set_drive_target(q[act.index(j)] if j in act else j.get_drive_target())
                j.set_drive_velocity_target(0.0)
        self._step(n)

    # ================================================================== perception
    def _noisy(self, pose: sapien.Pose) -> sapien.Pose:
        p = np.asarray(pose.p, float) + self.rng.normal(0.0, self.pos_noise, 3)
        axis = self.rng.normal(size=3)
        axis /= np.linalg.norm(axis) + 1e-12
        dq = t3d.quaternions.axangle2quat(axis, self.rng.normal(0.0, self.rot_noise))
        q = t3d.quaternions.qmult(dq, np.asarray(pose.q, float))
        return sapien.Pose(p, q / np.linalg.norm(q))

    def _true_box(self, name):
        """(world pose of the bounding-box centre, full extents) of an object, part or region."""
        name = self._canon(name)
        if name in self._parts:
            return self._parts[name]()
        if name in self._regions:
            return self._regions[name]()
        if name in self._local_boxes:
            comps = self._local_boxes[name]
            comp, boxes = comps[0]
            if len(comps) > 1:  # several components: world AABB of all of them
                lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
                for c, bx in comps:
                    lp, hs = _union_box(bx)
                    T = (c.entity.get_pose() * lp).to_transformation_matrix()
                    ext = np.abs(T[:3, :3]) @ hs
                    lo, hi = np.minimum(lo, T[:3, 3] - ext), np.maximum(hi, T[:3, 3] + ext)
                return sapien.Pose((lo + hi) / 2), hi - lo
            lp, hs = _union_box(boxes)
            return comp.entity.get_pose() * lp, 2 * hs
        return None

    @_traced
    def detect(self, name: str) -> ObjectState | None:
        """Pose (noisy), bounding box and confidence of an object named in language. None if unknown."""
        tb = self._true_box(name)
        if tb is None:
            return None
        pose, ext = tb
        n = self._noisy(pose)
        return ObjectState(name, np.asarray(n.p, float), np.asarray(n.q, float), np.asarray(ext, float), 1.0)

    @_traced
    def get_obstacles(self) -> list[Box]:
        """Occupied regions (noisy boxes) of every scene object except the robot, table and wall."""
        out = []
        for owner in self._bodies:
            if owner in ("table", "wall"):
                continue
            tb = self._true_box(owner)
            n = self._noisy(tb[0])
            out.append(Box(owner, np.asarray(n.p, float), np.asarray(n.q, float), np.asarray(tb[1], float)))
        return out

    @_traced
    def track(self, name: str, duration_s: float = 1.0, hz: float = 10.0) -> list[ObjectState]:
        """Short pose history (noisy) while the robot holds still. Advances the simulation."""
        out = []
        every = max(1, int(round(1.0 / (hz * DT))))
        for _ in range(max(1, int(duration_s * hz))):
            self._hold(None, every)
            s = self.detect(name)
            if s is not None:
                out.append(s)
        return out

    def camera_frame(self, view: str = "head") -> np.ndarray:
        """RGB uint8 image. view = head | left_wrist | right_wrist."""
        cams = self.env.cameras
        self.env._update_render()
        if view == "head":
            cam = cams.static_camera_list[cams.static_camera_name.index("head_camera")]
        elif view in ("left_wrist", "wrist_left", "left"):
            cam = cams.left_camera
        elif view in ("right_wrist", "wrist_right", "right", "wrist"):
            cam = cams.right_camera
        else:
            raise ValueError(f"unknown view {view!r}")
        cam.take_picture()
        rgba = cam.get_picture("Color")
        return (np.clip(rgba[..., :3], 0, 1) * 255).astype(np.uint8)

    # ================================================================== proprioception / checks
    def tcp_pose(self, arm: str = "right"):
        """Exact gripper (TCP) pose (p, q) of an arm."""
        self._arm(arm)
        ee = self.robot.get_left_ee_pose() if arm == "left" else self.robot.get_right_ee_pose()
        T = _mat(ee)
        T[:3, 3] += TCP_OFFSET * T[:3, 0]
        return _tuple(T)

    def gripper_opening(self, arm: str = "right") -> float:
        """Commanded gripper opening in [0, 1] (0 closed, 1 open)."""
        self._arm(arm)
        return float(self.robot.get_left_gripper_val() if arm == "left" else self.robot.get_right_gripper_val())

    def _touching(self, arm) -> dict:
        """owner -> set of finger ids of `arm` touching it."""
        fingers = self._finger_ids[arm]
        out = {}
        for c in self.scene.get_contacts():
            a, b = c.bodies[0].entity.per_scene_id, c.bodies[1].entity.per_scene_id
            for f, o in ((a, b), (b, a)):
                if f in fingers and o in self._owner_of:
                    if any(np.linalg.norm(pt.impulse) > 0 for pt in c.points) or c.points:
                        out.setdefault(self._owner_of[o], set()).add(f)
        return out

    def _arm_contacts(self, arm) -> set:
        """Owners in contact with any link of `arm` (or the shared robot body)."""
        prefix = "fl_" if arm == "left" else "fr_"
        out = set()
        for c in self.scene.get_contacts():
            e0, e1 = c.bodies[0].entity, c.bodies[1].entity
            for r, o in ((e0, e1), (e1, e0)):
                if r.per_scene_id in self._robot_ids and o.per_scene_id in self._owner_of:
                    if r.get_name().startswith(prefix) or not r.get_name().startswith(("fl_", "fr_")):
                        out.add(self._owner_of[o.per_scene_id])
        return out

    @_traced
    def gripper_holding(self, arm: str = "right") -> bool:
        """True if both fingers of `arm` touch the same object while the gripper is (partly) closed."""
        self._arm(arm)
        if self.gripper_opening(arm) > 0.8:
            return False
        touch = self._touching(arm)
        return any(len(f) >= 2 for o, f in touch.items() if o not in ("table", "wall"))

    def held_object(self, arm: str = "right") -> str | None:
        """Name of the object the gripper of `arm` holds (both fingers on it), else None."""
        if not self.gripper_holding(arm):
            return None
        touch = self._touching(arm)
        for o, f in touch.items():
            if len(f) >= 2 and o not in ("table", "wall"):
                return o
        return None

    @_traced
    def in_region(self, obj, region) -> bool:
        """True if the (detected) centre of `obj` lies inside `region` (a region name or a Box)."""
        s = obj if isinstance(obj, ObjectState) else self.detect(obj)
        if s is None:
            return False
        if isinstance(region, Box):
            c, q, ext = region.center, region.quaternion, region.extents
        else:
            tb = self._true_box(region)
            if tb is None:
                raise ValueError(f"unknown region {region!r}")
            n = self._noisy(tb[0])
            c, q, ext = n.p, n.q, tb[1]
        local = t3d.quaternions.quat2mat(np.asarray(q, float)).T @ (np.asarray(s.position) - np.asarray(c))
        return bool(np.all(np.abs(local) <= np.asarray(ext) / 2))

    @_traced
    def wait_until(self, predicate, timeout_s: float = 5.0, poll_s: float = 0.1) -> bool:
        """Hold still and poll predicate() every poll_s of simulated time. Advances the simulation."""
        n = max(1, int(round(poll_s / DT)))
        t_end = self.sim_time() + timeout_s
        while True:
            if predicate():
                return True
            if self.sim_time() >= t_end:
                return False
            self._hold(None, n)

    # ================================================================== planning world
    def _arm(self, arm):
        if arm not in ARMS:
            raise ValueError("arm must be 'left' or 'right'")

    def _planner(self, arm):
        return self.robot.left_planner if arm == "left" else self.robot.right_planner

    def _to_planner_frame(self, arm, T_world):
        pl = self._planner(arm)
        base = pl.robot_origion_pose
        p, q = pl._trans_from_world_to_base(np.r_[base.p, base.q], np.r_[T_world[:3, 3],
                                            t3d.quaternions.mat2quat(T_world[:3, :3])])
        T = t3d.affines.compose(p, t3d.quaternions.quat2mat(q), [1, 1, 1])
        if "aloha-agilex" in pl.yml_path:
            rot = t3d.axangles.axangle2mat([0, 0, 1], -0.02 if arm == "left" else -0.01)
            T_rot = t3d.affines.compose([0, 0, 0], rot, [1, 1, 1])
            T_bias = t3d.affines.compose(pl.frame_bias, np.eye(3), [1, 1, 1])
            return T_rot @ T_bias @ T
        T[:3, 3] += pl.frame_bias
        return T

    def _world_boxes(self, exclude: set):
        """[(owner, world 4x4 of box centre, half size)] of all bodies not in `exclude`."""
        out = []
        for owner, comps in self._local_boxes.items():
            if owner in exclude or owner == "wall":
                continue
            for comp, boxes in comps:
                Tb = comp.entity.get_pose()
                if owner in self._fine:
                    for lp, hs in boxes:
                        out.append((owner, (Tb * lp).to_transformation_matrix(), hs))
                elif boxes:
                    lp, hs = _union_box(boxes)
                    out.append((owner, (Tb * lp).to_transformation_matrix(), hs))
        return out

    def _expand(self, names) -> set:
        out = set()
        for n in names or ():
            n = self._canon(n)
            n = self._part_owner.get(n, n)
            out |= self._groups.get(n, {n})
        return out

    def _set_world(self, arm, exclude: set):
        from curobo.geom.types import WorldConfig
        cub = {}
        for i, (owner, T, hs) in enumerate(self._world_boxes(exclude)):
            Tp = self._to_planner_frame(arm, T)
            q = t3d.quaternions.mat2quat(Tp[:3, :3])
            cub[f"b{i}"] = {"dims": (2 * np.maximum(hs, 0.002)).tolist(), "pose": list(Tp[:3, 3]) + list(q)}
        pl = self._planner(arm)
        pl.motion_gen.update_world(WorldConfig.from_dict({"cuboid": cub}))

    def _allowed(self, arm, allow_contact) -> set:
        allowed = self._expand(allow_contact)
        for a in ARMS:
            if self._held[a]:
                allowed |= self._expand([self._held[a]])
        # bodies the arm already touches may be touched (e.g. right after a grasp or a release), but never
        # the table or the wall: allowing them let the planner and the force limit ignore the table for the
        # rest of a motion once a wrist had grazed it (backend v2 fix, ISSUES 9)
        allowed |= self._arm_contacts(arm) - FIXED_SCENE
        return allowed

    def _plan(self, arm, tcp_target7, exclude: set, keep_orientation=False):
        T = _mat(tcp_target7)
        T[:3, 3] -= TCP_OFFSET * T[:3, 0]
        ee = list(T[:3, 3]) + list(t3d.quaternions.mat2quat(T[:3, :3]))
        self._set_world(arm, exclude)
        cons = [1, 1, 1, 0, 0, 0] if keep_orientation else None
        fn = self.robot.left_plan_path if arm == "left" else self.robot.right_plan_path
        try:
            res = fn(ee, constraint_pose=cons)
        except Exception as e:  # curobo raises on degenerate goals
            self.last_error = f"planner error: {type(e).__name__}: {e}"
            return None
        if res.get("status") != "Success":
            self.last_error = "no collision-free path (unreachable, or blocked by an obstacle)"
            return None
        return res

    def _execute(self, arm, res, speed, allowed: set) -> bool:
        k = SPEED_REPEAT[speed]
        pos, vel = res["position"], res["velocity"]
        over = 0
        # a fixed body the arm already presses on when the motion starts (fingertips on the table after a
        # low grasp) may keep its contact force for the first 0.1 s while that press relaxes
        preload = self._arm_contacts(arm) & FIXED_SCENE
        t0 = self.steps
        for i in range(len(pos)):
            for _ in range(k):
                self.robot.set_arm_joints(pos[i], vel[i] / k, arm)
                self._step()
                # checked every step: against a fixed body (table, wall, fixed obstacle) the force builds to
                # kN within a few steps, so stop at the first step over the limit; against movable bodies
                # require two consecutive steps (backend v2, ISSUES 9)
                ignore = preload if self.steps - t0 <= 25 else set()
                who, f = self._max_contact_force(allowed | ignore)
                over = over + 1 if f > FORCE_LIMIT else 0
                if over >= (1 if who in self._static else 2):
                    self._hold(arm, 25)
                    self.last_error = f"blocked: pushing {who} with {f:.0f} N (limit {FORCE_LIMIT:.0f} N)"
                    return False
        return True

    def _max_contact_force(self, allowed: set):
        best, who = 0.0, None
        per = {}
        for c in self.scene.get_contacts():
            e0, e1 = c.bodies[0].entity.per_scene_id, c.bodies[1].entity.per_scene_id
            for r, o in ((e0, e1), (e1, e0)):
                if r in self._robot_ids and o in self._owner_of:
                    owner = self._owner_of[o]
                    if owner in allowed:
                        continue
                    f = sum(np.linalg.norm(pt.impulse) for pt in c.points) / DT
                    per[owner] = per.get(owner, 0.0) + f
        for o, f in per.items():
            if f > best:
                best, who = f, o
        return who, best

    def _table_z(self) -> float:
        return gah_env.TABLE_Z + float(getattr(self.env, "table_z_bias", 0.0))

    def _check_workspace(self, p) -> bool:
        for i, k in enumerate("xyz"):
            lo, hi = WORKSPACE[k]
            if not lo <= p[i] <= hi:
                self.last_error = f"target {np.round(p, 3).tolist()} outside the workspace {WORKSPACE}"
                return False
        return True

    # ================================================================== motion
    @_traced
    def move_to(self, pose, arm: str = "right", speed: str = "normal", avoid: bool = True,
                allow_contact=(), linear: bool = False, _plan_ignore=()) -> bool:
        """Move the gripper (TCP) of `arm` to `pose` with the collision-aware planner.
        pose: (p, q) | [x,y,z,qw,qx,qy,qz] | ObjectState | [x,y,z] (keeps the current orientation).
        linear=True moves along a straight line in 2 cm steps (orientation held if unchanged).
        allow_contact: names the planner may touch (and the force limit ignores).
        avoid=False plans against the table only. Returns False (and sets last_error) if no path
        exists or the motion was stopped by the force limit."""
        self._arm(arm)
        if speed not in SPEED_REPEAT:
            raise ValueError(f"speed must be one of {list(SPEED_REPEAT)}")
        p7 = _pose7(pose)
        if not isinstance(pose, (tuple, ObjectState, Box, dict, sapien.Pose)) and np.asarray(pose).size == 3:
            p7[3:] = self.tcp_pose(arm)[1]          # position only: keep the current orientation
        if not self._check_workspace(p7[:3]):
            return False
        allowed = self._allowed(arm, allow_contact)
        # a fixed body the arm already touches (e.g. fingertips on the table after a low grasp) is left out
        # of the planner's world so that the start pose is not "in collision" and the arm can move away;
        # the force limit still counts it, so a motion that pushes into it stops at once
        touching_fixed = self._arm_contacts(arm) & FIXED_SCENE
        exclude = (set(allowed) | set(_plan_ignore) | touching_fixed) if avoid else (set(self._bodies) - {"table"})
        if not linear:
            res = self._plan(arm, p7, exclude)
            if res is None:
                return False
            return self._execute(arm, res, speed, allowed)
        start_p, start_q = self.tcp_pose(arm)
        same_rot = abs(abs(np.dot(start_q, p7[3:] / np.linalg.norm(p7[3:]))) - 1) < 1e-4
        dist = np.linalg.norm(p7[:3] - start_p)
        n = max(1, int(np.ceil(dist / 0.02)))
        for i in range(1, n + 1):
            a = i / n
            wp = p7.copy()
            wp[:3] = start_p + a * (p7[:3] - start_p)
            if not same_rot:
                wp[3:] = _slerp(start_q, p7[3:], a)
            res = self._plan(arm, wp, exclude, keep_orientation=same_rot)
            if res is None and same_rot:
                res = self._plan(arm, wp, exclude)
            if res is None:
                self.last_error = f"linear move stopped at {i - 1}/{n} steps: " + self.last_error
                return False
            if not self._execute(arm, res, speed, allowed):
                return False
            allowed |= self._arm_contacts(arm) & self._expand(allow_contact)
        return True

    @_traced
    def move_by(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, arm: str = "right",
                speed: str = "normal", linear: bool = True, allow_contact=()) -> bool:
        """Translate the gripper of `arm` by (dx, dy, dz) in the world frame, keeping its orientation."""
        p, q = self.tcp_pose(arm)
        return self.move_to((p + [dx, dy, dz], q), arm, speed=speed, linear=linear, allow_contact=allow_contact)

    @_traced
    def home(self, arm: str = "right", speed: str = "normal") -> bool:
        """Move `arm` back to its start pose."""
        self._arm(arm)
        ee = self.robot.left_original_pose if arm == "left" else self.robot.right_original_pose
        T = _mat(ee)
        T[:3, 3] += TCP_OFFSET * T[:3, 0]
        return self.move_to(_tuple(T), arm, speed=speed)

    def _gripper(self, arm, target):
        fn = self.robot.left_plan_grippers if arm == "left" else self.robot.right_plan_grippers
        now = self.gripper_opening(arm)
        res = fn(now, target)
        n = int(res["num_step"])
        for i in range(n + int(0.5 * n)):
            self.robot.set_gripper(res["result"][min(i, n - 1)], arm, res["per_step"])
            self._step()

    @_traced
    def open_gripper(self, arm: str = "right", pos: float = 1.0):
        self._arm(arm)
        self._gripper(arm, float(pos))
        if pos > 0.5:
            self._held[arm] = None

    @_traced
    def close_gripper(self, arm: str = "right", pos: float = 0.0):
        self._arm(arm)
        self._gripper(arm, float(pos))
        self._held[arm] = self.held_object(arm)

    # ------------------------------------------------------------------ grasp
    def _grasp_candidates(self, name, arm, strategy, pre_dis, depth, contact_point, given=None):
        """[(score, pre_tcp7, grasp_tcp7 + opening)] from `given` (an ObjectState of `name`) or else
        from a fresh noisy detection of `name`."""
        canon = self._canon(name)
        actor_info = self._actors.get(canon)
        true_box = self._true_box(canon)
        cands = []
        M = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])
        if actor_info is not None:
            actor, link_of, cps = actor_info()    # RoboTwin Actor, frame provider, contact point list
            for idx, (frame_name, local) in enumerate(cps):
                if contact_point is not None and idx not in np.atleast_1d(contact_point):
                    continue
                Tf_true = link_of(frame_name)
                if given is not None:   # same rigid relation between the box and the grasp frame
                    Tf = _mat(given) @ np.linalg.inv(true_box[0].to_transformation_matrix()) @ Tf_true
                else:
                    Tf = self._noisy(sapien.Pose(Tf_true)).to_transformation_matrix()
                Tc = Tf @ local
                R = Tc[:3, :3] @ M
                for yaw in (0.0, 0.35, -0.35, 0.7, -0.7, 1.05, -1.05, 1.4, -1.4):
                    Rz = _rotz(yaw)
                    Rg = Rz @ R
                    c = Tc[:3, 3]
                    cands.append((Rg, c))
        else:
            s = given if given is not None else self.detect(canon)
            if s is None:
                self.last_error = f"cannot detect {name!r}"
                return []
            Ro = t3d.quaternions.quat2mat(s.quaternion)
            ext = s.bbox_extents
            # top-down, closing along the object's shorter horizontal axis
            horiz = [Ro[:, i] for i in range(3)]
            hz = sorted(range(3), key=lambda i: abs(horiz[i][2]))   # most horizontal axes first
            a1, a2 = hz[0], hz[1]
            close_ax = horiz[a1] if ext[a1] <= ext[a2] else horiz[a2]
            close_ax = close_ax - np.array([0, 0, close_ax[2]])
            top = s.position.copy()
            top[2] = s.position[2] + max(0.0, ext[hz[2]] / 2 - 0.02)
            for yaw in (0.0, np.pi / 2):
                cl = _rotz(yaw) @ close_ax
                x = np.array([0, 0, -1.0])
                y = cl / (np.linalg.norm(cl) + 1e-9)
                cands.append((np.stack([x, y, np.cross(x, y)], 1), top))
            # side grasps from the robot's side
            for d in (np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), np.array([-1.0, 0, 0])):
                x = d
                y = np.cross([0, 0, 1.0], x)
                cands.append((np.stack([x, y, np.cross(x, y)], 1), s.position.copy()))
        out = []
        pref_side = np.array([0.0, 1.0, 0.0])
        s_obj = None
        if actor_info is not None and canon not in self._part_owner:
            s_obj = given if given is not None else self.detect(canon)
        for Rg, c in cands:
            appr = Rg[:, 0]
            top_score = 1.0 - max(0.0, -appr[2])                 # 0 = straight down
            side_score = abs(appr[2]) + 0.5 * (1.0 - np.dot(appr, pref_side))
            score = {"top": top_score, "side": side_score}.get(strategy, min(0.7 * top_score + 0.3 * side_score,
                                                                             top_score, side_score))
            q = t3d.quaternions.mat2quat(Rg)
            c = np.array(c, float) + depth * appr
            # open just wide enough for the object across the closing axis (2 cm margin)
            opening = 1.0
            if s_obj is not None:
                Ro = t3d.quaternions.quat2mat(s_obj.quaternion)
                width = float(np.sum(np.abs(Ro.T @ Rg[:, 1]) * s_obj.bbox_extents))
                opening = float(np.clip((width + 0.02) / 0.09, 0.35, 1.0))
            # raise the grasp point until the finger boxes clear the table top
            low = c[2] + min((Rg @ finger_corners(opening).T)[2])
            need = self._table_z() + FINGER_CLEARANCE - low
            if need > 0:
                c[2] += need
            c[2] = max(c[2], self._table_z() + MIN_TCP_ABOVE_TABLE)
            g = np.r_[c, q, opening]
            pre = np.r_[c - pre_dis * appr, q]
            if strategy == "top" and appr[2] > -0.5:
                continue
            if strategy == "side" and abs(appr[2]) > 0.5:
                continue
            out.append((score, pre, g))
        out.sort(key=lambda t: t[0])
        return out

    @_traced
    def grasp(self, obj, arm: str = "right", strategy: str = "auto", pre_dis: float = 0.08,
              depth: float = 0.0, contact_point=None, max_tries: int = 2, speed: str = "normal") -> bool:
        """Grasp `obj` with `arm`: plan to a pre-grasp pose pre_dis behind the grasp point, approach in a
        straight line, close, and check the grasp. obj is a name (object or part, detected afresh with
        noise) or an ObjectState you detected yourself (e.g. averaged). Grasp points come from the
        object's annotated contact points (or its bounding box); the gripper pre-opens to the object's
        width + 2 cm. strategy: top | side | auto. depth: extra distance past the grasp point.
        Returns True if the gripper holds the object afterwards."""
        self._arm(arm)
        given = obj if isinstance(obj, ObjectState) else None
        canon = self._canon(given.name if given is not None else obj)
        cands = self._grasp_candidates(canon, arm, strategy, pre_dis, depth, contact_point, given)
        if not cands:
            self.last_error = self.last_error or f"no grasp candidates for {obj!r}"
            return False
        tries = 0
        owner = self._part_owner.get(canon, canon)
        # the straight-in approach may touch the whole object the grasp point belongs to (a drawer
        # handle sits millimetres in front of the cabinet frame)
        target = self._expand([self._group_of.get(owner, owner)])
        for _, pre, g in cands:
            g, opening = g[:7], float(g[7])
            if not self._check_workspace(g[:3]) or not self._check_workspace(pre[:3]):
                continue
            if abs(self.gripper_opening(arm) - opening) > 0.05:
                self.open_gripper(arm, opening)
            if not self.move_to(pre, arm, speed=speed):
                if "blocked" in self.last_error:
                    tries += 1
                    if tries >= max_tries:
                        return False
                continue
            if not self.move_to(g, arm, speed=speed, linear=True, allow_contact=list(target), _plan_ignore=["table"]):
                tries += 1
                self.move_to(pre, arm, speed=speed, linear=True, allow_contact=list(target))
                if tries >= max_tries:
                    return False
                continue
            self.close_gripper(arm)
            held = self._held[arm]
            if held is not None and held in self._expand([canon]):
                if canon in gah_env.MAIN_OBJECTS or held in gah_env.MAIN_OBJECTS:
                    self.object_arm = arm
                self._held[arm] = held
                self.last_error = ""
                return True
            tries += 1
            self.last_error = f"grasp of {obj!r} closed on nothing" if held is None else f"grasped {held!r} instead"
            self.open_gripper(arm)
            self.move_to(pre, arm, speed=speed, linear=True, allow_contact=list(target))
            if tries >= max_tries:
                return False
        if not self.last_error:
            self.last_error = f"no reachable grasp for {obj!r}"
        return False

    # ------------------------------------------------------------------ place
    @_traced
    def place(self, target, arm: str = "right", yaw=None, pre_dis: float = 0.10, release: bool = True,
              retreat: float = 0.08, speed: str = "normal", allow_contact=()) -> bool:
        """Put the object held by `arm` so that its centre ends at `target` ([x, y, z] of the OBJECT
        centre). The gripper keeps its grasp; it first goes pre_dis above the target, then down in a
        straight line, opens (release=True) and retreats straight up by `retreat`.
        yaw: rotation [rad] of the held object about the vertical axis; None searches yaw in [-pi, pi]
        (then small tilts) for a reachable pose. Returns False if no motion was possible (the object
        is then still held) or the final descent was stopped."""
        self._arm(arm)
        held = self._held[arm] or self.held_object(arm)
        if held is None:
            self.last_error = "place: the gripper holds nothing"
            return False
        s = self.detect(held)
        tp, tq = self.tcp_pose(arm)
        offset = s.position - tp
        target = np.asarray(target, float).reshape(-1)[:3]
        if yaw is not None:
            yaws = [float(yaw)]
        else:
            steps = np.arange(0.3, np.pi + 1e-6, 0.3)
            yaws = [0.0] + [v for st in steps for v in (st, -st)]
        allowed = list(allow_contact) + [held]
        excl = self._allowed(arm, allowed) | (self._arm_contacts(arm) & FIXED_SCENE)
        R0 = t3d.quaternions.quat2mat(tq)
        for tilt in (0.0, 0.35, -0.35):
            for a in yaws:
                Rr = _rotz(a) @ t3d.axangles.axangle2mat([1, 0, 0], tilt)
                q = t3d.quaternions.mat2quat(Rr @ R0)
                g = target - Rr @ offset
                pre = g + np.array([0, 0, pre_dis])
                if not (self._check_workspace(g) and self._check_workspace(pre)):
                    continue
                # cheap feasibility check of both poses before moving
                if self._plan(arm, np.r_[pre, q], excl) is None or self._plan(arm, np.r_[g, q], excl) is None:
                    continue
                if not self.move_to((pre, q), arm, speed=speed, allow_contact=allowed):
                    if "blocked" in self.last_error:
                        return False
                    continue
                ok = self.move_to((g, q), arm, speed=speed, linear=True, allow_contact=allowed)
                if release:
                    self.open_gripper(arm)
                    self.move_by(dz=retreat, arm=arm, speed=speed, allow_contact=allowed)
                return ok
        self.last_error = "place: no reachable pre-place pose (" + self.last_error + ")"
        return False


def _slerp(q0, q1, a):
    q0 = np.asarray(q0, float) / np.linalg.norm(q0)
    q1 = np.asarray(q1, float) / np.linalg.norm(q1)
    d = np.dot(q0, q1)
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(d)
    return (np.sin((1 - a) * th) * q0 + np.sin(a * th) * q1) / np.sin(th)
