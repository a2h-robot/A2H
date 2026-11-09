"""Primitive API for the LIBERO-Goal backend (Panda, single arm, robosuite OSC_POSE at 20 Hz).

Implements docs/api_spec.md. Pipelines define `run(api)` and use only this object.
Poses are (p, q) tuples: p = xyz in metres (world frame), q = quaternion wxyz.
The world frame has z up; the table top is at z = 0.90 and the robot base at (-0.66, 0, 0.91).

Perception is not ground truth: every detect() / get_obstacles() / track() call returns the
simulator pose plus fresh zero-mean Gaussian noise (default 1 cm, 3 deg), independently per call
(GAH_POS_NOISE, GAH_ROT_NOISE_DEG). Box sizes come from the object geometry without noise.

Motion: move_to() solves IK, plans a collision-free joint path against the whole scene
(motion.py) and tracks it with the OSC controller. Safety limits are enforced here:
workspace bounds, speed limits and a wrist-force limit.

Extensions beyond docs/api_spec.md (see ISSUES.md):
  objects(), parts(name), regions(), tcp_pose(), gripper_width(), pose helpers
  (pose_mul, pose_inv, rpy_to_quat, quat_to_rpy), move_to(..., linear=, allow_contact=),
  grasp(..., offset=, depth=, approach_dist=, yaw=).
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from motion import Planner, rotvec

SPEEDS = {"slow": (0.004, 0.02), "normal": (0.012, 0.06)}   # per 50 ms step: (m, rad)
OSC_MAX = np.array([0.05, 0.05, 0.05, 0.5, 0.5, 0.5])       # robosuite OSC output_max
WORKSPACE = {"lo": np.array([-0.60, -0.55, 0.895]), "hi": np.array([0.40, 0.55, 1.50])}
FORCE_LIMIT_N = 120.0
GRIPPER_STEPS = 15
HAND_TO_GRIP = 0.097   # hand frame to grip site, along the approach axis [m]


@dataclass
class ObjectState:
    name: str
    position: np.ndarray        # (3,) world xyz of the object frame [m]
    quaternion: np.ndarray      # (4,) wxyz
    bbox_extents: np.ndarray    # (3,) full extents of the bounding box, in the object frame [m]
    confidence: float = 1.0
    bbox_center: np.ndarray = field(default=None)  # (3,) world xyz of the bounding-box center


@dataclass
class Box:
    name: str
    center: np.ndarray          # (3,)
    quaternion: np.ndarray      # (4,) wxyz
    extents: np.ndarray         # (3,) full extents [m]


# --------------------------------------------------------------------------- #
# Pose helpers
# --------------------------------------------------------------------------- #
def _R(q):
    w, x, y, z = q
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def _q(R):
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def _as_pose(pose, default_q=None):
    if isinstance(pose, ObjectState):
        return np.asarray(pose.position, float), np.asarray(pose.quaternion, float)
    if isinstance(pose, dict):
        return np.asarray(pose["p"], float), np.asarray(pose.get("q", default_q), float)
    if isinstance(pose, tuple) and len(pose) == 2:
        q = default_q if pose[1] is None else pose[1]
        return np.asarray(pose[0], float), np.asarray(q, float)
    arr = np.asarray(pose, float)
    if arr.shape == (4, 4):
        return arr[:3, 3], _q(arr[:3, :3])
    if arr.size == 7:
        return arr[:3], arr[3:]
    if arr.size == 3:
        return arr, np.asarray(default_q, float)
    raise ValueError(f"cannot interpret pose {pose!r}")


def pose_mul(a, b):
    """a * b (apply b in a's frame)."""
    pa, qa = _as_pose(a, [1, 0, 0, 0])
    pb, qb = _as_pose(b, [1, 0, 0, 0])
    Ra = _R(qa)
    return pa + Ra @ pb, _q(Ra @ _R(qb))


def pose_inv(a):
    p, q = _as_pose(a, [1, 0, 0, 0])
    R = _R(q).T
    return -R @ p, _q(R)


def rpy_to_quat(roll, pitch, yaw):
    """Extrinsic x-y-z (roll about world x, then pitch about y, then yaw about z) -> wxyz."""
    return _q(Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())


def quat_to_rpy(q):
    return Rotation.from_matrix(_R(q)).as_euler("xyz")


def _warn(msg):
    print(f"[api] {msg}", file=sys.stderr)


# --------------------------------------------------------------------------- #
class RobotAPI:
    def __init__(self, env, noise_seed: int = 0, pos_noise: float | None = None,
                 rot_noise_deg: float | None = None):
        self.env = env
        self.rng = np.random.default_rng(noise_seed)
        self.pos_noise = float(os.environ.get("GAH_POS_NOISE", 0.01) if pos_noise is None else pos_noise)
        self.rot_noise = np.deg2rad(float(os.environ.get("GAH_ROT_NOISE_DEG", 3.0)
                                          if rot_noise_deg is None else rot_noise_deg))
        self.planner = Planner(env, np.random.default_rng(noise_seed + 1))
        self._grip = -1.0
        self._held = None
        self._target = self._eef()
        self.last_motion = "none"   # "ok" | "refused" (did not move) | "incomplete" (moved, stopped short)

    # ------------------------------------------------------------------ helpers
    def _arm(self, arm):
        if arm not in ("right", None):
            raise ValueError("the LIBERO Panda has a single arm; use arm='right'")

    def _eef(self):
        d, s = self.env.data, self.planner.site
        return d.site_xpos[s].copy(), d.site_xmat[s].reshape(3, 3).copy()

    def _noisy(self, p, q):
        p = np.asarray(p, float) + self.rng.normal(0.0, self.pos_noise, 3)
        axis = self.rng.normal(size=3)
        axis /= np.linalg.norm(axis) + 1e-12
        dR = Rotation.from_rotvec(axis * self.rng.normal(0.0, self.rot_noise)).as_matrix()
        return p, _q(dR @ _R(q))

    def _action(self, p, R):
        pc, Rc = self._eef()
        a = np.zeros(7)
        a[:3] = (p - pc) / OSC_MAX[:3]
        a[3:6] = rotvec(R @ Rc.T) / OSC_MAX[3:]
        a[:6] = np.clip(a[:6], -1, 1)
        a[6] = self._grip
        return a

    def _step_to(self, p, R):
        self.env.step(self._action(p, R))

    def _hold(self, n=1):
        p, R = self._target
        for _ in range(n):
            self._step_to(p, R)

    def _force(self):
        try:
            return float(np.linalg.norm(self.env.robot.ee_force))
        except Exception:
            return 0.0

    def _in_workspace(self, p):
        return bool(np.all(p >= WORKSPACE["lo"]) and np.all(p <= WORKSPACE["hi"]))

    def _track(self, targets, tol=(0.004, 0.04), settle_steps=40, lag_ok=(0.015, 0.15),
               max_wait=10, blocked=(0.05, 0.6)):
        """Follow a list of (p, R) targets with the OSC controller. A target is held (up to
        max_wait steps) until the error is within lag_ok, so large rotations slow the motion down
        instead of falling behind. Then converge on the last target. Stops early on excessive
        force, or when the arm stays more than `blocked` away from a target (contact)."""
        over = 0
        for k, (p, R) in enumerate(targets):
            self._target = (p, R)
            for _ in range(max_wait):
                self._step_to(p, R)
                over = over + 1 if self._force() > FORCE_LIMIT_N else 0
                pc, Rc = self._eef()
                ep, er = np.linalg.norm(p - pc), np.linalg.norm(rotvec(R @ Rc.T))
                if over >= 3 or (ep < lag_ok[0] and er < lag_ok[1]):
                    break
            if over >= 3 or ep > blocked[0] or er > blocked[1]:
                _warn(f"motion stopped: {'force limit' if over >= 3 else 'blocked (error %.3f m, %.0f deg)' % (ep, np.degrees(er))}")
                self._target = self._eef()
                return False
        p, R = targets[-1]
        for _ in range(settle_steps):
            pc, Rc = self._eef()
            if np.linalg.norm(p - pc) < tol[0] and np.linalg.norm(rotvec(R @ Rc.T)) < tol[1]:
                return True
            self._step_to(p, R)
            if self._force() > FORCE_LIMIT_N:
                break
        pc, Rc = self._eef()
        ok = np.linalg.norm(p - pc) < 2.5 * tol[0] and np.linalg.norm(rotvec(R @ Rc.T)) < 2.5 * tol[1]
        if not ok:
            _warn(f"target not reached: error {np.linalg.norm(p - pc):.3f} m, "
                  f"{np.degrees(np.linalg.norm(rotvec(R @ Rc.T))):.1f} deg")
        return bool(ok)

    def _run(self, targets):
        ok = self._track(targets)
        self.last_motion = "ok" if ok else "incomplete"
        return ok

    def _densify_joint_path(self, path, speed):
        dp, dr = SPEEDS[speed]
        out = []
        for a, b in zip(path[:-1], path[1:]):
            pa, Ra = self.planner.fk(a)
            pb, Rb = self.planner.fk(b)
            n = int(max(1, np.ceil(max(np.linalg.norm(pb - pa) / dp,
                                       np.linalg.norm(rotvec(Rb @ Ra.T)) / dr,
                                       np.max(np.abs(b - a)) / (dr * 1.5)))))
            for i in range(1, n + 1):
                out.append(self.planner.fk(a + (b - a) * i / n))
        return out

    def _linear_targets(self, p0, R0, p1, R1, speed):
        dp, dr = SPEEDS[speed]
        rv = rotvec(R1 @ R0.T)
        n = int(max(1, np.ceil(max(np.linalg.norm(p1 - p0) / dp, np.linalg.norm(rv) / dr))))
        return [(p0 + (p1 - p0) * i / n, Rotation.from_rotvec(rv * i / n).as_matrix() @ R0)
                for i in range(1, n + 1)]

    def _bbox(self, name):
        """(center_world, extents_in_object_frame) of an object's collision geometry."""
        g = self.env
        geoms = g.geoms_of(name, collision=True) or g.geoms_of(name, collision=False)
        pts = g.geom_corners(geoms)
        p, q = g.true_pose(name)
        R = _R(q)
        local = (pts - p) @ R
        lo, hi = local.min(0), local.max(0)
        return p + R @ ((lo + hi) / 2), hi - lo

    # --------------------------------------------------------------- perception
    def objects(self) -> list[str]:
        """Names of all task objects, fixtures and added scene objects."""
        return self.env.object_names()

    def regions(self) -> list[str]:
        """Named regions (sites) of the scene, e.g. drawer interiors or target zones on the table."""
        return self.env.region_names()

    def parts(self, name: str) -> list[str]:
        """Names of the movable or fixed parts of an object (e.g. drawers, a knob); usable in detect()."""
        g = self.env
        import mujoco
        return [mujoco.mj_id2name(g.model, mujoco.mjtObj.mjOBJ_BODY, b) for b in g.bodies_of(name)[1:]]

    def detect(self, name: str) -> ObjectState | None:
        """Object, part (see parts()) or region (see regions()) by name. None if not found."""
        import mujoco
        g = self.env
        try:
            obj = g.resolve(name)
            center, ext = self._bbox(obj)
            p, q = g.true_pose(obj)
        except KeyError:
            m, d = g.model, g.data
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
            s = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, name)
            if s >= 0 and name in g.region_names():
                p, q = d.site_xpos[s].copy(), _q(d.site_xmat[s].reshape(3, 3))
                center, ext = p.copy(), 2 * m.site_size[s].copy()
            elif b > 0:
                p, q = d.xpos[b].copy(), d.xquat[b].copy()
                geoms = g.geoms_of([b], collision=True)
                pts = g.geom_corners(geoms) if geoms else p[None]
                R = _R(q)
                local = (pts - p) @ R
                center, ext = p + R @ ((local.min(0) + local.max(0)) / 2), local.max(0) - local.min(0)
            else:
                return None
        pn, qn = self._noisy(p, q)
        Rn = _R(qn)
        offset = _R(q).T @ (center - p)
        return ObjectState(name, pn, qn, np.asarray(ext, float), 1.0, pn + Rn @ offset)

    def get_obstacles(self) -> list[Box]:
        """Oriented boxes (noisy) of every collision body except the robot, the table and the floor.
        Articulated objects are split into their parts (e.g. cabinet body and each drawer)."""
        import mujoco
        g = self.env
        boxes = []
        for name in g.object_names():
            for b in g.bodies_of(name):
                geoms = g.geoms_of([b], collision=True)
                if not geoms:
                    continue
                p, q = g.data.xpos[b].copy(), g.data.xquat[b].copy()
                R = _R(q)
                local = (g.geom_corners(geoms) - p) @ R
                lo, hi = local.min(0), local.max(0)
                pn, qn = self._noisy(p + R @ ((lo + hi) / 2), q)
                bname = name if b == g.root_body(name) else mujoco.mj_id2name(g.model, mujoco.mjtObj.mjOBJ_BODY, b)
                boxes.append(Box(bname, pn, qn, hi - lo))
        return boxes

    def track(self, name: str, duration_s: float):
        """Hold the arm still for duration_s and return [(t, ObjectState), ...], one per 50 ms step."""
        hist = []
        for i in range(max(1, int(round(duration_s / 0.05)))):
            self._hold()
            hist.append(((i + 1) * 0.05, self.detect(name)))
        return hist

    def camera_frame(self, view: str = "head") -> np.ndarray:
        """RGB image (H, W, 3) uint8 from the 'head' (agentview) or 'wrist' (eye-in-hand) camera."""
        cam = {"head": "agentview", "wrist": "robot0_eye_in_hand"}.get(view)
        if cam is None:
            raise ValueError("view must be 'head' or 'wrist'")
        s = self.env.camera_size
        img = self.env.env.sim.render(height=s, width=s, camera_name=cam)
        return np.ascontiguousarray(img[::-1])

    def tcp_pose(self):
        """Current grip-site pose (p, q), exact (proprioception). The gripper's z axis is the
        approach direction (pointing out of the fingers), its x axis is the finger closing axis."""
        p, R = self._eef()
        return p, _q(R)

    def gripper_width(self) -> float:
        """Distance between the fingers [m] (0.0 closed, about 0.08 fully open)."""
        q = self.env.data.qpos[[7, 8]]
        return float(q[0] - q[1])

    # ------------------------------------------------------------------- motion
    def move_to(self, pose, arm: str = "right", speed: str = "normal", linear: bool = False,
                allow_contact=None) -> bool:
        """Move the grip site to pose ((p, q), 7-vector, or xyz to keep the current orientation).
        Default: collision-free joint-space plan against the whole scene (the held object is
        carried and checked too). linear=True: straight Cartesian line (for pushing, pulling,
        inserting), still rejected if it would hit anything not in allow_contact.
        allow_contact: object names the robot may touch on this motion.
        Returns False (without moving, if possible) when no valid motion exists."""
        self._arm(arm)
        if speed not in SPEEDS:
            raise ValueError(f"speed must be one of {list(SPEEDS)}")
        p_now, R_now = self._eef()
        p, q = _as_pose(pose, default_q=_q(R_now))
        R = _R(q / np.linalg.norm(q))
        self.last_motion = "refused"
        if not self._in_workspace(p):
            _warn(f"target {np.round(p, 3)} outside workspace bounds {WORKSPACE}")
            return False
        allow = [self.env.resolve(n) for n in (allow_contact or [])]
        held = self._held_name()
        pl = self.planner
        pl.sync()
        q0 = pl.q_now()
        ctx = pl.context(attached=held if held in self.env.movable_objects() else None,
                         allow=allow + ([held] if held and held not in self.env.movable_objects() else []),
                         start_q=q0)
        if linear:
            targets = self._linear_targets(p_now, R_now, p, R, speed)
            qs = q0
            for k, (pt, Rt) in enumerate(targets[::max(1, len(targets) // 25)] + [targets[-1]]):
                sols = pl.ik(pt, Rt, qs, restarts=3)
                if not sols:
                    _warn(f"linear move: no IK solution at {np.round(pt, 3)}")
                    return False
                qs = sols[0]
                if pl.collides(qs, ctx):
                    _warn(f"linear move would collide with {pl.colliding_names(qs, ctx)}")
                    return False
            return self._run(targets)
        sols = pl.ik(p, R, q0, ctx=ctx, restarts=16)
        if not sols:
            hit = pl.ik(p, R, q0, restarts=24)
            sols = [h for h in hit if not pl.collides(h, ctx)]
            if not sols:
                names = sorted({n for h in hit for n in pl.colliding_names(h, ctx)})
                why = f"in collision with {names}" if hit else "unreachable (no IK solution)"
                _warn(f"move_to {np.round(p, 3)}: target {why}")
                return False
        for qg in sols[:3]:
            path = pl.plan(q0, qg, ctx)
            if path is not None:
                return self._run(self._densify_joint_path(path, speed) + [(p, R)])
        _warn(f"move_to {np.round(p, 3)}: no collision-free path found")
        return False

    def grasp(self, obj: str, arm: str = "right", strategy: str = "auto", offset=(0.0, 0.0, 0.0),
              depth: float | None = None, approach_dist: float = 0.08, yaw: float | None = None) -> bool:
        """Grasp obj (object or part name) using its noisy detection and bounding box.
        strategy: top (approach straight down), side (horizontal, from the robot base side),
        auto (top unless the box is taller than 0.12 m, then side).
        offset: shift of the grasp point in world xyz [m]. depth: how far below the top
        (top grasp) the grip site goes. yaw: gripper yaw for top grasps (default: close across
        the box's narrower horizontal side). Returns True if holding afterwards."""
        self._arm(arm)
        st = self.detect(obj)
        if st is None:
            _warn(f"grasp: '{obj}' not found")
            return False
        R_obj = _R(st.quaternion)
        ext = st.bbox_extents
        top = st.bbox_center[2] + 0.5 * abs(R_obj[2] @ ext)  # approximate world top
        height = abs(R_obj[2] @ ext) if ext is not None else 0.05
        if strategy == "auto":
            strategy = "side" if height > 0.12 else "top"
        if strategy == "top":
            approach = np.array([0.0, 0.0, -1.0])
            if yaw is None:
                ax = R_obj[:, 0] if ext[0] <= ext[1] else R_obj[:, 1]
                ax = np.array([ax[0], ax[1], 0.0])
                yaw = np.arctan2(ax[1], ax[0]) if np.linalg.norm(ax) > 1e-6 else 0.0
            yaw = self._closest_yaw(yaw)
            x = np.array([np.cos(yaw), np.sin(yaw), 0.0])
            dep = min(0.03, 0.5 * height) if depth is None else depth
            point = np.array([st.bbox_center[0], st.bbox_center[1], max(top - dep, 0.905)])
        elif strategy == "side":
            dxy = st.bbox_center[:2] - np.array([-0.66, 0.0])
            approach = np.array([dxy[0], dxy[1], 0.0]) / (np.linalg.norm(dxy) + 1e-9)
            x = np.cross([0.0, 0.0, 1.0], approach)
            point = np.array([*st.bbox_center[:2], max(st.bbox_center[2], 0.93)])
            if depth is not None:
                point = point + approach * depth
        else:
            raise ValueError("strategy must be top | side | auto")
        Rg = np.column_stack([x, np.cross(approach, x), approach])
        point = point + np.asarray(offset, float)
        self.open_gripper()
        if not self.move_to((point - approach * approach_dist, _q(Rg))):
            return False
        self.move_to((point, _q(Rg)), linear=True, speed="slow", allow_contact=[self._owner(obj)])
        if self.last_motion == "refused":
            return False
        self.close_gripper()  # also when the approach stopped short on contact with the object
        return self.gripper_holding()

    def place(self, pose, arm: str = "right") -> bool:
        """Carry the held object so that the grip site reaches pose, then open the gripper."""
        ok = self.move_to(pose, arm=arm)
        self.open_gripper(arm)
        return ok

    def open_gripper(self, arm: str = "right"):
        self._arm(arm)
        self._grip = -1.0
        self._held = None
        self._hold(GRIPPER_STEPS)

    def close_gripper(self, arm: str = "right"):
        self._arm(arm)
        self._grip = 1.0
        self._hold(GRIPPER_STEPS)

    def wait_until(self, predicate, timeout_s: float) -> bool:
        for _ in range(max(1, int(round(timeout_s / 0.05)))):
            if predicate():
                return True
            self._hold()
        return bool(predicate())

    # ------------------------------------------------------------------- checks
    def _contact_bodies(self, geom_names):
        import mujoco
        m, d = self.env.model, self.env.data
        ids = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in geom_names}
        out = set()
        for i in range(d.ncon):
            g1, g2 = d.contact.geom[i]
            if g1 in ids:
                out.add(m.body_rootid[m.geom_bodyid[g2]])
            elif g2 in ids:
                out.add(m.body_rootid[m.geom_bodyid[g1]])
        return out

    def _held_name(self):
        g = self.env
        if self._grip < 0:
            return None
        gr = self.env.robot.gripper.important_geoms
        both = self._contact_bodies(gr["left_finger"]) & self._contact_bodies(gr["right_finger"])
        for name in g.object_names():
            if g.root_body(name) in both:
                return name
        return None

    def _owner(self, name):
        """Object that owns an object/part/region name."""
        g = self.env
        try:
            return g.resolve(name)
        except KeyError:
            for n in g.object_names():
                if name.startswith(n):
                    return n
            raise

    def gripper_holding(self, arm: str = "right") -> bool:
        """True if both fingers touch the same object and the gripper is not fully closed."""
        self._arm(arm)
        return self._held_name() is not None and self.gripper_width() > 0.002

    def in_region(self, obj: str, region) -> bool:
        """region: a region name (see regions()) or {"center": [x, y, z], "half_extents": [hx, hy, hz]}
        (axis-aligned, world). Uses the noisy detected position of obj."""
        st = self.detect(obj)
        if st is None:
            return False
        if isinstance(region, str):
            r = self.detect(region)
            if r is None:
                raise KeyError(f"unknown region '{region}'")
            local = _R(r.quaternion).T @ (st.position - r.position)
            return bool(np.all(np.abs(local) <= r.bbox_extents / 2))
        c = np.asarray(region["center"], float)
        h = np.asarray(region["half_extents"], float)
        return bool(np.all(np.abs(st.position - c) <= h))

    def _closest_yaw(self, yaw):
        """Gripper yaw is symmetric under pi: pick the equivalent yaw closest to the current one."""
        R = self._eef()[1]
        cur = np.arctan2(R[1, 0], R[0, 0])
        cands = [yaw + k * np.pi for k in (-2, -1, 0, 1, 2)]
        return min(cands, key=lambda y: abs((y - cur + np.pi) % (2 * np.pi) - np.pi))
