"""Primitive API for the RoboDyna backend (dual UR5 with WSG grippers: arm="left" | "right").

Implements docs/api_spec.md. Pipelines define `run(api)` and use only this object.
Poses are 7-lists [x, y, z, qw, qx, qy, qz] in the world frame (metres). Gripper poses use
RoboDyna's (RoboTwin's) end-effector convention (the pose that RoboTwin's grasp/place helpers produce and that
`ee_pose(arm)` reports): the gripper's x axis points from the wrist towards the fingertips.

Perception is not ground truth: every detect() / get_obstacles() / track() sample is the simulator
pose plus fresh zero-mean Gaussian noise (default 1 cm per axis and 3 deg about a random axis,
override with GAH_POS_NOISE / GAH_ROT_NOISE_DEG), independently per call. Object keypoints
(RoboTwin's functional and contact points) move rigidly with the noisy pose. ObjectState.rgb is the
object's surface colour (its render material, plus 0.03 Gaussian noise per channel), so a pipeline
can judge appearance, e.g. how ripe a fruit is, without camera processing.

The world is dynamic: RoboDyna moves belts, balls and oscillating parts on every physics step, and
it keeps moving while the pipeline waits (wait, wait_until, track) or moves. time() is sim time.
Fixed or task-animated parts (a belt, a sorter, a red line) are listed by parts() and can be
detected like objects, but not grasped.

Motion uses RoboTwin's cuRobo planner. It always avoids the table; it avoids other objects only when
they are passed in `avoid` (names, Box, or ObjectState), as boxes inflated by 1 cm.
Grasp and place reuse RoboTwin's grasp_actor / place_actor geometry (contact points, functional
points, choose_grasp_pose, get_place_pose), computed on the perceived pose, not the true one.

Extensions beyond docs/api_spec.md (see ISSUES.md): objects(), parts(), ee_pose(), tcp_pose(),
gripper_opening(), time(), move_by(), move_both(), back_to_origin(), grasp_candidates(),
place_poses(), wait(), pose helpers (pose_mul, pose_inv, rpy_to_quat, quat_to_rpy, transform_point).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

import numpy as np
import sapien
import transforms3d as t3d

ARMS = ("left", "right")
SPEED_STRETCH = {"normal": 1.0, "slow": 2.5}      # slow = the planned trajectory played 2.5x longer
WORKSPACE = {"x": (-0.65, 0.65), "y": (-0.45, 0.45), "z": (0.745, 1.45)}   # gripper target bounds
AVOID_MARGIN = 0.01
HOLD_STEPS_PER_S = 250


@dataclass
class ObjectState:
    name: str
    position: np.ndarray          # (3,) world xyz [m] of the object frame
    quaternion: np.ndarray        # (4,) wxyz
    bbox_extents: np.ndarray      # (3,) full extents of the object's box, in the object frame [m]
    bbox_center: np.ndarray       # (3,) world xyz of that box's centre
    confidence: float = 1.0
    points: dict = field(default_factory=dict)   # "functional_<i>" / "contact_<i>" -> 7-list world pose
    time: float = 0.0             # sim time of the observation [s]
    rgb: list | None = None       # surface colour in [0, 1] (noisy), None if the object has no material

    @property
    def pose(self) -> list:
        return [*map(float, self.position), *map(float, self.quaternion)]


@dataclass
class Box:
    name: str
    center: np.ndarray            # (3,)
    quaternion: np.ndarray        # (4,) wxyz
    extents: np.ndarray           # (3,) full extents [m]


# --------------------------------------------------------------------------- #
# Pose helpers
# --------------------------------------------------------------------------- #
def _p7(pose) -> np.ndarray:
    if isinstance(pose, ObjectState):
        return np.asarray(pose.pose, dtype=np.float64)
    if isinstance(pose, sapien.Pose):
        return np.concatenate([pose.p, pose.q]).astype(np.float64)
    a = np.asarray(pose, dtype=np.float64).reshape(-1)
    if a.size == 3:
        return np.concatenate([a, [1, 0, 0, 0]])
    if a.size != 7:
        raise ValueError(f"pose must be xyz or xyz+wxyz, got {pose!r}")
    return a


def _mat(pose) -> np.ndarray:
    a = _p7(pose)
    T = np.eye(4)
    T[:3, :3] = t3d.quaternions.quat2mat(a[3:] / np.linalg.norm(a[3:]))
    T[:3, 3] = a[:3]
    return T


def _from_mat(T) -> list:
    return [*map(float, T[:3, 3]), *map(float, t3d.quaternions.mat2quat(T[:3, :3]))]


def pose_mul(a, b) -> list:
    """a * b: b expressed in a's frame, returned in the world frame."""
    return _from_mat(_mat(a) @ _mat(b))


def pose_inv(a) -> list:
    return _from_mat(np.linalg.inv(_mat(a)))


def transform_point(pose, xyz) -> np.ndarray:
    T = _mat(pose)
    return T[:3, :3] @ np.asarray(xyz, dtype=np.float64) + T[:3, 3]


def rpy_to_quat(roll, pitch, yaw) -> list:
    return list(map(float, t3d.euler.euler2quat(roll, pitch, yaw)))


def quat_to_rpy(q) -> list:
    return list(map(float, t3d.euler.quat2euler(np.asarray(q, dtype=np.float64))))


# --------------------------------------------------------------------------- #
# Planner world (cuRobo cuboids), shared with the oracle in gah_env.run_expert
# --------------------------------------------------------------------------- #
def true_box(name, actor) -> Box:
    import scene_edits
    lo, hi = scene_edits.world_aabb(getattr(actor, "actor", actor))
    return Box(name, (lo + hi) / 2, np.array([1.0, 0, 0, 0]), hi - lo)


def _world_to_planner(planner, arm, center, quat) -> list:
    """World pose -> the cuRobo base frame, exactly as CuroboPlanner.plan_path maps its targets."""
    base = np.concatenate([np.asarray(planner.robot_origion_pose.p), np.asarray(planner.robot_origion_pose.q)])
    p, q = planner._trans_from_world_to_base(base, np.concatenate([center, quat]))
    if "aloha-agilex" in planner.yml_path:
        T = t3d.affines.compose(p, t3d.quaternions.quat2mat(q), [1, 1, 1])
        T_bias = t3d.affines.compose(planner.frame_bias, np.eye(3), [1, 1, 1])
        rot = t3d.axangles.axangle2mat([0, 0, 1], -0.02 if arm == "left" else -0.01)
        T = t3d.affines.compose([0, 0, 0], rot, [1, 1, 1]) @ T_bias @ T
        p, q = T[:3, 3], t3d.quaternions.mat2quat(T[:3, :3])
    else:
        p = np.asarray(p) + np.asarray(planner.frame_bias)
    return [*map(float, p), *map(float, q)]


def set_planner_world(env, arm, boxes, margin=AVOID_MARGIN):
    """Table (as RoboTwin defines it) plus one cuboid per box."""
    from curobo.geom.types import Cuboid, WorldConfig
    planner = env.robot.left_planner if arm == "left" else env.robot.right_planner
    o = planner.robot_origion_pose
    cuboids = [Cuboid(name="table", dims=[0.7, 2, 0.04], pose=[o.p[1], 0.0, 0.74 - o.p[2], 1, 0, 0, 0])]
    for i, b in enumerate(boxes):
        cuboids.append(Cuboid(
            name=f"obs{i}_{b.name}",
            pose=_world_to_planner(planner, arm, np.asarray(b.center, dtype=np.float64),
                                   np.asarray(b.quaternion, dtype=np.float64)),
            dims=[float(max(1e-3, e + 2 * margin)) for e in b.extents]))
    world = WorldConfig(cuboid=cuboids)
    planner.motion_gen.update_world(world)
    planner.motion_gen_batch.update_world(world)


def _stretch(res: dict, k: float) -> dict:
    pos = np.asarray(res["position"])
    n = len(pos)
    if k == 1.0 or n < 2:
        return res
    m = int(np.ceil(n * k))
    t_old = np.linspace(0, 1, n)
    t_new = np.linspace(0, 1, m)
    vel = np.asarray(res["velocity"])
    return {"status": res["status"],
            "position": np.stack([np.interp(t_new, t_old, pos[:, j]) for j in range(pos.shape[1])], 1),
            "velocity": np.stack([np.interp(t_new, t_old, vel[:, j]) for j in range(vel.shape[1])], 1) / k}


# --------------------------------------------------------------------------- #
class RobotAPI:
    def __init__(self, genv, noise_seed: int = 0, pos_noise: float | None = None,
                 rot_noise_deg: float | None = None):
        self._genv = genv
        self._env = genv.env
        self._robot = genv.env.robot
        self._rng = np.random.default_rng(noise_seed)
        self.pos_noise = float(os.environ.get("GAH_POS_NOISE", 0.01) if pos_noise is None else pos_noise)
        self.rot_noise = np.deg2rad(float(os.environ.get("GAH_ROT_NOISE_DEG", 3.0)
                                          if rot_noise_deg is None else rot_noise_deg))
        self.trace = []
        self._world_sig = {"left": (), "right": ()}
        self._grip_cmd = {"left": 1.0, "right": 1.0}
        # the cuRobo planners outlive the episode (GahEnv is persistent): start every episode with a world
        # that holds only the table, matching _world_sig, instead of the previous episode's obstacles
        set_planner_world(self._env, "left", [])
        set_planner_world(self._env, "right", [])

    # ------------------------------------------------------------------ internals
    def _log(self, call, **kw):
        self.trace.append({"t": round(self._genv.sim_time, 2), "call": call, **kw})

    @staticmethod
    def _arm(arm):
        if arm not in ARMS:
            raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")
        from envs.utils.action import ArmTag
        return ArmTag(arm)

    def _noisy(self, pose: sapien.Pose) -> sapien.Pose:
        p = np.asarray(pose.p, dtype=np.float64) + self._rng.normal(0.0, self.pos_noise, 3)
        axis = self._rng.normal(size=3)
        axis /= np.linalg.norm(axis) + 1e-12
        dq = t3d.quaternions.axangle2quat(axis, self._rng.normal(0.0, self.rot_noise))
        q = t3d.quaternions.qmult(dq, np.asarray(pose.q, dtype=np.float64))
        return sapien.Pose(p, q)

    def _actor(self, name):
        objs = self._genv.objects()
        if name not in objs:
            raise KeyError(f"unknown object {name!r}; objects() = {sorted(objs)}")
        return objs[name]

    def _proxy(self, name, state: ObjectState):
        """A RoboTwin Actor whose pose is the perceived one, so RoboTwin's grasp/place geometry runs on
        perception rather than on ground truth."""
        from envs.utils.actor_utils import Actor
        real = self._actor(name)

        class _Seen:
            def __init__(self, pose):
                self._pose = pose

            def get_pose(self):
                return self._pose

            def get_name(self):
                return real.get_name()

            def get_components(self):
                return []

        proxy = Actor.__new__(Actor)
        proxy.actor = _Seen(sapien.Pose(state.position, state.quaternion))
        proxy.config = real.config
        return proxy

    def _hold(self, n):
        """Arms hold still while the world (belts, balls, gates) advances n physics steps."""
        self._genv.tick(n)

    def _set_world(self, arm, avoid, drop=()):
        boxes = []
        for item in avoid or []:
            if isinstance(item, Box):
                b = item
            elif isinstance(item, ObjectState):
                R = t3d.quaternions.quat2mat(item.quaternion)
                corners = np.array([[sx, sy, sz] for sx in (-.5, .5) for sy in (-.5, .5) for sz in (-.5, .5)])
                pts = (R @ (corners * item.bbox_extents).T).T + item.bbox_center
                b = Box(item.name, (pts.min(0) + pts.max(0)) / 2, np.array([1.0, 0, 0, 0]),
                        pts.max(0) - pts.min(0))
            elif isinstance(item, str):
                b = self._obstacle_box(item)
            else:
                raise TypeError(f"avoid items must be names, Box or ObjectState, got {item!r}")
            if b.name not in drop:
                boxes.append(b)
        sig = tuple((b.name, *np.round(b.center, 4), *np.round(b.extents, 4)) for b in boxes)
        if sig != self._world_sig[arm]:
            set_planner_world(self._env, arm, boxes)
            self._world_sig[arm] = sig

    def _plan(self, arm, pose, hold_orientation=False):
        constraint = [1, 1, 1, 0, 0, 0] if hold_orientation else None
        fn = self._robot.left_plan_path if arm == "left" else self._robot.right_plan_path
        res = fn(list(map(float, pose)), constraint_pose=constraint)
        return res if res and res.get("status") == "Success" else None

    def _execute(self, plans: dict, speed="normal"):
        if speed not in SPEED_STRETCH:
            raise ValueError(f"speed must be one of {list(SPEED_STRETCH)}")
        seq = {"left_arm": None, "left_gripper": None, "right_arm": None, "right_gripper": None}
        for arm, res in plans.items():
            seq[f"{arm}_arm"] = _stretch(res, SPEED_STRETCH[speed])
        self._env.take_dense_action(seq)

    def _in_workspace(self, pose) -> bool:
        x, y, z = pose[:3]
        w = WORKSPACE
        return w["x"][0] <= x <= w["x"][1] and w["y"][0] <= y <= w["y"][1] and w["z"][0] <= z <= w["z"][1]

    # ------------------------------------------------------------------ perception
    def objects(self) -> list[str]:
        """Movable objects detect() and grasp() accept: task objects, then objects added to the scene."""
        return list(self._genv.objects())

    def parts(self) -> list[str]:
        """Fixed or task-animated parts detect() accepts (not graspable)."""
        return list(self._genv.parts())

    def _color(self, entity):
        import scene_edits
        rgb = scene_edits.get_color(entity)
        if rgb is None:
            return None
        return [float(v) for v in np.clip(np.asarray(rgb) + self._rng.normal(0.0, 0.03, 3), 0.0, 1.0)]

    def detect(self, name: str) -> ObjectState | None:
        """Noisy pose, box, colour and keypoints of a named object or part, or None if there is none."""
        objs = self._genv.objects()
        if name not in objs:
            if name not in self._genv.parts():
                return None
            import scene_edits
            ent = self._genv.entity(name)
            seen = self._noisy(ent.get_pose())
            if hasattr(ent, "gah_extents"):        # a VirtualPart (e.g. a hole): pose and size only
                return ObjectState(name, np.asarray(seen.p, dtype=np.float64), np.asarray(seen.q, dtype=np.float64),
                                   np.asarray(ent.gah_extents), np.asarray(seen.p, dtype=np.float64), 1.0, {},
                                   round(self._genv.sim_time, 3), None)
            lo, hi = scene_edits.world_aabb(ent)
            T = _mat([*seen.p, *seen.q]) @ np.linalg.inv(_mat(ent.get_pose()))
            return ObjectState(name, np.asarray(seen.p, dtype=np.float64), np.asarray(seen.q, dtype=np.float64),
                               hi - lo, T[:3, :3] @ ((lo + hi) / 2) + T[:3, 3], 1.0, {},
                               round(self._genv.sim_time, 3), self._color(ent))
        actor = objs[name]
        if not hasattr(actor, "config"):          # a raw sapien.Entity: box from its collision shapes
            import scene_edits
            seen = self._noisy(actor.get_pose())
            lo, hi = scene_edits.world_aabb(actor)
            T = _mat([*seen.p, *seen.q]) @ np.linalg.inv(_mat(actor.get_pose()))
            return ObjectState(name, np.asarray(seen.p, dtype=np.float64), np.asarray(seen.q, dtype=np.float64),
                               hi - lo, T[:3, :3] @ ((lo + hi) / 2) + T[:3, 3], 1.0, {},
                               round(self._genv.sim_time, 3), self._color(actor))
        seen = self._noisy(actor.get_pose())
        cfg = actor.config or {}
        scale = np.ones(3) * np.asarray(cfg.get("scale", 1.0), dtype=np.float64)
        ext = np.asarray(cfg.get("extents", [0.0, 0.0, 0.0]), dtype=np.float64) * scale
        center_local = np.asarray(cfg.get("center", [0, 0, 0]), dtype=np.float64) * scale
        if ext.size != 3 or float(np.min(ext)) < 1e-3:   # no usable extents in the model data: collision shapes
            import scene_edits
            lo, hi = scene_edits.world_aabb(actor.actor)
            Rinv = t3d.quaternions.quat2mat(np.asarray(actor.get_pose().q, dtype=np.float64)).T
            corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
            local = (Rinv @ (corners - np.asarray(actor.get_pose().p)).T).T
            ext = local.max(0) - local.min(0)
            center_local = (local.max(0) + local.min(0)) / 2
        state = ObjectState(name, np.asarray(seen.p, dtype=np.float64), np.asarray(seen.q, dtype=np.float64),
                            ext, transform_point([*seen.p, *seen.q], center_local), 1.0, {},
                            round(self._genv.sim_time, 3), self._color(actor.actor))
        proxy = self._proxy(name, state)
        for kind, key in (("functional", "functional_matrix"), ("contact", "contact_points_pose")):
            for i in range(len(cfg.get(key) or [])):
                pt = proxy.get_point(kind, i, "list")
                if pt is not None:
                    state.points[f"{kind}_{i}"] = [float(v) for v in pt]
        return state

    def _obstacle_box(self, name) -> Box:
        b = true_box(name, self._actor(name))
        return Box(name, b.center + self._rng.normal(0.0, self.pos_noise, 3), b.quaternion, b.extents)

    def get_obstacles(self, exclude=()) -> list[Box]:
        """Axis-aligned boxes around every object except the robot and the table (noisy centres)."""
        return [self._obstacle_box(n) for n in self._genv.objects() if n not in exclude]

    def track(self, name: str, duration_s: float, hz: float = 10.0) -> list[ObjectState]:
        """detect(name) every 1/hz s for duration_s, holding the robot still. Advances time."""
        out = []
        n = max(1, int(round(duration_s * hz)))
        for i in range(n):
            out.append(self.detect(name))
            self._hold(HOLD_STEPS_PER_S / hz)
        return out

    def camera_frame(self, view: str = "head") -> np.ndarray:
        """RGB uint8 image. view = head | left_wrist | right_wrist | front."""
        key = {"head": "head_camera", "left_wrist": "left_camera", "right_wrist": "right_camera",
               "wrist": "right_camera", "front": "front_camera"}[view]
        self._env._update_render()
        self._env.cameras.update_picture()
        rgb = self._env.cameras.get_rgb()[key]["rgb"]
        return np.asarray(rgb if rgb.dtype == np.uint8 else (np.clip(rgb, 0, 1) * 255).astype(np.uint8))

    # ------------------------------------------------------------------ proprioception
    def ee_pose(self, arm: str) -> list:
        """Current gripper pose in the convention move_to() takes."""
        self._arm(arm)
        return list(map(float, self._robot.get_left_ee_pose() if arm == "left" else self._robot.get_right_ee_pose()))

    def tcp_pose(self, arm: str) -> list:
        """Pose of the point between the fingertips."""
        self._arm(arm)
        return list(map(float, self._robot.get_left_tcp_pose() if arm == "left" else self._robot.get_right_tcp_pose()))

    def gripper_opening(self, arm: str) -> float:
        """Measured finger opening, normalized: 0 = fully closed, 1 = fully open."""
        self._arm(arm)
        r = self._robot
        joint, ent, scale = ((r.left_gripper[0][0], r.left_entity, r.left_gripper_scale) if arm == "left"
                             else (r.right_gripper[0][0], r.right_entity, r.right_gripper_scale))
        names = [j.get_name() for j in ent.get_active_joints()]
        q = ent.get_qpos()[names.index(joint.get_name())]
        return float(np.clip((q - scale[0]) / (scale[1] - scale[0]), 0.0, 1.0))

    def time(self) -> float:
        return float(self._genv.sim_time)

    # ------------------------------------------------------------------ motion
    def move_to(self, pose, arm: str = "right", speed: str = "normal", avoid=None,
                hold_orientation: bool = False) -> bool:
        """Plan with cuRobo and execute. False if the target is outside the workspace or no
        collision-free path exists (then the arm does not move). hold_orientation keeps the gripper
        orientation fixed along the path (RoboTwin's approach constraint)."""
        self._arm(arm)
        pose = _p7(pose)
        if not self._in_workspace(pose):
            self._log("move_to", arm=arm, ok=False, why="outside workspace", pose=np.round(pose, 3).tolist())
            return False
        self._set_world(arm, avoid)
        res = self._plan(arm, pose, hold_orientation)
        self._log("move_to", arm=arm, ok=res is not None, pose=np.round(pose, 3).tolist())
        if res is None:
            return False
        self._execute({arm: res}, speed)
        return True

    def move_by(self, arm: str, dx=0.0, dy=0.0, dz=0.0, quat=None, frame: str = "world",
                speed: str = "normal", avoid=None, hold_orientation: bool = False) -> bool:
        """Relative move of the gripper. frame='world' adds (dx, dy, dz) in the world frame;
        frame='arm' moves dz backwards along the gripper's approach axis (RoboTwin's move_axis='arm')."""
        cur = np.asarray(self.ee_pose(arm), dtype=np.float64)
        if frame == "world":
            cur[:3] += [dx, dy, dz]
        elif frame == "arm":
            cur[:3] += -dz * _mat(cur)[:3, 0]
        else:
            raise ValueError("frame must be 'world' or 'arm'")
        if quat is not None:
            cur[3:] = quat
        return self.move_to(cur, arm, speed, avoid, hold_orientation)

    def move_both(self, left_pose, right_pose, speed: str = "normal", avoid=None) -> bool:
        """Plan both arms, then move them at the same time. Moves neither if either plan fails."""
        plans = {}
        for arm, pose in (("left", left_pose), ("right", right_pose)):
            pose = _p7(pose)
            if not self._in_workspace(pose):
                self._log("move_both", ok=False, why=f"{arm} target outside workspace")
                return False
            self._set_world(arm, avoid)
            plans[arm] = self._plan(arm, pose)
        ok = all(v is not None for v in plans.values())
        self._log("move_both", ok=ok)
        if ok:
            self._execute(plans, speed)
        return ok

    def back_to_origin(self, arm: str, speed: str = "normal") -> bool:
        self._arm(arm)
        home = self._robot.left_original_pose if arm == "left" else self._robot.right_original_pose
        return self.move_to(home, arm, speed)

    def open_gripper(self, arm: str, pos: float = 1.0):
        self._gripper(arm, pos)

    def close_gripper(self, arm: str, pos: float = 0.0):
        self._gripper(arm, pos)

    def _gripper(self, arm, pos):
        self._arm(arm)
        res = self._env.set_gripper(set_tag=arm, **{f"{arm}_pos": float(pos)})
        seq = {"left_arm": None, "left_gripper": None, "right_arm": None, "right_gripper": None,
               f"{arm}_gripper": res}
        self._env.take_dense_action(seq)
        self._grip_cmd[arm] = float(pos)
        self._log("gripper", arm=arm, pos=float(pos), opening=round(self.gripper_opening(arm), 3))

    def grasp_candidates(self, obj: str, arm: str = "right", pre_dis: float = 0.05, dis: float = 0.0,
                         state: ObjectState | None = None, contact_point=None, strategy: str = "auto",
                         avoid=None) -> list[dict]:
        """Reachable grasps from the object's contact points, best first:
        [{contact_point, pre_pose, pose, score}]. strategy: top | side | auto."""
        from envs._GLOBAL_CONFIGS import GRASP_DIRECTION_DIC
        from envs.utils import cal_quat_dis
        tag = self._arm(arm)
        state = state or self.detect(obj)
        if state is None:
            return []
        proxy = self._proxy(obj, state)
        self._set_world(arm, avoid, drop=(obj,))
        n = len(proxy.config.get("contact_points_pose") or [])
        ids = [contact_point] if contact_point is not None else list(range(n))
        top_ref = GRASP_DIRECTION_DIC["top_down_little_left" if arm == "right" else "top_down_little_right"]
        side_ref = GRASP_DIRECTION_DIC[self._robot.get_grasp_perfect_direction(tag)]
        out = []
        for i in ids:
            self._env.plan_success = True
            pre, pose = self._env.choose_grasp_pose(proxy, arm_tag=tag, pre_dis=pre_dis, target_dis=dis,
                                                    contact_point_id=i)
            if pre is None or pose is None or pre[0] == -1:
                continue
            top, side = cal_quat_dis(pose[-4:], top_ref), cal_quat_dis(pose[-4:], side_ref)
            score = {"top": top, "side": side, "auto": 0.7 * top + 0.3 * side}[strategy]
            out.append({"contact_point": i, "pre_pose": list(map(float, pre)), "pose": list(map(float, pose)),
                        "score": float(score)})
        self._env.plan_success = True
        return sorted(out, key=lambda c: c["score"])

    def grasp(self, obj: str, arm: str = "right", strategy: str = "auto", contact_point=None,
              pre_dis: float = 0.05, dis: float = 0.0, state: ObjectState | None = None,
              gripper_pos: float = 0.0, speed: str = "normal", avoid=None) -> bool:
        """Approach the best reachable contact point (pre-grasp, then straight in), close, and report
        gripper_holding(arm). `state` lets the caller pass its own (e.g. averaged) detection."""
        cands = self.grasp_candidates(obj, arm, pre_dis, dis, state, contact_point, strategy, avoid)
        self._log("grasp", obj=obj, arm=arm, n_candidates=len(cands))
        if not cands:
            return False
        if self._grip_cmd[arm] < 0.99:
            self.open_gripper(arm)
        c = cands[0]
        avoid_rest = [a for a in (avoid or []) if getattr(a, "name", a) != obj]
        if not self.move_to(c["pre_pose"], arm, speed, avoid_rest):
            return False
        if not self.move_to(c["pose"], arm, speed, avoid_rest, hold_orientation=True):
            return False
        self.close_gripper(arm, gripper_pos)
        held = self.gripper_holding(arm)
        self._log("grasp_done", obj=obj, arm=arm, contact_point=c["contact_point"], holding=held)
        return held

    def place_poses(self, target_pose, arm: str = "right", obj: str | None = None, functional_point=None,
                    constrain: str = "free", pre_dis: float = 0.05, dis: float = 0.0,
                    pre_dis_axis="grasp", align_axis=None, actor_axis=(1, 0, 0), actor_axis_type="actor",
                    state: ObjectState | None = None):
        """(pre_pose, pose) for the gripper so that the held object (or its functional point) lands at
        target_pose. RoboTwin's get_place_pose on the perceived object pose. constrain: free | align |
        auto. pre_dis_axis: 'grasp' (back off along the gripper) | 'fp' (along the target's z axis) |
        a vector in the target frame."""
        tag = self._arm(arm)
        state = state or self.detect(obj)
        proxy = self._proxy(obj, state)
        kw = dict(constrain=constrain, align_axis=align_axis, actor_axis=list(actor_axis),
                  actor_axis_type=actor_axis_type, functional_point_id=functional_point,
                  pre_dis_axis=pre_dis_axis)
        tp = list(map(float, _p7(target_pose)))
        self._env.plan_success = True
        pre = self._env.get_place_pose(proxy, tag, tp, pre_dis=pre_dis, **kw)
        pose = self._env.get_place_pose(proxy, tag, tp, pre_dis=dis, **kw)
        return list(map(float, pre)), list(map(float, pose))

    def place(self, pose, arm: str = "right", obj: str | None = None, functional_point=None,
              constrain: str = "free", pre_dis: float = 0.05, dis: float = 0.0, pre_dis_axis="grasp",
              align_axis=None, actor_axis=(1, 0, 0), actor_axis_type="actor", open: bool = True,
              state: ObjectState | None = None, speed: str = "normal", avoid=None) -> bool:
        """With obj=None, `pose` is the gripper target: move there and open.
        With obj, `pose` is the target for the held object (or its functional point): move to the
        pre-place pose, then to the place pose, then open. Returns whether both moves succeeded."""
        if obj is None:
            ok = self.move_to(pose, arm, speed, avoid)
        else:
            pre, fin = self.place_poses(pose, arm, obj, functional_point, constrain, pre_dis, dis,
                                        pre_dis_axis, align_axis, actor_axis, actor_axis_type, state)
            ok = self.move_to(pre, arm, speed, avoid) and self.move_to(fin, arm, speed, avoid)
        self._log("place", arm=arm, obj=obj, ok=ok)
        if ok and open:
            self.open_gripper(arm)
        return ok

    def wait(self, seconds: float):
        self._hold(seconds * HOLD_STEPS_PER_S)

    def wait_until(self, predicate, timeout_s: float, poll_s: float = 0.1) -> bool:
        t_end = self.time() + timeout_s
        while self.time() < t_end:
            if predicate():
                return True
            self._hold(poll_s * HOLD_STEPS_PER_S)
        return bool(predicate())

    # ------------------------------------------------------------------ checks
    def gripper_holding(self, arm: str) -> bool:
        """The gripper was commanded closed but its fingers stopped on something."""
        return self._grip_cmd[arm] < 0.5 and self.gripper_opening(arm) > HOLD_MIN_OPENING

    def in_region(self, obj, region) -> bool:
        """obj: name or ObjectState. region: Box, {center, extents} or {min, max} (world, axis-aligned)."""
        st = self.detect(obj) if isinstance(obj, str) else obj
        if st is None:
            return False
        if isinstance(region, Box):
            lo, hi = region.center - region.extents / 2, region.center + region.extents / 2
        elif "min" in region:
            lo, hi = np.asarray(region["min"]), np.asarray(region["max"])
        else:
            c, e = np.asarray(region["center"]), np.asarray(region["extents"])
            lo, hi = c - e / 2, c + e / 2
        p = np.asarray(st.position)
        return bool(np.all(p >= lo) and np.all(p <= hi))


HOLD_MIN_OPENING = 0.30   # ur5-wsg (SANITY.md): closed on nothing 0.186, grazing miss 0.206, on a 4 cm block 0.676
