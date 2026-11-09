"""Primitive API for the ManiSkill3 backend (Panda, single arm).

Implements docs/api_spec.md. Pipelines define `run(api)` and use only this object.
Poses are (p, q) tuples: p = xyz in metres (world frame), q = quaternion wxyz.

Perception is not ground truth: every detect()/get_obstacles()/task_frames() call
returns the simulator pose plus fresh zero-mean Gaussian noise (default 1 cm, 3 deg),
independently per call.

Extensions beyond docs/api_spec.md (needed for these tasks, see ISSUES.md):
  tcp_pose(), task_frames(), objects(), pose helpers (pose_mul, pose_inv, rpy_to_quat),
  move_to(..., avoid=[names]) and grasp(..., offset=[x, y, z], depth=...).
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import sapien
import trimesh
from transforms3d.euler import euler2quat
from transforms3d.quaternions import axangle2quat, qmult, quat2mat

from mani_skill.examples.motionplanning.base_motionplanner.utils import (
    compute_grasp_info_by_obb,
    get_actor_obb,
)
from mani_skill.examples.motionplanning.panda.motionplanner import PandaArmMotionPlanningSolver

SPEEDS = {"slow": 0.3, "normal": 0.9}
FINGER_LENGTH = 0.025


@dataclass
class ObjectState:
    name: str
    position: np.ndarray      # (3,) world xyz [m]
    quaternion: np.ndarray    # (4,) wxyz
    bbox_extents: np.ndarray  # (3,) full extents of the oriented bounding box [m]
    confidence: float = 1.0


@dataclass
class Box:
    name: str
    center: np.ndarray        # (3,)
    quaternion: np.ndarray    # (4,) wxyz
    extents: np.ndarray       # (3,) full extents [m]


# --------------------------------------------------------------------------- #
# Pose helpers
# --------------------------------------------------------------------------- #
def _to_sapien(pose) -> sapien.Pose:
    if isinstance(pose, sapien.Pose):
        return pose
    if isinstance(pose, ObjectState):
        return sapien.Pose(np.asarray(pose.position, np.float32), np.asarray(pose.quaternion, np.float32))
    if isinstance(pose, dict):
        return sapien.Pose(np.asarray(pose["p"], np.float32), np.asarray(pose.get("q", [1, 0, 0, 0]), np.float32))
    arr = np.asarray(pose[0] if isinstance(pose, tuple) else pose, dtype=np.float64)
    if isinstance(pose, tuple):
        return sapien.Pose(np.asarray(pose[0], np.float32), np.asarray(pose[1], np.float32))
    if arr.shape == (4, 4):
        return sapien.Pose(arr)
    if arr.size == 7:
        return sapien.Pose(arr[:3].astype(np.float32), arr[3:].astype(np.float32))
    if arr.size == 3:
        return sapien.Pose(arr.astype(np.float32))
    raise ValueError(f"cannot interpret pose {pose!r}")


def _to_tuple(pose: sapien.Pose):
    return np.asarray(pose.p, dtype=np.float64), np.asarray(pose.q, dtype=np.float64)


def pose_mul(a, b):
    """a * b (apply b in a's frame)."""
    return _to_tuple(_to_sapien(a) * _to_sapien(b))


def pose_inv(a):
    return _to_tuple(_to_sapien(a).inv())


def rpy_to_quat(roll, pitch, yaw):
    return np.array(euler2quat(roll, pitch, yaw))


def _np(x):
    return x.cpu().numpy().reshape(-1) if hasattr(x, "cpu") else np.asarray(x).reshape(-1)


def _matrix(pose: sapien.Pose) -> np.ndarray:
    """4x4 float64 transform with an exactly orthonormal rotation."""
    q = np.asarray(pose.q, dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = quat2mat(q / np.linalg.norm(q))
    T[:3, 3] = np.asarray(pose.p, dtype=np.float64)
    return T


# --------------------------------------------------------------------------- #
class RobotAPI:
    def __init__(self, env, noise_seed: int = 0, pos_noise: float | None = None,
                 rot_noise_deg: float | None = None):
        self.env = env
        self.rng = np.random.default_rng(noise_seed)
        self.pos_noise = float(os.environ.get("GAH_POS_NOISE", 0.01) if pos_noise is None else pos_noise)
        self.rot_noise = np.deg2rad(float(os.environ.get("GAH_ROT_NOISE_DEG", 3.0) if rot_noise_deg is None else rot_noise_deg))
        self.planner = PandaArmMotionPlanningSolver(
            env, debug=False, vis=False, base_pose=env.agent.robot.pose,
            visualize_target_grasp_pose=False, print_env_info=False,
            joint_vel_limits=SPEEDS["normal"], joint_acc_limits=SPEEDS["normal"],
        )

    # ------------------------------------------------------------------ helpers
    def _arm(self, arm):
        if arm not in ("right", None):
            raise ValueError("ManiSkill Panda has a single arm; use arm='right'")

    def _noisy(self, pose: sapien.Pose) -> sapien.Pose:
        p = np.asarray(pose.p, dtype=np.float64) + self.rng.normal(0.0, self.pos_noise, 3)
        axis = self.rng.normal(size=3)
        axis /= np.linalg.norm(axis) + 1e-12
        dq = axangle2quat(axis, self.rng.normal(0.0, self.rot_noise))
        q = qmult(dq, np.asarray(pose.q, dtype=np.float64))
        return sapien.Pose(p.astype(np.float32), q.astype(np.float32))

    def _true_pose(self, actor) -> sapien.Pose:
        return sapien.Pose(_np(actor.pose.p).astype(np.float32), _np(actor.pose.q).astype(np.float32))

    def _hold_action(self):
        qpos = self.env.agent.robot.get_qpos().cpu().numpy()[0]
        return np.concatenate([qpos[:7], [self.planner.gripper_state]])

    def _step_hold(self, n: int = 1):
        for _ in range(n):
            self.env.step(self._hold_action())

    def _set_speed(self, speed):
        if speed not in SPEEDS:
            raise ValueError(f"speed must be one of {list(SPEEDS)}")
        lim = SPEEDS[speed]
        mp = self.planner.planner
        mp.joint_vel_limits = np.ones_like(np.asarray(mp.joint_vel_limits)) * lim
        mp.joint_acc_limits = np.ones_like(np.asarray(mp.joint_acc_limits)) * lim

    # --------------------------------------------------------------- perception
    def objects(self) -> list[str]:
        """Names of all task objects and added scene objects."""
        return self.env.gah_object_names()

    def detect(self, name: str) -> ObjectState | None:
        try:
            actor = self.env.gah_actor(name)
        except KeyError:
            return None
        pose = self._noisy(self._true_pose(actor))
        extents = np.asarray(get_actor_obb(actor).primitive.extents, dtype=np.float64)
        return ObjectState(name, np.asarray(pose.p, np.float64), np.asarray(pose.q, np.float64), extents)

    def get_obstacles(self) -> list[Box]:
        """Oriented boxes of every object in the scene except the robot, table and ground."""
        boxes = []
        for name in self.objects():
            actor = self.env.gah_actor(name)
            obb = get_actor_obb(actor)
            T = np.asarray(obb.primitive.transform, dtype=np.float64)
            noisy = self._noisy(sapien.Pose(T))
            boxes.append(Box(name, np.asarray(noisy.p, np.float64), np.asarray(noisy.q, np.float64),
                             np.asarray(obb.primitive.extents, np.float64)))
        return boxes

    def task_frames(self) -> dict:
        """Task-specific frames as noisy (p, q) tuples (extension; see ISSUES.md)."""
        env, frames = self.env, {}
        task = env.gah_task
        if task == "PegInsertionSide-v1":
            frames["box_hole"] = env.box_hole_pose
            frames["peg_head"] = env.peg_head_pose
        elif task == "PlugCharger-v1":
            frames["goal"] = env.goal_pose
            frames["charger_base"] = env.charger_base_pose
        elif task == "PlaceSphere-v1":
            frames["bin"] = env.bin.pose
        out = {}
        for k, v in frames.items():
            sp = sapien.Pose(_np(v.p).astype(np.float32), _np(v.q).astype(np.float32))
            out[k] = _to_tuple(self._noisy(sp))
        return out

    def track(self, name: str, duration_s: float):
        """Hold the arm still for duration_s and return [(t, ObjectState), ...] each control step."""
        dt = self.env.control_timestep
        hist = []
        for i in range(max(1, int(round(duration_s / dt)))):
            self._step_hold()
            hist.append(((i + 1) * dt, self.detect(name)))
        return hist

    def camera_frame(self, view: str = "head") -> np.ndarray:
        if view != "head":
            raise NotImplementedError("only view='head' is available in the ManiSkill backend")
        img = self.env.render()
        return img.cpu().numpy()[0] if hasattr(img, "cpu") else np.asarray(img)[0]

    def tcp_pose(self):
        """Current gripper TCP pose (p, q) (extension; exact, from proprioception)."""
        tcp = self.env.agent.tcp.pose
        return _np(tcp.p), _np(tcp.q)

    # ------------------------------------------------------------------- motion
    def move_to(self, pose, arm: str = "right", speed: str = "normal", avoid=None) -> bool:
        """Move the TCP to pose. avoid: list of object names added as collision boxes
        (inflated by 1 cm) for this motion. Tries a screw motion, then RRT-Connect."""
        self._arm(arm)
        self._set_speed(speed)
        target = _to_sapien(pose)
        self.planner.clear_collisions()
        if avoid:
            for box in self.get_obstacles():
                if box.name in avoid:
                    self.planner.add_box_collision(box.extents + 0.02, sapien.Pose(box.center, box.quaternion))
        res = self.planner.move_to_pose_with_screw(target)
        if res == -1:
            res = self.planner.move_to_pose_with_RRTConnect(target)
        self.planner.clear_collisions()
        return res != -1

    def grasp(self, obj: str, arm: str = "right", strategy: str = "auto",
              offset=(0.0, 0.0, 0.0), depth: float = FINGER_LENGTH, approach_dist: float = 0.05,
              strength: float = 1.0) -> bool:
        """Grasp obj using its (noisy) oriented bounding box.
        strategy: top (approach along -z), side (horizontal, from the robot base), auto (= top).
        offset: shift of the grasp point in the grasp frame [m].
        strength: passed to close_gripper (lower lets the object pivot in the fingers).
        Returns True if holding afterwards."""
        self._arm(arm)
        actor = self.env.gah_actor(obj)
        true_obb = get_actor_obb(actor)
        true_pose = self._true_pose(actor)
        noisy_pose = self._noisy(true_pose)
        delta = _matrix(noisy_pose) @ np.linalg.inv(_matrix(true_pose))
        obb = trimesh.primitives.Box(extents=np.asarray(true_obb.primitive.extents, np.float64),
                                     transform=delta @ np.asarray(true_obb.primitive.transform, np.float64))
        if strategy in ("top", "auto"):
            approaching = np.array([0.0, 0.0, -1.0])
        elif strategy == "side":
            d = np.asarray(noisy_pose.p) - _np(self.env.agent.robot.pose.p)
            d[2] = 0.0
            approaching = d / (np.linalg.norm(d) + 1e-9)
        else:
            raise ValueError("strategy must be top | side | auto")
        closing_hint = self.env.agent.tcp.pose.to_transformation_matrix()[0, :3, 1].cpu().numpy()
        info = compute_grasp_info_by_obb(obb, approaching=approaching, target_closing=closing_hint, depth=depth)
        grasp_pose = self.env.agent.build_grasp_pose(approaching, info["closing"], info["center"])
        grasp_pose = grasp_pose * sapien.Pose(np.asarray(offset, np.float32))
        self.open_gripper()
        if not self.move_to(grasp_pose * sapien.Pose([0, 0, -approach_dist])):
            return False
        if not self.move_to(grasp_pose):
            return False
        self.close_gripper(strength=strength)
        return bool(_np(self.env.agent.is_grasping(actor))[0])

    def place(self, pose, arm: str = "right") -> bool:
        """Move the held object's TCP to pose and open the gripper."""
        ok = self.move_to(pose, arm=arm)
        self.open_gripper(arm)
        return ok

    def open_gripper(self, arm: str = "right"):
        self._arm(arm)
        self.planner.open_gripper()

    def close_gripper(self, arm: str = "right", strength: float = 1.0):
        """strength in (0, 1]; lower values close the fingers less tightly."""
        self._arm(arm)
        self.planner.close_gripper(gripper_state=-float(strength))

    def wait_until(self, predicate, timeout_s: float) -> bool:
        dt = self.env.control_timestep
        for _ in range(max(1, int(round(timeout_s / dt)))):
            if predicate():
                return True
            self._step_hold()
        return bool(predicate())

    # ------------------------------------------------------------------- checks
    def gripper_holding(self, arm: str = "right") -> bool:
        self._arm(arm)
        for name in self.objects():
            if bool(_np(self.env.agent.is_grasping(self.env.gah_actor(name)))[0]):
                return True
        return False

    def in_region(self, obj: str, region: dict) -> bool:
        """region: {"center": [x, y, z], "half_extents": [hx, hy, hz]} (axis-aligned, world)."""
        st = self.detect(obj)
        c = np.asarray(region["center"], np.float64)
        h = np.asarray(region["half_extents"], np.float64)
        return bool(np.all(np.abs(st.position - c) <= h))
