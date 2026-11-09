"""LIBERO-Goal environments with GAH scene edits baked in.

`GAHEnv(task, variation, seed)` builds LIBERO's OffScreenRenderEnv for a LIBERO-Goal task,
loads the benchmark's fixed init state number `seed % 50`, applies the variation's scene
edits (scene_edits.py) and lets the scene settle for 1 s. The task's success check is the
official one: LIBERO evaluates the BDDL goal with its predicates (`GAHEnv.success()`).

Control: robosuite OSC_POSE (delta) at 20 Hz, as in the official benchmark.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path

BENCH = Path(__file__).resolve().parent
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")


def _ensure_libero_config():
    """LIBERO asks interactively for paths on first import unless a config exists.
    If LIBERO_CONFIG_PATH is unset, write a repo-local config (never touch ~/.libero)."""
    cur = os.environ.get("LIBERO_CONFIG_PATH")
    if cur and (Path(cur) / "config.yaml").exists():
        return
    spec = importlib.util.find_spec("libero")
    if spec is None:
        raise ImportError("LIBERO is not installed (see benchmarks/libero/README.md)")
    root = Path(list(spec.submodule_search_locations)[0]) / "libero"
    cfg = BENCH / ".libero_config"
    cfg.mkdir(exist_ok=True)
    (cfg / "config.yaml").write_text(
        f"assets: {root}/assets\nbenchmark_root: {root}\nbddl_files: {root}/bddl_files\n"
        f"datasets: {root.parent}/datasets\ninit_states: {root}/init_files\n")
    os.environ["LIBERO_CONFIG_PATH"] = str(cfg)


_ensure_libero_config()

import contextlib  # noqa: E402
import io  # noqa: E402

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

with contextlib.redirect_stdout(io.StringIO()):
    from libero.libero import benchmark as _libero_benchmark, get_libero_path  # noqa: E402
    import libero.libero.envs.bddl_utils as BDDLUtils  # noqa: E402
    from libero.libero.envs import OffScreenRenderEnv, TASK_MAPPING  # noqa: E402

import scene_edits  # noqa: E402

SUITE = "libero_goal"
TASKS = [
    "open_the_middle_drawer_of_the_cabinet",
    "turn_on_the_stove",
    "push_the_plate_to_the_front_of_the_stove",
    "put_the_wine_bottle_on_the_rack",
    "put_the_cream_cheese_in_the_bowl",
]
DEFAULT_MAX_STEPS = 3000      # 150 s of simulated time at 20 Hz control
SETTLE_STEPS = 20             # 1 s after the edits are applied
CONTROL_DT = 0.05
ROBOT_BASE_XY = np.array([-0.66, 0.0])
REACH_XY = 0.85               # horizontal reach limit for task objects [m]

_suite = None
_init_cache = {}


class StepLimitExceeded(RuntimeError):
    pass


def suite():
    global _suite
    if _suite is None:
        with contextlib.redirect_stdout(io.StringIO()):
            _suite = _libero_benchmark.get_benchmark_dict()[SUITE]()
    return _suite


def all_tasks() -> list[str]:
    return [t.name for t in suite().tasks]


def task_index(task: str) -> int:
    names = all_tasks()
    if task not in names:
        raise ValueError(f"unknown task {task}; LIBERO-Goal tasks: {names}")
    return names.index(task)


def bddl_path(task: str) -> str:
    t = suite().get_task(task_index(task))
    return os.path.join(get_libero_path("bddl_files"), t.problem_folder, t.bddl_file)


def init_states(task: str):
    if task not in _init_cache:
        with contextlib.redirect_stdout(io.StringIO()):
            _init_cache[task] = suite().get_task_init_states(task_index(task))
    return _init_cache[task]


def language(task: str) -> str:
    return suite().get_task(task_index(task)).language


class GAHEnv:
    def __init__(self, task: str, variation: dict | None = None, seed: int = 0,
                 max_steps: int = DEFAULT_MAX_STEPS, camera_size: int = 256):
        self.task, self.seed, self.max_steps = task, seed, max_steps
        self.edits = scene_edits.normalize((variation or {}).get("edits", {}))
        self.camera_size = camera_size
        bddl = bddl_path(task)
        if self.edits.get("swap_object"):
            bddl = scene_edits.swapped_bddl(bddl, self.edits["swap_object"])
        problem = BDDLUtils.get_problem_info(bddl)["problem_name"]
        base_cls = TASK_MAPPING[problem]
        edits = self.edits

        class GAHProblem(base_cls):
            def _load_model(self_):
                super()._load_model()
                scene_edits.edit_model(self_, edits)

            def _pre_action(self_, action, policy_step=False):
                super()._pre_action(action, policy_step=policy_step)
                hook = getattr(self_, "gah_substep_hook", None)
                if hook is not None:
                    hook()  # runs before every physics substep (500 Hz)

        GAHProblem.__name__ = base_cls.__name__  # LIBERO asserts class name == BDDL problem name
        TASK_MAPPING[problem] = GAHProblem
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                # ignore_done: robosuite would otherwise end the episode at its horizon (LIBERO default
                # 1000 steps, settling included); the step limit is enforced by GAHEnv.step instead.
                self.cenv = OffScreenRenderEnv(bddl_file_name=bddl, use_camera_obs=False, hard_reset=False,
                                               camera_heights=camera_size, camera_widths=camera_size,
                                               horizon=max_steps + 4 * SETTLE_STEPS + 100, ignore_done=True)
                self.cenv.reset()
        finally:
            TASK_MAPPING[problem] = base_cls
        self.env = self.cenv.env
        self.robot = self.env.robots[0]
        self.model = self.env.sim.model._model
        self.data = self.env.sim.data._data
        self.added = dict(getattr(self.env, "gah_added", {}))
        self.motion_fired = {}
        self.base_light = {k: getattr(self.model, f"light_{k}").copy() for k in ("diffuse", "specular", "ambient")}
        self.steps = 0
        self._set_init_state(init_states(task)[seed % len(init_states(task))])
        scene_edits.apply_initial_placement(self)
        self._refresh()
        self.settle(SETTLE_STEPS)  # LIBERO init states hold objects ~7 cm above the table; let them land
        scene_edits.apply_episode_edits(self)
        self._refresh()
        before = {n: self.true_pose(n)[0] for n in self.movable_objects()}
        self.settle(SETTLE_STEPS)
        self.settle_drift = {n: float(np.linalg.norm(self.true_pose(n)[0] - p)) for n, p in before.items()}
        self.steps = 0
        self._t0 = float(self.data.time)
        if self.edits.get("object_motion"):
            self.env.gah_substep_hook = lambda: scene_edits.apply_motion(self, float(self.data.time) - self._t0)

    # ------------------------------------------------------------------ state
    def _set_init_state(self, state):
        m, d = self.model, self.data
        added_roots = {self.body_id(i["root_body"]) for i in self.added.values()}
        layout = []  # (qposadr, nq, dofadr, nv) of original joints, in model order
        for j in range(m.njnt):
            if m.body_rootid[m.jnt_bodyid[j]] in added_roots:
                continue
            nq = {0: 7, 1: 4}.get(int(m.jnt_type[j]), 1)
            nv = {0: 6, 1: 3}.get(int(m.jnt_type[j]), 1)
            layout.append((m.jnt_qposadr[j], nq, m.jnt_dofadr[j], nv))
        nq0, nv0 = sum(x[1] for x in layout), sum(x[3] for x in layout)
        if len(state) != 1 + nq0 + nv0:
            raise RuntimeError(f"init state size {len(state)} != 1 + {nq0} + {nv0}")
        d.time = state[0]
        qi, vi = 1, 1 + nq0
        for qa, nq, va, nv in layout:
            d.qpos[qa:qa + nq] = state[qi:qi + nq]
            d.qvel[va:va + nv] = state[vi:vi + nv]
            qi += nq
            vi += nv
        mujoco.mj_forward(m, d)

    def _refresh(self):
        mujoco.mj_forward(self.model, self.data)
        self.env.sim.forward()
        self.cenv.check_success()
        self.env._post_process()
        self.env._update_observables(force=True)

    def step(self, action):
        self.steps += 1
        if self.steps > self.max_steps:
            raise StepLimitExceeded(f"episode exceeded {self.max_steps} control steps")
        self.cenv.step(np.asarray(action, dtype=float))

    def settle(self, n):
        """Hold the arm (zero OSC delta) with the gripper opening, for n control steps."""
        a = np.zeros(7)
        a[-1] = -1.0
        for _ in range(n):
            self.step(a)

    def success(self) -> bool:
        return bool(self.cenv.check_success())

    def close(self):
        try:
            self.cenv.close()
        except Exception:
            pass

    # ------------------------------------------------------------------ names
    def body_id(self, name):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)

    def movable_objects(self) -> list[str]:
        return list(self.env.objects_dict) + [n for n, i in self.added.items() if not i["fixed"]]

    def fixtures(self) -> list[str]:
        return list(self.env.fixtures_dict) + [n for n, i in self.added.items() if i["fixed"]]

    def object_names(self) -> list[str]:
        return self.movable_objects() + self.fixtures()

    def region_names(self) -> list[str]:
        return list(self.env.object_sites_dict)

    def resolve(self, name: str) -> str:
        """Accept 'plate_1' or 'plate' (unique prefix match) for task and added objects."""
        names = self.object_names()
        if name in names:
            return name
        cands = [n for n in names if n == f"{name}_1" or n.rsplit("_", 1)[0] == name]
        if len(cands) == 1:
            return cands[0]
        raise KeyError(f"unknown object '{name}'; known: {names}")

    def root_body(self, name) -> int:
        name = self.resolve(name)
        if name in self.added:
            return self.body_id(self.added[name]["root_body"])
        return self.body_id(self.env.get_object(name).root_body)

    def bodies_of(self, name) -> list[int]:
        root = self.root_body(name)
        m = self.model
        out = [root]
        for b in range(root + 1, m.nbody):
            p = m.body_parentid[b]
            if p in out:
                out.append(b)
        return out

    def geoms_of(self, name_or_bodies, collision=True) -> list[int]:
        bodies = set(self.bodies_of(name_or_bodies) if isinstance(name_or_bodies, str) else name_or_bodies)
        m = self.model
        coll = (m.geom_contype != 0) | (m.geom_conaffinity != 0)
        return [g for g in range(m.ngeom) if m.geom_bodyid[g] in bodies and coll[g] == collision]

    def joints_of(self, name) -> list[int]:
        bodies = set(self.bodies_of(name))
        return [j for j in range(self.model.njnt) if self.model.jnt_bodyid[j] in bodies]

    def free_joint(self, name):
        for j in self.joints_of(name):
            if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                return j
        return None

    def is_free(self, name) -> bool:
        return self.free_joint(name) is not None

    # ------------------------------------------------------------------ poses
    def true_pose(self, name):
        b = self.root_body(name)
        return self.data.xpos[b].copy(), self.data.xquat[b].copy()

    def set_free_pose(self, name, p, q):
        j = self.free_joint(name)
        a = self.model.jnt_qposadr[j]
        self.data.qpos[a:a + 3] = p
        self.data.qpos[a + 3:a + 7] = np.asarray(q) / np.linalg.norm(q)
        va = self.model.jnt_dofadr[j]
        self.data.qvel[va:va + 6] = 0.0

    def set_free_velocity(self, name, v, keep_z=False):
        j = self.free_joint(name)
        va = self.model.jnt_dofadr[j]
        if keep_z:
            self.data.qvel[va:va + 2] = v[:2]
        else:
            self.data.qvel[va:va + 3] = v

    def geom_corners(self, geoms) -> np.ndarray:
        """World-frame corners of the geoms' local AABBs, shape (8 * len(geoms), 3)."""
        m, d = self.model, self.data
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
        pts = []
        for g in geoms:
            c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
            R = d.geom_xmat[g].reshape(3, 3)
            pts.append(d.geom_xpos[g] + (c + signs * h) @ R.T)
        return np.concatenate(pts) if pts else np.zeros((0, 3))

    def lowest_point(self, name) -> float:
        pts = self.geom_corners(self.geoms_of(name, collision=True))
        return float(pts[:, 2].min())

    # ------------------------------------------------------------------ validity
    def goal_objects(self) -> list[str]:
        """Objects named in the goal or obj_of_interest (regions mapped to their parent)."""
        names = set()
        for st in self.env.parsed_problem["goal_state"]:
            names.update(st[1:])
        names.update(self.env.obj_of_interest)
        out = set()
        for n in names:
            if n in self.env.object_sites_dict:
                parent = self.env.object_sites_dict[n].parent_name if hasattr(
                    self.env.object_sites_dict[n], "parent_name") else None
                if parent:
                    out.add(parent)
            elif n in self.object_names():
                out.add(n)
        return sorted(out)

    def validity(self) -> tuple[bool, str]:
        """Checks after reset + settling: reach, above the table, no interpenetration, not already solved."""
        for n in self.goal_objects() + [n for n in self.movable_objects()]:
            p = self.true_pose(n)[0]
            dxy = float(np.linalg.norm(p[:2] - ROBOT_BASE_XY))
            if n in self.goal_objects() and dxy > REACH_XY:
                return False, f"{n} is out of reach ({dxy:.2f} m from the robot base)"
            if p[2] < scene_edits.TABLE_TOP_Z - 0.05:
                return False, f"{n} is below the table top (z={p[2]:.3f})"
        moving = {self.resolve(s["name"]) for s in self.edits.get("object_motion", [])}
        for n, drift in self.settle_drift.items():
            if n not in moving and drift > 0.03:
                return False, f"{n} moved {drift:.3f} m while settling (interpenetration or unstable pose)"
        if self.success():
            return False, "the goal is already satisfied after reset"
        ok, why = self.articulation_check()
        if not ok:
            return False, why
        return True, "ok"

    def articulation_check(self, step=0.005, tol=0.002) -> tuple[bool, str]:
        """For articulated goals (Open/Close/TurnOn/TurnOff), sweep the goal joint from its current
        value in a scratch copy of the state and check that a value satisfying the goal is reachable
        without the moving part penetrating a fixed body (robot and movable objects are ignored,
        since they can be moved out of the way)."""
        m = self.model
        methods = {"open": "is_open", "close": "is_close", "turnon": "turn_on", "turnoff": "turn_off"}
        free_roots = {m.body_rootid[m.jnt_bodyid[j]] for j in range(m.njnt)
                      if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE}
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "" for b in range(m.nbody)]
        robot = np.array([n.startswith(("robot0_", "gripper0_", "mount0_")) for n in names])
        for st in self.env.parsed_problem["goal_state"]:
            pred = str(st[0]).lower()
            if pred not in methods or len(st) != 2:
                continue
            target = st[1]
            if target in self.env.object_sites_dict and hasattr(self.env.object_sites_dict[target], "parent_name"):
                site = self.env.object_sites_dict[target]
                obj, joints = self.env.get_object(site.parent_name), site.joints
            else:
                obj = self.env.get_object(target)
                joints = obj.joints
            check = getattr(obj, methods[pred])
            reachable_any = False
            for jn in joints:
                j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if j < 0:
                    continue
                part = self._subtree(m.jnt_bodyid[j])
                own_root = m.body_rootid[m.jnt_bodyid[j]]
                d2 = mujoco.MjData(m)
                d2.qpos[:] = self.data.qpos
                adr = m.jnt_qposadr[j]
                q0 = float(self.data.qpos[adr])
                base = self._part_hits(d2, part, own_root, free_roots, robot, tol)
                lo, hi = m.jnt_range[j]
                for end in (lo, hi):
                    n = max(1, int(abs(end - q0) / step))
                    for i in range(n + 1):
                        q = q0 + (end - q0) * i / n
                        d2.qpos[adr] = q
                        if self._part_hits(d2, part, own_root, free_roots, robot, tol) - base:
                            break
                        if check(q):
                            reachable_any = True
                            break
                    if reachable_any:
                        break
                if reachable_any:
                    break
            if not reachable_any:
                return False, (f"goal ({pred} {target}) is unreachable: the moving part hits a fixed body "
                               f"before the goal position")
        return True, "ok"

    def _subtree(self, body):
        m = self.model
        out = {body}
        for b in range(body + 1, m.nbody):
            if m.body_parentid[b] in out:
                out.add(b)
        return out

    def _part_hits(self, d2, part, own_root, free_roots, robot, tol):
        m = self.model
        mujoco.mj_fwdPosition(m, d2)
        hits = set()
        for i in range(d2.ncon):
            c = d2.contact[i]
            if c.dist >= -tol:
                continue
            b1, b2 = m.geom_bodyid[c.geom[0]], m.geom_bodyid[c.geom[1]]
            for a, b in ((b1, b2), (b2, b1)):
                if a in part and m.body_rootid[b] != own_root and m.body_rootid[b] not in free_roots \
                        and not robot[b]:
                    hits.add((int(c.geom[0]), int(c.geom[1])))
        return hits


def make_env(task, variation=None, seed=0, **kw) -> GAHEnv:
    return GAHEnv(task, variation, seed, **kw)
