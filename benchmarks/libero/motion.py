"""IK and collision-aware joint-space planning for the Panda in a LIBERO scene.

Planning runs on a scratch copy of the MuJoCo data (the live sim is never touched):
  - ik():        damped least squares on the grip site, with random restarts;
  - collides():  full robot (links 1-7, hand, fingers) plus the held object against every
                 other collision geom in the scene (MuJoCo's own collision detection);
  - plan():      straight joint-space segment if free, otherwise RRT-Connect + shortcutting.
The resulting joint path is executed by api.py through robosuite's OSC controller.
"""

from __future__ import annotations

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ARM_JOINTS = [f"robot0_joint{i}" for i in range(1, 8)]
FINGER_JOINTS = ["gripper0_finger_joint1", "gripper0_finger_joint2"]
GRIP_SITE = "gripper0_grip_site"
STATIC_ROBOT_BODIES = {"robot0_base", "robot0_link0"}


def rotvec(R_err):
    return Rotation.from_matrix(R_err).as_rotvec()


class Planner:
    def __init__(self, genv, rng: np.random.Generator):
        self.genv = genv
        self.m = genv.model
        self.d = mujoco.MjData(self.m)
        self.rng = rng
        m = self.m
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in ARM_JOINTS]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jid])
        self.vadr = np.array([m.jnt_dofadr[j] for j in jid])
        self.lo = m.jnt_range[jid, 0] + 1e-3
        self.hi = m.jnt_range[jid, 1] - 1e-3
        self.site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, GRIP_SITE)
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "" for b in range(m.nbody)]
        self.robot_moving = np.array([(n.startswith("robot0_") or n.startswith("gripper0_"))
                                      and n not in STATIC_ROBOT_BODIES for n in names])
        self.robot_static = np.array([n in STATIC_ROBOT_BODIES or n.startswith("mount0_") for n in names])
        self.home = np.clip(np.asarray(genv.robot.init_qpos, float), self.lo, self.hi)
        self.jacp = np.zeros((3, m.nv))
        self.jacr = np.zeros((3, m.nv))

    # ---------------------------------------------------------------- kinematics
    def sync(self):
        self.d.qpos[:] = self.genv.data.qpos
        self.d.qvel[:] = 0.0
        mujoco.mj_kinematics(self.m, self.d)

    def q_now(self):
        return self.genv.data.qpos[self.qadr].copy()

    def fk(self, q):
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)  # cdof/subtree_com, so mj_jacSite matches this configuration
        return self.d.site_xpos[self.site].copy(), self.d.site_xmat[self.site].reshape(3, 3).copy()

    def ik(self, p, R, q_seed, iters=200, restarts=10, tol_p=2e-3, tol_r=2e-2, ctx=None):
        """Return IK solutions sorted by distance to q_seed (collision-free ones only if ctx)."""
        sols = []
        home = self.home
        for k in range(restarts):
            # seeds: current, home posture, then perturbations of each (alternating)
            base = q_seed if k % 2 == 0 else home
            q = base.copy() if k < 2 else np.clip(
                base + self.rng.normal(0, 0.5 if k < restarts // 2 else 1.0, 7), self.lo, self.hi)
            q = self._dls(p, R, q, iters, tol_p, tol_r)
            if q is None:
                continue
            if ctx is not None and self.collides(q, ctx):
                continue
            sols.append(q)
            if len(sols) == 1 and np.linalg.norm(q - q_seed) < 1.0:
                break
        return sorted(sols, key=lambda q: float(np.linalg.norm(q - q_seed)))

    def _dls(self, p, R, q, iters, tol_p, tol_r, lam=0.05):
        for _ in range(iters):
            pc, Rc = self.fk(q)
            ep = p - pc
            er = rotvec(R @ Rc.T)
            if np.linalg.norm(ep) < tol_p and np.linalg.norm(er) < tol_r:
                return q
            mujoco.mj_jacSite(self.m, self.d, self.jacp, self.jacr, self.site)
            J = np.vstack([self.jacp[:, self.vadr], self.jacr[:, self.vadr]])
            e = np.concatenate([ep, er])
            dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
            n = np.linalg.norm(dq)
            if n > 0.4:
                dq *= 0.4 / n
            q = np.clip(q + dq, self.lo, self.hi)
        return None

    # ---------------------------------------------------------------- collisions
    def context(self, attached=None, allow=(), start_q=None, start_radius=0.05):
        """attached: name of the held free object (moves with the hand).
        allow: names of objects the robot may touch during this motion.
        Contacts present at start_q are ignored while the grip site is within start_radius of it."""
        g = self.genv
        ctx = {"moving": self.robot_moving.copy(), "ignore": self.robot_static.copy(),
               "attached": None, "start_pairs": set(), "start_p": None, "radius": start_radius}
        for name in allow:
            for b in g.bodies_of(name):
                ctx["ignore"][b] = True
        if attached is not None:
            j = g.free_joint(attached)
            b = g.root_body(attached)
            ps, Rs = self.fk(self.q_now()) if start_q is None else self.fk(start_q)
            Ts = np.eye(4); Ts[:3, :3] = Rs; Ts[:3, 3] = ps
            To = np.eye(4); To[:3, :3] = g.data.xmat[b].reshape(3, 3); To[:3, 3] = g.data.xpos[b]
            ctx["attached"] = (g.model.jnt_qposadr[j], np.linalg.inv(Ts) @ To)
            for bb in g.bodies_of(attached):
                ctx["moving"][bb] = True
        if start_q is not None:
            ctx["start_pairs"] = self._penetrating_pairs(start_q, ctx)
            ctx["start_p"] = self.fk(start_q)[0]
        return ctx

    def _place(self, q, ctx):
        p, R = self.fk(q)
        if ctx["attached"] is not None:
            adr, T = ctx["attached"]
            Ts = np.eye(4); Ts[:3, :3] = R; Ts[:3, 3] = p
            To = Ts @ T
            x, y, z, w = Rotation.from_matrix(To[:3, :3]).as_quat()
            self.d.qpos[adr:adr + 3] = To[:3, 3]
            self.d.qpos[adr + 3:adr + 7] = [w, x, y, z]
        mujoco.mj_fwdPosition(self.m, self.d)
        return p

    def _penetrating_pairs(self, q, ctx):
        self._place(q, ctx)
        return {pair for pair, _ in self._hits(ctx)}

    def _hits(self, ctx):
        d, m = self.d, self.m
        n = d.ncon
        if n == 0:
            return []
        geom = d.contact.geom[:n]
        dist = d.contact.dist[:n]
        b1, b2 = m.geom_bodyid[geom[:, 0]], m.geom_bodyid[geom[:, 1]]
        mv, ig = ctx["moving"], ctx["ignore"]
        bad = (dist < -1e-4) & ((mv[b1] & ~mv[b2] & ~ig[b2]) | (mv[b2] & ~mv[b1] & ~ig[b1]))
        return [((int(geom[i, 0]), int(geom[i, 1])), float(dist[i])) for i in np.nonzero(bad)[0]]

    def collides(self, q, ctx) -> bool:
        p = self._place(q, ctx)
        hits = self._hits(ctx)
        if not hits:
            return False
        if ctx["start_pairs"] and np.linalg.norm(p - ctx["start_p"]) < ctx["radius"]:
            hits = [h for h in hits if h[0] not in ctx["start_pairs"]]
        return bool(hits)

    def colliding_names(self, q, ctx):
        self._place(q, ctx)
        m = self.m
        out = set()
        for (g1, g2), _ in self._hits(ctx):
            for g in (g1, g2):
                b = m.geom_bodyid[g]
                if not ctx["moving"][b]:
                    out.add(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b))
        return sorted(out)

    # ---------------------------------------------------------------- planning
    def segment_free(self, q0, q1, ctx, res=0.03):
        n = max(1, int(np.ceil(np.max(np.abs(q1 - q0)) / res)))
        return all(not self.collides(q0 + (q1 - q0) * (i / n), ctx) for i in range(1, n + 1))

    def plan(self, q0, q1, ctx, max_iters=2500, step=0.2):
        if self.collides(q1, ctx):
            return None
        if self.segment_free(q0, q1, ctx):
            return [q0, q1]
        ta, tb = [q0], [q1]
        pa, pb = [-1], [-1]
        for _ in range(max_iters):
            qr = self.rng.uniform(self.lo, self.hi) if self.rng.random() > 0.1 else q1
            ia, qa = self._extend(ta, pa, qr, ctx, step)
            if qa is None:
                ta, tb, pa, pb = tb, ta, pb, pa
                continue
            ib, qb = self._connect(tb, pb, qa, ctx, step)
            if qb is not None and np.linalg.norm(qb - qa) < 1e-6:
                path_a = self._trace(ta, pa, ia)
                path_b = self._trace(tb, pb, ib)
                path = path_a[::-1] + path_b
                if not np.allclose(path[0], q0):
                    path = path[::-1]
                return self._shortcut(path, ctx)
            ta, tb, pa, pb = tb, ta, pb, pa
        return None

    def _nearest(self, tree, q):
        arr = np.asarray(tree)
        return int(np.argmin(np.linalg.norm(arr - q, axis=1)))

    def _extend(self, tree, parent, q, ctx, step):
        i = self._nearest(tree, q)
        qn = tree[i]
        dq = q - qn
        dist = np.linalg.norm(dq)
        qnew = q if dist <= step else qn + dq * (step / dist)
        if not self.segment_free(qn, qnew, ctx):
            return None, None
        tree.append(qnew)
        parent.append(i)
        return len(tree) - 1, qnew

    def _connect(self, tree, parent, q, ctx, step):
        last, qlast = None, None
        while True:
            i, qn = self._extend(tree, parent, q, ctx, step)
            if qn is None:
                return last, qlast
            last, qlast = i, qn
            if np.linalg.norm(qn - q) < 1e-6:
                return last, qlast

    @staticmethod
    def _trace(tree, parent, i):
        out = []
        while i != -1:
            out.append(tree[i])
            i = parent[i]
        return out

    def _shortcut(self, path, ctx, tries=60):
        path = list(path)
        for _ in range(tries):
            if len(path) <= 2:
                break
            i, j = sorted(self.rng.choice(len(path), 2, replace=False))
            if j - i < 2:
                continue
            if self.segment_free(path[i], path[j], ctx):
                path = path[:i + 1] + path[j:]
        return path
