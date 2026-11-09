"""place_block_belt: place the block on the belt so it rides upright into the moving exit bowl.

Plan
1. Perceive the static layout (belt, place line, blocker) and the block; infer the flow direction
   (from the block's side of the belt towards the far end) and the placing arm (the block's side).
2. Measure the belt speed from the head camera (the belt's tread pattern moves; phase shift of the
   pattern between frames) and keep sampling the bowl's y while doing anything else.
3. Grasp the block top-down with the fingers closing across the flow, lift, carry it high above the
   release point (before the place line, in the lane clear of the blocker), and set it down.
4. Fit the bowl's back-and-forth motion (triangle wave), predict when the block will reach the belt
   end, and open the gripper at the time that makes the bowl be under the block when it falls.
"""
from __future__ import annotations

import numpy as np

# ---------------------------------------------------------------- tunables
RELEASE_LAG_S = 0.78      # open command -> fingers off the block (measured: 0.76-0.80 s)
BELT_ARM_S = 0.05         # block resting & free -> belt starts carrying it
FALL_S = 0.08             # block leaves the belt end -> mid-fall above the bowl
CATCH_HALF_W = 0.045      # |bowl_y - lane_y| that still catches the falling block
LINE_MARGIN = 0.035       # release this far before the place line (block centre)
EDGE_MARGIN = 0.08        # lane centre this far inside the belt's outer bbox edge
CARRY_CLEAR = 0.07        # block bottom above the belt top while carrying
SET_CLEAR = 0.004         # block bottom above the belt top when released
FINGER_REACH = 0.043      # open fingertips below the tcp
TABLE_CLEAR = 0.010       # fingertips above the table when grasping


def note(api, **kw):
    api.trace.append({"t": round(api.time(), 2), "call": "note", **kw})


# ---------------------------------------------------------------- geometry helpers
def quat_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_quat(R):
    import transforms3d as t3d
    return list(map(float, t3d.quaternions.mat2quat(R)))


def world_extents(st):
    """World-axis extents of a box resting level (its most vertical axis is taken as exactly vertical,
    so a few degrees of perceived tilt do not inflate the height of a long, flat box)."""
    R = quat_mat(st.quaternion)
    e = np.asarray(st.bbox_extents, dtype=np.float64)
    u = int(np.argmax(np.abs(R[2])))
    j, k = [i for i in range(3) if i != u]
    th = np.arctan2(R[1, j], R[0, j])
    c, s = abs(np.cos(th)), abs(np.sin(th))
    return np.array([c * e[j] + s * e[k], s * e[j] + c * e[k], e[u]])


def avg_detect(api, name, n=5):
    """Mean of n detections (position, box centre, sign-aligned quaternion)."""
    sts = [api.detect(name) for _ in range(n)]
    sts = [s for s in sts if s is not None]
    if not sts:
        return None
    s = sts[0]
    s.position = np.mean([x.position for x in sts], 0)
    s.bbox_center = np.mean([x.bbox_center for x in sts], 0)
    q0 = np.asarray(sts[0].quaternion, dtype=np.float64)
    qs = [np.asarray(x.quaternion) * (1 if np.dot(x.quaternion, q0) >= 0 else -1) for x in sts]
    q = np.mean(qs, 0)
    s.quaternion = q / np.linalg.norm(q)
    return s


# ---------------------------------------------------------------- bowl model
class BowlTracker:
    def __init__(self, api):
        self.api = api
        self.t, self.y = [], []
        self.fit = None

    def sample(self):
        s = self.api.detect("bowl")
        if s is not None:
            self.t.append(self.api.time())
            self.y.append(float(s.position[1]))

    def hold(self, seconds, hz=20.0):
        """Hold still for `seconds`, sampling the bowl."""
        n = max(1, int(round(seconds * hz)))
        for _ in range(n):
            self.sample()
            self.api.wait(1.0 / hz)

    @staticmethod
    def tri(t, lo, hi, v, t0):
        L = max(hi - lo, 1e-6)
        s = np.mod(v * (np.asarray(t) - t0), 2 * L)
        return lo + np.where(s <= L, s, 2 * L - s)

    def fit_model(self):
        from scipy.optimize import least_squares
        t, y = np.asarray(self.t), np.asarray(self.y)
        lo0, hi0 = np.percentile(y, 3), np.percentile(y, 97)
        # speed guess from robust finite differences over short gaps
        dt = np.diff(t)
        dv = np.abs(np.diff(y)) / np.maximum(dt, 1e-3)
        ok = (dt > 0.03) & (dt < 0.2)
        v0 = float(np.median(dv[ok])) if ok.sum() > 5 else 0.3
        best = None
        L = max(hi0 - lo0, 0.05)
        for vv in (v0 * 0.9, v0, v0 * 1.1):
            per = 2 * L / max(vv, 1e-3)
            for t0 in np.linspace(t[0], t[0] + per, 24, endpoint=False):
                r = least_squares(lambda p: self.tri(t, *p) - y, [lo0, hi0, vv, t0], loss="soft_l1", f_scale=0.02)
                if best is None or r.cost < best.cost:
                    best = r
        self.fit = best.x
        resid = self.tri(t, *best.x) - y
        return {"lo": round(best.x[0], 4), "hi": round(best.x[1], 4), "v": round(best.x[2], 4),
                "rms": round(float(np.sqrt(np.mean(resid ** 2))), 4), "n": len(t), "span": round(t[-1] - t[0], 2)}

    def predict(self, t):
        return self.tri(t, *self.fit)


# ---------------------------------------------------------------- belt speed from the camera
def _sliding_min(a, k):
    pad = k // 2
    ap = np.pad(a, ((0, 0), (pad, pad)), mode="edge")
    return np.lib.stride_tricks.sliding_window_view(ap, k, axis=1).min(-1)


def _dark_run(gray_row, darkish_row, dark_thr):
    """Longest dark run of one image row -> (a, b) or None."""
    W = len(gray_row)
    if darkish_row.sum() < 40:
        return None
    d = np.diff(np.concatenate([[0], darkish_row.astype(int), [0]]))
    starts, ends = np.where(d == 1)[0], np.where(d == -1)[0] - 1
    j = int(np.argmax(ends - starts))
    a, b = starts[j], ends[j]
    if b - a < 40 or a <= 2 or b >= W - 3:
        return None
    ins = np.where(gray_row[a:b + 1] < dark_thr)[0]
    return a + ins.min(), a + ins.max()              # undo the min-filter dilation


def speed_from_frames(frames, times, belt_len, dark_thr=110.0, mot_thr=12.0, max_shift=6.0):
    """Speed [m/s, signed along image columns] of the moving surface pattern of a dark belt seen in
    `frames` (n, H, W, 3), with the camera and the arms still.
    Pass 1, per frame and image row: the longest dark run is the belt top; its two ends are the belt's
      end edges, which are straight lines in the image: fit each robustly over rows (RANSAC), so rows
      where an arm or the block cuts or extends the run do not count. A moving dark object next to the
      belt (the bowl) corrupts only some frames: take the median over frames of the fitted run length.
      Pixels per metre per row = run length / belt_len.
    Pass 2, per row: shift of the pattern between consecutive frames (least squares over the moving,
      non-red pixels strictly inside the fitted edges) -> pixels per second -> metres per second."""
    F = np.asarray(frames, dtype=np.float64)
    # exposure normalisation: the thresholds below are relative to the bright background (table/wall),
    # mapped to 230, so dimmer or brighter lighting does not move the belt across a fixed gray level
    bright = float(np.percentile(F[0].mean(-1), 90))
    F = F * (230.0 / max(bright, 1.0))
    gray = F.mean(-1)
    red = (F[0, :, :, 0] - np.maximum(F[0, :, :, 1], F[0, :, :, 2])) > 60
    motion = np.abs(np.diff(gray, axis=0)).mean(0)
    H, W = gray.shape[1:]
    top_rows = []
    for r in range(H):
        run = _dark_run(gray[0, r], _sliding_min(gray[0, r:r + 1], 11)[0] < dark_thr, dark_thr)
        if run is not None and (motion[r, run[0]:run[1] + 1] > mot_thr).mean() >= 0.5:
            top_rows.append(r)
    if len(top_rows) < 8:
        return None, {"why": "belt top not found", "rows": len(top_rows)}
    rows = np.asarray(top_rows, float)
    a_fits, b_fits = [], []
    for i in range(len(F)):
        dk = _sliding_min(gray[i, top_rows], 11) < dark_thr
        A, B = [], []
        for k, r in enumerate(top_rows):
            run = _dark_run(gray[i, r], dk[k], dark_thr)
            A.append(np.nan if run is None else run[0])
            B.append(np.nan if run is None else run[1])
        A, B = np.asarray(A, float), np.asarray(B, float)
        if np.isfinite(A).sum() < 8:
            continue
        a_fit, a_in = ransac_line(rows, A)
        b_fit, b_in = ransac_line(rows, B)
        if a_in.sum() >= 0.25 * len(rows) and b_in.sum() >= 0.25 * len(rows):
            a_fits.append(a_fit)
            b_fits.append(b_fit)
    if len(a_fits) < 3:
        return None, {"why": "belt edges not found"}
    a_fit = np.median(a_fits, 0)
    b_fit = np.median(b_fits, 0)
    ppm = (b_fit - a_fit + 1) / belt_len
    xs = np.arange(W, dtype=np.float64)
    cand = np.arange(-max_shift, max_shift + 1e-9, 0.1)
    v_rows = []
    for k, r in enumerate(rows.astype(int)):
        mask = np.zeros(W, bool)
        mask[int(np.ceil(a_fit[k])) + 3:int(np.floor(b_fit[k])) - 2] = True
        mask &= ~red[r] & (motion[r] > mot_thr)
        if mask.sum() < 20:
            continue
        tot = 0.0
        for i in range(len(F) - 1):
            p0, p1 = gray[i, r], gray[i + 1, r]
            errs = np.array([np.mean(((np.interp(xs + s, xs, p1) - p0)[mask]) ** 2) for s in cand])
            m = int(np.argmin(errs))
            off = 0.0
            if 0 < m < len(cand) - 1:
                den = errs[m - 1] - 2 * errs[m] + errs[m + 1]
                if den > 1e-9:
                    off = 0.5 * (errs[m - 1] - errs[m + 1]) / den * 0.1
            tot += cand[m] + off
        v_rows.append(tot / (times[-1] - times[0]) / ppm[k])
    if len(v_rows) < 5:
        return None, {"why": "too few usable rows", "rows": len(v_rows)}
    v = float(np.median(v_rows))
    return v, {"v": round(v, 4), "rows": len(v_rows), "frames_ok": len(a_fits),
               "ppm": [round(float(ppm[0]), 1), round(float(ppm[-1]), 1)],
               "spread": round(float(np.std(v_rows)), 4)}


def ransac_line(x, y, tol=1.5, iters=300):
    """Robust y = m x + c; returns (fitted y at x, inlier mask)."""
    rng = np.random.default_rng(0)
    best = None
    valid = np.where(np.isfinite(y))[0]
    span = x[valid].max() - x[valid].min()
    for _ in range(iters):
        i, j = rng.choice(valid, 2, replace=False)
        if abs(x[i] - x[j]) < 0.2 * span:
            continue
        m = (y[j] - y[i]) / (x[j] - x[i])
        inl = np.isfinite(y) & (np.abs(np.nan_to_num(y) - (y[i] + m * (x - x[i]))) <= tol)
        if best is None or inl.sum() > best.sum():
            best = inl
    A = np.stack([x[best], np.ones(best.sum())], 1)
    m, c = np.linalg.lstsq(A, y[best], rcond=None)[0]
    return m * x + c, best


def belt_speed_from_camera(api, belt, bowl, seconds=4.0, frame_dt=0.15):
    """Arms still: grab head frames every frame_dt for `seconds` (sampling the bowl meanwhile) and
    measure the belt surface speed [m/s]."""
    frames, times = [], []
    n = int(round(seconds / frame_dt)) + 1
    for k in range(n):
        frames.append(api.camera_frame("head"))
        times.append(api.time())
        if k < n - 1:
            bowl.sample()
            api.wait(frame_dt / 2)
            bowl.sample()
            api.wait(frame_dt / 2)
    v, info = speed_from_frames(frames, times, float(world_extents(belt)[0]))
    return (None if v is None else abs(v)), info


# ---------------------------------------------------------------- arm helpers
class Arm:
    def __init__(self, api, arm):
        self.api, self.arm = api, arm
        ee, tcp = api.ee_pose(arm), api.tcp_pose(arm)
        # tcp in the ee frame (rigid)
        Ree = quat_mat(ee[3:])
        self.tcp_in_ee = Ree.T @ (np.asarray(tcp[:3]) - np.asarray(ee[:3]))
        # top-down gripper orientation, fingers closing along world y (gripper y axis)
        self.q_down = [0.7071068, 0.0, 0.7071068, 0.0] if arm == "left" else [0.0, -0.7071068, 0.0, 0.7071068]

    def q_yaw(self, yaw):
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return mat_quat(Rz @ quat_mat(self.q_down))

    def ee_for_tcp(self, tcp_xyz, q):
        R = quat_mat(q)
        return [*(np.asarray(tcp_xyz) - R @ self.tcp_in_ee), *q]

    def tcp(self):
        return np.asarray(self.api.tcp_pose(self.arm)[:3])

    def move_tcp(self, xyz, q, **kw):
        return self.api.move_to(self.ee_for_tcp(xyz, q), self.arm, **kw)


def block_yaw(st):
    """Yaw of the block's horizontal face normal closest to world y, wrapped to [-pi/4, pi/4]."""
    R = quat_mat(st.quaternion)
    up = int(np.argmax(np.abs(R[2])))
    best = None
    for j in range(3):
        if j == up:
            continue
        a = np.arctan2(R[1, j], R[0, j]) - np.pi / 2       # rotate so that the normal maps onto y
        a = (a + np.pi / 4) % (np.pi / 2) - np.pi / 4
        if best is None or abs(a) < abs(best):
            best = a
    return float(best)


# ---------------------------------------------------------------- obstacles
def obstacle_boxes(api, exclude, n=5):
    """Every perceived obstacle except `exclude`, centres averaged over n reads."""
    import copy
    reads = [api.get_obstacles(exclude=exclude) for _ in range(n)]
    out = []
    for i, b in enumerate(reads[0]):
        same = [r[i] for r in reads if len(r) > i and r[i].name == b.name]
        bb = copy.copy(b)
        bb.center = np.mean([s.center for s in same], 0)
        out.append(bb)
    return out


def inflated(boxes, m):
    import copy
    out = []
    for b in boxes:
        bb = copy.copy(b)
        bb.extents = np.asarray(b.extents) + 2 * m
        out.append(bb)
    return out


def corridor_top(boxes, p0, p1, half_width):
    """Highest top of any box whose footprint (grown by half_width) meets the segment p0 -> p1 (xy)."""
    top = -np.inf
    p0, p1 = np.asarray(p0[:2], float), np.asarray(p1[:2], float)
    for b in boxes:
        lo = np.asarray(b.center[:2]) - np.asarray(b.extents[:2]) / 2 - half_width
        hi = np.asarray(b.center[:2]) + np.asarray(b.extents[:2]) / 2 + half_width
        for s in np.linspace(0, 1, 50):
            p = p0 + s * (p1 - p0)
            if np.all(p >= lo) and np.all(p <= hi):
                top = max(top, float(b.center[2] + b.extents[2] / 2))
                break
    return top


def straight(api, A, target, q, bowl=None, speed="normal", avoid=None, allow_free=True, _depth=0, max_step=None):
    """Move the tcp to `target` along (close to) the straight line, which the caller has checked is
    clear for the fingers and the payload.
    - Orientation-holding plans with the known obstacles first (the arm's links are in the planner's
      model; a plan that ignores obstacles can drive the forearm into one).
    - For a vertical move, orientation-holding plans with the table only (a start pose inside an
      inflated box makes every obstacle-aware plan fail).
    - Then split the move in half and try again (shorter constrained plans succeed more often).
    - A free plan (it can swing the payload anywhere) only if allow_free.
    Fails if the tcp does not arrive."""
    target = np.asarray(target, float)
    cur = A.tcp()
    if max_step is not None and np.linalg.norm(target - cur) > max_step:
        # hops: a long constrained plan from an awkward arm configuration can wander (seen sweeping a
        # held payload low across the work surface); short hops stay near the line
        n = int(np.ceil(np.linalg.norm(target - cur) / max_step))
        for i in range(1, n + 1):
            if not straight(api, A, cur + (target - cur) * i / n, q, bowl, speed, avoid, allow_free, _depth):
                return False
        return True
    vertical = np.linalg.norm(target[:2] - cur[:2]) < 0.01
    tries = [dict(hold_orientation=True, avoid=avoid)] * 2
    if avoid:      # the start may sit inside the safety inflation: retry with the bare boxes
        tries += [dict(hold_orientation=True, avoid=inflated(avoid, -0.01))]
    if vertical:
        tries += [dict(hold_orientation=True)] * 2
    ok = any(A.move_tcp(target, q, speed=speed, **kw) for kw in tries)
    if bowl is not None:
        bowl.sample()
    if not ok and _depth < 2 and np.linalg.norm(target - cur) > 0.03:
        mid = (cur + target) / 2
        ok = (straight(api, A, mid, q, bowl, speed, avoid, False, _depth + 1)
              and straight(api, A, target, q, bowl, speed, avoid, False, _depth + 1))
    if not ok and allow_free:
        ok = any(A.move_tcp(target, q, speed=speed, avoid=avoid) for _ in range(2))
    if not ok:
        note(api, why="straight: no plan", target=np.round(target, 3).tolist(), tcp=np.round(A.tcp(), 3).tolist())
        return False
    if np.linalg.norm(A.tcp() - target) > 0.025:      # blocked or clipped on the way
        note(api, why="straight: target not reached", target=np.round(target, 3).tolist(),
             tcp=np.round(A.tcp(), 3).tolist())
        return False
    return True


def transit(api, A, target, q, obstacles, finger_reach, bowl=None):
    """Up to a height where the fingertips clear every obstacle near the straight path, across, down."""
    cur = A.tcp()
    clear = corridor_top(obstacles, cur, target, half_width=0.08) + finger_reach + 0.04
    safe = max(cur[2], target[2], clear)
    av = inflated(obstacles, 0.01)
    legs = [[cur[0], cur[1], safe], [target[0], target[1], safe], list(target)]
    prev = cur
    for leg in legs:                                   # skip legs that do not go anywhere
        if np.linalg.norm(np.asarray(leg) - prev) < 0.01:
            continue
        if not straight(api, A, leg, q, bowl, avoid=av):
            return False
        prev = np.asarray(leg)
    return True


# ---------------------------------------------------------------- main
def run(api):
    bowl = BowlTracker(api)
    bowl.sample()
    belt = avg_detect(api, "belt", 20)
    line = avg_detect(api, "place line", 20)
    blocker = avg_detect(api, "blocker", 20)
    block = avg_detect(api, "block", 8)
    bext = world_extents(belt)
    belt_c = np.asarray(belt.bbox_center)
    belt_top = float(belt_c[2] + bext[2] / 2)
    table_top = 0.74
    sup = avg_detect(api, "belt support", 20)
    if sup is not None:
        table_top = float(sup.bbox_center[2] - world_extents(sup)[2] / 2)
    # everything else in the scene is an obstacle for the arm (the bowl moves on its own lane; the
    # block is the payload)
    obstacles = obstacle_boxes(api, exclude=("block", "bowl"))
    note(api, obstacles=[(b.name, np.round(b.center, 3).tolist(), np.round(b.extents, 3).tolist()) for b in obstacles])

    flow = 1.0 if belt_c[0] > block.position[0] else -1.0
    arm_name = "left" if block.position[0] < 0 else "right"
    A = Arm(api, arm_name)
    end_edge = belt_c[0] + flow * bext[0] / 2

    # lane: the side of the belt away from the blocker
    if blocker is not None:
        side = -1.0 if blocker.bbox_center[1] > belt_c[1] else 1.0
    else:
        side = 1.0
    lane_y = float(belt_c[1] + side * (bext[1] / 2 - EDGE_MARGIN))
    rel_x = float(line.position[0] - flow * LINE_MARGIN)
    note(api, flow=flow, arm=arm_name, lane_y=round(lane_y, 3), rel_x=round(rel_x, 3), belt_top=round(belt_top, 4),
         table_top=round(table_top, 4))

    # belt speed from the camera while the arms are still
    v_belt, info = belt_speed_from_camera(api, belt, bowl)
    note(api, belt_speed=info)
    if v_belt is None or not (0.005 < v_belt < 1.0):
        v_belt = 0.06
    # ------------------------------------------------ grasp
    blk_ext = world_extents(block)
    h = float(blk_ext[2])
    q_rel = A.q_down
    av = inflated(obstacles, 0.01)

    def block_in_hand(grip_dz):
        """The block is where it would be if it were still between the fingers."""
        st = avg_detect(api, "block", 5)
        if st is None:
            return False
        tcp = A.tcp()
        exp_z = tcp[2] - grip_dz
        return bool(np.linalg.norm(np.asarray(st.bbox_center[:2]) - tcp[:2]) < 0.04
                    and abs(float(st.bbox_center[2]) - exp_z) < 0.03)

    placed = False
    for attempt in range(4):
        # ---------------- pick
        block = avg_detect(api, "block", 8)
        yaw = block_yaw(block)
        q = A.q_yaw(yaw)
        bc = np.asarray(block.bbox_center)
        rest_z = float(bc[2])                     # current centre height of the (resting) block
        grasp_z = max(rest_z, table_top + FINGER_REACH + TABLE_CLEAR)
        pre = [bc[0], bc[1], rest_z + h / 2 + 0.10]
        # transit above everything between the gripper and the block (open fingers are not in the
        # planner's model, so obstacles are cleared explicitly by height), then straight down
        api.open_gripper(arm_name)
        if not transit(api, A, pre, q, obstacles, finger_reach=FINGER_REACH, bowl=bowl):
            note(api, why="pre-grasp transit failed", attempt=attempt)
            continue
        A.move_tcp([bc[0], bc[1], grasp_z], q, hold_orientation=True, speed="slow")
        reached = A.tcp()
        bowl.sample()
        api.close_gripper(arm_name)
        held = api.gripper_holding(arm_name) and api.gripper_opening(arm_name) < 0.85
        # the fingers must have closed on the block where it was, not on something else
        held = held and np.linalg.norm(reached[:2] - bc[:2]) < 0.03 and abs(reached[2] - grasp_z) < 0.03
        grip_dz = float(reached[2] - rest_z)      # tcp height above the block centre
        note(api, grasp_attempt=attempt, held=held, tcp=np.round(reached, 3).tolist(), target_z=round(grasp_z, 3),
             opening=round(api.gripper_opening(arm_name), 3))
        bowl.sample()
        if not held:
            api.open_gripper(arm_name)
            straight(api, A, pre, q, bowl, avoid=av)
            continue
        # ---------------- lift (slow: a fingertip grasp slips under jerky motion), verify
        st = avg_detect(api, "block", 8)
        off_xy = np.clip(np.asarray(st.bbox_center[:2]) - A.tcp()[:2], -0.015, 0.015)
        above_xy = [rel_x - off_xy[0], lane_y - off_xy[1]]
        # carry with the block's bottom above the belt and every obstacle near the straight path
        corridor = corridor_top(obstacles, A.tcp(), above_xy, half_width=0.06)
        carry_bottom = max(belt_top + CARRY_CLEAR, corridor + 0.03)
        carry_z = carry_bottom + h / 2 + grip_dz
        note(api, carry_bottom=round(carry_bottom, 3), corridor_top=round(corridor, 3), off_xy=np.round(off_xy, 4).tolist())
        straight(api, A, [*A.tcp()[:2], carry_z], q, bowl, avoid=av, allow_free=False, max_step=0.07)
        if not block_in_hand(grip_dz):
            note(api, why="block lost during lift", attempt=attempt)
            continue
        # ---------------- carry, verify
        above = [above_xy[0], above_xy[1], carry_z]
        ok = straight(api, A, above, q_rel, bowl, avoid=av, allow_free=False, max_step=0.07) or \
            straight(api, A, above, q_rel, bowl, avoid=av, allow_free=False, max_step=0.07)
        note(api, carry_ok=ok, tcp=np.round(A.tcp(), 3).tolist())
        if not block_in_hand(grip_dz):
            note(api, why="block lost during carry", attempt=attempt)
            continue
        if not ok:
            api.open_gripper(arm_name)       # put it down where it is and start over
            continue
        placed = True
        break
    if not placed:
        return
    set_z = belt_top + SET_CLEAR + h / 2 + grip_dz
    straight(api, A, [above[0], above[1], set_z], q_rel, bowl, speed="slow", avoid=av, allow_free=False, max_step=0.07)
    tcp_set = A.tcp()
    note(api, set_tcp=np.round(tcp_set, 4).tolist(), set_z=round(set_z, 4))
    # ------------------------------------------------ timing
    bowl.hold(2.0)
    x_contact = float(tcp_set[0] + off_xy[0])
    drop_x = end_edge + flow * (blk_ext[0] / 2)
    travel = abs(drop_x - x_contact) / v_belt
    lag = RELEASE_LAG_S + BELT_ARM_S + travel + FALL_S
    for it in range(2):
        fitinfo = bowl.fit_model()
        now = api.time()
        ts = np.arange(now + 0.3 + lag, now + 0.3 + lag + 6.0, 0.004)
        inside = np.abs(bowl.predict(ts) - lane_y) <= CATCH_HALF_W
        # windows: contiguous runs of `inside`; aim at the centre of the first one
        idx = np.where(inside)[0]
        if len(idx) == 0:
            t_hit = ts[np.argmin(np.abs(bowl.predict(ts) - lane_y))]
        else:
            runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
            run0 = runs[0] if runs[0][0] > 0 or len(runs) == 1 else runs[1]
            t_hit = 0.5 * (ts[run0[0]] + ts[run0[-1]])
        t_cmd = t_hit - lag
        note(api, fit=fitinfo, v_belt=round(v_belt, 4), travel=round(travel, 2), t_cmd=round(t_cmd, 2),
             t_hit=round(t_hit, 2), it=it)
        if it == 0 and t_cmd - api.time() > 1.2:
            bowl.hold(t_cmd - api.time() - 1.0)
    while api.time() < t_cmd - 0.06:
        bowl.sample()
        api.wait(min(0.05, t_cmd - api.time()))
    api.open_gripper(arm_name)
    note(api, opened_at=round(api.time(), 2))
    api.move_by(arm_name, dz=0.10, hold_orientation=True)
    # let the block ride; track it for diagnostics
    t_end = t_hit + 1.5
    while api.time() < t_end:
        s = api.detect("block")
        note(api, block=np.round(s.position, 3).tolist())
        api.wait(0.5)
