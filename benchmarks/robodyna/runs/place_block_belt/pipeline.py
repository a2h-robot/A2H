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


def _dark_run(gray_row, darkish_row, dark_thr, max_gap=25):
    """Longest dark run of one image row -> (a, b) or None."""
    W = len(gray_row)
    if darkish_row.sum() < 40:
        return None
    d = np.diff(np.concatenate([[0], darkish_row.astype(int), [0]]))
    starts, ends = list(np.where(d == 1)[0]), list(np.where(d == -1)[0] - 1)
    # a narrow bright thing lying across the belt (a bar, a bridge) splits the dark run: join runs
    # separated by short gaps, so the run still spans the belt from end to end
    i = 0
    while i < len(starts) - 1:
        if starts[i + 1] - ends[i] <= max_gap:
            ends[i] = ends[i + 1]
            del starts[i + 1], ends[i + 1]
        else:
            i += 1
    starts, ends = np.asarray(starts), np.asarray(ends)
    j = int(np.argmax(ends - starts))
    a, b = starts[j], ends[j]
    if b - a < 40 or a <= 2 or b >= W - 3:
        return None
    ins = np.where(gray_row[a:b + 1] < dark_thr)[0]
    return a + ins.min(), a + ins.max()              # undo the min-filter dilation


def _runs(gray_row, darkish_row, dark_thr, max_gap=25, min_len=15):
    """All dark runs (gap-joined) of one image row, as (a, b) column pairs."""
    d = np.diff(np.concatenate([[0], darkish_row.astype(int), [0]]))
    starts, ends = list(np.where(d == 1)[0]), list(np.where(d == -1)[0] - 1)
    i = 0
    while i < len(starts) - 1:
        if starts[i + 1] - ends[i] <= max_gap:
            ends[i] = ends[i + 1]
            del starts[i + 1], ends[i + 1]
        else:
            i += 1
    out = []
    for a, b in zip(starts, ends):
        if b - a < min_len:
            continue
        ins = np.where(gray_row[a:b + 1] < dark_thr)[0]
        if len(ins):
            out.append((a + ins.min(), a + ins.max()))
    return out


def _pattern_direction(gray, motion, mot_thr, rows):
    """+1 if the moving pattern travels towards higher image columns, -1 otherwise (majority over rows)."""
    W = gray.shape[2]
    xs = np.arange(W, dtype=np.float64)
    votes = 0
    for r in rows.astype(int)[::4]:
        mask = _sliding_min((motion[r] > mot_thr)[None].astype(float), 13)[0] > 0.5
        if mask.sum() < 20:
            continue
        p0, p1 = gray[0, r], gray[1, r]
        errs = [np.mean(((np.interp(xs + s, xs, p1) - p0)[mask]) ** 2) for s in np.arange(-6, 6.01, 0.5)]
        s = np.arange(-6, 6.01, 0.5)[int(np.argmin(errs))]
        votes += int(np.sign(s))
    return 1 if votes >= 0 else -1


def _line_scale(F, gray, rows, red, dark_thr, ref, motion, mot_thr, flow_sign_px):
    """Pixels per metre per row from two known points on the belt plane: the red place line and the
    belt's load-end edge. The load side in the image is where the pattern comes from (opposite to its
    image motion), and the load edge is the end of the dark run that contains the line on that side,
    fitted per frame (RANSAC over rows) and median over frames. Works when the far part of the belt is
    hidden. ref = (line_x, load_x)."""
    line_x, load_x = ref
    W = gray.shape[2]
    rr = rows.astype(int)
    C_line = []
    for r in rr:
        cols = np.where(red[r])[0]
        c_line = np.nan
        if len(cols):
            d = np.diff(np.concatenate([[-9], cols, [W + 9]]))
            starts = cols[np.where(d[:-1] > 1)[0]]
            ends = cols[np.where(d[1:] > 1)[0]]
            thin = [(s, e) for s, e in zip(starts, ends) if e - s <= 4]   # the line is a thin strip
            if thin:
                c_line = float(np.mean(thin[0]))
        C_line.append(c_line)
    C_line = np.asarray(C_line)
    if np.isfinite(C_line).sum() < 8:
        return None
    line_fit, _ = ransac_line(rows, C_line, tol=1.0)
    load_left = flow_sign_px > 0               # the pattern moves away from the load end
    fits = []
    for i in range(len(F)):
        dk = _sliding_min(gray[i, rr], 11) < dark_thr
        C = []
        for k, r in enumerate(rr):
            hit = [ab for ab in _runs(gray[i, r], dk[k], dark_thr) if ab[0] - 2 <= line_fit[k] <= ab[1] + 2]
            C.append(np.nan if not hit else (hit[0][0] if load_left else hit[0][1]))
        C = np.asarray(C, float)
        if np.isfinite(C).sum() < 8:
            continue
        fit, inl = ransac_line(rows, C)
        if inl.sum() >= 0.25 * len(rows):
            fits.append(fit)
    if len(fits) < 3:
        return None
    load_fit = np.median(fits, 0)
    return np.abs(line_fit - load_fit) / abs(line_x - load_x)


def speed_from_frames(frames, times, belt_len, dark_thr=110.0, mot_thr=12.0, max_shift=6.0, ref=None):
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
    # Per colour channel (white balance against the same background), so a tinted light does not make
    # everything look 'red' to the colour tests.
    bright = np.percentile(F[0].reshape(-1, 3), 90, axis=0)
    F = F * (230.0 / np.maximum(bright, 1.0))
    gray = F.mean(-1)
    red = (F[0, :, :, 0] - np.maximum(F[0, :, :, 1], F[0, :, :, 2])) > 60
    motion = np.abs(np.diff(gray, axis=0)).mean(0)
    H, W = gray.shape[1:]
    top_rows = []
    for r in range(H):
        runs = [ab for ab in _runs(gray[0, r], _sliding_min(gray[0, r:r + 1], 11)[0] < dark_thr, dark_thr)
                if (motion[r, ab[0]:ab[1] + 1] > mot_thr).mean() >= 0.5]
        if runs and max(ab[1] for ab in runs) - min(ab[0] for ab in runs) >= 40:
            top_rows.append(r)
    if len(top_rows) < 8:
        return None, {"why": "belt top not found", "rows": len(top_rows)}
    rows = np.asarray(top_rows, float)
    a_fits, b_fits = [], []
    for i in range(len(F)):
        dk = _sliding_min(gray[i, top_rows], 11) < dark_thr
        A, B = [], []
        for k, r in enumerate(top_rows):
            # the belt top is every dark run carrying the moving pattern; a cover or bar lying across
            # the belt splits it into pieces, so take the extent of all moving pieces together
            runs = [ab for ab in _runs(gray[i, r], dk[k], dark_thr)
                    if (motion[r, ab[0]:ab[1] + 1] > mot_thr).mean() >= 0.5]
            A.append(np.nan if not runs else min(ab[0] for ab in runs))
            B.append(np.nan if not runs else max(ab[1] for ab in runs))
        A, B = np.asarray(A, float), np.asarray(B, float)
        if np.isfinite(A).sum() < 8:
            continue
        a_fit, a_in = ransac_line(rows, A)
        b_fit, b_in = ransac_line(rows, B)
        if a_in.sum() >= 0.25 * len(rows) and b_in.sum() >= 0.25 * len(rows):
            a_fits.append(a_fit)
            b_fits.append(b_fit)
    ppm_full = None
    if len(a_fits) >= 3:
        a_fit = np.median(a_fits, 0)
        b_fit = np.median(b_fits, 0)
        ppm_full = (b_fit - a_fit + 1) / belt_len
    # second, independent scale from two known points (place line, load edge): it does not need the far
    # end of the belt to be visible. Use the full-length scale only when the two agree.
    ppm_line = (_line_scale(F, gray, rows, red, dark_thr, ref, motion, mot_thr,
                            _pattern_direction(gray, motion, mot_thr, rows)) if ref is not None else None)
    source = "full"
    if ppm_line is not None and ppm_full is None:
        ppm, source = ppm_line, "line"
    elif ppm_full is not None:
        ppm = ppm_full
    else:
        return None, {"why": "no pixel scale"}
    bounds = []
    for k, r in enumerate(rows.astype(int)):
        if source == "full":
            bounds.append((a_fit[k], b_fit[k]))
        else:     # the dark run that holds most of the moving pattern (the visible belt part)
            runs = _runs(gray[0, r], _sliding_min(gray[0, r:r + 1], 11)[0] < dark_thr, dark_thr)
            if not runs:
                bounds.append((0, -1))
                continue
            best = max(runs, key=lambda ab: (motion[r, ab[0]:ab[1] + 1] > mot_thr).sum())
            bounds.append(best)
    xs = np.arange(W, dtype=np.float64)
    cand = np.arange(-max_shift, max_shift + 1e-9, 0.1)
    v_rows = []
    for k, r in enumerate(rows.astype(int)):
        mask = np.zeros(W, bool)
        mask[int(np.ceil(bounds[k][0])) + 3:int(np.floor(bounds[k][1])) - 2] = True
        mask &= ~red[r] & (motion[r] > mot_thr)
        # keep only pixels whose whole shift window is moving surface: a still occluder next to the
        # window pulls the best shift towards zero
        k_er = 2 * int(np.ceil(max_shift)) + 1
        mask = _sliding_min(mask[None].astype(float), k_er)[0] > 0.5
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
    return v, {"v": round(v, 4), "rows": len(v_rows), "frames_ok": len(a_fits), "scale": source,
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


def belt_speed_from_camera(api, belt, bowl, seconds=4.0, frame_dt=0.15, ref=None):
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
    v, info = speed_from_frames(frames, times, float(world_extents(belt)[0]), ref=ref)
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


# ---------------------------------------------------------------- main
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


def column_free(boxes, xy, half_xy, z_lo, z_hi):
    """No box intersects the vertical column [xy +- half_xy] x [z_lo, z_hi]."""
    for b in boxes:
        lo, hi = np.asarray(b.center) - np.asarray(b.extents) / 2, np.asarray(b.center) + np.asarray(b.extents) / 2
        if (lo[0] < xy[0] + half_xy[0] and hi[0] > xy[0] - half_xy[0] and lo[1] < xy[1] + half_xy[1]
                and hi[1] > xy[1] - half_xy[1] and lo[2] < z_hi and hi[2] > z_lo):
            return False
    return True


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
    # release point: the allowed zone runs from the load edge to the place line; take the spot nearest
    # the line (shortest ride) whose vertical column, wide enough for the open gripper, is free
    obstacles = obstacle_boxes(api, exclude=("block", "bowl"))
    load_edge = belt_c[0] - flow * bext[0] / 2
    rel_x = None
    for x in np.arange(line.position[0] - flow * LINE_MARGIN, load_edge + flow * 0.025, -flow * 0.005):
        if column_free(obstacles, [x, lane_y], [0.05, 0.07], belt_top, belt_top + 0.35):
            rel_x = float(x)
            break
    if rel_x is None:
        rel_x = float(line.position[0] - flow * LINE_MARGIN)
    note(api, obstacles=[(b.name, np.round(b.center, 3).tolist()) for b in obstacles])
    note(api, flow=flow, arm=arm_name, lane_y=round(lane_y, 3), rel_x=round(rel_x, 3), belt_top=round(belt_top, 4),
         table_top=round(table_top, 4))

    # belt speed from the camera while the arms are still
    v_belt, info = belt_speed_from_camera(api, belt, bowl, ref=(float(line.position[0]), float(load_edge)))
    note(api, belt_speed=info)
    if v_belt is None or not (0.005 < v_belt < 1.0):
        v_belt = 0.06
    # ------------------------------------------------ grasp
    blk_ext = world_extents(block)
    h = float(blk_ext[2])
    held = False
    for attempt in range(3):
        block = avg_detect(api, "block", 8)
        yaw = block_yaw(block)
        q = A.q_yaw(yaw)
        bc = np.asarray(block.bbox_center)
        grasp_z = max(float(bc[2]), table_top + FINGER_REACH + TABLE_CLEAR)
        pre = [bc[0], bc[1], bc[2] + h / 2 + 0.10]
        if not A.move_tcp(pre, q):
            note(api, why="pre-grasp plan failed", attempt=attempt)
            bowl.sample()
            continue
        bowl.sample()
        api.open_gripper(arm_name)
        A.move_tcp([bc[0], bc[1], grasp_z], q, hold_orientation=True, speed="slow")
        reached = A.tcp()
        bowl.sample()
        api.close_gripper(arm_name)
        held = api.gripper_holding(arm_name)
        note(api, grasp_attempt=attempt, held=held, tcp=np.round(reached, 3).tolist(), target_z=round(grasp_z, 3))
        bowl.sample()
        if held:
            break
        api.open_gripper(arm_name)
        A.move_tcp(pre, q)
    if not held:
        return
    # block centre relative to the tcp, from where it was resting
    grip_dz = float(reached[2] - (table_top + h / 2))
    # ------------------------------------------------ lift and carry
    carry_z = belt_top + CARRY_CLEAR + h / 2 + grip_dz
    tcp_now = A.tcp()
    A.move_tcp([tcp_now[0], tcp_now[1], carry_z], q, hold_orientation=True)
    bowl.sample()
    # refine the in-hand offset in xy from the perceived block
    st = avg_detect(api, "block", 8)
    off_xy = np.asarray(st.bbox_center[:2]) - A.tcp()[:2]
    off_xy = np.clip(off_xy, -0.015, 0.015)
    q_rel = A.q_down
    above = [rel_x - off_xy[0], lane_y - off_xy[1], carry_z]
    avoid = [belt, sup] + inflated(obstacles, 0.01)      # every perceived obstacle (incl. the blocker)
    ok = A.move_tcp(above, q_rel, avoid=avoid)
    bowl.sample()
    note(api, carry_ok=ok, off_xy=np.round(off_xy, 4).tolist())
    if not ok:
        ok = A.move_tcp(above, q_rel)
        bowl.sample()
    set_z = belt_top + SET_CLEAR + h / 2 + grip_dz
    A.move_tcp([above[0], above[1], set_z], q_rel, hold_orientation=True, speed="slow")
    bowl.sample()
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
