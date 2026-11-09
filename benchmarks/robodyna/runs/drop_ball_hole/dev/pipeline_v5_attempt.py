"""drop_ball_hole: drop the ball through the real (ball-sized) hole of the rotating sorter.

Strategy
1. Perceive the ball, the sorter and both holes. The real hole is the one wider than the ball
   (the decoy is cut smaller than the ball). The arm on the ball's side works.
2. Phase model of the platform: yaw(t) = a + w t, fitted by least squares on averaged detections
   (sorter and hole orientations) sampled densely at the start and again whenever the arm is idle.
   The real hole's polar offset in the platform frame is averaged over the same samples.
3. Grasp the ball top-down, lift it, carry it over the real hole's orbit on the arm's side, and
   lower it until its bottom is a little above the platform top.
4. Re-measure the ball, solve for the next time the hole centre passes under the ball and command
   the release LEAD seconds earlier (gripper opening + fall). Then lift the arm clear.
Only the API in docs/api_spec.md (+ documented extensions) is used.
"""
import numpy as np

N_AVG = 24             # detections averaged per sample (1 cm / 3 deg noise each)
R_BALL_DEFAULT = 0.025
TRANSPORT_CLEAR = 0.08  # ball bottom above the platform top while carrying
RELEASE_CLEAR = 0.055   # ball bottom above the platform top at release: freed in the air, and the open
                        # fingertips (~ball centre height) stay above a ball that sinks into the moving hole
UPSTREAM = 0.010        # m: aim the landing this far before the hole centre along its travel
RADIAL_INSET = 0.004    # m: land the ball this far inside the hole's orbit; the spinning platform throws a
                        # ball that catches the rim outwards (and the ball drifts ~4 mm outwards on release)
RELEASE_DRIFT = 0.0     # m: ball shift along the gripper -z axis while the fingers open (measured -6..+4 mm: not systematic, so not compensated)
T_FREE = 0.465          # s from open_gripper until the ball leaves the fingers (it slides ~2 cm down first)
LEAD = T_FREE + float(np.sqrt(2 * max(0.0, RELEASE_CLEAR - 0.02) / 9.81))   # + free fall to the platform
MOVING = 0.01          # m/s: slower than this the ball counts as still
T_AHEAD = 2.6          # s: intercept a rolling ball this far ahead (pre-pose + descent take ~2 s)
CLOSE_LEAD = 0.4       # s: from close_gripper until the fingers meet the ball
CENTER_TOL = 0.015     # m: a still ball must be this close to the finger centre before closing
LATERAL_TOL = 0.03     # m: (open gap 11 cm - ball 5 cm) / 2
OBST_CLEAR = 0.035      # m: ball bottom above an obstacle's top while carrying over it
YAW_TRIES = (0.0, 0.5, -0.5, 1.0, -1.0, 1.57, -1.57)
GRIPPER_RADIUS = 0.045  # m: gripper body around the held ball (xy; the fingers stick out further only along one axis)
GRIPPER_COLUMN = 0.35   # m: ball + gripper + wrist height above the platform at release
FINGER_HALF = 0.06      # m: open fingertip offset from the gripper centre along the closing axis
AVOID = []             # perceived obstacle boxes, passed to every motion
RAW_OBST = []           # the same boxes, for carry-height decisions
DEBUG = {}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    w, x, y, z = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def circ_mean(a):
    a = np.asarray(a)
    return float(np.arctan2(np.sin(a).mean(), np.cos(a).mean()))


def safe_detect(api, name):
    try:
        return api.detect(name)
    except Exception:          # perception can fail on a frame; treat it as "not seen"
        return None


def mean_state(api, name, n=N_AVG):
    ss = [s for s in (safe_detect(api, name) for _ in range(n)) if s is not None]
    if not ss:
        return None
    return {"p": np.mean([s.position for s in ss], 0), "yaw": circ_mean([yaw_of(s.quaternion) for s in ss]),
            "ext": np.mean([s.bbox_extents for s in ss], 0), "c": np.mean([s.bbox_center for s in ss], 0)}


class Platform:
    """Tracks yaw(t) = a + w t of the sorter and the real hole's offset in its frame."""

    def __init__(self, api, target, decoy):
        self.api, self.target, self.decoy = api, target, decoy
        self.t, self.yaw, self.cxy = [], [], []
        self.loc = []       # (r, phi_local) of the target hole per sample
        self.w = None
        self.a = None

    def sample(self):
        api = self.api
        t = api.time()
        s = mean_state(api, "sorter")
        h = mean_state(api, self.target)
        d = mean_state(api, self.decoy) if self.decoy else None
        if h is None:
            return
        yaws = [x["yaw"] for x in (s, h, d) if x is not None]
        y = circ_mean(yaws)
        if s is not None:
            self.cxy.append(s["p"][:2])
        elif not self.cxy and d is not None:   # holes sit at opposite corners: midpoint ~ centre
            self.cxy.append((h["p"][:2] + d["p"][:2]) / 2)
        elif not self.cxy:
            return
        c = np.mean(self.cxy, 0)
        if self.w is not None:     # unwrap against the current model
            pred = self.a + self.w * t
            y = pred + wrap(y - pred)
        elif self.yaw:
            y = self.yaw[-1] + wrap(y - self.yaw[-1])
        self.t.append(t)
        self.yaw.append(y)
        d = h["p"][:2] - c
        self.loc.append((float(np.linalg.norm(d)), float(np.arctan2(d[1], d[0]) - y)))
        if len(self.t) >= 3:
            A = np.stack([np.ones(len(self.t)), np.asarray(self.t)], 1)
            self.a, self.w = map(float, np.linalg.lstsq(A, np.asarray(self.yaw), rcond=None)[0])

    @property
    def center(self):
        return np.mean(self.cxy, 0)

    @property
    def radius(self):
        return float(np.mean([r for r, _ in self.loc]))

    @property
    def phi_local(self):
        return circ_mean([p for _, p in self.loc])

    def hole_xy(self, t):
        ang = self.a + self.w * t + self.phi_local
        return self.center + self.radius * np.array([np.cos(ang), np.sin(ang)])

    def next_pass(self, xy, t_min):
        """Earliest t >= t_min at which the hole's polar angle equals that of xy."""
        d = np.asarray(xy[:2]) - self.center
        phi = np.arctan2(d[1], d[0])
        cur = self.a + self.phi_local + self.w * t_min
        gap = (phi - cur) % (2 * np.pi) if self.w > 0 else (cur - phi) % (2 * np.pi)
        return float(t_min + gap / abs(self.w))


def quat_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def ball_velocity(samples, window=0.5):
    """Least-squares xy velocity over the most recent `window` seconds of (t, xyz) samples."""
    t_last = samples[-1][0]
    s = [(t, p) for t, p in samples if t >= t_last - window]
    if len(s) < 2 or s[-1][0] - s[0][0] < 0.15:
        return np.zeros(2)
    T = np.array([t for t, _ in s])
    P = np.array([p[:2] for _, p in s])
    A = np.stack([np.ones_like(T), T - T.mean()], 1)
    return np.linalg.lstsq(A, P, rcond=None)[0][1]


def observe_ball(api, n=3, dt=0.1):
    out = []
    for i in range(n):
        out.append((api.time(), mean_state(api, "ball")["p"]))
        if i < n - 1:
            api.wait(dt)
    return out




def footprint_dist(box, p):
    lo, hi = box.center[:2] - box.extents[:2] / 2, box.center[:2] + box.extents[:2] / 2
    return float(np.linalg.norm(p - np.clip(p, lo, hi)))


def finger_clearance(center_xy, axis_xy):
    """xy distance from the two open fingertips (centre +- FINGER_HALF along the closing axis) to the
    nearest perceived obstacle footprint."""
    n = np.linalg.norm(axis_xy)
    axis_xy = axis_xy / n if n > 1e-6 else axis_xy
    pts = [center_xy + FINGER_HALF * axis_xy, center_xy - FINGER_HALF * axis_xy]
    return min([footprint_dist(o, p) for o in RAW_OBST for p in pts] or [1.0])


def yawed(c, about, yaw):
    """Grasp candidate c turned by yaw about the vertical line through `about` (pose and pre-pose)."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    qz = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    out = dict(c)
    for k in ("pose", "pre_pose"):
        p = np.asarray(c[k][:3])
        p = np.asarray(about) + Rz @ (p - np.asarray(about))
        out[k] = [*map(float, p), *map(float, qmul(qz, c[k][3:]))]
    return out


def acquire_ball(api, arm, plat):
    """One grasp attempt on a ball that may be rolling.
    Estimate its velocity; if it moves, pre-position the open gripper on its path T_AHEAD s ahead with
    the closing axis across the path, and close when it rolls between the fingers. Never close blind:
    re-measure the ball after the descent and abort (lift, retry) if it is not where expected."""
    samples = observe_ball(api, n=6, dt=0.1)
    p, v = samples[-1][1], ball_velocity(samples, 0.6)
    speed = float(np.linalg.norm(v))
    moving = speed > MOVING
    vhat = v / speed if moving else None
    aim = p.copy()
    if moving:
        aim[:2] = p[:2] + v * T_AHEAD
    st = api.detect("ball")
    st.position, st.quaternion, st.bbox_center = aim, np.array([1.0, 0, 0, 0]), aim.copy()
    cands = api.grasp_candidates("ball", arm, pre_dis=0.08, state=st, strategy="top", avoid=AVOID)
    cands = [c for c in cands if quat_mat(c["pose"][3:])[2, 0] < -0.9]    # approach axis pointing down
    if not cands:
        return False
    if RAW_OBST and not moving:    # a sphere can be grasped at any yaw: add yaws between the four candidates
        cands = cands + [yawed(c, aim, y) for c in cands[:1] for y in np.deg2rad(np.arange(15, 180, 15))]
    if RAW_OBST:   # open fingertips as far as possible from neighbouring objects (planner ignores the gripper)
        cands.sort(key=lambda c: -finger_clearance(aim[:2], quat_mat(c["pose"][3:])[:2, 1]))
        DEBUG.setdefault("finger_clearance", []).append(
            [round(finger_clearance(aim[:2], quat_mat(c["pose"][3:])[:2, 1]), 3) for c in cands])
    if moving:     # closing axis (gripper y) across the rolling direction
        cands.sort(key=lambda c: abs(float(quat_mat(c["pose"][3:])[:2, 1] @ vhat)))
        cands = [c for c in cands if abs(float(quat_mat(c["pose"][3:])[:2, 1] @ vhat)) < 0.5] or cands[:1]
    if api.gripper_opening(arm) < 0.8:
        api.open_gripper(arm)
    for c in cands:               # first candidate whose approach and straight descent both plan
        if api.move_to(c["pre_pose"], arm, avoid=AVOID):
            if api.move_to(c["pose"], arm, hold_orientation=True, avoid=AVOID):
                break
            DEBUG.setdefault("aborts", []).append(dict(t=round(api.time(), 2), why="descent did not plan"))
    else:
        api.move_by(arm, dz=0.08, avoid=AVOID)
        return False
    plat.sample()
    t_end = api.time() + 6.0
    fresh = 0
    while api.time() < t_end:
        samples.append((api.time(), mean_state(api, "ball")["p"]))
        fresh += 1
        tcp = np.asarray(api.tcp_pose(arm)[:2])
        rel = samples[-1][1][:2] - tcp
        v_now = ball_velocity(samples, 1.2)
        sp = float(np.linalg.norm(v_now))
        close_axis = quat_mat(c["pose"][3:])[:2, 1]
        if sp < MOVING:
            if np.linalg.norm(rel) < CENTER_TOL:
                break                       # still ball between the fingers
            DEBUG.setdefault("aborts", []).append(dict(t=round(api.time(), 2), why="still, off-centre",
                                                       rel=np.round(rel, 4).tolist()))
            if fresh >= 4:
                api.move_by(arm, dz=0.08, avoid=AVOID)   # it is not there and not coming: retry from above
                return False
        else:
            u = v_now / sp
            along = -float(rel @ u)         # > 0 while the ball is still before the fingers
            lateral = float(np.linalg.norm(rel + along * u))
            if lateral > LATERAL_TOL or along < -0.02 or abs(float(close_axis @ u)) > 0.5:
                api.move_by(arm, dz=0.08, avoid=AVOID)
                return False
            if along <= sp * CLOSE_LEAD:
                break
        api.wait(0.04)
    else:
        api.move_by(arm, dz=0.08, avoid=AVOID)
        return False
    DEBUG.setdefault("grasps", []).append(dict(t=round(api.time(), 2), moving=moving, speed=round(sp, 3),
                                               rel=np.round(rel, 4).tolist()))
    api.close_gripper(arm)
    if api.gripper_holding(arm):
        return True
    api.open_gripper(arm)
    api.move_by(arm, dz=0.08, avoid=AVOID)
    return False


def perceived_obstacles(api, n=10):
    """Boxes around every object other than the ball, centres averaged over n noisy calls."""
    acc = {}
    for _ in range(n):
        for b in api.get_obstacles(exclude=("ball",)):
            acc.setdefault(b.name, []).append(b)
    out = []
    for bs in acc.values():
        b = bs[0]
        b.center = np.mean([x.center for x in bs], 0)
        out.append(b)
    return out






def column_clear(xy, z_lo, z_hi, radius):
    """No perceived obstacle within `radius` (xy) of the vertical column xy between z_lo and z_hi."""
    for o in RAW_OBST:
        lo, hi = o.center - o.extents / 2, o.center + o.extents / 2
        if hi[2] < z_lo or lo[2] > z_hi:
            continue
        if float(np.linalg.norm(np.asarray(xy) - np.clip(xy, lo[:2], hi[:2]))) < radius:
            return False
    return True


def release_point(plat, side, top):
    """Any point of the hole's orbit works as a release point. Prefer the arm's own side, then the
    robot-facing side, and take the first whose column (ball + gripper + wrist) is free."""
    phi0 = 0.0 if side > 0 else np.pi
    offs = [0.0]
    for k in range(1, 7):
        offs += [k * np.pi / 12 * (-side), k * np.pi / 12 * side]   # turn towards -y first
    for d in offs:
        phi = phi0 + d
        q = plat.center + (plat.radius - RADIAL_INSET) * np.array([np.cos(phi), np.sin(phi)])
        if column_clear(q, top, top + GRIPPER_COLUMN, GRIPPER_RADIUS):
            DEBUG["release_phi_offset"] = round(float(d), 3)
            return q
    DEBUG["release_phi_offset"] = None
    return plat.center + side * np.array([plat.radius, 0.0])


def corridor_hits(box, a, b, half_width):
    """Does the box's xy footprint come within half_width of the segment a-b?"""
    lo, hi = box.center[:2] - box.extents[:2] / 2, box.center[:2] + box.extents[:2] / 2
    for s in np.linspace(0, 1, 41):
        p = a + s * (b - a)
        q = np.clip(p, lo, hi)
        if np.linalg.norm(p - q) < half_width:
            return True
    return False


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def move_ball_to(api, arm, target, hold=False, close_axis=None):
    """Move the gripper so the held ball's centre goes to `target`. A ball is symmetric about the
    vertical, so when a plan fails retry with the gripper turned about the vertical (the grasp's yaw is
    arbitrary), rather than continuing from wherever the arm stopped."""
    b = mean_state(api, "ball")["p"]
    ee = np.asarray(api.ee_pose(arm), dtype=np.float64)
    off = b - ee[:3]
    tries = list(YAW_TRIES)
    if close_axis is not None:   # first turn the gripper so its closing axis lies along close_axis (mod 180)
        cur = quat_mat(ee[3:])[:2, 1]
        d = wrap(np.arctan2(close_axis[1], close_axis[0]) - np.arctan2(cur[1], cur[0]))
        d = d - np.pi * np.round(d / np.pi)
        tries = [d + t for t in YAW_TRIES]
    for yaw in tries:
        c, s = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        q = qmul([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)], ee[3:])
        pos = np.asarray(target) - Rz @ off
        if api.move_to([*pos, *q], arm, avoid=AVOID, hold_orientation=hold and abs(yaw) < 1e-6):
            return True
    DEBUG.setdefault("carry_fail", []).append(np.round(target, 3).tolist())
    return False


def dense_samples(plat, n, dt):
    for _ in range(n):
        plat.sample()
        plat.api.wait(dt)


def run(api):
    ball = mean_state(api, "ball")
    holes = [h for h in ("hole A", "hole B") if api.detect(h) is not None]
    ext = {h: float(api.detect(h).bbox_extents[0]) for h in holes}
    r_ball = float(np.mean(ball["ext"])) / 2 if ball["ext"].min() > 1e-3 else R_BALL_DEFAULT
    fits = [h for h in holes if ext[h] > 2 * r_ball]
    target = max(fits or holes, key=lambda h: ext[h])
    decoy = next((h for h in holes if h != target), None)
    DEBUG.update(target=target, ext=ext, r_ball=r_ball)

    RAW_OBST[:] = perceived_obstacles(api)
    AVOID[:] = RAW_OBST
    DEBUG["obstacles"] = [o.name for o in AVOID]
    plat = Platform(api, target, decoy)
    dense_samples(plat, 10, 0.1)
    sorter = mean_state(api, "sorter")
    top = float(sorter["c"][2] + sorter["ext"][2] / 2) if sorter is not None \
        else float(mean_state(api, target)["p"][2] + 0.006)
    arm = "left" if ball["p"][0] < plat.center[0] else "right"
    side = -1.0 if arm == "left" else 1.0
    DEBUG.update(arm=arm, top=top, w=plat.w, center=plat.center.tolist(), radius=plat.radius)

    # ---- grasp (the ball may be rolling: intercept it, verify it is between the fingers before closing)
    held = False
    for _ in range(4):
        if acquire_ball(api, arm, plat):
            held = True
            break
    if not held:
        return
    plat.sample()

    # ---- lift and carry over the hole orbit on this arm's side, above anything in between
    P = release_point(plat, side, top)
    b = mean_state(api, "ball")["p"]
    z_carry = top + r_ball + TRANSPORT_CLEAR
    for o in RAW_OBST:            # the planner models the arm, not the held ball: carry the ball over
        if corridor_hits(o, b[:2], P, 0.08):
            z_carry = max(z_carry, float(o.center[2] + o.extents[2] / 2) + r_ball + OBST_CLEAR)
    DEBUG["z_carry"] = z_carry
    if not move_ball_to(api, arm, [b[0], b[1], z_carry + 0.02]):
        return
    plat.sample()
    radial = (P - plat.center) / max(1e-6, float(np.linalg.norm(P - plat.center)))
    # fingers radial at release: the moving hole drags the dropped ball tangentially, and an open finger
    # standing tangentially next to it can block it and lift it back out of the hole
    if not move_ball_to(api, arm, [P[0], P[1], z_carry], close_axis=radial):
        # no direct plan: go via a higher waypoint halfway, then on (never continue low through obstacles)
        mid = (b[:2] + P) / 2
        if not (move_ball_to(api, arm, [mid[0], mid[1], z_carry + 0.08]) and
                move_ball_to(api, arm, [P[0], P[1], z_carry], close_axis=radial)):
            return
    plat.sample()
    z_rel = top + r_ball + RELEASE_CLEAR
    move_ball_to(api, arm, [P[0], P[1], z_rel], hold=True)
    for _ in range(3):            # the planner stops a few mm off; the hole is only 5 mm wider than the ball
        # opening fingers push the ball ~5 mm along -z of the gripper frame: pre-compensate
        zax = quat_mat(api.ee_pose(arm)[3:])[:2, 2]
        aim = P + RELEASE_DRIFT * zax / max(1e-6, float(np.linalg.norm(zax)))
        b = mean_state(api, "ball")["p"]
        if np.linalg.norm(aim - b[:2]) <= 0.002 and abs(b[2] - z_rel) <= 0.01:
            break
        move_ball_to(api, arm, [aim[0], aim[1], z_rel], hold=True)
    b = mean_state(api, "ball")["p"]
    if np.linalg.norm(aim - b[:2]) > 0.01 or abs(b[2] - z_rel) > 0.03:
        DEBUG["release_pose_not_reached"] = np.round(b, 3).tolist()
        return                    # never open over the wrong spot: a miss sticks to the platform for good
    plat.sample()
    if not api.gripper_holding(arm):
        DEBUG["dropped_in_transit"] = True
    b = mean_state(api, "ball")["p"]
    plat.sample()

    # ---- time the release
    zax = quat_mat(api.ee_pose(arm)[3:])[:2, 2]
    landing = b[:2] - RELEASE_DRIFT * zax / max(1e-6, float(np.linalg.norm(zax)))
    t_land = plat.next_pass(landing, api.time() + LEAD + 0.05)
    # land a little upstream: the arriving hole then sweeps under the resting ball (it drops in), whereas a
    # ball that lands just after the hole centre has passed catches the receding rim and is dragged off
    t_cmd = t_land - LEAD - UPSTREAM / max(1e-3, abs(plat.w) * plat.radius)
    DEBUG.update(ball_at_release=b.tolist(), t_land=t_land, t_cmd=t_cmd, w_final=plat.w,
                 hole_pred=plat.hole_xy(t_land).tolist(), n_samples=len(plat.t))
    api.wait(max(0.0, t_cmd - api.time()))
    DEBUG["t_cmd_actual"] = api.time()
    api.open_gripper(arm)
    api.wait(0.3)
    api.move_by(arm, dz=0.1, avoid=AVOID)
    api.wait(1.0)
