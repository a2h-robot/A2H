"""catch_valley_ball: push the blue box past the red line to where the red ball will land.

Strategy
1. Perceive the red line (its x is the catch boundary, its y-extent is the ramp lane width), the
   box and both balls. The side the red ball starts on gives the travel direction and the arm.
2. While the arm moves into position, keep sampling both balls (averaged detections). Their lateral
   path on the ramp is a straight line in x that reflects off the rails; fit that model and roll it
   forward (with the equal-mass ball-ball bounce) to the ramp exit, then extend the exit heading in
   free flight to the landing x (a fixed distance past the red line).
3. Closed loop until just before release: re-predict, measure the box, and push it along the axis
   with the larger error from the face that points away from the target, going over the top of the
   box to change faces. Grip and approach depend on the free space around the box (obstacles).
   Then retract out of the ball's way.
Only the API in docs/api_spec.md (+ documented extensions) is used.
"""
import numpy as np

R_BALL = 0.018
FINGER = 0.116          # tcp-to-box-centre distance along y when the leading open finger touches a face
PUSH_Z = 0.80           # tcp height for pushing: open fingertips reach 4.3 cm below the tcp, under the 4 cm rim
FINGER_DROP = 0.043     # open fingertips below the tcp
RIM_Z = 0.775           # fingertips must be below this to meet the box wall (rim at 0.78)
HOVER_Z = 0.835
FINGER_X = 0.057        # tcp-to-box-centre distance along x when one open finger touches an x face
SIDE_Y = 0.06           # tcp y offset that puts that finger on the middle of the x face
X_STEP = 0.08           # x pushes in 8 cm segments (~0.3 s each; the box keeps up)
SHORT = 0.01            # main push stops this far short of the predicted landing (in +y)
T_MAX = 8.0             # the ball has always left the ramp by then (release at 5.7-6.6 s)
RETRACT_S = 0.45        # leave the box this long before the predicted release
T_REL_MIN = 5.3         # never trust an early extrapolation that has the ball leave sooner than this
Y_STEP = 0.05           # forward corrections in 5 cm segments
X_FIX = 0.02            # correct x when the box is this far from the landing x
Y_FIX = 0.012           # correct y when the box is this far from the predicted landing y
Y_TRUST_T = 2.8         # before this, cap the y target near the robot-side lane edge
RELOCATE_S = 0.9        # time to move the hand round to another face of the box
N_AVG = 16              # detections averaged per sample (1 cm noise each)
LAND_PAST_LINE = 0.142  # ball lands this far past the red line (in travel direction)
EXIT_BEFORE_LINE = 0.06  # ball leaves the ramp this far before the red line
REACH_X = 0.36
MIN_SEP = 2 * R_BALL
DEBUG = []            # (time, target xy, release time) per prediction, for offline inspection


def mean_pos(api, name, n=N_AVG):
    ps = [s.position for s in (api.detect(name) for _ in range(n)) if s is not None]
    return None if not ps else np.mean(ps, 0)


class Tracker:
    def __init__(self, api, names):
        self.api, self.names = api, names
        self.s = {n: [] for n in names}

    def sample(self):
        t = self.api.time()
        for n in self.names:
            p = mean_pos(self.api, n)
            if p is not None:
                self.s[n].append((t, *p))

    def arr(self, name, t_min=0.0):
        a = np.array([r for r in self.s[name] if r[0] >= t_min])
        return a.reshape(-1, 4)


def reflect(u, lo, hi):
    w = hi - lo
    ph = np.mod(u - lo, 2 * w)
    return np.where(ph <= w, lo + ph, hi - (ph - w))


def fit_lane(xs, ys, lo, hi, x_ref):
    """Fit y = reflect(u + s (x - x_ref)) between the rails; return (y, dy/dx) at x_ref."""
    if len(xs) < 2:
        return float(ys[-1]) if len(ys) else 0.5 * (lo + hi), 0.0
    w = hi - lo
    s_grid = np.linspace(-0.7, 0.7, 281)
    u_grid = lo + np.linspace(0, 2 * w, 240, endpoint=False)
    S, U = np.meshgrid(s_grid, u_grid, indexing="ij")
    pred = reflect(U[..., None] + S[..., None] * (xs - x_ref)[None, None, :], lo, hi)
    err = ((pred - ys[None, None, :]) ** 2).sum(-1)
    i, j = np.unravel_index(np.argmin(err), err.shape)
    s, u = S[i, j], U[i, j]
    ph = (u - lo) % (2 * w)
    y = float(reflect(np.array(u), lo, hi))
    return y, float(s if ph <= w else -s)


def fit_recent(xs, ys, lo, hi, x_ref, tol=0.004, min_n=8):
    """fit_lane on the longest recent stretch the model explains (rms <= tol): a ball-ball bounce
    swaps slopes, so older samples may follow a different line."""
    n = len(xs)
    if n <= min_n:
        return fit_lane(xs, ys, lo, hi, x_ref)
    best = None
    for k in range(n, min_n - 1, -1):
        y, s = fit_lane(xs[n - k:], ys[n - k:], lo, hi, x_ref)
        rms = _rms(xs[n - k:], ys[n - k:], lo, hi, x_ref, y, s)
        if rms <= tol:
            return y, s
        if best is None or rms < best[0]:
            best = (rms, y, s)
    return best[1], best[2]


def _rms(xs, ys, lo, hi, x_ref, y, s):
    """Residual of the reflected line through (x_ref, y) with slope s (walked backwards)."""
    out = []
    for x, yo in zip(xs, ys):
        yy, _ = lane_step(y, s, x - x_ref, lo, hi)
        out.append(yy - yo)
    return float(np.sqrt(np.mean(np.square(out))))


def lane_step(y, s, dx, lo, hi):
    y = y + s * dx
    for _ in range(6):
        if y < lo:
            y, s = lo + (lo - y), -s
        elif y > hi:
            y, s = hi - (y - hi), -s
        else:
            break
    return float(np.clip(y, lo, hi)), s


def roll_forward(yr, sr, yd, sd, x0, x1, lo, hi):
    """Integrate both balls' lateral motion from x0 to x1 (rails + equal-mass ball-ball bounce)."""
    n = max(1, int(abs(x1 - x0) / 0.004))
    dx = (x1 - x0) / n
    for _ in range(n):
        yr, sr = lane_step(yr, sr, dx, lo, hi)
        if yd is not None:
            yd, sd = lane_step(yd, sd, dx, lo, hi)
            if abs(yr - yd) < MIN_SEP:
                sr, sd = sd, sr
                mid = 0.5 * (yr + yd)
                if yr <= yd:
                    yr, yd = mid - MIN_SEP / 2, mid + MIN_SEP / 2
                else:
                    yr, yd = mid + MIN_SEP / 2, mid - MIN_SEP / 2
    return yr, sr, yd, sd


def predict_landing(r, d, lo, hi, x_exit, x_land, contact=0.042, min_after=6):
    """Landing y of the red ball (and of the black one) from sample arrays [t, x, y, z].
    A ball-ball bounce swaps the two lateral slopes; a fit window that straddles one mixes two
    lines. So split the history at the last near-contact: fit after it when there are enough
    samples, otherwise fit before it and let roll_forward simulate the bounce."""
    if d is not None and len(d) == len(r) and len(r) > 2:
        sep = np.abs(r[:, 2] - d[:, 2])
        near = np.where(sep < contact)[0]
        if len(near):
            ib = int(near[-1])
            i0 = ib
            while i0 > 0 and np.any(sep[max(0, i0 - 4):i0] < contact):   # bridge noisy gaps
                i0 -= 1
            if len(r) - 1 - ib >= min_after:
                r, d = r[ib + 1:], d[ib + 1:]
            elif i0 >= 3:
                r, d = r[:i0], d[:i0]
    xr = r[-1, 1]
    yr, sr = fit_recent(r[:, 1], r[:, 2], lo, hi, xr)
    yd = sd = None
    if d is not None and len(d):
        yd, sd = fit_recent(d[:, 1], d[:, 2], lo, hi, d[-1, 1])
    yr_e, sr_e, yd_e, sd_e = roll_forward(yr, sr, yd, sd, xr, x_exit, lo, hi)
    y_land = yr_e + sr_e * (x_land - x_exit)
    yd_land = None if yd_e is None else yd_e + sd_e * (x_land - x_exit)
    return float(y_land), (None if yd_land is None else float(yd_land))


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


class Hand:
    """Commands the fingertip point (tcp) instead of the wrist (ee) at a fixed orientation."""

    def __init__(self, api, arm, q):
        self.api, self.arm, self.q = api, arm, list(map(float, q))
        self.off = np.array([0.0, 0.0, 0.12])
        self.stuck = False
        self.avoid = []          # obstacles the planner must keep clear of on free-space moves

    def _update(self):
        self.off = np.asarray(self.api.ee_pose(self.arm)[:3]) - np.asarray(self.api.tcp_pose(self.arm)[:3])

    def tcp(self):
        return np.asarray(self.api.tcp_pose(self.arm)[:3])

    def to(self, xyz, straight=False):
        ee = np.asarray(xyz, dtype=float) + self.off
        ok = self.api.move_to([*ee, *self.q], self.arm, hold_orientation=straight,
                              avoid=None if straight else (self.avoid or None))
        self._update()
        return ok

    def slide(self, xy, z, step=0.03, on_step=None):
        """Move the fingertips to xy at height z in short straight segments: one long plan may arc
        over the box instead of pushing it."""
        cur = self.tcp()[:2]
        xy = np.asarray(xy, dtype=float)
        n = max(1, int(np.ceil(np.linalg.norm(xy - cur) / step)))
        blocked = 0
        for k in range(1, n + 1):
            before = self.tcp()[:2]
            goal = cur + (xy - cur) * k / n
            if not self.to([*goal, z], straight=True):
                return False
            if on_step:
                on_step()
            # the fingers did not get where they were sent: something they push does not give way
            moved = np.linalg.norm(self.tcp()[:2] - before)
            blocked = blocked + 1 if moved < 0.3 * np.linalg.norm(goal - before) else 0
            if blocked >= 2:
                self.stuck = True
                return False
        return True


def run(api):
    names = ["red ball"] + (["black ball"] if "black ball" in api.objects() else [])
    trk = Tracker(api, names)
    trk.sample()

    # ---- scene geometry from perception
    line = [api.detect("red line") for _ in range(12)]
    line_xy = np.mean([s.position for s in line], 0)
    hw = float(line[0].bbox_extents[1]) / 2          # ramp half width
    lane_half = max(0.01, hw - 0.022)                # minus half rail, ball radius
    lo, hi = line_xy[1] - lane_half, line_xy[1] + lane_half
    red0 = trk.s["red ball"][0]
    side = 1.0 if red0[1] < line_xy[0] else -1.0     # ball travels towards the red line
    arm = "right" if side > 0 else "left"
    x_exit = line_xy[0] - side * EXIT_BEFORE_LINE
    x_land = line_xy[0] + side * LAND_PAST_LINE
    x_land = float(np.clip(x_land, -REACH_X, REACH_X))
    # ---- what else is on the table: anything that is not a task object is an obstacle
    task_objs = {"red ball", "blue box", "black ball"}
    # obstacle boxes are noisy (1 cm per call): average a few looks per obstacle
    looks = [[o for o in api.get_obstacles() if o.name not in task_objs] for _ in range(10)]
    obstacles = looks[0]
    for i, o in enumerate(obstacles):
        same = [l[i] for l in looks if len(l) > i and l[i].name == o.name]
        o.center = np.mean([x.center for x in same], 0)
    # Vertical clearance: the open fingertips sweep 4.3 cm below the tcp. Low things near the box
    # (a rail, a cable, a tray rim) can be passed over by pushing a little higher, as long as the
    # fingertips still meet the 4 cm box wall (fingertip below RIM_Z).
    push_z = PUSH_Z
    for o in obstacles:
        top = float(o.center[2] + o.extents[2] / 2)
        near = np.linalg.norm(np.asarray(o.center[:2]) - np.array([x_land, mean_pos(api, "blue box", 4)[1]])) < 0.45
        if near and top + FINGER_DROP + 0.006 > push_z and top + FINGER_DROP + 0.006 <= RIM_Z + FINGER_DROP:
            push_z = top + FINGER_DROP + 0.006
    b0 = mean_pos(api, "blue box", 10)[:2]

    def blocked(xlo, xhi, ylo, yhi, ztop=0.95):
        for o in obstacles:
            lo_, hi_ = o.center - o.extents / 2 - 0.01, o.center + o.extents / 2 + 0.01
            if o.center[2] + o.extents[2] / 2 < push_z - FINGER_DROP:     # the fingertips pass over it
                continue
            if lo_[0] < xhi and hi_[0] > xlo and lo_[1] < yhi and hi_[1] > ylo and lo_[2] < ztop:
                return True
        return False

    # Two ways to hold the open gripper (fingers pointing down):
    #  "along":  turned 90 deg so the fingers line up with the y push; the leading finger pushes the
    #            box's -y face from 11.6 cm behind its centre and the trailing finger reaches 6 cm further back.
    #  "across": home orientation, fingers along x; the hand sits 6 cm to the side so one finger pushes
    #            the -y face from only 6 cm behind the centre: little free space needed behind the box.
    grips = {
        "along": dict(rot=True, yoff=0.116, xside=0.0, xoff=0.057, yside=-0.06, gap=0.03, rear=0.07),
        "across": dict(rot=False, yoff=0.06, xside=side * 0.06, xoff=0.116, yside=0.0, gap=0.012, rear=0.015),
    }

    def needs(g, bx, by):
        back = g["yoff"] + g["gap"] + g["rear"]     # pre-push gap + what the hand needs behind the tcp
        return (bx + g["xside"] - 0.05, bx + g["xside"] + 0.05, by - back, by - 0.046)

    grip = grips["along"]
    for bx in (b0[0], x_land):
        if blocked(*needs(grip, bx, b0[1])) and not blocked(*needs(grips["across"], bx, b0[1])):
            grip = grips["across"]
    FINGER, FINGER_X, SIDE_Y, XS, GAP = grip["yoff"], grip["xoff"], grip["yside"], grip["xside"], grip["gap"]
    q = api.ee_pose(arm)[3:]
    if grip["rot"]:
        q = qmul(q, [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])
    hand = Hand(api, arm, q)
    hand.avoid = obstacles

    def predict():
        """Predicted (box target xy, release time)."""
        t_now = api.time()
        r = trk.arr("red ball", t_min=max(0.25, t_now - 2.5))
        if len(r) < 2:
            r = trk.arr("red ball", 0.25)
        if len(r) < 1:
            return np.array([x_land, line_xy[1]]), 5.7
        d = trk.arr("black ball", t_min=max(0.25, t_now - 2.5)) if "black ball" in names else None
        y_land, yd_land = predict_landing(r, d, lo, hi, x_exit, x_land)
        y_box = y_land
        if yd_land is not None:
            gap = y_land - yd_land
            if abs(gap) < 0.08:                       # keep the black ball out of the box
                y_box = y_land + np.sign(gap if gap != 0 else 1.0) * 0.018
        # release time: quadratic fit of the ball's x(t) extrapolated to the exit
        t_rel = 5.7
        rr = trk.arr("red ball", max(0.3, t_now - 2.0))
        if len(rr) >= 3 and np.ptp(rr[:, 0]) > 0.4:
            c = np.polyfit(rr[:, 0], rr[:, 1] - x_exit, 2)
            roots = [z.real for z in np.roots(c) if abs(z.imag) < 1e-9 and z.real > t_now]
            if roots:
                t_rel = float(min(roots))
        t_rel = max(t_rel, T_REL_MIN)
        DEBUG.append((round(t_now, 2), round(x_land, 3), round(float(y_box), 3), round(t_rel, 2), round(y_land, 3),
                      None if yd_land is None else round(yd_land, 3)))
        return np.array([x_land, float(y_box)]), t_rel

    def box_xy():
        return mean_pos(api, "blue box", 10)[:2]

    # ---- 1-3. closed-loop placement. Every cycle: re-predict the landing, measure the box, and
    # correct the axis with the larger error by pushing on the face that points away from the target.
    # If the hand is not already on that face it goes over the top of the box (fingertips clear the
    # rim at HOVER_Z + 3 cm) and comes down beside it, so it never cuts across the box. Early on the
    # y target is capped near the robot-side lane edge: the first estimates can be far off, and
    # pushing forwards with the finger already in contact is much cheaper than going round.
    def contact_xy(d, b, gap):
        """tcp xy just behind the face that pushes the box in direction d (+x, -x, +y, -y)."""
        s = 1.0 if d[0] == "+" else -1.0
        if d[1] == "y":
            return np.array([b[0] + XS, b[1] - s * (FINGER + gap)])
        return np.array([b[0] - s * (FINGER_X + gap), b[1] + SIDE_Y])

    state = {"face": None}

    def relocate(d, b):
        p = contact_xy(d, b, GAP)
        c = hand.tcp()
        if state["face"] is not None and c[2] < HOVER_Z + 0.02:
            hand.to([c[0], c[1], HOVER_Z + 0.03], straight=True)           # straight up off the box
        ok = hand.to([p[0], p[1], HOVER_Z + 0.03])                          # over (obstacles avoided)
        trk.sample()
        ok = ok and hand.to([p[0], p[1], push_z], straight=True)           # straight down beside the face
        trk.sample()
        state["face"] = d if ok else "none"
        return ok

    bias = {"x": 0.0, "y": 0.0}     # learned extra travel per axis (contact offset not as calibrated)

    def push(d, b, goal):
        """Slide the box along d until its centre coordinate reaches goal, closing the loop on the box
        itself: after the nominal slide (which assumes an axis-aligned box and a known contact offset),
        keep sliding in short extra steps while the box is still short and still moving. A box that
        has been turned (a bump while a fingertip rested on it pivots it) presents a corner, not a face,
        so the nominal offset is wrong by up to 2 cm."""
        s = 1.0 if d[0] == "+" else -1.0
        k = 0 if d[1] == "x" else 1
        aim = goal + s * bias[d[1]]
        if d[1] == "y":
            hand.slide([b[0] + XS, aim - s * FINGER], push_z, step=Y_STEP, on_step=trk.sample)
        else:
            hand.slide([aim - s * FINGER_X, b[1] + SIDE_Y], push_z, step=X_STEP, on_step=trk.sample)
        b_prev, b_after = b, box_xy()
        for _ in range(3):
            short = s * (goal - b_after[k])
            if short < 0.008 or hand.stuck or t_rel - api.time() < RETRACT_S + 0.3:
                break
            c = hand.tcp()
            step = np.zeros(2)
            step[k] = s * min(short + 0.005, 0.04)
            hand.slide([c[0] + step[0], c[1] + step[1]], push_z, step=0.04, on_step=trk.sample)
            b_prev, b_after = b_after, box_xy()
        short = s * (goal - b_after[k])
        if d[1] == "x" and short > 0.005:
            bias[d[1]] = float(np.clip(bias[d[1]] + short, 0.0, 0.05))
        # a push that no longer moves the box is wasted time: after three, stop insisting on that axis
        moved = abs(b_after[k] - b[k])
        stall[d[1]] = stall[d[1]] + 1 if moved < 0.005 else 0

    stall = {"x": 0, "y": 0}

    def ball_left_ramp():
        r = trk.s["red ball"][-1]
        return side * (r[1] - x_exit) > -0.01 or api.time() > T_MAX

    t_rel = T_REL_MIN + 0.4
    while api.time() < t_rel - RETRACT_S and not ball_left_ramp() and not hand.stuck:
        tgt, t_rel = predict()
        y_goal = tgt[1] if api.time() > Y_TRUST_T else min(tgt[1] - SHORT, lo + 0.03)
        b = box_xy()
        ex, ey = x_land - b[0], y_goal - b[1]
        left = t_rel - api.time()
        # x first: the landing x is known from the start, and fixing x after y would mean going
        # round the box twice. A y push that needs the hand moved round must be worth it.
        dy_face = "+y" if ey > 0 else "-y"
        y_thr = Y_FIX if state["face"] == dy_face else 0.025
        if abs(ex) > X_FIX and stall["x"] < 3:
            d, goal = ("+x" if ex > 0 else "-x"), x_land
        elif abs(ey) > y_thr:
            d, goal = dy_face, y_goal
        else:
            api.wait(0.1)
            trk.sample()
            continue
        if state["face"] != d:
            if left < RETRACT_S + RELOCATE_S:
                api.wait(0.1)       # no time to go round: keep the box where it is
                trk.sample()
                continue
            if not relocate(d, b):
                continue
        push(d, box_xy(), goal)
        trk.sample()

    # ---- 4. get out of the ball's way before it leaves the ramp
    c = hand.tcp()
    if state["face"] == "-y":
        hand.to([c[0], c[1] + 0.05, push_z + 0.1])
    elif state["face"] in ("+x", "-x"):
        hand.to([c[0], c[1] - 0.12, push_z + 0.08])
    else:
        hand.to([c[0], c[1] - 0.04, push_z + 0.1])
