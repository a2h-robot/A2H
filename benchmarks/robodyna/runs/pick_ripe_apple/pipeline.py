"""pick_ripe_apple: pick the good apple (it ripens green -> red; the spoiled one turns yellow) inside
its ripeness window and drop it into the oscillating basket.

Uses only the RobotAPI (docs/api_spec.md + documented extensions of benchmarks/robodyna/api.py).
"""

import numpy as np

from api import Box

APPLES = ("left apple", "right apple")
RIPE_RGB = np.array([0.92, 0.10, 0.08])      # vivid red = ripe (window centre)
SPOILED_RGB = np.array([0.92, 0.78, 0.08])   # yellowing = spoiled
EE_TO_PINCH = 0.12                           # ee frame sits this far behind the pinch point (tcp)
PRE_DIS = (0.10, 0.08, 0.12)
# Horizontal pinch from the front (fingers open sideways).
# Headings (approach yaw in the table plane, 90 deg = +y) per arm, best first; tilts (deg up).
HEADINGS = {"left": (90, 45, 135, 60, 30, 120), "right": (135, 90, 45, 120, 150, 60)}
TILTS = (0,)
GRASP_LEAD_S = 0.0        # close command issued this long before the predicted ripe time
# The fruit comes off the stem at the first finger contact of a closing hand and starts to fall
# ~0.1 s later, so the squeeze must already be there when the close starts: pre-close to just
# above the "open" command (0.8) so the fingers rest against the fruit, then close.
PRECLOSE = 0.82
PINCH_BACK = 0.01         # pinch point this far short of the fruit centre: the palm must not touch it
PINCH_DZ = -0.012         # and this far below it: squeeze a round fruit below its equator, not above
TCP_TOL = 0.015           # reached pinch point must be this close to the apple centre
HOLD_ABOVE_RIM = 0.06     # apple centre above the basket rim while waiting to drop
END_MARGIN = 0.05         # hold this far inside the basket's travel end
TRIGGER_BEFORE = 0.03     # release when the basket is this far before the hold point, moving out
OBST_PAD = 0.0            # extra clearance around obstacles reported by get_obstacles(), near the fruit
OBST_PAD_TRANSIT = 0.0    # ... and while moving to the pre-grasp (the gripper is checked separately)
LOG = []


def log(api, msg, **kw):
    LOG.append({"t": round(api.time(), 2), "msg": msg, **kw})


def _rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def world_box(s):
    """(lo, hi) world AABB of a detection's box (its extents are in the object frame)."""
    R = _rot(np.asarray(s.quaternion, dtype=float))
    half = np.abs(R) @ (np.asarray(s.bbox_extents, dtype=float) / 2)
    c = np.asarray(s.bbox_center, dtype=float)
    return c - half, c + half


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        q = [0.25 * S, (R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / S, 0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S]
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / S, (R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S]
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / S, (R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S]
    q = np.asarray(q, dtype=float)
    return (q / np.linalg.norm(q)).tolist()


def grasp_quat(heading_deg, tilt_deg):
    """Gripper x = approach (heading in the table plane, pitched up by tilt), fingers horizontal."""
    h = np.array([np.cos(np.radians(heading_deg)), np.sin(np.radians(heading_deg)), 0.0])
    th = np.radians(tilt_deg)
    x = np.cos(th) * h + np.array([0, 0, np.sin(th)])
    y = np.cross([0, 0, 1.0], h)
    z = np.cross(x, y)
    return mat_to_quat(np.stack([x, y, z], 1))


def grasp_quats(arm):
    return [grasp_quat(h, t) for t in TILTS for h in HEADINGS[arm]]


GRIPPER_BOX = ((0.02, 0.165), (-0.065, 0.065), (-0.035, 0.035))   # ee frame: x along approach


def gripper_clear(ee, boxes, margin=0.01):
    """True if the open gripper's volume at ee pose stays out of every (axis-aligned) box."""
    if not boxes:
        return True
    R = _rot(np.asarray(ee[3:], dtype=float))
    g = np.array([[x, y, z] for x in np.linspace(*GRIPPER_BOX[0], 6) for y in np.linspace(*GRIPPER_BOX[1], 5)
                  for z in np.linspace(*GRIPPER_BOX[2], 3)])
    pts = g @ R.T + np.asarray(ee[:3], dtype=float)
    for b in boxes:
        lo = np.asarray(b.center) - np.asarray(b.extents) / 2 - margin
        hi = np.asarray(b.center) + np.asarray(b.extents) / 2 + margin
        if np.any(np.all((pts >= lo) & (pts <= hi), axis=1)):
            return False
    return True


def segment_clear(a, b, boxes, n=8, margin=0.01):
    """Gripper volume clear at n poses on the straight line a -> b (b's orientation throughout)."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    for f in np.linspace(0, 1, n):
        p = b.copy()
        p[:3] = a[:3] + (b[:3] - a[:3]) * f
        if not gripper_clear(p, boxes, margin):
            return False
    return True


def ee_for_pinch(center, quat, back):
    c = np.asarray(center, dtype=float)
    p = c - (EE_TO_PINCH + back) * _rot(quat)[:, 0]
    return [*map(float, p), *map(float, quat)]


def pinch(center, quat):
    return np.asarray(center, dtype=float) - PINCH_BACK * _rot(quat)[:, 0] + [0, 0, PINCH_DZ]


def observe(api, dur, samples, baskets, dt=0.2):
    t_end = api.time() + dur
    while api.time() < t_end:
        for n in APPLES:
            s = api.detect(n)
            if s is not None and s.rgb is not None:
                samples[n].append((s.time, np.asarray(s.rgb[:3]), np.asarray(s.bbox_center)))
        b = api.detect("basket")
        if b is not None:
            lo, hi = world_box(b)
            baskets.append((b.time, float(b.position[0]), lo, hi))
        api.wait(dt)


def progress(rgb, rgb0, target):
    d = target - rgb0
    return float(np.dot(rgb - rgb0, d) / max(1e-6, np.dot(d, d)))


def ripe_fit(samples, rgb0):
    """Linear fit of progress towards ripe red over time -> (slope, t_ripe)."""
    t = np.array([s[0] for s in samples])
    p = np.array([progress(s[1], rgb0, RIPE_RGB) for s in samples])
    A = np.stack([t, np.ones_like(t)], 1)
    k, c = np.linalg.lstsq(A, p, rcond=None)[0]
    t_ripe = (1.0 - c) / k if k > 1e-4 else None
    return float(k), (None if t_ripe is None else float(t_ripe))


def pick_good(samples):
    """Good apple = the one moving towards red, not towards yellow (green channel falls)."""
    score = {}
    for n in APPLES:
        s = samples[n]
        if len(s) < 4:
            score[n] = -9.0
            continue
        rgb = np.array([x[1] for x in s])
        k = len(rgb) // 3 or 1
        start, now = rgb[:k].mean(0), rgb[-k:].mean(0)
        score[n] = float(np.linalg.norm(now - SPOILED_RGB) - np.linalg.norm(now - RIPE_RGB)) \
            + float(start[1] - now[1])
    return max(score, key=score.get), score


def basket_sweep(baskets):
    """Box covering everywhere the basket has been seen (it oscillates), up to its rim."""
    lo = np.min([b[2] for b in baskets], 0)
    hi = np.max([b[3] for b in baskets], 0)
    rim = float(np.median([b[3][2] for b in baskets]))
    hi[2] = rim + 0.01
    lo[2] = 0.74
    return Box("basket_sweep", (lo + hi) / 2, np.array([1.0, 0, 0, 0]), hi - lo), rim


def settle(api, arm, max_s=1.5, step_s=0.1, eps=0.002):
    """The arm keeps converging after a move returns (tracking lag): wait until the tcp stops."""
    last = np.asarray(api.tcp_pose(arm)[:3])
    t_end = api.time() + max_s
    while api.time() < t_end:
        api.wait(step_s)
        cur = np.asarray(api.tcp_pose(arm)[:3])
        if np.linalg.norm(cur - last) < eps:
            break
        last = cur


def move_checked(api, arm, ee, tcp_target=None, tol=TCP_TOL, **kw):
    ok = api.move_to(ee, arm, **kw)
    if not ok:
        return False
    settle(api, arm)
    if tcp_target is None:
        return True
    return float(np.linalg.norm(np.asarray(api.tcp_pose(arm)[:3]) - tcp_target)) < tol


def run(api):
    LOG.clear()
    samples = {n: [] for n in APPLES}
    baskets = []
    observe(api, 3.0, samples, baskets)
    good, score = pick_good(samples)
    other = [n for n in APPLES if n != good][0]
    side = -1.0 if good == "left apple" else 1.0
    arm = "left" if side < 0 else "right"
    log(api, "picked", good=good, score=score)

    rgb0 = np.mean([s[1] for s in samples[good][:3]], 0)
    k0, t_ripe0 = ripe_fit(samples[good], rgb0)
    log(api, "ripe_first", k=k0, t_ripe=t_ripe0)
    center = np.mean([s[2] for s in samples[good]], 0)
    sweep, rim = basket_sweep(baskets)
    apple_box = api.detect(good)
    other_box = api.detect(other)
    # Everything else that occupies the workspace (static clutter, posts, ...).
    # The planner only keeps the ARM clear of what it is given (padded a little for tracking lag);
    # the gripper and open fingers (~16 cm long, ~13 cm wide) are checked here, against the raw boxes.
    # obstacle boxes are noisy per call: average a few looks (static clutter does not move)
    def sense():
        looks = [api.get_obstacles(exclude=(*APPLES, "basket")) for _ in range(6)]
        return [Box(o.name, np.mean([lk[i].center for lk in looks], 0), np.asarray(o.quaternion),
                    np.asarray(o.extents)) for i, o in enumerate(looks[0])]
    raw_obst = sense()

    def padded(pad):
        return [Box(o.name, np.asarray(o.center), np.asarray(o.quaternion), np.asarray(o.extents) + 2 * pad)
                for o in raw_obst]
    obst = padded(OBST_PAD)                 # close to the fruit: a little clearance only
    obst_far = padded(OBST_PAD_TRANSIT)     # in transit: room for the gripper the planner does not model
    log(api, "obstacles", n=len(obst), names=[b.name for b in obst])
    pre_avoid = [sweep, apple_box, other_box, *obst_far]

    def poses_for(q, d):
        pre = ee_for_pinch(center, q, d)
        fin = ee_for_pinch(pinch(center, q), q, 0.0)
        return pre, fin

    def path_clear(q, d, margin):
        pre, fin = poses_for(q, d)
        path = [np.asarray(pre) + (np.asarray(fin) - np.asarray(pre)) * f for f in np.linspace(0, 1, 7)]
        return all(gripper_clear(p_, raw_obst, margin) for p_ in path)

    cands = [(q, d) for q in grasp_quats(arm) for d in PRE_DIS]

    def any_clear(boxes, margin=0.01):
        for q, d in cands:
            pre, fin = poses_for(q, d)
            if all(gripper_clear(np.asarray(pre) + (np.asarray(fin) - np.asarray(pre)) * f, boxes, margin)
                   for f in np.linspace(0, 1, 7)):
                return True
        return False

    def push_aside(o, direction):
        """Slide a blocking object sideways (direction +1/-1 along x) with the open hand: come down
        beside the face opposite to the push, near its top, then sweep and lift away."""
        c, e = np.asarray(o.center, dtype=float), np.asarray(o.extents, dtype=float)
        out = float(direction)
        q = grasp_quat(90, 0)
        x = _rot(q)[:, 0]
        z = float(c[2] + e[2] / 2 - 0.03)
        start = np.array([c[0] - out * (e[0] / 2 + 0.085), c[1] - 0.03, z])
        high = start + [0, 0, float(c[2] + e[2] / 2 - z) + 0.08]

        def ee(tcp):
            return [*map(float, np.asarray(tcp) - EE_TO_PINCH * x), *map(float, q)]
        others = [b for b in raw_obst if b is not o]
        if not (gripper_clear(ee(high), raw_obst) and segment_clear(ee(high), ee(start), raw_obst, margin=0.0)):
            return False
        if not (api.move_to(ee(high), arm, avoid=[sweep, apple_box, other_box, *others])
                or api.move_to(ee(high), arm, avoid=[apple_box, other_box])):
            log(api, "push_high_failed", tcp=api.tcp_pose(arm)[:3])
            return False
        settle(api, arm)
        # straight down beside it (the gripper path was checked above; the arm tries with, then without
        # the obstacle boxes and the orientation constraint)
        if not (api.move_to(ee(start), arm, avoid=[sweep, *others], hold_orientation=True)
                or api.move_to(ee(start), arm, avoid=others, hold_orientation=True)
                or api.move_to(ee(start), arm)):
            log(api, "push_descend_failed", tcp=api.tcp_pose(arm)[:3], want=start.tolist())
            return False
        settle(api, arm)
        if float(np.linalg.norm(np.asarray(api.tcp_pose(arm)[:3]) - start)) > 0.03:
            log(api, "push_descend_missed", tcp=api.tcp_pose(arm)[:3], want=start.tolist())
            return False
        api.move_by(arm, dx=out * (0.22 if out * c[0] > 0 else 0.13), hold_orientation=True)
        api.move_by(arm, dz=0.12)
        return True

    if raw_obst and not any_clear(raw_obst):
        # Something stands in every approach corridor. If it is a single object whose removal opens
        # one, try to move it out of the way first (light clutter can be pushed); then look again.
        for o in raw_obst:
            if any_clear([b for b in raw_obst if b is not o]):
                # away from the centre line first (out of every corridor); if this arm cannot get
                # beside its inner face, push it towards the centre line from the outside instead
                # away from the centre line first (out of every corridor); if this arm cannot get
                # beside its inner face, push it towards the centre line from the outside instead
                outward = 1.0 if o.center[0] > 0 else -1.0
                ok = push_aside(o, outward)
                log(api, "push_aside", name=o.name, dir=outward, ok=ok, tcp=api.tcp_pose(arm)[:3])
                if not ok:
                    ok = push_aside(o, -outward)
                    log(api, "push_aside", name=o.name, dir=-outward, ok=ok, tcp=api.tcp_pose(arm)[:3])
                raw_obst = sense()
                obst = padded(OBST_PAD)
                obst_far = padded(OBST_PAD_TRANSIT)
                pre_avoid = [sweep, apple_box, other_box, *obst_far]
                log(api, "obstacles_after_push", centers=[np.asarray(b.center).tolist() for b in raw_obst])
                break

    clear, used_margin = [], None
    for margin in (0.01, 0.0, -0.01, -0.02):        # most clearance first; relax only if nothing fits
        clear = [c for c in cands if path_clear(*c, margin)]
        if clear:
            used_margin = margin
            break
    log(api, "approach_candidates", n=len(cands), clear=len(clear), margin=used_margin)

    def go_pre(q, d):
        pre, _ = poses_for(q, d)
        want = np.asarray(pre[:3]) + EE_TO_PINCH * _rot(q)[:, 0]
        cur = api.ee_pose(arm)
        route = None     # list of (ee pose, keep current orientation?)
        if not raw_obst or segment_clear(cur, pre, raw_obst):
            route = [(pre, False)]
        else:
            # The planner keeps only the arm clear; the gripper is checked here. Route: drop straight
            # down to the pre-grasp height with the current hand orientation, turn the hand there
            # (a staging pose behind the pre-grasp along the approach axis), then slide straight in.
            tcp_now = np.asarray(api.tcp_pose(arm)[:3])
            down = list(cur)
            down[2] = float(cur[2]) + (float(want[2]) - float(tcp_now[2]))
            if segment_clear(cur, down, raw_obst):
                x = _rot(q)[:, 0]
                for back in (0.08, 0.14, 0.20):
                    stage = list(pre)
                    stage[:3] = (np.asarray(pre[:3]) - back * x).tolist()
                    if segment_clear(stage, pre, raw_obst) and gripper_clear(stage, raw_obst):
                        route = [(down, True), (stage, False), (pre, True)]
                        break
        log(api, "route_pre", q=q, d=d, n=None if route is None else len(route))
        if route is None:
            return False
        ok = True
        for wp, keep in route:
            tcp_t = None if wp is not pre else want
            ok = api.move_to(wp, arm, avoid=pre_avoid, hold_orientation=keep)
            if ok:
                settle(api, arm)
                if tcp_t is not None:
                    ok = float(np.linalg.norm(np.asarray(api.tcp_pose(arm)[:3]) - tcp_t)) < 0.03
            if not ok:
                break
        log(api, "go_pre", q=q, d=d, ok=ok, tcp=api.tcp_pose(arm)[:3], want=want.tolist())
        return ok

    api.open_gripper(arm)
    quat = pre_d = None
    for q, d in clear:
        if go_pre(q, d):
            quat, pre_d = q, d
            break
    log(api, "pregrasp", ok=quat is not None, quat=quat, pre=pre_d, tcp=api.tcp_pose(arm)[:3],
        center=center.tolist())
    if quat is None:
        return

    # keep watching ripeness (and the basket) until shortly before ripe
    k, t_ripe = k0, t_ripe0
    while api.time() < 40:
        if t_ripe is not None and api.time() >= t_ripe - GRASP_LEAD_S - 3.0:
            break
        observe(api, 0.5, samples, baskets)
        before = [x for x in samples[good] if t_ripe0 is None or x[0] <= t_ripe0 - 0.5]
        if len(before) >= 6:
            k1, t1 = ripe_fit(before, rgb0)
            if t1 is not None and k1 > 0.02:
                k, t_ripe = k1, t1
    sweep, rim = basket_sweep(baskets)
    log(api, "ripe_fit", k=k, t_ripe=t_ripe, rim=rim, sweep_c=sweep.center.tolist(),
        sweep_e=sweep.extents.tolist())

    # straight in (orientation held); check the pinch point really reached the apple centre
    def go_in(q):
        settle(api, arm)
        target = pinch(center, q)
        for avoid in ([sweep, *obst], obst, None):
            if move_checked(api, arm, ee_for_pinch(target, q, 0.0), target, hold_orientation=True,
                            avoid=avoid, speed="slow"):
                return True
            if np.linalg.norm(np.asarray(api.tcp_pose(arm)[:3]) - target) < 0.05:
                return False
        return False

    ok = go_in(quat)
    log(api, "in", ok=ok, tcp=api.tcp_pose(arm)[:3], center=center.tolist(), quat=quat)
    if not ok:
        for q, d in clear:
            if q == quat or not go_pre(q, d):
                continue
            if go_in(q):
                ok, quat = True, q
                break
        log(api, "in_retry", ok=ok, tcp=api.tcp_pose(arm)[:3])
        if not ok:
            return
    # the stem breaks at first finger contact and the fruit drops shortly after: pre-close so the
    # final close reaches it at once
    api.close_gripper(arm, PRECLOSE)
    log(api, "preclosed", opening=api.gripper_opening(arm), tcp=api.tcp_pose(arm)[:3])
    if t_ripe is not None:
        api.wait_until(lambda: api.time() >= t_ripe - GRASP_LEAD_S, 10.0)
    api.close_gripper(arm)
    held = api.gripper_holding(arm)
    log(api, "closed", holding=held, opening=api.gripper_opening(arm))
    if not held:
        return

    # clear: drop a little away from the branch above, slide outward past the branch tip, then lift.
    # Short moves near the start pose: no sweep box (the grasp pose may already sit close to it).
    api.move_by(arm, dz=-0.015, speed="slow", hold_orientation=True, avoid=obst)
    api.move_by(arm, dx=0.08 * side, speed="slow", hold_orientation=True, avoid=obst)
    cur = api.ee_pose(arm)
    lift = 0.0
    for dz in (0.10, 0.08, 0.06, 0.04, 0.02):     # highest lift whose straight path keeps the gripper clear
        up = list(cur)
        up[2] += dz
        if segment_clear(cur, up, raw_obst):
            lift = dz
            break
    if lift > 0 and not api.move_by(arm, dz=lift, speed="slow", avoid=[sweep, *obst]):
        if not api.move_by(arm, dz=lift, speed="slow", avoid=obst):
            api.move_by(arm, dz=lift, speed="slow")
    settle(api, arm)
    log(api, "cleared", tcp=api.tcp_pose(arm)[:3], holding=api.gripper_holding(arm))

    # drop: hold above the basket's travel end on this side and release as it arrives.
    # The ends come from observation: keep watching (holding the fruit) until the basket has been
    # seen turning around on this side, at most ~10 s (one full period at the slowest speed).
    t_watch = api.time() + 6.0
    while api.time() < t_watch:
        xs = np.array([b[1] for b in baskets[-60:]])
        ts = np.array([b[0] for b in baskets[-60:]])
        if len(xs) > 10:
            k_end = int(np.argmax(xs * side))
            # a turn on our side: the extreme lies inside the recent window, not at its edges
            if 3 <= k_end <= len(xs) - 4 and ts[-1] - ts[0] > 2.0:
                break
        observe(api, 0.3, samples, baskets, dt=0.1)
    xs = np.array([b[1] for b in baskets])
    x_end = float(xs.max() - 0.01) if side > 0 else float(xs.min() + 0.01)   # extreme seen, less noise
    x_hold = x_end - side * END_MARGIN
    lo_y = float(np.median([b[2][1] for b in baskets]))
    hi_y = float(np.median([b[3][1] for b in baskets]))
    by = (lo_y + hi_y) / 2
    target_c = np.array([x_hold, by + 0.02, rim + HOLD_ABOVE_RIM])

    def go(tc, avoid):
        tcp = np.asarray(api.tcp_pose(arm)[:3])
        ee = np.asarray(api.ee_pose(arm), dtype=float)
        ee[:3] += np.asarray(tc) - tcp
        # slow while carrying: a fast swing flings the fruit out of a friction grip
        return move_checked(api, arm, ee, np.asarray(tc), tol=0.02, avoid=avoid, speed="slow")

    # short straight legs (towards the front at carry height, across, then down) instead of one
    # long free-form plan, which may swing the fruit through a wide arc
    cur = np.asarray(api.tcp_pose(arm)[:3])
    ee_now = np.asarray(api.ee_pose(arm), dtype=float)
    off = ee_now[:3] - cur

    def ee_at(tc):
        e = ee_now.copy()
        e[:3] = np.asarray(tc) + off
        return e

    z_want = max(float(cur[2]), rim + HOLD_ABOVE_RIM + 0.06)
    z_carry = z_want
    for z in np.arange(z_want, rim + HOLD_ABOVE_RIM - 1e-6, -0.01):
        legs = [cur, [cur[0], cur[1], z], [cur[0], target_c[1], z], [target_c[0], target_c[1], z], target_c]
        if all(segment_clear(ee_at(a), ee_at(b), raw_obst) for a, b in zip(legs[:-1], legs[1:])):
            z_carry = float(z)
            break
    log(api, "carry", z_want=z_want, z_carry=z_carry)
    def go_any(tc):     # the gripper's straight path is checked above; the arm tries with, then without boxes
        return go(tc, [sweep, *obst]) or go(tc, obst) or go(tc, None)
    if abs(z_carry - cur[2]) > 0.01:
        go_any([cur[0], cur[1], z_carry])
    go_any([cur[0], target_c[1], z_carry])
    go_any([target_c[0], target_c[1], z_carry])
    placed = False
    for dz in (0.0, 0.03, 0.06):
        tc = target_c + [0, 0, dz]
        if go(tc, [sweep, *obst]) or go(tc, obst):
            placed = True
            break
    tcp = np.asarray(api.tcp_pose(arm)[:3])
    log(api, "over_basket", ok=placed, tcp=tcp.tolist(), x_end=x_end, x_hold=x_hold, rim=rim)
    if not placed:
        # hold wherever we are, but only if that is over the basket's lane and above its rim
        if not (lo_y + 0.03 < tcp[1] < hi_y - 0.03 and tcp[2] > rim + 0.03 and
                xs.min() < tcp[0] < xs.max()):
            return
        x_hold = float(tcp[0])
    if not api.gripper_holding(arm):
        return

    # basket x from a short sliding window of detections: fitted position and velocity
    win = []

    def arriving():
        b = api.detect("basket")
        if b is not None:
            win.append((b.time, float(b.position[0])))
        del win[:-8]
        if len(win) < 6:
            return False
        t = np.array([w[0] for w in win])
        x = np.array([w[1] for w in win])
        v, c = np.polyfit(t - t[-1], x, 1)
        return v * side > 0.03 and (c - (x_hold - side * TRIGGER_BEFORE)) * side >= 0
    ok = api.wait_until(arriving, 15.0, poll_s=0.04)
    log(api, "release", trigger=ok, win=win[-1] if win else None)
    api.open_gripper(arm)
    api.move_by(arm, dz=0.08)
    api.wait(1.0)
    log(api, "done")
