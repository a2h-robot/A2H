"""open_the_middle_drawer_of_the_cabinet: grasp the middle drawer's handle and pull it out.

Only the primitive API (benchmarks/libero/api.py) is used.
"""

import sys

import numpy as np


CAB = "wooden_cabinet_1"
DRAWER = "wooden_cabinet_1_cabinet_middle"
BODY = "wooden_cabinet_1_base"
OPEN_TARGET = 0.152      # drawer travel to aim for [m] (goal: > 0.14, joint limit 0.16)
OPEN_MIN = 0.145         # accept once measured travel exceeds this
HANDLE_DEPTH = 0.008     # half thickness of the handle bar [m]
GRIPPER_OPENING = 0.08   # fully open finger gap [m]
PRE_DIST = 0.10          # pre-grasp standoff in front of the handle [m]


def log(*a):
    print("[pipeline]", *a, file=sys.stderr)


def R_of(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_of(R):
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def avg_detect(api, name, n=20):
    """Average n noisy detections (positions, bbox centers; quaternion of the first, sign-aligned mean)."""
    sts = [api.detect(name) for _ in range(n)]
    sts = [s for s in sts if s is not None]
    if not sts:
        return None
    pos = np.mean([s.position for s in sts], axis=0)
    ctr = np.mean([s.bbox_center for s in sts], axis=0)
    q0 = sts[0].quaternion
    qs = np.array([s.quaternion if np.dot(s.quaternion, q0) >= 0 else -s.quaternion for s in sts])
    q = qs.mean(0)
    q /= np.linalg.norm(q)
    return pos, q, ctr, sts[0].bbox_extents


def drawer_frame(api):
    """Handle point and the horizontal pull direction of the drawer, from perception."""
    pos, q, ctr, ext = avg_detect(api, DRAWER)
    _, _, bctr, _ = avg_detect(api, BODY)
    R = R_of(q)
    # The drawer box sticks out of the cabinet body on its front side (handle): the pull
    # direction is the drawer-frame horizontal axis best aligned with body->drawer offset.
    off = ctr - bctr
    best = None
    for i in (0, 1):
        for s in (1, -1):
            ax = s * R[:, i]
            ax = np.array([ax[0], ax[1], 0.0])
            ax /= np.linalg.norm(ax)
            score = off[:2] @ ax[:2]
            if best is None or score > best[0]:
                best = (score, ax, i)
    _, pull, i = best
    half = 0.5 * ext[i]
    handle = ctr + pull * (half - HANDLE_DEPTH)
    handle[2] = ctr[2]
    return handle, pull, pos


# Approach pitched down by this much (palm stays above low clutter). Steeper than ~25 deg the
# open upper finger reaches past the handle's standoff and hits the drawer front.
TILTS_DEG = (25, 15, 5)
YAWS_DEG = (0, 30, -30, 55, -55)  # approach yawed away from the pull axis (around obstacles)
IMMOVABLE = set()                 # obstacles that did not move when grasped and lifted


def grasp_orientations(pull):
    """Gripper approaches against the pull direction (optionally yawed), pitched down by a tilt,
    fingers closing in the vertical plane that contains the approach (pinching the horizontal bar).
    Yields (approach_axis, closing_axis, quaternion)."""
    for yaw in np.deg2rad(YAWS_DEG):
        c, s_ = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
        for tilt in np.deg2rad(TILTS_DEG):
            z = Rz @ (-pull * np.cos(tilt) + np.array([0.0, 0.0, -np.sin(tilt)]))
            x0 = Rz @ (-pull * np.sin(tilt) + np.array([0.0, 0.0, np.cos(tilt)]))
            for xs in (1.0, -1.0):
                x = xs * x0
                y = np.cross(z, x)
                yield z, x, q_of(np.column_stack([x, y, z]))


def hand_samples(grip, z, x):
    """Points on the fingers, palm (wide along the closing axis) and wrist for a grip pose."""
    pts = [grip - z * d for d in (0.04, 0.08, 0.12, 0.16, 0.20, 0.25, 0.30, 0.35)]
    pts += [grip - z * 0.10 + x * w for w in (-0.08, -0.04, 0.04, 0.08)]
    return pts


def in_box(pt, b, margin):
    local = R_of(b.quaternion).T @ (pt - b.center)
    return bool(np.all(np.abs(local) <= 0.5 * np.asarray(b.extents) + margin))


def hand_path_clear(obstacles, grips, z, x, margin=0.04):
    return not any(in_box(pt, b, margin) for g in grips for pt in hand_samples(g, z, x) for b in obstacles)


def travel(api, pos0, pull):
    pos, _, _, _ = avg_detect(api, DRAWER)
    return float((pos - pos0) @ pull)


PULL_LEAD = 0.01         # commanded lead of the hand over its current position while pulling [m]


def grasp_handle(api, handle, pull, handle_closed):
    """Pinch the handle bar; returns the grasp quaternion only if the gripper actually holds it.
    Approaches whose hand/wrist would hit an obstacle anywhere along the pull (from now to fully
    open) are tried last."""
    obstacles = [b for b in averaged_obstacles(api) if not b.name.startswith(CAB)
                 and box_corners(b)[:, 2].max() > handle[2] - 0.08]
    grips = [handle, handle_closed + pull * (OPEN_TARGET * 0.5), handle_closed + pull * OPEN_TARGET]
    cands = list(grasp_orientations(pull))
    pre_grips = [handle]
    def hits(c):
        return sum(in_box(pt, b, 0.04) for g in grips for pt in hand_samples(g, c[0], c[1]) for b in obstacles)
    scored = sorted(range(len(cands)), key=lambda i: (hits(cands[i]), i))
    order = [cands[i] for i in scored]
    log(sum(hits(c) == 0 for c in cands), "of", len(cands), "approaches keep the hand clear over the whole pull")
    for z, x, q in order:
        log("try approach", np.round(z, 2))
        pre = handle - z * PRE_DIST
        if not api.move_to((pre, q)):
            continue
        api.move_to((handle - z * 0.003, q), linear=True, speed="slow", allow_contact=[CAB])
        if api.last_motion == "refused":
            continue
        api.close_gripper()
        if api.gripper_holding():
            return q
        log("closed on nothing, retrying another approach")
        api.open_gripper()
        api.move_to((pre, q), linear=True, allow_contact=[CAB])
    return None


def pull_compliant(api, q, pos0, pull):
    """Pull in small increments relative to where the hand actually is, so the force on the
    handle stays bounded however stiff the drawer is; stop on slip or when it stops moving."""
    last = travel(api, pos0, pull)
    stuck = 0
    for _ in range(80):
        remaining = OPEN_TARGET - last
        if remaining <= 0.002:
            break
        p, _ = api.tcp_pose()
        api.move_to((p + pull * min(PULL_LEAD, remaining + 0.003), q), linear=True, speed="slow",
                    allow_contact=[CAB])
        if api.last_motion == "refused":
            log("pull step refused")
            break
        if not api.gripper_holding():
            log("handle slipped out of the gripper")
            break
        t = travel(api, pos0, pull)
        stuck = stuck + 1 if t - last < 0.001 else 0
        last = t
        if stuck >= 4:
            log("drawer stopped moving")
            break
    return last


TABLE_Z = 0.90
BASE_XY = np.array([-0.66, 0.0])


def averaged_obstacles(api, n=8):
    """get_obstacles() averaged over n calls (positions; orientation of the first call)."""
    calls = [api.get_obstacles() for _ in range(n)]
    out = []
    for b in calls[0]:
        same = [c for call in calls for c in call if c.name == b.name]
        b.center = np.mean([c.center for c in same], axis=0)
        out.append(b)
    return out


def box_corners(b):
    R = R_of(b.quaternion)
    h = 0.5 * np.asarray(b.extents)
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    return b.center + (signs * h) @ R.T


def work_volume_blockers(api, handle, pull):
    """Objects (other than the cabinet) that intrude into the volume the hand and forearm sweep
    while approaching the handle and pulling the drawer out: along the pull direction from just
    in front of the closed handle to well behind the fully opened grip point, a band around the
    handle laterally, and everything from slightly below the handle upward."""
    lat = np.cross([0.0, 0.0, 1.0], pull)
    out = []
    for b in averaged_obstacles(api):
        if b.name.startswith(CAB) or b.name in IMMOVABLE or b.name.split("_main")[0] in IMMOVABLE:
            continue
        c = box_corners(b)
        a = (c - handle) @ pull
        l = (c - handle) @ lat
        if (a.max() > -0.02 and a.min() < OPEN_TARGET + 0.22 and l.max() > -0.09 and l.min() < 0.09
                and c[:, 2].max() > handle[2] - 0.08):
            out.append(b.name)
    return out


def seg_dist(pt, a, b):
    ab = b - a
    t = np.clip((pt - a) @ ab / (ab @ ab), 0.0, 1.0)
    return float(np.linalg.norm(pt - (a + t * ab)))


def free_spot(api, name, handle, pull, footprint, avoid=()):
    """A table spot clear of every other obstacle, of the hand's work volume and of the band
    the forearm sweeps between the robot base and the hand, inside comfortable reach;
    the nearest such spot to the object's current position."""
    st = avg_detect(api, name, n=10)
    cur = st[2][:2]
    lat = np.cross([0.0, 0.0, 1.0], pull)
    others = []
    for b in averaged_obstacles(api):
        if b.name == name or b.name.startswith(name + "_"):
            continue
        c = box_corners(b)
        if c[:, 2].max() < TABLE_Z + 0.005:
            continue
        others.append((c[:, :2].min(0), c[:, :2].max(0)))
    r = 0.5 * footprint + 0.06   # the hand holding it is wider than the object
    h_open = (handle + pull * OPEN_TARGET)[:2]
    best = None
    for x in np.arange(-0.35, 0.20, 0.02):
        for y in np.arange(-0.40, 0.401, 0.02):
            pt = np.array([x, y])
            if not 0.35 < np.linalg.norm(pt - BASE_XY) < 0.78:
                continue
            d = np.append(pt, handle[2]) - handle
            m = r + 0.03
            if -0.02 - m < d @ pull < OPEN_TARGET + 0.22 + m and abs(d @ lat) < 0.09 + m:
                continue
            if min(seg_dist(pt, BASE_XY, handle[:2]), seg_dist(pt, BASE_XY, h_open)) < r + 0.12:
                continue
            if any(np.all(pt > lo - r) and np.all(pt < hi + r) for lo, hi in others):
                continue
            if any(np.linalg.norm(pt - a) < 0.15 for a in avoid):
                continue
            dist = np.linalg.norm(pt - cur)
            if best is None or dist < best[0]:
                best = (dist, pt)
    return None if best is None else best[1]


def top_grasp_averaged(api, name, depth=0.04, edge=False):
    """Top grasp from an averaged detection (single detections are noisy enough to land a
    finger on the rim of a narrow object). edge=True: for objects wider than the gripper,
    pinch the wall/rim on the side nearest the robot, fingers closing across it.
    True if holding afterwards."""
    pos, q, ctr, ext = avg_detect(api, name)
    R = R_of(q)
    ext_w = np.abs(R) @ ext
    top = ctr[2] + 0.5 * ext_w[2]
    ax = R[:, 0] if ext[0] <= ext[1] else R[:, 1]
    if edge:
        d = BASE_XY - ctr[:2]
        d /= np.linalg.norm(d)
        ax = np.array([d[0], d[1], 0.0])
        rad = 0.5 * float(np.min(ext_w[:2]))
        ctr = ctr + ax * (rad - 0.012)
        depth = min(depth, 0.02)
    yaw = np.arctan2(ax[1], ax[0])
    cur = R_of(api.tcp_pose()[1])
    cy = np.arctan2(cur[1, 0], cur[0, 0])
    yaw = min((yaw + k * np.pi for k in (-2, -1, 0, 1, 2)),
              key=lambda y: abs((y - cy + np.pi) % (2 * np.pi) - np.pi))
    x = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    qg = q_of(np.column_stack([x, np.cross(z, x), z]))
    point = np.array([ctr[0], ctr[1], max(top - min(depth, 0.5 * ext_w[2]), TABLE_Z + 0.005)])
    api.open_gripper()
    if not api.move_to((point + [0, 0, 0.10], qg)):
        return False
    api.move_to((point, qg), linear=True, speed="slow", allow_contact=[name])
    if api.last_motion == "refused":
        return False
    api.close_gripper()
    return api.gripper_holding()


def clear_object(api, name, handle, pull):
    """Pick a movable object out of the work volume and put it down on a free table spot.
    Verifies that the object actually rose with the hand and ended up at the spot."""
    st = api.detect(name)
    if st is None:
        return False
    ext_w = np.abs(R_of(st.quaternion)) @ st.bbox_extents
    footprint = float(np.max(ext_w[:2]))
    wide = min(st.bbox_extents[0], st.bbox_extents[1]) > GRIPPER_OPENING - 0.01
    if wide and footprint > 0.15:
        log(name, "is far wider than the gripper: cannot be picked, planning around it")
        IMMOVABLE.add(name)
        return False
    bad_spots = []
    spot = free_spot(api, name, handle, pull, footprint)
    if spot is None:
        log("no free spot for", name)
        return False
    for attempt in range(3):
        before = avg_detect(api, name, n=10)[2]
        if not top_grasp_averaged(api, name, depth=0.05, edge=wide):
            log("could not grasp", name)
            api.open_gripper()
            p, q = api.tcp_pose()
            api.move_to((p + [0, 0, 0.08], q))
            continue
        p, q = api.tcp_pose()
        api.move_to((p + [0, 0, 0.03], q), linear=True, speed="slow")
        obj = avg_detect(api, name, n=10)[2]
        if obj[2] - before[2] < 0.015 and api.gripper_holding() and not wide:
            log(name, "is held but does not lift: treating it as immovable")
            IMMOVABLE.add(name)
            api.open_gripper()
            p, q = api.tcp_pose()
            api.move_to((p + [0, 0, 0.06], q))
            return False
        if obj[2] - before[2] < 0.015 or not api.gripper_holding():
            log("object did not lift with the hand; regrasping")
            api.open_gripper()
            p, q = api.tcp_pose()
            api.move_to((p + [0, 0, 0.06], q))
            continue
        bottom = obj[2] - 0.5 * ext_w[2]
        p, q = api.tcp_pose()
        target = np.array([p[0] + spot[0] - obj[0], p[1] + spot[1] - obj[1],
                           p[2] - (bottom - (TABLE_Z + 0.01))])
        placed = api.move_to((target, q))
        tries = 0
        while not placed and api.last_motion == "refused" and tries < 4:
            log("spot", np.round(spot, 3), "cannot be reached with the object; trying another")
            bad_spots.append(spot)
            spot = free_spot(api, name, handle, pull, footprint, avoid=bad_spots)
            if spot is None:
                break
            p, q = api.tcp_pose()
            target = np.array([p[0] + spot[0] - obj[0], p[1] + spot[1] - obj[1], target[2]])
            placed = api.move_to((target, q))
            tries += 1
        api.open_gripper()
        p, q = api.tcp_pose()
        api.move_to((p + [0, 0, 0.08], q), linear=True, allow_contact=[name])
        if api.last_motion == "refused":
            api.move_to((p + [0, 0, 0.08], q))
        after = avg_detect(api, name, n=10)[2]
        log("moved", name, "to", np.round(after[:2], 3), "(spot", np.round(spot, 3), ")")
        if name not in work_volume_blockers(api, handle, pull):
            return True
    log("giving up on clearing", name, "- planning around it from now on")
    IMMOVABLE.add(name)
    return False


def wait_until_still(api, names, timeout_s=5.0):
    """Hold the arm until the named objects stop moving (averaged track windows agree)."""
    t = 0.0
    while t < timeout_s:
        moving = False
        for n in names:
            hist = [st for _, st in api.track(n, 0.5) if st is not None]
            t += 0.5
            if len(hist) >= 6:
                a = np.mean([h.position for h in hist[:3]], axis=0)
                b = np.mean([h.position for h in hist[-3:]], axis=0)
                if np.linalg.norm(a - b) > 0.02:
                    moving = True
        if not moving:
            return True
        log("waiting for", names, "to come to rest")
    return False


def clear_work_volume(api, handle, pull):
    blockers = work_volume_blockers(api, handle, pull)
    if blockers:
        wait_until_still(api, blockers)
    for name in work_volume_blockers(api, handle, pull):
        log("clearing", name, "from the work volume")
        clear_object(api, name, handle, pull)


def run(api):
    IMMOVABLE.clear()
    handle, pull, pos0 = drawer_frame(api)
    api.open_gripper()
    for attempt in range(5):
        done = travel(api, pos0, pull)
        log("attempt", attempt, "travel", round(done, 3))
        if done >= OPEN_MIN:
            break
        # The scene may have changed since the last look: re-perceive and re-clear every attempt.
        # The work volume is defined from the closed drawer, whatever it is now.
        handle, pull_now, _ = drawer_frame(api)
        clear_work_volume(api, handle - pull_now * max(done, 0.0), pull_now)
        handle, pull_now, _ = drawer_frame(api)
        q = grasp_handle(api, handle, pull_now, handle - pull_now * max(done, 0.0))
        if q is None:
            log("could not grasp the handle")
            continue
        pull_compliant(api, q, pos0, pull)
        api.open_gripper()
        # Retreat the way the hand came in (back along its approach axis), not along the pull.
        p, qn = api.tcp_pose()
        back = p - R_of(qn)[:, 2] * 0.08
        if not api.move_to((back, qn), linear=True, allow_contact=[CAB]):
            api.move_to((back, qn))
    log("final travel", round(travel(api, pos0, pull), 3))
