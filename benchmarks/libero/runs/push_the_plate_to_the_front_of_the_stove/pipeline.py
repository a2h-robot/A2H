"""push_the_plate_to_the_front_of_the_stove: "push the plate to the front of the stove".

Plain pushing does not work on this plate: it weighs ~11 g and its rim is a shallow cone, so
closed fingers pushing from inside ride over the rim, and fingers pushing from outside wedge
under the lip and lift the plate (found by probing). Instead the pipeline pinches the rim lip,
lifts that edge a few cm (the far edge may still drag on the table), moves it with the
collision-aware planner, lowers it and releases it. Closed loop: re-detect after every
placement and repeat while the plate centre is not inside the goal region with a margin.
"""

import sys

import numpy as np

PLATE = "plate_1"
REGION = "main_table_stove_front_region"

N_AVG = 30              # detections averaged per estimate (detections are noisy, independent)
RIM_R = 0.067           # radius of the pinch point on the rim lip [m]
GRASP_DEPTH = 0.005     # grip site this far below the plate top
LIFT = 0.06              # lift of the grasped edge before transport (the far edge may still drag)
MAX_ATTEMPTS = 4
GRIPPER_SPAN = 0.07      # widest object side that fits between the open fingers (opening 0.08 m)


def log(*a):
    print("[pipe]", *a, file=sys.stderr)


# ------------------------------------------------------------------ perception
def detect_avg(api, name, n=N_AVG):
    sts = [api.detect(name) for _ in range(n)]
    sts = [s for s in sts if s is not None]
    if not sts:
        return None
    s = sts[0]
    s.position = np.mean([x.position for x in sts], axis=0)
    s.bbox_center = np.mean([x.bbox_center for x in sts], axis=0)
    return s


def yaw_of(q):
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def obstacle_rects(api, plate_top):
    """2D footprints (center xy, yaw, half extents xy, name) of everything the plate could hit."""
    rects = []
    for b in api.get_obstacles():
        if b.name.startswith("plate"):
            continue
        if b.center[2] - b.extents[2] / 2 > plate_top + 0.01:
            continue
        rects.append((np.asarray(b.center[:2]), yaw_of(b.quaternion), np.asarray(b.extents[:2]) / 2, b.name))
    return rects


def rect_dist(p, rect):
    c, yaw, h, _ = rect
    cy, sy = np.cos(yaw), np.sin(yaw)
    d = p - c
    local = np.array([cy * d[0] + sy * d[1], -sy * d[0] + cy * d[1]])
    q = np.abs(local) - h
    return float(np.linalg.norm(np.maximum(q, 0.0)) + min(max(q[0], q[1]), 0.0))


def clearance(p, rects, radius):
    if not rects:
        return 1.0
    return min(rect_dist(p, r) for r in rects) - radius


def choose_target(region, rects, radius, margin=0.015):
    """Point inside the region (shrunk by margin) near its centre with clearance from objects."""
    c = region.position[:2]
    h = region.bbox_extents[:2] / 2 - margin
    best, best_score = c.copy(), -1e9
    for dx in np.linspace(-h[0], h[0], 11):
        for dy in np.linspace(-h[1], h[1], 11):
            p = c + [dx, dy]
            cl = clearance(p, rects, radius)
            score = min(cl, 0.015) * 10 - np.linalg.norm([dx, dy])
            if score > best_score:
                best, best_score = p, score
    return best


def in_target(p, region, margin=0.008):
    h = region.bbox_extents[:2] / 2 - margin
    return bool(np.all(np.abs(p - region.position[:2]) <= h))


def rot_to_quat(R):
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def push_quat(api, d):
    """Top-down gripper whose y axis (narrow side of the closed fingers) points along d;
    of the two equivalent yaws, the one closest to the current hand orientation."""
    z = np.array([0.0, 0.0, -1.0])
    y = np.array([d[0], d[1], 0.0]) / (np.linalg.norm(d) + 1e-9)
    _, qc = api.tcp_pose()
    best = None
    for sgn in (1, -1):
        yy = y * sgn
        q = rot_to_quat(np.column_stack([np.cross(yy, z), yy, z]))
        dot = abs(float(np.dot(q, qc)))
        if best is None or dot > best[0]:
            best = (dot, q)
    return best[1]


# ------------------------------------------------------------------ motion
def retreat(api, dz=0.08):
    p, q = api.tcp_pose()
    api.move_to((p + [0, 0, dz], q), linear=True, allow_contact=[PLATE])
    if api.last_motion == "refused":
        api.move_to((p + [0, 0, dz], q))


def plate_yaw(api, n=N_AVG):
    """Averaged yaw of the plate (the rim has 8-fold symmetry, so average on 8*yaw)."""
    v = np.zeros(2)
    for _ in range(n):
        y8 = 8 * yaw_of(api.detect(PLATE).quaternion)
        v += [np.cos(y8), np.sin(y8)]
    return np.arctan2(v[1], v[0]) / 8


def rim_angles(api, direction):
    """Pinch angles to try. The rim is made of 8 flat segments, one every 45 deg in the plate
    frame; pinching the middle of a segment holds, pinching a junction between two segments
    does not. The lip pinch acts as a hinge about the rim tangent, so the plate swings (and
    slips out) when carried across that hinge: prefer segments whose tangent is along the
    carry direction."""
    a0 = np.arctan2(direction[1], direction[0])
    y = plate_yaw(api)
    cands = [y + k * np.pi / 4 for k in range(8)]
    return sorted(cands, key=lambda th: abs(np.cos(th - a0)))


def rot_to_quat(R):
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def push_quat(api, d):
    """Top-down gripper whose y axis (narrow side of the closed fingers) points along d;
    of the two equivalent yaws, the one closest to the current hand orientation."""
    z = np.array([0.0, 0.0, -1.0])
    y = np.array([d[0], d[1], 0.0]) / (np.linalg.norm(d) + 1e-9)
    _, qc = api.tcp_pose()
    best = None
    for sgn in (1, -1):
        yy = y * sgn
        q = rot_to_quat(np.column_stack([np.cross(yy, z), yy, z]))
        dot = abs(float(np.dot(q, qc)))
        if best is None or dot > best[0]:
            best = (dot, q)
    return best[1]


# ------------------------------------------------------------------ motion
def retreat(api, dz=0.08):
    p, q = api.tcp_pose()
    api.move_to((p + [0, 0, dz], q), linear=True, allow_contact=[PLATE])
    if api.last_motion == "refused":
        api.move_to((p + [0, 0, dz], q))


def top_quat(yaw):
    x = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    return rot_to_quat(np.column_stack([x, np.cross(z, x), z]))


def pinch(api, plate, th):
    """Top-down pinch of the rim lip at angle th, fingers closing radially. Uses the averaged
    detection (api.grasp() would use a single detection with ~1 cm noise)."""
    c = plate.bbox_center
    top = c[2] + plate.bbox_extents[2] / 2
    point = np.array([c[0] + RIM_R * np.cos(th), c[1] + RIM_R * np.sin(th), top - GRASP_DEPTH])
    # of the two equivalent yaws pick the one closest to the current hand yaw
    _, qc = api.tcp_pose()
    q = max((top_quat(th), top_quat(th + np.pi)), key=lambda qq: abs(float(np.dot(qq, qc))))
    api.open_gripper()
    if not api.move_to((point + [0, 0, 0.08], q)):
        return False
    api.move_to((point, q), linear=True, speed="slow", allow_contact=[PLATE])
    if api.last_motion == "refused":
        return False
    api.close_gripper()
    return api.gripper_holding()


def grasp_rim(api, plate, direction):
    for th in rim_angles(api, direction):
        ok = pinch(api, plate, th)
        if ok:
            log("rim grasp at angle", round(float(np.degrees(th))), "carry dir", round(float(np.degrees(np.arctan2(direction[1], direction[0])))))
            return True
        if api.last_motion != "refused":   # moved and failed: back off before the next try
            api.open_gripper()
            retreat(api)
    return False


def plate_axis(api, n=N_AVG):
    """Averaged plate z axis (its normal) from n noisy detections."""
    zs = []
    for _ in range(n):
        st = api.detect(PLATE)
        w, x, y, z = st.quaternion
        zs.append([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
    a = np.mean(zs, axis=0)
    return a / np.linalg.norm(a)


def predicted_landing(api, radius):
    """Where the centre of the hanging plate lands when released: it falls flat about its
    lowest edge, so the centre moves towards the pinch by radius * (1 - cos(tilt))."""
    held = detect_avg(api, PLATE)
    p, _ = api.tcp_pose()
    u = held.position[:2] - p[:2]
    u = u / (np.linalg.norm(u) + 1e-9)
    tilt = float(np.arccos(np.clip(plate_axis(api)[2], -1, 1)))
    return held.position[:2] - u * radius * (1 - np.cos(tilt)), tilt, held.position


def lifted(api, rest_z):
    """The plate hangs from the gripper (gripper_holding() flickers on this thin lip,
    so check that the plate is off the table instead)."""
    return detect_avg(api, PLATE).position[2] > rest_z + 0.008


def try_move(api, p, q):
    if api.move_to((p, q), speed="slow"):
        return True
    return api.move_to((p + [0, 0, 0.03], q), speed="slow")


def transfer(api, target, radius, region=None):
    plate = detect_avg(api, PLATE)
    for _ in range(4):
        if grasp_rim(api, plate, target - plate.position[:2]):
            break
        log("no rim grasp")
        if region is None or not declutter(api, plate, region, radius, max_objects=1):
            return False
        plate = detect_avg(api, PLATE)
    else:
        return False
    rest_z = plate.position[2]
    p, q = api.tcp_pose()
    grip_z = p[2]
    if not api.move_to((p + [0, 0, LIFT], q), speed="slow"):
        log("lift refused")
    # carry: aim the predicted landing point at the target, re-measure, correct
    for k in range(4):
        if not lifted(api, rest_z):
            log("dropped while carrying")
            api.open_gripper()
            retreat(api)
            return False
        land, tilt, hc = predicted_landing(api, radius)
        err = target - land
        log("carry", k, "hang", np.round(hc, 3), "tilt", round(np.degrees(tilt)), "land", np.round(land, 3), "err", np.round(err, 3))
        if np.linalg.norm(err) < 0.005:
            break
        p, q = api.tcp_pose()
        if not try_move(api, np.r_[p[:2] + err, p[2]], q):
            log("carry move refused")
        # the plate must have followed the hand (it can also rest on something, still "lifted")
        moved_hand = np.linalg.norm(api.tcp_pose()[0][:2] - p[:2])
        moved_plate = np.linalg.norm(detect_avg(api, PLATE).position[:2] - hc[:2])
        if moved_hand > 0.03 and moved_plate < 0.5 * moved_hand:
            log("plate did not follow the hand: dropped")
            api.open_gripper()
            retreat(api)
            return False
    # lower as far as the planner allows (the hanging plate's low edge reaches the table)
    p, q = api.tcp_pose()
    for dz in np.arange(0.01, 0.12, 0.01):
        z = p[2] - dz
        if z < grip_z or not api.move_to((np.r_[p[:2], z], q), speed="slow"):
            break
    if lifted(api, rest_z):
        land, tilt, hc = predicted_landing(api, radius)
        err = target - land
        log("low", "tilt", round(np.degrees(tilt)), "land", np.round(land, 3), "err", np.round(err, 3))
        if np.linalg.norm(err) > 0.005:
            p, q = api.tcp_pose()
            api.move_to((np.r_[p[:2] + err, p[2]], q), speed="slow")
    api.open_gripper()
    api.wait_until(lambda: False, 0.5)
    retreat(api)
    log("released, plate", np.round(detect_avg(api, PLATE).position, 3))
    return True


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def free_spot(api, name, plate_xy, target, radius, rects, region):
    """A table spot for a blocking object: clear of everything else, of the goal and of the
    plate's straight carry corridor, close to where the object is now (reachability)."""
    st = detect_avg(api, name)
    r_obj = 0.5 * float(np.hypot(st.bbox_extents[0], st.bbox_extents[1]))
    others = [r for r in rects if not r[3].startswith(name)]
    best, best_cost = None, 1e9
    for x in np.arange(-0.20, 0.16, 0.02):
        for y in np.arange(-0.10, 0.42, 0.02):
            p = np.array([x, y])
            if clearance(p, others, r_obj) < 0.03:
                continue
            # away from every point the plate may be put on (the whole goal region)
            if np.linalg.norm(p - region.position[:2]) < radius + r_obj + 0.07:
                continue
            if seg_dist(p, plate_xy, target) < radius + r_obj + 0.03:
                continue      # also keeps it away from the plate itself (segment start)
            if np.hypot(x + 0.66, y) > 0.72:       # stay well inside the arm's reach
                continue
            cost = np.linalg.norm(p - st.position[:2])
            if cost < best_cost:
                best, best_cost = p, cost
    return best, st


def push_away(api, name, region, plate_xy, radius, rects, relax=1.0):
    """Push a blocking object (too wide to grasp) out of the goal region with the closed
    fingers: pick the push direction whose end position is clear of other objects, of the
    goal region and of the plate's carry corridor, and whose start pose is reachable."""
    st = detect_avg(api, name)
    c = st.bbox_center[:2]
    yaw = yaw_of(st.quaternion)
    R2 = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    h = st.bbox_extents[:2] / 2
    r_obj = float(np.linalg.norm(h))
    bottom = st.bbox_center[2] - st.bbox_extents[2] / 2
    z = max(0.925, bottom + 0.5 * st.bbox_extents[2])
    others = [r for r in rects if not r[3].startswith(name)]
    need = radius + r_obj + 0.03
    cands = []
    for ang in np.radians(np.arange(0, 360, 22.5)):
        d = np.array([np.cos(ang), np.sin(ang)])
        # travel until the object centre is `need` away from the region centre
        rc = c - region.position[:2]
        bq = np.dot(rc, d)
        L = -bq + np.sqrt(max(bq * bq - (np.dot(rc, rc) - need ** 2), 0.0))
        end = c + d * L
        if clearance(end, others, r_obj) < 0.02 or seg_dist(end, plate_xy, region.position[:2]) < (radius + r_obj) * relax:
            continue
        reach_d = abs(np.dot(R2.T @ d, [1, 0])) * h[0] + abs(np.dot(R2.T @ d, [0, 1])) * h[1]
        start = c - d * (reach_d + 0.035)
        fin_xy = c + d * (L - reach_d + 0.01)              # finger position at the end of the push
        if clearance(start, others, 0.02) < 0.0 or np.hypot(fin_xy[0] + 0.66, fin_xy[1]) > 0.75 + 0.05 * (1 - relax) / 0.4:
            continue
        corridor = seg_dist(end, plate_xy, region.position[:2])
        cands.append((L - min(corridor, 0.30), d, start, end, reach_d, L))
    cands.sort(key=lambda x: x[0])   # short pushes that end far from the plate's carry corridor
    for _, d, start, end, reach_d, L in cands:
        q = top_quat(np.arctan2(d[1], d[0]) + np.pi / 2)   # wide finger face against the object
        api.close_gripper()
        if not api.move_to((np.r_[start, z + 0.10], q)):
            continue
        api.move_to((np.r_[start, z], q), linear=True, speed="slow", allow_contact=[name])
        if api.last_motion == "refused":
            retreat(api)
            continue
        fin = start + d * (L + 0.035 + 0.01)
        api.move_to((np.r_[fin, z], q), linear=True, speed="slow", allow_contact=[name])
        log("pushed", name, "dir", round(float(np.degrees(np.arctan2(d[1], d[0])))), api.last_motion)
        retreat(api)
        return True
    if relax > 0.6:
        log("no push direction for", name, "- relaxing the corridor and reach limits")
        return push_away(api, name, region, plate_xy, radius, rects, relax=0.6)
    log("no push direction for", name)
    return False


def clear_goal(api, region, plate, radius):
    """If no point of the goal region leaves room for the plate, move the movable object that
    blocks it out of the way (grasp, lift, place on a free spot). Returns True if it moved one."""
    top = plate.bbox_center[2] + plate.bbox_extents[2] / 2
    rects = obstacle_rects(api, top)
    target = choose_target(region, rects, radius)
    if clearance(target, rects, radius) > -0.005:
        return False
    names = set(api.objects())
    blockers = sorted(((rect_dist(target, r), r[3]) for r in rects), key=lambda x: x[0])
    for _, name in blockers:
        if name not in names or name == PLATE:
            continue          # a part of a fixture, or the plate itself
        spot, st = free_spot(api, name, plate.position[:2], target, radius, rects, region)
        if spot is None and min(st.bbox_extents[0], st.bbox_extents[1]) <= GRIPPER_SPAN:
            log("no free spot for", name)
            return False
        if min(st.bbox_extents[0], st.bbox_extents[1]) > GRIPPER_SPAN:
            log("goal blocked by", name, "(too wide to grasp) -> pushing it away")
            moved = push_away(api, name, region, plate.position[:2], radius, rects)
            return moved and np.linalg.norm(detect_avg(api, name).position[:2] - st.position[:2]) > 0.03
        log("goal blocked by", name, "-> moving it to", np.round(spot, 3))
        if not pick_and_place(api, name, st, spot):
            log("could not grasp", name, "-> pushing it away")
            return push_away(api, name, region, plate.position[:2], radius, rects)
        return True
    return False


def pick_and_place(api, name, st, spot):
    """Grasp a small movable object, put it down at spot. True if it moved."""
    if not api.grasp(name, strategy="auto"):
        api.open_gripper()
        retreat(api)
        return False
    p, q = api.tcp_pose()
    h = p[2] - (st.bbox_center[2] - st.bbox_extents[2] / 2)   # grip height above its base
    api.move_to((p + [0, 0, 0.10], q))
    api.place((np.r_[spot, st.bbox_center[2] - st.bbox_extents[2] / 2 + h + 0.01], q))
    retreat(api)
    moved = np.linalg.norm(detect_avg(api, name).position[:2] - st.position[:2])
    log("moved", name, "by", round(float(moved), 3))
    return moved > 0.03


def declutter(api, plate, region, radius, max_objects=4):
    """Move small movable objects that stand within a finger's reach of the plate rim or in
    the plate's carry corridor (they block the rim pinch and the dangling plate lands on
    them) to free spots. Returns the number of objects moved."""
    n_moved = 0
    tried = set()
    for _ in range(max_objects):
        top = plate.bbox_center[2] + plate.bbox_extents[2] / 2
        rects = obstacle_rects(api, top + 0.2)
        names = set(api.objects())
        c = plate.position[:2]
        target = choose_target(region, rects, radius)
        near = []
        for r in rects:
            if r[3] not in names or r[3] == PLATE or r[3] in tried:
                continue
            gap = rect_dist(c, r) - radius
            corridor = seg_dist(r[0], c, target) - radius - float(np.linalg.norm(r[2]))
            if gap < 0.05 or corridor < 0.02:
                near.append((min(gap, corridor), r[3]))
        moved = False
        for gap, name in sorted(near):
            tried.add(name)
            st = detect_avg(api, name)
            if min(st.bbox_extents[0], st.bbox_extents[1]) > GRIPPER_SPAN or st.bbox_extents[2] < 0.03:
                continue      # too wide or too flat to pick up
            spot, _ = free_spot(api, name, c, target, radius, rects, region)
            if spot is None:
                continue
            log("clutter: moving", name, "(gap %.3f) to" % gap, np.round(spot, 3))
            if pick_and_place(api, name, st, spot):
                n_moved += 1
                moved = True
                break
        if not moved:
            break
        plate = detect_avg(api, PLATE)
    return n_moved


def run(api):
    region = detect_avg(api, REGION)
    pl = detect_avg(api, PLATE)
    radius = 0.5 * max(pl.bbox_extents[0], pl.bbox_extents[1])
    for _ in range(2):
        if not clear_goal(api, region, pl, radius):
            break
    declutter(api, pl, region, radius)
    for it in range(MAX_ATTEMPTS):
        pl = detect_avg(api, PLATE)
        c = pl.position[:2]
        if in_target(c, region):
            log("plate in target", np.round(c, 3))
            break
        radius = 0.5 * max(pl.bbox_extents[0], pl.bbox_extents[1])
        top = pl.bbox_center[2] + pl.bbox_extents[2] / 2
        target = choose_target(region, obstacle_rects(api, top), radius)
        log("attempt", it, "plate", np.round(c, 3), "target", np.round(target, 3))
        transfer(api, target, radius, region)
    p, q = api.tcp_pose()
    api.move_to((np.r_[p[:2], max(p[2], 1.05)], q))
