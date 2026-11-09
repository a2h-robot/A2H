"""put_cup_on_coaster: pick up the cup and put it on the coaster."""

import numpy as np
import transforms3d as t3d

from api import gripper_quat

N_DET = 8            # detections averaged per estimate (each detection has ~1 cm / 3 deg noise)
LIFT = 0.08          # m to lift the cup after grasping
DROP_MARGIN = 0.002  # m between the cup bottom and the surface at release (a higher drop tips the cup)
PLACE_TOL = 0.012    # m: re-place if the cup ends further than this from the coaster centre
HOVER = 0.05         # m above the release height where the cup is aligned over the target
ALIGN_TOL = 0.004    # m of xy error accepted before lowering
RETREAT = 0.08       # m to back out along the approach after release (straight up drags the rim)
PITCHES = (45, 60)   # deg below horizontal of the grasp approach (a level side grasp is unreachable at table height)
YAWS = (0.0, 0.4, -0.4, 0.8, -0.8, np.pi / 2, -np.pi / 2)        # rad around the line from the hand to the cup
CARRY_YAWS = (0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.6, -1.6)  # hand rotations tried to carry the cup
GRIPPER_SPAN = 0.09  # m, widest opening of the gripper
BODY_CLEARANCE = 0.025  # m of opening beyond the cup width needed for a body grasp (fingers + noise)
RIM_DEPTH = 0.02     # m past the rim contact point, so the finger pads overlap the wall
RIM_BELOW = 0.015    # m below the rim top for the grasp point of a manual rim pinch
RIM_OPEN = 0.5       # gripper opening for a rim pinch (a full opening puts the inner finger on the far wall)
RIM_PITCHES = (70, 55, 85)                  # deg below horizontal for a manual rim pinch
RIM_AZIMUTHS = (0.0, 0.35, -0.35, 0.7, -0.7)  # rad around the cup axis from the side facing the hand
CLUTTER_GAP = 0.07   # m: a movable object closer than this to the cup's side is cleared away before the grasp
MOVABLE_MAX = 0.20   # m: objects larger than this are treated as furniture (not moved)
MAX_CLEAR = 3        # objects moved at most, one at a time between grasp attempts
CLEAR_LIFT = 0.06    # m a cleared object is lifted before it is carried aside
FREE_GAP = 0.13      # m: a set-down spot keeps this distance from the cup, the coaster and other objects
KEEP = ("cup", "coaster")
MOVED = 0.015        # m: the cup counts as moved if two averaged estimates differ by more than this
SETTLE_DT = 0.5      # s between the two estimates of a stillness check
SETTLE_MAX = 6.0     # s to wait for a moving cup to come to rest


UPRIGHT_COS = np.cos(np.deg2rad(30))   # the cup counts as upright if its axis is within 30 deg of vertical
FINGER_REACH = 0.036 # m the fingertips extend past the grasp point along the approach
TABLE_CLEAR = 0.012  # m kept between the fingertips and the table on a grasp of a lying cup (closer is refused)
LYING_COS = np.cos(np.deg2rad(60))      # the cup counts as lying if its axis is within 30 deg of horizontal
FLIP_TILTS = (0.0, 0.35, -0.35, 0.7, -0.7)   # rad of approach tilt about the closing axis for grasping a lying cup
MAX_REFUSED = 4      # refused final approaches per arm before a lying-cup grasp gives up on that arm


PLACED = "placed"    # returned by stand_up / pick when the cup already went straight onto the coaster


class CupMoved(Exception):
    """The cup is no longer where the current grasp was planned."""


def estimate(api, name, n=N_DET):
    """Average n noisy detections of `name` (position only; the first orientation is kept)."""
    ss = [s for s in (api.detect(name) for _ in range(n)) if s is not None]
    if not ss:
        return None
    s = ss[0]
    s.position = np.mean([x.position for x in ss], axis=0)
    return s


def world_height(s):
    """Vertical extent [m] of a detected object's bounding box."""
    R = t3d.quaternions.quat2mat(s.quaternion)
    return float(np.sum(np.abs(R[2, :]) * s.bbox_extents))


def bottom_centre(api, name="cup", n=N_DET):
    """Averaged bottom centre of an object's bounding box (the cup may hang tilted in the gripper)."""
    pts = []
    for _ in range(n):
        s = api.detect(name)
        if s is None:
            continue
        R = t3d.quaternions.quat2mat(s.quaternion)
        up = R[:, int(np.argmax(np.abs(R[2, :])))]          # object axis closest to vertical
        up = up if up[2] > 0 else -up
        pts.append(s.position - up * world_height(s) / 2)
    return np.mean(pts, axis=0)


def horizontal_width(s):
    """Largest horizontal extent [m] of a detected object's bounding box."""
    R = t3d.quaternions.quat2mat(s.quaternion)
    ext = np.abs(R[:2, :]) @ s.bbox_extents
    return float(max(ext))


def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def open_to(api, arm, opening=1.0):
    """Open the gripper to `opening` only if it is not there already (each gripper motion costs ~1 s)."""
    if abs(api.gripper_opening(arm) - opening) > 0.05:
        api.open_gripper(arm, opening)


def other(arm):
    return "left" if arm == "right" else "right"


# ------------------------------------------------------------------ grasp
def wait_still(api, name="cup"):
    """Wait until two averaged estimates SETTLE_DT apart agree within MOVED; return the last estimate."""
    prev = estimate(api, name)
    t0 = api.sim_time()
    while api.sim_time() - t0 < SETTLE_MAX:
        api.wait_until(lambda: False, timeout_s=SETTLE_DT)
        cur = estimate(api, name)
        if np.linalg.norm(cur.position[:2] - prev.position[:2]) < MOVED:
            return cur
        prev = cur
    return prev


CUP_AXIS = 1         # the detector's object frame has its y axis along the cup, from the base to the rim


def cup_axis(cup):
    """(unit axis of the cup from its base to its rim, height). Taken from the detected frame: box extents cannot
    tell a squat cup's axis from a diameter."""
    R = t3d.quaternions.quat2mat(cup.quaternion)
    return R[:, CUP_AXIS], float(cup.bbox_extents[CUP_AXIS])


def is_lying(cup):
    return abs(cup_axis(cup)[0][2]) < LYING_COS


def is_standing(cup):
    """The cup stands on its base or on its rim (either way its footprint centre is under its axis)."""
    return abs(cup_axis(cup)[0][2]) > UPRIGHT_COS


def stand_up(api, cup, arms, table_z, crowd=(), onto=None):
    """See _stand_up; if the cup moves during an approach, wait until it is still and plan once more."""
    for _ in range(2):
        try:
            return _stand_up(api, cup, arms, table_z, crowd, onto)
        except CupMoved:
            for a in arms:
                open_to(api, a)
            cup = wait_still(api)
            if is_standing(cup):
                return True
    return False


def _stand_up(api, cup, arms, table_z, crowd=(), onto=None):
    """Stand a lying cup up: grasp it from above across its middle, lift it, turn the hand a quarter turn about
    the fingers' closing axis (in whichever direction keeps the hand reachable; the cup may end on its base or on
    its rim), then set it down: straight onto `onto` = (xy, surface height) if given and reachable (returns
    PLACED), else where it was (returns True). Returns False if it could not be stood up."""
    a, h = cup_axis(cup)
    ah = np.r_[a[:2], 0.0]
    ah /= np.linalg.norm(ah) + 1e-9
    close = np.cross([0, 0, 1.0], ah)                        # fingers close across the cup's axis
    cands = []
    for tilt in FLIP_TILTS:                                  # approach tilted about the closing axis
        R_t = t3d.axangles.axangle2mat(close, tilt)
        d = R_t @ np.array([0, 0, -1.0])
        for shift in (0.0, 0.015, -0.015):                   # along the cup
            g = np.r_[cup.position[:2] + shift * ah[:2], 0.0]
            g[2] = table_z + TABLE_CLEAR - FINGER_REACH * d[2]  # fingertips just above the table
            cands.append((tilt, g, d, gripper_quat(d, close)))
    for arm in arms:
        grasped, refused, dead_tilts = False, 0, set()
        for tilt, g, d, q in cands:
            if tilt in dead_tilts or refused >= MAX_REFUSED:
                continue
            open_to(api, arm)
            if not api.move_to((g - 0.10 * d, q), arm=arm):
                continue
            check_unmoved(api, cup)
            if not api.move_to((g, q), arm=arm, linear=True, allow_contact=["cup", *crowd]):
                api.move_to((g - 0.10 * d, q), arm=arm, linear=True)
                refused += 1
                dead_tilts.add(tilt)                         # shifting along the cup rarely fixes a refused descent
                continue
            api.close_gripper(arm)
            if api.gripper_holding(arm):
                grasped = True
                break
            open_to(api, arm)
            api.move_to((g - 0.10 * d, q), arm=arm, linear=True)
        if not grasped:
            api.home(arm)
            continue
        api.move_by(dz=0.10, arm=arm)
        centre = cup.position + [0, 0, 0.10]
        t_now, q_now = api.tcp_pose(arm)
        for sgn in (-1.0, 1.0):
            R = t3d.axangles.axangle2mat(close, sgn * np.pi / 2)
            d_f = R @ t3d.quaternions.quat2mat(q_now)[:, 0]
            if not api.move_to((centre + R @ (t_now - centre), t3d.quaternions.mat2quat(R @ t3d.quaternions.quat2mat(q_now))),
                               arm=arm):
                continue
            if onto is not None and put_down(api, arm, d_f, onto[0], onto[1]):
                return PLACED                                # stood up and carried straight onto the target
            held = estimate(api, "cup")
            api.move_by(dz=(table_z + world_height(held) / 2 + 0.01) - held.position[2], arm=arm, allow_contact=["cup"])
            api.open_gripper(arm)
            p_, q_ = api.tcp_pose(arm)
            api.move_to((p_ - RETREAT * d_f, q_), arm=arm, linear=True, allow_contact=["cup"])
            api.home(arm)
            return True
        # could not turn it: put it back down
        api.move_by(dz=-0.09, arm=arm, allow_contact=["cup"])
        open_to(api, arm)
        api.move_by(dz=0.10, arm=arm)
        api.home(arm)
    return False


def check_unmoved(api, cup):
    """Raise CupMoved if the cup is no longer where this grasp was planned (checked just before committing)."""
    now = estimate(api, "cup")
    if np.linalg.norm(now.position[:2] - cup.position[:2]) > MOVED:
        raise CupMoved()


def grasp_body(api, cup, arm, crowd=()):
    """Grasp the cup's upper body from above at a tilt, fingers closing across it.
    Returns the approach direction if held, else None."""
    tcp = api.tcp_pose(arm)[0]
    yaw0 = np.arctan2(cup.position[1] - tcp[1], cup.position[0] - tcp[0])
    c = cup.position.copy()
    c[2] += 0.25 * world_height(cup)
    for pitch in PITCHES:
        p = np.deg2rad(pitch)
        for dyaw in YAWS:
            y = yaw0 + dyaw
            d = np.array([np.cos(y) * np.cos(p), np.sin(y) * np.cos(p), -np.sin(p)])
            q = gripper_quat(d)
            pre = c - 0.10 * d
            open_to(api, arm)
            if not api.move_to((pre, q), arm=arm):
                continue
            check_unmoved(api, cup)
            if not api.move_to((c, q), arm=arm, linear=True, allow_contact=["cup", *crowd]):
                api.move_to((pre, q), arm=arm, linear=True)
                continue
            api.close_gripper(arm)
            if api.gripper_holding(arm):
                return d
            api.open_gripper(arm)
            api.move_to((pre, q), arm=arm, linear=True)
    return None


def grasp_rim(api, cup, arm, crowd=()):
    """Pinch the cup's rim wall: first with the cup's annotated grasp points (near-vertical), then with a
    tilted pinch of the wall facing the hand (reaches further). Returns the approach direction if held."""
    if api.grasp(cup, arm=arm, depth=RIM_DEPTH, max_tries=3):
        return t3d.quaternions.quat2mat(api.tcp_pose(arm)[1])[:, 0]
    api.open_gripper(arm)
    tcp = api.tcp_pose(arm)[0]
    az0 = np.arctan2(tcp[1] - cup.position[1], tcp[0] - cup.position[0])   # cup axis -> hand
    r = horizontal_width(cup) / 2
    rim_z = cup.position[2] + world_height(cup) / 2 - RIM_BELOW
    for pitch in RIM_PITCHES:
        p = np.deg2rad(pitch)
        for daz in RIM_AZIMUTHS:
            u = np.array([np.cos(az0 + daz), np.sin(az0 + daz), 0.0])     # outward normal of that wall
            g = np.r_[cup.position[:2] + r * u[:2], rim_z]
            d = np.r_[-np.cos(p) * u[:2], -np.sin(p)]                     # towards the cup, downwards
            q = gripper_quat(d, u)                                          # fingers close across the wall
            pre = g - 0.10 * d
            open_to(api, arm, RIM_OPEN)
            if not api.move_to((pre, q), arm=arm):
                continue
            check_unmoved(api, cup)
            if not api.move_to((g, q), arm=arm, linear=True, allow_contact=["cup", *crowd]):
                api.move_to((pre, q), arm=arm, linear=True)
                continue
            api.close_gripper(arm)
            if api.gripper_holding(arm):
                return d
            api.open_gripper(arm, RIM_OPEN)
            api.move_to((pre, q), arm=arm, linear=True)
    return None


def grasp_cup(api, cup, arm, crowd=()):
    """A narrow cup is grasped around its body (tilted), a wide one by its rim; the other way is the fallback.
    If the cup moves during an approach, wait until it is still and plan again from its new position."""
    for _ in range(3):
        try:
            wide = horizontal_width(cup) + BODY_CLEARANCE > GRIPPER_SPAN
            for g in ((grasp_rim, grasp_body) if wide else (grasp_body, grasp_rim)):
                d = g(api, cup, arm, crowd)
                if d is not None:
                    return d
                open_to(api, arm)
            return None
        except CupMoved:
            open_to(api, arm)
            cup = wait_still(api)
    return None


def world_half_xy(b):
    """Half extents [m] along world x and y of an obstacle box."""
    R = t3d.quaternions.quat2mat(b.quaternion)
    return (np.abs(R[:2, :]) @ b.extents) / 2


def neighbours(api, cup):
    """Movable objects whose side is within CLUTTER_GAP of the cup's side: [(gap, Box)], closest first."""
    r = horizontal_width(cup) / 2
    out = []
    for b in api.get_obstacles():
        if b.name in KEEP or max(b.extents) > MOVABLE_MAX:
            continue
        half = world_half_xy(b)
        dx = max(0.0, abs(b.center[0] - cup.position[0]) - half[0] - r)
        dy = max(0.0, abs(b.center[1] - cup.position[1]) - half[1] - r)
        gap = float(np.hypot(dx, dy))
        if gap < CLUTTER_GAP:
            out.append((gap, b))
    return sorted(out, key=lambda t: t[0])


def free_spot(api, near, avoid_pts, table_z, arm):
    """Table spots on the arm's side, at least FREE_GAP from every point in avoid_pts and from every obstacle,
    closest to `near` first."""
    obs = [b.center[:2] for b in api.get_obstacles()]
    sx = 1.0 if arm == "right" else -1.0
    cands = []
    for x in np.arange(-0.35, 0.351, 0.07):
        for y in np.arange(-0.25, 0.101, 0.07):
            p = np.array([x, y])
            if x * sx < -0.05:
                continue
            if any(np.linalg.norm(p - q) < FREE_GAP for q in list(avoid_pts) + obs):
                continue
            cands.append(p)
    return sorted(cands, key=lambda p: np.linalg.norm(p - near[:2]))


def grasp_top(api, b, arm, cup, crowd=()):
    """Top-down grasp of a box-like object packed against the cup: the fingers close across the axis that is
    most perpendicular to the cup direction, and touching the cup on the way down is allowed (the gap is
    narrower than the planner's margin). True if it holds the object."""
    R = t3d.quaternions.quat2mat(b.quaternion)
    to_cup = cup.position[:2] - b.center[:2]
    to_cup = to_cup / (np.linalg.norm(to_cup) + 1e-9)
    axes = sorted([(abs(np.dot(R[:2, i], to_cup)), i) for i in range(3) if abs(R[2, i]) < 0.7])
    top = b.center[2] + float(np.sum(np.abs(R[2, :]) * b.extents)) / 2
    for _, i in axes:
        close_ax = np.r_[R[:2, i], 0.0]
        width = float(b.extents[i])
        if width + 0.01 > GRIPPER_SPAN:
            continue
        q = gripper_quat([0, 0, -1.0], close_ax)
        g = np.r_[b.center[:2], top - 0.025]
        open_to(api, arm, min(1.0, (width + 0.015) / GRIPPER_SPAN))
        if not api.move_to((g + [0, 0, 0.08], q), arm=arm):
            continue
        if not api.move_to((g, q), arm=arm, linear=True, allow_contact=[b.name, "cup", *crowd]):
            api.move_to((g + [0, 0, 0.08], q), arm=arm, linear=True)
            continue
        api.close_gripper(arm)
        if api.held_object(arm) == b.name:
            return True
        open_to(api, arm)
        api.move_to((g + [0, 0, 0.08], q), arm=arm, linear=True)
    return False


def seg_points(a, b, n=5):
    return [a + (b - a) * t for t in np.linspace(0, 1, n)]


def clear_around(api, cup, arm, coaster, table_z, limit=MAX_CLEAR):
    """Move the movable objects crowding the cup out of the way, the ones on the hand's side first."""
    tcp = api.tcp_pose(arm)[0]
    to_hand = (tcp[:2] - cup.position[:2]) / (np.linalg.norm(tcp[:2] - cup.position[:2]) + 1e-9)
    moved, failed = 0, set()
    while moved < limit:
        nb = [t for t in neighbours(api, cup) if t[1].name not in failed]
        if not nb:
            break
        # most in the way first: in the hand's direction from the cup
        nb.sort(key=lambda t: -np.dot((t[1].center[:2] - cup.position[:2]) /
                                        (np.linalg.norm(t[1].center[:2] - cup.position[:2]) + 1e-9), to_hand))
        b = nb[0][1]
        crowd = [t[1].name for t in nb]
        held = None
        for a in (arm, other(arm)):                    # whichever arm reaches this object from above
            if grasp_top(api, b, a, cup, crowd) or api.grasp(b.name, arm=a):
                held = a
                break
            open_to(api, a)
            api.home(a)
        if held is None:
            failed.add(b.name)                         # cannot grasp it: try the next one
            continue
        arm_used, arm = arm, held
        api.move_by(dz=CLEAR_LIFT, arm=arm)
        avoid = seg_points(cup.position[:2], coaster.position[:2])
        tcp, q = api.tcp_pose(arm)
        for spot in free_spot(api, b.center, avoid, table_z, arm)[:6]:
            # carry it aside without turning the hand, lower it back to the table and let go
            if api.move_to((tcp + np.r_[spot - b.center[:2], 0.0], q), arm=arm):
                break
        api.move_by(dz=-(CLEAR_LIFT - 0.005), arm=arm, allow_contact=[b.name])
        open_to(api, arm)
        api.move_by(dz=0.06, arm=arm, allow_contact=[b.name])
        api.home(arm)
        arm = arm_used
        moved += 1
    return moved


def pick(api, arms):
    """Grasp and lift the cup with the first arm in `arms` that can.
    Returns (arm holding it, approach direction), or (None, None)."""
    coaster = estimate(api, "coaster")
    table_z = coaster.position[2] - world_height(coaster) / 2
    for a in arms:
        d = None
        for k in range(MAX_CLEAR + 1):                 # clear one crowding object, try the grasp, repeat
            cup = wait_still(api)
            if neighbours(api, cup) and k < MAX_CLEAR:
                if not clear_around(api, cup, a, coaster, table_z, limit=1) and k > 0:
                    break                              # nothing more can be moved
                cup = wait_still(api)
            crowd = [b.name for _, b in neighbours(api, cup)]   # light objects still within touching distance
            if not is_standing(cup):                   # tipping cup: let it settle; lying cup: stand it up first
                api.wait_until(lambda: False, timeout_s=1.0)
                cup = wait_still(api)
            if is_lying(cup):
                top = coaster.position[2] + world_height(coaster) / 2
                r = stand_up(api, cup, (a, other(a)), table_z, crowd, onto=(coaster.position[:2], top))
                if r == PLACED:
                    return PLACED, None
                if not r:
                    break
                cup = wait_still(api)
                crowd = [b.name for _, b in neighbours(api, cup)]
            d = grasp_cup(api, cup, a, crowd)
            if d is not None or not crowd:
                break
        if d is not None:
            api.move_by(dz=LIFT, arm=a)
            if api.gripper_holding(a):
                return a, d
        api.open_gripper(a)
        api.home(a)
    return None, None


# ------------------------------------------------------------------ place
def carry_and_lower(api, arm, approach, xy, surface_z):
    """Bring the held cup over `xy`, align it, and lower its bottom to just above `surface_z`, rotating the
    hand about the vertical when the current orientation cannot reach. Returns the approach direction at
    release, or None if no orientation worked (the cup is then still held, at hover height)."""
    goal = np.r_[xy, surface_z + DROP_MARGIN + HOVER]
    tcp, q = api.tcp_pose(arm)
    off = bottom_centre(api) - tcp
    for a in CARRY_YAWS:
        R = rotz(a)
        q2 = t3d.quaternions.mat2quat(R @ t3d.quaternions.quat2mat(q))
        if not api.move_to((goal - R @ off, q2), arm=arm):
            continue
        for _ in range(3):                                   # align the cup bottom over the target
            err = goal - bottom_centre(api)
            if np.linalg.norm(err[:2]) < ALIGN_TOL and abs(err[2]) < 0.01:
                break
            api.move_by(*err, arm=arm, linear=False)
        dz = surface_z + DROP_MARGIN - bottom_centre(api)[2]
        if api.move_by(dz=dz, arm=arm, allow_contact=["cup", "coaster"]):
            return R @ approach
        p_, q_ = api.tcp_pose(arm)                           # straight descent refused: one planned move
        dz = surface_z + DROP_MARGIN - bottom_centre(api)[2]
        if api.move_to((p_ + [0, 0, dz], q_), arm=arm, allow_contact=["cup", "coaster"]):
            return R @ approach
        api.move_by(dz=HOVER, arm=arm, linear=False)         # back up and try the next orientation
    return None


def put_down(api, arm, approach, xy, surface_z):
    """Carry the held cup over `xy`, set it on the surface at `surface_z`, release and back out.
    Returns False (cup still held) if the arm cannot get it there."""
    d = carry_and_lower(api, arm, approach, xy, surface_z)
    if d is None:
        return False
    api.open_gripper(arm)
    p, q = api.tcp_pose(arm)
    api.move_to((p - RETREAT * d, q), arm=arm, linear=True, allow_contact=["cup", "coaster"])
    api.home(arm)
    return True


def place_on_coaster(api, arm, approach):
    """Put the held cup on the coaster centre. If this arm cannot reach the coaster, set the cup down halfway
    and hand over through the table to the other arm. Returns True if the cup was released on the coaster."""
    coaster = estimate(api, "coaster")
    top = coaster.position[2] + world_height(coaster) / 2
    if put_down(api, arm, approach, coaster.position[:2], top):
        return True
    table = coaster.position[2] - world_height(coaster) / 2   # the coaster lies on the table
    held_xy = bottom_centre(api)[:2]
    mid = (held_xy + coaster.position[:2]) / 2
    if not put_down(api, arm, approach, mid, table):
        return False
    arm2, approach2 = pick(api, [other(arm)])
    if arm2 is None:
        return False
    return put_down(api, arm2, approach2, estimate(api, "coaster").position[:2], top)


def choose_arm(api, cup, coaster):
    """Arms ordered by how well they reach both objects: the start pose closest to the farther of the two."""
    def cost(a):
        h = api.tcp_pose(a)[0][:2]
        return max(np.linalg.norm(cup.position[:2] - h), np.linalg.norm(coaster.position[:2] - h))
    return sorted(("left", "right"), key=cost)


def run(api):
    cup, coaster = estimate(api, "cup"), estimate(api, "coaster")
    arms = choose_arm(api, cup, coaster)
    for attempt in range(3):
        arm, approach = pick(api, arms)
        if arm is None:
            return
        if arm != PLACED:
            place_on_coaster(api, arm, approach)
        cup, coaster = estimate(api, "cup"), estimate(api, "coaster")
        if np.linalg.norm(cup.position[:2] - coaster.position[:2]) <= PLACE_TOL:
            break
        arms = choose_arm(api, cup, coaster)              # off-centre or set down elsewhere: try again
    for a in ("left", "right"):
        if api.gripper_opening(a) < 0.9:
            api.open_gripper(a)
