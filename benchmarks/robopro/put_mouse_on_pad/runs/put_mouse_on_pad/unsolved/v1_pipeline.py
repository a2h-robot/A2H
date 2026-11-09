"""put_mouse_on_pad: pick up the computer mouse and put it on the mouse pad.

The success check compares the mouse's model origin with the pad centre in x and in y (2 cm each) and needs
both grippers open; the origin lies straight below the mouse's bounding-box centre, so the box centre is aligned
over the pad centre.
"""

import numpy as np
import transforms3d as t3d

from api import gripper_quat

N_DET = 8              # detections averaged per estimate (each detection has ~1 cm / 3 deg noise)
LIFT = 0.06            # m to lift the mouse after grasping
HOVER = 0.04           # m between the mouse bottom and the pad at the alignment step
DROP_MARGIN = 0.003    # m between the mouse bottom and the pad at release
ALIGN_TOL = 0.004      # m of xy error accepted before lowering
RETREAT = 0.06         # m to back out along the approach after release
CARRY_YAWS = (0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.6, -1.6)   # hand rotations tried to carry the mouse
GRIPPER_SPAN = 0.09    # m, widest opening of the gripper
CLUTTER_GAP = 0.05     # m: a movable object closer than this to the mouse is moved away before the grasp
MOVABLE_MAX = 0.20     # m: objects larger than this are treated as furniture (not moved)
MAX_CLEAR = 4          # objects moved at most
CLEAR_LIFT = 0.04      # m a cleared object is lifted before it is carried aside
CLEAR_TRIES = 2        # grasp attempts per crowding object (a finger in a 1.5 cm gap can slip off)
TOP_PRE = 0.05         # m above a crowding object's grasp point where the hand starts its descent
TOP_GRIP = 0.03        # m below an object's top where a top-down grasp closes
TABLE_Z = 0.74         # m, table top
FINGER_REACH = 0.036   # m the fingertips reach past the grasp point
TABLE_CLEAR = 0.006    # m between the fingertips and the table at a top-down grasp
FREE_GAP = 0.13        # m: a set-down spot keeps this distance from the mouse, the pad, the path and other objects
KEEP = ("mouse", "pad")
PUSH_Z = 0.02          # m above the table where the closed fingertips push a crowding object
PUSH_CLEAR = 0.015     # m the pushed object's end ends past the mouse's side


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


def bottom_centre(api, name="mouse", n=N_DET):
    """Averaged bottom centre of an object's bounding box."""
    s = estimate(api, name, n)
    return s.position - [0, 0, world_height(s) / 2]


def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def world_half_xy(b):
    """Half extents [m] along world x and y of an obstacle box or detected object."""
    R = t3d.quaternions.quat2mat(b.quaternion)
    ext = b.extents if hasattr(b, "extents") else b.bbox_extents
    return (np.abs(R[:2, :]) @ ext) / 2


def other(arm):
    return "left" if arm == "right" else "right"


def open_to(api, arm, opening=1.0):
    if abs(api.gripper_opening(arm) - opening) > 0.05:
        api.open_gripper(arm, opening)


def closing_axis(mouse):
    """World direction (horizontal) across the mouse's narrower horizontal side: the fingers of a grasp from above
    close along it and come down on both sides of the mouse along it."""
    R = t3d.quaternions.quat2mat(mouse.quaternion)
    hz = [(mouse.bbox_extents[i], R[:, i]) for i in range(3) if abs(R[2, i]) < 0.7]
    ax = min(hz, key=lambda t: t[0])[1]
    ax = np.r_[ax[:2], 0.0]
    return ax / np.linalg.norm(ax)


def neighbours(api, mouse):
    """Movable objects whose side is within CLUTTER_GAP of the mouse's side: [(gap, Box)], closest first. (The
    grasp from above can come down beside any side of the mouse, so every side counts.)"""
    hm = world_half_xy(mouse)
    out = []
    for b in api.get_obstacles():
        if b.name in KEEP or max(b.extents) > MOVABLE_MAX:
            continue
        d = b.center[:2] - mouse.position[:2]
        half = world_half_xy(b)
        dx = max(0.0, abs(d[0]) - half[0] - hm[0])
        dy = max(0.0, abs(d[1]) - half[1] - hm[1])
        gap = float(np.hypot(dx, dy))
        if gap < CLUTTER_GAP:
            out.append((gap, b))
    return sorted(out, key=lambda t: t[0])


def seg_points(a, b, n=5):
    return [a + (b - a) * t for t in np.linspace(0, 1, n)]


def free_spot(api, near, avoid_pts, arm):
    """Table spots on the arm's side, at least FREE_GAP from every point in avoid_pts and from every obstacle,
    closest to `near` first."""
    obs = [b.center[:2] for b in api.get_obstacles()]
    sx = 1.0 if arm == "right" else -1.0
    cands = []
    for x in np.arange(-0.45, 0.451, 0.05):
        for y in np.arange(-0.25, 0.051, 0.05):
            pt = np.array([x, y])
            if x * sx < 0.05:
                continue
            if any(np.linalg.norm(pt - q) < FREE_GAP for q in list(avoid_pts) + obs):
                continue
            cands.append(pt)
    return sorted(cands, key=lambda pt: np.linalg.norm(pt - near[:2]))


def grasp_top(api, b, arm, mouse, crowd=()):
    """Top-down grasp of a box-like object packed against the mouse: the fingers close across a horizontal axis
    the object fits, the one most perpendicular to the mouse direction first; touching the mouse and the other
    crowding objects on the way down is allowed. True if it holds the object."""
    R = t3d.quaternions.quat2mat(b.quaternion)
    to_m = mouse.position[:2] - b.center[:2]
    to_m = to_m / (np.linalg.norm(to_m) + 1e-9)
    axes = sorted([(abs(np.dot(R[:2, i], to_m)), i) for i in range(3) if abs(R[2, i]) < 0.7])
    top = b.center[2] + float(np.sum(np.abs(R[2, :]) * b.extents)) / 2
    for _, i in axes:
        width = float(b.extents[i])
        if width + 0.01 > GRIPPER_SPAN:
            continue
        q = gripper_quat([0, 0, -1.0], np.r_[R[:2, i], 0.0])
        g = np.r_[b.center[:2], max(top - TOP_GRIP, TABLE_Z + FINGER_REACH + TABLE_CLEAR)]   # fingertips above the table
        open_to(api, arm, min(1.0, (width + 0.015) / GRIPPER_SPAN))
        if not api.move_to((g + [0, 0, TOP_PRE], q), arm=arm):
            continue
        if not api.move_to((g, q), arm=arm, linear=True, allow_contact=[b.name, "mouse", *crowd]):
            api.move_to((g + [0, 0, TOP_PRE], q), arm=arm, linear=True)
            continue
        api.close_gripper(arm)
        if api.held_object(arm) == b.name:
            return True
        open_to(api, arm)
        api.move_to((g + [0, 0, TOP_PRE], q), arm=arm, linear=True)
    return False


def slide_aside(api, b, mouse, arm):
    """Push a crowding object that cannot be lifted along its long horizontal axis, from its end, until it is past
    the mouse (the fingers fit at its end but not in the narrow gap beside the mouse). True if the push ran."""
    R = t3d.quaternions.quat2mat(b.quaternion)
    hz = [i for i in range(3) if abs(R[2, i]) < 0.7]
    i = max(hz, key=lambda k: b.extents[k])
    u = np.r_[R[:2, i], 0.0]
    u /= np.linalg.norm(u)
    half = b.extents[i] / 2
    reach = float(np.max(np.abs(world_half_xy(mouse))))      # mouse extent along any world axis
    for d in (u, -u):
        # the mouse's extent along d, measured from the object's centre line
        along = float(np.dot(mouse.position[:2] - b.center[:2], d[:2]))
        dist = along + reach + half + PUSH_CLEAR               # how far the object's trailing end must travel
        start = np.r_[b.center[:2] - d[:2] * (half + 0.025), TABLE_Z + PUSH_Z + FINGER_REACH]
        q = gripper_quat([0, 0, -1.0], np.cross([0, 0, 1.0], d))
        open_to(api, arm, 0.0)
        if not api.move_to((start + [0, 0, TOP_PRE], q), arm=arm):
            continue
        if not api.move_to((start, q), arm=arm, linear=True, allow_contact=[b.name]):
            api.move_by(dz=TOP_PRE, arm=arm)
            continue
        api.move_to((start + d * dist, q), arm=arm, linear=True, allow_contact=[b.name, "mouse"])
        api.move_by(dz=TOP_PRE, arm=arm, allow_contact=[b.name])
        return True
    return False


def clear_around(api, mouse, pad_xy, arm):
    """Move the movable objects crowding the mouse to free table spots away from the mouse, the pad and the path
    between them, the ones on the hand's side first. Returns the number moved."""
    tcp = api.tcp_pose(arm)[0]
    to_hand = (tcp[:2] - mouse.position[:2]) / (np.linalg.norm(tcp[:2] - mouse.position[:2]) + 1e-9)
    moved, tries, slid = 0, {}, set()
    while moved < MAX_CLEAR:
        nb = [t for t in neighbours(api, mouse) if tries.get(t[1].name, 0) < CLEAR_TRIES]
        if not nb:
            stuck = [t[1] for t in neighbours(api, mouse) if t[1].name not in slid]
            if not stuck:
                break
            slid.add(stuck[0].name)                    # could not lift it: slide it past the mouse instead
            if slide_aside(api, stuck[0], mouse, arm):
                moved += 1
            continue
        # the sides the fingers close on first, then the ones towards the hand
        c = closing_axis(mouse)

        def order(t):
            d = t[1].center[:2] - mouse.position[:2]
            d = d / (np.linalg.norm(d) + 1e-9)
            return (-round(abs(np.dot(d, c[:2]))), -np.dot(d, to_hand))
        nb.sort(key=order)
        b = nb[0][1]
        tries[b.name] = tries.get(b.name, 0) + 1
        crowd = [t[1].name for t in nb]
        held = arm if grasp_top(api, b, arm, mouse, crowd) else None
        if held is None:
            open_to(api, arm)
            continue
        api.move_by(dz=CLEAR_LIFT, arm=held)
        if api.held_object(held) != b.name:            # it slipped out on the lift: try it again from a new estimate
            open_to(api, held)
            api.move_by(dz=0.04, arm=held)
            continue
        avoid = seg_points(mouse.position[:2], np.asarray(pad_xy, float))
        tcp, q = api.tcp_pose(held)
        for spot in free_spot(api, b.center, avoid, held)[:6]:
            if api.move_to((tcp + np.r_[spot - b.center[:2], 0.0], q), arm=held):
                break
        open_to(api, held)            # let it drop the CLEAR_LIFT onto the table (a light box lands upright)
        moved += 1
    return moved


def pick(api, arm):
    """Grasp the mouse from above and lift it. Returns True if it is held."""
    mouse = estimate(api, "mouse")
    for strategy in ("top", "auto"):
        if api.grasp(mouse, arm=arm, strategy=strategy):
            api.move_by(dz=LIFT, arm=arm, allow_contact=["mouse"])
            if api.gripper_holding(arm):
                return True
            api.open_gripper(arm)
        mouse = estimate(api, "mouse")
    return False


def put_on_pad(api, arm, pad_xy, pad_z):
    """Carry the held mouse over the pad centre, align it, lower it to just above the pad, release and back out.
    Rotates the hand about the vertical when the current orientation cannot reach. Returns True if released."""
    goal = np.r_[pad_xy, pad_z + DROP_MARGIN + HOVER]
    tcp, q = api.tcp_pose(arm)
    off = bottom_centre(api) - tcp
    for a in CARRY_YAWS:
        R = rotz(a)
        q2 = t3d.quaternions.mat2quat(R @ t3d.quaternions.quat2mat(q))
        if not api.move_to((goal - R @ off, q2), arm=arm):
            continue
        for _ in range(3):                                   # align the mouse over the pad centre
            err = goal - bottom_centre(api)
            if np.linalg.norm(err[:2]) < ALIGN_TOL and abs(err[2]) < 0.01:
                break
            api.move_by(*err, arm=arm, linear=False)
        dz = pad_z + DROP_MARGIN - bottom_centre(api)[2]
        if not api.move_by(dz=dz, arm=arm, allow_contact=["mouse", "pad"]):
            p_, q_ = api.tcp_pose(arm)                       # straight descent refused: one planned move
            api.move_to((p_ + [0, 0, bottom_centre(api)[2] * 0 + pad_z + DROP_MARGIN - bottom_centre(api)[2]], q_),
                        arm=arm, allow_contact=["mouse", "pad"])
        api.open_gripper(arm)
        p, qq = api.tcp_pose(arm)
        appr = t3d.quaternions.quat2mat(qq)[:, 0]
        api.move_to((p - RETREAT * appr, qq), arm=arm, linear=True, allow_contact=["mouse", "pad"])
        api.home(arm)
        return True
    return False


def run(api):
    pad = estimate(api, "pad")
    pad_z = pad.position[2] + world_height(pad) / 2
    mouse = estimate(api, "mouse")
    arm = "right" if mouse.position[0] > 0 else "left"
    for attempt in range(2):
        mouse = estimate(api, "mouse")
        clear_around(api, mouse, pad.position[:2], arm)      # (again before a retry: a box may have fallen back)
        if not pick(api, arm):
            continue
        if put_on_pad(api, arm, pad.position[:2], pad_z):
            return
        api.open_gripper(arm)
        api.home(arm)
