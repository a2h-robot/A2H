"""put_mouse_on_pad: pick up the computer mouse and put it on the mouse pad.

The success check compares the mouse's model origin with the pad centre in x and in y (2 cm each) and needs
both grippers open; the origin lies straight below the mouse's bounding-box centre, so the box centre is aligned
over the pad centre.
"""

import numpy as np
import transforms3d as t3d

from api import ObjectState, gripper_quat

N_DET = 8              # detections averaged per estimate (each detection has ~1 cm / 3 deg noise)
LIFT = 0.06            # m to lift the mouse after grasping
HOVER = 0.04           # m between the mouse bottom and the pad at the alignment step
DROP_MARGIN = 0.003    # m between the mouse bottom and the pad at release
ALIGN_TOL = 0.004      # m of xy error accepted before lowering
RETREAT = 0.06         # m to back out along the approach after release
CARRY_YAWS = (0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.6, -1.6)   # hand rotations tried to carry the mouse
# when no hand pointing straight down reaches over the pad (something overhangs it), the hand is also tilted about
# its closing axis so that the wrist leans towards the robot; the mouse tilts with it (or pivots back level
# between the fingers) and settles flat when let go, a few mm from where it was aimed
TILTS = (0.35, 0.55)   # rad
TILT_PRESS = 0.012     # m a tilted mouse is lowered past first contact, so that it lies flat on the pad before it is let go
TABLE_Z = 0.74         # m, table top
GRIPPER_SPAN = 0.09    # m, widest opening of the gripper
TOP_GRIP = 0.03        # m below an object's top where a top-down grasp closes
FINGER_REACH = 0.036   # m the fingertips reach past the grasp point
TABLE_CLEAR = 0.006    # m between the fingertips and the table at a top-down grasp
TOP_PRE = 0.05         # m above the grasp point where the hand starts its descent
MOVABLE_MAX = 0.20     # m: objects larger than this are treated as furniture (not moved)
PAD_MARGIN = 0.03      # m around the pad within which a standing object is in the way of the put-down
CLEAR_LIFT = 0.04      # m an object moved off the pad is lifted before it is carried aside (then dropped)
FREE_GAP = 0.13        # m: a set-down spot keeps this distance from the mouse, the pad, the path and other objects
KEEP = ("mouse", "pad")
END_IN = (0.03, 0.02)  # m from the mouse's free end to where it is grasped when its other part is under something
HAND_CLEAR = 0.11      # m between the edge of what overhangs the mouse and the mouse's centre once it is drawn out
                       # (room for the hand to come down at the centre)
TILT_REACH = 0.047     # m below the grasp point the fingertips reach with the hand tilted ~30 deg
LEVEL_Z = 0.036        # m above the table of a level grasp point (the fingers reach 3.1 cm above and below it)
LEVEL_PRE = 0.10       # m in front of a level grasp point where the straight approach starts
MIDDLE = 0.12          # m: a pad within this of the robot's middle (x = 0) is reached by the arm on the mouse's side
RELAY_X = 0.0          # m, x of the hand-over spot on the table (both arms reach the middle)


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


def open_to(api, arm, opening=1.0):
    if abs(api.gripper_opening(arm) - opening) > 0.05:
        api.open_gripper(arm, opening)


def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def grasp_from_above(api, s, arm):
    """Top-down grasp of a detected object across its narrowest horizontal side, fingertips just above the table
    (for when its annotated grasp points are unusable, e.g. the object lies upside down). True if it holds it."""
    R = t3d.quaternions.quat2mat(s.quaternion)
    top = s.position[2] + world_height(s) / 2
    for i in sorted([i for i in range(3) if abs(R[2, i]) < 0.7], key=lambda k: s.bbox_extents[k]):
        width = float(s.bbox_extents[i])
        if width + 0.01 > GRIPPER_SPAN:
            continue
        q = gripper_quat([0, 0, -1.0], np.r_[R[:2, i], 0.0])
        g = np.r_[s.position[:2], max(top - TOP_GRIP, TABLE_Z + FINGER_REACH + TABLE_CLEAR)]
        pre = np.r_[g[:2], max(g[2] + TOP_PRE, top + FINGER_REACH + 0.015)]   # fingertips above its top
        open_to(api, arm, min(1.0, (width + 0.02) / GRIPPER_SPAN))
        if not api.move_to((pre, q), arm=arm):
            continue
        if not api.move_to((g, q), arm=arm, linear=True, allow_contact=[s.name]):
            api.move_to((pre, q), arm=arm, linear=True)
            continue
        api.close_gripper(arm)
        if api.held_object(arm) == s.name:
            return True
        open_to(api, arm)
        api.move_to((pre, q), arm=arm, linear=True)
    return False


def world_half_xy(b):
    """Half extents [m] along world x and y of an obstacle box."""
    R = t3d.quaternions.quat2mat(b.quaternion)
    return (np.abs(R[:2, :]) @ b.extents) / 2


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


def clear_pad(api, pad, mouse, arm):
    """While the hands are free: move every small object standing on the pad (or within PAD_MARGIN of it) to a
    free table spot away from the pad, the mouse and the way between them. Returns the number moved."""
    ph = world_half_xy(type("B", (), {"quaternion": pad.quaternion, "extents": pad.bbox_extents})) + PAD_MARGIN
    moved, tried = 0, set()
    while True:
        on = [b for b in api.get_obstacles() if b.name not in KEEP and b.name not in tried
              and max(b.extents) <= MOVABLE_MAX
              and np.all(np.abs(b.center[:2] - pad.position[:2]) < ph + world_half_xy(b))]
        if not on:
            return moved
        b = on[0]
        tried.add(b.name)
        s = ObjectState(b.name, np.asarray(b.center, float), np.asarray(b.quaternion, float),
                        np.asarray(b.extents, float))
        if not grasp_from_above(api, s, arm):
            open_to(api, arm)
            continue
        api.move_by(dz=CLEAR_LIFT, arm=arm)
        avoid = [pad.position[:2], mouse.position[:2], (pad.position[:2] + mouse.position[:2]) / 2]
        tcp, q = api.tcp_pose(arm)
        for spot in free_spot(api, b.center, avoid, arm)[:6]:
            if api.move_to((tcp + np.r_[spot - b.center[:2], 0.0], q), arm=arm):
                break
        open_to(api, arm)                                 # let it drop onto the table
        api.home(arm)
        moved += 1


def overhangs(api, s):
    """Obstacle boxes whose underside is above the object's top and that overlap its footprint."""
    top = s.position[2] + world_height(s) / 2
    hm = (np.abs(t3d.quaternions.quat2mat(s.quaternion)[:2, :]) @ s.bbox_extents) / 2
    out = []
    for b in api.get_obstacles():
        if b.name in KEEP:
            continue
        R = t3d.quaternions.quat2mat(b.quaternion)
        under = b.center[2] - float(np.sum(np.abs(R[2, :]) * b.extents)) / 2
        if under > top + 0.005 and np.all(np.abs(b.center[:2] - s.position[:2]) < world_half_xy(b) + hm):
            out.append(b)
    return out


def draw_out_level(api, s, arm):
    """The mouse is partly under something: come in level from its open end along its length, fingers on its two
    long sides and low enough to pass under the overhang, close at its centre, slide it out until it is clear and
    lift it. True if it is held afterwards."""
    ov = overhangs(api, s)
    if not ov:
        return False
    b = ov[0]
    R = t3d.quaternions.quat2mat(s.quaternion)
    hz = [i for i in range(3) if abs(R[2, i]) < 0.7]
    i_long = max(hz, key=lambda k: s.bbox_extents[k])
    i_narrow = min(hz, key=lambda k: s.bbox_extents[k])
    u = np.r_[R[:2, i_long], 0.0]
    u /= np.linalg.norm(u)
    d = u if np.dot(s.position[:2] - b.center[:2], u[:2]) > 0 else -u          # from the overhang towards the open
    c = np.r_[R[:2, i_narrow], 0.0]
    c /= np.linalg.norm(c)
    q = gripper_quat(-d, c)                                                   # level, pointing under the overhang
    g = np.r_[s.position[:2], TABLE_Z + LEVEL_Z]
    pre = g + d * LEVEL_PRE
    edge = float(np.dot(b.center[:2], d[:2]) + np.dot(world_half_xy(b), np.abs(d[:2])))
    half = s.bbox_extents[i_long] / 2
    dist = max(0.0, edge - (float(np.dot(s.position[:2], d[:2])) - half)) + 0.03   # far end out past the edge
    open_to(api, arm, min(1.0, (float(s.bbox_extents[i_narrow]) + 0.02) / GRIPPER_SPAN))
    if not api.move_to((pre, q), arm=arm):
        return False
    if not api.move_to((g, q), arm=arm, linear=True, allow_contact=["mouse"]):
        api.move_to((pre, q), arm=arm, linear=True)
        return False
    api.close_gripper(arm)
    if api.held_object(arm) != "mouse":
        open_to(api, arm)
        api.move_to((pre, q), arm=arm, linear=True)
        return False
    api.move_to((g + d * dist + [0, 0, 0.004], q), arm=arm, linear=True, allow_contact=["mouse"])
    api.move_by(dz=LIFT, arm=arm, allow_contact=["mouse"])
    return api.gripper_holding(arm)


def draw_out(api, s, arm):
    """The mouse is partly under something: grasp its exposed end (straight down, else with the wrist leaning away
    from the overhang), slide it out lengthwise along the table until it is clear, and let go (held by one end it
    would slip out of the hand on the way). True if it was drawn out."""
    ov = overhangs(api, s)
    if not ov:
        return False
    b = ov[0]
    R = t3d.quaternions.quat2mat(s.quaternion)
    hz = [i for i in range(3) if abs(R[2, i]) < 0.7]
    i_long = max(hz, key=lambda k: s.bbox_extents[k])
    i_narrow = min(hz, key=lambda k: s.bbox_extents[k])
    u = np.r_[R[:2, i_long], 0.0]
    u /= np.linalg.norm(u)
    d = u if np.dot(s.position[:2] - b.center[:2], u[:2]) > 0 else -u          # from the overhang towards the open
    c = np.r_[R[:2, i_narrow], 0.0]
    c /= np.linalg.norm(c)
    half = s.bbox_extents[i_long] / 2
    top = s.position[2] + world_height(s) / 2
    # how far the mouse must travel along d for its centre to be HAND_CLEAR past the overhang's edge
    edge = float(np.dot(b.center[:2], d[:2]) + np.dot(world_half_xy(b), np.abs(d[:2])))
    dist = max(0.0, edge + HAND_CLEAR - float(np.dot(s.position[:2], d[:2])))
    width = float(s.bbox_extents[i_narrow])
    for tilt, end_in in [(t, e_) for e_ in END_IN for t in (0.35, 0.55, 0.0)]:   # wrist leaning away first
        e = s.position + d * (half - end_in)
        for sgn in (1.0, -1.0):
            a = t3d.axangles.axangle2mat(c, sgn * tilt) @ np.array([0, 0, -1.0])
            if tilt == 0.0 or np.dot(a[:2], d[:2]) < 0:                     # fingers towards the overhang, wrist away
                break
        q = gripper_quat(a, c)
        reach = FINGER_REACH if tilt == 0.0 else TILT_REACH
        g = np.r_[e[:2], max(top - TOP_GRIP, TABLE_Z + reach + TABLE_CLEAR)]
        pre = g - a * 0.06
        open_to(api, arm, min(1.0, (width + 0.02) / GRIPPER_SPAN))
        if not api.move_to((pre, q), arm=arm):
            continue
        if not api.move_to((g, q), arm=arm, linear=True, allow_contact=["mouse"]):
            api.move_to((pre, q), arm=arm, linear=True)
            continue
        api.close_gripper(arm)
        if api.held_object(arm) != "mouse":
            open_to(api, arm)
            api.move_to((pre, q), arm=arm, linear=True)
            continue
        api.move_to((g + d * dist + [0, 0, 0.004], q), arm=arm, linear=True, allow_contact=["mouse"])
        open_to(api, arm)
        api.move_to((g + d * dist - a * 0.06, q), arm=arm, linear=True, allow_contact=["mouse"])
        return True
    return False


def pick(api, arm):
    """Grasp the mouse from above and lift it. Returns True if it is held."""
    mouse = estimate(api, "mouse")
    if overhangs(api, mouse):                                 # part of it is under something: draw it out first
        if draw_out_level(api, mouse, arm):
            return True
        if draw_out(api, estimate(api, "mouse"), arm):
            mouse = estimate(api, "mouse")
    for strategy in ("top", "auto"):
        if api.grasp(mouse, arm=arm, strategy=strategy):
            api.move_by(dz=LIFT, arm=arm, allow_contact=["mouse"])
            if api.gripper_holding(arm):
                return True
            api.open_gripper(arm)
        mouse = estimate(api, "mouse")
    # the mouse's annotated grasp points are unusable (they face the table when it lies upside down): take its box
    if grasp_from_above(api, mouse, arm):
        api.move_by(dz=LIFT, arm=arm, allow_contact=["mouse"])
        if api.gripper_holding(arm):
            return True
        api.open_gripper(arm)
    # part of the mouse is under something (a riser, a shelf): draw it out by its exposed end, then take it normally
    if draw_out(api, estimate(api, "mouse"), arm):
        mouse = estimate(api, "mouse")
        if api.grasp(mouse, arm=arm, strategy="top") or grasp_from_above(api, mouse, arm):
            api.move_by(dz=LIFT, arm=arm, allow_contact=["mouse"])
            if api.gripper_holding(arm):
                return True
            api.open_gripper(arm)
    return False


def side(x):
    return "right" if x > 0 else "left"


def put_on_pad(api, arm, pad_xy, pad_z):
    """Carry the held mouse over the pad centre, align it, lower it to just above the pad, release and back out.
    Rotates the hand about the vertical when the current orientation cannot reach. Returns True if released."""
    goal = np.r_[pad_xy, pad_z + DROP_MARGIN + HOVER]
    tcp, q = api.tcp_pose(arm)
    off = bottom_centre(api) - tcp
    Rq = t3d.quaternions.quat2mat(q)
    turns = [rotz(a) for a in CARRY_YAWS]
    for tilt in TILTS:
        for a in CARRY_YAWS:
            c = rotz(a) @ Rq[:, 1]                           # closing axis after the yaw
            for sgn in (1.0, -1.0):
                M = t3d.axangles.axangle2mat(c, sgn * tilt) @ rotz(a)
                if (M @ Rq[:, 0])[1] > 0.1:                  # approach leans away from the robot: wrist towards it
                    turns.append(M)
    for k, R in enumerate(turns):
        tilted = k >= len(CARRY_YAWS)
        q2 = t3d.quaternions.mat2quat(R @ Rq)
        if not api.move_to((goal - R @ off, q2), arm=arm):
            continue
        if not api.gripper_holding(arm):                     # it slipped out on the way: start again from the pick
            return False
        for _ in range(3):                                   # align the mouse over the pad centre
            err = goal - bottom_centre(api)
            if np.linalg.norm(err[:2]) < ALIGN_TOL and abs(err[2]) < 0.01:
                break
            api.move_by(*err, arm=arm, linear=False)
        dz = pad_z + DROP_MARGIN - bottom_centre(api)[2] - (TILT_PRESS if tilted else 0.0)
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
    arm = side(mouse.position[0])
    # the arm on the mouse's side reaches the middle of the desk, not the far side: a pad on the other arm's side
    # is reached by a relay -- set the mouse down in the middle, the other arm takes it from there
    place_arm = arm if abs(pad.position[0]) < MIDDLE else side(pad.position[0])
    clear_pad(api, pad, mouse, place_arm)                 # clear the put-down spot while the hands are free
    if place_arm != arm:
        relay = np.array([RELAY_X, np.clip((mouse.position[1] + pad.position[1]) / 2, -0.20, 0.0)])
        for attempt in range(2):
            if pick(api, arm) and put_on_pad(api, arm, relay, TABLE_Z):
                break
            api.open_gripper(arm)
            api.home(arm)
        arm = place_arm
    for attempt in range(2):
        if not pick(api, arm):
            continue
        if put_on_pad(api, arm, pad.position[:2], pad_z):
            return
        api.open_gripper(arm)
        api.home(arm)
