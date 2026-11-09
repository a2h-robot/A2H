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
UNJAM_TOL = 0.05       # rad between the open hand's closing axis and the mouse's narrow side before backing out
RODE_UP = 0.02         # m: a mouse whose bottom is this far above the pad after the hand backed out went with it
WIGGLE = 0.25          # rad the hand turns either way to shake a jammed mouse loose
WIDE = 0.065           # m: a mouse wider than this is grasped by its measured box, not its annotated grasp points
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


def pick(api, arm):
    """Grasp the mouse from above and lift it. Returns True if it is held."""
    mouse = estimate(api, "mouse")
    R = t3d.quaternions.quat2mat(mouse.quaternion)
    narrow = min(mouse.bbox_extents[i] for i in range(3) if abs(R[2, i]) < 0.7)
    # a large mouse is taken by its measured box: the annotated grasp points scale with the model and leave the
    # opened fingers holding on to it
    for strategy in (() if narrow > WIDE else ("top", "auto")):
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
        release_and_back_out(api, arm, pad_z)
        api.home(arm)
        return True
    return False


def narrow_axis(s):
    """Horizontal unit vector across a detected object's narrower horizontal side."""
    R = t3d.quaternions.quat2mat(s.quaternion)
    i = min([i for i in range(3) if abs(R[2, i]) < 0.7], key=lambda k: s.bbox_extents[k])
    v = np.r_[R[:2, i], 0.0]
    return v / np.linalg.norm(v)


def turn_hand_to(api, arm, axis):
    """Turn the hand about the vertical so that its closing axis lines up with `axis` (either sign)."""
    p, q = api.tcp_pose(arm)
    R = t3d.quaternions.quat2mat(q)
    c = R[:, 1]
    ang = np.arctan2(c[0] * axis[1] - c[1] * axis[0], c[0] * axis[0] + c[1] * axis[1])
    ang = (ang + np.pi / 2) % np.pi - np.pi / 2                     # the closing axis has no sign
    if abs(ang) > UNJAM_TOL:
        api.move_to((p, t3d.quaternions.mat2quat(rotz(ang) @ R)), arm=arm, allow_contact=["mouse", "pad"])


def release_and_back_out(api, arm, pad_z):
    """With the gripper open: line the fingers up with the mouse (a long mouse that turned a little in the hand
    jams diagonally between the open fingers), back out, and check that the mouse stayed behind; if it rode up with
    the hand, go back down, shake it loose and back out again."""
    for attempt in range(2):
        turn_hand_to(api, arm, narrow_axis(estimate(api, "mouse")))
        p, qq = api.tcp_pose(arm)
        appr = t3d.quaternions.quat2mat(qq)[:, 0]
        api.move_to((p - RETREAT * appr, qq), arm=arm, linear=True, allow_contact=["mouse", "pad"])
        if bottom_centre(api)[2] < pad_z + RODE_UP:
            return
        api.move_to((p, qq), arm=arm, linear=True, allow_contact=["mouse", "pad"])   # it came along: back down
        for w in (WIGGLE, -WIGGLE):
            pw, qw = api.tcp_pose(arm)
            api.move_to((pw, t3d.quaternions.mat2quat(rotz(w) @ t3d.quaternions.quat2mat(qw))), arm=arm,
                        allow_contact=["mouse", "pad"])


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
