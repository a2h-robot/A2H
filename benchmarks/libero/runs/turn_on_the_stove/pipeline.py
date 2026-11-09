"""turn_on_the_stove: grasp the stove knob's fin from above and twist it counter-clockwise."""

import sys

import numpy as np
from scipy.spatial.transform import Rotation

STOVE = "flat_stove_1"
KNOB = "flat_stove_1_button"
BASE = "flat_stove_1_base"
TURN_GOAL = 1.0      # rad of knob rotation to command (LIBERO counts >= 0.5 as on)
TURN_OK = 0.7        # rad measured rotation we accept as done


def log(msg):
    print(f"[pipe] {msg}", file=sys.stderr)


def R_of(q):
    w, x, y, z = q
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def q_of(R):
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    R = R_of(q)
    return np.arctan2(R[1, 0], R[0, 0])


def knob_rel(api, n=3):
    """Knob yaw relative to the stove base (averaged over n noisy detections)."""
    vals = []
    for _ in range(n):
        k, b = api.detect(KNOB), api.detect(BASE)
        vals.append(wrap(yaw_of(k.quaternion) - yaw_of(b.quaternion)))
    return float(np.angle(np.mean(np.exp(1j * np.array(vals)))))


def top_R(x):
    """Top-down gripper rotation whose finger-closing axis is the horizontal vector x."""
    x = np.array([x[0], x[1], 0.0])
    x /= np.linalg.norm(x)
    a = np.array([0.0, 0.0, -1.0])
    return np.column_stack([x, np.cross(a, x), a])


def rot_axis(axis, ang):
    return Rotation.from_rotvec(np.asarray(axis, float) * ang).as_matrix()


# Candidate hand orientations for the pinch, as (pitch about the closing axis, i.e. leaning along
# the fin; roll about the fin's long axis), tried in order of how far they are from straight down.
TILTS = sorted({(p, r) for p in (0.0, 0.5, -0.5, 1.0, -1.0) for r in (0.0, 0.4, -0.4)},
               key=lambda t: (abs(t[0]) + abs(t[1]), abs(t[1])))
APPROACH = (0.045, 0.08)                 # pre-pinch distances along the approach axis [m]
MAX_PINCH_MOVES = 4                      # pinch candidates allowed to move (each costs time)


def twist_pinch(api, rel0):
    """Pinch the fin and rotate the wrist about the knob axis. The pinch approaches straight down
    if possible, otherwise rolled about the fin's long axis so the hand leans away from clutter."""
    k = api.detect(KNOB)
    Rk = R_of(k.quaternion)
    fin_thin = Rk[:, 0]; fin_thin[2] = 0; fin_thin /= np.linalg.norm(fin_thin)  # closing axis
    cur_x = R_of(api.tcp_pose()[1])[:, 0]
    if fin_thin @ cur_x < 0:                     # pinch is symmetric: keep the wrist near its yaw
        fin_thin = -fin_thin
    fin_long = np.cross([0, 0, 1.0], fin_thin)
    height = abs(Rk[2] @ k.bbox_extents)
    point = k.bbox_center.copy()
    point[2] = k.bbox_center[2] + 0.5 * height - min(0.025, 0.5 * height)
    api.open_gripper()
    grasped = False
    moves = 0
    for pitch, roll in TILTS:
        if moves >= MAX_PINCH_MOVES:
            break
        R = rot_axis(fin_long, roll) @ rot_axis(fin_thin, pitch) @ top_R(fin_thin)
        a = R[:, 2]
        pre = None
        for dist in APPROACH:
            if api.move_to((point - a * dist, q_of(R))):
                pre = dist
                break
        if pre is None:
            continue
        moves += 1
        api.move_to((point, q_of(R)), linear=True, speed="slow", allow_contact=[STOVE])
        if api.last_motion == "refused":
            log(f"pinch: descent refused at pitch {pitch:+.2f} roll {roll:+.2f}")
            continue
        api.close_gripper()
        w = api.gripper_width()
        log(f"pinch: pitch {pitch:+.2f} roll {roll:+.2f} pre {pre}, motion {api.last_motion}, width {w:.3f}")
        if 0.004 < w < 0.075:
            grasped = True
            break
        api.open_gripper()
        p, q = api.tcp_pose()
        api.move_to((p - a * 0.04, q), linear=True, allow_contact=[STOVE])
    if not grasped:
        return False
    axis = api.detect(KNOB).position.copy()
    best, fails = wrap(knob_rel(api) - rel0), 0
    for step in range(8):
        if best >= TURN_GOAL - 0.05 or fails >= 2:
            break
        d = min(0.3, TURN_GOAL - best + 0.05)
        p, q = api.tcp_pose()
        Rd = Rz(d)
        p2 = axis + Rd @ (p - axis)
        p2[2] = p[2]
        api.move_to((p2, q_of(Rd @ R_of(q))), linear=True, speed="slow", allow_contact=[STOVE])
        turned = wrap(knob_rel(api) - rel0)
        fails = fails + 1 if (api.last_motion == "refused" or turned < best + 0.05) else 0
        best = max(best, turned)
        log(f"  twist step {step}: d={d:.2f} motion={api.last_motion} turned={turned:.2f}")
    api.open_gripper()
    p, q = api.tcp_pose()
    api.move_to((p - R_of(q)[:, 2] * 0.04, q), linear=True, allow_contact=[STOVE])
    return True


def hand_R(a, x_hint):
    """Gripper rotation with approach axis a and closing axis as close as possible to x_hint."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    x = np.asarray(x_hint, float) - (np.asarray(x_hint, float) @ a) * a
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(a, x), a])


PUSH_PITCHES = (0.0, 0.5, 1.0, np.pi / 2)   # approach tilt below horizontal (pi/2 = straight down)


def push_seat(api, ends):
    """Place the closed fingertips behind one end of the fin (from a fresh detection), ready to
    push it tangentially along the approach axis. Returns (center, start, R, end) or None."""
    k = api.detect(KNOB)
    Rk = R_of(k.quaternion)
    c = k.position.copy()
    u = Rk[:, 1].copy(); u[2] = 0; u /= np.linalg.norm(u)   # fin long axis (+u = tall end)
    t = np.cross([0, 0, 1.0], u)                             # at +u end, pushing +t turns +z
    r, back = 0.03, 0.035
    # Tall end (+1) first; approach along the push direction or yawed towards the knob axis
    # (pushing partly into the fin keeps contact); the short end (-1) only as a last resort.
    cands = [(e, pt, yo, h) for e in ends for yo in (0.0, -0.5, 0.5, -0.9) for pt in PUSH_PITCHES
             for h in ((0.04, 0.03) if e > 0 else (0.03,))]
    for sgn, pitch, yoff, dz in cands:
        dpush = sgn * t
        start = c + sgn * r * u - back * dpush
        start[2] = c[2] + dz                                  # above the knob's base ring
        dh = Rz(sgn * yoff) @ dpush                           # yawed approach, horizontal part
        a = np.cos(pitch) * dh + np.sin(pitch) * np.array([0, 0, -1.0])
        R = hand_R(a, np.cross([0, 0, 1.0], dh))
        # The start pose is clear of the knob, so plan to it directly (a refusal costs no time);
        # only if that fails, come in along the approach axis from a pre-pose.
        if not api.move_to((start, q_of(R))):
            if not any(api.move_to((start - a * dist, q_of(R))) for dist in (0.06, 0.03)):
                continue
            api.move_to((start, q_of(R)), linear=True, speed="slow", allow_contact=[STOVE])
        if api.last_motion != "refused":
            log(f"push: seated at end {sgn:+.0f}, pitch {pitch:.2f}, yaw off {yoff:+.1f}, dz {dz}")
            return c, start, R, sgn
    return None


def twist_push(api, rel0, seats=4):
    """Fallback without a pinch: push one end of the fin tangentially with the closed fingertips
    (pushing along the approach axis) and sweep an arc about the knob axis. Both fin ends, pushed
    in opposite directions, turn the knob the same way. When the knob stops following (the
    fingertip slides off the fin as it turns), back off and re-seat from a fresh detection."""
    api.close_gripper()
    ends = (1.0, -1.0)
    for seat in range(seats):
        placed = push_seat(api, ends)
        if placed is None:
            log("push: no end / approach reaches the fin")
            break
        c, start, R, sgn = placed
        ends = (sgn, -sgn)
        z = start[2]
        theta, stall, best = 0.0, 0, wrap(knob_rel(api) - rel0)
        while theta < 1.2 and stall < 3:
            if best >= TURN_GOAL:
                break
            theta += 0.15
            p = c + Rz(theta) @ (start - c)
            p[2] = z
            api.move_to((p, q_of(Rz(theta) @ R)), linear=True, speed="slow", allow_contact=[STOVE])
            if api.last_motion == "refused":           # cannot turn the hand further: keep its yaw
                api.move_to((p, api.tcp_pose()[1]), linear=True, speed="slow", allow_contact=[STOVE])
            if api.last_motion == "refused":
                break
            turned = wrap(knob_rel(api) - rel0)
            stall = stall + 1 if turned < best + 0.03 else 0
            best = max(best, turned)
            log(f"  push seat {seat} arc {theta:.2f}: motion={api.last_motion} turned={turned:.2f}")
        p, q = api.tcp_pose()
        api.move_to((p - R_of(q)[:, 2] * 0.05, q), linear=True, allow_contact=[STOVE])
        if wrap(knob_rel(api) - rel0) >= TURN_OK:
            break
    api.open_gripper()
    return True


CLEAR_R = 0.13      # horizontal radius around the knob that the pinching hand sweeps [m]


def clutter_near_knob(api):
    """Objects (not the stove, not fixtures' parts) whose box comes within CLEAR_R of the knob."""
    c = api.detect(KNOB).position
    out = []
    for name in api.objects():
        if name == STOVE or api.parts(name):          # articulated fixtures have parts
            continue
        st = api.detect(name)
        if st is None:
            continue
        R = R_of(st.quaternion)
        half_xy = 0.5 * np.abs(R[:2, :]) @ st.bbox_extents
        gap = np.maximum(np.abs(st.bbox_center[:2] - c[:2]) - half_xy, 0.0)
        top = st.bbox_center[2] + 0.5 * abs(R[2] @ st.bbox_extents)
        bottom = st.bbox_center[2] - 0.5 * abs(R[2] @ st.bbox_extents)
        if bottom > c[2] + 0.06:                       # hangs above the knob: not resting clutter
            continue
        if np.linalg.norm(gap) < CLEAR_R and top > c[2] + 0.03:
            out.append((float(np.linalg.norm(gap)), name))
    return [n for _, n in sorted(out)]


def relocate(api, name):
    """Pick an object up and set it down further from the knob."""
    c = api.detect(KNOB).position
    st = api.detect(name)
    held = False
    for strat in ("top", "side"):
        if api.grasp(name, strategy=strat):
            held = True
            break
        log(f"clear: {strat} grasp of {name} failed (motion {api.last_motion})")
        api.open_gripper()
        p, q = api.tcp_pose()
        api.move_to((p + [0, 0, 0.08], q))
    if not held:
        return push_away(api, name, c)
    p, q = api.tcp_pose()
    api.move_to((p + [0, 0, 0.10], q))
    away = st.bbox_center[:2] - c[:2]
    away /= np.linalg.norm(away) + 1e-9
    lift = api.tcp_pose()[0]
    for ang in (0.0, 0.6, -0.6, 1.2, -1.2):
        d = Rz(ang)[:2, :2] @ away
        dest = p.copy()
        dest[:2] = p[:2] + 0.15 * d
        dest[2] = p[2] + 0.01
        if api.move_to((dest + [0, 0, lift[2] - dest[2]], q)):
            api.place((dest, q))
            pp, qq = api.tcp_pose()
            api.move_to((pp + [0, 0, 0.10], qq))
            log(f"clear: moved {name} by {np.round(0.15 * d, 2)}")
            return True
    api.place((p, q))
    log(f"clear: no free spot for {name}, put it back")
    return False


def push_away(api, name, c):
    """Slide an object at least 10 cm along a direction that is not toward the knob, pushing it
    with the closed fingertips at a third of its height."""
    st = api.detect(name)
    away = st.bbox_center[:2] - c[:2]
    away /= np.linalg.norm(away) + 1e-9
    R = R_of(st.quaternion)
    half_xy = 0.5 * np.abs(R[:2, :]) @ st.bbox_extents
    h = abs(R[2] @ st.bbox_extents)
    z = max(0.92, st.bbox_center[2] - 0.5 * h + h / 3)
    api.close_gripper()
    for ang in (0.0, np.pi / 2, -np.pi / 2, np.pi / 4, -np.pi / 4):
        d = Rz(ang)[:2, :2] @ away
        reach = np.abs(d) @ half_xy + 0.03
        start = np.array([*(st.bbox_center[:2] - reach * d), z])
        Rh = top_R(np.array([-d[1], d[0], 0.0]))   # fingers side by side across the push
        if not api.move_to((start + [0, 0, 0.12], q_of(Rh))):
            continue
        api.move_to((start, q_of(Rh)), linear=True, speed="slow")
        if api.last_motion == "refused":
            api.move_to((start + [0, 0, 0.12], q_of(Rh)))
            continue
        api.move_to((start + [*(0.13 * d), 0], q_of(Rh)), linear=True, speed="slow", allow_contact=[name])
        p, q = api.tcp_pose()
        api.move_to((p + [0, 0, 0.12], q), linear=True)
        moved = np.linalg.norm(api.detect(name).bbox_center[:2] - st.bbox_center[:2])
        log(f"clear: pushed {name} along {np.round(d, 2)}, moved {moved:.3f}")
        if moved > 0.06:
            api.open_gripper()
            return True
        st = api.detect(name)
    api.open_gripper()
    return False


def run(api):
    rel0 = knob_rel(api)
    log(f"knob rel yaw at start {rel0:.2f}")
    cleared = False
    for attempt in range(4):
        before = wrap(knob_rel(api) - rel0)
        pinched = twist_pinch(api, rel0)
        turned = wrap(knob_rel(api) - rel0)
        log(f"attempt {attempt}: pinch {pinched}, turned {turned:.2f}")
        if turned >= TURN_OK:
            return
        if not pinched and not cleared:
            cleared = True
            names = clutter_near_knob(api)
            for name in names:
                relocate(api, name)
            if names:
                continue
        if turned - before < 0.15:                 # the pinch cannot turn it: push the fin
            twist_push(api, rel0)
            turned = wrap(knob_rel(api) - rel0)
            log(f"attempt {attempt}: push, turned {turned:.2f}")
            if turned >= TURN_OK:
                return
