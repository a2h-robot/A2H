"""LiftPegUpright-v1: lift the peg lying on the table to an upright pose.

Strategy: average many noisy detections, top-grasp the peg firmly a little off-centre,
lift it, measure where the peg sits in the hand, then compute the hand pose that puts
the peg upright just above the table and lower it there before releasing.
"""

import numpy as np

from api import pose_inv, pose_mul

N_DETECT = 60
LIFT_Z = 0.25
LIFT_MIN = 0.10
PEG_HALF_LENGTH = 0.12
PEG_HALF_WIDTH = 0.025
RELEASE_CLEARANCE = 0.008
MAX_ATTEMPTS = 3
GRASP_PITCHES = (np.pi / 2, np.pi / 3, np.pi / 4)   # hand elevation: vertical first
GRASP_OFFSETS = (0.03, -0.03, 0.06, -0.06, 0.0, 0.09, -0.09)
FINGER_CENTER = 0.05      # finger centre from the TCP along the closing axis when open [m]
ROBOT_BASE_XY = (-0.615, 0.0)


# ------------------------------------------------------------------ math helpers
def quat_to_mat(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.asarray(q)
    return q / np.linalg.norm(q)


def mean_pose(api, name, n=N_DETECT):
    """Average n noisy detections (position mean, sign-aligned quaternion mean)."""
    ps, qs = [], []
    for _ in range(n):
        st = api.detect(name)
        if st is None:
            continue
        q = np.asarray(st.quaternion, float)
        if qs and np.dot(q, qs[0]) < 0:
            q = -q
        ps.append(st.position)
        qs.append(q)
    if not ps:
        return None
    q = np.mean(qs, axis=0)
    return np.mean(ps, axis=0), q / np.linalg.norm(q)


def long_axis(q):
    """World direction of the peg's longest bbox axis (its local x)."""
    return quat_to_mat(q)[:, 0]


def scene_obstacles(api, n=20):
    """Names and world AABBs (lo, hi) of every object other than the peg (averaged over n
    noisy readings)."""
    acc = {}
    for _ in range(n):
        for b in api.get_obstacles():
            if b.name == "peg":
                continue
            half = np.abs(quat_to_mat(b.quaternion)) @ (np.asarray(b.extents) / 2)
            acc.setdefault(b.name, []).append(np.r_[b.center - half, b.center + half])
    names = list(acc)
    boxes = [(np.mean(acc[k], 0)[:3], np.mean(acc[k], 0)[3:]) for k in names]
    return names, boxes


def fingers_clear(g, closing, u, boxes, margin=0.01):
    """True if the open fingers at grasp point g (closing direction `closing`, peg axis u)
    stay clear of every obstacle box."""
    for side in (-1.0, 1.0):
        c = g + side * FINGER_CENTER * closing
        half = np.abs(np.outer(closing, [1])).ravel() * 0.012 + np.abs(u) * 0.012 + margin
        lo_f = c - half
        hi_f = c + half
        # From the peg's bottom face up: the peg need not rest on the table.
        lo_f[2], hi_f[2] = g[2] - PEG_HALF_WIDTH + 0.005, g[2] + 0.10
        for lo, hi in boxes:
            if np.all(hi_f > lo) and np.all(lo_f < hi):
                return False
    return True


def hand_clear(g, closing, u, boxes, margin=0.01):
    """True if the palm of a vertical hand at grasp point g misses every obstacle box."""
    half = np.abs(closing) * 0.105 + np.abs(u) * 0.03 + margin
    lo_h, hi_h = g - half, g + half
    lo_h[2], hi_h[2] = g[2] + 0.05, g[2] + 0.13
    return not any(np.all(hi_h > lo) and np.all(lo_h < hi) for lo, hi in boxes)


def graspable_at(c, closing, u, boxes):
    """True if some grasp offset along the peg at centre c leaves room for fingers and palm."""
    return any(fingers_clear(c + off * u, closing, u, boxes) and hand_clear(c + off * u, closing, u, boxes)
               for off in GRASP_OFFSETS)


def peg_path_clear(p, c, closing, u, boxes, margin=0.003):
    """True if the lying peg, slid from p to c, never overlaps an obstacle box."""
    half = np.abs(u) * PEG_HALF_LENGTH + np.abs(closing) * PEG_HALF_WIDTH + margin
    half[2] = PEG_HALF_WIDTH
    for t in np.linspace(0.0, 1.0, 6):
        m = p + t * (c - p)
        if any(np.all(m + half > lo) and np.all(m - half < hi) for lo, hi in boxes):
            return False
    return True


def slide_to_graspable(api, p, u, closing, boxes, avoid):
    """No grasp fits where the peg lies (e.g. against a wall along its length): push it along
    its own axis, closed fingers against one end face, until some grasp would fit. Shortest
    push first (towards the robot on ties). Returns True if it pushed the peg."""
    options = []
    for s in (-1.0, 1.0):
        for d in np.arange(0.04, 0.301, 0.02):
            c = p + s * d * u
            if np.hypot(c[0] - ROBOT_BASE_XY[0], c[1] - ROBOT_BASE_XY[1]) > 0.75:
                break
            if not peg_path_clear(p, c, closing, u, boxes):
                break
            if graspable_at(c, closing, u, boxes):
                options.append((d + 0.05 * (s * u[0] > 0), s, d + 0.04))   # 4 cm past the minimum
                break
    down = np.array([0.0, 0.0, -1.0])
    q = mat_to_quat(np.column_stack([np.cross(closing, down), closing, down]))
    for _, s, d in sorted(options):
        push = s * u
        # Fingertips against the upper half of the end face (the palm must stay above
        # neighbouring obstacles); the planner decides which height fits.
        # Start a little behind the end face (farther if the palm does not fit there).
        for gap, dz in ((0.04, 0.02), (0.04, 0.03), (0.06, 0.02), (0.06, 0.03)):
            start = p - push * (PEG_HALF_LENGTH + gap)
            start[2] = p[2] + dz
            api.close_gripper()
            if not api.move_to((start + [0, 0, 0.08], q), avoid=avoid + ["peg"]):
                continue
            if not api.move_to((start, q), speed="slow", avoid=avoid + ["peg"]) or \
                    np.linalg.norm(api.tcp_pose()[0] - start) > 0.01:
                tcp = api.tcp_pose()
                api.move_to((tcp[0] + [0, 0, 0.05], tcp[1]), speed="slow", avoid=avoid)
                continue
            # The planner does not know the peg is being pushed: plan without it.
            pushed = api.move_to((start + push * (d + gap - 0.01), q), speed="slow", avoid=avoid)
            tcp = api.tcp_pose()
            api.move_to((tcp[0] - 0.02 * push, tcp[1]), speed="slow", avoid=avoid)
            tcp = api.tcp_pose()
            api.move_to((tcp[0] + [0, 0, 0.08], tcp[1]), speed="slow", avoid=avoid + ["peg"])
            api.open_gripper()
            now = mean_pose(api, "peg", 20)
            if now is not None and np.linalg.norm(now[0][:2] - p[:2]) > 0.02:
                return True
            if not pushed:
                continue
    return False


def wait_until_still(api, timeout_s=6.0, tol=0.01):
    """Hold the arm still until no obstacle moves more than tol between two averaged looks
    0.5 s apart. Returns the settled (names, boxes)."""
    prev = scene_obstacles(api)
    for _ in range(int(timeout_s / 0.5)):
        api.wait_until(lambda: False, 0.5)
        cur = scene_obstacles(api)
        shift = [np.linalg.norm((a[0] + a[1]) / 2 - (b[0] + b[1]) / 2) for a, b in zip(prev[1], cur[1])]
        prev = cur
        if not shift or max(shift) < tol:
            break
    return prev


def footprint_clear(xy, boxes, margin=0.04):
    """True if the upright peg's footprint at xy (plus margin) misses every obstacle box."""
    r = PEG_HALF_WIDTH + margin
    return not any(np.all(xy + r > lo[:2]) and np.all(xy - r < hi[:2]) for lo, hi in boxes)


def go_above(api, xys, rotations, T_tcp_peg, avoid, heights):
    """Swing/carry the held peg upright above the first reachable release spot.
    Returns (xy, q) or None."""
    for xy in xys:
        for h in heights:
            for R_c in rotations:
                q_c = mat_to_quat(R_c)
                above = pose_mul((np.r_[xy, PEG_HALF_LENGTH + RELEASE_CLEARANCE + h], q_c), pose_inv(T_tcp_peg))
                # slow: a fast swing flings the peg out of the fingers
                if api.move_to(above, speed="slow", avoid=avoid):
                    return xy, q_c
    return None


def release_candidates(p_xy, boxes, margin=0.04):
    """Release spots near the pick spot whose upright-peg footprint is clear of obstacles,
    nearest first. The planner (with avoid=) then rejects spots where the arm does not fit."""
    out = []
    for dx in np.arange(-0.40, 0.201, 0.05):
        for dy in np.arange(-0.35, 0.351, 0.05):
            xy = p_xy + [dx, dy]
            if not footprint_clear(xy, boxes, margin):
                continue
            if np.hypot(xy[0] - ROBOT_BASE_XY[0], xy[1] - ROBOT_BASE_XY[1]) > 0.80:
                continue
            out.append((np.hypot(dx, dy), xy))
    return [xy for _, xy in sorted(out, key=lambda t: t[0])]


def vertical_corridor(xy, boxes, radius=0.30):
    """(floor, ceiling) of the free space around xy: the highest top of obstacles resting
    near the table and the lowest bottom of obstacles hanging above the work area."""
    floor, ceiling = 0.0, np.inf
    for lo, hi in boxes:
        near = np.all(xy + radius > lo[:2]) and np.all(xy - radius < hi[:2])
        if not near:
            continue
        if lo[2] < 0.10:
            floor = max(floor, hi[2])
        else:
            ceiling = min(ceiling, lo[2])
    return floor, ceiling


def escape_if_stuck(api, avoid):
    """If the planner cannot even make a 1 cm move, the arm starts in (inflated) contact with
    an obstacle: back away from it with a short straight move planned without obstacles."""
    tcp = api.tcp_pose()
    if api.move_to((tcp[0] + [0, 0, -0.01], tcp[1]), speed="slow", avoid=avoid):
        return
    for d in ([0, 0, -0.05], [-0.05, 0, 0], [0, 0, 0.05]):
        if api.move_to((tcp[0] + d, tcp[1]), speed="slow"):
            tcp = api.tcp_pose()
            if api.move_to((tcp[0] + [0, 0, -0.01], tcp[1]), speed="slow", avoid=avoid):
                return


# ---------------------------------------------------------------------- program
def is_upright(api):
    peg = mean_pose(api, "peg", 20)
    if peg is None:
        return False
    return abs(long_axis(peg[1])[2]) > 0.995 and abs(peg[0][2] - PEG_HALF_LENGTH) < 0.01


def run(api):
    escape_if_stuck(api, scene_obstacles(api)[0])
    attempts = slides = 0
    while attempts < MAX_ATTEMPTS:
        status = attempt(api, allow_slide=slides < 3)
        if status == "slid":          # the peg was moved to where it can be grasped: look again
            slides += 1
            continue
        attempts += 1
        if status == "placed" or is_upright(api):
            return
        # Lost the peg or could not reach: back off upwards and start over from fresh detections.
        api.open_gripper()
        tcp = api.tcp_pose()
        api.move_to((tcp[0] + [0, 0, 0.10], tcp[1]), avoid=scene_obstacles(api)[0])


def attempt(api, allow_slide=True):
    api.open_gripper()
    avoid, boxes = scene_obstacles(api)
    peg = mean_pose(api, "peg")
    if peg is None:
        return "no_peg"
    floor, ceiling = vertical_corridor(peg[0][:2], boxes)
    p, q = peg
    u = long_axis(q)
    u[2] = 0.0
    u /= np.linalg.norm(u)
    if u[0] < 0:                      # grasp towards the end that is farther from the robot
        u = -u

    # Grasp: fingers close across the peg (horizontal, perpendicular to u). Try a vertical
    # hand first, then hands pitched back towards the robot (they need less headroom), at
    # every offset along the peg where the open fingers fit between the peg and its
    # neighbours; the obstacle-aware planner has the final say.
    closing = np.cross([0.0, 0.0, 1.0], u)
    closing /= np.linalg.norm(closing)
    cur_closing = quat_to_mat(api.tcp_pose()[1])[:, 1]
    if np.dot(closing, cur_closing) < 0:
        closing = -closing
    grasped = False
    any_fit = False
    for pitch in GRASP_PITCHES:
        approach = np.cos(pitch) * u + np.sin(pitch) * np.array([0.0, 0.0, -1.0])
        q_grasp = mat_to_quat(np.column_stack([np.cross(closing, approach), closing, approach]))
        for off in GRASP_OFFSETS:
            g = p + off * u
            g[2] = p[2]
            # A pitched hand reaches deeper into the peg along its approach for the same TCP
            # height; back off so the palm stays above the peg (finger depth ~2 cm).
            g = g - max(0.0, PEG_HALF_WIDTH / np.sin(pitch) - 0.02) * approach
            if not fingers_clear(g, closing, u, boxes):
                continue
            any_fit = True
            if not any(api.move_to((g - d * approach, q_grasp), avoid=avoid) for d in (0.10, 0.06)):
                continue
            # move_to reports a successful plan even if contact stopped the hand short of the
            # target; closing there shoves the peg. Check the hand really arrived.
            if api.move_to((g, q_grasp), speed="slow", avoid=avoid) and \
                    np.linalg.norm(api.tcp_pose()[0] - g) < 0.01:
                grasped = True
                break
            tcp = api.tcp_pose()
            api.move_to((tcp[0] - 0.05 * approach, tcp[1]), speed="slow", avoid=avoid)
        if grasped:
            break
    if not grasped:
        # No room for the fingers anywhere along the peg: move the peg instead of giving up.
        if not any_fit and allow_slide and slide_to_graspable(api, p, u, closing, boxes, avoid):
            return "slid"
        return "unreachable"
    api.close_gripper()
    if not api.gripper_holding():
        return "missed"

    # Lift height: above the obstacles resting on the table nearby, but low enough that the
    # hand (and the held peg, which the planner does not know about) stays under anything
    # hanging overhead.
    # The planner checks the arm; the cap keeps the held peg under overhead objects. Do not
    # guess how much headroom the arm needs: try heights from high to low and keep the first
    # one the planner accepts (never end the lift in contact with an overhead object).
    top = min(max(LIFT_Z, floor + 0.08), ceiling - 0.08)
    for z in np.arange(top, LIFT_MIN - 1e-6, -0.03) if top > LIFT_MIN else [LIFT_MIN]:
        if api.move_to((g + [0, 0, z - g[2]], q_grasp), avoid=avoid):
            break

    # Where is the peg in the hand?
    tcp = api.tcp_pose()
    peg_now = mean_pose(api, "peg")
    T_tcp_peg = pose_mul(pose_inv(tcp), peg_now)

    # Target peg orientation: long axis vertical with the grasped end up; one width axis
    # along world x (keeps the success check's Euler decomposition away from gimbal lock).
    # Either end may go up: prefer the hand pointing away from the robot (+x, reachable)
    # and the hand above the peg centre (keeps it off the table).
    best = None
    for x_up in (1.0, -1.0):
        x_new = np.array([0.0, 0.0, x_up])
        for y_axis in (np.array([1.0, 0, 0]), np.array([-1.0, 0, 0])):
            R_t = np.column_stack([x_new, y_axis, np.cross(x_new, y_axis)])
            R_tcp_t = R_t @ quat_to_mat(T_tcp_peg[1]).T
            tcp_above_centre = -(R_t @ quat_to_mat(T_tcp_peg[1]).T @ T_tcp_peg[0])[2]
            score = R_tcp_t[0, 2] + 0.5 * np.clip(tcp_above_centre / 0.03, -1, 1)
            if best is None or score > best[0]:
                best = (score, R_t)
    R_t = best[1]
    q_t = mat_to_quat(R_t)

    # The hand and forearm need free space beside the upright peg; the pick spot may not
    # have it. Search release spots (nearest first) and let the obstacle-aware planner
    # reject those where the arm would collide; try both equivalent peg yaws.
    R_alt = R_t @ np.diag([1.0, -1.0, -1.0])
    # Hover well above the table first; descend only after the final look at the scene.
    boxes_at_start = boxes
    res = go_above(api, release_candidates(p[:2], boxes), (R_t, R_alt), T_tcp_peg, avoid, (0.10, 0.06, 0.03))
    if res is None:
        return "unreachable"
    place_xy, q_t = res
    if not api.gripper_holding():
        return "dropped"
    # The scene may have changed while we were busy: look again before committing to the
    # release spot, and carry the peg (above everything new) to another spot if it is taken.
    avoid, boxes = scene_obstacles(api)
    moved = [np.linalg.norm((a[0] + a[1]) / 2 - (b[0] + b[1]) / 2) for a, b in zip(boxes_at_start, boxes)]
    if moved and max(moved) > 0.02:       # something is moving: let it come to rest first
        avoid, boxes = wait_until_still(api)
    if not footprint_clear(place_xy, boxes):
        # Carry height: the peg's bottom (unknown to the planner) must clear every obstacle top.
        R_now = quat_to_mat(q_t)
        carry_h = max([0.06] + [hi[2] + 0.03 - RELEASE_CLEARANCE for lo, hi in boxes if lo[2] < 0.10])
        res = go_above(api, release_candidates(place_xy, boxes), (R_now, R_now @ np.diag([1.0, -1.0, -1.0])),
                       T_tcp_peg, avoid, (carry_h, carry_h + 0.03))
        if res is None:
            return "unreachable"
        place_xy, q_t = res
    # The peg can pivot in the fingers during the swing: re-measure it in the hand and
    # correct the hand pose before the precise release.
    for _ in range(2):
        tcp = api.tcp_pose()
        T_tcp_peg = pose_mul(pose_inv(tcp), mean_pose(api, "peg"))
        above = pose_mul((np.r_[place_xy, PEG_HALF_LENGTH + RELEASE_CLEARANCE + 0.02], q_t), pose_inv(T_tcp_peg))
        if not api.move_to(above, speed="slow", avoid=avoid):
            break
    down = pose_mul((np.r_[place_xy, PEG_HALF_LENGTH + RELEASE_CLEARANCE], q_t), pose_inv(T_tcp_peg))
    api.move_to(down, speed="slow", avoid=avoid)
    api.open_gripper()

    # Retreat backwards along the approach direction, then up with the standing peg as an
    # obstacle (a retreat planned without it can sweep through it and knock it over).
    # The first, short move slides the open fingers straight off the peg (the peg cannot be in
    # avoid yet: the fingers start inside its inflated box and the planner would detour).
    tcp = api.tcp_pose()
    back = quat_to_mat(tcp[1])[:, 2]
    if not api.move_to((tcp[0] - 0.08 * back, tcp[1]), speed="slow", avoid=avoid):
        return "placed"           # stay put rather than risk a detour through the peg
    tcp = api.tcp_pose()
    api.move_to((tcp[0] + [0, 0, 0.10], tcp[1]), avoid=avoid + ["peg"])
    return "placed"
