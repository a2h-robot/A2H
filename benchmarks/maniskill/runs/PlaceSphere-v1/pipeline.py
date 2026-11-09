"""PlaceSphere-v1: place the sphere into the bin.

Strategy:
- perception: average many noisy detections; estimate the sphere's velocity from a short
  time series (one averaged detection per control step) before every grasp attempt;
- grasp: top-down with the fingers 45 deg to the bin axes when the sphere is still; if it
  rolls, wait for it at a point ahead on its path with the fingers across the path and
  close when it arrives; verify the grasp and retry;
- place: lift, re-measure the sphere in the hand and the bin, centre the sphere over the
  bin, lower until just above the rim, release, retreat.
"""

import sys

import numpy as np

from api import pose_mul, rpy_to_quat

N_AVG = 60
SPHERE_R = 0.02
BIN_FLOOR_TOP = 0.005      # bin floor top above bin origin (bin origin is mid-floor)
STILL_SPEED = 0.01         # m/s; below this the sphere is treated as static
ARM_TIME = 4.0             # s; rough time to reach a meeting point, re-measure and descend
MAX_ATTEMPTS = 4
YAWS = [np.pi / 4, -np.pi / 4, 0.0, np.pi / 2]   # first two put the fingers in the bin corners
TILTS = [0.0, 0.6, 0.9]    # rad; hand tilted forward (fingers away from the robot) to reach under overhangs
TASK_OBJECTS = ("sphere", "bin")
HELD_CLEARANCE = 0.03      # m; gap between the held sphere and any obstacle during transit
MIN_CARRY_Z = 0.12         # m; lowest TCP height for carrying
BIN_HALF = 0.025           # m; half width of the bin footprint
AVOID = []                 # names of non-task objects, given to the planner on every motion


def move(api, pose, speed="normal"):
    return api.move_to(pose, speed=speed, avoid=AVOID or None)


def obstacle_bounds(api, names, n=20):
    """World axis-aligned bounds (lo, hi) of each named object (averaged boxes, all 8 corners)."""
    out = []
    for name in names:
        boxes = [b for _ in range(n) for b in api.get_obstacles() if b.name == name]
        if not boxes:
            continue
        c = np.median([b.center for b in boxes], axis=0)
        e = boxes[0].extents
        q = boxes[0].quaternion
        corners = [pose_mul((c, q), (np.array([sx, sy, sz]) * e / 2, [1, 0, 0, 0]))[0]
                   for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        out.append((np.min(corners, axis=0), np.max(corners, axis=0)))
    return out


def carry_heights(bounds, a, b, margin=0.06):
    """Candidate TCP heights for carrying the held sphere from xy a to xy b, lowest first.
    Obstacles bound the free space from below AND above: a height is kept only if the held
    sphere (which the planner does not model) clears every obstacle over the corridor."""
    lo_xy = np.minimum(a[:2], b[:2]) - margin
    hi_xy = np.maximum(a[:2], b[:2]) + margin
    cands = [MIN_CARRY_Z] + [float(hi[2]) + HELD_CLEARANCE + SPHERE_R for _, hi in bounds]
    ok = []
    for z in sorted(set(cands)):
        clear = True
        for lo, hi in bounds:
            if np.any(hi[:2] < lo_xy) or np.any(lo[:2] > hi_xy):
                continue                                  # not over the corridor
            if z - SPHERE_R - HELD_CLEARANCE < hi[2] and z + SPHERE_R + HELD_CLEARANCE > lo[2]:
                clear = False                             # sphere would pass through it
        if clear:
            ok.append(z)
    return ok


def log(*a):
    print("[pipeline]", *a, file=sys.stderr)


def avg_pos(api, name, n=N_AVG):
    ps = [api.detect(name).position for _ in range(n)]
    return np.median(np.array(ps), axis=0)


def measure_motion(api, name, duration_s=1.0, n_per_step=40):
    """Position now and velocity, from a line fit to per-step averaged detections."""
    dt = 0.05  # 20 Hz control
    samples = []

    def collect():
        samples.append(avg_pos(api, name, n_per_step))
        return False

    api.wait_until(collect, duration_s)
    P = np.array(samples)
    t = np.arange(len(P)) * dt
    A = np.stack([t, np.ones_like(t)], 1)
    coef, *_ = np.linalg.lstsq(A, P, rcond=None)
    v, p0 = coef[0], coef[1]
    v[2] = 0.0
    p_now = p0 + v * t[-1]
    p_now[2] = np.median(P[:, 2])
    return p_now, v


def yaw_quat(q0, yaw):
    """q0 rotated by yaw about the tool z axis (the approach axis for a top-down grasp)."""
    return pose_mul(([0, 0, 0], q0), ([0, 0, 0], rpy_to_quat(0, 0, yaw)))[1]


def closing_axis(q):
    return pose_mul(([0, 0, 0], q), ([0, 1, 0], [1, 0, 0, 0]))[0]


def yaw_for_closing(q0, direction):
    """Tool yaw in (-pi/2, pi/2] that makes the closing axis parallel to `direction` (xy)."""
    best, best_err = 0.0, 1e9
    d = np.asarray(direction, float)[:2]
    d /= np.linalg.norm(d) + 1e-12
    for yaw in np.linspace(-np.pi / 2, np.pi / 2, 181):
        c = closing_axis(yaw_quat(q0, yaw))[:2]
        err = 1.0 - abs(float(c @ d)) / (np.linalg.norm(c) + 1e-12)
        if err < best_err:
            best, best_err = yaw, err
    return best


def yaw_candidates(q0, first=None):
    """Wrist yaws to try, best first: diagonal to the bin walls, then along the walls.
    A planner failure costs no motion, so trying alternatives is cheap."""
    qs = [yaw_quat(q0, y) for y in YAWS]
    if first is not None:
        qs = [first] + [q for q in qs if not np.allclose(q, first)]
    return qs


def grasp_static(api, p, qs):
    api.open_gripper()
    for q in qs:
        if not move(api, (np.array([p[0], p[1], p[2] + 0.08]), q)):
            continue
        if not move(api, (np.array([p[0], p[1], p[2]]), q), speed="slow"):
            continue
        api.close_gripper()
        return api.gripper_holding()
    log("no collision-free grasp found")
    return False


def goal_occupants(api, bin_p):
    """Non-task objects resting in the bin (inside its footprint, below its rim + a bit)."""
    out = []
    for name in AVOID:
        c = avg_pos(api, name, 30)
        if np.linalg.norm(c[:2] - bin_p[:2]) < BIN_HALF and c[2] < bin_p[2] + 0.06:
            out.append(name)
    return out


def clear_goal(api, q0, sphere_p):
    """Lift anything sitting in the bin out and set it down on free table away from the task."""
    for _ in range(2):
        bin_p = avg_pos(api, "bin")
        occ = goal_occupants(api, bin_p)
        if not occ:
            return True
        for name in occ:
            log("goal occupied by", name, "- clearing it")
            c = avg_pos(api, name)
            # drop spot: beside the bin, on the side away from the sphere
            side = -1.0 if sphere_p[1] > bin_p[1] else 1.0
            drop = np.array([bin_p[0], bin_p[1] + side * 0.2, c[2] + 0.02])
            AVOID.remove(name)                     # it is the thing we grasp now
            try:
                api.open_gripper()
                ok = False
                for q in yaw_candidates(q0):
                    if not move(api, (c + [0, 0, 0.08], q)):
                        continue
                    if move(api, (c + [0, 0, 0.005], q), speed="slow"):
                        api.close_gripper()
                        ok = api.gripper_holding()
                        break
                if not ok:
                    log("could not grasp", name)
                    api.open_gripper()
                    tcp_p, tcp_q = api.tcp_pose()
                    move(api, (tcp_p + [0, 0, 0.08], tcp_q))
                    continue
                tcp_p, tcp_q = api.tcp_pose()
                move(api, (tcp_p + [0, 0, 0.1], tcp_q))
                move(api, (drop + [0, 0, 0.1], tcp_q))
                move(api, (drop, tcp_q), speed="slow")
                api.open_gripper()
                tcp_p, tcp_q = api.tcp_pose()
                move(api, (tcp_p + [0, 0, 0.1], tcp_q))
            finally:
                AVOID.append(name)
    return not goal_occupants(api, avg_pos(api, "bin"))


def grasp_rolling(api, q0, p, v):
    """Wait for a rolling sphere ahead on its path with the fingers across the path."""
    speed = np.linalg.norm(v)
    vhat = v / speed
    perp = np.array([-vhat[1], vhat[0], 0.0])
    q = yaw_quat(q0, yaw_for_closing(q0, perp))
    lead = 0.04 + speed * ARM_TIME                # be there well before the sphere
    meet = p + vhat * lead
    api.open_gripper()
    move(api, (np.array([meet[0], meet[1], p[2] + 0.08]), q))
    # re-measure and put the fingers on the actual path
    p2, v2 = measure_motion(api, "sphere", 0.6)
    if np.linalg.norm(v2) > STILL_SPEED:
        speed = float(np.linalg.norm(v2))
        vhat = v2 / speed
        base = (p2 - p) * [1, 1, 0]
        if np.linalg.norm(base) > 0.05:           # long baseline: far better heading
            vhat = base / np.linalg.norm(base)
        if abs(float(closing_axis(q)[:2] @ vhat[:2])) > 0.3:   # fingers no longer across the path
            q = yaw_quat(q0, yaw_for_closing(q0, [-vhat[1], vhat[0], 0.0]))
    tcp = api.tcp_pose()[0]
    s = float((tcp - p2)[:2] @ vhat[:2])          # distance still to travel
    if s < 0.04:                                  # sphere is (almost) past us: retry
        log("sphere already past the meeting point", round(s, 3))
        return False
    meet = p2 + vhat * s
    move(api, (np.array([meet[0], meet[1], p2[2]]), q))

    def arrived():
        b = avg_pos(api, "sphere", 15)
        # close so that the sphere reaches the TCP about half way through the close
        return float((api.tcp_pose()[0] - b)[:2] @ vhat[:2]) < speed * 0.15

    ok = api.wait_until(arrived, 6.0)
    api.close_gripper()
    log("rolling grasp, arrived", ok)
    return api.gripper_holding()


def run(api):
    q0 = np.array([0.0, 1.0, 0.0, 0.0])        # exactly top-down (the start pose is slightly tilted)
    AVOID[:] = [n for n in api.objects() if n not in TASK_OBJECTS]
    log("avoid", AVOID)
    # the goal must be free before the sphere is picked up (with an empty hand we can clear it)
    if AVOID and not clear_goal(api, q0, avg_pos(api, "sphere")):
        log("goal still occupied")
    bounds = obstacle_bounds(api, AVOID)        # measured after anything was moved

    held = False
    for attempt in range(MAX_ATTEMPTS):
        p, v = measure_motion(api, "sphere")
        speed = float(np.linalg.norm(v))
        log("attempt", attempt, "sphere", p.round(3), "speed", round(speed, 4))
        if speed < STILL_SPEED:
            # go above, check again that it is still before descending
            api.open_gripper()
            q_used = None
            for q in yaw_candidates(q0):
                if move(api, (np.array([p[0], p[1], p[2] + 0.08]), q)):
                    q_used = q
                    break
            p, v = measure_motion(api, "sphere", 0.5)
            if np.linalg.norm(v) < STILL_SPEED:
                held = grasp_static(api, avg_pos(api, "sphere"), yaw_candidates(q0, q_used))
            else:
                held = grasp_rolling(api, q0, p, v)
        else:
            held = grasp_rolling(api, q0, p, v)
        if held:
            break
        log("grasp failed, retrying")
        api.open_gripper()
        tcp_p, tcp_q = api.tcp_pose()
        move(api, (tcp_p + [0, 0, 0.08], tcp_q))
    if not held:
        log("giving up")
        return

    bin_p = avg_pos(api, "bin")
    goal_center = bin_p + [0, 0, BIN_FLOOR_TOP / 2 + 0.0025 + SPHERE_R]  # sphere centre when seated
    tcp_p, tcp_q = api.tcp_pose()
    off = avg_pos(api, "sphere") - tcp_p
    log("in-hand offset", off.round(4))
    above = goal_center - off
    # lowest carry height first; at each, the first wrist yaw the planner accepts down to the bin
    diagonal, placed = True, False
    heights = carry_heights(bounds, tcp_p, above)
    log("carry heights", np.round(heights, 3))
    for z in heights:
        tcp_p, tcp_q = api.tcp_pose()
        if not move(api, (np.array([tcp_p[0], tcp_p[1], z]), tcp_q)):
            continue
        for tilt in TILTS:
            for i, qy in enumerate(yaw_candidates(q0)):
                q = pose_mul(([0, 0, 0], rpy_to_quat(0, -tilt, 0)), ([0, 0, 0], qy))[1]
                if not move(api, (np.array([above[0], above[1], z]), q)):
                    continue
                if move(api, (goal_center + [0, 0, 0.06] - off, q)):
                    diagonal, placed = i < 2 and tilt == 0, True
                    break
            if placed:
                break
        if placed:
            log("carried at", round(z, 3), "tilt", tilt)
            break
    if not placed:
        log("no collision-free transfer to the bin")
    # re-measure after the transfer and correct
    tcp_p, tcp_q = api.tcp_pose()
    off = avg_pos(api, "sphere") - tcp_p
    goal_center = avg_pos(api, "bin") + [0, 0, BIN_FLOOR_TOP / 2 + 0.0025 + SPHERE_R]
    # fingers along the walls would hit the rim: release from higher
    release = goal_center + [0, 0, 0.012 if diagonal else 0.025] - off
    move(api, (goal_center + [0, 0, 0.03] - off, tcp_q), speed="slow")
    move(api, (release, tcp_q), speed="slow")
    api.open_gripper()
    tcp_p, tcp_q = api.tcp_pose()
    move(api, (tcp_p + [0, 0, 0.08], tcp_q), speed="slow")
    api.wait_until(lambda: False, 1.0)
