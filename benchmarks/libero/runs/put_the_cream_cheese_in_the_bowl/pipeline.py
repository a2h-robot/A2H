"""put the cream cheese in the bowl (LIBERO-Goal). Uses only the public RobotAPI."""

import numpy as np
from api import rpy_to_quat, quat_to_rpy, pose_mul

OBJ = "cream_cheese_1"
BOWL = "akita_black_bowl_1"
BASE_XY = np.array([-0.66, 0.0])   # robot base (world frame, from the API docs)
SAFE_Z = 1.02                      # retreat height after a failed attempt


def detect_avg(api, name, n=8):
    """Average several independent noisy detections -> (bbox center, extents, yaw)."""
    sts = [api.detect(name) for _ in range(n)]
    yaws = [quat_to_rpy(s.quaternion)[2] for s in sts]
    yaw = np.arctan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
    return np.mean([s.bbox_center for s in sts], 0), sts[0].bbox_extents, yaw


def lift(api, dz=0.12):
    p, q = api.tcp_pose()
    api.move_to((p + [0, 0, dz], q))


def retreat(api):
    """Go up to a fixed safe height (relative lifts would accumulate over retries)."""
    p, q = api.tcp_pose()
    api.move_to((np.array([p[0], p[1], max(p[2], SAFE_Z)]), q))


def top_grasp(api):
    c, ext, yaw = detect_avg(api, OBJ)
    # close the fingers across the narrower horizontal side
    gyaw = yaw if ext[0] <= ext[1] else yaw + np.pi / 2
    gyaw = (gyaw + np.pi / 2) % np.pi - np.pi / 2
    q = rpy_to_quat(np.pi, 0.0, gyaw)  # pointing down
    top = c[2] + ext[2] / 2
    close_ax = np.array([np.cos(gyaw), np.sin(gyaw), 0.0])
    # spare finger opening on each side of the object; shift the grip centre away from the
    # nearest neighbour along the closing axis if the centred descent would hit it
    spare = max(0.0, (0.08 - min(ext[0], ext[1])) / 2 - 0.003)
    near = nearest_neighbour(api, c)
    sgn = -np.sign(close_ax[:2] @ (near.center[:2] - c[:2])) or 1.0
    api.open_gripper()
    # candidates: centred, shifted, then tilted so the wide hand leans away from the neighbour
    # and, for objects far from the base, approaches leaning forward (away from the robot) so
    # the arm need not stretch to a vertical wrist at table height
    radial = c[:2] - BASE_XY
    fsgn = np.sign(np.array([-np.sin(gyaw), np.cos(gyaw)]) @ radial) or 1.0
    cands = [(0.0, 0.0, 1, 0.0), (0.6 * spare, 0.0, 1, 0.0), (spare, 0.0, 1, 0.0),
             (0.6 * spare, 0.0, -1, 0.0), (spare, 0.0, -1, 0.0), (spare, 0.12, 1, 0.0),
             (spare, 0.2, 1, 0.0), (spare, 0.12, -1, 0.0),
             (0.0, 0.0, 1, 0.25), (0.0, 0.0, 1, 0.45), (0.0, 0.0, 1, 0.65)]
    for shift, tilt, side, fwd in cands:
        g = c + side * sgn * shift * close_ax
        if fwd:
            q = rpy_to_quat(np.pi + fsgn * fwd, 0.0, gyaw)
            tilt = fwd
        else:
            q = rpy_to_quat(np.pi, side * sgn * tilt, gyaw)
        if not api.move_to((np.array([g[0], g[1], top + 0.08]), q)):
            continue
        for z in np.arange(c[2], top - 0.002 + 0.02 * tilt, 0.002):
            api.move_to((np.array([g[0], g[1], z]), api.tcp_pose()[1]), linear=True, speed="slow",
                        allow_contact=[OBJ])
            if api.last_motion != "refused":
                break
        else:
            continue
        break
    else:
        return "blocked"
    api.close_gripper()
    return api.gripper_holding()


def neighbours(api):
    """Occupied boxes (every collision body, parts of fixtures included) except the object."""
    return [b for b in api.get_obstacles() if not b.name.startswith(OBJ)]


def nearest_neighbour(api, c):
    """The neighbouring box whose footprint is closest to c; returned with a .center."""
    return min(neighbours(api), key=lambda b: footprint_dist(c, b))


def footprint_dist(p, st):
    """Horizontal distance from point p to the footprint of an obstacle Box (0 inside)."""
    R = np.array(quat_to_R(st.quaternion))
    local = R.T @ (np.asarray(p, float) - st.center)
    half = st.extents / 2
    d = np.maximum(np.abs(local[:2]) - half[:2], 0.0)
    return float(np.linalg.norm(d))


def quat_to_R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def relocate_blocker(api, moved):
    """No collision-free grasp: the hand is wide along the finger-closing axis, so a tall
    neighbour beside the object blocks every top grasp. Pick the nearest small movable
    neighbour up by its rim/far side and set it down where it has the most free space."""
    c, ext, yaw = detect_avg(api, OBJ)
    cands = []
    for n in api.objects():
        if n == OBJ or n in moved:
            continue
        st = api.detect(n)
        if st is None or max(st.bbox_extents) > 0.15 or st.bbox_extents[2] > 0.12:
            continue  # fixtures and tall objects are not relocated
        cands.append((np.linalg.norm(st.bbox_center[:2] - c[:2]), n))
    if not cands:
        return False
    _, name = min(cands)
    nc, next_, _ = detect_avg(api, name)
    d = nc[:2] - c[:2]
    d = d / (np.linalg.norm(d) + 1e-9)
    # grasp the wall on the far side from the object, fingers closing across the wall
    rim = max(0.0, min(next_[0], next_[1]) / 2 - 0.012)
    offset = np.array([d[0] * rim, d[1] * rim, 0.0])
    ok = api.grasp(name, strategy="top", offset=offset + np.r_[nc[:2] - api.detect(name).bbox_center[:2], 0.0],
                   depth=0.02, yaw=float(np.arctan2(d[1], d[0])))
    if not ok:
        api.open_gripper()
        lift(api, 0.08)
        return False
    moved.append(name)
    p, q = api.tcp_pose()
    grip_h = p[2]
    lift(api, 0.10)
    # new spot: most free space first; the planner validates each one (the hand and the
    # carried object must fit there too)
    spots = []
    r = max(next_[0], next_[1]) / 2
    for dist in (0.08, 0.12, 0.16):
        for k in range(12):
            a = k * np.pi / 6
            spot = nc + dist * np.array([np.cos(a), np.sin(a), 0.0])
            free = min(footprint_dist(spot, b) for b in neighbours(api) if not b.name.startswith(name)) - r
            free = min(free, np.linalg.norm(spot[:2] - c[:2]) - r - 0.06)
            spots.append((free, dist, spot))
    spots.sort(key=lambda t: (-round(t[0], 2), t[1]))
    off = (p - nc) * np.array([1, 1, 0])  # keep the grasp offset
    for free, _, spot in spots[:8] + [(0.0, 0.0, nc)]:
        t = spot + off
        if api.move_to((np.array([t[0], t[1], grip_h + 0.10]), q)):
            break
    api.move_to((np.array([t[0], t[1], grip_h + 0.01]), q), linear=True, speed="slow")
    if api.last_motion == "refused":
        api.move_to((np.array([t[0], t[1], grip_h + 0.015]), q))
    api.open_gripper()
    lift(api, 0.10)
    return True


def planar_speed(api, name, window_s=2.0):
    """Least-squares horizontal speed of an object over a short tracked window (holds the arm
    still). Detection noise is ~1 cm per sample, so a 2 s window resolves ~3 mm/s."""
    hist = [(t, st.position) for t, st in api.track(name, window_s) if st is not None]
    t = np.array([h[0] for h in hist])
    P = np.array([h[1][:2] for h in hist])
    t = t - t.mean()
    v = (t[:, None] * (P - P.mean(0))).sum(0) / (t ** 2).sum()
    return float(np.linalg.norm(v))


def wait_until_still(api, name, max_s=60.0, thresh=0.006):
    """Do not grasp a moving object: wait (arm still) until it stops."""
    waited = 0.0
    while waited < max_s:
        if planar_speed(api, name) < thresh:
            return True
        waited += 2.0
    return False


def pick(api, moved):
    for _ in range(5):
        wait_until_still(api, OBJ)
        r = top_grasp(api)
        if r is True:
            lift(api, 0.12)
            if api.gripper_holding():
                return True
            r = False
        api.open_gripper()
        retreat(api)
        if r == "blocked" and len(moved) < 2:
            relocate_blocker(api, moved)
    return False


def run(api):
    moved = []
    for _ in range(3):
        if not pick(api, moved):
            return
        if place_in_container(api):
            return
        api.open_gripper()   # dropped on the way: pick it up again from where it fell
        retreat(api)


def carry_to(api, target, q):
    """Free-space move with the held object; if refused or stopped, retry via a waypoint
    between here and the target at a safe height."""
    if api.move_to((target, q)):
        return True
    if not api.gripper_holding():
        return False
    p = api.tcp_pose()[0]
    mid = (p + target) / 2
    mid[2] = max(p[2], target[2], SAFE_Z + 0.08)
    api.move_to((mid, q))
    return api.gripper_holding() and api.move_to((target, q))


def place_in_container(api, tol=0.02):
    """Carry the held object over the container and release it. The container is re-detected
    after every motion; if it moved, re-target (it may be pushed or sliding). If the hand does
    not fit over the container in the current yaw (something tall beside it), rotate the hand
    about the vertical and try again; never release while the descent was refused.
    Returns False if the object was lost on the way."""
    q0 = api.tcp_pose()[1]
    yaws = [pose_mul((np.zeros(3), rpy_to_quat(0.0, 0.0, d)), (np.zeros(3), q0))[1]
            for d in (0.0, np.pi / 2, np.pi / 4, -np.pi / 4)]
    c, ext, _ = detect_avg(api, BOWL)
    k, low = 0, False
    for _ in range(10):
        q = yaws[k]
        top = c[2] + ext[2] / 2
        if not carry_to(api, np.array([c[0], c[1], top + 0.10]), q):
            if not api.gripper_holding():
                return False
            k += 1
            if k == len(yaws):
                break
            continue
        c2 = detect_avg(api, BOWL)[0]
        if np.linalg.norm(c2[:2] - c[:2]) > tol:
            c = c2
            continue
        api.move_to((np.array([c[0], c[1], top + 0.025]), q), linear=True, speed="slow",
                    allow_contact=[BOWL])
        if api.last_motion == "refused":
            k += 1
            if k == len(yaws):
                break
            continue
        c3 = detect_avg(api, BOWL)[0]
        if np.linalg.norm(c3[:2] - c[:2]) > tol:
            c = c3
            lift(api, 0.06)
            continue
        low = True
        break
    if not api.gripper_holding():
        return False
    if not low:
        # last resort: get as low over the container as the planner allows, then release
        p, q = api.tcp_pose()
        if np.linalg.norm(p[:2] - c[:2]) > 0.03:
            return_to = np.array([c[0], c[1], p[2]])
            if not api.move_to((return_to, q)):
                api.move_to((return_to, yaws[0]))
    api.open_gripper()
    lift(api, 0.10)
    return True
