"""PegInsertionSide-v1: grasp the peg near its tail, align it with the hole, insert from the side.

Uses only the RobotAPI. Every detection is noisy (about 1 cm, 3 deg per call, independent),
while the hole clearance is 3 mm, so every pose the pipeline acts on is an average of many
detections, and the in-hand peg pose is re-measured after grasping.
"""

import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat

from api import pose_inv, pose_mul

N_AVG = 300          # detections averaged per pose estimate (noise / sqrt(N))
PRE_GAP = 0.015      # peg head stops this far in front of the box face before insertion
LIFT = 0.12          # lift height after grasp [m]
INSERT_PAST = 0.01   # target the peg head this far past the hole centre
N_FINE = 1000        # detections per in-hand measurement during the final alignment
INSERT_STEP = 0.02   # insertion step between re-alignments [m]
MOVED_TOL = 0.005    # re-plan the grasp if the peg moved more than this since it was planned
SETTLE_S = 0.3       # let the arm come to rest before measuring
CLEAR = 0.01         # required gap between the hand / fingers and any other object at the grasp
SHIFT = 0.10         # how far to set the peg aside when no tail grasp is free of obstacles
SHIFT_IN = 0.15      # how far to bring the peg toward the workspace centre when it is out of reach
TASK_OBJECTS = ("peg", "box_with_hole")
# Robot model (not perception): where the arm is mounted and how far from it the top-down hand still
# moves well. Near the reach limit (~0.85 m) poses are reached only through long detours, if at all.
ROBOT_BASE_XY = np.array([-0.615, 0.0])
REACH_COMFORT = 0.75

# Hand geometry in the grasp frame (x = along the peg, y = closing, z = approach; TCP at the origin)
# as boxes (centre, half extents): the two fingers (open), the hand body above them, and the wrist
# and forearm that rise above the hand when it grasps top-down.
HAND_BOXES = [
    (np.array([0.0, 0.05, -0.022]), np.array([0.012, 0.012, 0.025])),
    (np.array([0.0, -0.05, -0.022]), np.array([0.012, 0.012, 0.025])),
    (np.array([0.0, 0.0, -0.075]), np.array([0.03, 0.11, 0.03])),
    (np.array([0.0, 0.0, -0.225]), np.array([0.05, 0.05, 0.12])),
]


def _avg_quat(qs):
    qs = np.asarray(qs, dtype=np.float64)
    ref = qs[0]
    qs = np.where((qs @ ref)[:, None] < 0, -qs, qs)
    q = qs.mean(axis=0)
    return q / np.linalg.norm(q)


def _avg(samples):
    ps = np.array([s[0] for s in samples])
    return ps.mean(axis=0), _avg_quat([s[1] for s in samples])


def detect_avg(api, name, n=N_AVG):
    sts = [api.detect(name) for _ in range(n)]
    p, q = _avg([(s.position, s.quaternion) for s in sts])
    return (p, q), np.asarray(sts[0].bbox_extents, dtype=np.float64)


def frame_avg(api, key, n=N_AVG):
    return _avg([api.task_frames()[key] for _ in range(n)])


def _rot(q):
    return quat2mat(np.asarray(q, dtype=np.float64))


def _pose_from_axes(x, y, z, p):
    return np.asarray(p, dtype=np.float64), mat2quat(np.stack([x, y, z], axis=1))


def wait_static(api, name, n=200, dt=0.25, tol=0.004, timeout_s=6.0):
    """Averaged pose of `name` once two consecutive estimates dt apart agree within tol."""
    prev, ext = detect_avg(api, name, n)
    t = 0.0
    while t < timeout_s:
        api.wait_until(lambda: False, dt)
        t += dt
        cur, ext = detect_avg(api, name, n)
        if np.linalg.norm(cur[0] - prev[0]) < tol:
            return cur, ext
        prev = cur
    return prev, ext


def other_obstacles(api, n=50):
    """Averaged boxes of every object that is not part of the task (clutter, fixtures)."""
    names = [o for o in api.objects() if o not in TASK_OBJECTS]
    if not names:
        return []
    samples = [[b for b in api.get_obstacles() if b.name in names] for _ in range(n)]
    out = []
    for i, b in enumerate(samples[0]):
        p, q = _avg([(s[i].center, s[i].quaternion) for s in samples])
        out.append((b.name, p, q, b.extents))
    return out


def _inside(points, obstacles, margin):
    for _, bp, bq, bext in obstacles:
        local = (np.asarray(points) - bp) @ _rot(bq)
        if np.any(np.all(np.abs(local) < bext / 2 + margin, axis=1)):
            return True
    return False


def hand_clear(grasp, obstacles):
    """True if the open hand at `grasp` keeps CLEAR from every obstacle box (sampled corner/edge points)."""
    for _, bp, bq, bext in obstacles:
        to_box = pose_mul(pose_inv((bp, bq)), grasp)
        R, t = _rot(to_box[1]), np.asarray(to_box[0])
        for c, h in HAND_BOXES:
            g = np.stack(np.meshgrid(*[np.linspace(-1, 1, 5)] * 3), -1).reshape(-1, 3) * h + c
            pts = g @ R.T + t
            if np.any(np.all(np.abs(pts) < bext / 2 + CLEAR, axis=1)):
                return False
    return True


def plan_grasps(api, peg, ext, obstacles=(), offsets=None):
    """Grasps from above, square to the peg's measured 3D axis (it need not lie flat), at the given
    offsets behind its centre (default: near the tail, preferred first) where the hand is clear of
    the obstacles; yields (pregrasp, grasp)."""
    half_len = float(ext.max()) / 2
    peg_x = _rot(peg[1])[:, 0]                   # +x points from tail to head
    down = np.array([0.0, 0.0, -1.0])
    approach = down - (down @ peg_x) * peg_x     # straight down, made perpendicular to the axis
    approach /= np.linalg.norm(approach)
    closing = np.cross(approach, peg_x)
    closing /= np.linalg.norm(closing)
    tcp_y = _rot(api.tcp_pose()[1])[:, 1]
    if closing @ tcp_y < 0:
        closing = -closing
    ortho = np.cross(closing, approach)
    if offsets is None:
        # The hand needs ~5 cm from the box face at full depth: tail grasps only, preferred first.
        pref = max(0.05, half_len / 2 + 0.01)
        offsets = [pref] + [o for o in np.arange(0.05, half_len - 0.015, 0.01) if abs(o - pref) > 1e-6]
    for tail_off in offsets:
        center = np.asarray(peg[0]) - tail_off * peg_x
        grasp = _pose_from_axes(ortho, closing, approach, center)
        if hand_clear(grasp, obstacles):
            yield pose_mul(grasp, ([0, 0, -0.08], [1, 0, 0, 0])), grasp


def reach_grasp(api, plans, avoid, tries=1):
    """Move to the pre-grasp of the first of the first `tries` plans the arm can reach; returns that
    plan or None. Few tries: a far-fetched plan that the planner does reach takes a long detour
    and leaves the arm in an awkward configuration."""
    for _, (pregrasp, grasp) in zip(range(tries), plans):
        if api.move_to(pregrasp, avoid=avoid):
            return pregrasp, grasp
    return None


def set_aside(api, peg, ext, obstacles, avoid, home):
    """No tail grasp is free or reachable: pick the peg up wherever the hand fits and reaches, put it
    down SHIFT away (from the obstacles if there are any, else toward the workspace centre `home`),
    and release it."""
    half_len = float(ext.max()) / 2
    peg_x = _rot(peg[1])[:, 0]
    # Grasp points nearest the workspace centre first: they are the most likely to be reachable.
    near = lambda o: np.linalg.norm((np.asarray(peg[0]) - o * peg_x - np.asarray(home))[:2])
    plan = reach_grasp(api, plan_grasps(api, peg, ext, obstacles, offsets=sorted(
        np.arange(-half_len + 0.015, half_len - 0.015, 0.01), key=near)), avoid, tries=3)
    if plan is None or not api.move_to(plan[1], speed="slow"):
        return False
    grasp = plan[1]
    api.close_gripper()
    shift = put_down_shift(api, peg, ext, obstacles, home, _rot(grasp[1])[:, 1], peg_x)
    up = np.array([0, 0, 0.05])
    api.move_to((np.asarray(grasp[0]) + up, grasp[1]), speed="slow")
    dest = np.asarray(grasp[0]) + shift
    api.move_to((dest + up, grasp[1]), avoid=avoid)
    api.move_to((dest + np.array([0, 0, 0.005]), grasp[1]), speed="slow")
    api.open_gripper()
    api.move_to((dest + np.array([0, 0, 0.10]), grasp[1]), avoid=avoid)
    return True


def put_down_shift(api, peg, ext, obstacles, home, side, peg_x):
    """Where to put the peg down: the first candidate shift (toward the workspace centre, sideways,
    along the peg) after which the peg itself fits, and its preferred grasp is free and comfortably
    reachable. The box with the hole counts as an obstacle for the peg here."""
    half_len, r = float(ext.max()) / 2, float(np.sort(ext)[1]) / 2
    fixtures = obstacles + [(n, *_avg([(b.center, b.quaternion) for b in bs]), bs[0].extents)
                            for n, bs in [("box_with_hole", [b for _ in range(50) for b in api.get_obstacles()
                                                             if b.name == "box_with_hole"])] if bs]
    to_home = np.append((np.asarray(home) - np.asarray(peg[0]))[:2], 0.0)
    to_home /= np.linalg.norm(to_home)
    flat = lambda v: np.append(v[:2] / np.linalg.norm(v[:2]), 0.0)
    for d in (SHIFT, SHIFT_IN):
        for u in (to_home, flat(side), -flat(side), flat(peg_x), -flat(peg_x)):
            p = np.asarray(peg[0]) + d * u
            body = [p + t * np.asarray(peg_x) for t in np.linspace(-half_len, half_len, 9)]
            if _inside(body, fixtures, r + CLEAR):
                continue
            g = next(plan_grasps(api, (p, peg[1]), ext, obstacles), None)
            if g is not None and np.linalg.norm(g[1][0][:2] - ROBOT_BASE_XY) <= REACH_COMFORT:
                return d * u
    return SHIFT * to_home


def grasp_peg(api, attempts=3):
    """Grasp with re-perception: the peg may move between detection and contact, so its pose is
    re-measured (after it comes to rest) at the pre-grasp pose and the grasp is re-planned if it moved.
    If no tail grasp is free of obstacles and reachable, the peg is first moved somewhere better."""
    home = api.tcp_pose()[0]
    obstacles = other_obstacles(api)
    avoid = [o[0] for o in obstacles] or None
    for _ in range(attempts):
        api.open_gripper()
        peg, ext = wait_static(api, "peg")
        plans = plan_grasps(api, peg, ext, obstacles)
        first = next(plans, None)
        if first is not None and np.linalg.norm(first[1][0][:2] - ROBOT_BASE_XY) > REACH_COMFORT:
            # Near the edge of the workspace: bring the peg closer before the real grasp.
            set_aside(api, peg, ext, obstacles, avoid, home)
            continue
        plan = reach_grasp(api, [first] if first is not None else [], avoid)
        if plan is None:
            # Something stands where the hand has to go, or the arm cannot reach: move the peg first.
            set_aside(api, peg, ext, obstacles, avoid, home)
            continue
        pregrasp, grasp = plan
        for _ in range(3):
            peg2, ext = wait_static(api, "peg")
            if np.linalg.norm(peg2[0] - peg[0]) < MOVED_TOL:
                break
            peg = peg2
            plan = reach_grasp(api, plan_grasps(api, peg, ext, obstacles), avoid)
            if plan is None:
                break
            pregrasp, grasp = plan
        if plan is None:
            continue
        if not api.move_to(grasp, speed="slow"):
            # The pre-grasp was reachable but the grasp itself is not: move the peg first.
            set_aside(api, peg, ext, obstacles, avoid, home)
            continue
        api.close_gripper()
        lift = pose_mul(([0, 0, LIFT], [1, 0, 0, 0]), api.tcp_pose())
        api.move_to(lift, avoid=avoid)
        if api.gripper_holding():
            return ext
        api.open_gripper()
    return None


def run(api):
    # ---------------------------------------------------------------- grasp near the tail, top-down
    ext = grasp_peg(api)
    if ext is None:
        return
    half_len = float(ext.max()) / 2

    # ---------------------------------------------------------------- align with the hole
    hole = frame_avg(api, "box_hole")

    def peg_in_tcp():
        peg_now, _ = detect_avg(api, "peg")
        return pose_mul(pose_inv(api.tcp_pose()), peg_now)

    def peg_axis_target(x_head):
        """Peg pose on the hole axis with its head at x_head in the hole frame (0 = hole centre)."""
        return pose_mul(hole, ([x_head - half_len, 0, 0], [1, 0, 0, 0]))

    def servo(peg_target, cmd, iters, tol_yz=0.0008):
        """Correct the TCP command by the measured peg error until the peg is at peg_target.
        The arm lags behind its commands and the peg may shift in the fingers, so the command,
        not the measured TCP pose, is what gets corrected; the arm settles before each measurement."""
        for _ in range(iters):
            api.wait_until(lambda: False, SETTLE_S)
            peg_now, _ = detect_avg(api, "peg", N_FINE)
            err = pose_mul(pose_inv(peg_target), peg_now)
            # Check both ends of the peg, so a tilt counts as misalignment, not just an offset.
            ends = [pose_mul(err, ([sx * half_len, 0, 0], [1, 0, 0, 0]))[0] for sx in (1, -1)]
            if max(np.linalg.norm(e[1:]) for e in ends) < tol_yz and abs(err[0][0]) < 0.003:
                break
            cmd = pose_mul(pose_mul(peg_target, pose_inv(peg_now)), cmd)
            if not api.move_to(cmd, speed="slow"):
                return None
        return cmd

    # Pre-insert: peg on the hole axis, head PRE_GAP in front of the box face (face at x = -half_len).
    pre_peg = peg_axis_target(-half_len - PRE_GAP)
    cmd = pose_mul(pre_peg, pose_inv(peg_in_tcp()))
    if not api.move_to(cmd):
        return
    cmd = servo(pre_peg, cmd, 8)
    if cmd is None:
        return

    # ---------------------------------------------------------------- insert along the hole axis
    # Small steps with re-alignment: a single long move lags off the axis and jams on the rim.
    x, prev = -half_len - PRE_GAP, pre_peg
    while x < INSERT_PAST - 1e-6:
        x = min(x + INSERT_STEP, INSERT_PAST)
        target = peg_axis_target(x)
        cmd = pose_mul(pose_mul(target, pose_inv(prev)), cmd)   # shift the command along the axis
        prev = target
        api.move_to(cmd, speed="slow")
        cmd = servo(target, cmd, 2)
        if cmd is None:
            return
