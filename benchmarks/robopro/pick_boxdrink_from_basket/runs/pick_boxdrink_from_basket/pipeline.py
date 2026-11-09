"""pick_boxdrink_from_basket: take the box drink out of the basket and stand it on the table.

Plan: average a few noisy detections of the box; grasp it near its centre of mass from the robot's side
(shallow approaches first, so the arm can still lift it high enough); lift until its bottom clears the
basket rim; pick a free table spot (clearance from every obstacle footprint, nothing hanging over it) on
the whole table; leave the basket towards the robot, then carry the box in straight segments at a height
clear of everything under and over its path (the planner does not see the held box); lower it upright and
release only at resting height.
"""

import numpy as np
import transforms3d as t3d

from api import gripper_quat

N_DETECT = 8            # detections averaged per estimate (1 cm / 3 deg noise each)
PRE_DIS = 0.09          # m, pre-grasp distance back along the approach: fingertips clear of the box
                        # even with a detection error of a centimetre
LIFT_CLEAR = 0.02       # m, box bottom above the basket rim before moving sideways
DROP = 0.01             # m, release height of the box bottom above the table
CARRY_CLEAR = 0.03      # m, carried object above (below) everything under (over) its path
# grasp heights as a fraction of the box height above its bottom: near the centre of mass first, so the
# box cannot swing in the fingers
HEIGHTS = (0.5, 0.62, 0.75)
ELEVATIONS = (0.35, 0.6, 0.2, 0.85)    # rad below horizontal; shallow first: the arm then reaches highest,
                                       # and the lift out of the container needs height
HEADINGS = (90, 120, 60)               # deg, approach heading for the left arm (90 = straight away from the
                                       # robot); mirrored for the right arm


def averaged(api, name, n=N_DETECT):
    states = [api.detect(name) for _ in range(n)]
    states = [s for s in states if s is not None]
    if not states:
        return None
    s = states[0]
    s.position = np.mean([x.position for x in states], axis=0)
    qs = np.array([x.quaternion for x in states])
    qs = np.array([q if np.dot(q, qs[0]) >= 0 else -q for q in qs])   # same hemisphere before averaging
    s.quaternion = qs.mean(axis=0) / np.linalg.norm(qs.mean(axis=0))
    return s


def world_extent(s, axis):
    """Extent of a detected object's box along a world axis (its box is in the object's own frame)."""
    R = t3d.quaternions.quat2mat(s.quaternion)
    return float(np.sum(np.abs(R[axis, :]) * s.bbox_extents))


def box_geometry(s):
    """(length, index of its long axis, rotation matrix) of a detected box. The long axis is the one closest
    to vertical; the box may lean, so use box_axis() for its actual direction."""
    R = t3d.quaternions.quat2mat(s.quaternion)
    k = int(np.argmax(np.abs(R[2, :])))
    return float(s.bbox_extents[k]), k, R


def box_axis(s):
    """Unit vector along the box's long axis, pointing up."""
    _, k, R = box_geometry(s)
    u = R[:, k].copy()
    return u if u[2] >= 0 else -u


def table_top(api):
    t = averaged(api, "table")
    return float(t.position[2] + world_extent(t, 2) / 2) if t is not None else 0.74


def grasp_candidates(box, arm):
    """[(pre (p, q), grasp (p, q), opening)] for grasping the upright box from the robot's side, fingers
    closing horizontally; lowest grasp points and shallowest approaches first."""
    height, k, R = box_geometry(box)
    u = box_axis(box)                       # the box may lean: grasp points lie on its actual axis
    out = []
    for frac in HEIGHTS:
        point = box.position + (frac - 0.5) * height * u
        for elev in ELEVATIONS:
            for heading in HEADINGS:
                psi = np.deg2rad(heading if arm == "left" else 180 - heading)
                approach = np.array([np.cos(elev) * np.cos(psi), np.cos(elev) * np.sin(psi), -np.sin(elev)])
                close = np.array([-np.sin(psi), np.cos(psi), 0.0])
                close = close - np.dot(close, u) * u        # fingers close across the box's axis
                if np.linalg.norm(close) < 0.3:
                    continue
                close /= np.linalg.norm(close)
                width = float(np.sum(np.abs(R.T @ close) * box.bbox_extents))
                q = gripper_quat(approach, close)
                out.append(((point - PRE_DIS * approach, q), (point, q),
                            float(np.clip((width + 0.03) / 0.09, 0.4, 1.0))))
    return out


def table_bounds(api):
    """(x_min, x_max, y_min, y_max) of the table top from its detected box."""
    t = averaged(api, "table")
    if t is None:
        return -0.6, 0.6, -0.35, 0.35
    ex, ey = world_extent(t, 0) / 2, world_extent(t, 1) / 2
    return t.position[0] - ex, t.position[0] + ex, t.position[1] - ey, t.position[1] + ey


def free_spots(api, near_x, table_z, half_xy, arm, height=0.25, min_clear=0.04, n=8):
    """Free table spots, nearest to an easy-reach point for `arm` first. A spot is free when the object's
    footprint keeps `min_clear` from every obstacle's footprint (nearby movable objects included) and
    nothing hangs over it lower than the object's height plus room for the hand and the carry (35 cm);
    the whole table is searched, not just a preferred area, since that area may be cluttered."""
    obstacles = [b for b in api.get_obstacles() if b.name != "boxdrink"]
    x0, x1, y0, y1 = table_bounds(api)
    margin = half_xy + 0.03                       # keep the object off the table edge
    xs = np.arange(x0 + margin, x1 - margin + 1e-9, 0.025)
    ys = np.arange(max(y0 + margin, -0.24), min(y1 - margin, 0.06) + 1e-9, 0.025)   # in reach of the arms
    want = np.array([np.clip(near_x, -0.40, -0.10) if arm == "left" else np.clip(near_x, 0.10, 0.40), -0.10])
    free = []
    for x in xs:
        for y in ys:
            clear = spot_clearance(obstacles, x, y, table_z, half_xy, height)
            if clear >= min_clear:
                free.append((np.hypot(x - want[0], y - want[1]), np.array([x, y])))
    free.sort(key=lambda t: t[0])
    # spread the candidates: skip spots within 6 cm of one already chosen
    out = []
    for _, p in free:
        if all(np.linalg.norm(p - q) > 0.06 for q in out):
            out.append(p)
        if len(out) >= n:
            break
    return out


def spot_clearance(obstacles, x, y, table_z, half_xy, height):
    """Clearance [m] between an object footprint (half size half_xy) at (x, y) and every obstacle footprint;
    an obstacle hanging over the spot lower than the object's height + 35 cm counts with 5 cm less."""
    clear = np.inf
    for b in obstacles:
        dx = max(0.0, abs(x - b.center[0]) - world_extent_box(b, 0) / 2 - half_xy)
        dy = max(0.0, abs(y - b.center[1]) - world_extent_box(b, 1) / 2 - half_xy)
        gap = np.hypot(dx, dy)
        bottom = b.center[2] - world_extent_box(b, 2) / 2
        if bottom > table_z + 0.05:               # hangs above the table (shelf, wall cabinet)
            if bottom > table_z + height + 0.35:  # high enough to pass under with the object
                continue
            gap = gap - 0.05                      # keep the hand out from under it as well
        clear = min(clear, gap)
    return clear


def wait_until_still(api, timeout_s=4.0, tol=0.025, box_name="boxdrink"):
    """Hold still until no obstacle has moved more than `tol` in 0.5 s (things may still be moving).
    Each snapshot averages 4 noisy detections."""
    def snapshot():
        acc = {}
        for _ in range(4):
            for b in api.get_obstacles():
                if b.name != box_name:
                    acc.setdefault(b.name, []).append(np.array(b.center, float))
        return {k: np.mean(v, axis=0) for k, v in acc.items()}
    t0 = api.sim_time()
    prev = snapshot()
    while api.sim_time() - t0 < timeout_s:
        api.wait_until(lambda: False, timeout_s=0.5)
        cur = snapshot()
        if all(np.linalg.norm(cur[k] - prev[k]) < tol for k in cur if k in prev):
            return True
        prev = cur
    return False


def world_extent_box(b, axis):
    """Extent of an obstacle Box along a world axis (its extents are in its own frame)."""
    R = t3d.quaternions.quat2mat(b.quaternion)
    return float(np.sum(np.abs(R[axis, :]) * b.extents))


def segment_hits(p0, p1, center, half):
    """True if the 2D segment p0-p1 passes through the rectangle centre +- half."""
    n = max(2, int(np.linalg.norm(p1 - p0) / 0.02) + 1)
    for t in np.linspace(0.0, 1.0, n):
        q = p0 + t * (p1 - p0)
        if abs(q[0] - center[0]) <= half[0] and abs(q[1] - center[1]) <= half[1]:
            return True
    return False


def carry_band(api, p0, p1, foot, table_z, ignore=("boxdrink",)):
    """(lowest bottom, highest top) a carried object may have while moving straight from p0 to p1 (xy):
    above everything standing under the swept path and below anything hanging over it."""
    lo, hi = table_z, np.inf
    for b in api.get_obstacles():
        if b.name in ignore:
            continue
        half = np.array([world_extent_box(b, 0) / 2 + foot, world_extent_box(b, 1) / 2 + foot])
        if not segment_hits(np.asarray(p0[:2], float), np.asarray(p1[:2], float), b.center[:2], half):
            continue
        hz = world_extent_box(b, 2) / 2
        if b.center[2] - hz > table_z + 0.30:     # hangs over the path (shelf, wall cabinet)
            hi = min(hi, b.center[2] - hz)
        else:
            lo = max(lo, b.center[2] + hz)
    return lo + CARRY_CLEAR, hi - CARRY_CLEAR


def carry(api, arm, spot, height, bottom_off, foot, table_z):
    """Move the held object (its bottom `bottom_off` below the TCP) straight to above `spot`: change height
    in place to a band clear of everything under and over the path, then cross at that height. The planner
    does not see the held object, so its free paths may sweep it through clutter."""
    p, q = api.tcp_pose(arm)
    lo, hi = carry_band(api, p[:2], spot, foot, table_z)
    if lo + height > hi:
        return False
    z = lo + bottom_off                             # as low as allowed: the arm reaches farther down there
    if not api.move_to(([p[0], p[1], z], q), arm, linear=True, allow_contact=["boxdrink"]):
        return False
    # a heavy load sags below the commanded height: measure and top up before crossing over obstacles
    sag = lo - lowest_point(api)
    if sag > 0.005:
        api.move_by(dz=sag + 0.005, arm=arm, linear=True, allow_contact=["boxdrink"])
    z = api.tcp_pose(arm)[0][2]
    return api.move_to(([spot[0], spot[1], z], q), arm, linear=True, allow_contact=["boxdrink"])


def place_upright(api, arm, spots, table_z, height, box_name="boxdrink"):
    """Stand the held box on one of `spots`: level gripper only (a tilted release topples a tall box),
    any yaw about the vertical, straight descent, release, back out along the approach, then up."""
    held = averaged(api, box_name)
    tcp_p, tcp_q = api.tcp_pose(arm)
    offset = held.position - tcp_p                      # box centre relative to the TCP (world frame)
    R0 = t3d.quaternions.quat2mat(tcp_q)
    for spot in spots:
        centre = np.array([spot[0], spot[1], table_z + height / 2 + DROP])
        for yaw in (0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.6, -1.6):
            Rz = t3d.axangles.axangle2mat([0, 0, 1], yaw)
            q = t3d.quaternions.mat2quat(Rz @ R0)
            g = centre - Rz @ offset
            if not api.move_to((g + [0, 0, 0.06], q), arm):
                continue
            if not api.move_to((g, q), arm, linear=True, allow_contact=[box_name]):
                # the release height was not reached: back up and try elsewhere, never drop it from the air
                api.move_to((g + [0, 0, 0.06], q), arm, linear=True, allow_contact=[box_name])
                continue
            gap = lowest_point(api, box_name) - (table_z + DROP)    # measured, not commanded (sag, slip)
            if gap > 0.01:
                api.move_by(dz=-gap, arm=arm, linear=True, allow_contact=[box_name])
            api.open_gripper(arm)
            approach = (Rz @ R0)[:, 0]
            api.move_by(*(-0.06 * approach), arm=arm, allow_contact=[box_name])
            api.move_by(dz=0.08, arm=arm, allow_contact=[box_name])
            return True
    return False


def grasp_box(api, arm):
    box = averaged(api, "boxdrink")
    if box is None:
        return None
    for pre, g, opening in grasp_candidates(box, arm):
        api.open_gripper(arm, opening)
        if not api.move_to(pre, arm):
            continue
        # re-detect from close by and shift the grasp by the correction
        again = averaged(api, "boxdrink")
        shift = np.r_[again.position[:2] - box.position[:2], 0.0]
        if not api.move_to((g[0] + shift, g[1]), arm, linear=True, allow_contact=["boxdrink"]):
            api.move_to(pre, arm, linear=True, allow_contact=["boxdrink"])
            continue
        api.close_gripper(arm)
        if api.gripper_holding(arm):
            return again
        api.open_gripper(arm, opening)
        api.move_to(pre, arm, linear=True, allow_contact=["boxdrink"])
    return None


def lowest_point(api, box_name="boxdrink"):
    b = averaged(api, box_name)
    return b.position[2] - world_extent(b, 2) / 2


def lift_clear(api, arm, clear_z, box_name="boxdrink", tries=4, extra=0.04):
    """Lift the held box until its MEASURED lowest point is above clear_z. A heavy load makes the arm sag
    below the commanded height, so measure and correct; if the arm runs out of reach, move 3 cm towards
    the robot (it reaches higher there) and continue. Corrections are capped at `extra` beyond the lift
    first needed: a box sliding down in the fingers must not be chased upwards."""
    start = api.tcp_pose(arm)[0][2]
    need0 = clear_z - lowest_point(api, box_name)
    moved_in = False
    for _ in range(tries):
        need = clear_z - lowest_point(api, box_name)
        if need <= 0.005:
            return True
        step = min(need + 0.005, need0 + extra - (api.tcp_pose(arm)[0][2] - start))
        if step <= 0.002:
            break
        if not api.move_by(dz=step, arm=arm, linear=True, allow_contact=[box_name]) and not moved_in:
            api.move_by(dy=-0.03, arm=arm, linear=True, allow_contact=[box_name])
            moved_in = True
    return clear_z - lowest_point(api, box_name) <= 0.01


def upright(api, arm, box_name="boxdrink", tol_deg=6.0):
    """Turn the held box upright in the hand (rotate the gripper about the grasp point so the box's long
    axis is vertical): it may have been grasped leaning, and it must stand when placed."""
    held = averaged(api, box_name)
    u = box_axis(held)
    tilt = float(np.degrees(np.arccos(np.clip(u[2], -1.0, 1.0))))
    if tilt < tol_deg:
        return True
    axis = np.cross(u, [0.0, 0.0, 1.0])
    axis /= np.linalg.norm(axis)
    Rc = t3d.axangles.axangle2mat(axis, np.radians(tilt))
    p, q = api.tcp_pose(arm)
    q_new = t3d.quaternions.mat2quat(Rc @ t3d.quaternions.quat2mat(q))
    # keep the box centre where it is: the TCP moves around it
    c = held.position
    p_new = c + Rc @ (p - c)
    return api.move_to((p_new, q_new), arm, allow_contact=[box_name])


def run(api):
    box = averaged(api, "boxdrink")
    if box is None:
        return
    arm = "left" if box.position[0] < 0 else "right"
    height, k, _ = box_geometry(box)
    horiz = [i for i in range(3) if i != k]
    held = grasp_box(api, arm)
    if held is None:
        return

    # lift straight up until the box bottom clears the basket rim; if the arm runs out of reach, move the
    # box a little towards the robot inside the basket (the arm reaches higher there) and lift again
    basket = averaged(api, "basket", 4)
    rim = basket.position[2] + world_extent(basket, 2) / 2 if basket is not None else held.position[2]
    if not lift_clear(api, arm, rim + LIFT_CLEAR):
        return                                   # never drag the box through the basket wall

    table_z = table_top(api)
    foot = float(np.hypot(*[held.bbox_extents[i] / 2 for i in horiz]))      # half-diagonal of the footprint
    spots = free_spots(api, held.position[0], table_z, max(held.bbox_extents[i] for i in horiz) / 2, arm,
                       height=height)
    if not spots:
        return
    # leave the basket towards the robot at this height, until the box is clear of the basket's footprint
    if basket is not None:
        front = basket.position[1] - world_extent(basket, 1) / 2
        p, _ = api.tcp_pose(arm)
        dy = min(0.0, front - foot - 0.03 - held.position[1])
        api.move_by(dy=dy, arm=arm, linear=True, allow_contact=["boxdrink"])
    now = averaged(api, "boxdrink")
    bottom_off = api.tcp_pose(arm)[0][2] - (now.position[2] - world_extent(now, 2) / 2)   # TCP above box bottom
    half_xy = max(held.bbox_extents[i] for i in horiz) / 2
    for spot in spots:
        if carry(api, arm, spot, height, bottom_off, foot, table_z):
            # a box grasped leaning is turned upright here, over free table, lower than the lift height
            # (the arm reaches the rotated pose there); try a little higher if it is out of reach
            if not upright(api, arm):
                api.move_by(dz=0.05, arm=arm, linear=True, allow_contact=["boxdrink"])
                upright(api, arm)
            # the scene may have changed during the carry: let it come to rest, re-check the spot, and if it
            # is taken now, choose again from here (a few times)
            for _ in range(3):
                wait_until_still(api)
                if spot_clearance([b for b in api.get_obstacles() if b.name != "boxdrink"], spot[0], spot[1],
                                  table_z, half_xy, height) >= 0.03:
                    break
                fresh = free_spots(api, held.position[0], table_z, half_xy, arm, height=height)
                if not fresh or not carry(api, arm, fresh[0], height, bottom_off, foot, table_z):
                    return
                spot = fresh[0]
            if place_upright(api, arm, [spot], table_z, height):
                api.home(arm)
            return
