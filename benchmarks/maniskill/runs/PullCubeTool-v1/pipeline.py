"""PullCubeTool-v1: grasp the L-shaped tool, hook it behind the cube, pull the cube toward the base."""

import os
import sys

import numpy as np

from api import pose_inv, pose_mul, rpy_to_quat

# Tool geometry in its own frame (origin at the free end of the handle, handle along +x,
# hook at the far end pointing +y).
HANDLE_LEN = 0.20
HOOK_LEN = 0.05      # hook spans tool x in [HANDLE_LEN - HOOK_LEN, HANDLE_LEN]
HANDLE_HALF_W = 0.025
HOOK_REACH = 0.10    # hook spans tool y in [0, HOOK_REACH]
CUBE_HALF = 0.02
GRIP_AT = 0.025      # grasp point along the handle from its free end (short lever arm to the hook = more reach)
BASE_XY = np.array([-0.615, 0.0])   # robot base position on the table
REACH = 0.80                        # max horizontal TCP distance from the base near table height
DOWN = np.array([0.0, 1.0, 0.0, 0.0])  # TCP pointing down, fingers closing along world y


def dbg(*a):
    if os.environ.get("GAH_DEBUG"):
        print(*a, file=sys.stderr)


def q2R(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R2q(R):
    w = np.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-3:
        return np.array([w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    v = np.zeros(3)
    v[i] = np.sqrt(max(1e-12, 1 + R[i, i] - R[j, j] - R[k, k])) / 2
    v[j] = (R[j, i] + R[i, j]) / (4 * v[i])
    v[k] = (R[k, i] + R[i, k]) / (4 * v[i])
    w = (R[k, j] - R[j, k]) / (4 * v[i])
    q = np.r_[w, v]
    return q if q[0] >= 0 else -q


def detect_avg(api, name, n=15):
    """Median position and chordal-mean full rotation of n noisy detections."""
    states = [api.detect(name) for _ in range(n)]
    p = np.median([s.position for s in states], axis=0)
    M = np.mean([q2R(s.quaternion) for s in states], axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    return p, R


def wait_still(api, name, window_s=0.5, tol=0.012, timeout_s=8.0):
    """Hold the arm until two consecutive half-windows of detections agree within tol."""
    t = 0.0
    while t < timeout_s:
        hist = [st.position for _, st in api.track(name, window_s)]
        h = len(hist) // 2
        t += window_s
        if np.linalg.norm(np.median(hist[:h], axis=0) - np.median(hist[h:], axis=0)) < tol:
            return True
    return False


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def box_aabb(o, R, lo, hi):
    """World AABB (lo_xy, hi_xy, z_lo, z_hi) of the box [lo, hi] given in a frame at (o, R)."""
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = o + corners @ R.T
    return w.min(0)[:2], w.max(0)[:2], w.min(0)[2], w.max(0)[2]


# Tool parts and the gripper (hand + fingers) as boxes; tool parts in the tool frame, gripper in
# the hand frame (origin at the TCP, z away from the fingers, y along the closing direction).
TOOL_BOXES = [([0, -HANDLE_HALF_W, -0.025], [HANDLE_LEN, HANDLE_HALF_W, 0.025]),
              ([HANDLE_LEN - HOOK_LEN, 0, -0.025], [HANDLE_LEN, HOOK_REACH, 0.025])]
HAND_BOXES = [([-0.035, -0.105, 0.035], [0.035, 0.105, 0.30]),
              ([-0.013, -0.05, -0.012], [0.013, 0.05, 0.035])]
OPEN_FINGERS = [([-0.013, -0.065, -0.012], [0.013, 0.065, 0.035])]
GRIP_CANDIDATES = [0.025, 0.045, 0.065, 0.085, 0.105, 0.125]   # along the handle, before the hook


def scene_obstacles(api, exclude, n=10):
    """Averaged xy-AABB and height range of every other object: [(name, lo_xy, hi_xy, z_lo, z_hi)]."""
    acc = {}
    for _ in range(n):
        for b in api.get_obstacles():
            if b.name not in exclude:
                acc.setdefault(b.name, []).append(b)
    out = []
    for name, boxes in acc.items():
        c = np.median([b.center for b in boxes], axis=0)
        R = np.mean([q2R(b.quaternion) for b in boxes], axis=0)
        half = np.abs(R) @ (boxes[0].extents / 2)       # AABB half extents of the oriented box
        out.append((name, c[:2] - half[:2], c[:2] + half[:2], c[2] - half[2], c[2] + half[2]))
    return out


def footprint_hits(rects, obstacles, margin):
    """rects: [(lo_xy, hi_xy, z_lo, z_hi)] world AABBs. True if any overlaps an obstacle (+margin)."""
    for lo, hi, zl, zh in rects:
        for _, olo, ohi, ozl, ozh in obstacles:
            if (np.all(lo < ohi + margin) and np.all(hi > olo - margin) and zl < ozh + margin and zh > ozl):
                return True
    return False


def down_q(yaw):
    return pose_mul(([0, 0, 0], rpy_to_quat(0, 0, yaw)), ([0, 0, 0], DOWN))[1]


def run(api):
    tool_p, tool_R = detect_avg(api, "l_shape_tool")
    cube_p, cube_R = detect_avg(api, "cube")
    # Handle direction (tool +x) in the table plane; the hook direction comes from the full
    # rotation, because the tool may lie flipped over (hook pointing the other way).
    tool_yaw = np.arctan2(tool_R[1, 0], tool_R[0, 0])
    cube_yaw = np.arctan2(cube_R[1, 0], cube_R[0, 0])

    # ---- grasp the handle top-down, closing across it, at the point nearest its free end whose
    # approach volume (hand + open fingers, from 10 cm above down to the grasp) is free of clutter.
    # The grasp height comes from the perceived tool pose, never from an assumed table height.
    clutter = scene_obstacles(api, exclude={"l_shape_tool"})

    def grasp_xy(p, yaw, g):
        return p[:2] + g * np.array([np.cos(yaw), np.sin(yaw)])

    def approach_clear(p, yaw, g):
        xy = grasp_xy(p, yaw, g)
        for h in np.linspace(0.0, 0.10, 6):
            tcp = np.r_[xy, p[2] + h]
            rects = [box_aabb(tcp, Rz(yaw), lo, hi) for lo, hi in HAND_BOXES[:1] + OPEN_FINGERS]
            if footprint_hits(rects, clutter, 0.01):
                return False
        return True

    def grasp_tool(p, yaw, g):
        xy, q = grasp_xy(p, yaw, g), down_q(yaw)
        api.open_gripper()
        api.move_to((np.r_[xy, p[2] + 0.10], q))
        api.move_to((np.r_[xy, p[2]], q), speed="slow")
        api.close_gripper()
        return xy, q

    lift_z = max(0.15, tool_p[2] + 0.12)
    grip_z = tool_p[2]
    g = next((g for g in GRIP_CANDIDATES if approach_clear(tool_p, tool_yaw, g)), GRIP_AT)
    dbg("grasp at", g)
    grip_xy, q = grasp_tool(tool_p, tool_yaw, g)
    if not api.gripper_holding():
        return
    api.move_to((np.r_[grip_xy, lift_z], q), speed="slow")

    if g > GRIP_AT + 1e-6:
        # Grasped away from the preferred point (that costs reach later): set the tool down in a
        # free spot on the table and regrasp it at the preferred point.
        for d in [(0.0, 0.12), (0.0, -0.12), (0.12, 0.0), (0.12, 0.12), (0.12, -0.12), (0.0, 0.22), (0.0, -0.22)]:
            spot = np.r_[tool_p[:2] + d, tool_p[2]]   # same support surface as before
            corners = [box_aabb(spot, tool_R, lo, hi) for lo, hi in TOOL_BOXES]
            if not footprint_hits(corners, clutter, 0.02) and approach_clear(spot, tool_yaw, GRIP_AT):
                break
        hand_xy = grasp_xy(spot, tool_yaw, g)
        api.move_to((np.r_[hand_xy, lift_z], q), speed="slow")
        api.move_to((np.r_[hand_xy, spot[2] + 0.005], q), speed="slow")
        api.open_gripper()
        api.move_to((np.r_[hand_xy, lift_z], q))
        tool_p, tool_R = detect_avg(api, "l_shape_tool")
        tool_yaw = np.arctan2(tool_R[1, 0], tool_R[0, 0])
        grip_z = tool_p[2]
        grip_xy, q = grasp_tool(tool_p, tool_yaw, GRIP_AT)
        if not api.gripper_holding():
            return
        api.move_to((np.r_[grip_xy, lift_z], q), speed="slow")

    # The tool can pivot or slip in the fingers whenever it is carried, so its pose in the hand is
    # re-measured after every carry, and the hand is oriented so that the TOOL ends up level.
    def tool_in_hand():
        p, R = detect_avg(api, "l_shape_tool", n=20)
        return pose_mul(pose_inv(api.tcp_pose()), (p, R2q(R))), R

    tcp_T_tool, tool_R2 = tool_in_hand()
    # Test lift: a heavy tool held near its end pivots in the fingers (gravity torque > grip
    # friction). Then it cannot be carried; put it back down and slide it on the table instead.
    hang = np.degrees(np.arcsin(min(1.0, abs(tool_R2[2, 0]))))
    carry = hang < 12.0
    dbg("hang", round(hang, 1), "carry", carry)
    if not carry:
        api.move_to((np.r_[grip_xy, grip_z + 0.01], q), speed="slow")
        tcp_T_tool, tool_R2 = tool_in_hand()
    # Level target orientation, handle along world +x; the hook points +y (upright) or -y (flipped).
    hook_sign = 1.0 if tool_R2[2, 2] > 0 else -1.0
    R_target = np.diag([1.0, hook_sign, hook_sign])

    # Re-perceive the cube right before acting on it: it may have moved while the tool was fetched.
    wait_still(api, "cube")
    cube_p, cube_R = detect_avg(api, "cube", n=20)
    cube_yaw = np.arctan2(cube_R[1, 0], cube_R[0, 0])

    # Work in the radial frame of the robot: u points from the base to the cube, v is
    # perpendicular. The tool is laid along u and the pull goes straight toward the base, which
    # keeps the hand as close to the base as possible (reach) and pulls straight at the goal.
    u = (cube_p[:2] - BASE_XY) / np.linalg.norm(cube_p[:2] - BASE_XY)
    v = np.array([-u[1], u[0]])
    psi = np.arctan2(u[1], u[0])
    U, V = np.r_[u, 0.0], np.r_[v, 0.0]
    c = abs(np.cos(cube_yaw - psi)) + abs(np.sin(cube_yaw - psi))
    half_ext = CUBE_HALF * max(c, 1.0)      # cube half extent along u and v
    cube_half_box = CUBE_HALF * np.sqrt(2)  # conservative world AABB half size of the cube
    obstacles = scene_obstacles(api, exclude={"cube", "l_shape_tool"})
    dx = np.linalg.norm(cube_p[:2] - BASE_XY) - 0.45   # pull until the cube is 0.45 m from the base
    side = hook_sign      # the handle face can push the cube sideways toward the hook side (+v)
    gap_y = 0.015
    tcp_in_tool, q_tool_tcp = pose_inv(tcp_T_tool)

    def placement(pitch):
        """Tool rotation and origin that put the hook just beyond the cube with the hook's lower
        edge 4 mm above the table; pitch > 0 tilts the hook end down and raises the hand."""
        R = Rz(psi) @ Ry(pitch) @ R_target
        along_u = half_ext + 0.018 - (HANDLE_LEN - HOOK_LEN) * np.cos(pitch) + 0.025 * np.sin(pitch)
        along_v = -hook_sign * (half_ext + HANDLE_HALF_W + gap_y)
        lowest = min(box_aabb(np.zeros(3), R, lo, hi)[2] for lo, hi in TOOL_BOXES)
        return R, np.r_[cube_p[:2] + along_u * u + along_v * v, 0.004 - lowest]

    def footprint(R, o, cxy):
        rects = [box_aabb(o, R, lo, hi) for lo, hi in TOOL_BOXES]
        tcp = o + R @ tcp_in_tool
        R_hand = R @ q2R(q_tool_tcp) @ np.diag([1.0, -1.0, -1.0])
        rects += [box_aabb(tcp, R_hand, lo, hi) for lo, hi in HAND_BOXES]
        rects.append((cxy - cube_half_box, cxy + cube_half_box, 0.0, 0.045))
        return rects

    def reachable(R, o):
        tcp = o + R @ tcp_in_tool
        return np.linalg.norm(tcp[:2] - BASE_XY) <= REACH

    def path_clear(R, o, lateral):
        pts = [(o + [0, 0, h], cube_p[:2]) for h in np.linspace(lift_z - o[2], 0, 6)] if carry else []
        for t in np.linspace(0, 1, int(lateral / 0.01) + 2):
            l = lateral * t
            pts.append((o + side * l * V, cube_p[:2] + side * max(0.0, l - gap_y) * v))
        cxy = cube_p[:2] + side * max(0.0, lateral - gap_y) * v
        for t in np.linspace(0, 1, int(dx / 0.01) + 2):
            pts.append((o + side * lateral * V - dx * t * U, cxy - dx * t * u))
        for oo, cc in pts:
            fp = footprint(R, oo, cc)
            # 7 mm around tool and cube (they move along known straight lines), 1.5 cm around the hand
            if footprint_hits(fp[:2] + fp[-1:], obstacles, 0.007) or footprint_hits(fp[2:-1], obstacles, 0.015):
                return False
        return reachable(R, o + side * lateral * V)

    # Choose the least unusual plan whose whole swept footprint (tool, cube, hand) is collision-free:
    # level tool first, then a tilted tool (hand higher); straight pull first, then a sideways detour.
    plan = None
    for pitch in np.deg2rad([0.0, 15.0, 22.0] if carry else [0.0]):
        R, o = placement(pitch)
        lateral = next((l for l in np.arange(0.0, 0.31, 0.02) if path_clear(R, o, l)), None)
        if lateral is not None:
            plan = (R, o, lateral)
            break
    if plan is None:
        plan = (*placement(0.0), 0.0)
    R, o, lateral = plan
    dbg("obstacles", obstacles, "plan R/origin/lateral", np.round(R, 2).tolist(), np.round(o, 3), lateral)

    def tcp_for(o_tool):
        return pose_mul((o_tool, R2q(R)), pose_inv(tcp_T_tool))

    def move_tool_straight(a, b, step=0.04):
        """Contact moves must be straight lines: split into short hops, each planned as a straight
        (screw) motion, so a planner fallback can never take a wild detour while touching things."""
        n = max(1, int(np.ceil(np.linalg.norm(np.asarray(b) - a) / step)))
        for t in np.linspace(0, 1, n + 1)[1:]:
            api.move_to(tcp_for(a + (np.asarray(b) - a) * t), speed="slow")

    if carry:
        # ---- carry the tool above its placement, re-measure it in the hand, lower it beside the cube
        above = o + [0, 0, lift_z - o[2]]
        api.move_to(tcp_for(above), speed="slow")
        tcp_T_tool, _ = tool_in_hand()
        api.move_to(tcp_for(above), speed="slow")
        api.move_to(tcp_for(o), speed="slow")
    else:
        # ---- slide the tool on the table: to a pre-hook pose offset away from the hook side (hook
        # clear of the cube), then sideways into the hooking pose. The cube is an obstacle here.
        o_now = detect_avg(api, "l_shape_tool", n=20)[0]
        o_now[2] = o[2]
        pre = o - side * 0.09 * V
        cube_box = ("cube", cube_p[:2] - cube_half_box, cube_p[:2] + cube_half_box, 0.0, 0.045)

        def leg_clear(a, b):
            for t in np.linspace(0, 1, int(np.linalg.norm(b - a) / 0.01) + 2):
                fp = footprint(R, a + (b - a) * t, cube_p[:2])
                if footprint_hits(fp[:2], obstacles + [cube_box], 0.007) or footprint_hits(fp[2:-1], obstacles, 0.015):
                    return False
            return reachable(R, b)

        d = pre - o_now
        corners = [[], [o_now + (d @ V) * V], [o_now + (d @ U) * U]]
        route = next((c for c in corners if all(leg_clear(a, b) for a, b in zip([o_now] + c, c + [pre]))), corners[1])
        dbg("slide route", [np.round(w, 3) for w in route + [pre]])
        for a, b in zip([o_now] + route, route + [pre]):
            move_tool_straight(a, b)
        tcp_T_tool, _ = tool_in_hand()
        move_tool_straight(pre, o)
    tcp_T_tool, _ = tool_in_hand()

    # ---- optional sideways detour (handle pushes the cube), then pull straight toward the base
    o_side = o + side * lateral * V
    if lateral > 0:
        move_tool_straight(o, o_side)
    move_tool_straight(o_side, o_side - dx * U)
