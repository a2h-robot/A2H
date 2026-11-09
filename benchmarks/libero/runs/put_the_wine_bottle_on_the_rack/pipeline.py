"""put_the_wine_bottle_on_the_rack: "put the wine bottle on the rack".

The rack holds bottles lying on an inclined shelf, against a low front lip; the rack's top
region sits at that lip and its x axis runs along the shelf. Plan: top-grasp the standing bottle by the neck (fingers
closing along y), lift, pitch the wrist about
the horizontal axis across the shelf so the bottle hangs tilted, bottom along the shelf, set the bottom
on the shelf just up-slope of the lip, release so it lies down, retreat.
"""

import sys

import numpy as np
from scipy.spatial.transform import Rotation

BOTTLE = "wine_bottle_1"
RACK = "wine_rack_1"
REGION = "wine_rack_1_top_region"
Z = np.array([0.0, 0.0, 1.0])
BASE_XY = np.array([-0.66, 0.0])

NECK_DEPTH = 0.03           # grip site below the bottle top [m]
BOTTLE_RADIUS = 0.02
SLOPE_OFFSET = 0.035        # bottle bottom up-slope from the region centre (lip) [m]
CLEARANCE = 0.015           # bottom centre height above resting, along the shelf normal [m]
LYING_GRIP_FROM_BOTTOM = (0.04, 0.025, 0.055)  # body grasp points along a lying bottle [m]
LYING_GRIP_HEIGHT = 0.0         # grip site at the axis height: horizontal contact normals
LYING_SLOPE_OFFSET = 0.045      # bottle axis up-slope of the lip at release (fingers clear the lip)
LYING_CLEARANCE = 0.006
TRANSFER_Z = 1.10            # height at which a lying bottle is turned to the placing pose [m]
MAX_ATTEMPTS = 3
SIDE_GRIP_FROM_BOTTOM = 0.045  # side grasp point on a standing bottle's body [m]
SIDE_AZIMUTHS = (0, 30, -30, 60, -60)   # approach azimuths relative to base->bottle [deg]
SIDE_TILTS = (10, 0, 20)      # approach pitched down [deg]
SIDE_SLIDE_OUT = 0.12         # horizontal retreat with the bottle before lifting [m]
SETTLE_ALONG = 0.035         # press point along the bottle axis from its bottom (the body) [m]
SETTLE_DEPTH = 0.010         # how far past the nominal body top the fingertips press [m]
RELEASE_TOL = 0.02           # max distance from the planned release pose to still release [m]
NECK_GRASP_YAWS = (0, 90, 45, -45, 135, -135)   # closing-axis yaw offsets tried [deg]
RELEASE_CANDIDATES = ((0.06, 60), (0.08, 60), (0.03, 60), (0.06, 50), (0.08, 50), (0.03, 50))
# (bottle bottom along the shelf from the region centre [m], bottle tilt from vertical [deg])
CENTRED_OFFSET = 0.03       # bottle bottom along the shelf from the region centre [m]


def log(*a):
    print("[pipeline]", *a, file=sys.stderr)


def _R(q):
    w, x, y, z = q
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def _q(R):
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def unit(v):
    return v / np.linalg.norm(v)


def detect_avg(api, name, n=10):
    """Average n noisy detections: (position, rotation, bbox centre, extents) or None."""
    sts = [s for s in (api.detect(name) for _ in range(n)) if s is not None]
    if not sts:
        return None
    p = np.mean([s.position for s in sts], axis=0)
    R = Rotation.from_matrix([_R(s.quaternion) for s in sts]).mean().as_matrix()
    c = np.mean([s.bbox_center for s in sts], axis=0)
    return p, R, c, sts[0].bbox_extents


def top_frame(c):
    """Top-down gripper frame with the fingers closing along horizontal axis c."""
    c = unit(np.array([c[0], c[1], 0.0]))
    return np.column_stack([c, np.cross(-Z, c), -Z])


def closest_sign(api, c):
    """Gripper yaw is symmetric under pi: pick +-c closest to the current closing axis."""
    _, q = api.tcp_pose()
    return c if _R(q)[:, 0] @ c >= 0 else -c


def grasp_neck(api, c, det):
    """Top grasp of the neck. Prefers the fingers closing along c; if the fingers would hit
    something there, tries other closing directions (the yaw is corrected in the air later).
    Returns the gripper rotation used."""
    p_b, R_b, c_b, ext = det
    top = c_b[2] + 0.5 * abs(R_b[2] @ ext)
    P = np.array([c_b[0], c_b[1], top - NECK_DEPTH])
    for dyaw in NECK_GRASP_YAWS:
        cy = closest_sign(api, Rotation.from_rotvec(np.radians(dyaw) * Z).as_matrix() @ c)
        for cc in (cy, -cy):
            Rg = top_frame(cc)
            q = _q(Rg)
            api.open_gripper()
            if not api.move_to((P + Z * 0.08, q)):
                continue
            api.move_to((P, q), linear=True, speed="slow", allow_contact=[BOTTLE])
            if api.last_motion == "refused":
                api.move_to((P + Z * 0.08, q))
                continue
            api.close_gripper()
            if api.gripper_holding():
                return Rg
            log("grasp missed")
            api.open_gripper()
            api.move_to((P + Z * 0.08, q), linear=True, allow_contact=[BOTTLE])
            return None
    log("no neck grasp")
    return None


def shelf_frame(api):
    """(region centre, up-slope unit vector, shelf normal pointing up)."""
    p_s, R_s, _, _ = detect_avg(api, REGION)
    normal = -R_s[:, 1]
    if normal[2] < 0:
        normal = -normal
    up = unit(R_s[:, 2] - normal * (R_s[:, 2] @ normal))
    if up[2] < 0:
        up = -up
    return p_s, up, normal


def shelf_axis(R_s):
    """Horizontal direction along which a bottle lies on the shelf (region x axis)."""
    return unit(np.array([R_s[0, 0], R_s[1, 0], 0.0]))


def grasp_side_standing(api, det):
    """Fallback when the neck cannot be reached from above (e.g. something overhead): grasp the
    standing bottle's body from the side with a near-horizontal approach, then slide it out
    horizontally before lifting. Returns the gripper rotation, or None."""
    p_b, R_b, _, _ = det
    P = p_b + R_b[:, 2] * SIDE_GRIP_FROM_BOTTOM
    base_dir = unit(np.append(P[:2] - BASE_XY, 0.0))
    for daz in SIDE_AZIMUTHS:
        a_h = Rotation.from_rotvec(np.radians(daz) * Z).as_matrix() @ base_dir
        for tilt in SIDE_TILTS:
            a = np.cos(np.radians(tilt)) * a_h - np.sin(np.radians(tilt)) * Z
            x = unit(np.cross(Z, a))
            Rg = np.column_stack([x, np.cross(a, x), a])
            q = _q(Rg)
            api.open_gripper()
            if not api.move_to((P - a * 0.10, q)):
                continue
            api.move_to((P + a * 0.005, q), linear=True, speed="slow", allow_contact=[BOTTLE])
            if api.last_motion == "refused":
                continue
            api.close_gripper()
            if not api.gripper_holding():
                log("side grasp missed")
                api.open_gripper()
                api.move_to((P - a * 0.10, q), linear=True, allow_contact=[BOTTLE])
                continue
            # Slide out from under whatever blocked the top approach, then it can be lifted.
            p, _ = api.tcp_pose()
            api.move_to((p + Z * 0.01, q), linear=True, speed="slow")
            p, _ = api.tcp_pose()
            api.move_to((p - a_h * SIDE_SLIDE_OUT, q), linear=True, speed="slow")
            return Rg
    log("no side grasp")
    return None


def grasp_body_lying(api, det):
    """Top grasp across the body of a bottle lying on its side. Tries several grasp points along
    the body and small yaw changes when the fingers would hit a neighbour.
    Returns the gripper rotation."""
    p_b, R_b, _, _ = det
    b = R_b[:, 2]
    c0 = np.cross(Z, unit(np.array([b[0], b[1], 0.0])))
    for along in LYING_GRIP_FROM_BOTTOM:
        P = p_b + b * along
        P[2] = p_b[2] + LYING_GRIP_HEIGHT
        for dyaw in (0.0, np.radians(15), np.radians(-15)):
            c = closest_sign(api, Rotation.from_rotvec(dyaw * Z).as_matrix() @ c0)
            for cc in (c, -c):
                Rg = top_frame(cc)
                q = _q(Rg)
                api.open_gripper()
                if not api.move_to((P + Z * 0.10, q)):
                    continue
                api.move_to((P, q), linear=True, speed="slow", allow_contact=[BOTTLE])
                if api.last_motion == "refused":
                    continue
                api.close_gripper()
                if api.gripper_holding():
                    return Rg
                log("grasp (lying) missed")
                api.open_gripper()
                api.move_to((P + Z * 0.10, q), linear=True, allow_contact=[BOTTLE])
    log("no lying grasp")
    return None


def transfer(api, q_t):
    """Lift and turn the held bottle to the placing orientation in free space above the table."""
    p, q = api.tcp_pose()
    up_p = p + Z * max(0.0, TRANSFER_Z - p[2])
    api.move_to((up_p, q), linear=True)
    api.move_to((api.tcp_pose()[0], q_t))


def lower_and_release(api, p_t, q_t, back_dir, backs):
    """Pre-place along back_dir, descend to the release pose, release. Returns a status.
    Does not release if the hand stopped far from the release pose (blocked / out of reach)."""
    # Straight-line descent only: a joint-space "descent" near the reach limit can swing the
    # held bottle through the scene. Try several pre-place distances for the line.
    pre = None
    for b in backs:
        if not api.move_to((p_t + back_dir * b, q_t)):
            continue
        api.move_to((p_t, q_t), linear=True, speed="slow", allow_contact=[RACK])
        if api.last_motion != "refused":
            pre = p_t + back_dir * b
            break
    if pre is None:
        return "unreachable"
    err = np.linalg.norm(api.tcp_pose()[0] - p_t)
    if err > RELEASE_TOL:
        log("release pose missed by", round(err, 3), "- backing off")
        api.move_to((pre, q_t), linear=True, allow_contact=[RACK])
        if api.last_motion == "refused":
            api.move_to((pre, q_t), allow_contact=[RACK])
        return "unreachable" if api.gripper_holding() else "lost"
    if not api.gripper_holding():
        return "lost"
    api.open_gripper()
    p, q = api.tcp_pose()
    api.move_to((p + back_dir * 0.08, q), linear=True, speed="slow", allow_contact=[RACK, BOTTLE])
    if api.last_motion == "refused":
        api.move_to((p + Z * 0.08, q), allow_contact=[RACK, BOTTLE])
    settle(api, shelf_frame(api)[2])
    return "released"


def settle(api, normal):
    """After release, press the bottle body gently onto the shelf with the closed fingertips so
    it seats flat against the shelf and lip instead of staying perched on an edge."""
    det = detect_avg(api, BOTTLE)
    if det is None:
        return
    p_b, R_b, _, _ = det
    b = R_b[:, 2]
    if abs(b @ normal) > 0.5:          # not lying: nothing to seat
        return
    press = p_b + b * SETTLE_ALONG + normal * BOTTLE_RADIUS
    a = -normal
    x = unit(b - a * (b @ a))
    R = np.column_stack([x, np.cross(a, x), a])
    q = _q(R)
    api.close_gripper()
    if not api.move_to((press + normal * 0.05, q), allow_contact=[BOTTLE]):
        api.open_gripper()
        return
    api.move_to((press - normal * SETTLE_DEPTH, q), linear=True, speed="slow",
                allow_contact=[BOTTLE, RACK])
    p, _ = api.tcp_pose()
    api.move_to((p + normal * 0.06, q), linear=True, speed="slow", allow_contact=[BOTTLE, RACK])
    api.open_gripper()


def place_lying(api, Rg, o, b_g):
    """Lower a horizontally held bottle onto the shelf along the shelf axis, approach along the
    shelf normal, fingers closing along the slope."""
    p_s, up, normal = shelf_frame(api)
    a = -normal
    status = "unreachable"
    for x_t in (up, -up):
        R_t = np.column_stack([x_t, np.cross(a, x_t), a])
        neck = R_t @ b_g
        bottom = (p_s + up * LYING_SLOPE_OFFSET + normal * (BOTTLE_RADIUS + LYING_CLEARANCE)
                  - unit(neck - normal * (neck @ normal)) * CENTRED_OFFSET)
        p_t = bottom - R_t @ o
        q_t = _q(R_t)
        log("lying place target", np.round(p_t, 3))
        transfer(api, q_t)
        status = lower_and_release(api, p_t, q_t, normal, (0.08, 0.05, 0.12))
        if status != "unreachable":
            return status
    return status


def place_standing(api, Rg, o, dirs):
    """Pitch the neck-held bottle so it hangs tilted, set its bottom on the shelf, release."""
    p_s, up, normal = shelf_frame(api)
    status = "unreachable"
    # Hand on the robot side (approach pointing away from the base) first.
    # Release-pose candidates: nominal first, then shifted along the shelf (inside the goal
    # region) and less tilted, for racks near the edge of the arm's reach.
    cands = [(d_b, off, tilt) for d_b in dirs for off, tilt in RELEASE_CANDIDATES]
    for d_b, off, tilt in cands:
        # Pitch the wrist about the horizontal axis normal to d_b so the bottom swings toward d_b.
        R_t = Rotation.from_rotvec(-np.radians(tilt) * np.cross(Z, d_b)).as_matrix() @ Rg
        bottom = p_s + up * SLOPE_OFFSET + normal * (BOTTLE_RADIUS + CLEARANCE) + d_b * off
        p_t = bottom - R_t @ o
        q_t = _q(R_t)
        log("place target", np.round(p_t, 3), "approach", np.round(R_t[:, 2], 2), off, tilt)
        status = lower_and_release(api, p_t, q_t, Z, (0.08, 0.05, 0.12))
        if status != "unreachable":
            return status
    return status


def run(api):
    # The shelf (not the world) defines how the bottle must lie: read it first.
    p_s, R_s, _, _ = detect_avg(api, REGION)
    ax = shelf_axis(R_s)
    to_rack = np.append(p_s[:2] - BASE_XY, 0.0)
    # Bottle bottom pointing away from the robot first (hand stays on the near side).
    dirs = sorted([ax, -ax], key=lambda d: -(d @ to_rack))

    for attempt in range(MAX_ATTEMPTS):
        det = detect_avg(api, BOTTLE)           # re-detect: an earlier attempt may have moved it
        if det is None:
            return
        lying = abs(det[1][2, 2]) < 0.5
        side = False
        if lying:
            Rg = grasp_body_lying(api, det)
        else:
            Rg = grasp_neck(api, np.cross(Z, dirs[0]), det)
            if Rg is None and abs(det[1][2, 2]) > 0.5:
                # Neck not reachable from above: grasp the body from the side; the bottle is then
                # held across its body like a lying one and is placed the same way.
                Rg = grasp_side_standing(api, det)
                side = Rg is not None
        if Rg is None:
            api.open_gripper()
            p, q = api.tcp_pose()
            api.move_to((p + Z * 0.10, q), allow_contact=[BOTTLE])
            continue
        p, q = api.tcp_pose()
        api.move_to((p + Z * 0.12, q), linear=True)
        if not api.gripper_holding():
            log("lost the bottle while lifting")
            api.open_gripper()
            continue
        if not lying and not side:
            # A neck-held bottle hangs along the approach axis: turn the wrist about the vertical
            # to the closing axis the shelf needs (the grasp may have used another one).
            c_want = closest_sign(api, np.cross(Z, dirs[0]))
            R_want = top_frame(c_want)
            if abs(Rg[:, 0] @ c_want) < 0.99:
                p, _ = api.tcp_pose()
                if api.move_to((p, _q(R_want))) or api.move_to((p, _q(top_frame(-c_want)))):
                    Rg = _R(api.tcp_pose()[1])
                    Rg = top_frame(Rg[:, 0])
                if not api.gripper_holding():
                    log("lost the bottle while turning")
                    api.open_gripper()
                    continue
        # In-hand bottle bottom and axis, in the commanded gripper frame.
        p_tcp, _ = api.tcp_pose()
        p_b, R_b, _, _ = detect_avg(api, BOTTLE)
        o = Rg.T @ (p_b - p_tcp)
        if lying or side:
            status = place_lying(api, Rg, o, Rg.T @ R_b[:, 2])
        else:
            status = place_standing(api, Rg, o, dirs)
        log("attempt", attempt, "lying" if lying else "standing", status)
        if status == "released" or status == "unreachable":
            return
        api.open_gripper()
        p, q = api.tcp_pose()
        api.move_to((p + Z * 0.10, q))
