"""place_bowl_in_dishrack_ks: put the bowl into the dish rack.

The rack's floor wires are low (up to 5 cm), so a bowl laid flat in the rack rests too low to count as
being in it; it has to stand on its rim. Nothing in the middle of the rack holds a 14 cm bowl on edge, but
the utensil holder at one end does: stood on its rim next to the holder, opening towards the middle of the
rack, the bowl leans back against the holder.

Plan: average a few noisy detections; grasp the bowl's rim from above (a pinch across the rim wall); lift
it flat. The planner does not see the held bowl, so every motion with it is short or straight: carry it flat
(stable in a pinch) high over the rack; there turn it on edge in small steps, so that it ends hanging below
the pinch with the pinch at the top of the rim (a pinch holds that pose; a pinch at the side of the rim lets
the bowl swing). The bowl is symmetric about its axis, so the turned pose is free in rotation about that axis;
the gripper-highest reachable one is taken. Then lower it straight down, release with the rim 2 cm above the
floor wires, slide the fingers out of the bowl along its axis and go up.
"""

import numpy as np
import transforms3d as t3d

N_DETECT = 8            # detections averaged per estimate (1 cm / 3 deg noise each)
LIFT = 0.15             # m, lift after the grasp
RELEASE = 0.07          # m, rim bottom above the counter at release (floor wires reach 5 cm)
GAP = 0.012             # m, foot of the bowl to the holder's face at release
CLEAR = 0.04            # m, bowl bottom above the rack's top while it is carried
PHIS = np.deg2rad(np.arange(0, 360, 30))   # rotations of the bowl about its own axis
TILTS = (0.0, 0.2, -0.2)                   # opening raised (lowered) by this much (tan) from horizontal
TURN_STEPS = 5                             # waypoints while turning the bowl on edge


def averaged(api, name, n=N_DETECT):
    states = [api.detect(name) for _ in range(n)]
    states = [s for s in states if s is not None]
    if not states:
        return None
    s = states[0]
    s.position = np.mean([x.position for x in states], axis=0)
    qs = np.array([x.quaternion for x in states])
    qs = np.array([q if np.dot(q, qs[0]) >= 0 else -q for q in qs])
    s.quaternion = qs.mean(axis=0) / np.linalg.norm(qs.mean(axis=0))
    return s


def mat(p, q):
    T = np.eye(4)
    T[:3, :3] = t3d.quaternions.quat2mat(q)
    T[:3, 3] = p
    return T


def rot(axis, angle):
    return t3d.axangles.axangle2mat(axis, angle)


def align(a, b):
    """Rotation taking unit vector a to unit vector b."""
    v = np.cross(a, b)
    s, c = np.linalg.norm(v), float(np.dot(a, b))
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        perp = np.cross(a, [1.0, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1.0, 0])
        return rot(perp / np.linalg.norm(perp), np.pi)
    return rot(v / s, np.arctan2(s, c))


def table_top(api):
    t = averaged(api, "table")
    if t is None:
        return 0.74
    R = t3d.quaternions.quat2mat(t.quaternion)
    return float(t.position[2] + np.sum(np.abs(R[2, :]) * t.bbox_extents) / 2)


def bowl_axis_body(s):
    """(index of the bowl's axis in its own frame, sign so that it points out of the opening). The bowl
    stands on the counter at the start, so the opening is the end of the axis that points up."""
    k = int(np.argmin(s.bbox_extents))
    R = t3d.quaternions.quat2mat(s.quaternion)
    return k, (1.0 if R[2, k] >= 0 else -1.0)


def bowl_pose_for(T_bowl, G):
    """Gripper pose (p, q) that puts the bowl at T_bowl, holding it as it is held now (G)."""
    Tg = T_bowl @ G
    return Tg[:3, 3].copy(), t3d.quaternions.mat2quat(Tg[:3, :3])


def candidates(Tb, k, sign, side):
    """Bowl orientations on edge, opening towards the rack middle (`side` along x), best first: level, then
    tilted; every rotation about the bowl's own axis."""
    up_now = Tb[:3, :3][:, k] * sign
    out = []
    for tilt in TILTS:
        a = np.array([side, 0.0, tilt])
        a /= np.linalg.norm(a)
        R0 = align(up_now, a) @ Tb[:3, :3]
        for phi in PHIS:
            out.append((rot(a, phi) @ R0, a))
    return out


def slerp(q0, q1, t):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    if np.dot(q0, q1) < 0:
        q1 = -q1
    d = float(np.clip(np.dot(q0, q1), -1, 1))
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
    else:
        th = np.arccos(d)
        q = (np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)
    return q / np.linalg.norm(q)


def run(api):
    table_z = table_top(api)
    rack = averaged(api, "dishrack")
    holder = averaged(api, "utensil_holder")
    bowl = averaged(api, "bowl")
    k, sign = bowl_axis_body(bowl)
    ext = np.asarray(bowl.bbox_extents, float)
    radius, height = float(np.max(ext)) / 2, float(ext[k])
    arm = "right" if bowl.position[0] > 0 else "left"
    rack_top = float(rack.position[2] + rack.bbox_extents[2] / 2)

    if not api.grasp(bowl, arm, strategy="top"):
        return
    b = averaged(api, "bowl")
    p, q = api.tcp_pose(arm)
    Tb = mat(b.position, b.quaternion)
    G = np.linalg.inv(Tb) @ mat(p, q)                     # gripper in the bowl's frame
    side = 1.0 if rack.position[0] > holder.position[0] else -1.0   # from the holder towards the rack middle
    face = holder.position[0] + side * holder.bbox_extents[0] / 2
    release = np.array([face + side * (height / 2 + GAP), holder.position[1], table_z + RELEASE + radius])
    above = release.copy()
    above[2] = rack_top + CLEAR + radius                  # bowl centre when on edge above the rack

    # carry it flat: straight up, then straight over to above the release point
    lift = rack_top + CLEAR + height / 2 + 0.02 - b.position[2]
    api.move_by(dz=max(lift, 0.08), arm=arm)
    b = averaged(api, "bowl")
    flat_c = np.array([above[0], above[1], b.position[2]])
    if not api.move_to(bowl_pose_for(mat(flat_c, b.quaternion), G), arm, linear=True):
        api.move_to(bowl_pose_for(mat(flat_c, b.quaternion), G), arm)
    b = averaged(api, "bowl")
    Tb = mat(b.position, b.quaternion)

    cands = candidates(Tb, k, sign, side)
    # gripper highest first at the release pose (the pinch is then at the top of the rim)
    cands.sort(key=lambda c: -bowl_pose_for(mat(release, t3d.quaternions.mat2quat(c[0])), G)[0][2])
    cands.sort(key=lambda c: abs(c[1][2]))                 # level before tilted (stable sort)
    start_c, start_q = b.position.copy(), b.quaternion.copy()
    G = np.linalg.inv(Tb) @ mat(*api.tcp_pose(arm))       # the bowl swings a little in a pinch: re-measure
    for R, a in cands:
        qb = t3d.quaternions.mat2quat(R)
        ok = True
        for i in range(1, TURN_STEPS + 1):               # turn in small steps
            t = i / TURN_STEPS
            c = start_c + t * (above - start_c)
            if not api.move_to(bowl_pose_for(mat(c, slerp(start_q, qb, t)), G), arm, speed="slow"):
                ok = False
                break
        if not ok:
            if i > 1:                                      # back to flat for the next candidate
                api.move_to(bowl_pose_for(mat(start_c, start_q), G), arm, speed="slow")
            continue
        # where does the bowl actually hang? Lower it by translation only, to the release point
        api.wait_until(lambda: False, timeout_s=0.5)
        b = averaged(api, "bowl")
        p, q = api.tcp_pose(arm)
        api.move_to((p + (release - b.position), q), arm, linear=True, speed="slow", allow_contact=["dishrack"])
        api.open_gripper(arm)
        if not api.move_by(*(0.07 * a), arm=arm, allow_contact=["dishrack", "bowl"]):
            p, q = api.tcp_pose(arm)
            api.move_to((p + 0.07 * a + [0, 0, 0.03], q), arm, allow_contact=["dishrack", "bowl"])
        api.move_by(dz=0.08, arm=arm, allow_contact=["dishrack", "bowl"])
        return
