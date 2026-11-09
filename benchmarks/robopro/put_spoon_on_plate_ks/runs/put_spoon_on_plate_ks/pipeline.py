"""put_spoon_on_plate_ks: pick the spoon up from the counter and lay it on the plate.

Plan: average several noisy detections of the spoon, grasp it with the arm on its side of the robot, lift,
find the plate where it is NOW (track it until it has stopped moving), carry the spoon slowly above it
(checking that it is still held), check the plate again, put the spoon's frame over the plate's frame,
release, retreat. A dropped spoon is detected again and picked up again.
"""

import numpy as np

from api import gripper_quat

N_DET = 8            # detections averaged per estimate (1 cm noise per call -> ~3.5 mm)
FINGER_REACH = 0.036 # fingertips reach this far past the gripper's grasp point (TCP)
LIFT = 0.10          # m above the counter before carrying
TEST_LIFT = 0.03     # short lift that checks a grasp before the object is carried
MAX_PICKS = 2
# grasps tried in turn after the library grasp: (at the box centre, fingertip height above the underside,
# shift along the long axis). A marginal grip (heavy or slippery object) holds on some contacts, not others.
GRASP_VARIANTS = [(True, 0.004, 0.0), (True, 0.006, 0.015), (True, 0.002, -0.015), (True, 0.004, 0.03),
                  (True, 0.006, -0.03)]


def avg_detect(api, name, n=N_DET):
    states = [s for s in (api.detect(name) for _ in range(n)) if s is not None]
    if not states:
        return None
    s = states[0]
    s.position = np.mean([x.position for x in states], axis=0)
    ref = states[0].quaternion
    q = np.mean([x.quaternion * np.sign(np.dot(x.quaternion, ref) or 1.0) for x in states], axis=0)
    s.quaternion = q / np.linalg.norm(q)
    centres = [x.bbox_center for x in states if x.bbox_center is not None]
    if centres:
        s.bbox_center = np.mean(centres, axis=0)
    return s


def settled_detect(api, name, window=0.5, tol=0.012, timeout=8.0):
    """Detection of an object that may be moving: track it over short windows until the mean position of
    the second half of a window is within tol of the first half (bounded by timeout), then average."""
    t_end = api.sim_time() + timeout
    while True:
        hist = api.track(name, duration_s=window, hz=12)
        if len(hist) < 4:
            return avg_detect(api, name)
        ps = np.array([h.position for h in hist])
        half = len(ps) // 2
        if np.linalg.norm(ps[half:].mean(0) - ps[:half].mean(0)) < tol or api.sim_time() >= t_end:
            break
    s = avg_detect(api, name, n=4)
    tail = hist[half:]
    s.position = np.mean([h.position for h in tail] + [s.position], axis=0)
    if s.bbox_center is not None:
        s.bbox_center = np.mean([h.bbox_center for h in tail] + [s.bbox_center], axis=0)
    return s


def carry_over(api, arm, plate):
    """Move the held spoon (keeping hand orientation and height) so that its frame is above the plate's."""
    held = avg_detect(api, "spoon")
    tcp, q = api.tcp_pose(arm)
    above = tcp + np.r_[plate.position[:2] - held.position[:2], 0.0]   # frame over frame
    return api.move_to((above, q), arm=arm, speed="slow")


def quat_to_mat(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def world_height(s):
    """Vertical extent of a detection: bbox extents are given in the object's own frame."""
    return float(np.abs(quat_to_mat(s.quaternion)[2, :]) @ np.asarray(s.bbox_extents))


def top_down_grasp(api, arm, s, at_centre=False, depth=0.006, shift=0.0):
    """Grasp derived from the object's current geometry instead of its annotated grasp frames (which turn
    with the object, e.g. point at the counter when it lies upside down): straight down onto its frame
    point (or its box centre, the best guess of its centre of mass), fingers closing across its longest
    horizontal axis, fingertips just above its underside."""
    R = quat_to_mat(s.quaternion)
    ext = np.asarray(s.bbox_extents, float)
    horiz = [R[:, i] - np.r_[0.0, 0.0, R[2, i]] for i in range(3)]
    i = max(range(3), key=lambda k: ext[k] * np.linalg.norm(horiz[k]))     # longest horizontal axis
    long_ax = horiz[i] / np.linalg.norm(horiz[i])
    q = gripper_quat([0.0, 0.0, -1.0], np.cross([0.0, 0.0, 1.0], long_ax))
    bottom = s.bbox_center[2] - world_height(s) / 2
    point = (s.bbox_center if at_centre else s.position) + shift * long_ax
    g = np.r_[point[:2], bottom + FINGER_REACH + depth]
    api.open_gripper(arm, 0.6)
    if not api.move_to((g + [0.0, 0.0, 0.10], q), arm=arm):
        return False
    # the last centimetres put the fingertips next to the counter top: allow that contact for this short,
    # straight descent (the planner would otherwise refuse it)
    if not api.move_to((g, q), arm=arm, linear=True, allow_contact=["spoon", "table"]):
        return False
    api.close_gripper(arm)
    return api.gripper_holding(arm)


def pick(api):
    """Detect and grasp the spoon, lift it. Every grasp is checked by a short test lift before the full lift;
    if the spoon slips, it is released, detected again and grasped differently (GRASP_VARIANTS).
    Returns (arm, spoon detection) or (None, None)."""
    spoon = avg_detect(api, "spoon")
    for k in range(1 + len(GRASP_VARIANTS)):
        if spoon is None:
            return None, None
        arm = "right" if spoon.position[0] > 0 else "left"
        if k == 0:
            held = api.grasp(spoon, arm=arm) or top_down_grasp(api, arm, spoon)
        else:
            at_centre, depth, shift = GRASP_VARIANTS[k - 1]
            held = top_down_grasp(api, arm, spoon, at_centre, depth, shift)
        if held and api.move_by(dz=TEST_LIFT, arm=arm, speed="slow") and api.gripper_holding(arm):
            if api.move_by(dz=LIFT - TEST_LIFT, arm=arm, speed="slow") and api.gripper_holding(arm):
                return arm, spoon
        api.open_gripper(arm)
        api.move_by(dz=0.05, arm=arm)
        spoon = avg_detect(api, "spoon")
    return None, None


def run(api):
    for _ in range(MAX_PICKS):
        arm, spoon = pick(api)
        if arm is None:
            continue
        # the plate may have been moved while the spoon was picked: find it where it is now, carry, then
        # check again right before releasing and follow it if it moved
        plate = settled_detect(api, "plate")
        carry_over(api, arm, plate)
        for _ in range(2):
            now = settled_detect(api, "plate")
            if np.linalg.norm(now.position[:2] - plate.position[:2]) < 0.015:
                break
            plate = now
            carry_over(api, arm, plate)
        if not api.gripper_holding(arm):           # slipped out on the way: pick it up again
            api.open_gripper(arm)
            api.home(arm)
            continue
        # the spoon's frame goes over the plate's frame (both are the objects' model frames); release just
        # above the plate's top surface
        held = avg_detect(api, "spoon")          # its orientation in the hand now (it may be rolled)
        target = plate.position.copy()
        target[2] = plate.bbox_center[2] + world_height(plate) / 2 + world_height(held) / 2 + 0.005
        api.place(target, arm=arm, allow_contact=["plate"], speed="slow")
        api.home(arm)
        return
