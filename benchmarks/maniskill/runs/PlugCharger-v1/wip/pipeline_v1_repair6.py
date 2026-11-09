"""PlugCharger-v1: plug the charger into the receptacle.

Strategy: average many noisy detections (noise is independent per call), grasp the charger
base from the top, re-estimate the charger pose in the gripper after grasping, align in front
of the goal with closed-loop correction on the (exact) TCP pose, then insert along the goal's
x axis in small servoed steps.
"""

import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat

from api import pose_inv, pose_mul, rpy_to_quat

N_AVG = 20000
BASE_HALF = np.array([0.02, 0.015, 0.012])  # charger base half size
LOG = []


def avg_poses(samples):
    ps = np.array([s[0] for s in samples])
    qs = np.array([s[1] for s in samples])
    qs[qs @ qs[0] < 0] *= -1
    q = qs.mean(0)
    return ps.mean(0), q / np.linalg.norm(q)


def frames(api, n=N_AVG):
    raw = [api.task_frames() for _ in range(n)]
    return {k: avg_poses([r[k] for r in raw]) for k in raw[0]}


def charger_from_base(base):
    return pose_mul(base, ([BASE_HALF[0], 0, 0], [1, 0, 0, 0]))


def top_grasp_pose(base, tcp_q):
    p, q = base
    R = quat2mat(q)
    approaching = np.array([0.0, 0.0, -1.0])
    closing = R[:, 1].copy()
    closing[2] = 0
    closing /= np.linalg.norm(closing)
    if closing @ quat2mat(tcp_q)[:, 1] < 0:
        closing = -closing
    ortho = np.cross(closing, approaching)
    M = np.stack([ortho, closing, approaching], axis=1)
    grasp = (np.array(p), mat2quat(M))  # base center (base is shorter than finger length)
    # pitch so the approach leans toward the pegs (charger +x): the palm then stays behind the
    # charger, away from the receptacle face, whichever way the closing axis points
    pitch = np.deg2rad(15) * (1 if ortho @ R[:, 0] > 0 else -1)
    return pose_mul(grasp, ([0, 0, 0], rpy_to_quat(0, pitch, 0)))


def move_checked(api, pose, tries=3, tol=0.003, speed="normal"):
    """move_to, re-planned from where the arm ended: the screw planner integrates its Jacobian
    open loop, and on a large rotation the endpoint can be several cm off while reporting success."""
    ok = False
    for _ in range(tries):
        ok = api.move_to(pose, speed=speed)
        tcp = api.tcp_pose()
        dq = abs(float(np.dot(tcp[1], pose[1])))
        if np.linalg.norm(np.asarray(tcp[0]) - pose[0]) < tol and dq > np.cos(np.deg2rad(2) / 2):
            return ok
    return ok


def servo_to(api, charger_target, hold, axis, iters=5, tol=2e-4, back=0.004, keep_q=False):
    """Bring the held charger to charger_target, correcting the residual error measured at the
    charger (exact TCP pose composed with the in-hand estimate `hold`).
    The planner's endpoint is off by up to a few mm, and a correction below ~1 mm yields a
    one-waypoint plan that does not move the arm, so every attempt re-approaches the
    (corrected) target from `back` metres behind it along `axis`."""
    tq = api.tcp_pose()[1] if keep_q else pose_mul(charger_target, pose_inv(hold))[1]
    cp = pose_mul(charger_target, pose_inv(hold))[0]
    axis = np.asarray(axis) / np.linalg.norm(axis)
    for _ in range(iters):
        api.move_to((cp - back * axis, tq), speed="slow")
        api.move_to((cp, tq), speed="slow")
        api.wait_until(lambda: False, 0.1)
        # keep the reached orientation: a small residual rotation in the target makes the screw
        # planner's endpoint drift by a few tenths of a mm, so corrections are pure translations
        tq = api.tcp_pose()[1]
        ep = np.asarray(charger_target[0]) - pose_mul(api.tcp_pose(), hold)[0]
        if np.linalg.norm(ep) < tol:
            break
        cp = cp + ep
    err = float(np.linalg.norm(np.asarray(charger_target[0]) - pose_mul(api.tcp_pose(), hold)[0]))
    LOG.append(round(err * 1000, 2))
    return err


def estimate_hold(api):
    """Charger pose in the TCP frame (exact TCP pose, averaged charger detections)."""
    tcp = api.tcp_pose()
    return pose_mul(pose_inv(tcp), charger_from_base(frames(api)["charger_base"]))


def is_flat(base):
    return quat2mat(base[1])[2, 2] > 0.7  # charger z axis points up


def lay_flat(api, base, lift_z=0.15):
    """The charger is not resting on its bottom face. A top grasp on it cannot be used for the
    insertion (the hand would be rolled 90 deg and hit the table), so regrasp: roll it 90 deg about
    its x axis in the hand and set it back down. The fingers close along the charger x axis, which
    stays horizontal during the roll, and the approach bisects world-down before and after the roll
    (45 deg pitched both times), so neither the grasp nor the placement brings the hand near the
    table and the charger is put down instead of dropped."""
    R = quat2mat(base[1])
    u = R.T @ np.array([0.0, 0.0, 1.0])  # world up in the charger frame
    u[0] = 0
    u /= np.linalg.norm(u)
    # target "up" axis for this step: the charger z, or a side axis when it is upside down
    t = np.array([0.0, 0.0, 1.0]) if u[2] > -0.3 else np.array([0.0, np.sign(u[1]) or 1.0, 0.0])
    a_c = -(u + t)
    a_c /= np.linalg.norm(a_c)            # approach, in the charger frame
    c_c = np.array([1.0, 0.0, 0.0])
    if (R @ c_c) @ quat2mat(api.tcp_pose()[1])[:, 1] < 0:
        c_c = -c_c
    M = np.stack([np.cross(c_c, a_c), c_c, a_c], axis=1)
    grasp = pose_mul(base, (-0.004 * a_c, mat2quat(M)))
    api.open_gripper()
    if not move_checked(api, pose_mul(grasp, ([0, 0, -0.06], [1, 0, 0, 0]))):
        return False
    rg = move_checked(api, grasp, speed="slow")
    api.close_gripper()
    tcp = api.tcp_pose()
    rl = api.move_to(([tcp[0][0], tcp[0][1], lift_z], tcp[1]))
    LOG.append(("lift", rg, rl, api.gripper_holding(), [round(float(x), 3) for x in grasp[0]], [round(float(x), 2) for x in quat2mat(grasp[1])[:, 2]], [round(float(x), 3) for x in api.tcp_pose()[0]]))
    hold = estimate_hold(api)
    p, q = pose_mul(api.tcp_pose(), hold)
    Rc = quat2mat(q)
    v = Rc.T @ np.array([0.0, 0.0, 1.0])
    th = np.arctan2(v[2], v[1]) - np.arctan2(t[2], t[1])  # roll about x taking t to world up
    new_q = mat2quat(Rc @ quat2mat(rpy_to_quat(th, 0, 0)))
    move_checked(api, pose_mul((p, new_q), pose_inv(hold)))
    # lower until the charger is 2 mm above the table (half height of the face it rests on)
    half = BASE_HALF[2] if t[2] else BASE_HALF[1]
    c = pose_mul(api.tcp_pose(), hold)
    tcp = api.tcp_pose()
    r = api.move_to((np.array(tcp[0]) + [0, 0, half + 0.002 - c[0][2]], tcp[1]), speed="slow")
    LOG.append(("place", r, round(float(th), 2), [round(float(x), 3) for x in pose_mul(api.tcp_pose(), hold)[0]]))
    api.open_gripper()
    api.move_to(pose_mul(api.tcp_pose(), ([0, 0, -0.06], [1, 0, 0, 0])), speed="slow")
    tcp = api.tcp_pose()
    api.move_to(([tcp[0][0], tcp[0][1], lift_z], tcp[1]))
    return True


def run(api):
    LOG.clear()
    f = frames(api)
    for _ in range(2):
        if is_flat(f["charger_base"]):
            break
        ok = lay_flat(api, f["charger_base"])
        f = frames(api)
        R = quat2mat(f["charger_base"][1])
        LOG.append(("flat", ok, [round(float(v), 3) for v in f["charger_base"][0]], [round(float(v), 2) for v in R[:, 2]], [round(float(v), 2) for v in R[:, 0]]))
    goal = f["goal"]
    grasp = top_grasp_pose(f["charger_base"], api.tcp_pose()[1])

    api.open_gripper()
    if not move_checked(api, pose_mul(grasp, ([0, 0, -0.05], [1, 0, 0, 0]))):
        return
    move_checked(api, grasp)
    api.close_gripper()
    lift = pose_mul(([0, 0, 0.05], [1, 0, 0, 0]), api.tcp_pose())
    api.move_to(lift)

    def goal_offset(dx):
        return pose_mul(goal, ([dx, 0, 0], [1, 0, 0, 0]))

    def goal_offset_tcp(dx, tcp_T_charger):
        return pose_mul(goal_offset(dx), pose_inv(tcp_T_charger))

    hold = estimate_hold(api)
    api.move_to(goal_offset_tcp(-0.04, hold))
    axis = quat2mat(goal[1])[:, 0]  # insertion direction
    for attempt in range(3):
        hold = estimate_hold(api)  # re-estimated every attempt: the charger can shift in the fingers
        servo_to(api, goal_offset(-0.019), hold, axis)
        # pegs enter the holes between dx=-0.016 and -0.014; a large residual means they hit the face
        if servo_to(api, goal_offset(-0.013), hold, axis, iters=2, back=0.0, keep_q=True) < 1e-3:
            break
        api.move_to(goal_offset_tcp(-0.03, hold), speed="slow")  # back off and retry
        goal = avg_poses([goal, frames(api)["goal"]])  # refine the goal estimate with fresh samples
    for dx in (-0.007, 0.0):
        servo_to(api, goal_offset(dx), hold, axis, iters=2, back=0.0, keep_q=True)
