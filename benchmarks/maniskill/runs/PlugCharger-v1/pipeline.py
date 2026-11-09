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
SAG = [np.zeros(3)]  # command minus reached TCP position: the arm sags under a heavy part


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


def top_grasp_pose(base, tcp_q, lean=None, flip=False):
    p, q = base
    R = quat2mat(q)
    approaching = np.array([0.0, 0.0, -1.0])
    closing = R[:, 1].copy()
    closing[2] = 0
    closing /= np.linalg.norm(closing)
    if (closing @ quat2mat(tcp_q)[:, 1] < 0) != flip:
        closing = -closing
    ortho = np.cross(closing, approaching)
    M = np.stack([ortho, closing, approaching], axis=1)
    grasp = (np.array(p), mat2quat(M))  # base center (base is shorter than finger length)
    # pitch so the approach leans toward the pegs (charger +x): the palm then stays behind the
    # charger, away from the receptacle face, whichever way the closing axis points
    # (lean: another horizontal direction for the approach to lean toward, for grasps that are
    # not used for insertion)
    lean = R[:, 0] if lean is None else np.asarray(lean)
    pitch = np.deg2rad(15) * (1 if ortho @ lean > 0 else -1)
    return pose_mul(grasp, ([0, 0, 0], rpy_to_quat(0, pitch, 0)))


def quat_angle(q1, q2):
    return 2 * np.arccos(min(1.0, abs(float(np.dot(q1, q2)))))


def servo_to(api, charger_target, hold, axis, iters=5, tol=2e-4, back=0.004, keep_q=False,
             measure=None, rot_tol=0.005):
    """Bring the held charger to charger_target, correcting the residual error measured at the
    charger (exact TCP pose composed with the in-hand estimate `hold`).
    The planner's endpoint is off by up to a few mm, and a correction below ~1 mm yields a
    one-waypoint plan that does not move the arm, so every attempt re-approaches the
    (corrected) target from `back` metres behind it along `axis`.
    measure: callable returning a fresh in-hand estimate. The part can creep (pivot) in the
    fingers, so with it the hold is re-measured after every move, and an orientation error
    above rot_tol re-aims the TCP orientation (unless keep_q)."""
    tcp_t = pose_mul(charger_target, pose_inv(hold))
    tq = api.tcp_pose()[1] if keep_q else tcp_t[1]
    cp = np.asarray(tcp_t[0]) + SAG[0]  # start from the offset learned by earlier moves
    axis = np.asarray(axis) / np.linalg.norm(axis)
    reaims = 0
    for _ in range(iters):
        api.move_to((cp - back * axis, tq), speed="slow")
        api.move_to((cp, tq), speed="slow")
        api.wait_until(lambda: False, 0.1)
        if measure is not None:
            hold = measure()
        cur = pose_mul(api.tcp_pose(), hold)
        if measure is not None and not keep_q and reaims < 4 and \
                quat_angle(cur[1], charger_target[1]) > rot_tol:
            # integral correction of the full pose: a heavy part makes the arm sag by a steady
            # offset, so re-commanding the same pose never converges. Shift the command by the
            # offset between the last command and the pose actually reached.
            reaims += 1
            LOG.append(("reaim", round(quat_angle(cur[1], charger_target[1]), 4)))
            want = pose_mul(charger_target, pose_inv(hold))
            cp, tq = pose_mul(pose_mul((cp, tq), pose_inv(api.tcp_pose())), want)
            continue
        # keep the reached orientation: a small residual rotation in the target makes the screw
        # planner's endpoint drift by a few tenths of a mm, so corrections are pure translations
        tq = api.tcp_pose()[1]
        if not keep_q:  # free space (no contact): learn the steady command-vs-reached offset
            SAG[0] = np.asarray(cp) - np.asarray(api.tcp_pose()[0])
        ep = np.asarray(charger_target[0]) - cur[0]
        if np.linalg.norm(ep) < tol:
            break
        cp = cp + ep
    err = float(np.linalg.norm(np.asarray(charger_target[0]) - pose_mul(api.tcp_pose(), hold)[0]))
    LOG.append(round(err * 1000, 2))
    return err


def settled_base(api, n=2000, tol=0.002, max_wait=6.0):
    """Charger base pose once it has stopped moving: averaged estimates 0.25 s apart must agree
    (noise of a 2000-sample average is ~0.2 mm, so 2 mm means real motion)."""
    prev = frames(api, n)["charger_base"]
    for _ in range(int(max_wait / 0.25)):
        api.wait_until(lambda: False, 0.25)
        cur = frames(api, n)["charger_base"]
        if np.linalg.norm(cur[0] - prev[0]) < tol:
            break
        prev = cur
    return frames(api)["charger_base"]


TASK_OBJECTS = ("charger", "receptacle")


def others(api):
    """Every object that is not part of the task: treat it as an obstacle for free-space moves."""
    return [n for n in api.objects() if n not in TASK_OBJECTS]


def obstacle_top(api, names, n=200):
    """Highest point of the named objects (oriented boxes, detections averaged over n calls)."""
    tops = []
    for name in names:
        bs = [b for _ in range(n) for b in api.get_obstacles() if b.name == name]
        c = np.mean([b.center for b in bs], 0)
        q = avg_poses([(b.center, b.quaternion) for b in bs])[1]
        tops.append(c[2] + 0.5 * (np.abs(quat2mat(q)) @ bs[0].extents)[2])
    return max(tops)


def transfer(api, target, clearance=0.05):
    """Free-space move with a held part. The planner only knows the arm, so a straight move can
    drag the part (which hangs below the fingers) through anything in between: rise above the
    highest obstacle, travel there, then descend onto the target, all with obstacle avoidance."""
    names = others(api)
    if not names:
        return api.move_to(target)
    high = max(obstacle_top(api, names) + clearance, target[0][2], api.tcp_pose()[0][2])
    tcp = api.tcp_pose()
    api.move_to(([tcp[0][0], tcp[0][1], high], tcp[1]), avoid=names)
    api.move_to(([target[0][0], target[0][1], high], target[1]), avoid=names)
    ok = api.move_to(target, avoid=names)
    LOG.append(("transfer", round(float(high), 3), ok))
    return ok


def move_checked(api, pose, tries=3, tol=0.003, speed="normal", avoid=None):
    """move_to, re-planned from where the arm ended: the screw planner integrates its Jacobian
    open loop, and on a large rotation the endpoint can be cm off while reporting success.
    Returns whether the pose was actually reached."""
    ok = False
    for _ in range(tries):
        ok = api.move_to(pose, speed=speed, avoid=avoid)
        tcp = api.tcp_pose()
        if np.linalg.norm(np.asarray(tcp[0]) - pose[0]) < tol and quat_angle(tcp[1], pose[1]) < 0.035:
            return True
    return False


def reach_pregrasp(api, base, leans):
    """Move above the charger with the first top grasp the arm can actually reach: the two
    closing directions (and the given lean directions) need very different wrist angles, and
    one of them can be outside the joint range for a given part pose. Returns the grasp."""
    for lean in leans:
        for flip in (False, True):
            grasp = top_grasp_pose(base, api.tcp_pose()[1], lean=lean, flip=flip)
            if move_checked(api, pose_mul(grasp, ([0, 0, -0.05], [1, 0, 0, 0])), avoid=others(api)):
                return grasp
            LOG.append(("unreachable", flip))
    return None


def yaw_of(q):
    x = quat2mat(q)[:, 0]
    return np.arctan2(x[1], x[0])


def yaw_error(base, goal):
    """Rotation about the vertical that brings the charger's x axis onto the insertion axis."""
    return (yaw_of(goal[1]) - yaw_of(base[1]) + np.pi) % (2 * np.pi) - np.pi


def turn_on_table(api, base, angle):
    """Pick the charger up 3 cm, turn it by angle about the vertical, and put it back down."""
    # this grasp is only for turning: lean the approach away from the hand's current position
    # (the natural reaching posture), not toward the pegs, which may point back at the robot
    away = np.asarray(base[0]) - api.tcp_pose()[0]
    away[2] = 0
    api.open_gripper()
    grasp = reach_pregrasp(api, base, [away, -away])
    if grasp is None:
        return False
    move_checked(api, grasp)
    api.close_gripper()
    if not api.wait_until(api.gripper_holding, 0.5):
        api.open_gripper()
        api.move_to(pose_mul(grasp, ([0, 0, -0.05], [1, 0, 0, 0])))
        return False
    p, q = api.tcp_pose()
    api.move_to((p + [0, 0, 0.03], q))
    turned = (p + [0, 0, 0.03], mat2quat(quat2mat(rpy_to_quat(0, 0, angle)) @ quat2mat(q)))
    move_checked(api, turned)
    api.move_to((p + [0, 0, 0.002], turned[1]), speed="slow")
    api.open_gripper()
    api.move_to(pose_mul(api.tcp_pose(), ([0, 0, -0.05], [1, 0, 0, 0])))
    return True


def run(api):
    LOG.clear()
    SAG[0] = np.zeros(3)
    f = frames(api)
    goal = f["goal"]

    def estimate_hold():
        tcp = api.tcp_pose()
        charger = charger_from_base(frames(api)["charger_base"])
        return pose_mul(pose_inv(tcp), charger)

    base = f["charger_base"]
    # the wrist cannot turn the part by much more than 90 deg between grasp and insertion: if
    # its yaw is far from the insertion direction, turn it on the table first (place, regrasp)
    for _ in range(3):
        e = yaw_error(base, goal)
        if abs(e) < np.deg2rad(60):
            break
        LOG.append(("turn", round(float(e), 2)))
        turn_on_table(api, base, float(np.clip(e, -np.pi / 2, np.pi / 2)))
        base = settled_base(api)
    for attempt in range(4):
        api.open_gripper()
        grasp = reach_pregrasp(api, base, [None])  # lean toward the pegs (needed to insert)
        if grasp is None:
            return
        # the charger may have moved since it was measured: look again right before contact
        now = settled_base(api)
        if np.linalg.norm(now[0] - base[0]) > 0.003:
            LOG.append(("moved", attempt, round(float(np.linalg.norm(now[0] - base[0])) * 1000, 1)))
            base = now
            continue
        move_checked(api, grasp)
        api.close_gripper()
        # contact takes a few steps to register: an immediate check reports a miss on a good grasp
        if not api.wait_until(api.gripper_holding, 0.5):
            LOG.append(("missed", attempt))
            api.open_gripper()
            api.move_to(pose_mul(grasp, ([0, 0, -0.05], [1, 0, 0, 0])))
            base = settled_base(api)
            continue
        down = api.tcp_pose()
        api.move_to(pose_mul(([0, 0, 0.05], [1, 0, 0, 0]), down))
        # held is not enough: the part can slide in the fingers while they close, and a grasp at
        # the peg end blocks the insertion. Compare the in-hand pose with the planned one.
        planned = pose_mul(pose_inv(grasp), charger_from_base(base))
        off = np.linalg.norm(np.asarray(estimate_hold()[0]) - planned[0])
        if off < 0.008:
            break
        LOG.append(("regrasp", attempt, round(float(off) * 1000, 1)))
        api.move_to(down, speed="slow")
        api.open_gripper()
        api.move_to(pose_mul(down, ([0, 0, -0.05], [1, 0, 0, 0])))
        base = settled_base(api)

    def goal_offset(dx):
        return pose_mul(goal, ([dx, 0, 0], [1, 0, 0, 0]))

    def goal_offset_tcp(dx, tcp_T_charger):
        return pose_mul(goal_offset(dx), pose_inv(tcp_T_charger))

    hold = estimate_hold()
    transfer(api, goal_offset_tcp(-0.04, hold))
    axis = quat2mat(goal[1])[:, 0]  # insertion direction
    for attempt in range(3):
        hold = estimate_hold()  # re-estimated every attempt: the charger can shift in the fingers
        servo_to(api, goal_offset(-0.019), hold, axis, iters=8, measure=estimate_hold)
        hold = estimate_hold()
        # pegs enter the holes between dx=-0.016 and -0.014; a large residual means they hit the face
        if servo_to(api, goal_offset(-0.013), hold, axis, iters=2, back=0.0, keep_q=True) < 1e-3:
            break
        api.move_to(goal_offset_tcp(-0.03, hold), speed="slow")  # back off and retry
        goal = avg_poses([goal, frames(api)["goal"]])  # refine the goal estimate with fresh samples
    for dx in (-0.007, 0.0):
        hold = estimate_hold()
        servo_to(api, goal_offset(dx), hold, axis, iters=2, back=0.0, keep_q=True)
