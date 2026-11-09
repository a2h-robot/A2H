"""DEV ONLY: pipeline.py with a GT-height place instead of a drop (backend probe)."""
"""pick_ripe_apple: pick the good apple (it ripens green -> red; the spoiled one turns yellow) inside
its ripeness window and drop it into the oscillating basket.

Uses only the RobotAPI (docs/api_spec.md + documented extensions of benchmarks/robodyna/api.py).
"""

import numpy as np

APPLES = ("left apple", "right apple")
RIPE_RGB = np.array([0.92, 0.10, 0.08])      # vivid red = ripe (window centre)
SPOILED_RGB = np.array([0.92, 0.78, 0.08])   # yellowing = spoiled
EE_TO_PINCH = 0.12                           # ee frame sits this far behind the pinch point (tcp)
PRE_DIS = 0.10
# horizontal front pinch: gripper x (approach) in the horizontal plane, fingers open horizontally
FRONT_QUATS = {
    "left": ([-0.707, 0, 0, -0.707], [-0.924, 0, 0, -0.383], [-0.383, 0, 0, -0.924]),
    "right": ([-0.383, 0, 0, -0.924], [-0.924, 0, 0, -0.383], [-0.707, 0, 0, -0.707]),
}
GRASP_LEAD_S = 0.6        # close command issued this long before the predicted ripe time
LOG = []


def log(api, msg, **kw):
    LOG.append({"t": round(api.time(), 2), "msg": msg, **kw})


def _rotx(q):
    w, x, y, z = q
    return np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])


def ee_for_pinch(center, quat, back):
    c = np.asarray(center, dtype=float)
    p = c - (EE_TO_PINCH + back) * _rotx(quat)
    return [*map(float, p), *map(float, quat)]


def observe(api, dur, samples, basket_xs, dt=0.2):
    t_end = api.time() + dur
    while api.time() < t_end:
        for n in APPLES:
            s = api.detect(n)
            if s is not None and s.rgb is not None:
                samples[n].append((s.time, np.asarray(s.rgb[:3]), np.asarray(s.bbox_center)))
        b = api.detect("basket")
        if b is not None:
            basket_xs.append((b.time, float(b.position[0]), float(b.position[1]), float(b.position[2]),
                              np.asarray(b.bbox_extents), np.asarray(b.bbox_center)))
        api.wait(dt)


def progress(rgb, rgb0, target):
    d = target - rgb0
    return float(np.dot(rgb - rgb0, d) / max(1e-6, np.dot(d, d)))


def ripe_fit(samples, rgb0):
    """Linear fit of progress towards ripe red over time -> (slope, t_ripe)."""
    t = np.array([s[0] for s in samples])
    p = np.array([progress(s[1], rgb0, RIPE_RGB) for s in samples])
    A = np.stack([t, np.ones_like(t)], 1)
    k, c = np.linalg.lstsq(A, p, rcond=None)[0]
    t_ripe = (1.0 - c) / k if k > 1e-4 else None
    return float(k), (None if t_ripe is None else float(t_ripe))


def pick_good(samples):
    """Good apple = the one moving towards red, not towards yellow (green channel falls)."""
    score = {}
    for n in APPLES:
        s = samples[n]
        if len(s) < 4:
            score[n] = 0.0
            continue
        rgb = np.array([x[1] for x in s])
        k = len(rgb) // 3 or 1
        start, now = rgb[:k].mean(0), rgb[-k:].mean(0)
        score[n] = float(np.linalg.norm(now - SPOILED_RGB) - np.linalg.norm(now - RIPE_RGB)) \
            + float(start[1] - now[1])
    return max(score, key=score.get), score


def basket_model(basket_xs):
    t = np.array([b[0] for b in basket_xs])
    x = np.array([b[1] for b in basket_xs])
    return t, x


def run(api):
    LOG.clear()
    samples = {n: [] for n in APPLES}
    basket_xs = []
    observe(api, 3.0, samples, basket_xs)
    good, score = pick_good(samples)
    side = -1.0 if good == "left apple" else 1.0
    arm = "left" if side < 0 else "right"
    log(api, "picked", good=good, score=score)

    rgb0 = np.mean([s[1] for s in samples[good][:3]], 0)
    center = np.mean([s[2] for s in samples[good]], 0)

    # move to the pre-grasp pose while the fruit ripens
    api.open_gripper(arm)
    quat = None
    for q in FRONT_QUATS[arm]:
        if api.move_to(ee_for_pinch(center, q, PRE_DIS), arm):
            quat = q
            break
    log(api, "pregrasp", ok=quat is not None, quat=quat, tcp=api.tcp_pose(arm)[:3], center=center.tolist())
    if quat is None:
        return

    # keep watching ripeness (and the basket) until shortly before ripe
    while True:
        observe(api, 0.5, samples, basket_xs)
        center = np.mean([s[2] for s in samples[good]], 0)
        k, t_ripe = ripe_fit(samples[good], rgb0)
        if t_ripe is not None and api.time() >= t_ripe - GRASP_LEAD_S - 1.5:
            break
        if api.time() > 40:
            break
    log(api, "ripe_fit", k=k, t_ripe=t_ripe)

    # straight in, then wait for the ripe moment and close
    ok = api.move_to(ee_for_pinch(center, quat, 0.0), arm, hold_orientation=True)
    log(api, "in", ok=ok, tcp=api.tcp_pose(arm)[:3], center=center.tolist())
    if t_ripe is not None:
        api.wait_until(lambda: api.time() >= t_ripe - GRASP_LEAD_S, 10.0)
    api.close_gripper(arm)
    log(api, "closed", holding=api.gripper_holding(arm), opening=api.gripper_opening(arm))

    # clear outward past the branch tip, then lift
    api.move_by(arm, dx=0.08 * side)
    api.move_by(arm, dz=0.10)
    log(api, "cleared", tcp=api.tcp_pose(arm)[:3], holding=api.gripper_holding(arm))

    # DEV: place (not drop) -- hold the apple inside the basket's success band and release as it passes
    env = api._genv.env
    top = float(env.basket_top_z)
    t, x = basket_model(basket_xs)
    by = float(np.mean([b[2] for b in basket_xs]))
    x_end = float(x.max()) if side > 0 else float(x.min())
    x_hold = x_end - side * 0.04
    tcp = np.asarray(api.tcp_pose(arm)[:3])
    target_c = np.array([x_hold, by + 0.02, top + 0.10])
    ee = np.asarray(api.ee_pose(arm), dtype=float)
    ee[:3] += target_c - tcp
    ok = api.move_to(ee, arm)
    log(api, "over_basket_high", ok=ok, tcp=api.tcp_pose(arm)[:3], top=top)
    def basket_far():
        return abs(float(env.basket.get_pose().p[0]) - x_hold) > 0.15
    api.wait_until(basket_far, 12.0, poll_s=0.04)
    ok = api.move_by(arm, dz=-0.075)
    log(api, "lowered", ok=ok, tcp=api.tcp_pose(arm)[:3], apple=env._apple_eval_p(env.apple).tolist())
    def basket_ready():
        return abs(float(env.basket.get_pose().p[0]) - x_hold) < 0.02
    api.wait_until(basket_ready, 12.0, poll_s=0.02)
    api.open_gripper(arm)
    log(api, "opened", apple=env._apple_eval_p(env.apple).tolist())
    api.move_by(arm, dy=-0.06)
    api.move_by(arm, dz=0.08)
    log(api, "done")
