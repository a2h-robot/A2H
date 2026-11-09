"""catch_shelf_marble: keep the belt bowl under the descending red ball by pressing the left / right keys.

The ball descends a stack of tilted shelves (that the API cannot see) slowly (~4.5 cm/s
horizontally) and finally falls 0.15-0.5 m onto the belt. The bowl moves at a fixed speed
(~18 cm/s) only while a key is held, ~4x faster than the sliding ball, so a closed-loop servo keeps
the bowl under an aim point:
  - while the ball slides: its x plus a lead in its direction of travel (the final fall leaves a
    shelf edge with extra horizontal speed, so it lands a few cm further along);
  - while the ball falls (fast downward z velocity): the ballistic landing x at the bowl rim height,
    from a fit of the recent x(t), z(t) samples.

Each arm hovers just above its key with the gripper closed; a "press" is a ~2 cm move down that
pushes the spring key past its trigger depth, a "release" is the move back up. The press is held
until the bowl reaches the aim point minus the measured release overshoot.
"""
import copy
import os

import numpy as np

DEBUG = bool(os.environ.get("PIPE_DEBUG"))

BALL, BOWL = "red ball", "bowl"
KEYS = {"left": "left key", "right": "right key"}   # left key drives the bowl to -x, right key to +x
N_AVG = 20                # detections averaged per observation
HOVER_DZ = 0.180          # gripper (ee) height above the key top while idle
PRESS_DZ = 0.150          # ee height above the key top while pressing
SLIDE_LEAD = 0.045        # aim this far ahead of a sliding ball, in its direction of travel
V_MOVING = 0.015          # |vx| above this: the ball is sliding sideways
VZ_FALL = -0.3            # vz below this: the ball is falling
JUMP_V = 0.12             # m/s: x moving faster than any slide = a jump or a fast fall
JUMP_LEAD = 0.050         # after a jump, aim this far beyond the last x in the jump direction
JUMP_DX = 0.02            # minimum x step counted as a jump
FALL_DZ = 0.012           # z drop between two samples that counts as falling
FALL_VX = 0.17            # sideways speed of the final fall, in the direction of travel
LOW_Z = 0.03              # ball this close above the rim height: hands off
MIN_T_LAND = 0.12         # do not start a press when the ball lands sooner than this
DEADBAND = 0.020          # start a press when the bowl is further than this from the aim point
HIST_S = 0.6              # fit window
POLL = 0.02
T_END = 56.0
G = 9.81


def _avg(api, name, n=N_AVG):
    ps = []
    for _ in range(n):
        s = api.detect(name)
        if s is not None:
            ps.append(np.asarray(s.position, dtype=float))
    return np.mean(ps, axis=0) if ps else None


class Servo:
    def __init__(self, api, catch_z):
        self.api = api
        self.catch_z = catch_z  # ball-centre height at which it enters the bowl
        self.hist = []          # (t, x, z) of the ball
        self.overshoot = {"left": 0.012, "right": 0.012}   # bowl travel after the release command
        self.press_dz = PRESS_DZ
        self.mode = "idle"
        self.last_dir = 0.0
        self.t_land = None

    def observe(self):
        api = self.api
        b = _avg(api, BALL)
        t = api.time()
        if b is not None:
            self.hist.append((t, float(b[0]), float(b[2])))
        self.hist = [h for h in self.hist if t - h[0] <= HIST_S]
        return b

    def aim(self):
        """Aim x for the bowl. Also sets self.t_land (s until the ball reaches the bowl rim while it
        falls, else None) and self.mode."""
        self.t_land = None
        h = self.hist
        if not h:
            return None
        t = np.array([p[0] for p in h]) - h[-1][0]
        x = np.array([p[1] for p in h])
        z = np.array([p[2] for p in h])
        n = len(t)
        if n >= 2:
            dt = np.maximum(np.diff(t), 1e-3)
            dx, dz = np.diff(x), np.diff(z)
            falling = (dz < -FALL_DZ) & (dz / dt < VZ_FALL)            # step i -> i+1 is part of a fall
            jump = (np.abs(dx) > JUMP_DX) & (np.abs(dx) / dt > JUMP_V) & ~falling
        else:
            falling = jump = np.zeros(0, bool)
        if len(falling) and falling[-1]:
            # falling now: the tail of consecutive fall steps
            k = n - 1
            while k > 0 and falling[k - 1]:
                k -= 1
            k = max(0, k - 1)                                           # include the last shelf sample
            ts, xs, zs = t[k:], x[k:], z[k:]
            self.mode = "fall"
            if len(ts) >= 3:
                A = np.stack([np.ones_like(ts), ts], 1)
                z0, vz0 = np.linalg.lstsq(A, zs + 0.5 * G * ts ** 2, rcond=None)[0]
                vx = float(np.polyfit(ts, xs, 1)[0])
            else:
                z0 = zs[-1]
                vz0 = (zs[-1] - zs[0]) / max(ts[-1] - ts[0], 1e-3)
                vx = (xs[-1] - xs[0]) / max(ts[-1] - ts[0], 1e-3)
            if abs(vx) < 0.5 * FALL_VX:          # too few samples to see the fall's sideways speed yet
                vx = self.last_dir * FALL_VX
            dzr = float(zs[-1] - self.catch_z)
            if dzr <= 0:
                self.t_land = 0.0
                return float(xs[-1])
            vz_now = float(vz0) if len(ts) < 3 else float(vz0)
            self.t_land = float((vz_now + np.sqrt(vz_now ** 2 + 2 * G * dzr)) / G)
            return float(xs[-1] + np.clip(vx, -0.3, 0.3) * self.t_land)
        # start of the current segment: after the last fall step or sideways jump
        cut = np.where(falling | jump)[0]
        k0 = int(cut[-1]) + 1 if len(cut) else 0
        if len(cut) and jump[cut[-1]]:
            self.last_dir = float(np.sign(dx[cut[-1]]))
            if n - k0 <= 3:                       # just jumped (the ball is about to drop that way)
                self.mode = "jump"
                return float(x[-1]) + self.last_dir * JUMP_LEAD
        ts, xs = t[k0:], x[k0:]
        if len(ts) < 4 or ts[-1] - ts[0] < 0.15:
            self.mode = "idle"
            return float(np.mean(xs[-3:]))
        vx, x0 = (float(v) for v in np.polyfit(ts, xs, 1))
        if abs(vx) > V_MOVING:
            self.mode = "slide"
            self.last_dir = float(np.sign(vx))
            return x0 + self.last_dir * SLIDE_LEAD
        self.mode = "idle"
        return x0

    def bowl_x(self):
        b = _avg(self.api, BOWL)
        return None if b is None else float(b[0])

# gripper volume (WSG-50 on UR5, home yaw): palm/fingers ~16 x 8 cm in x, y, reaching this far below the
# flange (ee) and the wrist a little above it
GRIP_XY = (0.16, 0.09)
GRIP_BELOW_EE = 0.17
GRIP_ABOVE_EE = 0.05
CLEAR = 0.015
OBST_CHECK_S = 0.2       # re-check the obstacles this often [s]
OBST_LOOKAHEAD_S = 1.0   # plan against where moving obstacles will be this much later [s]
OBST_MOVED = 0.03        # an obstacle that moved more than this triggers re-planning [m]
KEY_CAPTURE = 0.035     # press point may sit this far from the key centre (the key fires within ~4.5 cm)


def _footprint_pen(key, cfg, obstacles, clear=CLEAR):
    """Overlap area [m^2] of the gripper footprint in configuration cfg with the obstacle boxes
    (each grown by clear) over the press-to-hover height range."""
    kx, ky, top = key
    yaw, dx, dy = cfg[:3]
    zlo, zhi = top + PRESS_DZ - GRIP_BELOW_EE, top + HOVER_DZ + GRIP_ABOVE_EE
    hx, hy = (GRIP_XY if yaw == 0.0 else GRIP_XY[::-1])
    cx, cy = kx + dx, ky + dy
    pen = 0.0
    for o in obstacles:
        ox, oy = (np.asarray(o.extents[:2]) / 2 + clear)
        ix = min(cx + hx / 2, o.center[0] + ox) - max(cx - hx / 2, o.center[0] - ox)
        iy = min(cy + hy / 2, o.center[1] + oy) - max(cy - hy / 2, o.center[1] - oy)
        iz = min(zhi, o.center[2] + o.extents[2] / 2) - max(zlo, o.center[2] - o.extents[2] / 2)
        if ix > 0 and iy > 0 and iz > 0:
            pen += ix * iy
    return pen


def _press_config(key, obstacles, clear=CLEAR):
    """(yaw, dx, dy, top_of_nearby_obstacles): the gripper yaw and press-point offset closest to the key
    centre whose gripper volume, from the press height up to above the hover height, clears every
    obstacle box by CLEAR."""
    kx, ky, top = key
    zlo, zhi = top + PRESS_DZ - GRIP_BELOW_EE, top + HOVER_DZ + GRIP_ABOVE_EE
    near = [o for o in obstacles if np.hypot(o.center[0] - kx, o.center[1] - ky) < 0.3]
    high = max([float(o.center[2] + o.extents[2] / 2) for o in near] + [0.0])
    best, best_cost = (0.0, 0.0, 0.0), None
    offs = [(dx, dy) for dx in np.arange(-KEY_CAPTURE, KEY_CAPTURE + 1e-9, 0.005)
            for dy in np.arange(-KEY_CAPTURE, KEY_CAPTURE + 1e-9, 0.005) if np.hypot(dx, dy) <= KEY_CAPTURE + 1e-9]
    for yaw in (0.0, np.pi / 2):
        hx, hy = (GRIP_XY if yaw == 0.0 else GRIP_XY[::-1])
        for dx, dy in offs:
            cx, cy = kx + dx, ky + dy
            pen = 0.0
            for o in near:
                ox, oy = (np.asarray(o.extents[:2]) / 2 + clear)
                ix = min(cx + hx / 2, o.center[0] + ox) - max(cx - hx / 2, o.center[0] - ox)
                iy = min(cy + hy / 2, o.center[1] + oy) - max(cy - hy / 2, o.center[1] - oy)
                iz = min(zhi, o.center[2] + o.extents[2] / 2) - max(zlo, o.center[2] - o.extents[2] / 2)
                if ix > 0 and iy > 0 and iz > 0:
                    pen += ix * iy
            cost = (pen > 0, pen, np.hypot(dx, dy) + 0.01 * (yaw != 0.0))
            if best_cost is None or cost < best_cost:
                best, best_cost = (yaw, float(dx), float(dy)), cost
    return (*best, high)


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def _yawed(q, yaw):
    """q rotated by yaw about the world z axis (the gripper keeps pointing down)."""
    return _qmul([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)], list(q))


def _snapshot(api, n=5):
    """Obstacle boxes (everything but the ball and the bowl) with centres averaged over n readings."""
    reads = [api.get_obstacles(exclude=(BALL, BOWL)) for _ in range(n)]
    out = []
    for b in reads[0]:
        cs = [o.center for r in reads for o in r if o.name == b.name]
        b.center = np.mean(cs, axis=0)
        out.append(b)
    return out


def _moved(api, snap, tol=OBST_MOVED):
    """True if any obstacle appeared, vanished or moved more than tol since the snapshot (one reading)."""
    now = {o.name: np.asarray(o.center) for o in api.get_obstacles(exclude=(BALL, BOWL))}
    old = {o.name: np.asarray(o.center) for o in snap}
    if set(now) != set(old):
        return True
    return any(np.linalg.norm(now[k][:2] - old[k][:2]) > tol for k in now)


def run(api):
    api.close_gripper("left")
    api.close_gripper("right")
    keys = {}
    for side, name in KEYS.items():
        st = [api.detect(name) for _ in range(50)]
        c = np.mean([s.bbox_center for s in st], axis=0)
        top = float(c[2] + np.mean([s.bbox_extents[2] for s in st]) / 2)
        keys[side] = (float(c[0]), float(c[1]), top)
    home_q = {arm: api.ee_pose(arm)[3:] for arm in ("left", "right")}
    bowl = [api.detect(BOWL) for _ in range(10)]
    rim_z = float(np.mean([b.bbox_center[2] + min(b.bbox_extents) / 2 for b in bowl]))   # height = smallest extent
    ball_r = float(np.mean([api.detect(BALL).bbox_extents[0] for _ in range(5)])) / 2
    # everything else on the table is something the arms must not hit; things get moved, so this
    # snapshot is re-checked during the task (see refresh)
    obstacles = _snapshot(api)

    press_dz = {"left": PRESS_DZ, "right": PRESS_DZ}
    misses = {"left": 0, "right": 0}
    cfg = {side: _press_config(keys[side], obstacles) for side in KEYS}

    def pose(side, dz):
        kx, ky, top = keys[side]
        yaw, ox, oy = cfg[side][:3]
        return [kx + ox, ky + oy, top + dz, *_yawed(home_q[side], yaw)]

    def go(side, dz):
        """Collision-aware move; True only if the gripper really got there."""
        target = pose(side, dz)
        if not api.move_to(target, side, avoid=obstacles):
            return False
        return float(np.linalg.norm(np.asarray(api.ee_pose(side)[:3]) - target[:3])) < 0.012

    def to_hover(side):
        # turn and line up above everything near the key first, then come straight down
        safe_dz = max(HOVER_DZ, cfg[side][3] - keys[side][2] + GRIP_BELOW_EE + 0.02)
        if safe_dz > HOVER_DZ:
            go(side, safe_dz)
        go(side, HOVER_DZ)

    for side in KEYS:
        to_hover(side)

    last_check = [api.time()]
    snap_t = [api.time()]

    def refresh(force=False):
        """Re-read the obstacles; if something moved, re-plan the press configurations and move any
        arm whose configuration changed (up and clear first, then down in the new one)."""
        if not force and api.time() - last_check[0] < OBST_CHECK_S:
            return
        last_check[0] = api.time()
        if not force and not _moved(api, obstacles):
            return
        prev = {o.name: (np.asarray(o.center).copy(), snap_t[0]) for o in obstacles}
        obstacles[:] = _snapshot(api)
        snap_t[0] = api.time()
        # where moving obstacles will be shortly: plan against the union of now and then, so the arm
        # gets out of the way before something arrives instead of after it hits
        ahead = []
        for o in obstacles:
            if o.name in prev and snap_t[0] - prev[o.name][1] > 0.05:
                v = (np.asarray(o.center) - prev[o.name][0]) / (snap_t[0] - prev[o.name][1])
                if np.linalg.norm(v[:2]) > 0.02:
                    for k in (0.5, 1.0):
                        f = copy.copy(o)
                        f.center = np.asarray(o.center) + v * OBST_LOOKAHEAD_S * k
                        ahead.append(f)
        plan_obs = obstacles + ahead
        for side in KEYS:
            # re-planning because something moved: take a wider margin so the new choice survives the
            # obstacle settling a few cm further on (a second re-plan costs another second)
            new_cfg = _press_config(keys[side], plan_obs, 2 * CLEAR)
            old_cfg = cfg[side]
            # keep the current configuration while it still clears everything (with half the margin,
            # so sensor noise cannot make the arm hop between equivalent configurations)
            if _footprint_pen(keys[side], old_cfg, plan_obs, CLEAR / 2) > 0 and \
                    (new_cfg[0] != old_cfg[0] or np.hypot(new_cfg[1] - old_cfg[1], new_cfg[2] - old_cfg[2]) > 0.006):
                lift = max(HOVER_DZ, new_cfg[3] - keys[side][2] + GRIP_BELOW_EE + 0.02)
                go(side, lift)                    # clear of the new obstacle in the old configuration
                cfg[side] = new_cfg
                to_hover(side)
            else:
                cfg[side] = (*old_cfg[:3], new_cfg[3])

    s = Servo(api, rim_z + ball_r)
    landed_since = None
    while api.time() < T_END:
        refresh()
        b = s.observe()
        if b is None:
            api.wait(POLL)
            continue
        # done once the ball rests low (in the bowl or on the belt)
        if b[2] < rim_z:
            landed_since = landed_since or api.time()
            if api.time() - landed_since > 0.6:
                break
        aim = s.aim()
        bx = s.bowl_x()
        if aim is None or bx is None:
            api.wait(POLL)
            continue
        err = aim - bx
        if DEBUG:
            print(f"   [pipe] t={api.time():.2f} ball=({b[0]:+.3f},{b[2]:.3f}) mode={s.mode} aim={aim:+.3f} "
                  f"bowl={bx:+.3f} t_land={s.t_land}", flush=True)
        # once the ball is at the rim it is in the bowl or past it: moving the bowl now can only hurt
        late = (s.t_land is not None and s.t_land < MIN_T_LAND) or b[2] < s.catch_z + LOW_Z
        if late or abs(err) <= max(DEADBAND, 0.5 * min(s.overshoot.values())):
            api.wait(POLL)
            continue
        side = "right" if err > 0 else "left"
        sign = 1.0 if side == "right" else -1.0
        x_before = bx
        refresh(force=misses[side] > 0)
        if not go(side, press_dz[side]):
            go(side, HOVER_DZ)        # the stroke did not complete: back to hover, re-decide

        def reached():
            bb = s.observe()
            if bb is not None and bb[2] < s.catch_z + LOW_Z:
                return True
            a = s.aim()
            x = s.bowl_x()
            return a is None or x is None or sign * (a - x) <= s.overshoot[side]

        api.wait_until(reached, timeout_s=2.5, poll_s=POLL)
        x_rel = s.bowl_x()
        go(side, HOVER_DZ)
        if api.ee_pose(side)[2] < keys[side][2] + HOVER_DZ - 0.006:   # stopped short: still on the key
            go(side, HOVER_DZ + 0.01)
        api.wait(0.02)
        x_after = s.bowl_x()
        if x_rel is not None and x_after is not None:
            ov = sign * (x_after - x_rel)
            s.overshoot[side] = float(np.clip(0.6 * s.overshoot[side] + 0.4 * ov, 0.0, 0.04))
        if x_after is not None and abs(x_after - x_before) < 0.003:
            # the key did not fire: press deeper, and after repeated misses try another configuration
            misses[side] += 1
            press_dz[side] = max(0.13, press_dz[side] - 0.005)
        else:
            misses[side] = 0
