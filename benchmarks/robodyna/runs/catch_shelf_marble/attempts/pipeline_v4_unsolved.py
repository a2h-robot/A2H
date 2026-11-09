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
DROP_X = 0.52            # |x| of the set-down spot for items taken out of the bowl
TABLE_Z = 0.74           # table top (scene convention)
FINGER_REACH = 0.165    # fingertips are this far beyond the flange (ee) along the approach axis
KEY_CAPTURE = 0.035     # press point may sit this far from the key centre (the key fires within ~4.5 cm)


def _press_config(key, obstacles):
    """(yaw, dx, dy, top_of_nearby_obstacles): the gripper yaw and press-point offset closest to the key
    centre whose gripper volume, from the press height up to above the hover height, clears every
    obstacle box by CLEAR."""
    kx, ky, top = key
    zlo, zhi = top + PRESS_DZ - GRIP_BELOW_EE, top + HOVER_DZ + GRIP_ABOVE_EE
    near = [o for o in obstacles if np.hypot(o.center[0] - kx, o.center[1] - ky) < 0.3]
    best, best_cost = (0.0, 0.0, 0.0), None
    offs = [(dx, dy) for dx in np.arange(-KEY_CAPTURE, KEY_CAPTURE + 1e-9, 0.005)
            for dy in np.arange(-KEY_CAPTURE, KEY_CAPTURE + 1e-9, 0.005) if np.hypot(dx, dy) <= KEY_CAPTURE + 1e-9]
    for yaw in (0.0, np.pi / 2):
        hx, hy = (GRIP_XY if yaw == 0.0 else GRIP_XY[::-1])
        for dx, dy in offs:
            cx, cy = kx + dx, ky + dy
            pen = 0.0
            for o in near:
                ox, oy = (np.asarray(o.extents[:2]) / 2 + CLEAR)
                ix = min(cx + hx / 2, o.center[0] + ox) - max(cx - hx / 2, o.center[0] - ox)
                iy = min(cy + hy / 2, o.center[1] + oy) - max(cy - hy / 2, o.center[1] - oy)
                iz = min(zhi, o.center[2] + o.extents[2] / 2) - max(zlo, o.center[2] - o.extents[2] / 2)
                if ix > 0 and iy > 0 and iz > 0:
                    pen += ix * iy
            cost = (pen > 0, pen, np.hypot(dx, dy) + 0.01 * (yaw != 0.0))
            if best_cost is None or cost < best_cost:
                best, best_cost = (yaw, float(dx), float(dy)), cost
    # obstacles close to the descent column (gripper footprint + 5 cm): approach from above them
    yaw, dx, dy = best
    hx, hy = (GRIP_XY if yaw == 0.0 else GRIP_XY[::-1])
    high = 0.0
    for o in near:
        ox, oy = np.asarray(o.extents[:2]) / 2 + 0.05
        if abs(o.center[0] - (kx + dx)) < hx / 2 + ox and abs(o.center[1] - (ky + dy)) < hy / 2 + oy:
            high = max(high, float(o.center[2] + o.extents[2] / 2))
    return (*best, high)


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def _yawed(q, yaw):
    """q rotated by yaw about the world z axis (the gripper keeps pointing down)."""
    return _qmul([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)], list(q))


def _in_bowl(api, rim_z):
    """Objects (other than the ball) standing in the bowl."""
    b = _avg(api, BOWL)
    out = []
    for name in api.objects():
        if name in (BALL, BOWL):
            continue
        st = api.detect(name)
        if st is None:
            continue
        c = np.asarray(st.bbox_center)
        if np.hypot(c[0] - b[0], c[1] - b[1]) < 0.05 and c[2] - st.bbox_extents[2] / 2 < rim_z:
            out.append(name)
    return out


def _clear_bowl(api, home_q, rim_z):
    """Lift anything standing in the bowl out of it and set it down on the table, with a horizontal
    gripper coming in from the robot side just above the rim (below the shelves, which a top-down
    approach could hit). Returns the arm used or None."""
    used = None
    for attempt in range(2):
        occ = _in_bowl(api, rim_z)
        if not occ:
            break
        name = occ[0]
        sts = [api.detect(name) for _ in range(10)]
        c = np.mean([st.bbox_center for st in sts], axis=0)
        top = float(c[2] + np.mean([st.bbox_extents[2] for st in sts]) / 2)
        # the right arm reaches the middle of the belt in this pose; the left one only well left of it
        arm = ("right" if c[0] >= -0.10 else "left") if attempt == 0 else ("left" if used == "right" else "right")
        used = arm
        if api.gripper_opening(arm) < 0.8:
            api.open_gripper(arm)
        q = _qmul([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0], list(home_q[arm]))   # fingers point +y
        # finger band (3 cm tall around the flange height): as low as the rim allows, below the top
        z = float(min(rim_z + 0.022, top - 0.012))
        x = float(c[0])
        y_grip = float(c[1]) + 0.01 - FINGER_REACH        # fingertips just past the item's centre line
        api.move_to([x, y_grip - 0.15, z + 0.04, *q], arm)
        api.move_to([x, y_grip - 0.15, z, *q], arm)
        if not api.move_to([x, y_grip, z, *q], arm, hold_orientation=True):
            api.move_by(arm, dz=0.08)
            continue                                   # this arm cannot get there: try the other one
        api.close_gripper(arm)
        # straight up until the item's bottom is well above the rim, then slowly back out towards the robot
        api.move_to([x, y_grip, z + 0.075, *q], arm, hold_orientation=True)
        # back out towards the robot (clear of the bowl's travel, between the two keys) and set it down
        y_out = max(y_grip - 0.17, -0.34)
        api.move_to([x, y_out, z + 0.075, *q], arm, hold_orientation=True)
        z_down = TABLE_Z + (z - c[2]) + float(np.mean([st.bbox_extents[2] for st in sts])) / 2 + 0.01
        for up in (0.02, 0.05):                   # as low as the arm can reach here, so it lands standing
            if api.move_to([x, y_out, z_down + up, *q], arm):
                break
        api.open_gripper(arm)
        api.move_by(arm, dy=-0.07)                      # fingers off the item before anything closes them
    return used

def run(api):
    home_q = {arm: api.ee_pose(arm)[3:] for arm in ("left", "right")}
    bowl0 = [api.detect(BOWL) for _ in range(10)]
    rim_z = float(np.mean([b.bbox_center[2] + min(b.bbox_extents) / 2 for b in bowl0]))   # height = smallest extent
    cleared_with = None
    if _in_bowl(api, rim_z):          # the receptacle must be empty before the ball arrives
        cleared_with = _clear_bowl(api, home_q, rim_z)
    for arm in ("left", "right"):     # closed fingers press the key cleanly (open ones can catch on its bezel)
        if api.gripper_opening(arm) > 0.5:
            api.close_gripper(arm)
    keys = {}
    for side, name in KEYS.items():
        st = [api.detect(name) for _ in range(50)]
        c = np.mean([s.bbox_center for s in st], axis=0)
        top = float(c[2] + np.mean([s.bbox_extents[2] for s in st]) / 2)
        keys[side] = (float(c[0]), float(c[1]), top)
    ball_r = float(np.mean([api.detect(BALL).bbox_extents[0] for _ in range(5)])) / 2
    # everything else on the table is something the arms must not hit (static, so measure it once)
    obstacles = api.get_obstacles(exclude=(BALL, BOWL))

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

    for side in KEYS:
        # turn and line up above everything near the key first, then come straight down
        safe_dz = max(HOVER_DZ, cfg[side][3] - keys[side][2] + GRIP_BELOW_EE + 0.02)
        if safe_dz > HOVER_DZ:
            go(side, safe_dz)
        go(side, HOVER_DZ)

    s = Servo(api, rim_z + ball_r)
    landed_since = None
    while api.time() < T_END:
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
