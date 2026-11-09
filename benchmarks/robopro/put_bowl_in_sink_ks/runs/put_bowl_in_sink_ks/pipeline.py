"""put_bowl_in_sink_ks: put the bowl into the sink.

Plan: average a few noisy detections of the bowl, the sink and the counter; grasp the bowl's rim from
above (a pinch across the rim wall); lift it straight up until its bottom clears the counter and everything
standing in the sink; pick the release spot in the basin with the most room around the bowl's footprint
(the basin may hold other things); carry the bowl straight over to above that spot; lower it straight down;
release it a few centimetres above the basin floor (or as low as the hand can go); open, and go straight up
(the fingers straddle the rim wall along the approach, so they leave the bowl going up).
"""

import numpy as np
import transforms3d as t3d

N_DETECT = 8            # detections averaged per estimate (1 cm / 3 deg noise each)
CLEAR = 0.05            # m, bowl bottom above the counter while it is carried
RELEASE = 0.03          # m, bowl bottom above the basin floor at release
WALL = 0.01             # m, sink wall thickness (the sink's box includes its walls)
GRID = 0.01             # m, release-spot search step
N_OBST = 5              # obstacle snapshots averaged
SETTLED = 0.015         # m, an obstacle that moved less than this in 0.5 s counts as at rest
LEAN_CLEAR = 0.03       # m, a tilted landing needs the bowl's centre this far from walls and obstacles
PITCHES = (0.0, 0.5, 0.9)                  # rad, hand pitched towards the far side to reach further
ROBOT_XY = (0.0, -0.65)                    # robot base (the arms reach out from here)


def rot(axis, angle):
    return t3d.axangles.axangle2mat(axis, angle)


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


def centre(s):
    """Box centre of a detection (ObjectState) or an obstacle (Box)."""
    return np.asarray(s.position if hasattr(s, "position") else s.center, float)


def world_extent(s, axis):
    """Extent of a detected box along a world axis (the box is in the object's own frame)."""
    R = t3d.quaternions.quat2mat(s.quaternion)
    ext = s.bbox_extents if hasattr(s, "bbox_extents") else s.extents
    return float(np.sum(np.abs(R[axis, :]) * ext))


def bottom(s):
    return float(centre(s)[2] - world_extent(s, 2) / 2)


def footprint(o):
    """World xy rectangle [x0, y0, x1, y1] and top z of a detected box."""
    c = centre(o)
    ex, ey = world_extent(o, 0) / 2, world_extent(o, 1) / 2
    top = float(c[2] + world_extent(o, 2) / 2)
    return np.array([c[0] - ex, c[1] - ey, c[0] + ex, c[1] + ey]), top


def obstacles_in(api, sink, floor_z, ignore=("sink", "bowl")):
    """Averaged boxes of the things standing in the basin: [(name, xy rectangle, top z)]."""
    snaps = {}
    for _ in range(N_OBST):
        for o in api.get_obstacles():
            if o.name not in ignore and not o.name.startswith("sink"):
                snaps.setdefault(o.name, []).append(o)
    rect_s, _ = footprint(sink)
    out = []
    for name, ss in snaps.items():
        o = ss[0]
        o.center = np.mean([x.center for x in ss], axis=0)
        r, top = footprint(o)
        overlaps = r[0] < rect_s[2] and r[2] > rect_s[0] and r[1] < rect_s[3] and r[3] > rect_s[1]
        if overlaps and top > floor_z + 0.01:
            out.append((name, r, top))
    return out


def room_at(xy, sink, inside, radius):
    """Free distance [m] around a bowl footprint centred at xy: to the basin's walls and to everything inside."""
    rect_s, _ = footprint(sink)
    x0, y0, x1, y1 = rect_s + [WALL, WALL, -WALL, -WALL]
    x, y = xy
    room = min(x - x0, x1 - x, y - y0, y1 - y) - radius
    for _, r, _ in inside:
        dx = max(r[0] - x, 0.0, x - r[2])
        dy = max(r[1] - y, 0.0, y - r[3])
        room = min(room, float(np.hypot(dx, dy)) - radius)
    return room


def centre_clear(xy, sink, inside):
    """Free distance [m] from xy to the basin's walls and to everything inside (a point, not a footprint)."""
    return room_at(xy, sink, inside, 0.0)


def release_spots(sink, inside, radius, n=6):
    """Bowl-centre xy candidates in the basin, best first, at least 4 cm apart. First the spots with room for
    the bowl's whole footprint (most room, to the nearest centimetre, then nearest the robot, where the arm
    reaches lower); then, in case the arm reaches none of those, spots where only the bowl's centre is over
    free floor (the bowl then settles tilted against what is there, still inside the basin), least overlap
    first."""
    rect_s, _ = footprint(sink)
    x0, y0, x1, y1 = rect_s + [WALL, WALL, -WALL, -WALL]
    full, lean = [], []
    for x in np.arange(x0 + radius / 2, x1 - radius / 2 + 1e-9, GRID):
        for y in np.arange(y0 + radius / 2, y1 - radius / 2 + 1e-9, GRID):
            room = room_at((x, y), sink, inside, radius)
            if room >= 0:
                full.append((-round(room, 2), y, np.array([x, y])))
            elif centre_clear((x, y), sink, inside) >= LEAN_CLEAR:
                lean.append((round(room, 2) * -1, y, np.array([x, y])))
    full.sort(key=lambda c: (c[0], c[1]))
    lean.sort(key=lambda c: c[0])
    out = []
    for tier in (full, lean):
        picked = []
        for _, _, c in tier:
            if all(np.linalg.norm(c - o) >= 0.04 for o in picked):
                picked.append(c)
            if len(picked) == n:
                break
        out += picked
    return out or [np.array([(x0 + x1) / 2, (y0 + y1) / 2])]


def settled_inside(api, sink, floor_z, tries=6):
    """What stands in the basin once it has stopped moving: two averaged looks 0.5 s apart that agree."""
    prev = obstacles_in(api, sink, floor_z)
    for _ in range(tries):
        api.wait_until(lambda: False, timeout_s=0.5)
        cur = obstacles_in(api, sink, floor_z)
        same = sorted(n for n, _, _ in prev) == sorted(n for n, _, _ in cur) and all(
            np.linalg.norm((r[:2] + r[2:]) / 2 - (r2[:2] + r2[2:]) / 2) < SETTLED
            for (n, r, _) in prev for (n2, r2, _) in cur if n == n2)
        if same:
            return cur
        prev = cur
    return prev


def over_basin(b, sink):
    rect_s, _ = footprint(sink)
    c = centre(b)
    return rect_s[0] + WALL < c[0] < rect_s[2] - WALL and rect_s[1] + WALL < c[1] < rect_s[3] - WALL


def reach_over(api, arm, xy, radius):
    """Bring the held bowl's centre over xy: straight over first. A hand pointing straight down does not reach
    the far half of the basin at any height, so then pitch the hand towards the far side (the bowl, held
    rigidly by its rim, tilts with it and is lifted by the extra drop of its low edge), and only then turn
    the hand about the vertical so that the bowl hangs on the far side of it."""
    b = averaged(api, "bowl")
    p, q = api.tcp_pose(arm)
    off = b.position - p                                   # hand -> bowl centre
    shift = np.array([xy[0] - b.position[0], xy[1] - b.position[1], 0.0])
    if api.move_to((p + shift, q), arm, linear=True) or api.move_to((p + shift, q), arm):
        return True
    R = t3d.quaternions.quat2mat(q)
    away = np.array([xy[0] - ROBOT_XY[0], xy[1] - ROBOT_XY[1], 0.0])
    away /= np.linalg.norm(away) + 1e-9
    axis = np.cross([0, 0, -1.0], away)                    # pitching about this tips the hand towards `away`
    for yaw in (0.0, np.pi):
        for pitch in PITCHES:
            Rn = rot(axis, pitch) @ rot([0, 0, 1.0], yaw) @ R
            o = rot(axis, pitch) @ rot([0, 0, 1.0], yaw) @ off
            centre_z = b.position[2] + radius * np.sin(pitch)
            target = np.array([xy[0] - o[0], xy[1] - o[1], centre_z - o[2]])
            if (yaw or pitch) and api.move_to((target, t3d.quaternions.mat2quat(Rn)), arm, speed="slow"):
                return True
    return False


def deliver(api, arm, sink, spots, floor_z, radius):
    """Carry the held bowl over a release spot in the basin and release it there. Just before lowering,
    wait until the basin is still and re-check the spot; if something now occupies it, choose again.
    False (bowl still held) if the arm reaches none of the spots."""
    queue, tries = list(spots), 0
    inside0 = obstacles_in(api, sink, floor_z)
    spot_full = {tuple(np.round(c, 3)): room_at(c, sink, inside0, radius) >= 0 for c in queue}
    while queue and tries < 12:
        spot = queue.pop(0)
        tries += 1
        # bowl centre above the release spot (the next spot if the arm cannot get there)
        if not reach_over(api, arm, spot, radius):
            continue
        inside = settled_inside(api, sink, floor_z)
        full_before = room_at(spot, sink, [], radius) >= 0 and spot_full.get(tuple(np.round(spot, 3)), True)
        ok_now = (room_at(spot, sink, inside, radius) >= -0.005 if full_before
                  else centre_clear(spot, sink, inside) >= LEAN_CLEAR - 0.005)
        if not ok_now:                                     # taken since it was chosen: choose again
            queue = release_spots(sink, inside, radius)
            spot_full.update({tuple(np.round(c, 3)): room_at(c, sink, inside, radius) >= 0 for c in queue})
            continue
        if not over_basin(averaged(api, "bowl"), sink):
            continue
        # straight down into the basin, release above its floor (or as low as the hand gets)
        b = averaged(api, "bowl")
        p, q = api.tcp_pose(arm)
        api.move_to((p + [0, 0, floor_z + RELEASE - bottom(b)], q), arm, linear=True, speed="slow",
                    allow_contact=["sink"])
        api.open_gripper(arm)
        api.move_by(dz=0.15, arm=arm, allow_contact=["sink", "bowl"])
        return True
    return False


def relay_spot(api, sink, radius, table_z):
    """A free counter spot between the two arms (both reach it), on the sink's side of the middle."""
    side = 1.0 if centre(sink)[0] > 0 else -1.0
    items = []
    for o in api.get_obstacles():
        if o.name not in ("sink", "bowl") and not o.name.startswith("sink") and bottom(o) < table_z + 0.05:
            items.append(footprint(o)[0])
    rect_s, _ = footprint(sink)
    for x in (0.05 * side, 0.0, 0.1 * side, -0.05 * side):
        for y in (-0.16, -0.12, -0.2):
            if rect_s[0] - radius < x < rect_s[2] + radius and rect_s[1] - radius < y < rect_s[3] + radius:
                continue                                   # on or over the sink
            if all(max(r[0] - x, 0.0, x - r[2]) ** 2 + max(r[1] - y, 0.0, y - r[3]) ** 2 > (radius + 0.02) ** 2
                   for r in items):
                return np.array([x, y])
    return np.array([0.0, -0.16])


def put_down(api, arm, xy, table_z):
    """Set the held bowl down on the counter, centred at xy, and let go."""
    b = averaged(api, "bowl")
    p, q = api.tcp_pose(arm)
    shift = np.array([xy[0] - b.position[0], xy[1] - b.position[1], 0.0])
    if not api.move_to((p + shift, q), arm, linear=True):
        api.move_to((p + shift, q), arm)
    b = averaged(api, "bowl")
    p, q = api.tcp_pose(arm)
    api.move_to((p + [0, 0, table_z + 0.01 - bottom(b)], q), arm, linear=True, speed="slow", allow_contact=["table"])
    api.open_gripper(arm)
    api.move_by(dz=0.12, arm=arm, allow_contact=["bowl", "table"])


def run(api):
    counter = averaged(api, "table")
    table_z = float(counter.position[2] + world_extent(counter, 2) / 2) if counter is not None else 0.74
    sink = averaged(api, "sink")
    bowl = averaged(api, "bowl")
    floor_z = bottom(sink)
    radius = float(max(bowl.bbox_extents)) / 2
    inside = obstacles_in(api, sink, floor_z)
    spots = release_spots(sink, inside, radius)
    clear_z = max([table_z] + [top for _, _, top in inside]) + CLEAR   # bowl bottom while carried
    arms = ["right", "left"] if bowl.position[0] > 0 else ["left", "right"]

    for k, arm in enumerate(arms):
        if k:                                              # second arm: pick the bowl up where the first left it
            bowl = averaged(api, "bowl")
        if not api.grasp(bowl, arm, strategy="top"):
            return
        # straight up until the bowl's bottom clears the counter and whatever stands in the sink
        b = averaged(api, "bowl")
        api.move_by(dz=max(0.05, clear_z - bottom(b)), arm=arm)
        if deliver(api, arm, sink, spots, floor_z, radius):
            return
        if k:
            return
        # this arm cannot bring the bowl over the sink: hand it over through the counter, between the arms
        put_down(api, arm, relay_spot(api, sink, radius, table_z), table_z)
        api.home(arm)
