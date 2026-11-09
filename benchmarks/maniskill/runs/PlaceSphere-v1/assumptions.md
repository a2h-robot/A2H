# Assumptions of pipeline.py (round 0, s0 = 1.0)

Line numbers refer to the Phase 1 pipeline (commit "PlaceSphere-v1: phase 1, s0=1.0").

| # | Category | Assumption | Line(s) | Risk on a real robot |
|---|---|---|---|---|
| A1 | motion | The sphere does not move between the single perception pass at t=0 and the grasp (a sphere rolls if nudged or if the table is not level). | 32, 38-39 | high |
| A2 | between gripper and target | The straight (screw) paths pre-grasp -> grasp -> lift -> above-bin -> release are free of obstacles; nothing is passed to `avoid`, and the lift height (z = 0.12) is fixed. | 38, 43-44, 53-54, 59 | high |
| A3 | orientation | The bin is axis-aligned (yaw 0), so a 45 deg finger yaw puts the fingers in the bin's corners and they clear the rim at release. The finger yaw is fixed in the world, not relative to the bin. | 30, 58-60 | medium |
| A4 | position | Both objects are where a top-down grasp from the initial wrist orientation is reachable; no reachability check, no retry. | 29-30, 38-39 | medium |
| A5 | contact | The sphere is light and grippy enough that one close at the equator holds it through lift and transfer; the failure of `gripper_holding()` is only logged, never retried. | 40-42 | medium |
| A6 | contact | The bin is fixed (kinematic) and its pose measured at t=0 is still valid at release; the bin is never re-detected. | 33, 52 | medium |
| A7 | geometry | The sphere radius is 2 cm and the bin floor is 5 mm thick (hard-coded constants, not taken from the bbox). | 15-16, 52 | low (sim) |
| A8 | perception | Detection noise is zero-mean and independent per call, so the median of 60 calls is accurate to ~1-2 mm; a biased detector would not be averaged out. | 22-25 | medium |
| A9 | perception | Only the target object answers to the name "sphere" (a similar object nearby does not confuse detection). | 32, 48, 57 | low in this backend (name-based detection) |
| A10 | appearance / camera | Colour, lighting and camera pose do not matter (perception is pose-based). | - | none in this backend |
| A11 | contact | After release the sphere settles in the bin and nothing (fingers, an object) pushes it out; the gripper retreats straight up. | 61-64 | low |

# Assumptions after round 2 (pipeline of commit "round 2, axis=obstacle")

A1 (motion before grasp) and A2 (free paths) are now handled. Line numbers refer to the round-2 pipeline.

| # | Category | Assumption | Line(s) | Status |
|---|---|---|---|---|
| B1 | contact | One close at full strength holds the sphere through lift and transit at "normal" speed, whatever its mass and friction; a slip in transit is never detected (holding is checked only right after the close). | 122, 162, 206, 217-219 | attacked in round 3 |
| B2 | motion | The sphere moves at constant velocity (line fit); it does not accelerate, curve or bounce, and it moves slowly enough that a meeting point within reach exists (lead = 0.04 + 4 s x speed). | 63-82, 134 | open |
| B3 | motion | The bin does not move; it is detected once before transit and once above it. | 209, 226 | open |
| B4 | between gripper and target | Obstacles only matter by their top: the carry height is above the highest obstacle, so an overhang or ceiling over the bin would block the descent. | 36-51, 171 | open |
| B5 | position | The sphere is on the table (GRASP_Z = radius); it is never on a raised surface or in a corner where the top-down approach is the only one that fails. | 22, 118-120 | open |
| B6 | geometry | The sphere radius (2 cm) and the bin floor (5 mm) are hard-coded. | 15-16, 210, 227 | low (sim) |
| B7 | perception | Detection noise is zero-mean; colour, lighting and camera do not matter (perception is pose-based). | 58-61 | none in this backend |
| B8 | contact | After release the sphere settles; the fingers opening and the retreat do not disturb it. | 232-235 | open |

# Assumptions after round 4

| # | Category | Assumption | Line(s) | Status |
|---|---|---|---|---|
| C1 | between gripper and target | The goal (bin interior) is empty: nothing has to be removed before placing; a non-task object in the bin is only treated as an obstacle to avoid. | run(): AVOID, bin descent loop | attacked in round 5 |
| C2 | motion | The sphere moves at constant velocity (line fit); no acceleration, curves or bounces. | measure_motion | open |
| C3 | contact | After release the sphere settles; nothing disturbs it. | end of run() | open |
