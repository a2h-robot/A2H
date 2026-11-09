# Assumptions of pipeline.py (Phase 1 version, commit 0070989)

Line numbers refer to `pipeline.py` as committed after Phase 1.

| # | Category | Assumption | Line(s) relying on it | Attacked in |
|---|---|---|---|---|
| A1 | where objects are | The cream cheese is within reach and has enough free space on at least one side of its closing axis that a top-down hand fits (only shift/tilt along the closing axis are tried, never a push to free it) | 33-53 | v1 (round 1, accepted) |
| A2 | where objects are | The bowl is reachable from directly above, with 10 cm of free space above its rim | 74-77 | |
| A3 | orientation | The cheese lies flat: bbox extents[2] is the vertical one, extents[0]/[1] are horizontal (top = c_z + ext[2]/2, closing axis chosen from ext[0] vs ext[1] and the yaw only) | 13-14, 24-29, 46 | |
| A4 | orientation | The bowl is upright, so the top of its bbox is the rim and its bbox centre is above its opening | 71-76 | |
| A5 | appearance / size | The cheese's narrow side is < 8 cm (fits the open gripper) and it is at most ~2-3 cm tall (grip depth range c_z..top) | 33, 46 | |
| A6 | appearance / size | The bowl opening is wide enough to accept the cheese dropped from 2.5 cm above the rim at the grip site | 76-78 | |
| A7 | motion | The cheese does not move between the averaged detection and the finger closing (single detection at the start of each attempt) | 24, 44-56 | v5 (round 5, accepted) |
| A8 | motion | The bowl does not move after it is detected once (line 71); the place target is never refreshed during the carry or the descent | 71-78 | v2 (round 2, accepted) |
| A9 | between gripper and target | The free-space planner finds a path from the lifted pose to above the bowl; nothing occupies the 10 cm above the rim | 70, 75 | v6 (round 6, accepted) |
| A10 | between gripper and target | The nearest neighbour (by bbox centre) is the one that obstructs the grasp; only one side is considered | 34-37 | |
| A11 | camera / perception | Detections are unbiased with independent noise, so averaging 8 calls removes it; the object name exists (detect never returns None for OBJ/BOWL) | 10-15, 24, 71 | |
| A12 | contact response | Once `gripper_holding()` is true after closing, the cheese stays in the hand during lift and carry (never re-checked; no recovery if dropped) | 57, 70-78 | |
| A13 | contact response | The cheese is held at the grip site (no offset), so moving the grip site over the bowl centre puts the cheese over the bowl | 74-76 | |
| A14 | contact response | The cheese is light and has enough friction for a 1.8 cm-deep pinch to hold it | 46, 56 | |
| A15 | contact response | Opening the gripper releases the cheese and it settles inside the bowl without bouncing out | 78 | |
| A16 | where objects are | The bowl is the only object at its own location (nothing on top of it or inside it) | 71-78 | v4 (round 4, unsolved) |

## Assumptions introduced by the repairs (final pipeline, not yet attacked)

| # | Assumption | Where |
|---|---|---|
| B1 | A neighbour that blocks the grasp is small, movable and graspable by its far-side wall (bowl-like); fixtures/tall objects are never relocated | `relocate_blocker` |
| B2 | A free table spot exists within 8-20 cm of the relocated object and the container may be moved (the goal does not fix its location) | `set_aside` |
| B3 | Objects move slowly enough to be measured in a 2 s window and eventually stop (wait at most 60 s) | `wait_until_still`, `planar_speed` |
| B4 | A release from 2.5 cm above the rim is fine in any of the four tried hand yaws (object symmetric enough under yaw) | `place_in_container` |
| B5 | Leaning the hand forward by up to 0.65 rad still lets a 1.8 cm-tall object be pinched | `top_grasp` (fwd candidates) |
| B6 | The container is empty (the v4 repair was reverted; still violated by v4) | `place_in_container` |
