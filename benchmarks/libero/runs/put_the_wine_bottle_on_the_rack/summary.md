# put_the_wine_bottle_on_the_rack: GAH training summary

Benchmark libero (LIBERO-Goal, Panda, robosuite OSC). Goal: "put the wine bottle on the rack".
TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6. Stopped because 5 variations were accepted.

## Task and Phase 1
The success check (`On wine_bottle_1 wine_rack_1_top_region`) requires the bottle's body origin (the
centre of its bottom) to sit inside a thin slab at the front lip of the rack's upper, inclined shelf,
in contact with the rack. In practice the bottle has to lie on the shelf, its axis along the shelf,
resting against the lip. The margin normal to the shelf is about 3 mm: a bottle perched on an edge
fails.

Phase-1 pipeline: average 10 detections (1 cm / 3 deg noise each); top-grasp the neck; lift; pitch the
wrist 60 deg so the bottle hangs tilted with its bottom over the shelf; lower the bottom next to the
lip; release so it lies down and rolls against the lip. **s0 = 1.0** (recorded in s0.json).

## Rounds
| Round | Variation | Axis | Edit | s_break | s_recovered | Result |
|---|---|---|---|---|---|---|
| 1 | v1 | orientation | rack yawed 90 deg | 0.0 | 0.9 | accepted |
| - | v2a | dynamics | bottle mass 0.6 kg | 0.7 | - | rejected: did not break |
| 2 | v2 | orientation | bottle lying on its side | 0.0 | 0.8 | accepted |
| - | v3a | dynamics | bottle 1.2 kg, friction 0.5 | 1.0 | - | rejected: did not break |
| - | v3b | object_motion | bottle dragged at 2.8 cm/s for 6 s | 1.0 | - | rejected: the motion tipped the bottle; the retry loop plus the lying branch recovered |
| 3 | v3 | distractor | tall box 4 cm beside the neck | 0.0 | 0.8 | accepted |
| 4 | v4 | object_position | rack 10 cm closer to the robot | 0.4 | 0.8 | accepted |
| - | v5a | size | bottle scale 0.75 | 0.7 | - | rejected: did not break |
| - | v5b | obstacle | wall panel at the rack's near end | 0.9 | - | rejected: did not break |
| 5 | v5 | obstacle | board 3.5 cm above the bottle top | 0.0 | 0.9 | accepted |

s_recovered is the minimum over the base task and all accepted variations in that round's final
`--all` run. Round-5 `--all`: base 1.0, v1 0.9, v2 0.9, v3 1.0, v4 0.9, v5 1.0.
Rejected variation files are kept in `variations/rejected/`.

**Axes that broke the program:** orientation of the goal fixture (v1), orientation (pose class) of the
object (v2), clutter beside the grasp (v3), fixture position near the reach limit (v4), and an obstacle
over the grasp corridor (v5).
**Axes that did not:** mass and friction (up to 1.2 kg), object motion (the retry loop recovered), a
smaller object, and a wall beside the goal (the planner went around it). Appearance axes (color,
lighting, camera) were not tried, because perception in this backend is pose-based.

## What the pipeline does now
1. It reads the rack's region frame first. The shelf axis, the up-slope direction and the normal
   define every target orientation and offset.
2. It re-detects the bottle on every attempt (up to 3) and classifies its pose: upright or lying.
3. It grasps:
   - upright: a neck grasp from above, trying 6 yaws, with an in-air yaw correction afterwards;
   - if the top is blocked: a side grasp of the body, then a slide-out before lifting;
   - lying: a grasp across the body, trying several points along it and +-15 deg yaw.
4. It checks `gripper_holding()` after lifting, after turning and before releasing.
5. It places:
   - neck-held: a tilted release, with candidate release poses inside the goal region (offset along
     the shelf 6/8/3 cm, tilt 60/50 deg), descending only on straight lines;
   - body-held: the bottle is laid down horizontally with the approach along the shelf normal.
   It releases only if the hand is within 2 cm of the planned pose.
6. After release it presses the bottle's body onto the shelf with the closed fingertips, so the
   bottle seats flat.

## Rules found (general_rule)
1. Express every target orientation and offset in the frame of the perceived goal object (its axes,
   not just its position), never in world axes that only coincide with it in the nominal scene.
2. Use the perceived orientation of the object to choose the grasp and the in-hand-to-goal
   transform, with one branch per stable pose class the object can be in.
3. Choose the grasp from what the surroundings allow, using a set of alternatives, and fix the
   orientation the goal needs afterwards in free space (re-grasp or in-air rotation).
4. Treat the goal as a region with several reachable release poses inside it, and never execute an
   irreversible action (release, drop) unless the robot verifiably reached the pose it was planned
   for.
5. (not novel, refines 3) Keep alternatives across grasp families with different approach corridors
   (from above, from the side), not just parameter variations of one family, and plan the
   extraction out of a confined grasp before lifting.

## Other lessons
- The goal predicate has a millimetre-scale tolerance normal to the shelf. The resting state after a
  release is stochastic: the bottle bounces, rolls skewed with its neck against the lip, or perches on
  a polygon edge. Reruns of the same scene gave 0.7-1.0. The press added in round 4 reduced this but
  did not remove it. A real robot needs an explicit "seat and verify" step after releasing onto a
  constrained support.
- Fallback motions can be worse than failing. A joint-space plan used as a "descent" near the reach
  limit swung the held object through the scene.

## Cost
- Wall-clock: about 1 h 35 min (10:31 to 12:05 EDT), about 45 eval/gate runs on <sim-host> (7 workers).
- Tokens: not measured exactly; roughly 250k tokens of context in this session.
