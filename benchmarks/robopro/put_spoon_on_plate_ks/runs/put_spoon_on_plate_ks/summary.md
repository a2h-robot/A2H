# GAH training summary: put_spoon_on_plate_ks (RoboPRO, small kitchen)

Goal: "put the spoon on the plate" (RoboPRO `put_spoon_on_plate_ks`, aloha-agilex, config
`bench_demo_kitchens_clean`). Success: the spoon's model origin within 3 cm (x and y separately) of the
plate's origin, both grippers open. TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6.

**Stopped after 5 rounds: 3 accepted variations (v1, v4, v7, all `novel: true`), 2 unsolved (v8, v9), 4 rejected by the break gate.** The convergence rule (5 accepted, or 2 consecutive non-novel accepted rounds) was not met. The program withstood every other single-assumption attack designed or screened (13 candidates screened outside the gate); the only class that still breaks it, no room for the hand at the grasp point, needs pushing the object into the open or whole-arm clearance planning, which the pipeline did not achieve.

## Phase 1

- First draft: average 8 detections of the spoon, grasp with the arm on its side, lift, carry, place over the
  plate's centre. Three things went wrong on the way to s0:
  - `bbox_extents` are in the object's frame and the plate's frame is rotated, so its diameter was used as its
    height: the spoon was released 10 cm above the plate and bounced;
  - the light spoon slipped out of the hand during fast carries while `place()` still returned True: carry
    slowly and check `gripper_holding()` after the lift and the carry;
  - every miss was 3.4-4 cm on one axis: the success check measures the spoon's model origin, which is 2.5 cm
    from its bounding-box centre (the backend then started reporting task objects' model frames, ISSUES 3).
- **s0 = 0.9** (10 seeds). Seed 40030 fails in every scene: the spoon lies right next to the microwave and the
  final grasp approach is refused.

## Variations

| # | Variation | Axis | Assumption attacked | s_break | Result | Repairs |
|---|---|---|---|---|---|---|
| 1 | v1: the plate slides 7 cm towards the robot during the pick | object_motion | A1: the goal stays where it was seen at the start | 0.0 | **accepted**, s_recovered 1.0 | 1 |
| - | v2: a flat box in the middle of the plate | distractor | A7: the plate's top is clear | 0.5 | rejected (the long spoon still comes to rest on the box, xy-only check) | - |
| - | v3: spoon in the right front corner, plate far left | object_position | A3: one arm reaches spoon and plate | 0.9 | rejected (the arm reaches across) | - |
| 2 | v4: the spoon lies upside down | orientation | A4: the library grasp frames point at the spoon from above | 0.0 | **accepted**, s_recovered 0.9 | 1 |
| - | v5: a glass standing on the plate centre | distractor | A7 | 0.7 | rejected (3 cm per-axis tolerance leaves room beside a glass) | - |
| - | v6: a 25 cm divider between spoon and plate | obstacle | A6: the held spoon clears obstacles | 0.5 | rejected | - |
| 3 | v7: heavier (250 g), slippery (friction 0.2) spoon | dynamics | A5: the pinch holds the spoon | 0.1 | **accepted**, s_recovered 0.8 | 4 |
| - | v8: the spoon partly under a low overhang (7.5 cm exposed) | obstacle | A4: room for the hand above the grasp point | 0.0 | **unsolved** (best 0.0) | 3 |
| - | v9: as v8, but ~10 cm of the bowl end exposed (labelled parameter tuning) | obstacle | A4 | 0.0 | **unsolved** (best 0.3) | 2 |

Gate and screening statistics: 9 variations gated: 5 passed (v1, v4, v7, v8, v9), 4 rejected by the break gate (v2, v3, v5, v6), none by the variance gate, integrity or validity. 3 accepted and recovered, 2 unsolved. 13 candidates were screened against the frozen pipeline outside the gate (`screening/results.txt`); 3 broke it (one became v7, the other two are the no-room-for-the-hand class).

### Axes that broke the program
- the goal moving after it was perceived (v1);
- an object in a non-nominal orientation (v4);
- a heavier, slippery object (v7);
- no room for the hand at the grasp point (v8, v9, screening s5).

### Axes that did not break it (gate rejections and screening, `screening/results.txt`)
- something on the target (a flat box, a glass), a divider or a wall next to the plate, a plate on a stand;
- the far-side placement (the arm reaches across), a spoon nudged before the grasp, a box crossing the carry;
- a teaspoon or a serving spoon, the plate nudged late or drifting slowly, a box resting on the spoon.

## Rules found (reflections.jsonl)
- Round 1 (v1, object_motion, accepted): Re-perceive the goal location right before acting on it and after every long step, waiting until it is stationary, instead of reusing an estimate taken before the manipulation began.
- Round 2 (v4, orientation, accepted): Do not depend on one grasp source tied to an object's nominal pose; when it yields nothing, derive a grasp from the object's currently observed geometry, approaching vertically and closing across its narrow horizontal dimension just above its underside.
- Round 3 (v7, dynamics, accepted): Check every grasp with a short test lift before carrying, and when the object slips, change the grasp (nearer the centre of mass, another contact height or position) instead of repeating the one that just failed.
- Round 4 (v8, obstacle, unsolved): Before reaching, check that the whole hand plus the planner's safety margin fits around the grasp point, because a reach that is refused halfway still pushes the object deeper into clutter; if no point on the object has that room, the object must be moved into the open before it can be grasped.
- Round 5 (v9, obstacle, unsolved) (not novel: same rule as round 4): Before reaching, check that the whole hand plus the planner's safety margin fits around the grasp point, because a reach that is refused halfway still pushes the object deeper into clutter; if no point on the object has that room, the object must be moved into the open before it can be grasped.

## Final pipeline
Final pipeline = round 3 (commit "round 3, axis=dynamics"); the round-4 and round-5 attempts are in `unsolved/`.
`eval.py --all`, 10 seeds each:
- at acceptance of round 3: base 0.8, v1 1.0, v4 0.9, v7 0.8;
- final re-run (a quiet GPU, 2 workers, 338 s for 40 episodes): base 0.9, v1 0.9, v4 0.9, **v7 0.7**.

v7 (250 g, friction 0.2) is physically marginal: the pinch holds on some contacts and not others, so it scores 0.7-0.8 between runs. Seed 40030 fails in every scene (the grasp approach next to the microwave is refused).
