# GAH training summary: put_cup_on_coaster (RoboPRO)

Goal: "Place the cup onto the coaster on the table" (RoboPRO `put_cup_on_coaster`, study scene, clean config,
aloha-agilex; 4 cup models, 3 furniture layouts). TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6. Success:
cup within 2 cm of the coaster centre in x and in y, both grippers open.

**Converged with 5 accepted variations** (v1, v2, v3, v6, v8, all `novel: true`), the MAX_ROUNDS stop rule. The run
took 7 rounds:
- 2 unsolved (v4, v7);
- 1 design rejected by the validity gate (v5).

Final pipeline = round 7 (commit "round 7, axis=distractor"). `eval.py --all`, 10 seeds each: **base 0.9, v1 0.9,
v2 0.8, v3 0.8, v6 0.9, v8 0.8.**

## Phase 1

| Eval | Pipeline | Result |
|---|---|---|
| p1a | average 8 detections; API grasp (the cup's annotated rim points); API place() at the coaster centre | 0.1: the rim pinch overlapped the 2-3 mm wall by ~9 mm and the cup slipped out on the lift |
| p1b | tilted (45/60 deg) body grasp; API place() | 0.8: the cup pivots in the hand while place() rotates the hand, and is released 3-8 cm off; backing straight up knocks it over |
| p1c | own placement: translate, align the cup's bottom centre over the coaster, lower, release, back out along the approach | 0.7: placements within 1 cm, but far-apart layouts unreachable without turning the hand |
| p1d-g | carry with a hand-yaw search; table relay to the other arm; 2 mm drop margin; arm = the one closest to the farther of cup and coaster | **1.0 (s0 = 1.0)**; 1.0 on seeds 10-19 |

## Variations

| # | Variation | Axis | Assumption attacked | s_break | Result | Repairs |
|---|---|---|---|---|---|---|
| 1 | v1: cup scaled 1.5x (mug size, 7-8 cm) | size | A5: small cup | 0.0 | **accepted**, s_recovered 0.9 | 2 |
| 2 | v2: four movable boxes 1.5 cm around the cup | distractor | A2: free approach to the cup | 0.0 | **accepted**, s_recovered 0.8 | 4 |
| 3 | v3: cup slides 9 cm during the approach (slick cup and table) | object_motion | A4: static cup | 0.0 | **accepted**, s_recovered 0.9 | 3 |
| - | v4: cup lying on its side | orientation | A5: upright cup | 0.0 | **unsolved** (best 0.7) | 6 |
| - | v5: boxes around the coaster | distractor | A1 | - | rejected by validity (a box inside furniture on 1 seed of 10) | - |
| 4 | v6: four movable boxes 2 cm around the coaster | distractor | A1: free space at the coaster | 0.3 | **accepted**, s_recovered 1.0 | 6 |
| - | v7: cup at the back on one side, coaster at the front edge on the other | object_position | A10: reach | 0.5 | **unsolved** (0.3; variation flawed, see below) | 1 |
| 5 | v8: mug-size cup boxed in by four light boxes | distractor | A2 + A5 | 0.3 | **accepted**, s_recovered 0.8 | 1 |

Gate statistics:
- 8 variations gated: 7 passed and 1 was rejected by validity.
- None was rejected by the break gate, the variance gate or integrity. No parameter-tuning variation was designed.
- Of the 7 that passed, 5 were accepted and recovered, and 2 ended unsolved.

Before gating, 32 candidate scenes were screened with `eval.py --variation` (screening/results.txt, all in
logs/runs.jsonl), and 20 of them did not break the pipeline. Plain screening was for picking breaks, not for passing gates. From round 7 on,
every layout with pinned positions was first checked on all 10 seeds for validity and furniture overlap.

### Axes that broke the program
- object size (a mug-size cup);
- small movable objects packed around the cup, around the coaster, and around a mug-size cup;
- the cup moving during the approach;
- the cup lying on its side;
- a cup next to furniture at the back of the table.

### Axes that did not break it (screening, >= 0.6, against the pipeline of the time)
- **Size:** an espresso-size cup; a mug with a handle, also scaled 1.3x (after round 1).
- **Weight and friction:** a heavy cup (0.6 kg) or a wet heavy cup.
- **The coaster:**
  - slick and light, also on a glossy table;
  - pulled 12 cm away during the carry or later;
  - raised on a book;
  - at the back of the table, or at the front edge near the robot.
- **Things nearby:**
  - walls around the coaster;
  - an object lying on the coaster (the xy-only success check also counts a cup resting on it);
  - a divider, or a box sliding across the carry path;
  - two boxes next to each of cup and coaster;
  - a sliding cup with a boxed-in coaster.

## Unsolved variations: why

- **v4 (cup lying on its side), best 0.7 in 6 repairs.**
  - **Planning back from the final grasp was unreachable.** A stand-up grasp planned backwards from the upright final
    grasp gave no reachable candidate whenever the rim pointed away from the robot. Every downward final approach
    then maps to an approach from the far side, heading towards the robot.
  - **Flip and regrasp works, but costs about double the motion.** The flip (top-down grasp across the body, quarter
    turn about the closing axis in the reachable direction, set down standing, regrasp) works in isolation, but needs
    roughly twice the motion of a pick-and-place.
  - **Remaining failures:**
    - cups at the back or edge of the table, where no flip grasp is reachable;
    - episodes past the 120 s budget;
    - occasional cups flung by the physics during the turn.
  - **The axis has to come from the detected frame.** Box extents cannot tell a squat cup's axis from a diameter.
- **v7 (cross-table reach), 0.3 after 1 repair.**
  - **The variation is flawed.** The furniture layout changes with the seed, and on 5 of 10 seeds the pinned cup lies
    inside the low bookcase's footprint (4-5 cm overlap in x and y): in its lower compartment, under a fixed overhang.
    The padded planner cannot approach there, and the validity check does not detect it (ISSUES 8). 0.8 was therefore
    unreachable whatever the pipeline did.
  - **The one repair still found two real weaknesses:** lift away from an overhang, and cap refused re-approaches.

## Rules found (one per round, `reflections.jsonl`)
1. Choose the grasp from the object's measured size relative to the gripper opening, and keep a second grasp family that stays reachable when the first is not.
2. If the target is boxed in by small movable objects, move them to a free spot before grasping instead of searching more approach angles until the time runs out.
3. Re-measure the target right before the irreversible part of an approach and replan if it has moved; never reuse one estimate for a whole sequence of attempts.
4. (unsolved) Check an object's orientation before manipulating it, and if it must be turned, choose the turn direction and the grasp from reachability and budget the extra motion before committing.
5. Clear the spot where an object will be put down as well as where it is picked up, before picking it up while the hands are free; never start a corrective retry that cannot finish in the remaining time.
6. (unsolved) When placing objects at fixed coordinates in scenes whose furniture moves between instances, check every instance for containment under furniture; retry a refused approach only a few times, then change strategy.
7. Measure an object's size in its own frame, not from its world-axis bounding box, before using it to choose a grasp or a clearance, and sanity-check derived sizes against what the object can plausibly be.

## Notes for the team
- **Noise in a single check.** One `--all` varies by about ±0.2 per scene between runs (curobo nondeterminism, shifting
  noise sequence). With 5-6 scenes, the "all >= 0.8 in one check" rule passes partly by chance (ISSUES 9). Rounds 4 and
  5 needed 6 repairs each, partly for this reason.
- **The time budget.** Most failures on the harder scenes were the 120 s budget, not wrong motions (ISSUES 10).
- **A late bug fix.** The round-7 width bug had been present since round 1 and was invisible in open scenes. The
  final numbers include its fix.
