# GAH training run: push_the_plate_to_the_front_of_the_stove (libero)

Goal: "push the plate to the front of the stove". TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6.

- **Wall-clock:** about 2 h 05 min (10:30 to 12:35 local time, 2026-10-23), including about 45 min of Phase-1 probing.
- **Tokens:** not measured exactly. Roughly 0.35 M context tokens consumed by the agent.
- **Stop reason:** I stopped by choice to stay within budget. Neither formal stop condition was met: there were 3 accepted variations (not 5), and the novel=false rounds were not two in a row (round 4 was novel=false, rounds 5 and 6 were unsolved with novel=true). Twelve candidate variations were rejected by the gate, most of them because the closed-loop pipeline still passed them.

## Phase 1: s0 = 1.0 (10 seeds)

Plain pushing does not work on this plate, which I found by probing:
- it weighs about 11 g and its rim is a shallow cone of 8 flat segments;
- closed fingers pushing from inside the dish ride over the rim;
- fingers pushing from outside wedge under the lip and lift the plate;
- the side that would have to be pushed faces the cabinet, so there is no room for the hand.

The final strategy is a "pinch and carry":
- average 30 noisy detections (api.grasp() uses a single detection with about 1 cm of noise);
- pinch the middle of one rim segment, never a junction between two segments, and pick a segment whose tangent runs along the carry direction (the pinch acts as a hinge);
- lift that edge 6 cm and carry it with the planner;
- lower it as far as the planner allows and release;
- predict where the hanging plate will land (it falls flat about its lowest edge), then re-detect and repeat until the plate is inside the goal region with a margin.

The target point inside the region is chosen to keep clear of nearby objects.

## Rounds

| round | variation | axis | assumption attacked | s_break | s_recovered | outcome |
|---|---|---|---|---|---|---|
| 1 | v1 plate mass 0.4 kg | dynamics | the plate is light enough to lift by its lip (F1) | 0.0 | 0.5 (best of 6 repairs) | **unsolved**, reverted |
| 2 | v2 cup (5 cm x 10 cm, 0.2 kg) in the middle of the goal region | distractor | part of the goal region is free (A3/D2) | 0.0 | 0.9 | accepted |
| 3 | v3 10 cm x 10 cm box (0.3 kg) in the goal region | distractor | a goal blocker fits in the gripper (round-2 fix) | 0.0 | 0.9 | accepted |
| 4 | v4 four cans 3 cm from the rim on every reachable side | distractor | the space around the rim and the carry path is free (E1/A2) | 0.4 | 0.9 | accepted (novel=false) |
| 5 | v5 plate scale 0.7 | size | the pinch radius and depth are constants (B3) | 0.0 | 0.5 (best of 6 repairs) | **unsolved**, reverted |
| 6 | v6 fixed board 7.5 cm above the far half of the plate | obstacle | free space above the rim for the whole hand (E1) | 0.0 | 0.0 | **unsolved** (no collision-free pinch pose exists; probably not physically solvable with this gripper, my design mistake) |

Final pipeline (HEAD, the round-4 version) on `eval.py --all`: base 0.9, v2 1.0, v3 0.8, v4 0.9.

### Rejected candidates (the frozen pipeline was not broken)

| candidate | edit | s_break (gate) |
|---|---|---|
| rejected1_size | plate scale 1.3 | 0.9 |
| rejected2_dynamics | plate mass 0.15 kg | 0.8 |
| rejected3_swap_invalid | swap plate with akita_black_bowl | invalid scene (name clash with the existing bowl) |
| rejected4_swap | swap plate with white_bowl | 1.0 |
| rejected5_obstacle | 12 cm wall in the carry path | 0.8 |
| rejected6_distractor | 4.5 cm block in the goal (shoved aside by the plate) | 0.6 |
| rejected7_motion | plate slides 12 cm during the first approach | 0.9 |
| rejected8_clutter3 | three cans around the rim | 0.6 |
| rejected9_friction | plate friction 0.3 | 0.9 |
| rejected10_on_stove | plate starts on top of the stove | 1.0 |
| rejected11_motion_into_goal | a can slides into the goal at t = 6 s | 0.6 |
| rejected12_motion_into_goal_early | a can slides into the goal at t = 1 s | 0.6 |

## Which axes broke the program and which did not

- **Broke it:**
  - realistic mass;
  - smaller object size;
  - a blocked goal (graspable blocker, then an ungraspable one);
  - clutter around the grasp points;
  - an overhead obstacle.

  All of these attack the single grasp primitive (a top-down lip pinch) or the assumption that the goal and the area around the object are empty.
- **Did not break it:**
  - larger size;
  - moderate mass (0.15 kg);
  - low friction;
  - a different dish (white bowl);
  - a different start support (the stove top);
  - a wall in the carry path;
  - the plate moving during the approach;
  - objects sliding into the goal during the run;
  - small blockers that the plate simply shoves aside.

  The closed loop (re-detect, retry, predict the landing) absorbs one-off disturbances and position changes. Colour, lighting and camera edits were not tried: perception in this backend is pose-based, so they cannot break it.
- **Unsolved:** three of six gated rounds. Each one hit the physical limits of the only grasp primitive:
  - too heavy for a lip pinch;
  - a lip too low for the fingers' clearance above the table;
  - no vertical approach room.

  A pipeline with a second, non-prehensile or angled contact primitive would be needed.

## Rules found (general_rule of each round)

1. Before committing to a grasp-and-lift strategy for an object, check that it still works when the object's mass and friction are scaled to real-world values. Keep a non-prehensile or support-sharing fallback whose required force does not depend on grip friction. (round 1, unsolved)
2. Before placing an object at a goal, check that the goal area is actually free. If something movable occupies it, first move the blocker to a spot that is clear of the whole goal area and of the planned path. (round 2)
3. When a skill has to handle an arbitrary object (remove, grasp, move), check the object's size against the gripper's capacity and keep a non-grasping alternative. Confirm that a carried object is really attached by checking that it moved with the hand. (round 3)
4. Before manipulating an object, clear small movable clutter from both the volume the gripper needs around the grasp point and the volume the object will sweep while carried. Do not rely on trying other grasp angles. (round 4, novel=false: it extends rule 2)
5. Derive grasp points and depths from the object's measured size rather than from constants. Before relying on a grasp, check that the part to be gripped is taller than the gripper's minimum clearance above the support surface. Test size changes in both directions. (round 5, unsolved)
6. When choosing a grasp, check that the whole hand, not just the finger tips, fits in the free space above the grasp point. Keep at least one grasp or contact primitive whose approach direction is not vertical. (round 6, unsolved)

Lessons from Phase 1 that are not tied to a round:
- Average repeated noisy detections before any precise approach, and do not let a helper (api.grasp()) silently use a single noisy detection.
- Pinch flat faces, not corners or junctions.
- A pinch on a thin edge acts as a hinge, so carry the object along the hinge axis.
- The "holding" flag can lie. Verify by checking that the object's height and position follow the hand.

## Backend notes found during the run

- `swap_object` with a category that is already in the scene (`akita_black_bowl` while `akita_black_bowl_1` exists) fails with `KeyError: 'akita_black_bowl_1'` in every validity seed.
- `gripper_holding()` flickers while pinching the thin plate lip: it reports False while the plate is clearly held.
- The planner treats a carried object as rigidly attached. It refuses any motion in which a dragged object touches the table, so sliding an object while holding it is impossible, and a dangling object's real pose differs from the planner's model.
- Results are not fully deterministic for the same seed: the same pipeline gave s0 = 1.0 in eval and 0.9 in later gate runs.
