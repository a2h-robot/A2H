# GAH training summary: pick_boxdrink_from_basket (RoboPRO, kitchen scene)

Goal: "pick up the box drink from the basket". RoboPRO's success check requires the box to end standing on
the table, outside the basket. TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6. Robot: aloha-agilex
(two 6-DoF arms) with the cuRobo planner. Perception has 1 cm / 3 deg noise.

The seeds mix two layouts: the basket on the table, and the basket on top of a microwave. In the second the
box must be lifted 13 cm over the rim, near the top of the arm's reach.

**Stopped after 5 rounds (MAX_ROUNDS = 5): 4 accepted variations (v1, v2, v4, v5) and 1 unsolved (v3); all 5
reflections novel.** The convergence rule (5 accepted, or 2 consecutive non-novel rounds) was not reached. One
known breaker remains: a 30 % larger carton (0.2 against the round-4 pipeline in screening). It needs a grasp
across the carton's narrow side, from a side approach, and was not gated.

Final pipeline (round 5) on `eval --all`, 10 seeds each:

| base | v1 | v2 | v4 | v5 |
|---|---|---|---|---|
| 0.8 | 1.0 | 0.9 | 0.8 | 0.9 |

v3 is unsolved and excluded. Earlier evaluations of the same scenes ranged 0.8-1.0 (cuRobo is not deterministic).

## Phase 1

s0 = 1.0 (seeds 0-9), and 20/20 on seeds 10-29. Five evaluated iterations (0.1 -> 0.3 -> 0.5 -> 0.5 -> 1.0).
- Grasp the box near its centre of mass, from the robot's side, with shallow approaches first. A grasp near
  the top let the box swing, and steep approaches leave too little reach for the lift.
- Pre-grasp 9 cm back, so that 1 cm of detection error does not put the fingertips into the box.
- Lift until the box clears the rim. The rim is the basket's world-z extent; its box extents are in its own
  rotated frame.
- Place upright with a level gripper only. The API's tilted-release fallback topples a 22 cm box.

## Variations

| # | Variation | Axis | Assumption attacked | s_break | Oracle | Result | Repairs |
|---|---|---|---|---|---|---|---|
| 1 | v1: eight movable boxes clutter the front-left table area | distractor | A9: free table space in a fixed region | 0.0 | 0.0 | **accepted**, s_recovered 1.0 | 3 |
| 2 | v2: the box leans ~34 deg against the far basket wall | orientation | A1: the box stands upright | 0.1 | 0.3 | **accepted**, s_recovered 0.9 | 1 |
| - | v3: a fixed 36 cm item (towel holder) stands in front of the basket | obstacle | A8: the path towards the robot is free | 0.2 | 0.0 | **unsolved** (best 0.7 at repair 2; pipeline reverted to round 2) | 6 |
| 3 | v4: a tray slides onto the placement area during the carry | object_motion | A12: nothing moves on its own | 0.4 | 0.6 | **accepted**, s_recovered 1.0 | 1 |
| 4 | v5: a full carton, 1.2 kg and friction 0.3 | dynamics | A5/A11: a light box; the arm reaches commanded heights | 0.2 | 0.5 | **accepted**, s_recovered 0.9 | 2 |

**Gate statistics.**
- Gate decisions: one integrity rejection and five passes.
- The rejection came from a stale hash file after a backend change: a tooling error, not a design.
- No variation was rejected by the break gate or the variance gate. No parameter-tuning variation was
  designed.

**Screening.** Candidate edits were screened against the frozen pipeline (5 seeds) before gating; see
`screening/`.
- 9 did not break it: box yawed 45 deg; 0.8 kg carton; wet 0.6 kg carton; wet 1 kg carton; 1 kg carton with
  friction 0.3; 15 % larger carton; nudge during the approach; basket 6 cm farther; 12 cm item in front of
  the box.
- 5 were invalid: box against the near wall; a full-height carton in front of it; items on both sides; box
  lying down; a 16 cm item in front. The bowl-shaped basket makes them interpenetrate and eject the box at
  setup.
- 1 broke it but was not gated: a 30 % larger carton (0.2 against the round-4 pipeline).

### Axes that broke the program
- clutter in the placement area (v1);
- a leaning object (v2);
- a fixed obstacle between the robot and the container (v3, unsolved);
- an object moving into the destination (v4);
- a heavy object (v5).

### Axes that did not break it (screening, >= 0.6)
- object yaw;
- moderate weight or friction changes;
- a 15 % larger object;
- small nudges;
- container position;
- appearance (perception is pose-based).

## Rules found (reflections.jsonl)

1. Treat a held object as part of the robot: route it yourself through space checked for its full volume;
   pick destinations by measured clearance over the whole reachable surface; release only at resting
   height. (round 1)
2. Measure an object's actual orientation, averaging repeated detections; grasp relative to its real axes;
   correct its orientation in the hand before placing. (round 2)
3. Check that the whole sequence a grasp implies (grasp, lift, exit) stays inside comfortable reach; when
   obstacles remove those approaches, change the plan instead of stacking fallbacks at the reach limit.
   (unsolved round)
4. Re-observe the target area, and let moving things come to rest, immediately before an irreversible
   action. (round 3, v4)
5. Judge every motion by its measured effect on the object, not by what was commanded; bound corrective loops
   so they cannot chase a slipping object. (round 4, v5)

## Infrastructure notes
- The run used one GB10 node at a time, as a low-priority guest beside other users' jobs.
- It ran on four nodes in one evening, moving each time another user's job arrived.
- Once, a job arriving mid-evaluation stopped the pool, so 7 queued episodes never ran and that repair's
  evaluation was re-run in full. The repeated v1 gate was caused by the stale hash file, not by compute.
- With 2 workers, a full `--all` evaluation (40-50 episodes) took 15-30 min.
