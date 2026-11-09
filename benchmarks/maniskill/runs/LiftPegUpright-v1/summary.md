# GAH training summary: LiftPegUpright-v1 (ManiSkill3)

Goal: "lift the peg lying on the table to an upright pose". TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6.

**Status: finished. Stopping rule: 5 accepted variations** (v1, v2, v4, v5, v6). v3 passed the gate but could not be recovered and is marked `unsolved`. Every round gave `novel: true`, so the other stopping rule (two `novel: false` rounds in a row) was never reached.

The run was interrupted after round 4. The GPU machine jinbang-4080 went offline from 2026-10-24 13:50 until about 11:40 on 10-25, then again from about 12:00 to 12:56. Rounds 5 and 6 ran on 10-25.

## Phase 1
- The pipeline averages 60 noisy detections and grasps the peg from the top, firmly. It lifts the peg, then measures where the peg sits in the hand. From that it computes the hand pose that stands the peg upright just above the table, re-measures after the swing, lowers the peg and releases it.
- **s0 = 1.0** (10 seeds). TAU was never lowered.
- Two details were specific to this task:
  - The success check decomposes the pose into XYZ Euler angles, so it needs one width axis of the peg along world x (otherwise gimbal lock).
  - A fast swing flings the peg out of the fingers, so the swing runs at slow speed.

## Variations

| Round | Variation | Axis | Assumption attacked | s0 | s_break | Oracle | Result |
|---|---|---|---|---|---|---|---|
| 1 | v1: low wall between the robot and the peg | obstacle | the space around the pick spot is free, so the peg can be released where it was picked | 1.0 | 0.4 | 1.0 | recovered, 1.0 (2 repairs) |
| 2 | v2: block 1 cm beside the peg at the grasp point | distractor | the open fingers fit at the single fixed grasp point | 1.0 | 0.0 | 0.4 | recovered, 1.0 (1 repair) |
| 3 | v3: shelf 0.39 m above the work area | obstacle (overhead) | unlimited headroom; "higher is safer"; a plan the planner accepts always arrives | 1.0 | 0.0 | 0.0 | **unsolved**: 0.1 → 0.6 after 6 repairs; reverted |
| 4 | v4: box slides into the release area after the pick | object_motion | obstacles are static, so one look per attempt is enough | 0.7* | 0.1 | 1.0 | recovered, 1.0 (4 repairs) |
| 5 | v5: peg 1 cm from a 6 cm wall along its whole length | obstacle | the peg can be grasped where it lies | 1.0 | 0.0 | 0.0 | recovered, 0.9 (4 repairs) |
| 6 | v6: peg on a 4 cm platform | object position (support) | the peg rests on the table (finger check swept down to z = 0) | 0.9 | 0.0 | 0.0 | recovered, 1.0 (1 repair)** |

\* The gate re-measured the post-round-3 pipeline on the base task at 0.7; a 20-seed run gave 0.85. The drop came from the round-3 retreat change, which round 4 fixed.

\*\* The gate's automatic check labelled v6 as parameter tuning, because it uses the same edit keys as v5. It attacks a different assumption, and the tuning streak before it was 0.

**Final pipeline** (`eval.py --all`, 10 seeds each): base 1.0, v1 0.9, v2 1.0, v4 1.0, v5 1.0, v6 1.0. Extra checks on seeds 10-19: v1 0.8, v4 1.0, v5 0.8-0.9, v6 0.9.

Results vary between identical runs by about ±0.1-0.2 (v4 once gave 0.7 and then 1.0 / 1.0). The failing seeds pass when re-run one at a time. The remaining failures come from the planner's random RRT fallback, which sometimes takes a very long detour and runs out the 2000-step budget.

### Axes that did not break the program
These were screened before choosing a variation, using the frozen pipeline of that round:
- **object position:**
  - peg shifted forward and sideways ([+0.10, +0.12] → 1.0; [0, +0.26] → 0.9);
  - 0.20 m closer to the robot → 1.0; [-0.25, +0.12] → 0.7;
  - far out on the diagonal [+0.22, -0.15] → 0.1, not used: partly beyond the 0.85 m reach check.
- **object motion:**
  - peg sliding constantly (nominal 0.03, 0.2 or 0.1 m/s) → 1.0, 1.0, 0.9;
  - peg nudged sideways just before the grasp (impulse 0.6 m/s) → 0.8.

  The retry loop and the grasp across the peg tolerate both.
- **dynamics:**
  - mass up to 3.5 kg with mu 0.3-0.5 → 0.6-1.0;
  - heavy and slippery within a real arm's payload (3 kg with mu 0.1, 2.5 kg with mu 0.2, 1.5 kg with mu 0.08) → 1.0; 2 kg with mu 0.1 → 0.7;
  - 4.5-6 kg → 0.0-0.3: the peg pivots in the fingers and drops. Not used, because it exceeds a real arm's 3 kg payload.
- **orientation:**
  - yaw 45° or 90° → 1.0;
  - rolled onto another face → 0.7 (round 2), 0.9 (round 5);
  - leaning on a block → 0.8.
- **color, lighting, camera:** not tried as variations, because perception in this backend is pose-based (`camera_frame` is unused).

These broke the round-5 pipeline but were not chosen: a board lying on top of the peg (0.0), the peg in a corner of two walls (0.0, close to v5), and the peg in a tray with 4 cm rims (0.0, probably not graspable at all).

## Rules found (`general_rule`, all `novel: true`)
1. Plan every motion against the obstacles you perceive, including the place/release pose and the retreat. Treat the place location as a choice you check for free space for the whole arm; do not assume the pick location is free.
2. Every collision check you add needs alternatives to try when it rejects the default (grasp points, approach directions, place spots). Choose the grasp by checking that the gripper's own volume (open fingers) fits beside the object's neighbours.
3. (Unsolved round) Derive clearance from the actual free space in every direction, overhead included, not from "higher is safer". Never trust a planner's success for things it does not model: check that each motion arrived, and account yourself for held and just-placed objects as obstacles.
4. Perceive again just before every irreversible or contact-making step (grasp, release, insertion) and compare with what you planned on. If something moved, wait for it to settle and replan. Keep held objects well clear of surfaces until that final check.
5. When no grasp fits the object where it lies, do not give up. Change its situation with a non-prehensile action (push or slide it through free space to where a grasp fits). Predict that action from the perceived free space, and look again afterwards to confirm it worked.
6. Express every geometric check relative to what you perceive (the object's own extent and the surface it actually rests on). Never use a hard-coded world height or reference surface: a scene that differs only in that reference will make the check reject everything.

Cross-cutting lessons from the repairs:
- Average noisy perception, obstacles included.
- Re-measure a held object after any large motion, because it pivots in the fingers.
- The planner's random (RRT) fallback can turn a short move into a long detour through things it does not know about. Keep moves near objects short and straight, and stop rather than risk a detour.
- When pushing, plan without the pushed object, since the planner would otherwise avoid it. Allow for the gap before contact, and judge success by where the object ended up, not by what the planner returned.

## Files
- `pipeline.py`: final pipeline.
- `assumptions.md`: assumption list, screening results and attack order for each round.
- `variations/v1-v6.yaml` and their `.gate.json`; `gate_history.jsonl`; `reflections.jsonl`; `s0.json`.
