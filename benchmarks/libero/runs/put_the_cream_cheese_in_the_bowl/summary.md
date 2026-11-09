# put_the_cream_cheese_in_the_bowl — GAH training summary (LIBERO-Goal, sim)

Settings: TAU=0.8, SEEDS=10, MAX_ROUNDS=5, MAX_REPAIR=6. Branch `a-libero-put_the_cream_cheese_in_the_bowl`.
Wall clock: about 1 h 45 min (10:31-12:15 EDT, 2026-10-23), all simulation on <sim-host> with `--workers 7`.
Token usage: roughly 0.2 M tokens of context consumed by this session (from the session's remaining-token counter; not an exact billing figure).

## Result

- **s0 = 1.0** (Phase 1 pipeline: averaged detections, top grasp across the narrow side with a depth search, lateral shift/tilt candidates, drop 2.5 cm above the bowl rim).
- 6 variations designed, all passed the gate; **5 accepted and recovered, 1 unsolved** (v4). Stopped at 5 accepted variations.
- Final `eval.py --all` (pipeline at the last commit): base 1.0, v1 0.9, v2 1.0, v3 1.0, v5 0.9, v6 1.0.

| Round | Variation | Axis | Assumption attacked | s_break | s_recovered | Status |
|---|---|---|---|---|---|---|
| 1 | v1 | object_position | cheese 7 mm from the bowl wall: no room for the hand | 0.0 | 1.0 | accepted |
| 2 | v2 | object_motion | bowl slides 10 cm during the carry | 0.2 | 1.0 | accepted |
| 3 | v3 | object_position | cheese at the edge of reach (0.83 m from base) | 0.0 | 0.9 | accepted |
| 4 | v4 | distractor | 5 cm block already inside the bowl | 0.0 | 0.4 (best) | **unsolved** (6 repairs, reverted) |
| 5 | v5 | object_motion | cheese slides at 1.2 cm/s for 14 s | 0.4 | 0.9 | accepted |
| 6 | v6 | obstacle | tall post beside the bowl in line with the hand's wide axis | 0.0 | 1.0 | accepted |

## Axes that broke the program

- **Clearance for the hand, not the fingers** (v1, v6): the Panda hand is ~20 cm wide along the finger-closing axis and only ~4 cm above the grip point, so a 5 cm-tall neighbour within ~10 cm along that axis blocks top grasps (v1) and top-down releases (v6). Fixed by relocating the movable blocker (v1) and by searching hand yaw at the release (v6).
- **Kinematic reach** (v3): no vertical-wrist IK at table height near the workspace edge. Fixed with approach directions leaning away from the base.
- **Motion** (v2 target moves, v5 object moves): single (averaged) snapshots go stale. Fixed by re-detecting the target before every irreversible step and by estimating object velocity from a 2 s track and waiting until it stops.
- **Occupied container** (v4): not recovered. Clearing it needs a partial gripper opening the API does not offer (open_gripper always opens to 8 cm and the open fingers hit the bowl rim around a 5 cm block); releasing off-centre with a tilted hand reached only 0.3-0.4.

## Axes screened that did NOT break the program (not gated; run with eval.py on scratch variations)

- dynamics: cheese mass 0.4-2.0 kg with friction 0.1-0.25 (0.8-1.0);
- orientation: cheese rolled 90 deg on its long edge + yawed, or pitched 90 deg on its end (1.0);
- appearance / size: cheese swapped for alphabet_soup, cookies, tomato_sauce (1.0); cheese scale 0.5-1.5 (0.9-1.0); bowl scale 0.5-0.7 (0.9-1.0);
- lighting / color / camera were not tried: perception in this backend is pose-based, so they cannot break it.
- A shelf 28-40 cm above the table over the bowl broke the program (0.0-0.2) but was dropped before gating: it also blocks the forearm when reaching the cheese and looked unsolvable with the API's planner, so it was not a fair variation.

## Rules found (general_rule of each round)

1. When no collision-free grasp exists because a movable neighbour is in the way of the hand (not just the fingers), move the neighbour away first instead of only varying the grasp pose; estimate clearance with the full hand geometry along the closing axis, not the finger opening.
2. Re-perceive the goal location immediately before every irreversible step (release, insertion, contact) and re-target if it moved.
3. When every motion to a target is refused, distinguish unreachable (kinematics) from blocked (collision); answer reach failures with an approach direction leaning away from the base; make retry motions absolute so repeated attempts cannot drift out of the workspace.
4. (unsolved round) Before releasing into a container, perceive what is already inside and plan the release into the actual free volume or clear it first; check that the gripper can physically fit the clearing grasp before relying on it.
5. Estimate an object's velocity from a short time series of detections before acting on it, and wait or predict; averaging repeated detections removes noise but hides motion.
6. Before any irreversible action, check that the preceding approach motion succeeded; when an approach is refused, search the end-effector's free degrees of freedom (e.g. yaw about the approach axis) instead of acting from wherever the arm stopped.

## Notes on the process

- v1 took the most repair work (7 pipeline revisions; it first reached TAU at the 2nd, an end-push version, but that was fragile and re-ran at 0.7, so I kept going): pushing along the long axis and fingertip dragging were both unreliable (friction balance makes top-contact dragging slip; long-axis pushes hit the plate or the stove burner). Relocating the bowl was the robust fix.
- The release step had silently ignored refused descents since Phase 1 (found in v3 and v6).
- Files: `pipeline.py` (final), `assumptions.md`, `variations/v1..v6.yaml` (+ `.gate.json`), `gate_history.jsonl`, `reflections.jsonl`.
