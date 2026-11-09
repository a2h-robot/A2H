# GAH training summary: put_mouse_on_pad (RoboPRO)

Goal: "Put the mouse on the mouse pad" (RoboPRO `put_mouse_on_pad`, office scene, clean config, aloha-agilex; three
mouse models, three furniture layouts). TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6. Success: the mouse's
model origin within 2 cm of the pad centre in x and in y (no height or orientation condition), both grippers open.

**Converged with 5 accepted variations** (v2, v3, v4, v5, v7; all `novel: true`), the MAX_ROUNDS stop rule, in six
rounds: one variation unsolved (v1), one rejected by the break gate (v6). This run replaced `put_rubikscube_in_drawer`,
whose Phase 1 did not reach a stable 0.8; see `../put_rubikscube_in_drawer/ATTEMPT.md` and ISSUES 11.

Final pipeline = round 6 (commit "round 6, axis=obstacle"). `eval.py --all`, 10 seeds each: **base 1.0, v2 1.0,
v3 0.8, v4 0.8, v5 0.9, v7 0.8.**

## How solid each acceptance is

One `--all` check varies by about ±0.2 per scene between runs with identical code (curobo is not deterministic, and
the noise sequence of the detections shifts with every extra call). In this run the same round-6 repair-3 pipeline
scored v7 0.4 and then 0.8 on re-measurement. Acceptance requires every scene >= 0.8 in one check, so with six
scenes it is partly luck. Read the acceptances accordingly:
- **Clear:** v2 (1.0 in every check after its repair), v3 (0.8-1.0 across eight checks).
- **Marginal:** v4 (0.8-0.9 across six checks), v5 (0.7-0.9 across four checks), v7 (accepted at exactly 0.8 after a
  0.4 / 0.8 pair on the code before the last repair).
- The base task stayed at 0.9-1.0 in every check of the run.

## Phase 1

| Eval | Pipeline | Result |
|---|---|---|
| p1a | 8-detection estimates; API grasp of the mouse (annotated grasp points); lift; carry with a hand-yaw search; align the box bottom over the pad; lower to 3 mm; release; back out along the approach | **1.0**, every placement within 8 mm; 1.0 again on the `--record-s0` run. **s0 = 1.0** |

## Variations

| # | Variation | Axis | Assumption attacked | s_break | Result | Repairs |
|---|---|---|---|---|---|---|
| - | v1: mouse boxed in by four light, taller objects 1.5-2 cm away | distractor | A2: free grasp from above | 0.0 | **unsolved** (best 0.7) | 6 |
| 1 | v2: pad on the far side of the desk (0.68 m from the mouse) | object_position | A1: the mouse-side arm reaches the pad | 0.0 | **accepted**, s_recovered 1.0 | 1 |
| 2 | v3: mouse upside down | orientation | A6: mouse right side up | 0.0 | **accepted**, s_recovered 0.9 | 1 |
| 3 | v4: a mug-sized object standing on the pad | distractor | A5: the space above the pad is free | 0.0 | **accepted**, s_recovered 0.9 | 2 |
| 4 | v5: pad at the back, under the wall shelf's board | object_position | A10: the pad is reachable from straight above | 0.0 | **accepted**, s_recovered 0.8 | 2 |
| - | v6: a 1.3x mouse | size | A11: the released mouse stays | 0.7 | rejected by the break gate (after a backend fix, see below) | - |
| 5 | v7: mouse half under a monitor riser (a board 8 cm up) | obstacle | A2: the space above the mouse is free | 0.0 | **accepted**, s_recovered 0.8 | 5 |

Gate statistics: 7 variations gated; 6 passed, 1 rejected by the break gate (v6). None was rejected by validity, the
parameter-tuning streak or integrity. Of the 6 that passed, 5 were accepted and recovered, 1 ended unsolved.

Before gating rounds 5 and 6, ten candidates were screened with `eval.py --variation` against the pipeline of the time
(`screening/results.txt`, all in `../../logs/runs.jsonl`).

### Axes that broke the program
- clutter around the object (four light objects in a ring);
- the goal on the other arm's side of the robot;
- the object upside down (its stored grasp points face the table);
- an object standing on the goal;
- an overhang above the goal (the hand, not the object, collides);
- an overhang above the object.

### Axes that did not break it (screening, against the pipeline of the time)
- tall walls between the mouse and the pad (1.0): the planner keeps the hand clear and the mouse rides at fingertip
  level;
- the pad at the desk's front edge (0.9);
- a slippery mouse, pad and desk (0.9); a heavy mouse, 0.45 kg (1.0);
- the mouse sliding 9 cm during the approach, early or late (0.8, 1.0): the grasp re-estimates after a miss;
- a 1.3x mouse, once the backend bug below was fixed (0.7 at the gate).
- A 1.5x mouse (0.4) and a notebook lying on the mouse (0.0) did break it, but were not designed into rounds: the
  large model is about as wide as the gripper opening, and a notebook wider than the opening can only be pushed off.

## Unsolved variation: why

- **v1 (mouse boxed in by four light objects), best 0.7 in 6 repairs** (v1 0.0, 0.3, 0.7, 0.6, 0.5, 0.4; base 0.9-1.0).
  - Clearing four objects one by one costs 50-80 s of the 120 s budget, and the runs that failed ran out of time.
  - The object along the mouse's long side stands in a 1.5 cm gap, narrower than a finger, so a grasp of it slips,
    and sliding it lengthwise past the mouse did not work reliably either.
  - The best approach was repair 3 (clear every side; 0.7). Repairs 4-6 added retries and a slide, and each cost time
    the budget did not have, so the trace is not a steady decline: the approach peaked at repair 3, and the later
    changes traded time for reach.

## Rules found (one per round, `reflections.jsonl`)
1. (unsolved) When the object to grasp is hemmed in, first work out which neighbours actually block the grasp you will use and how long clearing each one costs, and move only those, cheapest first, keeping enough of the time budget for the task itself; a gap narrower than a finger cannot be used for a grasp.
2. Choose the arm for each reach from where that target is, not once for the whole task; when no single arm reaches both the start and the goal, hand the object over through a spot both arms reach.
3. Do not depend on one source of grasp points: when an object's pose makes its stored grasps unusable (they point into its support), fall back to a grasp computed from its measured shape in the pose it is actually in.
4. Before carrying something to where it must go, check that place for anything standing on it and clear it while the hands are still free; and start every approach from above with the fingertips clear of the object's top, not at a fixed distance above the grasp point.
5. Check the space the hand and wrist need above a goal, not only the goal itself; when something overhangs it, tilt the hand so the wrist stays clear, and press a tilted object flat onto the surface before letting go so that it does not shift as it settles.
6. When part of an object is under something, do not reach down at it: come in level from the open side, low enough to pass under the overhang, take it by its middle and slide it out into the open before lifting.

## Notes for the team
- **A backend fix during round 6** (ISSUES 15, commit "robopro backend fix"). A rescaled or swapped task object was
  rebuilt from the previous episode's model id on the office tasks, because a worker keeps its task object across
  episodes. Only `object_scale` and `swap_object` were affected, and only v6 uses them. Its first gate (s_break 0.3,
  kept in `superseded/`) measured "a 1.3x mouse, mostly model 2"; re-gated after the fix it was rejected at 0.7.
  `tools/INTEGRITY.json` was regenerated for `scene_edits.py`; the gates of v1-v5 ran against the previous hashes, and
  the v3 break measurement was repeated under the fixed backend (see `session_log.md`).
- **One worker per GPU.** A worker needs about 10 GB of GPU memory on a 16 GB card; on cards shared with other
  processes the render ran out of memory and hung the worker (ISSUES 13-14 describe the pool guard and the hang
  watchdog this led to).
- **The time budget** dominated the cluttered variation (v1), as in the put_cup_on_coaster run.
