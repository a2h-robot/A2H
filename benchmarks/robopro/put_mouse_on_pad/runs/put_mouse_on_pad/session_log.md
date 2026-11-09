# Session log: GAH training on put_mouse_on_pad (RoboPRO, office scene)

This is a curated record of the agent session (Claude Opus 5.5 in Claude Code, 2026-10-26). It covers what was built,
what was run, what failed and why, and the decisions taken.
- The machine-readable records are in the repo: commits per round, `variations/*.yaml` with their `.gate.json`,
  `reflections.jsonl`, `screening/results.txt`, `superseded/` and `../../logs/runs.jsonl`.
- Times are Toronto time (EDT). Unless noted otherwise, every success rate is over 10 bank seeds.

Compute: x86 Slurm nodes with two RTX 5080 GPUs (16 GB) each, one pool worker per GPU, the pool's job queue on a
shared file system. Every number of record (both tasks' Phase 1 evaluations, all gates, all repairs) is on that one
hardware type. An aarch64 GB10 box was used only for scene probes and two developer traces, never for a scored number.

## 1. Assignment (15:15)

- **Task:** RoboPRO `put_rubikscube_in_drawer` (office scene), as a further RoboPRO run: the only RoboPRO task that
  puts an object inside an articulated container, in a scene no earlier run had used.
- **Protocol:** TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5 accepted variations, MAX_REPAIR = 6. Branch per task, PR into
  `d-robopro`, unmerged.
- **Escape hatch, agreed up front:** if the base task could not reach 0.8 in about 45 minutes, switch to another task
  in the same scene (`put_mouse_on_pad` or `put_phone_on_holder`).

## 2. put_rubikscube_in_drawer: Phase 1 attempt (15:20-17:05), abandoned

Full record: `../put_rubikscube_in_drawer/ATTEMPT.md`; ISSUES 11.
- **The drawer starts open** (RoboPRO opens the bottom drawer to its 0.178 m limit at reset), so no articulation is
  needed; RoboPRO's expert solved 4/4 probe seeds.
- **Hidden requirement.** `check_success` measures the cube's model origin, which lies on the cube's back face, not at
  its centre, against the drawer floor height with 2 cm tolerance. An upright cube fails by 0.7-1.2 cm: the cube must
  go in turned 90 deg, back face down. The expert does exactly that (model-y axis from -y to +z on all probe seeds).
- **Backend additions for the office scene** (before any hash was written; ISSUES 12): the wall shelf is one
  non-convex triangle mesh, so the planner boxes are now one per connected part instead of one solid box that filled
  the shelf; the chest of drawers' links are registered (`drawer`, `cabinet`) and planned per convex piece; a
  `drawer_inside` region; a 1 s settle at reset.
- **Platform.** The run moved to the x86 nodes for speed. The first RoboPRO tree there was an older version (30 of 94
  shared task files differ, including every scene's base class); the run used a copy of the GB10 code tree instead,
  with matching `check_success` hashes on both machines.
- **Seven pipeline versions, no stable 0.8:** 0.0, 0.1, then 0.8 / 0.6 / 0.6 on the same seeds with identical code,
  0.4, 0.4, 0.6, 0.5. Causes: a downward hand is reachable only low and close to the robot while the cube must clear
  the drawer's 0.884 m front panel; the two-finger pinch is nearly a free pivot about the axis the cube is turned
  about (intermittently; a true-state trace showed the cube turning with the hand on one machine); a cube released
  9 cm up tumbles. A grasp offset chosen so gravity would hold the right face down (the expert's 3.5 cm) gave 0.5, so
  that mechanism is **not** demonstrated: the pivot question is open.
- **17:05: switched to `put_mouse_on_pad`**, keeping the office-scene backend.

## 3. Pool reliability (16:20-17:30)

Two infrastructure failures, both fixed before any number of the mouse run (ISSUES 13-14):
- **A worker that fails every job instantly.** On one node a worker's right-arm planner ran out of GPU memory during
  warm-up (another user's processes held about 8.5 GB of each card); the half-built task object stayed cached and every
  later job failed in 0.06 s, taking 80 of 90 seed checks of one scan. `pool.py` now requeues a job that fails in under
  0.8 s and the worker exits after 3 in a row (this guard went into the tools before INTEGRITY.json was written). That
  node was left out.
- **A worker that hangs.** On cards shared with other processes, the end-of-episode render ran out of GPU memory and
  the worker never returned. One worker needs about 10 GB on a 16 GB card. The run's Slurm launcher (outside the
  hashed tools) watches the workers' heartbeats: a worker silent for 3 minutes is killed, its job requeued, and the
  card gets a fresh worker at most twice, and only while the card has >= 11 GB free. Training evaluations use
  `--no-frames`.
- A driver launched with `srun --overlap` inside the pool's allocation without `-n1` started four copies of the same
  evaluation (section 6).

## 4. Phase 1 (17:05-17:20)

- Seed bank on the x86 nodes: 60 of 75 scanned seeds (2 unstable, 11 expert failures), `seeds/put_mouse_on_pad.json`.
- Success check (RoboPRO's, unchanged and hashed): mouse origin within 2 cm of the pad centre in x and in y, both
  grippers open. The mouse origin is straight below its box centre, so aligning the box centre suffices.
- Pipeline: 8-detection estimates; the API grasp from above (the mouse's annotated grasp points); lift 6 cm; carry
  with a hand-yaw search; align the box bottom over the pad centre; lower to 3 mm; release; back out along the
  approach. **1.0** (every placement within 8 mm), and 1.0 again on the `--record-s0` run. **s0 = 1.0**, commit
  "phase 1".

## 5. Round 1: v1, the mouse boxed in by four light objects (17:25-18:05), UNSOLVED

Assumption A2 (nothing next to the mouse). Four movable boxes (0.15 kg, 6 cm tall) 1.5-2 cm from the mouse's sides
and ends. Gate: s0 1.0, **s_break 0.0** (every grasp approach refused).

| Repair | Change | base / v1 |
|---|---|---|
| 1 | Move every small object within 5 cm of the mouse to a free table spot first | 1.0 / 0.0. The object grasps put the fingertips into the table (grasp point 3 cm below a 6 cm top, fingertips 3.6 cm below it) |
| 2 | Fingertips kept above the table; clear only the objects on the closing-axis sides | 0.9 / 0.3. The end objects block the API's mouse grasp too |
| 3 | Clear all sides again, no second-arm retry | 1.0 / **0.7** |
| 4 | A second try for an object that slips off on the lift; clear again before a retry | 1.0 / 0.6, now over the 120 s budget |
| 5 | Drop the objects from 4 cm instead of lowering them; no move home in between | 1.0 / 0.5 |
| 6 | Slide an object that cannot be lifted lengthwise past the mouse | 1.0 / 0.4 |

The best approach was repair 3; repairs 4-6 traded time for reach and the budget did not have it. A finger cannot use
the 1.5 cm gap beside the mouse's long side. Marked unsolved, phase-1 pipeline kept, last attempt in `unsolved/`.

## 6. Infrastructure audit and the racing-driver records (18:05)

- Every per-episode record of the mouse run was classified: no infrastructure error (CUDA, worker exception, pool
  timeout) appears in any scored evaluation or gate; every failure is `StepLimitExceeded` or the pipeline's own refused
  or blocked motion. The tools score an episode that raises as a failure; they could not be changed mid-run, so the
  summary script flags infra errors and any evaluation containing one would have been re-run (none was needed).
- **Records to disregard in `logs/runs.jsonl`: lines 15-22** (1-based; four `base` / `v1` pairs written by `eval.py`,
  wall times 155, 307, 444 and 597 s). They are four copies of the repair-1 evaluation that raced for the same job
  queue after a launch bug. **The valid repair-1 evaluation is lines 23-24** (base 1.0, v1 0.0, 157 s).

## 7. Round 2: v2, the pad on the far side (18:05-18:10)

Assumption A1 (the arm on the mouse's side reaches the pad). Mouse at x = 0.38, pad at x = -0.30. Gate: **s_break 0.0**.
Repair 1: choose the placing arm from where the pad is; if it is the other arm, set the mouse down on the table in
the middle and let the other arm take it from there: base 1.0, v2 **1.0**. **Accepted.**

## 8. Round 3: v3, the mouse upside down (18:10-18:15)

Assumption A6. The mouse rolled half a turn about its long axis. Gate: **s_break 0.0**: every annotated grasp point
faced the table, every target lay below the table top. Repair 1: when the annotated grasps fail, grasp the mouse's
measured box from above across its narrowest side: base 1.0, v2 1.0, v3 **0.9**. **Accepted.**

## 9. Round 4: v4, a mug-sized object on the pad (18:15-18:25)

Assumption A5 (the space above the pad is free). A movable 7 x 7 x 10 cm, 0.3 kg box in the pad's middle. Gate:
**s_break 0.0**.

| Repair | Change | base / v2 / v3 / v4 |
|---|---|---|
| 1 | Clear the pad while the hands are free | 0.9 / 1.0 / 0.8 / 0.3. The descent to the 10 cm object started 5 cm above a grasp point 3 cm below its top: fingertips inside it |
| 2 | Start every approach from above with the fingertips clear of the object's top | **1.0 / 1.0 / 0.9 / 0.9** |

**Accepted.**

## 10. Round 5: v5, the pad under the shelf (18:25-18:40)

Five candidates screened against the round-4 pipeline (`screening/results.txt`): walls between mouse and pad 1.0, the
pad at the front edge 0.9, a slippery desk 0.9, a 1.5x mouse 0.4 (its largest model about as wide as the gripper
opening), the pad at the back 0.0. v5 = the pad at the back of the desk, where the wall shelf's lower board overhangs
its back edge 15 cm up (assumption A10). Gate: **s_break 0.0**.

| Repair | Change | base / v2 / v3 / v4 / v5 |
|---|---|---|
| 1 | When no straight-down hand reaches over the pad, also try the hand tilted 20-31 deg about its closing axis, wrist towards the robot | 0.9 / 1.0 / 0.8 / 0.8 / 0.7. Misses of 2.2-2.5 cm: the tilted mouse shifts as it falls flat |
| 2 | Lower a tilted mouse 1.2 cm past first contact so it lies flat before release | **0.9 / 1.0 / 0.8 / 0.8 / 0.8** |

**Accepted.**

## 11. Round 6: v6 rejected after a backend fix; v7 accepted (18:40-19:30)

- Five more candidates screened against the round-5 pipeline: the mouse sliding during the approach (0.8, 1.0), a
  heavy mouse 1.0, a notebook lying on the mouse 0.0 (wider than the gripper opening: it can only be pushed off), a
  1.3x mouse 0.4.
- **v6, a 1.3x mouse:** first gate passed, s_break 0.3. Repair 1 (turn the opened hand to line up with the mouse,
  check that it stayed behind) did not help, and a developer trace showed why the failures were all one model: **a
  backend bug.** `scene_edits` rebuilds a rescaled object from the env's model-id attribute and the rebuild sets
  `target_id`; a worker keeps its task object across episodes, so on the office tasks every rescaled mouse after a
  worker's first one was that first episode's model (here model 2). Only `object_scale` / `swap_object` were affected,
  and only v6 uses them.
  - **Fix and integrity rewrite (commit "robopro backend fix", 19:15):** read the task's own id attribute
    (`mouse_id`, `cube_id`) before `target_id`. `tools/INTEGRITY.json` was regenerated; the only change is the hash of
    `scene_edits.py`. The gates of v1-v5 had been checked against the previous INTEGRITY.json; none of v1-v5 uses a
    rescale or swap.
  - **Verification:** the v3 break measurement (the frozen pipeline v3 was gated with, blob `9df325d52b8f`, on v3,
    10 seeds) was repeated under the fixed backend: **0.0**, the same as its gate (`logs/runs.jsonl`
    line 128, program `frozen_v3_gate.py@9df325d52b8f`).
  - Re-gated, v6 measured s_break **0.7** and was **rejected by the break gate**. Its first gate, gate-history line
    and repair attempt are in `superseded/`; the variation files are `v6.rejected.*`.
- **v7, the mouse half under a monitor riser** (a fixed board 8 cm above the desk over the mouse's back half;
  assumption A2 from above). Gate: **s_break 0.0**.

| Repair | Change | base / v2 / v3 / v4 / v5 / v7 |
|---|---|---|
| 1 | Grasp the exposed end and draw the mouse out | 1.0 / 0.9 / 1.0 / 0.8 / 0.8 / 0.1. Held 3 cm from its end, the mouse slips out of the hand on the carry |
| 2 | Let go once out, grasp again at the centre; check the grip before aligning over the pad | v7 0.1. The hand at the centre was still too close to the board |
| 3 | Draw it out until its centre is 11 cm past the board | v7 0.4; the same code re-measured (an edit script failed, so it ran again unchanged): 0.8, with v5 0.7 |
| 4 | Look for an overhang first; end grasps with the wrist leaning away | v7 0.6. On the smaller mice only 4-5 cm stick out, less than the fingers need |
| 5 | Come in level from the open end, fingers on the long sides passing under the board, close at the centre, slide out, lift | **1.0 / 1.0 / 0.8 / 0.8 / 0.9 / 0.8** |

**Accepted.** Five accepted variations: the run stops (MAX_ROUNDS). Several scenes sit at exactly 0.8, and the
same code moved v7 by 0.4 between two checks; `summary.md` says which acceptances are marginal.

## 12. Clips (19:30-20:05)

Recorded with `website/tools/record_eval.py` and `patch_video_robotwin.py` from `main`, applied in a throwaway
checkout (never committed): the countertop camera (the head camera sees only the middle of the desk, and the mice
start at its edges), one frame per 50 simulated steps written at 5 fps (real time), 32 render samples per pixel
(training ran at 8; perception is pose-based, so the training numbers do not depend on it), and `--fixed-offset 0`
for every clip: the outcome of the first seed is reported, whatever it is.

13 clips, all at seed offset 0 (bank seed 1), in the team dataset under `sim/robopro/put_mouse_on_pad/` with
`index.json` (v1's entries marked `unsolved`) and `index.meta.json`. Ten match the expected story. The other three are
reported as recorded:
- **v1 after fails:** v1 is unsolved, so the final pipeline has no fix for it.
- **v3 after fails:** the mouse ends just outside the 2 cm tolerance on this seed, which also missed by 2.1-2.9 cm in
  several training checks (v3 is 0.8-1.0 over 10 seeds).
- **v4 before succeeds:** the round-3 pipeline, which does not clear the pad, carries the mouse into the mug, knocks
  the mug over and off the pad, and the mouse lands on the pad. Its gate scored that pipeline 0/10 on the same seeds:
  run-to-run variation, a success by collision.
- `v3__before.mp4` is a single frame: the round-2 pipeline's grasp targets for the upside-down mouse all lie below the
  table and are refused before any motion, so the robot never moves.

Two recording details: the patch's countertop camera needed `camera_frame()` to accept any static camera by name, a
render-only change made in the throwaway checkout; and `clips_hf.py` on `main` uploads under `videos/sim/`, while the
dataset (and its README) keep every task under `sim/`, so the upload used the dataset's layout.
