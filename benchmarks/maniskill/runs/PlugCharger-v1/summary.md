# PlugCharger-v1: GAH training run (ManiSkill3)

Goal: "plug the charger into the receptacle". TAU = 0.8, 10 seeds, MAX_ROUNDS = 5, MAX_REPAIR = 6.

**Result:** stopped on the 5-accepted-variations rule. The final pipeline scores **1.0 on the base task and on all
four accepted variations it was evaluated on** (`eval.py --all --seeds 10`: base, v2, v3, v4, v5 all 1.0).
One more variation (v1) was gated but not recovered and is marked `unsolved`. Two round-5 candidates were
rejected by the break gate.

The run was interrupted twice by the simulation host (jinbang-4080) going offline. The first time was
2026-10-24 13:50 EDT, back on 2026-10-25 11:40; the second was 12:00-12:56 EDT. On 10-25 at 13:05 it was
throttled because it overheated. Work resumed from the state recorded in the interim summary.

## Phase 1: s0 = 1.0
- **Perception:** average 20 000 `task_frames()` samples, because detection noise is independent per call.
- **Grasp and transfer:** top grasp on the charger base, then re-estimate the charger pose in the hand after
  grasping.
- **Alignment:** servo the charger (not the TCP) to 19 mm in front of the goal with gain-1 integral corrections
  on the exact `tcp_pose()`.
- **Insertion:** insert in translation-only steps; if the pegs jam, back off, re-estimate and retry.

Lessons (from diagnosis runs, some with perception noise switched off; these appear in `logs/runs.jsonl`):
- The screw planner's endpoint is 0.5-3 mm off, and any residual rotation in the target couples into
  translation, so fine corrections must be pure translations.
- Corrections below about 1 mm produce a one-waypoint plan that does not move the arm, so re-approach from a few
  mm back instead.
- Measure the error at the held part, not at the TCP.

## Variations

| Round | Variation | Axis | Assumption attacked | s_break | Oracle | Result |
|---|---|---|---|---|---|---|
| 1 | v1: charger tipped on its side (roll 90 deg) | orientation | A1 lies flat | 0.0 | 0.0 | **unsolved** (best 0.3 after 6 repairs) |
| 2 | v2: charger bumped ~26 mm while the hand arrives | object_motion | A3 static until grasp | 0.4 | 0.3 | recovered 1.0 |
| 3 | v3: 9 cm wall between the pick and place areas | obstacle | A7 straight transfer is free | 0.0 | 0.0 | recovered 1.0 |
| 4 | v4: heavy (0.3 kg), slippery (mu 0.15) charger | dynamics | A9/A10 light, grippy part | 0.0 | 0.0 | recovered 1.0 |
| 5 | v5: charger reversed, pegs toward the robot | orientation | A12 wrist can reach insertion | 0.0 | 0.0 | recovered 1.0 |
| (5) | receptacle turned +70 deg | orientation | A11 receptacle faces the robot | 1.0 | - | rejected: did not break the pipeline |
| (5) | 1 kg block 2 mm beside the charger | distractor | A2/A6 finger space is free | 0.9 | - | rejected: did not break the pipeline |

The rejected candidates are kept as `variations/v5_rejected_*.yaml`.

### Axes that broke the program
- **Orientation of the part to be grasped (v1, v5).** Everything assumed an upright part within +-60 deg of the
  insertion direction. v5 was recovered by turning the part on the table in steps of at most 90 deg and by
  falling back to alternative grasps the arm can actually reach. v1 needs a 90 deg roll, i.e. a regrasp
  that sets the part down instead of dropping it. That was the hardest problem in the run and was not made
  reliable within 6 iterations: it reached 0.3. The causes were drops from 10 cm (the hand cannot get low
  with its closing axis vertical), the part riding on the open finger, open-loop planner drift on large
  rotations, and the 2000-step budget.
- **Motion after perception (v2).** One accurate measurement at t = 0 went stale. The real damage was that the part
  slid in the fingers while they closed, so it was "held" at the wrong spot.
- **Obstacles between start and goal (v3).** All moves were straight and arm-only; the part hanging below the
  fingers was dragged through the wall.
- **Dynamics of the held part (v4).** The part did not slip. The arm sagged under the load, and the per-call
  integral correction was thrown away between servo calls.

### Axes that did not break it (gate rejected)
- **Receptacle orientation (+70 deg).** The goal-relative servoing handled it unchanged.
- **Clutter next to the grasp.** After rounds 2-3 (re-detect at rest before contact, check the in-hand pose,
  regrasp) a heavy block beside the charger cost only one seed.
- **Not attempted, because this backend's perception is pose-based by name:** colour, lighting, camera.

## Rules found (reflections.jsonl)
1. (v1, unsolved) When the required in-hand pose cannot be grasped from the resting pose, plan the regrasp as a
   *place*, not a drop. Choose a grasp axis that stays horizontal through the reorientation and an approach
   clear of the support before and after, and verify after every grasp that the object is actually held.
2. (v2) Re-observe the target immediately before committing to contact, acting only on a measurement taken at
   rest. After grasping, check that the object sits in the hand where the plan assumed, not merely that
   something is held.
3. (v3) Plan free-space moves for the arm plus what it holds: clear obstacles by the held object's extent
   (rise above the tallest obstacle, travel, descend). Do not trust a straight-line or arm-only collision check.
4. (v4) Treat the controller's steady-state tracking error under the current load as a calibration: measure
   commanded vs reached in free space just before a precision move and carry that offset into the contact moves.
5. (v5) Before committing to a grasp, check that the whole sequence it implies stays within the arm's joint
   range. Reorient in steps through place-and-regrasp when it does not, and verify that each motion reached its
   target instead of trusting the planner's success flag.

All five are marked `novel: true`, so the novelty stopping rule never triggered.

Cross-cutting observations:
- **The planner's success flag was wrong three times:** in v1 and v5 (open-loop drift or joint limits) and in
  v3 (contact sensing lags the close command by a few steps).
- **Checking the outcome fixed every case:** reading the reached pose, the measured in-hand pose, or a held
  signal after a short wait.

## Process notes
- **One round-2 evaluation was wasted.** The `ms` wrapper syncs `runs/` back after each command, so an edit
  made while it was running was overwritten. After that I edited only when no `ms` command was running and
  checked the file with `cmp` after every run.
- **Repair iterations:** v1 used 6/6, v2 1, v3 2, v4 3 and v5 5. Iterations were judged with a per-seed debug
  script (`dbg/diag.py`, not committed). Acceptance always used the official `eval.py --all`.
- **v1's last attempt** is kept in `wip/pipeline_v1_repair6.py`. `pipeline.py` does not include it.
