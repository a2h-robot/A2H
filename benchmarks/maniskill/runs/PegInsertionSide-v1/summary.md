# PegInsertionSide-v1: GAH training summary

Goal: "insert the peg into the hole from the side". Settings: TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5,
MAX_REPAIR = 6, perception noise 1 cm / 3 deg per detection.

**Status: finished.** The run stopped at MAX_ROUNDS: 5 accepted variations (v2, v6, v9, v11, v13). One round
(v4) ended unsolved. The final pipeline scores base 0.9, v2 1.0, v6 1.0, v9 0.9, v11 0.8, v13 0.9.

**Deadline:** the 8-hour budget in GAH_TASK.md (from about 13:00 EDT on 2026-10-24) was missed. The
simulation machine (jinbang-4080) dropped off tailscale at about 14:20 EDT on 2026-10-24, in the round-2 repair. It
came back at 11:42 EDT on 2026-10-25, dropped again 12:05-12:56, and on its return the operator killed
all running simulations at 13:05 because it was overheating. After that `ms` commands queued
(at most 2 at once across all runs, 3 workers each). Rounds 2 (rest of the repair) to 6 ran on
2026-10-25 between 11:42 and 16:25 EDT. No round was skipped because of the outage. Interrupted
evaluations were re-run, and their results are the ones reported here.

## Phase 1
- s0 = **0.9**; the official motion-planning oracle scores 0.4 on the same seeds, so TAU stayed at 0.8.
- What it took: averaging 300-1000 noisy detections per pose estimate; re-measuring the peg in the
  hand after grasping; correcting the *command* (not the measured TCP pose) in a closed loop, because
  the arm does not reach its commands exactly; letting the arm settle before measuring; inserting in
  2 cm steps; and checking alignment at both ends of the peg (tilt), not only at its centre.

## Variations
| Var | Axis | Edit | s_break | Gate | Outcome |
|---|---|---|---|---|---|
| v1 | orientation | peg yaw +180 deg | 0.7 | rejected | the planner absorbs a 180 deg wrist yaw |
| v2 | object_motion | peg slides 0.15 m/s for 0.5 s after detection | 0.1 | pass (oracle 0) | **round 1**: recovered (v2 1.0, base 0.9) |
| v3 | dynamics | peg 1 kg, friction 0.3 | 0.6 | rejected | |
| v4 | dynamics | peg 2 kg, friction 0.15 (tuning of v3) | 0.0 | pass (oracle 0) | **round 2: unsolved** after 6 repairs, reverted |
| v5 | distractor | loose box 1-2 cm beside the peg's grasp region | 0.9 | rejected | the finger just pushes it aside |
| v6 | obstacle | the same box, immovable | 0.0 | pass (oracle 0) | **round 3**: recovered in 1 repair (v6 1.0) |
| v7 | orientation | box (hole) turned -90 deg | 0.6 | rejected | some IK failures, mostly absorbed |
| v8 | orientation | v7 + peg reversed | 0.6 | rejected | |
| v9 | object_position | peg ~0.82 m from the robot base (limit 0.85) | 0.0 | pass (oracle 0) | **round 4**: recovered in 4 repairs (v9 0.9) |
| v10 | dynamics | friction 2.0 on peg and box | 1.0 | rejected | sticky contact does not hurt |
| v11 | obstacle | shelf 15 cm above the peg's rear half | 0.0 | pass (oracle 0) | **round 5**: recovered in 1 repair (v11 0.9, later 0.8) |
| v12 | orientation | peg leaning ~18 deg on a 4 cm block | 0.5 | rejected (just above 0.45) | |
| v13 | orientation | peg leaning ~25 deg on a 6 cm block (tuning of v12) | 0.3 | pass (oracle 0) | **round 6**: recovered in 1 repair (v13 0.9) |

Final `eval.py --all --seeds 10`: base 0.9, v2 1.0, v6 1.0, v9 0.9, v11 0.8, v13 0.9.

### Which axes broke the program and which did not
- **Broke it:**
  - object motion after detection (v2)
  - heavy and slippery object (v4; not recovered)
  - immovable obstacles at the grasp: beside it (v6) or above it (v11)
  - object near the edge of the reach (v9)
  - object not lying flat (v13)
- **Did not break it:**
  - yaw changes of the peg or the fixture (v1, v7, v8: 0.6-0.7); motion planning absorbs them
  - moderate mass/friction (v3)
  - high friction (v10)
  - loose clutter that the fingers can push aside (v5)
  - a mild lean (v12)
- **Not attacked:**
  - camera, lighting and colour (A10): perception in this backend is pose-based, so these edits cannot reach the pipeline.
  - box motion (A4): the box with the hole is kinematic, and `object_motion` does not move it (checked).
  - perception bias (A9): the noise model is part of the API, not a scene edit.

### Round 2 (v4, unsolved)
A tail grasp lets the 2 kg, slippery peg pivot in the fingers. A grasp within ~3 cm of its centre
holds it level (v4 reached 0.8), but then the hand hits the box face ~2 cm before the head is deep
enough, so the default scene dropped to 0.2-0.7. Grasping 4.5 cm back clears the face, but v4 then fell
to 0.1. No single grasp point satisfies both constraints. The likely solution, not tested within the
budget, is a regrasp part-way: insert with the centre grasp until the hole supports the peg, then
release and push or regrasp at the tail.

## Rules found
1. (round 1) Re-perceive the target immediately before every contact action. Require it to be stationary
   across two estimates separated in time, and verify the result of the action with a retry loop,
   instead of trusting an estimate taken before the arm started moving.
2. (round 2, unsolved) When one grasp must serve both transport and final placement, derive the grasp
   point from both constraints: the torque from the centre of mass during transport, and the hand's
   clearance from the target at the end. When they conflict, plan a regrasp or hand-off part-way
   instead of tuning a single compromise grasp point.
3. (round 3) Before committing to a grasp, check the whole hand's footprint (open fingers and palm) against
   every perceived object, and search over alternative grasps when it collides. If the task-preferred
   grasp is blocked, first move the object to free space with any grasp that fits, then regrasp.
4. (round 4) Check every planned grasp against the robot's comfortable workspace (a margin inside its reach
   limit, not just planner success). When the object lies outside it, first move it toward the
   workspace centre with whatever grasp is easiest. Never accept a motion the planner only finds by a long
   detour.
5. (round 5, not novel: a refinement of rule 3) The footprint must include the wrist and arm above the
   hand. When relocating an object, verify the precondition of the next action (a free, reachable
   grasp) at the destination before choosing it.
6. (round 6) Plan grasps from the object's full measured 6-DoF pose, never from a projection that
   assumes its usual resting orientation.

Across rounds, most failures came from a quantity the program computed from the target object
alone (grasp point, grasp orientation, reach) while the scene or the robot also constrains it.
Writing down every constraint on each decision, including the robot and the surrounding objects, would have
caught rounds 2-6 in advance.

## Files
- `pipeline.py`: final pipeline, frozen at the round-6 commit.
- `assumptions.md`: assumptions A1-A13 of the Phase 1 pipeline, with what each round found.
- `variations/v*.yaml`, `variations/v*.gate.json`, `gate_history.jsonl`: variations and their gate records.
  v4 is marked `unsolved: true`.
- `reflections.jsonl`: one reflection per round (6 lines).
