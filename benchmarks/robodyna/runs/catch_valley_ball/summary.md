# catch_valley_ball: GAH training summary (pilot, RoboDyna Opt 1+2)

Backend history: the first attempt was blocked because the box stayed kinematic on the eval path
(s0 = 0.0, `backend_issues.md` item 1). It was fixed upstream (RoboDyna 5303d554). Phase 1 was then
re-recorded on the fixed backend: **s0 = 0.8** (`s0.json`).

Status: **4 accepted variations (v1, v2, v3, v5), 1 unsolved (v4), 2 rejected by the break gate (v6, v7).**
The stopping rule (5 accepted, or 2 accepted rounds in a row with novel: false) was not reached.
I stopped because the two designs after round 5 no longer broke the pipeline (s_break 0.7 and 0.8).
The remaining axes cannot be expressed for this task in this backend: box mass, friction and
rotation are reset by RoboDyna, and the ball is kinematic. Others do not reach the pipeline,
because appearance, lighting and camera are irrelevant to `detect()`-by-name.

## Variations

| Round | Variation | Axis | Edit | Gate s0 / s_break | Result |
|---|---|---|---|---|---|
| 1 | v1 | object_position | box starts at the table centre line (x = 0), 0.3 m from the catch x | 0.7 / 0.0 | recovered 0.8 (base 0.8) |
| 2 | v2 | obstacle | 12 cm block 11-14 cm behind the box's spawn | 0.7 / 0.0 | recovered 0.9 (base 0.8, v1 0.8) |
| 3 | v3 | object_motion | box knocked +x by ~7 cm at t = 3.5 s | 0.8 / 0.2 | recovered 0.8 after 6 repairs (base 1.0, v1 0.8, v2 1.0) |
| 4 | v4 | object_motion (tuning) | box knocked +y (away from the robot) at t = 4.3 s | 0.9 / 0.4 | **unsolved** after 5 repairs (best 0.4); reverted |
| 5 | v5 | obstacle | 2.2 cm rail 3-9 cm behind the box | 0.9 / 0.2 | recovered 0.8 (base 0.9, v1 0.9, v2 1.0, v3 0.8) |
| - | v6 | obstacle (auto-tuning) | 5 cm post beside the box's path to the landing | 0.9 / 0.7 | rejected (does not break) |
| - | v7 | distractor | loose 1 kg block in the landing area | 0.9 / 0.8 | rejected (does not break) |

Axes that broke the program: object position (v1); obstacles behind the object (v2) and under the
hand (v5); the object moved after placement (v3); a late disturbance in the costly direction (v4,
not recovered). Axes that did not break the final program: a post beside the path (v6) and loose
clutter at the goal (v7). The closed-loop controller from round 3 absorbed both.

## How the program evolved

- **Phase 1:** predict the landing by fitting the ball's reflected lateral path (rails plus
  ball-ball bounces) and extending it to the landing x. Push the box from the robot side with a
  fingertip, then correct y.
- **Round 1:** push x first, then y.
- **Round 2:** read the obstacles and choose between two grips by the free space each needs behind
  the box. The first push stops at the lane centre, and the robot creeps forward as the estimate
  firms up.
- **Round 3:** replace the script with a closed-loop controller. Each cycle it re-predicts, measures
  the box on both axes and fixes x first. It goes over the box to change faces only when time
  allows. It learns the push shortfall.
- **Round 5:**
  - Pick the push height from the obstacle heights (averaged obstacle boxes).
  - Close each push on the box position (short extra slides while the box is short and still moving).
  - A box turned by a bump presents a corner, so the fixed contact offset left it 2-5 cm short.
- **v4 attempts (not kept, `dev/pipeline_v4_last_attempt.py`):**
  - Face-dependent grips, with a compact grip for the far face.
  - A keep-out zone for the ball's flight corridor.
  - Home orientation, to avoid 0.5 s wrist turns.
  - Longer push segments.

  None of these was fast enough to push back from the far side in the ~1-2 s left.

## Rules found (reflections.jsonl)

1. Plan a push from the full vector between where the object is and where it must go, pushing along
   each needed direction from the appropriate face. Do not assume the object starts where it
   usually does.
2. Before choosing how to hold or approach an object, check the free space that grasp or approach
   needs against everything perceived nearby. Keep a grip with a smaller footprint for clutter.
3. After acting on an object, keep re-measuring every quantity the success condition depends on
   until the deadline. Correct with a general closed loop that can approach from any direction, not
   a script that re-checks only what it last changed.
4. (unsolved v4) Budget the worst-case time to reverse a correction against the deadline before
   committing to a one-sided strategy. If reversing takes longer than the time left, constrain the
   object on both sides (or hold it) instead of pushing from one side.
5. Use the measured heights of nearby objects, not just their footprints, to pick the contact height
   of a push or grasp. Close the loop on the object's position, not the hand's.

## Notes for the team

- `dev/` holds diagnostics that use simulator internals (never the pipeline): probes,
  `patched.py` (logs ball and box trajectories), `offline_pred.py`, and the v4 attempt.
- Backend issues 4-6 in `backend_issues.md`:
  - `remote.sh` hangs after the remote exits.
  - Worker timeouts count as failures and slow the gates.
  - Box pose, mass and friction edits are overridden by the task.
