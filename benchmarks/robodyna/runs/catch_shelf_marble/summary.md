# catch_shelf_marble: GAH training summary (RoboDyna, Opt 1+2)

Goal: "Move the bowl to left and right by pressing the corresponding keys to catch the red ball."
Settings: TAU 0.8, 10 seeds (RoboDyna's Opt 1+2 evaluation seeds), MAX_ROUNDS 5, MAX_REPAIR 6.

## Phase 1: solver
`pipeline.py` is a closed-loop servo. The arms close their grippers and hover just above their keys. A 2-3 cm press stroke fires a key, and the release stroke lets it spring back. Each loop the pipeline averages 20 detections of the ball and the bowl, works out an aim point and presses until the bowl reaches it, minus a learned release overshoot. The aim point is:
- while the ball slides: its x plus 4.5 cm in its direction of travel;
- right after a 2 cm-plus sideways jump: the jump direction plus 5 cm;
- while the ball falls: the ballistic landing x at the rim height.

The pipeline cannot see the shelves.

**s0 = 0.8.** Repeated 10-seed runs of near-identical code gave 0.6 to 0.9. The recorded value was the second 0.8 in a row of the final code, and a re-check before round 4 also gave 0.8. Seeds 10001 and 10008 fail on every run. On those seeds the ball reverses on the oscillating bottom shelf, then teleports 2-5 cm to RoboDyna's planned edge and leaves with a 0.15-0.2 m/s physics kick in the opposite direction.

## Variations

| v | axis | what | gate (s0, s_break) | outcome |
|---|---|---|---|---|
| v1 | object_motion | belt creep: bowl drifts +6 cm/s all episode | rejected (0.8, 0.8) | closed loop absorbs it |
| **v2** | obstacle | 18 cm post 1.5 cm beside the left key | **accepted (0.8, 0.3)** | **recovered: base 0.8, v2 0.8** |
| v3 | distractor | thin 3x8 cm stick standing in the bowl | rejected (0.8, 0.5) | stick slides aside or is flung out |
| **v4** | distractor (tuning of v3) | 5x8 cm canister standing in the bowl | **accepted (0.8, 0.1)** | **unsolved, best 0.4** |
| v5 | distractor | block lying on the left key | rejected (0.8, 0.8) | the press shoves it off |
| v6 | object_motion | v2's post slides in at 7-9 s | rejected (0.8, 0.5) | too late for half the seeds |
| **v7** | object_motion (tuning of v6) | post slides in at 4-6 s | **accepted (0.8, 0.4)** | **unsolved, best 0.7** |
| v8 | lighting (+camera, colour) | 30 % warm light, moved head camera, recoloured ball and bowl | rejected (0.8, 0.8) | pipeline never reads images |

The final `pipeline.py` is the round-1 program. The attempts for v4 and v7 are kept in `attempts/`.

## What broke the program and what did not
- **Broke it:**
  - clutter next to the button (the gripper's width, v2);
  - an occupied receptacle (v4);
  - an obstacle that appears after the program has looked (v7).
- **Did not break it:**
  - a drifting controlled object (v1): every quantity the servo uses is re-measured at 50 Hz;
  - a loose item on the button (v5);
  - a thin item in the bowl (v3);
  - appearance changes (v8): no image is used.
- **Unsolved for time reasons.** v4 and v7 have working strategies:
  - v4: side-grasp the item out of the bowl, under the shelves;
  - v7: re-perceive the obstacles and re-plan the press pose.

  Both cost 1-10 s of blocking, one-arm-at-a-time primitives, and the ball lands at 5.7-11.7 s on several seeds.
- **Edit space exhausted.** Keys, belt and shelves are parts and cannot be edited. Detection noise is fixed. RoboDyna resets the marble's dynamics at the final drop, and the bowl is kinematic. So the remaining assumptions (key mapping, detector confusion, ball dynamics) could not be attacked. The run stopped after 8 designed / 3 accepted variations, before the 5-accepted rule fired. All three reflections are novel.

## Rules found (reflections.jsonl)
1. Before approaching a target, check the tool's full footprint (not just its tip) against known obstacles over the whole approach and stroke. Choose an orientation or offset that clears them, and verify the reached pose instead of trusting the planner's success flag.
2. Check that the goal location or receptacle is actually free before relying on it. When it must be cleared first, budget the clearing time against the deadline at the start instead of discovering the overrun afterwards.
3. Treat every perceived fact the plan depends on, including obstacles and free space, as something that can change. Re-check it inside the control loop and re-plan when it no longer holds, rather than snapshotting it once at the start.

## Backend observations (no backend issue filed)
- The world responds to the robot as documented. Pressing a key through the API moves the bowl at 0.18 m/s while it is held. Success latches.
- cuRobo does not model the WSG gripper. `move_to` returns True for paths that jam the palm into an obstacle.
- The marble teleports at the hand-off to physics when the final shelf is the oscillating one. This is a RoboDyna plan/live-phase mismatch, and the reason two base seeds look out of reach for a purely reactive program.
- The shared <sim-host> GPUs ran out of memory several times (CUDA OOM at reset; up to 3 of 10 episodes in one call). Those episodes count as failures in the tools' rates, which adds to the ±0.1-0.2 run-to-run spread.
