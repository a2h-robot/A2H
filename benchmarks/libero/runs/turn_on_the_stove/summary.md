# turn_on_the_stove: GAH training summary

Benchmark: LIBERO-Goal (Panda, single arm). Goal: "turn on the stove" (knob joint >= 0.5 rad).
TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6.
Wall clock: about 1 h (10:30 to 11:30). Token usage: not available to the agent.

## Result

- **s0 = 1.0.** The Phase-1 pipeline pinches the knob's fin from above at the fin's yaw and rotates the wrist about the knob axis in 0.3 rad steps. Closed loop: after each step it measures the knob yaw relative to the stove base. It worked on the first attempt.
- **Accepted rounds: 1** (v1). **Unsolved: 1** (v2). The run then stopped at round 3: none of 8 candidate variations across 6 axes broke the round-2 pipeline (break gate), and the only breaking axis was unsolvable within MAX_REPAIR.
- Final pipeline on the base task and v1: 1.0 / 1.0. On the unsolved v2 it scores 0.4.

| Round | Variation | Axis | s_break | s_recovered | Outcome |
|---|---|---|---|---|---|
| 1 | v1: tall movable box 6 cm beside the knob | distractor | 0.0 | 1.0 | accepted, recovered in 5 repairs |
| 2 | (v2 try a) knob damping 1 -> 10 | dynamics | 1.0 | - | rejected (no break) |
| 2 | (v2 try b) knob damping 1 -> 40 | dynamics (tuning) | 1.0 | - | rejected (no break) |
| 2 | v2: fixed shelf 12 cm above the knob | obstacle | 0.0 | 0.4 | accepted by the gate, **unsolved** after 6 repairs, reverted |
| 3 | stove rotated -90 deg + shifted 12 cm | orientation | 1.0 | - | rejected |
| 3 | stove scaled 0.6 (small knob) | size | 1.0 | - | rejected |
| 3 | fixed side panel, 9 cm from the knob axis | obstacle | 1.0 | - | rejected |
| 3 | bowl slides in beside the knob during the run | object_motion | 0.9 | - | rejected |
| 3 | heavy (6 kg, high friction) pot beside the knob | distractor | 1.0 | - | rejected (a 15 kg pot fell through the table: invalid) |
| 3 | stiff knob (damping 40) + spice box | dynamics | 0.9 | - | rejected |
| 3 | v2 shelf raised 4.5 cm (v3.yaml, last candidate) | obstacle (tuning) | 0.8 | - | rejected |

Not tried: lighting, colour and camera. The pipeline never uses the camera (perception in this backend is pose-based), so these variations cannot break it. That is a property of the harness, not robustness of the program.

## What broke the program and what did not

- **Broke it: lack of space around the hand.**
  - *Beside the knob (v1).* A box inside the ~20 cm footprint of the hand along its closing axis made the only strategy, a top-down pinch, infeasible.
  - *Above the knob (v2).* A shelf left room to reach the fin, but no room for the twist that must follow the pinch.
  - In both cases the planner refused the motions, and the Phase-1 program did not notice the refusal: it then "twisted" in empty air.
- **Did not break it:**
  - pose and orientation of the appliance, and part size, because everything is computed in the detected part frame;
  - stiffness of the joint, because the loop measures progress and re-grasps;
  - moving clutter, because the scene is re-detected after each failure;
  - fixed or heavy clutter after round 2, because the program now has several contact strategies (rolled or pitched pinches, a fingertip push) and the planner selects among them.
- **Why v2 stayed unsolved.** A fingertip push of the fin's tall end from a horizontal hand works under the shelf (7/10 seeds when run alone). In the full program, failed attempts (candidate seats that first move to a pre-pose and are then refused) use up the 150 s budget. 6 of 10 seeds ended with StepLimitExceeded.

## Rules found

1. Check every primitive's outcome (refused, incomplete, or a gripper that did not close) before acting on it. When the area around a target is cluttered, plan for the space the whole hand needs, and clear movable clutter before falling back to weaker contact strategies. (round 1, novel)
2. Before committing to a contact, check that the whole manipulation (approach, the actuating motion and the retreat) fits in the actual free space. Budget time: try options that are cheap to refuse first, cap expensive retries, and keep a non-grasping strategy (push or nudge) that needs less clearance. (round 2, unsolved, novel)

Observations behind the rules:
- Measuring the task variable after each action (here, the joint angle) and retrying made the program robust to stiffness, slipping and disturbances. None of the dynamics variations broke it.
- `gripper_holding()` is True when pinching a fixed object, so "I grasped it" does not mean "I can move it". Check that a relocated object actually moved.
