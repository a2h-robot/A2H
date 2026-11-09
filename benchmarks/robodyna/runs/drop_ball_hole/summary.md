# drop_ball_hole: GAH training summary (RoboDyna Opt 1+2, ur5-wsg)

Goal: "Guide the ball through the target hole of the rotating sorter into the container."
Settings: TAU 0.8, 10 seeds (RoboDyna's Opt 1+2 evaluation seeds, offset 0), MAX_ROUNDS 5, MAX_REPAIR 6.

## Phase 1

The pipeline works like this:
- **Platform model.** It fits yaw(t) = a + w t by least squares to averaged detections of the sorter and both holes. The platform turns at 2.4-3.6 rad/s, and w is recovered to about 0.003 rad/s.
- **Target hole.** It picks the hole that is wider than the ball (the decoy is 4.6 cm, the ball 5 cm).
- **Grasp and carry.** It grasps the ball top-down and carries it over the hole's orbit.
- **Timed release.** It commands the release LEAD seconds before the hole passes under the ball.

Two things were calibrated in Phase 1:
- **Release height.** Releasing the ball close to the platform made it rest on the platform while the fingers still touched it, so the spinning platform dragged it. Releasing it in the air, 3 cm above the platform, fixed this.
- **Lead time.** The ball leaves the opening fingers after a fairly constant 0.50 s.

**s0 = 1.0** (10/10, twice).

## Variations

| Round | Variation | Axis | Assumption attacked | Gate (s0, s_break) | Result |
|---|---|---|---|---|---|
| 1 | v1 | object_motion | The ball lies still between detection and grasp. In v1 it rolls at 4 cm/s from t = 1.2 s. | 1.0, 0.0 | recovered, 1.0 (base 1.0) |
| 2 | v2 | obstacle | Nothing stands between the ball and the sorter. v2 adds walls 30 cm high on both sides. | 1.0, 0.0 | recovered, 0.8 (base 1.0, v1 0.9) |
| 3 | v3 | obstacle | The release point on the arm's own side is free. v3 adds brackets over it. | 1.0, 0.0 | recovered, 0.9 (all scenes >= 0.8) |
| 4 | v4 | distractor | The default grasp yaw has room for the fingers. v4 puts a box 6 mm beside the ball. | 1.0, 0.3 | recovered, 0.9 on the 6th repair (base 1.0, v1-v3 0.8) |
| 5 | v5 | distractor (parameter tuning) | One of the four candidate yaws is free. v5 boxes the ball in on two sides. | 1.0, 0.1 | **unsolved** (stopped after 4 of 6 repair iterations; min 0.6-0.7 in each); reverted |

Two variations were rejected by the gate:
- **v1, first attempt.** Rejected by the integrity gate because RoboDyna was updated during the run. It passed after merging `e-robodyna`.
- **v5, first design** (`dynamics: friction 0.3`). Rejected by the break gate with s_break 0.9: the frozen pipeline still succeeded.

Two other designs were dropped before gating:
- **A heavier or more slippery ball** (`mass 0.3`, or `mass 0.18` with `friction 0.5`). The ball slips out of the WSG grip while it is lifted. That would test grip strength, not the program, and may not be solvable.
- **A block riding on the platform.** It did not break the pipeline: 3/3 seeds still succeeded in the probe.

## Which axes broke the program

These axes broke it:
- **object_motion (v1).** A rolling target.
- **obstacle (v2, v3).** Clutter on the transport path, and a blocked default goal pose.
- **distractor (v4, v5).** Clutter around the grasp.

These did not:
- **dynamics (friction).** The landing tolerance absorbed the earlier release.
- **distractor on the platform.**

Not tried: lighting, colour and camera. The pipeline uses `detect()` only, so these edits cannot affect it, and the gate would reject them.

## Rules found (from reflections.jsonl)

1. **Velocity before contact.** Before an irreversible contact action, estimate the target's velocity, not only its position. Re-verify its pose relative to the end effector immediately before acting. Intercept a moving target at a predicted point with the gripper oriented to receive it, and abort rather than act blind.
2. **Swept volume and failed motions.** Size every transport waypoint to the swept volume of the arm plus the held object, because the planner usually models only the arm. Pass perceived obstacles to every plan. Treat a failed motion as a stop condition that triggers an alternative, never as a step to skip.
3. **Families of goal poses.** When the goal admits a family of equivalent target poses, keep the whole family and choose one at run time against the perceived free space. Do not hard-code the first one that worked.
4. **Gripper footprint.** Check the gripper's own footprint (the open fingertips) against neighbouring objects when choosing a grasp. At release, orient and retract the fingers so they are outside the region the released object will move through.
5. **Tolerance versus noise.** When success hinges on a tolerance near the noise floor, measure the full error distribution of the actuation before tuning means. Change the strategy (more margin, closed-loop correction, a second chance) when the tolerance is under about three standard deviations.

## Notes for the team

- **The planner does not keep the WSG gripper body clear of obstacles.** In v2 the fingers and the gripper base touched a wall that was passed in `avoid`, with the plan reported as successful. cuRobo's collision model for ur5-wsg appears to omit the gripper or to under-size it. Rule 4 comes from this.
- **The drop is a coin flip near 0.8 per episode.** The hole is 5 mm wider than the ball and moves at about 0.2 m/s. Across reruns of identical code, a 10-seed success rate swings by +-0.1-0.2. The base task stays at 0.9-1.0, but the variations swing between 0.5 and 0.9, so ">= 0.8 on every scene in one --all run" is partly luck. This is why v5 ended unsolved, and why v4's recovery needed all 6 repairs.
- **Probe and eval disagree on single seeds.** Ground-truth probes (`dev/probe.py`) and `eval.py` gave different outcomes on the same seed. The likely cause is GPU planner nondeterminism.
- **Backend flakes.** Two episodes failed at reset with infrastructure errors: CUDA out of memory, and a cuRobo tensor shape mismatch. One run hit "no collision shapes attached" when calling `detect('sorter')`, and the pipeline now catches it.
- **The container is cylindrical in these seeds.** The geometry the pipeline uses comes from detection, so this needed no change.
