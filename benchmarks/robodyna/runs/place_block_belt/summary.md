# place_block_belt: GAH training summary

Benchmark RoboDyna (Opt 1+2: moving exit bowl, blocker on the belt), ur5-wsg, 10 RoboDyna eval seeds,
TAU = 0.8, MAX_ROUNDS = 5, MAX_REPAIR = 6. Stopped because two consecutive accepted rounds (4 and 5)
produced `novel: false`.

## Pipeline (final)

1. Perceive belt, place line, blocker, block (averaged detections; level-box extents). Flow direction from
   the block's side; placing arm on the block's side.
2. Belt speed from the head camera (4 s, 27 frames): per-channel white balance against the background,
   belt extent per row = span of all dark pieces carrying the moving tread pattern (RANSAC over rows,
   median over frames), pixels/metre from the belt's known length, pattern shift by least squares over
   moving pixels whose whole matching window moves. Error within 1% on every stored frame set.
3. Bowl y sampled throughout and fitted as a triangle wave (ends, speed, phase).
4. Top-down grasp across the flow; release point = the spot nearest the place line, in the lane away from
   the blocker, whose open-gripper column is free of every perceived obstacle; carry with all obstacles in
   the planner; set the block 4 mm above the belt.
5. Open the gripper at the time that puts the bowl under the block when it leaves the belt
   (measured open->release lag 0.78 s, belt pick-up 0.05 s, travel = distance / measured speed).

s0 = 1.0 (10/10).

## Variations

| Round | Variation | Axis | Assumption attacked | Gate (s0, s_break) | Result |
|---|---|---|---|---|---|
| 1 | v1 | lighting | image thresholds are absolute gray levels (35% warm light) | pass (1.0, 0.5) | recovered 1.0 |
| - | v2 | object_position | block starts on the load side (flow inferred from it) | rejected (0.7: only non-mirrored seeds affected) | - |
| 2 | v3 | obstacle | nothing but belt/blocker near the paths (bars over both belt ends) | pass (1.0, 0.0) | unsolved (best 0.7 in 8 repair evals); pipeline reverted |
| - | v4 | dynamics | the grasp holds any block (0.6 kg, friction 0.35) | rejected (1.0) | - |
| 3 | v5 | obstacle | the fixed release spot and its vertical drop are free (bars over the release zone) | pass (1.0, 0.3) | recovered 1.0 |
| - | v6 | object_motion | block stays where detected (slow push) | rejected (1.0): the edit did not move the block (backend_issues.md) | - |
| - | v7 | object_motion | block slides away during the approach (friction 0.2) | rejected (0.7): re-detection on retry recovers | - |
| - | v8 | object_motion | block shuttles back and forth | rejected (0.9) | - |
| - | v9 | camera | both belt ends visible (14 cm cover) | rejected (0.6: wrong timing still hits the bowl about half the time) | - |
| 4 | v10 | camera | both belt ends visible (22 cm cover; parameter_tuning of v9) | pass (1.0, 0.4) | recovered 1.0 |
| 5 | v11 | lighting | light is white, so "red" means a red object (red-tinted light) | pass (1.0, 0.5) | recovered 1.0 |

Final `eval.py --all`: base 1.0, v1 0.9, v5 0.9, v10 1.0, v11 1.0 (the two misses were CUDA out-of-memory at
reset on the shared GPU; the pipeline never ran on them).

## What broke the program and what did not

- Broke it: perception that leaned on how the default render looks (gray levels, white light, an
  unobstructed belt) - rounds 1, 4, 5; and geometry it never checked (a fixed release spot, obstacles the
  planner was not told about, the open fingers and held block the planner does not model) - rounds 2, 3.
- Did not break it: heavier/slicker block, a block pushed or sliding before the grasp (re-detection on each
  grasp attempt recovers), a panned/rolled head camera (row-wise scale and shift cancel), layout mirroring.
- Gate observation: a wrong belt-speed estimate still succeeds about half the time (the bowl sweeps the
  whole exit lane), so perception variations sit right at the s0/2 break threshold.
- Unsolved: v3. With bars right beside the pick area the carry had to go higher; the planner's long or free
  plans for the right arm then wandered (joint-configuration flips, start states inside inflated boxes), and
  a swing that passes the held block low over the belt past the place line fails the task.

## Rules found (reflections.jsonl)

1. Make every image threshold relative to a reference measured in the same image (e.g. normalise by the
   background brightness), and treat a perception step that returns nothing as a failure to report or retry
   rather than silently substituting a nominal value. (rounds 1, 5)
2. Give the motion planner every perceived obstacle on every move and separately clear whatever the planner
   does not model (open fingers, the held object) by explicit margins; check after each move that the tool
   arrived and the payload is still held, and never accept an unconstrained replan while carrying something
   near a region where contact has consequences. (round 2, unsolved)
3. When the goal allows a region rather than a point, choose the point inside it by checking that the tool's
   whole approach volume is free, and make image measurements of a surface robust to static things lying
   across it instead of assuming an unobstructed view. (rounds 3, 4)

Backend notes: backend_issues.md (object_motion has no effect on a grippy resting object; CUDA OOM at reset
on the shared machine).
