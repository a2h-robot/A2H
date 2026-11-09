# Assumptions of pipeline.py (catch_valley_ball)

Re-read after round 3; line numbers refer to `pipeline.py` at 9f9fbaa. Last column: the round
that attacked it (the round-1 to round-3 rows describe the pipeline as it was before that round's repair).

| # | Category | Assumption | Code that relies on it | Attacked |
|---|---|---|---|---|
| A1 | where objects are | The box starts roughly at the landing x, so a +y push is enough (round-1 pipeline) | (removed: the controller now fixes x first, 357-372) | v1 |
| A2 | where objects are | The landing is reachable by the catch-side arm, and the box faces can be reached from all four sides (+y side needs the tcp 15 cm beyond the box) | 236, 314-321, 323-335 | |
| A3 | where objects are | The red line marks the ramp: its x - 6 cm is the exit, its x + 14.2 cm the landing, its y-extent the lane width | 229-238, 35-36 | |
| A4 | orientation | The box stays axis-aligned: the face contact offsets `FINGER`/`FINGER_X` hold and a push along a world axis moves it along that axis | 19-22, 314-321, 337-350 (the x bias only partly compensates) | |
| A5 | whether objects move | Once placed, the box stays put (round-2 pipeline re-checked only y) | (removed: every cycle re-measures both axes, 357-381) | v3 |
| A6 | whether objects move | The ball's lateral path is a line in x reflecting off rails, changed only by equal-mass bounces with the black ball | 64-169, 277-303 | |
| A7 | whether objects move | Release is 5.3-6.6 s after start; nothing delays or speeds the ball | 28, 293-301, 352-356 | |
| A8 | between gripper and target | Obstacles are only near the box's start; the box's own path to the landing and the hand's route over the box are free | 241-275 (only the start strip is checked), 323-335 | v2 (start strip), v5 (low obstacle under the hand) |
| A9 | between gripper and target | Nothing but the black ball can land in the box | 224, 288-291 | |
| A10 | contact response | The box is light and has high friction with the table: it moves with the finger and stops when the finger stops (no coasting), so "push to goal - offset" puts it at the goal | 20, 200-221 (`slide`), 337-350 | (v4 attempt, unsolved) |
| A11 | contact response | Open fingertips reach 4.3 cm below the tcp, so a tcp at 0.80 m hits the 4 cm box wall | 20-21, 323-335 | v5 |
| A12 | appearance / camera | Not used: `detect()` is by name | 42-44 | |
| A13 | sensing | 16 averaged detections give ~2.5 mm noise; the box pose from 10 detections is good to ~3 mm | 30, 42-44, 305-306 | |

Backend note: the box's y and orientation are re-parked by RoboDyna's `_prepare_push_box()` after scene
edits, so only its x can be changed with `object_pose`; `object_motion` acts during the episode.
