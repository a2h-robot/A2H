# Assumptions of pipeline.py (round 0 version)

Line references are to the Phase-1 pipeline (commit c732269).

| # | Category | Assumption | Code that relies on it |
|---|---|---|---|
| A1 | where | The knob is a detectable part `flat_stove_1_button`, and its detected position is the rotation axis. | `axis = api.detect(KNOB).position` (twist loop) |
| A2 | orientation | The knob's fin (the graspable ridge) is thin along the knob's local x axis, so a top grasp closing along that axis pinches it. | `fin_thin = Rk[:, 0]`, `yaw = arctan2(...)` |
| A3 | orientation | The stove is upright: the knob axis is world z, so a twist is a rotation about world z. | `Rz(d)` in the twist loop, `p2[2] = p[2]` |
| A4 | orientation / state | The knob starts in the off position and "on" means rotating it counter-clockwise (positive about +z) by >= 0.5 rad; progress is the change of the knob yaw relative to the stove base since the start. | `rel0 = knob_rel(api)`, `turned = wrap(knob_rel - rel0)`, `TURN_GOAL = 1.0` |
| A5 | between gripper and target | Nothing is above the knob: the straight vertical approach from 8 cm above is free. | `api.grasp(KNOB, strategy="top", ...)` |
| A6 | between gripper and target | Nothing is beside the fin within the open finger span (about 4-5 cm each side along the closing axis), so the fingers can descend around it. | `api.grasp(KNOB, strategy="top", yaw=yaw)` (one fixed yaw, no alternative) |
| A7 | contact response | The knob turns easily: the friction of a pinch on the fin is enough to drag it along with a 0.3 rad wrist rotation at "slow" speed, and the arm does not hit its force limit. | twist loop, `d = min(0.3, ...)`, `speed="slow"` |
| A8 | contact response | If the grasp did not close on the fin, the gripper closes fully (width < 4 mm); otherwise it is pinching the fin. A refused approach (gripper still open) is NOT detected. | `if api.gripper_width() < 0.004` |
| A9 | motion | The stove and knob do not move during the episode except for the knob rotation. | `axis` computed once after the grasp |
| A10 | where | The knob is within reach with a top-down hand and the wrist can rotate +1 rad from the grasp yaw without hitting joint limits. | `api.grasp(...)`, twist loop |
| A11 | appearance / camera | None: the pipeline never uses the camera, colour or lighting (perception is pose-based). | - |
| A12 | size | The knob's bounding box gives the right grasp height (default depth = min(3 cm, half the height) below the top). | `api.grasp(KNOB, strategy="top", yaw=yaw)` (default `depth`) |
| A13 | perception | Detection noise (about 1 cm, 3 deg) is small compared with the fin and the finger opening; averaging 3 detections is enough to measure the knob angle. | `knob_rel(api, n=3)` |

## Added by the round-1 repair (pipeline after v1)

| # | Category | Assumption | Code that relies on it |
|---|---|---|---|
| A14 | between gripper and target | Anything that blocks the pinch is a movable object within 13 cm of the knob whose top is above the knob; fixtures with parts are never clutter. | `clutter_near_knob()` (`CLEAR_R`, `api.parts(name)`) |
| A15 | contact response | Clutter can be top- or side-grasped, or slides when pushed, and there is free table 15 cm further from the knob. | `relocate()`, `push_away()` |
| A16 | orientation | One of the hand rolls (0, +-0.35, +-0.6 rad) about the fin axis gives a feasible pinch. | `TILTS` in `twist_pinch()` |
| A17 | contact response | A pinch that closed to between 4 mm and 75 mm holds the fin firmly enough that rotating the wrist rotates the knob; 8 twist steps of <= 0.3 rad suffice. | `0.004 < w < 0.075`, twist loop in `twist_pinch()` |

## Added by the round-2 repair attempt (v2 unsolved; changes kept)

| # | Category | Assumption | Code that relies on it |
|---|---|---|---|
| A18 | between gripper and target | A pinch that reaches the fin also leaves room for the twist motion around the knob axis. | `twist_pinch()` twist loop |
| A19 | contact response | Pushing the fin's tall end tangentially with closed fingertips at 3-4 cm above the knob origin turns the knob; the short end works only rarely. | `push_seat()`, `twist_push()` |
| A20 | time | Failed attempts are cheap: at most 4 moving pinch candidates, 4 push seats, and 3 stalled arc steps fit in the 150 s budget. | `MAX_PINCH_MOVES`, `seats=4`, `stall < 3` |
| A21 | between gripper and target | Objects whose bottom is more than 6 cm above the knob are overhangs, not movable clutter. | `clutter_near_knob()` |
