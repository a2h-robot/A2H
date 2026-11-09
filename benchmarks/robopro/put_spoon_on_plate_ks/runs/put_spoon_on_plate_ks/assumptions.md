# Assumptions of the put_spoon_on_plate_ks pipeline

Line numbers refer to `pipeline.py` as committed after Phase 1 (s0 = 0.9). Status: open = not attacked yet.

| # | Assumption | Code that relies on it | Status |
|---|---|---|---|
| A1 | The plate stays where it was seen at the start: it is detected once, before the pick, and never again. | l.57 `plate = avg_detect(api, "plate")`; l.65 and l.73 use that one estimate | attacked by v1 (round 1): plate slid 7 cm during the pick; fixed by settled re-detection after the pick and before release |
| A2 | The spoon lies still where it was detected until the gripper closes on it. | l.44 detect, l.49 grasp of that estimate | screened (s2, spoon nudged 6 cm just before the grasp): 0.9, the re-detect-and-retry loop already handles it |
| A3 | The arm on the spoon's side of the robot can reach both the spoon and the plate (one-arm carry, possibly across the centre line). | l.47 arm from the sign of x only | tested by v3 (rejected, s_break 0.9): the right arm reaches a plate on the far left |
| A4 | Nothing crowds the spoon: its annotated grasp points can be approached and the fingers fit around it. | l.49 `api.grasp(spoon)` with default candidates | attacked by v4 (round 2): spoon upside down, every annotated grasp approached from under the counter; fixed by a geometric top-down fallback grasp. v8 (round 4, unsolved): spoon partly under a low overhang, no room for the hand anywhere on it; screening s5 (spoon in a narrow slot) also breaks the program |
| A5 | The spoon stays in the hand during the lift and the carry (nominal mass and friction); a drop is only noticed after the carry. | l.49, l.66-67 | attacked by v7 (round 3): 250 g spoon with friction 0.2 slid out of the pinch during the lift; fixed by a test lift and changed re-grasps |
| A6 | The carry at 10 cm above the counter is free of tall or moving obstacles (the planner sees static ones only). | l.63-66 `move_to(above)` | tested by v6 (rejected, s_break 0.5): a 25 cm divider between spoon and plate; screening s3 (a box crossing the carry) did not break it either |
| A7 | The plate's top is clear: nothing lies where the spoon goes, so it can be lowered straight onto the plate centre. | l.73-75 `place(target)` | tested by v2 (rejected, s_break 0.5): with a box on the plate the long spoon still ends on top of the box (the success check is xy only) on half the seeds |
| A8 | Plate and spoon have their nominal shapes: the release height comes from their box heights and the plate is a shallow dish. | l.74 | open |
| A9 | Retreating and homing after the release do not disturb the spoon. | l.75-76 | open |
| A10 | Appearance does not matter: perception is pose-based (colour, lighting and camera pose do not change detections). | all `detect` calls | not a weakness of this program; a variation on these axes cannot pass the break gate |
| A11 | The idle arm stays at home and never gets in the way. | l.47 single-arm plan | open |
