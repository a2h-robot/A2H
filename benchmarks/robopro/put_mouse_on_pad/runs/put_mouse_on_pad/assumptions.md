# Assumptions of the put_mouse_on_pad pipeline

Pipeline: `runs/put_mouse_on_pad/pipeline.py` (phase 1, s0 = 1.0). Line numbers refer to that version.

| # | Assumption about the scene | Relied on at |
|---|---|---|
| A1 | The mouse is on the side of the arm that picks it up: the arm is chosen once from the sign of the mouse's x, and the same arm must reach the pad. | `run`, l. 94 |
| A2 | Nothing stands next to the mouse, so a grasp from above (fingers beside its long sides) is free; the only fallback is the API's other grasp candidates. | `pick`, l. 50-51 |
| A3 | The mouse stays where it was estimated while the arm approaches: one estimate per grasp attempt. | `pick`, l. 49, 56 |
| A4 | The pad stays where it was estimated at the start: it is estimated once and never re-checked before the release. | `run`, l. 91 |
| A5 | The space above the pad is free: the mouse is lowered to the pad top (DROP_MARGIN above it) without looking for anything lying on the pad. | `put_on_pad`, l. 76 |
| A6 | The mouse lies flat and right side up: its bottom is taken as the box centre minus half its vertical extent, and it is grasped from above. | `world_height`, l. 30-39; `pick`, l. 50 |
| A7 | The mouse is light and grips well: holding is checked once after the lift, never during the carry. | `pick`, l. 52-53 |
| A8 | The mouse does not move in the hand during the carry: the hand-to-mouse offset is measured once before the carry. | `put_on_pad`, l. 65, 69 |
| A9 | The way from the mouse to the pad is free for the held mouse: the planner avoids obstacles with the arm only, the held mouse is not part of its model. | `put_on_pad`, l. 69 |
| A10 | The pad is reachable from above with the grasp's hand orientation or one rotated about the vertical by at most 1.6 rad. | `CARRY_YAWS`, l. 17; l. 66-69 |
| A11 | The released mouse stays where it was put: the gripper opens and backs out with no check of the end position. | `put_on_pad`, l. 81-84 |
| A12 | The mouse is narrower than the gripper opening across the axis the API grasps it on. | `pick` (API grasp candidates) |
| A13 | The pad is a flat static board at table height; its top is its box centre plus half its vertical extent. | `run`, l. 92 |
