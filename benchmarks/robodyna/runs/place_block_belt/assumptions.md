# Assumptions of pipeline.py (place_block_belt)

Line numbers refer to `pipeline.py` as committed after Phase 1 (e125dec). Attacked = used by a variation.

## Where objects are
| # | Assumption | Code |
|---|---|---|
| A1 | The block starts on the load side of the belt, so the belt flows from the block's side towards the far end (flow direction is inferred from layout, never observed). | l.315 `flow = 1.0 if belt_c[0] > block.position[0] ...` |
| A2 | The arm on the block's side of x = 0 can reach both the block and the release point. | l.316 `arm_name = "left" if block.position[0] < 0 ...` |
| A3 | The exit end of the belt is the bbox edge in the flow direction, and the block leaves the belt there. | l.318, l.393 |
| A4 | The blocker sits on one side of the belt centre line, so the far half-lane is clear; with no blocker either side is fine. | l.321-325 |
| A5 | The release point (3.5 cm before the place line, 8 cm inside the belt edge) is free and reachable. | l.325-326, l.377 |
| A6 | The table top is the bottom of the belt support. | l.311-313 |
| A7 | Nothing but the belt, its support and the blocker is near the carry path; the held block itself is not modelled in the planner. | l.378-379 |

## How objects are oriented / shaped
| # | Assumption | Code |
|---|---|---|
| O1 | The block rests level on the table (its most vertical axis is vertical); only its yaw varies, and a top-down grasp across two faces works. | l.47-56 (`world_extents`), l.284-296, l.341-345 |
| O2 | The block is small enough that the fingers, grasping its upper part (tcp >= 5.3 cm above the table), hold it. | l.344 |
| O3 | The block does not slip or rotate in the hand after the grasp: its height offset under the tcp is fixed at grasp time. | l.366 |
| O4 | The belt and its support are level boxes. | l.307-313 |

## What objects look like (camera)
| # | Assumption | Code |
|---|---|---|
| C1 | The belt top is darker than gray 110 and the table around it is brighter (fixed absolute thresholds). | `speed_from_frames` `dark_thr=110`, `mot_thr=12` |
| C2 | The belt top has a visible moving pattern whose image shift equals the belt surface speed. | `speed_from_frames` pass 2 |
| C3 | The belt's two end edges are visible as straight lines in a majority of rows/frames (arms, block, bowl only occlude part). | `speed_from_frames` pass 1, `ransac_line` |
| C4 | The camera and the arms stay still during the 4 s speed measurement, and nothing else on the belt moves. | l.331, `belt_speed_from_camera` |
| C5 | The bowl, blocker and place line are found by name with ~1 cm noise. | l.303-306, `BowlTracker.sample` |

## Whether objects move
| # | Assumption | Code |
|---|---|---|
| M1 | The block does not move between the detection and the grasp (detected once per attempt). | l.340-352 |
| M2 | The bowl moves back and forth at constant speed between two fixed ends (triangle wave), the same before and after the release. | `BowlTracker.tri`, `fit_model`, l.396-409 |
| M3 | The belt speed is constant from the measurement (t < 5 s) until the block reaches the end (t ~ 20 s). | l.394 |
| M4 | The gripper releases the block 0.78 s after the open command and the belt picks it up 0.05 s later. | l.18-19, l.395 |
| M5 | The static parts (belt, line, blocker) do not move; they are detected once. | l.303-305 |

## What is between the gripper and the target
| # | Assumption | Code |
|---|---|---|
| P1 | The straight-down approach to the block is free. | l.352 |
| P2 | The vertical lift and the carry at 7 cm above the belt are free except for the listed obstacles. | l.370, l.379 |
| P3 | The vertical descent onto the belt at the release point is free. | l.386 |
| P4 | Lifting the open gripper 10 cm straight up after release is free and does not touch the block. | l.419 |

## How objects respond to contact
| # | Assumption | Code |
|---|---|---|
| R1 | Closing on the block holds it (one of 3 attempts), and `gripper_holding` is truthful. | l.355-360 |
| R2 | The block falls from the belt end into the bowl when the bowl is within 4.5 cm of the lane (catch radius). | l.21, l.400 |
| R3 | The block, set down 4 mm above the belt, lands upright and is carried at belt speed. | l.25, l.385 |
