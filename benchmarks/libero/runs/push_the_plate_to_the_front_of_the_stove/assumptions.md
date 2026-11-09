# Assumptions of pipeline.py (Phase-1 version, commit f7c6ac5)

Line numbers refer to the Phase-1 pipeline. "Attacked" marks assumptions that a round has already targeted.

## Where objects are
| # | Assumption | Code |
|---|---|---|
| A1 | The plate and the goal region can be found by their names `plate_1`, `main_table_stove_front_region`. | L15-16, `detect_avg` L30 |
| A2 | The plate is within reach and a pinch point on at least one rim segment is reachable top-down at table height. | `grasp_rim` L199-208, `pinch` L190 |
| A3 | Some point of the goal region (shrunk by 1.5 cm) keeps the plate disk clear of other objects; only obstacle footprints lower than the plate top + 1 cm matter. | `obstacle_rects` L44-55, `choose_target` L73-85 |
| A4 | The collision-aware planner finds a carry path for the hanging plate; if not, 3 cm higher is enough. | `try_move` L239-242 |

## How objects are oriented / shaped
| # | Assumption | Code |
|---|---|---|
| B1 | The plate lies flat (its yaw is the only rotation that matters). | `plate_yaw` L123-129, `rest_z` L250 |
| B2 | The rim is 8 flat segments, one every 45 deg in the plate frame; the middle of a segment is pinchable. | `rim_angles` L140 |
| B3 | **The plate has a fixed size**: the rim lip is at a hard-coded radius `RIM_R = 0.067` m from the centre and the pinch depth `GRASP_DEPTH = 0.005` m below the top suits its lip thickness. | L19-20, `pinch` L185 |
| B4 | The plate is round: its footprint radius is half the larger bbox side. | `run` L298 |

## What objects look like
| # | Assumption | Code |
|---|---|---|
| C1 | Perception is pose-based; colour/lighting/camera do not matter. Detection noise is zero-mean and independent per call, so averaging 30 calls removes it. | `detect_avg` L30, `N_AVG` L18 |

## Whether objects move
| # | Assumption | Code |
|---|---|---|
| D1 | The plate stays where it was detected between the detection and the grasp (one detection per attempt). | `transfer` L246-247 |
| D2 | Other objects do not move into the goal region. | `choose_target` once per attempt |

## What is between the gripper and the target
| # | Assumption | Code |
|---|---|---|
| E1 | Space above the pinch point is free for a vertical approach from 8 cm. | `pinch` L190-192 |
| E2 | Obstacles are known to `get_obstacles()` and the planner. | `obstacle_rects`, `move_to` |

## How objects respond to contact
| # | Assumption | Code |
|---|---|---|
| F1 | **The plate is light** (~11 g): a two-finger pinch on its thin lip can lift one edge 6 cm and carry it (far edge dragging) without slipping. | `LIFT` L21, `transfer` L253-269 |
| F2 | A hanging plate lands where a rigid "falls flat about its lowest edge" model predicts. | `predicted_landing` L222-230 |
| F3 | The plate is lifted iff its detected centre is 8 mm above its resting height (gripper_holding() flickers). | `lifted` L233-236 |
| F4 | The plate does not slide away after release (friction high enough). | `run` loop, `in_target` L88 |

## What the camera sees
| # | Assumption | Code |
|---|---|---|
| G1 | Not used: no image-based step; detections are always available (detect never returns None for the plate). | `detect_avg` L30-38 |

## Attacked so far
(updated per round in reflections.jsonl)
- Round 1 (v1, dynamics, UNSOLVED): F1 - the plate is light enough to be lifted by a lip pinch (0.4 kg breaks it; no recovery found in 6 repairs).
- Round 2 (v2, distractor, accepted): A3/D2 - part of the goal region is free. Fixed by clear_goal() (relocate a movable blocker).
- Rejected (did not break the frozen pipeline): plate scale 1.3 (B3), plate mass 0.15 kg (F1), white_bowl swap (B2/B3), a 12 cm wall in the carry path (A4), a 4.5 cm block in the goal (A3). The akita_black_bowl swap was invalid (name clash with the existing bowl).
- Round 3 (v3, distractor, accepted): the goal blocker introduced by clear_goal() fits the gripper. Fixed by push_away() for objects wider than the gripper span, plus a "plate followed the hand" check during carry (the height-based lifted() check is fooled when the plate rests on another object).
- Rejected: plate sliding 12 cm during the first approach (object_motion, D1) - the closed loop re-detects and recovers (0.9).

## Assumptions added by later repairs
| # | Assumption | Code |
|---|---|---|
| H1 | A blocker in the goal is a whole movable object listed by objects(); fixture parts are never moved. | `clear_goal` |
| H2 | A free table spot exists within 0.72 m of the base, away from the goal region and the carry corridor. | `free_spot` |
| H3 | An object wider than 7 cm can be pushed with the closed fingers at half its height without tipping. | `push_away` |
| H4 | The blocker can be moved before the plate; nothing moves back into the goal afterwards. | `run` |
- Round 4 (v4, distractor/clutter, accepted): E1/A2 - the space around the rim and along the carry is free. Fixed by declutter() (relocate small movable objects near the rim or in the corridor before the transfer). A 3-can version was rejected (0.6).
- Round 5 (v5, size 0.7, UNSOLVED): B3 - fixed pinch radius/depth; even with size-derived values a 1.3 cm lip is too low to pinch above the fingers' minimum clearance. Rejected before it: friction 0.3 (0.9), plate starting on the stove (1.0).
- Round 6 (v6, overhead board, UNSOLVED): E1 - free space above the rim for the whole 20 cm hand; no collision-free pinch pose exists. Probably physically unsolvable for this gripper (designer mistake).
