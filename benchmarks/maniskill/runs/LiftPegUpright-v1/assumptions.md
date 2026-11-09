# Assumptions of pipeline.py (round 0, s0 = 1.0)

Line numbers refer to `pipeline.py` as committed after Phase 1.

| # | Assumption about the scene | Category | Relies on it |
|---|---|---|---|
| A1 | The peg lies flat: its long axis is horizontal, so a straight top-down grasp across its width works. | orientation | L97-99 (`u[2] = 0`), L104 (approach = -z), L113 (`g[2] = p[2]`) |
| A2 | The peg rests on the face that puts its local y axis up (the default face), so after tipping it the gripper points along ±x and the success check's Euler decomposition is away from gimbal lock. | orientation | L131-145 (peg y forced along ±x, score prefers gripper pointing +x) |
| A3 | The peg is inside the default ±0.1 m square in front of the robot, so a horizontal, forward-pointing hand can reach the release pose **directly above the pick location**. | object position / reach | L148 (`place_xy = p[:2]`), L152-160 (only the height above the release pose is varied) |
| A4 | The space around the pick location is free: nothing blocks the vertical approach, the lift, or the horizontal hand and forearm while the peg is lowered upright at the pick location. No collision objects are passed to the planner. | obstacle | L115-124, L155-172 (`move_to` without `avoid=`) |
| A5 | The peg does not move between the averaged detection and the grasp; one detection is enough. | object motion | L93 (`mean_pose` once per attempt), L112-118 (grasp at that pose) |
| A6 | The peg does not slip in a full-strength grasp while it is swung upright (enough friction for its mass), so `T_tcp_peg` measured after the lift is still valid at release. | dynamics | L119 (`close_gripper()`), L127-129 (measured once), L155-164 (open-loop use) |
| A7 | The only object named "peg" is the target; there is no other similar object whose detection could be confused. Detection is unbiased noise, so averaging 60 calls gives ~1.3 mm / 0.4 deg. | perception / distractor | L12, L45-60 (`mean_pose`) |
| A8 | The table is flat and at z = 0, and the peg half length is 0.12 m (release height). | geometry | L15-16, L155, L163 |
| A9 | The peg is light enough (0.6 kg) to be held off-centre at `GRASP_OFFSET` without rotating in the fingers. | dynamics | L13, L112, L119 |
| A10 | The release pose is free, so retreating 8 cm straight back and 10 cm up does not hit anything or knock the peg. | obstacle | L167-172 |
| A11 | Camera, lighting and colour do not matter (perception is pose-based in this backend). | appearance / camera | (no line: `camera_frame` is never used) |

## Attack order (most likely on a real robot first)
1. A3: the peg is farther out or off to the side than the default region. Tried before round 1 (delta [0.10, 0.12] -> 1.0, delta [0, 0.26] -> 0.9): does not break the program, so not a weakness.
1b. A4 (round 1, v1): a wall between robot and peg -> s_break 0.4. Fixed.
2. A4: clutter next to the peg occupies the space the hand needs while placing.
3. A5: the peg is bumped or slides before it is grasped.
4. A6 / A9: heavier or slipperier peg.
5. A1 / A2: the peg rests in another orientation.

# Round 2 re-read (pipeline after round 1, commit 3f9a14e)

| # | Assumption | Category | Relies on it |
|---|---|---|---|
| A4' | Obstacles are static and known once per attempt; the peg itself is not an obstacle the planner sees while held. | obstacle | L121 (`scene_obstacles` once), L144-215 (`avoid=` fixed list) |
| A5 | The peg does not move between the averaged detection and the grasp closing (detection once per attempt; grasp at that spot; a miss only triggers a full retry, at most 3). | object motion | L109-116 (3 attempts), L122 (detect once), L141-150 (grasp at stale pose) |
| A6 | The peg does not slip in the grasp; it is re-measured once after the swing, but not during the descent to the release height. | dynamics | L148, L199-206 |
| A12 | The release spot does not need to be far from the pick spot (candidates within -0.10..+0.20 m in x and ±0.15 m in y). | object position / obstacle | L84-97 |

Round 2 target: A5 — real objects get nudged or are on moving surfaces; the program never checks that the peg is still where it was detected.
| A13 | There is room for both open fingers at the single fixed grasp point (centre + 3 cm); if the planner rejects it there is no other grasp point to try. | distractor / clutter | L141-147 (`g = p + GRASP_OFFSET * u`, one candidate) |

Screened before round 2 and **not** breaking (frozen pipeline 1af2ab2): peg sliding at nominal 0.03/0.2/0.1 m/s (1.0/1.0/0.9), mass 2-3 kg with mu 0.3-0.5 (0.6-0.9), mu 0.1 (1.0), yaw 45/90 deg (1.0/1.0), rolled onto another face (0.7). Round 2 (v2): a block 1 cm beside the peg at the grasp point -> s_break 0.0. Fixed.
| A14 | Free space above the workspace: lift to "above every obstacle top"; vertical hand needs ~0.3 m headroom; plans that the planner accepts actually arrive; held and placed objects need no planning. | obstacle (overhead) | lift `top = max(LIFT_Z, tops + 0.08)`, grasp `move_to` without arrival check, retreat without the peg in `avoid` |

Round 3 (v3, overhead shelf at 0.39 m): s_break 0.0, oracle 0.0; recovered only to 0.6 in 6 repairs -> marked `unsolved`. The robustness repairs were kept (base/v1/v2 = 1.0).

# Round 4 re-read (pipeline after round 3, commit 76900b0)
| A4' | Obstacles are static: read once at the start of an attempt and used for a release ~10 s later; the held peg hovers 3-6 cm above the table before release. | obstacle / object motion | `attempt`: `avoid, boxes = scene_obstacles(api)` once; release spot chosen before the swing |
| A15 | Retreat moves may treat the standing peg as an obstacle from the start (fingers are inside its inflated box). | obstacle | retreat block |

Round 4 (v4): a box slides into the release area after the pick -> s_break 0.1 (gate s0 0.7). Fixed: 1.0.

# Round 5 re-read (pipeline after round 4, commit 81615e7)
| # | Assumption | Category | Relies on it |
|---|---|---|---|
| A16 | The peg can be grasped where it lies: some grasp offset along it leaves room for both open fingers. If none does, there is nothing else to try. | obstacle / clutter | `attempt`: grasp loop over `GRASP_PITCHES` x `GRASP_OFFSETS`, then `return "unreachable"` |
| A6/A9 | The grasp holds a heavy peg without pivoting in the fingers (off-centre grasp at `GRASP_OFFSETS[0]` = 3 cm). | dynamics | `GRASP_OFFSETS`, lift and swing |
| A1 | The peg lies flat (`u[2] = 0`). | orientation | `attempt`: `u[2] = 0.0` |

Screened before round 5 (frozen pipeline 81615e7, 10 seeds) and **not** breaking: v1 re-check 0.9 / 0.8 (seeds 0-9 / 10-19); mass 2.5-3.5 kg with mu 0.3-0.5 → 1.0; heavy and slippery (3 kg with mu 0.1-0.15, 2.5 kg with mu 0.2, 1.5 kg with mu 0.08) → 1.0, 2 kg with mu 0.1 → 0.7; mass 4 kg with mu 0.2 → 0.6; rolled onto another face → 0.9; leaning on a block → 0.8; peg 0.20 m closer to the robot → 1.0; peg shifted [-0.25, +0.12] → 0.7.
Breaking but not chosen: mass 4.5-6 kg (0.0-0.3; the peg pivots in the fingers and drops, but this is above a real arm's 3 kg payload); peg far out on the diagonal [+0.22, -0.15] (0.1; partly beyond the 0.85 m reach check).
Round 5 target: A16. A peg lying against a low wall along its whole length (the rim of a tray or bin) leaves no room for a finger anywhere, which is common in real clutter.

# Round 6 re-read (pipeline after round 5, commit 1e33311)
| # | Assumption | Category | Relies on it |
|---|---|---|---|
| A8' | The peg rests on the table: the open-finger volume is checked from z = 0 up, and the slide check assumes the peg sits on the surface it slides on. | object position / support | `fingers_clear`: `lo_f[2] = 0.0`; `peg_path_clear` |
| A17 | Nothing lies on top of the peg. | clutter | grasp loop (fingers and palm from above) |
| A18 | A push along the peg's axis towards the robot is free (the corner case). | obstacle | `slide_to_graspable` |

Screened before round 6 (frozen pipeline 1e33311, 10 seeds): peg nudged sideways (impulse 0.6 m/s at t = 2.5 s) → 0.8 (not breaking); board lying on top of the peg → 0.0; peg on a 4 cm platform → 0.0; peg in a corner of two low walls → 0.0 (close to v5's assumption); peg inside a tray with 4 cm rims → 0.0 (probably not graspable at all: a finger would touch the rim).
Round 6 target: A8'. Objects are often picked from a box, shelf or platform rather than from the table the robot stands at.
