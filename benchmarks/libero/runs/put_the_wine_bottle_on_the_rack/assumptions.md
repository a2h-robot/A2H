# Assumptions of the Phase-1 pipeline (pipeline.py @ phase 1)

Line numbers refer to the Phase-1 pipeline. "Attacked" marks the round that targeted it.

| # | Assumption about the scene | Code that relies on it | Attacked |
|---|---|---|---|
| A1 | The rack's shelf runs along world x: a bottle lying on it has its axis along +x, so the wrist pitches about world y and the bottle bottom is offset along world +x. | L111 `Rotation.from_rotvec(-TILT * [0,1,0]) @ R_TOP`, L112 `X * BOTTOM_X_OFFSET`, L61 fingers close along world y | v1 (broke) |
| A2 | The bottle stands upright: its top is the neck, a top-down grasp at `top - 0.03` pinches the neck, and the in-hand bottle axis is the gripper approach axis. | L69-70 (`top - NECK_DEPTH`), L61 `R_TOP`, L111 (tilt assumes axis = approach) | v2 (broke) |
| A3 | The bottle is light enough that a pinch on the thin neck holds it against the tipping torque about the finger-closing axis when tilted 60 deg. | L61 (closing axis = pitch axis), L21 neck grasp, L111 | v2a, v3a rejected (0.6 / 1.2 kg did not break) |
| A4 | The bottle does not move between detection and grasp (one averaged detection, then an open-loop approach). | L65-76 | v3b rejected (motion tipped the bottle; retry + lying branch recovered) |
| A5 | Nothing is above or around the bottle neck: the 8 cm vertical pre-grasp and straight descent are free. | L73, L76 | v3 (side clutter, broke), v5 (overhead board, broke) |
| A6 | The bottle has the nominal size: radius 0.02 m, the neck lies within 3 cm of the top. | L21-22, L112 | v5a rejected (scale 0.75 did not break) |
| A7 | The release spot on the shelf (3.5 cm up-slope of the lip, bottom 3 cm along +x of the region centre) is free; no object already lies there. | L112 | |
| A8 | The rack's top-region frame encodes the shelf: its -y axis is the shelf normal, its z axis the up-slope direction, its centre sits at the lip. | L87-96 | |
| A9 | The rack is placed so that the hand can approach from the robot side (-x) and the bottle bottom can point away (+x); both reachable. | L111-112 (fixed sign of tilt and X offset), L117-122 | v4 (broke) |
| A10 | After release the bottle lies down and rolls against the lip and stays (friction, mass, lip height as nominal). | L112, L126 | |
| A11 | `gripper_holding()` after closing means the neck is held (not a fixed object, not a slipping grasp); no re-check after lift or before release. | L81, L103-126 | |
| A12 | Perception is pose-based and independent of appearance: color, lighting and camera pose do not matter; detections are unbiased, so averaging 10 removes most of the noise. | L47-55 | |
| A13 | The gripper path from the lift pose to the pre-place pose above the rack is free (planner) and the region is reachable at 0.08-0.16 m above the release pose. | L117-122 | v5b rejected (wall at rack end did not break) |
| A14 | The robot base is the only arm; the hand-to-bottle offset measured once after lifting stays valid during the 60 deg wrist pitch (no slip). | L106-113 | |
