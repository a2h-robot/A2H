# Assumptions of the PlugCharger-v1 pipeline

Line numbers refer to `pipeline.py` as committed after Phase 1 (s0 = 1.0). The status column is
updated as rounds attack them.

| # | Category | Assumption | Code that relies on it | Status |
|---|---|---|---|---|
| A1 | orientation | The charger lies flat on the table (only yaw varies), so its y axis is horizontal and a top-down grasp closes across the base's 3 cm width. If the charger is on its side, `closing[2] = 0` leaves a near-zero vector and the grasp frame is NaN/garbage. | L39-L47 (`closing = R[:, 1]; closing[2] = 0; closing /= norm`) | round 1 (v1): broken, not recovered (unsolved) |
| A2 | position | The charger base center is the grasp point: base half-size is the nominal one, and the base is a box shorter than the finger length, so grasping at its center is safe. | L15 (`BASE_HALF`), L47 | tested (1 kg block 2 mm beside the base): not broken after round 3 (s_break 0.9, gate rejected) |
| A3 | motion | The charger does not move between the one-shot perception at the start and the grasp (no re-detection before closing the gripper). | L78-L86 | round 2 (v2): broken (0.4), fixed: re-detect at rest at pre-grasp, verify in-hand pose |
| A4 | motion | Both objects are static while 20 000 detections are averaged, so averaging removes only noise (a moving object would be averaged into a stale pose). | L27-L29 (`frames`), L78, L98 | |
| A5 | motion | The receptacle (goal) does not move after it is measured at the start; it is only re-measured if insertion fails. | L79, L90-L91, L111 | |
| A6 | obstacles | The path from pre-grasp (5 cm above along the approach axis) down to the grasp is free. | L83-L85 (`move_to` without `avoid`) | |
| A7 | obstacles | Lifting 5 cm and moving straight to 4 cm in front of the receptacle is collision free; nothing stands between the charger and the receptacle, or around the receptacle face. | L87-L88, L102 (`move_to` without `avoid`) | round 3 (v3): broken (0.0), fixed: transfer over the highest obstacle with avoid |
| A8 | obstacles | Nothing touches the charger, gripper or arm during the servoed approach and insertion (all servo moves ignore obstacles). | L106, L108, L110, L113 | |
| A9 | dynamics | The charger does not slip or rotate in the fingers between the in-hand estimate and the end of insertion; `hold` is estimated once per attempt and reused for every insertion step. | L105-L113 | round 4 (v4): heavy slippery part broke it (0.0) through arm sag, not slip; fixed: learned command offset carried into insertion |
| A10 | dynamics | The charger is light enough and its friction high enough for a full-strength grip to carry it, and the pegs slide into the 0.5 mm clearance without sticking. | L86, L108, L113 | |
| A11 | reachability | The receptacle faces roughly toward the robot (yaw ~ pi +- 22.5 deg), so an approach from 4 cm in front of it along the goal x axis is reachable with the grasp the pipeline chose. | L102-L103 | tested (receptacle +70 deg): not broken (s_break 1.0, gate rejected) |
| A12 | reachability | The grasp chosen from the initial charger yaw (closing aligned with the current TCP y axis, plus a 15 deg pitch) leaves a wrist configuration that can still reach the insertion pose. | L43-L48, L102 | round 5 (v5): reversed charger broke it (0.0), fixed: turn on the table in <=90 deg steps, reachable-grasp fallback |
| A13 | perception | Detection noise is zero mean and independent per call, so averaging N calls shrinks the error by sqrt(N). A biased detector would not be fixed. | L27-L29 | |
| A14 | perception | Objects are found by name (`task_frames`), so color, lighting, camera pose and look-alike distractors do not affect the pipeline. | L28, L98 | |
| A15 | control | `move_to` failures are not fatal except for the pre-grasp move; a failed transfer or servo move is not detected (only the residual at the insertion check is). | L83, L85, L88, L102 | |
| A16 | control | The arm is not disturbed during a move; the final residual measured through `tcp_pose()` is the planner's own endpoint error, which translation-only re-approaches remove. | `servo_to` L51-L74 | |
