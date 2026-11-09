# Assumptions of pipeline.py (drop_ball_hole)

Line references are to the Phase-1 pipeline (commit 30315e3); later rounds note which ones they attacked.

| # | Category | Assumption | Code that relies on it |
|---|---|---|---|
| A1 | motion | The ball lies still on the table from the moment it is detected until the gripper closes on it. | `run`: `m = mean_state(api, "ball")` then `api.grasp(..., state=st)` (one averaged snapshot, no velocity) |
| A2 | motion | The ball stays in the gripper during lift and carry; nothing knocks it out. | `run`: `if not api.gripper_holding(arm): DEBUG[...]` (only logged, no recovery) |
| A3 | motion | The platform spins at a constant angular velocity for the whole episode (yaw = a + w t). | `Platform.sample`: least-squares line fit; `next_pass` extrapolates it seconds ahead |
| A4 | position | The platform centre and the holes' offsets in the platform frame are fixed. | `Platform.center`, `Platform.radius`, `Platform.phi_local` (averaged over all samples) |
| A5 | position | The ball starts on the arm's side of the sorter and the arm on that side can reach both it and the release point `P = center + side * [r, 0]`. | `arm = "left" if ball["p"][0] < plat.center[0]`; `P = plat.center + side * np.array([plat.radius, 0.0])` |
| A6 | obstacle | Nothing stands between the ball, the lift path and the point above the hole orbit; motions pass no `avoid`. | every `api.move_by(...)` and `api.grasp(...)` in `run` |
| A7 | size / appearance | The real hole is the one wider than the ball, and only two holes exist; perception gives their true size. | `fits = [h for h in holes if ext[h] > 2 * r_ball]`; `target = max(...)` |
| A8 | size | The ball's size from its box and the platform top from the sorter's box are right. | `r_ball = ...ball["ext"]...`; `top = sorter["c"][2] + sorter["ext"][2] / 2` |
| A9 | dynamics | The release delay is a constant: the ball leaves the opening fingers 0.51 s after `open_gripper`, then falls about 3 cm (LEAD = 0.55 s). This was calibrated on this ball's mass and friction. | `LEAD`, `RELEASE_CLEAR`; `t_cmd = t_land - LEAD` |
| A10 | dynamics | The ball falls straight down when released (it is not dragged sideways by the fingers or the platform). | `RELEASE_CLEAR = 0.03` (release in the air); `next_pass(b, ...)` uses the ball's xy at release |
| A11 | dynamics | Contact with the gripper while approaching does not push the ball away (a sphere rolls freely). | `api.grasp(..., pre_dis=0.08)`; retry loop re-detects but assumes the ball is still within reach |
| A12 | camera | No camera is used; perception is `detect()` only, averaged 24 times (noise is independent per call). | `mean_state` (`N_AVG = 24`) |
| A13 | timing | The arm reaches the release pose with time to spare before the latest useful pass; any pass of the hole is as good as another. | `t_land = plat.next_pass(b, api.time() + LEAD + 0.05)` |
| A14 | other objects | Nothing else is on the platform or the table near the ball (no distractors to confuse `detect("ball")`, nothing riding on the platform). | `detect("ball")` by name; release point chosen without checking the platform surface |
