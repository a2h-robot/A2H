# Assumptions of pipeline.py (open_the_middle_drawer_of_the_cabinet)

Line numbers refer to the Phase-1 pipeline (commit c828fd8). Status column is updated per round.

| # | Category | Assumption | Code that relies on it | Status |
|---|---|---|---|---|
| A1 | dynamics / contact | The drawer slides freely: a single slow pull at constant speed moves it, the pinch grasp on the handle does not slip, and the pulling force stays far below the wrist force limit. | l.125 one `move_to(..., linear=True, speed="slow")` for the whole travel; l.103 at most 3 attempts | attacked v1 (accepted, recovered) |
| A2 | position | The cabinet stands where the arm can reach the handle with the gripper pitched down 5-45 deg and pulled 15 cm toward the robot's left (+y). | l.78 `TILTS_DEG`, l.86-87 (orientations), l.124-125 pull target | attacked v5 (accepted, recovered) |
| A3 | orientation | The cabinet is upright and its drawer slides along a horizontal axis of the drawer box; the pull direction is the drawer axis best aligned with the body->drawer bbox offset. | l.63-70 | attacked v5 (accepted, recovered) |
| A4 | orientation | The approach to the handle comes from the open side of the drawer only (no yaw alternatives, only pitch). Whatever direction the drawer faces, the space in front of and above the handle is free for the palm. | l.85-91, l.111-112 | attacked v4 (accepted, recovered) |
| A5 | geometry / appearance | The handle is a horizontal bar centred on the drawer front, at the drawer's mid-height, 0.8 cm deep, so that the drawer bbox extreme on the pull side minus 0.8 cm is the bar centre. | l.16, l.72-74 | |
| A6 | obstacles | Nothing fixed is in the pre-grasp/approach volume (10 cm back along the approach) or in the 15 cm pull path of the hand; low clutter (plate) is below the palm when pitched down. | l.111-114, l.125, l.129 | attacked v2 (accepted, recovered) |
| A7 | state | The drawer starts closed: travel is measured relative to the drawer pose at the start (pos0), and 15.2 cm of travel is the target. | l.100, l.104, l.124 | |
| A8 | motion | The scene is static: the cabinet does not move during the episode, so the handle pose measured once (or once per attempt) stays valid. | l.100, l.108 | attacked v3 (accepted, recovered) |
| A9 | perception | The drawer part and the cabinet body can be detected by name and their averaged poses (20 samples, ~2 mm noise) are accurate; appearance/lighting/camera do not matter because no image is used. | l.39-51, l.56-57 | |
| A10 | size | Drawer travel needed (> 14 cm) and handle thickness are fixed metric values independent of the cabinet's size. | l.14-16 | |
| A11 | contact | Only the cabinet is touched; touching the cabinet face with the upper finger while closing is harmless (a closed drawer cannot be pushed in further). | l.114 `allow_contact=[CAB]` | |
| A12 | gripper | Closing the gripper on the bar yields a grasp (the result of the grasp is never checked before pulling; success is checked only via the measured travel). | l.117-125 | fixed in round 1 (gripper_holding check) |
