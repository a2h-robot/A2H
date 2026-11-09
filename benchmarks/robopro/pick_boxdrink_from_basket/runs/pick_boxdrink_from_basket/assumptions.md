# Assumptions of the phase-1 pipeline (pick_boxdrink_from_basket)

Line numbers refer to `pipeline.py` at the phase-1 commit (s0 = 1.0).

| # | Assumption | Code that relies on it |
|---|---|---|
| A1 | The box stands upright: its longest axis is the one closest to vertical, and the grasp heights and the final placement assume it. | `box_geometry` (L44-48), grasp points `bottom + frac * height` (L60-63), placement height (L107) |
| A2 | The box does not move between the detection at the pre-grasp pose and the closing of the fingers (one re-detection, then an open-loop straight approach). | L131-137 |
| A3 | There is free space on the robot's side of the box inside the basket: every grasp approach comes from the robot (headings 60-120 deg) and the fingers enter the gap between the box and the near wall. | `HEADINGS` (L24), `grasp_candidates` (L62-72) |
| A4 | A box face, not its diagonal, is across the fingers: the closing axis is fixed by the approach heading, not aligned to the box's faces, and the opening is the box's width along that axis + 3 cm (max 9 cm). | L68-72 |
| A5 | The box is light enough (0.1 kg) for one pinch at mid-height to lift and carry it without slipping or turning; the hold is checked once, right after closing. | L137-139 |
| A6 | The box sits within reach of the arm on its side; the arm is chosen by the sign of x only. | L149 |
| A7 | The basket is the only thing the box must clear: the lift height is the basket rim + 2 cm, and nothing above the basket blocks the straight vertical lift. | L158-167 |
| A8 | The straight line at carry height from above the basket to above the chosen spot is free (the planner does not see the held box). | L172-173 |
| A9 | There is free table space in a fixed region in front of the robot on the arm's side, and every obstacle there shows up in `get_obstacles()` as a box; bodies more than 25 cm above the table never conflict. | `free_spots` (L76-96) |
| A10 | The table is flat at the detected height, and a 1 cm drop leaves the box standing. | L51-53, L107, L115 |
| A11 | The box does not slip in the fingers during the lift, so its bottom height is known from the grasp-time detection. | L161 |
| A12 | Nothing in the scene moves on its own (basket, box, other bodies are static until touched). | whole pipeline (single detections, open-loop motions) |
| A13 | The basket's rim height is uniform and given by its bounding box top. | L159 |
| A14 | Appearance (color, lighting, camera) is irrelevant: perception is pose-based. | `averaged` (L28-35) |

Most likely to be violated on a real robot (ranked): A3 (baskets usually hold several items), A4 (objects
lie at arbitrary yaw), A5 (a full carton weighs ~0.5-1 kg), A2/A12 (objects shift or are bumped), A7/A8
(shelves and other objects around the basket), A1 (items lying down).
