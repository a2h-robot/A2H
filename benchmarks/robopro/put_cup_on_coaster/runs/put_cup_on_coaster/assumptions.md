# Assumptions of the put_cup_on_coaster pipeline

Line numbers refer to the round-0 pipeline (commit "phase 1 pipeline, s0=1.0"). Each assumption is followed by the
code that relies on it. The status column is updated as rounds attack them.

| # | Assumption | Code that relies on it | Status |
|---|---|---|---|
| A1 | **Free space around the coaster.** The hand, tilted 45-60 deg and offset ~10 cm from the cup towards the arm, can hover 5 cm above the coaster and descend straight down. Nothing stands next to the coaster. | `carry_and_lower` L107-L124: hover goal, translation-only alignment, straight descent; only the hand yaw is searched, never its pitch | screened (round 1, walls around the coaster): 0.8, no break. **Round 5 (v6, movable boxes packed around the coaster): broke 1.0 -> 0.3; fixed** (clear the destination before the pick; narrow release; time-aware retries). v5 rejected by the validity gate (a box inside the furniture on 1 seed of 10) |
| A2 | **Free space around the cup.** A tilted approach line (pre-grasp 10 cm back) to the upper body of the cup is collision-free for at least one yaw. | `grasp_body` L64-L79 | **round 2 (v2, four movable boxes packed around the cup): broke 1.0 -> 0.0; fixed** (clear movable neighbours first). Fixed walls around the cup screened 0.0, not gated (no room for a finger) |
| A3 | **Static coaster.** The coaster does not move between its estimate at the start of placing and the release. | `place_on_coaster` L147-L148 (one estimate); `carry_and_lower` L107 goal fixed for the whole carry and descent | screened (round 1, coaster slides mid-carry): 0.9, no break (the end check re-places); round 7: coaster pulled 12 cm during the carry 1.0, or late 1.0 |
| A4 | **Static cup before the grasp.** The cup stays where it was estimated while the arm approaches. | `pick` L90 (estimate), then `grasp_body` approaches that point | **round 3 (v3, cup slides 9 cm on a slick table during the approach): broke 0.9 -> 0.0; fixed** (wait until still; re-check before committing to the approach) |
| A5 | **Upright, cup-sized object.** The cup stands upright, fits across the 9 cm gripper, and is about as tall as wide; the grasp aims 25% of its height above the centre. | `grasp_body` L66 (grasp point), L74 (open fully); `bottom_centre` L44 (most vertical axis = cup axis) | **round 1 (v1, cup 1.5x): broke 1.0 -> 0.0; fixed** (grasp chosen by width; tilted rim pinch). Mug with handle screened 0.5, then 1.0 after round 1. **Round 4 (v4, cup lying on its side): broke 1.0 -> 0.0; UNSOLVED** (best 0.7 in 6 repairs; reverted). **Round 7 (v8, mug-size cup boxed in): broke 1.0 -> 0.3; fixed** (cup width measured in the object frame). Round 7 also screened an espresso-size cup 0.7 and a big mug 1.0 |
| A6 | **Light, grippy cup.** A pinch on the upper body holds it through the lift and the carry without slipping or swinging much. | `pick` L94-L95 (lift, then one holding check); `carry_and_lower` measures the hold offset once (L109) | screened (round 1, cup 0.6 kg): 1.0, no break |
| A7 | **The coaster stays put when the cup lands.** It does not slide or tilt on contact (friction, mass). | `carry_and_lower` L120-L121 allows contact with the coaster and descends to 2 mm above its top | screened (round 3, light slick coaster) 1.0; (round 7, 20 g coaster, friction 0.05, on a glossy table) 1.0 |
| A8 | **Flat coaster on the table.** Its top is the top of its bounding box, and the table is just below it (used for the hand-over relay). | `place_on_coaster` L148, L151 | screened (round 7, coaster on a 4 cm book) 1.0 |
| A9 | **One cup and one coaster.** `detect("cup")` and `detect("coaster")` always return the intended objects. | `estimate` L22-L26, everywhere | |
| A10 | **Reachability.** At least one arm reaches both objects, or both arms reach a midpoint (relay). | `choose_arm` L162-L168; `place_on_coaster` relay L150-L158 | screened (round 4, coaster at the back): 0.8, no break. **Round 6 (v7, cup at the back / coaster at the front edge): broke 1.0 -> 0.5; UNSOLVED** (variation flawed: in 5 of 10 seeds the cup lies inside the bookcase footprint) |
| A11 | **Perception is only mildly noisy** (~1 cm, 3 deg per call); 8-sample averages are accurate to a few mm. | `N_DET` L8, `ALIGN_TOL` L13, `PLACE_TOL` L11 | |
| A12 | **Appearance and viewpoint do not matter.** Perception is pose-based, so colour, lighting and camera changes cannot affect it. | (no code: the pipeline never uses images) | |
| A13 | **Time budget.** The whole pick-and-place, including retries and relays, fits in the 120 s episode budget. | `run` L173-L180 (up to 3 attempts), `pick` tries both arms | |

**Ranking by how likely a real robot is to meet each violation:**
1. Clutter next to the target (A1, A2). Coasters sit between laptops, books and pen holders.
2. Other cups and mugs: a different size or shape (A5) or a heavier, fuller cup (A6).
3. Things that move (A3, A4).
4. Placement-surface dynamics (A7, A8).

Appearance (A12) cannot break a pose-based pipeline.
