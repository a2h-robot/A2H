# GAH training summary: open_the_middle_drawer_of_the_cabinet (LIBERO, pilot run 2)

Goal: "open the middle drawer of the cabinet". TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6.
Stopped because 5 variations were accepted (all five rounds produced `novel: true`).

## Phase 1

Pipeline: detect the middle drawer and cabinet body (20-sample averaged detections), infer the pull
direction from the body->drawer bbox offset, put the handle at the drawer bbox extreme on that side,
pinch the horizontal bar with the gripper approaching against the pull direction pitched down
(the palm stays above the plate in front of the drawer), pull 15.2 cm with a linear move, measure
the travel by detection and retry. First attempt (horizontal approach) scored 0.2: the palm hit the
plate; pitching the approach down gave **s0 = 1.0**.

## Rounds

| Round | Variation | Axis | Assumption attacked | s_break | s_recovered (variation) | Repairs | Result |
|---|---|---|---|---|---|---|---|
| 1 | v1: drawer joint damping 3000 N*s/m | dynamics | drawer slides freely; one constant-speed pull suffices | 0.0 | 1.0 | 1 | accepted, recovered |
| 2 | v2: tall movable box where the hand ends up after the pull | distractor | nothing in the volume the hand/arm sweeps | 0.0 | 1.0 | 5 | accepted, recovered |
| 3 | v3: box slides into the work volume 1 s after the start | object_motion | scene is static after the first look | 0.0 | 1.0 | 2 | accepted, recovered |
| 4 | v4: fixed post just beyond the opened drawer front, on the pull axis | obstacle | space straight in front of the handle is free (approach only varies in pitch) | 0.0 | 0.9 | 3 | accepted, recovered |
| 5 | v5: cabinet turned 90 deg to face the robot, moved right (plate moved off its footprint) | orientation | cabinet stands where it usually does, facing sideways | 0.0 | 0.9 | 4 | accepted, recovered |

No variation was rejected by the gate and none was marked unsolved. Final `eval.py --all`
(10 seeds each): base 1.0, v1 1.0, v2 1.0, v3 1.0, v4 0.9, v5 0.9.

### Axes that broke the program
All five attacked axes broke it completely (s_break = 0.0 each): dynamics (stiff drawer),
distractor (movable clutter in the sweep), object_motion (clutter arriving after the first look),
obstacle (fixed clutter in the sweep), orientation/position (furniture in an unusual pose).

### Axes not attacked, and why
- color, lighting, camera: perception in this backend is pose-based and the pipeline never uses
  images, so these cannot break it (SCENE_EDITS.md says so); attacking them would only test the gate.
- size (`object_scale` of the cabinet): the slide joint range is not scaled with the mesh, so a
  scaled cabinet gives a physically odd drawer; judged less informative than the five above.
- A5 (handle shape/position on the front) and A7 (drawer starts closed) cannot be expressed with
  the available scene edits.

## Rules found (from reflections.jsonl)

1. When moving an object through contact, command small increments relative to the current measured
   pose and verify after each that the object itself moved and is still held; never infer object
   motion from the robot's own motion status.
2. Before executing a contact motion, derive the full volume the robot will sweep (hand, palm and
   forearm, over the whole motion), and if a movable object intrudes, relocate it to a spot outside
   that volume and verify it moved, instead of hoping the planner finds a way around.
3. Re-perceive before every attempt and every phase of a multi-step manipulation, derive safety
   volumes from fixed task references rather than from state that your own actions change, and treat
   a refused or blocked motion as a sign the scene changed, not as a reason to repeat the same motion.
4. When something is in the way, first test whether it can be moved (and remember the answer); for
   what cannot be moved, search over the full set of approach directions and check the whole
   hand-and-wrist sweep of the planned motion against it before committing, and always retreat along
   the direction you came in.
5. Before acting on an object, check the action's preconditions against the gripper's own geometry
   (opening width, finger length and reach) and the arm's reach, choose a grasp mode that fits (body,
   edge, or none), and bound the tries per object with memory of what failed, so a strategy tuned in
   one layout degrades into planning around rather than burning the time budget in another.

Cross-cutting observation: in four of five rounds the repair revealed a second, hidden weakness
(single noisy detections used for grasps, safety volumes computed from moving state, a fixed
retreat direction, gripper-geometry limits). Variations that move clutter or furniture tend to
exercise the whole recovery chain, not just the assumption they target.

## Backend issues
None found in this run. No edits to api.py, motion.py, gah_env.py, scene_edits.py, tools/ or the
success check. (Behaviours a pipeline must defend against, not bugs: `move_to(linear=True)`
reports "ok" when the hand moves freely after the object slipped out; `gripper_holding()` is True
when pinching a fixed obstacle.)

## Cost
- Wall clock: about 62 minutes (08:38 to 09:40 local), all simulation on <sim-host>.
- Tokens: roughly 0.21 M tokens of context consumed by the session (session token counter;
  input/output split not available).
