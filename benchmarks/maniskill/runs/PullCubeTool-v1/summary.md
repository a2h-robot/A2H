# GAH run summary: maniskill / PullCubeTool-v1

Goal: "use the tool to pull the cube within reach". TAU = 0.8, 10 seeds per evaluation.
Base pipeline: s0 = 1.0 (seeds 0-9).

## Variations designed

| Round | Variation | Axis | Edit | s_break | Oracle | s_recovered | Status |
|---|---|---|---|---|---|---|---|
| 1 | rejected_r1a | orientation | tool yaw +180 deg | 0.9 | - | - | rejected (did not break) |
| 1 | v1 | orientation | tool flipped (roll 180 deg) | 0.0 | 0.0 | 1.0 | accepted |
| 2 | rejected_r2a | object_motion | cube pushed 1.5 cm while the tool is fetched | 1.0 | - | - | rejected (did not break) |
| 2 | v2 | object_motion | cube pushed 13 cm sideways (0.2 m/s for 1.2 s from t=0.5 s) | 0.0 | 1.0 | 1.0 | accepted |
| 3 | v3 | obstacle | immovable block directly behind a far cube | 0.0 | 0.0 | 0.0 | unsolved (no detour within reach) |
| 3 | v4 | obstacle | immovable block halfway down the pull lane (parameter_tuning) | 0.0 | 0.0 | 1.0 | accepted |
| 4 | dyn_* sweep | dynamics | heavy cube, high-friction cube, slippery tool | >= 0.8 | - | - | did not break (logged in runs.jsonl) |
| 4 | v5 | dynamics | tool mass 1.0 kg | 0.0 | 0.0 | 1.0 | accepted |
| 5 | rejected_r5a | object_position | tool on a 6 cm platform | 1.0 | - | - | rejected (did not break) |
| 5 | v6 | obstacle | tall (15 cm) block right beside the tool's grasp point | 0.0 | 0.0 | 1.0 | accepted |

## Axes that broke the program

- **orientation** (v1): a flipped tool, invisible to a yaw-only model.
- **object_motion** (v2): the cube moved after its only detection.
- **obstacle** (v4; v3 broke it too but is unsolvable): a block in the straight pull lane.
- **dynamics** (v5): a heavy tool pivoting in the fingers.
- **obstacle at the grasp** (v6): clutter beside the one fixed grasp point on the tool.

## Axes that did not break it

- Tool yaw: the pipeline already used the detected yaw.
- A small push of the cube (1.5 cm): it stayed within the hook's clearance.
- Cube dynamics (heavy, high friction) and a slippery tool: the slow pull and the grasp had enough margin.
- A raised tool (6 cm platform): the grasp height was hard-coded at table level, but the hand's compliance still produced a grasp and the program never depended on that height afterwards.

## Rules found

1. Use the full detected rotation; never reduce an object's pose to yaw. Read which side a functional part (the hook) is on from the rotation (round 1).
2. Re-perceive anything acted on long after it was seen, and wait for it to be still. Re-measure the in-hand pose of a held tool after each carry instead of trusting one measurement (round 2).
3. Plan contact strokes against the obstacles in the scene, including the hand and fingers, not only the tool and the object. Try the plans in order of how unusual they are. Report a layout as unsolved when no detour exists within reach, rather than forcing one (round 3).
4. Check how the tool hangs when lifted; if it pivots in the grasp, do not carry it in the air, keep it on the table and slide it. Never let a contact move fall back to a free-space planner. Check that every waypoint is within reach (round 4).
5. Before grasping, check the gripper's approach (hand and open fingers) for clutter at several candidate grasp points and take the first clear one. Take the grasp height from the perceived pose, not from a constant. If the clear grasp is unsuitable for using the tool, use it only to move the tool to free space, then regrasp properly (round 5).

## Stopping rule

The run stopped at MAX_ROUNDS: 5 accepted variations (v1, v2, v4, v5, v6). Every reflection has novel: true, so the novelty stop rule (two consecutive accepted rounds with novel: false) never triggered. The final `pipeline.py` scores 1.0 on the base task and every accepted variation (seeds 0-9). On held-out seeds 100-119 it scores 1.0 everywhere except v6 (0.95, seed 115). The maximum is 972 steps, within the 2000-step budget.

Round 5 was interrupted on 2026-10-24 when the simulation host (jinbang-4080) went offline before the v6 gate could run. The run resumed on 2026-10-25 after the host rebooted. The repair written while it was down was applied unchanged and recovered v6 to 1.0 on its first evaluation.

## Process notes

- Every accepted variation broke the frozen (committed) pipeline completely (s_break 0.0) and was recovered to 1.0 on seeds 0-9. Held-out seeds 100-119: 1.0 on every scene except v4 (0.95) after round 4 and v6 (0.95) after round 5.
- Four first attempts (rounds 1, 2 and 5, plus the dynamics sweep in round 4) were rejected or did not break the pipeline by the break gate. Each one showed which assumption was already handled, and the next design aimed at what it had revealed.
- Repairs also exposed latent weaknesses on the base task itself: the tool pivoting in the fingers during fast carries (round 2), and a silent RRT fallback plus unreachable waypoints (round 4).
