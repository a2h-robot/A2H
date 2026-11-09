# Assumptions of pipeline.py (catch_shelf_marble)

Line numbers refer to `pipeline.py` as committed for phase 1 (73a9144). Status: **attacked** = a variation was built against it.

## Where objects are
| # | Assumption | Code |
|---|---|---|
| A1 | The two keys are at fixed positions found once at the start; they do not move during the episode. | L144-149 (`keys` measured once), L155-157 `pose()` |
| A2 | The key top is at `bbox_center.z + extents.z/2` of the detected key, and pressing to `top + 0.150` fires it while `top + 0.180` does not. | L148, L25-26, L192, L204 |
| A3 | The bowl rim height is `bbox_center.z + min(extents)/2` and the ball enters the bowl at rim + ball radius. | L152-153, L163 |
| A4 | The ball always ends on the belt within the bowl's travel range; the pipeline never checks reachability. | L189-200 (no range check) |

## How they are oriented
| A5 | The grippers' home orientation is top-down and is good for pressing; it is reused for every key pose. | L150, L157 |

## What they look like
| A6 | `detect("red ball")` / `detect("bowl")` always return the right object (names never confused, never None for long). Averaging 20 detections gives a ~2 mm estimate (independent noise per call). | L24, L44-50, L66 |
| A7 | No image is used, so lighting / camera changes cannot matter. | (no `camera_frame` call) |

## Whether they move
| A8 | **The bowl moves only while a key is held, at a constant speed, and stays where it was when the key is released** (except a learned release overshoot of 0-4 cm). Between presses and while an arm is moving (~0.15 s per move, blind), the bowl is assumed still. | L58, L186 (deadband), L198-211 (release target = aim - overshoot, EMA of measured overshoot) |
| A9 | The left key drives the bowl to -x, the right key to +x. | L23, L189 |
| A10 | The ball slides at <= ~0.07 m/s, falls with gravity 9.81 from shelf edges, and its final fall leaves with ~0.17 m/s sideways in its direction of travel (or of its last sideways jump). | L27-34, L101, L107-108, L123, L132 |
| A11 | A sideways jump > 2 cm faster than 0.12 m/s means the ball is about to drop in that direction. | L30-32, L118-123 |
| A12 | The ball takes at least a few seconds to descend, so closing both grippers (2.4 s) and moving to the keys first is affordable. | L142-143, L159 |

## What is between the gripper and the target
| A13 | The paths from home to the key hover poses and the 3 cm press stroke are free: `move_to` / `move_both` get no `avoid` list. | L159-161, L192, L204 |
| A14 | Nothing sits on the belt or between the bowl and the ball's path. | (not modelled) |

## What the camera sees
| A15 | Not used (see A7). |

## How objects respond to contact
| A16 | The key is a spring: depth follows the gripper height, fires past a threshold, springs back when the gripper rises. A failed press is detected as "bowl did not move" and the press goes 5 mm deeper. | L205-213 |
| A17 | The ball, once inside the bowl, stays there when the bowl moves; the pipeline stops acting once the ball is at the rim. | L185, L196 |

## Ranking for a real robot
1. A8 (the controlled object stays still when not driven): real conveyors creep, coast and get bumped; the pipeline's quiet periods (deadband, blind arm moves) are exactly when it would drift. **attacked in v1: rejected by the break gate (s_break 0.8): the closed loop already absorbs a 6 cm/s drift.**
2. A13 (free path to the key): real workspaces have clutter near buttons. **attacked in v2: accepted (s_break 0.3), recovered to 0.8.** The repaired pipeline (round 1) passes static obstacles as `avoid`, chooses the gripper yaw / press offset from a gripper footprint, approaches from above, and checks the reached pose.
3. A18 (added after round 1): **the bowl is empty**, nothing in the code reads its contents. **attacked in v3 (thin stick: rejected, s_break 0.5, it slides aside or is flung out) and v4 (5 cm canister: accepted, s_break 0.1; unsolved, best 0.4: clearing it takes ~10 s, more than the time to the first landings).**
4. A19 (added by the round-1 fix): **obstacles are static**, measured once at the start. **attacked in v6 (post slides in at 7-9 s: rejected, s_break 0.5) and v7 (at 4-6 s: accepted, s_break 0.4; unsolved, best 0.7).**
5. A2/A16 (the key top is free): **attacked in v5 (block lying on the key): rejected, s_break 0.8, the press just shoves the block off.**
6. A7 (appearance): **attacked in v8 (30 % warm light, moved head camera, recoloured ball and bowl): rejected, s_break 0.8, as expected for a pipeline that never reads an image.**
7. Not attackable with this backend's edit keys: A9 (key-to-direction mapping) and A1 (keys move), because keys, belt and shelves are parts and cannot be edited; A6 (detector confusion / correlated noise), because detection noise is fixed in the API; A10 (ball dynamics), because RoboDyna re-applies the marble's mass and material when it hands the final fall to physics; bowl dynamics, because the bowl is kinematic.

## Pipeline-specific facts worth keeping (not variations)
- The backend planner (cuRobo) does not model the WSG gripper: `move_to` returns True for paths that drive the palm into an obstacle (found in round 1).
- The marble's final fall is a PhysX drop with ~0.15-0.2 m/s sideways speed, and when it leaves the oscillating shelf it often teleports 2-5 cm to the planned edge first (RoboDyna's kinematic-plan / live-phase mismatch). Seeds 10001 and 10008 fail on the base task for that reason with a reactive servo.
- 10-seed success rates of the same program vary by +-0.1 to 0.2 between runs (planner timing on a shared GPU host); s0 = 0.8 was the second consecutive 0.8 of the final phase-1 code.
