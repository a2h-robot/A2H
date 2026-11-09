# pick_ripe_apple: assumptions of pipeline.py

Line numbers refer to `pipeline.py` as of the commit that adds this file (s0 = 1.0).

## Where objects are / how they are oriented
| # | Assumption | Code |
|---|---|---|
| A1 | Both apples hang where `detect()` says for the whole observation and they do not move until grasped (their centre is averaged over many detections) | L185, L209 `center = np.mean(...)` |
| A2 | The good apple's side decides the arm: left apple -> left arm, right apple -> right arm (each apple is reachable by the arm on its side) | L180-181 |
| A3 | The apple can be reached by a horizontal front pinch from one of three fixed headings (90/45/135 deg) at a 10/8/12 cm standoff | L18-19 `HEADINGS`, `TILTS=(0,)`, L193-197 |
| A4 | The branch is directly above the apple and ends just outside it, so "down 1.5 cm, slide 8 cm outward, lift 10 cm" clears it | L252-255 |
| A5 | The basket stays in one lane (fixed y band), oscillates along x between two ends that were seen during the first ~10 s, and keeps the same speed and end points afterwards | L141-148, L260-266, L301-311 |
| A6 | The basket rim height is the top of its world-frame box | L145 |

## What objects look like
| # | Assumption | Code |
|---|---|---|
| B1 | The good apple turns from green towards a known red `RIPE_RGB`, the spoiled one towards a known yellow `SPOILED_RGB`; the green channel falls only on the good one | L12-13, L125-138 |
| B2 | Ripeness progresses linearly in time towards that red and the ripe moment is when the colour reaches it | L110-122, L211, L242-243 |
| B3 | `ObjectState.rgb` is available (not None) for both apples | L101 |

## Whether objects move
| # | Assumption | Code |
|---|---|---|
| C1 | Only the basket moves before the grasp; the apples are static while attached | L185, L209 |
| C2 | The basket's motion is a constant-speed back-and-forth that can be predicted from an 8-sample sliding fit (velocity > 3 cm/s outward triggers the release) | L301-311 |

## What is between the gripper and the target / along the path
| # | Assumption | Code |
|---|---|---|
| D1 | Nothing besides the two apples and the basket's sweep occupies the workspace: `get_obstacles()` is never called; the planner's world only holds the sweep box and the apples | L186-188, L195, L254, L279-284 |
| D2 | The straight-in segment from the pre-grasp to the pinch, and the short clear moves, are collision free without any obstacle model | L219-221, L252-255 |
| D3 | The carry legs (forward at carry height, across, down) are free | L277-286 |

## What the camera sees
| # | Assumption | Code |
|---|---|---|
| E1 | Not used: the pipeline relies on `detect()` (pose, box, colour) only, never on `camera_frame()`; lighting and camera pose do not affect it | (whole file) |

## How objects respond to contact
| # | Assumption | Code |
|---|---|---|
| F1 | A pre-close to 0.82 leaves the fingers resting on (not pushing) the fruit, and a full close holds a ~5 cm, ~0.3 kg apple by friction | L24, L240-247 |
| F2 | The apple comes off the stem as soon as the closing fingers touch it (no pull needed) | L240-244 |
| F3 | A slow carry does not shake the apple loose; the apple falls straight down when released and comes to rest in the basket | L273, L314 |
| F4 | `gripper_holding()` (opening > 0.30) tells a held apple from a miss | L245-247, L295 |
