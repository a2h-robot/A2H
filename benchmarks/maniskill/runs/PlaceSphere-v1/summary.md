# PlaceSphere-v1: GAH training summary

**Status: finished.** The stopping rule "MAX_ROUNDS = 5 accepted variations" was met on 2026-10-25.
Settings: TAU = 0.8, SEEDS = 10 (seeds 0-9), MAX_ROUNDS = 5, MAX_REPAIR = 6.
TAU stayed at 0.8: the Phase 1 pipeline beat the official motion-planning oracle (0.6 on seeds 0-9), so the
0.6 fallback was not needed.

Interruption: the GPU host jinbang-4080 was offline from 2026-10-24 ~13:50 to 2026-10-25 ~11:40 EDT (and
again briefly at ~12:00 and 13:05 for overheating). Rounds 2-5 ran after it came back, under the new load
limits (3 workers, one ms command at a time). No results were lost. The interrupted v2 gate was re-run
from scratch.

## Phase 1
- s0 = **1.0** on the first pipeline revision (median of many noisy detections, top-down grasp with the
  fingers at 45 deg into the bin corners, in-hand offset re-measured after lifting, release just above the rim).

## Variations

All five passed the gate with s_break = 0.0 (frozen pre-round pipeline, s0 = 1.0). The official oracle failed
four of them, so those count only because the agent recovered them (Phase 4).

| Round | File | Axis | Assumption violated | s_break | Oracle | s_recovered | Repair iterations |
|---|---|---|---|---|---|---|---|
| 1 | v1.yaml | object_motion | A1: the sphere stays where it was perceived (it rolls at ~4 cm/s from t = 0.3 s) | 0.0 | 0.0 | 0.9 | 4 |
| 2 | v2.yaml | obstacle | A2: the paths between waypoints are free (15 cm wall between pick and place, bin moved +8 cm) | 0.0 | 0.0 | 1.0 | 2 |
| 3 | v3.yaml | obstacle | B4: obstacles only matter by their top (shelf 30 cm above the bin, bin moved +10 cm) | 0.0 | 0.0 | 1.0 | 2 |
| 4 | v4.yaml | object_position | B5: the sphere rests on the table plane (it sits on a 12 cm platform) | 0.0 | 0.1 | 1.0 | 1 (+1 lost run, see notes) |
| 5 | v5.yaml | distractor | C1: the goal is empty (a 3 cm block sits in the bin; bin position fixed) | 0.0 | 0.0 | 1.0 | 1 |

Final pipeline (round 5), `eval.py --all --seeds 10`:

| base | v1 | v2 | v3 | v4 | v5 |
|---|---|---|---|---|---|
| 1.0 | 0.9 | 1.0 | 1.0 | 1.0 | 1.0 |

## Axes that broke the program and axes that did not
- **Broke it:** object motion (v1), obstacles between pick and place (v2) and over the goal (v3), object not on
  the table plane (v4), occupied goal (v5). Every one dropped the pre-round pipeline from 1.0 to 0.0.
- **Did not break it** (screened, not gated): sphere dynamics. With mass 0.3 kg and friction 0.1 (a polished
  steel ball) the round-2 pipeline scored 5/5, because the simulated grasp holds it firmly.
- **Not attacked:** colour, lighting and camera pose. Perception in this backend is pose-based, so they cannot
  affect the pipeline (SCENE_EDITS.md). Bin motion was also not attacked: the bin is kinematic, so
  `object_motion` cannot move it.

## Rules found (all five marked novel)
1. Re-perceive an object immediately before every contact and estimate its velocity, not just its position; if
   it moves, plan to where it will be after your own motion time and verify the grasp before continuing.
2. Give the planner every known obstacle, inflate for the object you are carrying (which the planner does not
   see), treat the hand's footprint and orientation as free variables to search over, and check every motion's
   return value before the next action.
3. Free space is bounded from above as well as below: choose the lowest motion that keeps the carried object
   clear in every direction, and when a target is unreachable, search the hand's full orientation (tilt as well
   as yaw) before concluding it is blocked.
4. Derive every coordinate of a target pose from perception; any constant that encodes where an object
   "normally" is (support height, orientation, size) is an untested assumption that will fail the first time the
   object is somewhere else.
5. Before an action, check its preconditions on the target (free, reachable, stable), and when one fails,
   consider changing the scene (move the blocker, while the hand is free) instead of only planning around it.

The convergence rule (two consecutive novel: false) did not trigger. Rules 2 and 3 overlap partly (both say:
search over hand orientation), but rule 3's main point, that obstacles can bound free space from above, is new.

## Notes and known weaknesses
- **v1 seed 1 fails in every evaluation.** The sphere gets kicked away during a failed rolling grasp and the
  pipeline runs out of attempts ("giving up"; earlier versions hit the 2000-step limit). There is no
  reachability check before retrying.
- **Episodes are slow** (~40-200 s of wall clock) because of heavy detection averaging (up to 40 detections per
  control step while estimating velocity).
- **Remaining assumptions, not attacked:** the sphere moves at constant velocity (no acceleration, curves or
  bounces); the hand only tilts forward along world x (the robot is assumed to face +x); the sphere radius and
  bin geometry are hard-coded; nothing disturbs the sphere after release.
- **Process note:** `bin/ms` copies `runs/` back from jinbang-4080 after each command. An edit to
  `runs/<task>/pipeline.py` made while an ms command is running is silently overwritten. This cost one
  repair evaluation in round 4 (it ran the unfixed pipeline and scored v4 = 0.0). Edit only between ms commands.
- Assumption lists: `assumptions.md` (A1-A11 from Phase 1, B1-B8 after round 2, C1-C3 after round 4).
  Reflections: `reflections.jsonl`.
