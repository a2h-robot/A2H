# Session log: RoboPRO backend and GAH training on put_spoon_on_plate_ks

This is a curated record of the agent session (Claude Opus 5.5 in Claude Code, 2026-10-23). It covers what was
built, what was run, what failed and why, and the decisions taken.
- The machine-readable records are in the repo: commits per round, `variations/*.yaml` with their `.gate.json`,
  `reflections.jsonl`, `screening/`, `unsolved/` and `../../logs/runs.jsonl`.
- Times are Toronto time (EDT).
- Unless noted otherwise, every success rate is over the first 10 seeds of `seeds/put_spoon_on_plate_ks.json`.

**Compute.** GB10 nodes (aarch64, one GPU, unified memory), as an out-of-Slurm guest next to other users' jobs,
under rules set by the team's compute coordinator:
- a pool of 1-3 workers, nice 10, memory guards;
- the pool stops at once if a backbone-study job lands on the node.

The pool moved four times during the evening because nodes were taken by Slurm jobs. One warm worker holds about
4.2 GB host memory plus 9.3 GB GPU memory.

## 1. Task and setup (15:35-16:00)

**The user's requests:**
- learn how to access and use the RoboPRO simulator (no policy checkpoints needed);
- then "work on this task: Put spoon on plate", in folder `benchmarks/robopro/` on branch `d-robopro-<task>`, with
  up to 2 GB10 nodes or 2 RTX 5080 GPUs.

**Task:** RoboPRO `put_spoon_on_plate_ks` in the small-kitchen scene (three layouts of microwave, dish rack and
sink at the back of the counter).
- A 0.05 kg spoon (20 cm long) lies at |x| >= 0.3 m with a random yaw.
- A static plate sits near the centre, clear of the sink.
- Success: the spoon's model origin is within 3 cm of the plate's origin in x and in y, and both grippers are open.

**Simulator check.** RoboPRO's scripted solution (`play_once`) solved the first 3 seeds.
- Warm-up: ~20 s; reset ~0.8 s; expert episode 5-6 s.
- A scene probe listed the collision bodies:
  - the counter is 4 boxes around a sink hole, plus a sink basin with 4 rims;
  - the dish rack is one triangle mesh;
  - the microwave is a 4-link articulation;
  - the spoon is a dynamic convex hull with 4 annotated grasp points;
  - the plate is static, with 22 convex pieces, and its model frame is rotated.

**Backend: a port of the RoboTwin 2.0 backend (backend v2).** It keeps the same API, tools, flags and JSON. The changes:
- `gah_env`: imports the bench tasks and names the furniture (counter, wall, sink, dish rack, microwave) and the
  task objects (spoon, plate);
- curobo box cache forced to 64. The task passes its own cache of 3 boxes, so the RoboTwin patch that only sets a
  default would not have worked;
- SAPIEN render pools capped by wrapping `set_global_config`. RoboPRO hard-codes 50000 entries and builds a
  renderer per reset;
- scene edits: name resolution, swap and scale for the spoon and the plate, kinematic motion of static bodies,
  and validity rules for the counter and the sink hole;
- seeds: RoboPRO's frozen clean seed bank first, then the integers after it. A seed is kept only if its scene is
  stable and the expert solves it: 40 kept of 119. The expert solved 14 of 20 bank seeds, but only about 1 in 5
  of the integers after them.

**Findings while bringing it up:**
- RoboPRO's `create_cylinder` cannot build static bodies, so the backend builds its own.
- **The success point is not the box centre.** The spoon's mesh spans -7.5 to +12.5 cm along its axis, so the
  model origin (what the success check measures) is 2.46 cm from the bounding-box centre. The `center` annotation
  in the model file says 0, which is wrong for the mesh. With `detect()` reporting box centres, as the RoboTwin
  backend does, every placement missed by 2.5-3.8 cm on one axis: base success 0.5.
  - Decision: `detect()` of task objects reports the model frame, as a 6D pose estimator does, plus a new
    `bbox_center`.
  - This is a divergence from the RoboTwin backend (ISSUES 3).
- Two memory experiments, both negative:
  - Capping curobo's mesh cache (and skipping RoboPRO's per-reset mesh push, which the box-only API never uses)
    did not shrink a worker (9.31 -> 9.28 GB GPU). Planning was unchanged: the same 10 seeds gave 0.5 vs 0.5, with
    final positions within 1 cm.
  - The rasterizer shader gave identical numbers. Perception is pose-based, so the shader cannot affect results.
  - Recorded in ISSUES 7-8.

**Sanity results:** all PASS.
- (a) a trivial pipeline runs at 0.0;
- (b) no-op variations, (c) a third tuning variation in a row, (d) a modified success check (in a shadow
  benchmark tree) and a modified tool file are all rejected;
- every edit key applies, and the validity check rejects a seed where a rescaled spoon moved 3.4 cm while settling.

## 2. Phase 1 (15:50-16:20)

| Eval | Change | Result |
|---|---|---|
| v0 (3 seeds) | Average 8 detections, grasp with the arm on the spoon's side, lift 10 cm, place over the plate | 2/3. The release height used the plate's diameter (box extents are in the object frame), so the spoon dropped 10 cm |
| v1 (3 seeds) | Release height from the world-vertical extent | 2/3. The spoon slipped out of the hand while the arm crossed the centre line, and `place()` still returned True |
| v2 | Slow, separate carry; check the hold after the lift and the carry; re-pick on a drop | 0.5. Every miss was 3.4-4 cm on one axis (origin vs box centre) |
| v3 | Backend reports task objects' model frames; aim the spoon's frame at the plate's frame | **0.9, s0 = 0.9**. Seed 40030 fails everywhere: the grasp approach next to the microwave is refused |

## 3. Round 1: v1, the plate slides 7 cm during the pick (16:25-16:40)

Gate: s0 = 0.9, s_break = 0.0. The spoon was released where the plate had been.

| Repair | Change | base / v1 |
|---|---|---|
| 1 | Detect the plate after the pick, tracking it in 0.5 s windows until stationary. Check again before the release and follow it if it moved more than 1.5 cm | **0.9 / 1.0** |

## 4. Round 2: v4, the spoon upside down (16:45-17:05)

- **v2**, a flat box on the plate centre, was **rejected** at s_break 0.5. The long spoon still came to rest on
  top of the box, which the xy-only check accepts.
- **v3**, the spoon in the right front corner with the plate far left, was **rejected** at s_break 0.9. The arm
  reaches across.
- **v4**, the spoon flipped. Gate: s_break 0.0. Every annotated grasp frame now approached from under the counter.

| Repair | Change | base / v1 / v4 |
|---|---|---|
| 1 | When the library grasp finds nothing, a top-down grasp from the current box: vertical approach, closing across the long axis, fingertips 6 mm above the underside | **0.9 / 1.0 / 0.9** |

## 5. Round 3: v7, a heavier and slippery spoon (17:05-17:40)

- **v5**, a 6 cm glass on the plate centre, was **rejected** at s_break 0.7.
- **v6**, a 25 cm divider between spoon and plate, was **rejected** at s_break 0.5.
- A **screening batch** of 5 candidates against the frozen pipeline followed (`screening/results.txt`). Three
  broke it: the heavy, slippery spoon, a low overhang, and a narrow slot.
- **v7**: a 250 g spoon with friction 0.2. Gate: s_break 0.1.

**Debugging (through the pool):**
- The library grasp holds for 2 cm of lift and loses the spoon in the next 8 cm. The spoon rises about 1 cm and
  slides down out of the fingers: a friction-limited pinch.
- The carry style made no difference. Fingertip height mattered seed by seed: a marginal grip.

| Repair | Change | Result |
|---|---|---|
| 1 | Re-grasp at the box centre (centre-of-mass guess), slow lift | v7 0.4 |
| 2 | 3 cm test lift before the full lift; on a slip, release, re-detect and cycle grasp variants (centre, fingertip height 2-6 mm, ±1.5 / 3 cm along the axis) | v7 0.5. The remaining failures were refused approaches near the counter |
| 3 | + roll the hand 90° about the spoon so a finger is under it | base 0.8, v1 0.9, v4 0.8, v7 0.5. No gain, and the rolled spoon landed off target. Removed |
| 4 | Let the fallback grasp's short final descent touch the counter (the planner refused the last centimetres) | **base 0.8, v1 1.0, v4 0.9, v7 0.8** |

## 6. Round 4: v8, the spoon partly under a low overhang (17:45-18:30), unsolved

Gate: s_break 0.0.

| Repair | Change | v8 |
|---|---|---|
| 1 | Grasp points along the spoon filtered by the hand's footprint against the obstacle boxes | 0.0. The blocked library grasps ran first and pushed the spoon further in |
| 2 | Check the space above before any reach; give up if the spoon was knocked off the counter | 0.0 |
| 3 | Approach tilted 20-35° from the open side | 0.0 |

The exposed end (7.5 cm) is shorter than the hand plus the planner's margin, so v8 was marked **unsolved**. The
variation and the pipeline were both reverted to round 3.

**Compute during this round:**
- A backbone-study job landed on the node and the guard stopped the pool. The eval in progress lost 7 episodes to
  queue timeouts.
- The run moved to another node.

## 7. Search for round 5 (18:30-18:50)

Eight more candidates were screened against the round-3 pipeline. None broke it (0.7-0.9):
- a teaspoon and a serving spoon;
- the plate nudged late, or drifting slowly;
- a wall at the plate's rim, or the plate on an 8 cm or 12 cm stand;
- a 0.4 kg box resting on the spoon's handle.

The only breaking class left was "no room for the hand at the grasp point".

## 8. Round 5: v9, a smaller overhang (18:50-19:25), unsolved

**v9** is labelled parameter tuning: only v8's overhang extent changes, leaving about 10 cm of the bowl end
exposed. Gate: s_break 0.0.
- The gate ran into the eval tool's 1800 s batch timeout. The frozen pipeline's failing episodes each use most of
  the 120 s simulation budget, and a timeout counts as a failure.
- The record was briefly set aside as "invalid" and then restored unchanged, after checking that the timeouts were
  the tool's own and not caused by the pool stopping.

| Repair | Change | v9 |
|---|---|---|
| 1 | Clearance-checked grasp points chosen before reaching (round 4's repair 2, vertical approach) | 0.3 |
| 2 | Widen the hand's footprint by the planner's margin | 0.2. More episodes ran out of budget; on one seed the spoon was flung off the counter |

v9 was marked **unsolved**, with `novel: false`: the same rule as v8. The pipeline was reverted to round 3.

## 9. Outcome

Training stopped after 5 rounds:
- **3 accepted:** v1, v4, v7, all novel;
- **2 unsolved:** v8, v9;
- **4 rejected** by the break gate: v2, v3, v5, v6;
- 13 candidates screened.

Five accepted variations was not reached. The program withstood every other single-assumption attack that was
designed and screened, and the one class that still breaks it needs either moving the object first or whole-arm
clearance planning.

Final eval of the round-3 pipeline: base 0.9, v1 0.9, v4 0.9, v7 0.7 (10 seeds each, 338 s for 40 episodes on 2 workers). v7 is marginal: 0.8 at acceptance, 0.7 in this re-run.

**Lessons:**
- **Check what the success test measures.** Here it was a model origin 2.5 cm from the box centre, and it decided
  whether s0 was 0.5 or 0.9.
- **Rejections were informative.** The 3 cm per-axis tolerance let objects on the target pass. Reach and planning
  handled far-side and raised targets.
- **Screening before gating saved several full gate runs.**
- **Wall time came from failing episodes.** Each used the 120 s simulation budget in retries, so worker count
  mattered little.
