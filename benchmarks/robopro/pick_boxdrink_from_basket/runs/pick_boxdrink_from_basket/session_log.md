# Session log: RoboPRO backend and GAH training on pick_boxdrink_from_basket

This is a curated record of the agent session (Claude Opus 5.5 in Claude Code, 2026-10-23). It covers what was
built, what was run, what failed and why, and the decisions taken.
- The machine-readable records are in the repo: commits per round, `variations/*.yaml` with their `.gate.json`,
  `gate_log.jsonl`, `reflections.jsonl`, `screening/` and `../../logs/runs.jsonl`.
- Times are Toronto time (EDT). The cluster clock is UTC, so file names under `logs/frames/` are 4 h later.
- Unless noted otherwise, every success rate is over the first 10 seeds of `seeds/pick_boxdrink_from_basket.json`.

Compute: one GB10 node (aarch64, one Blackwell GPU, unified memory) at a time, with 2-5 simulation workers
running as low-priority guests beside other users' jobs. Each worker reuses one RoboPRO task object, so the
robot load and cuRobo warm-up (~13 s) happen once per worker; a reset takes ~1 s and an episode 15-60 s.

## User requests

| Request | Outcome |
|---|---|
| Read the RoboPRO access notes; learn to run the simulator and evaluator | Done. The install, both interpreters and the seed banks were verified. Its baseline-checkpoint links were broken, but checkpoints were not needed. |
| "No checkpoints needed, we only need access to the simulator" | Scope narrowed to the simulator. |
| Read the GAH repo and get ready for a RoboPRO task | Done |
| Task: pick_boxdrink_from_basket; branch `d-robopro-<task>`, folder `benchmarks/robopro/`; up to 2 GB10s or 2 5080 GPUs | Done. The whole run is pinned to one GB10 at a time, because s0 and gate numbers must come from one hardware type. |
| Share GB10 nodes with another GAH session when none were free | Done, under the team's compute coordinator; see "Compute" below. |

## 1. Simulator access and backend (15:50-16:55)

**Simulator probe.** A sim-only smoke test built the task the way RoboPRO's evaluator does and ran RoboPRO's
scripted expert (`play_once`):
- The box drink was never in the basket at the start: it lay on the floor or table, and the expert failed on
  every seed.
- A per-step trace showed why. The box is created at the basket's origin, with its bottom 2.2 cm inside the
  basket's floor mesh (contact separation -0.022 m). On this aarch64 PhysX build the first step ejects it at
  ~3 m/s.
- The task's `spawn_pose.p[1] -= 0.02` is a no-op, because `sapien.Pose.p` returns a copy. A first lift fix
  failed for the same reason. The task's constant `BASKET_BOXDRINK_LOCAL = [0, 0, 0.03]` is never used.
- A 3 cm lift applied before the first physics step (backend-side, success check untouched) fixed it. The box
  then rests upright in the basket and the expert solves 6/6 seeds.

**Backend design** (ported from the RoboTwin backend v2 by another team member, adapted to RoboPRO's bench scenes):
- `gah_env.py` builds the task from RoboPRO's bench config. It applies three fixes to RoboPRO's scene setup:
  the spawn lift; renderer pools capped at 4000 (RoboPRO builds a renderer per reset with 50000-entry pools, a
  known per-episode memory leak); and the curobo collision cache raised from 3 to 96 cuboids.
- The planner world adds the kitchen furniture (one box per link) and models the basket as a floor slab plus
  four walls, so the gripper can reach inside. A rim-height map of the basket mesh later confirmed this model.
- Because RoboPRO's expert works here, `tools/oracle.py` runs it on any variation. The gate records its success
  rate as information only: it is open-loop, and its planner does not know added bodies.
- **Seeds:** RoboPRO's own 20-seed clean bank plus 40 sequential seeds, filtered to those the expert solves.
  That gives 32 of 60 (16 with the basket on the table, 16 with it on the microwave). The expert fails every
  seed of a third layout (basket mid-table), so that layout is absent.
- **Integrity:** the hashes cover the backend, the tools, and `check_success` plus the helper it calls.

**Sanity checks:** (a) a trivial pipeline runs at 0.0; the gate rejects (b) a no-op variation, (c) a third
tuning variation in a row, (d) a modified success check and (d') a modified backend file.

## 2. Phase 1 (16:57-17:17)

| Eval | Change | Result |
|---|---|---|
| 1 | Grasp the top of the box from above (tilted fallbacks), lift, place on a free spot | 0.1. Top-down poses were out of reach, and the box swung in the fingers (gripped 3.5 cm below its top) |
| 2-3 | Mid-height grasps entering the basket on the robot's side | Every pre-grasp was refused. A probe showed a 5 cm pre-grasp distance put the fingertips in the box under 1 cm detection noise. With 9 cm and steep approaches first: 0.3 |
| 4 | Spots near an easy-reach point; yaw search in `place` | 0.5. `place()` falls back to 20 deg tilted poses, and a 22 cm box released tilted topples |
| 5 | Own upright placement: level gripper only, several spots and yaws | 0.5. The lift ran out of reach, and the box was dragged through the basket wall |
| debug | The rim was computed from the basket's depth (its box extents are in its own frame, rotated 90 deg) | The required lift was 13.1 cm, not 15.6. A reach map showed shallow approaches reach highest |
| 6 | World-z extents; shallow approaches first; verified lift (never drag through the wall) | **1.0, s0 = 1.0** (commit 6910a0a); 20/20 on seeds 10-29 |

## 3. Round 1: v1, a cluttered table (17:20-18:45)

**Screening** (frozen pipeline, 5 seeds):
- Box yawed 45 deg 1.0, heavy carton 1.0, basket 6 cm farther 0.8, 12 cm item in front of the box 0.8.
- Three breakers were invalid: the box against the near wall, a tall carton in front, and items on both sides.
  The basket is a sloped bowl, so these interpenetrate it and the box is ejected at setup.
- Box at the far side: valid, 0.0 (kept for later). Cluttered table: valid, 0.2.

**Gate:** the first attempt was rejected on integrity. The dev sync from the laptop had overwritten the
regenerated hash file with a stale copy, and the gate caught the mismatch. After regenerating it: s0 = 1.0,
s_break = 0.0, oracle 0.0 (the expert's fixed target spot lies in the clutter).

| Repair | Change | base / v1 |
|---|---|---|
| 1 | Search spots over the whole table, with true clearance (world-frame extents) | 1.0 / 0.5. Spots under the wall cabinet; the box was dropped when a descent stopped short |
| 2 | Overhang check; release only at resting height | 1.0 / 0.6. Free planner paths swept the held box through the clutter |
| 3 | Carry the held box in straight segments: leave the basket towards the robot, change height in place to a band clear of everything under and over the path, cross, descend | **1.0 / 1.0** (commit 3408f4f) |

## 4. Round 2: v2, the box leans against the basket wall (18:46-19:30)

**Screening:**
- A 16 cm item in front of the box and the box lying down were invalid (bowl slope).
- A wet heavy carton scored 0.8, a larger carton 1.0, a nudge during the approach 1.0.
- So v2 is the box pushed to the far side, where it settles leaning ~34 deg.
- A pre-check with a draft pipeline solved 2/5 (frozen: 0/5) before gating, as evidence of solvability.

**Gate:** s0 = 1.0, s_break = 0.1, oracle 0.3.

| Repair | Change | base / v1 / v2 |
|---|---|---|
| 1 | Average orientation as well as position (one detection carried 3 deg of noise, which tilted an upright box); grasp on the actual axis with fingers across it; lift from the lowest point; turn the box upright in the hand over the free spot | **0.9 / 0.9 / 0.9** (commit ab918cc) |

## 5. Round 3: v3, a fixed item in front of the basket (19:31-21:55), unsolved

**Screening:**
- A tray sliding onto the likely spot: valid, 0.4. A full 1 kg carton: 0.6.
- A paper-towel holder (static cylinder) in front of the basket first crashed: RoboPRO's `create_cylinder` has
  no static option, a backend bug, fixed in commit e2e4a43. Once fixed: valid, 0.0.

**Gate:** s0 = 1.0, s_break = 0.2, oracle 0.0.

| Repair | Change | base / v1 / v2 / v3 |
|---|---|---|
| 1 | Leave the basket in the first direction whose swept path is clear (robot side, then the workspace centre, then outwards) | 0.9 / 1.0 / 0.8 / 0.1. The item blocks the shallow grasps; steep grasps cannot lift high enough |
| 2 | Add a heading from the workspace-centre side; open the gripper only when needed (each opening costs 1.2 s); lift fallbacks (move towards the robot, then pitch the hand 0.3 rad shallower), checked by detection | v3 0.7 |
| 3 | Hold checks; upright low over the spot first; `run` retries from the current state | 1.0 / 1.0 / 0.8 / 0.6 |
| 4 | Shallow elevations first across all heights and headings | 1.0 / 1.0 / 0.8 / 0.5 |
| 5 | Add a lower grasp height (38 %) | 1.0 / 1.0 / 0.8 / 0.3. The first run was cut short when another project's job took the node (7 episodes never ran); this is the full re-run on the next node |
| 6 | Retry with the other arm | 1.0 / 1.0 / 0.7 / 0.5 |

Outcome: **unsolved**. Where the basket sits on the microwave, the box must rise above the arm's comfortable
reach, and the item removes the approaches that still reach that high. The fallbacks produced motions at the
reach limit: tracking errors, contacts with the cabinet and the table, and tilted boxes. The last attempt is
in `unsolved/v3_pipeline_attempt.py`. The pipeline was reverted to round 2; a re-check gave base 0.9, v1 1.0,
v2 0.9 (commit d035fae).

## 6. Round 4: v4, a tray slides onto the placement area (21:55-22:20)

**Gate:** s0 = 1.0, s_break = 0.4, oracle 0.6. The pipeline chose its spot before the carry and set the box
down on the tray that had arrived there.

| Repair | Change | base / v1 / v2 / v4 |
|---|---|---|
| 1 | Before lowering: wait until the scene is still (averaged obstacle snapshots 0.5 s apart; single snapshots are too noisy to compare); re-check the spot's clearance; re-choose and re-carry if it is taken | **1.0 / 1.0 / 0.9 / 1.0** (commit 35ae926) |

## 7. Round 5: v5, a full 1.2 kg carton (22:20-23:25)

**Screening** (against the round-4 pipeline):
- A wet 1 kg carton scored 0.6.
- A 1.2 kg carton (friction 0.3) scored 0.2, and so did a 30 % larger carton; both were valid.
- The 1.2 kg trace showed the lift reporting success, then the pipeline stopping. Under the load the arm sags
  1-2 cm below the commanded height, so the check on the commanded height failed although the box had cleared
  the rim.

**Gate:** s0 = 0.9, s_break = 0.2, oracle 0.5.

| Repair | Change | base / v1 / v2 / v4 / v5 |
|---|---|---|
| 1 | Closed-loop heights from detection: lift, carry and release | 1.0 / 0.9 / 0.7 / 0.9 / 0.9. An uncapped lift loop chased a box sliding in the fingers to a height outside the workspace |
| 2 | Cap lift corrections at 4 cm beyond the first estimate | **0.8 / 1.0 / 0.9 / 0.8 / 0.9** (commit 48831b5) |

## Compute

- **Placement.** One GB10 at a time, as an out-of-Slurm guest at nice 10, with private CUDA MPS directories and
  node-local scratch. The run used four nodes, moving each time another user's job arrived (once just
  before the pool started).
- **Footprint.** A busy worker costs ~10 GB of node memory, not the 4.4 GB its PSS shows: GPU allocations come
  out of unified memory.
- **Incident.** The node's memory guard kills the *largest Slurm job*, not the guest. With 5 pool workers plus
  extra single-process probes, the node dropped to ~28 GB free, and another project's job was killed.
- **Supervisor policy after the incident** (`tools/pool.py`):
  - 0 workers while a backbone-study job runs on the node;
  - beside any other job, at most `(MemTotal - the jobs' memory requests - 5 GB) / 10 GB` workers, and
    MemAvailable kept >= 60 GB;
  - new workers only above 80 GB, 75 s apart. A 60/70 GB band oscillated, because a worker's warm-up takes
    ~10 GB.
  - No ad-hoc processes beside the pool while a job is present.
