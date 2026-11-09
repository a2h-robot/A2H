# Session log: RoboPRO backend and GAH training on put_cup_on_coaster

This is a curated record of the agent session (Claude Opus 5.5 in Claude Code, 2026-10-23 to 10-24). It covers what was built, what was run, what failed and why, and the decisions taken.
- The machine-readable records are in the repo: commits per round, `variations/*.yaml` with their `.gate.json`, `reflections.jsonl`, `screening/results.txt` and `../../logs/runs.jsonl`.
- Times are Toronto time (EDT).
- Unless noted otherwise, every success rate is over 10 bank seeds.

Compute: one standalone GB10 (aarch64, 1 Blackwell GPU, 20 cores, 121 GB unified memory) for every number of record. The node was shared under the team's guest rules:
- outside Slurm, nice 10;
- start a worker only at >= 40 GB free;
- stop the newest worker below 20 GB, or after 2 minutes above 89 °C.

The whole run (s0, gates, recoveries) is on that one hardware type.

## 1. Task and assignment (15:30)

- **Task:** RoboPRO `put_cup_on_coaster`, study scene, clean config. Goal: "Place the cup onto the coaster on the table."
- **Success check (RoboPRO's, unchanged and hashed):** |cup.xy - coaster.xy| < 2 cm on each axis (actor origins), and both grippers open.
- **User's requests:**
  - run on the ethernet-attached GB10;
  - work on branch `d-robopro-put_cup_on_coaster` in `benchmarks/robopro/`;
  - PR into `d-robopro`, unmerged.
  - Later: "cluster GB10 now, move to the ethernet GB10 later" while the latter was down.
- **Settings:** TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6.

## 2. Backend port (15:40-16:30, CPU only)

The assigned node was offline for this whole period (no ethernet carrier), so the backend was written before anything could run.

**Why a port of the RoboTwin backend.** RoboPRO's `customized_robotwin/` is a RoboTwin 2.0 fork:
- the same aloha-agilex robot;
- the same curobo planner wrapper and method names;
- the same study-table frame.

So the RoboTwin backend v2 (API, scene-edit keys, tools, JSON formats, force-limit fix) carries over. RoboPRO-specific decisions, in `gah_env.py` unless noted:
- **Task construction.** Build the bench task from the clean study config, with RoboPRO's evaluation instrumentation switched off (collision metrics, per-step proximity tracking, eval mode).
- **Collision cache.** The task asks curobo for a 3-box collision cache, and the planner is built once per worker with the first episode's size. The patch *forces* >= 128 boxes.
- **Render leak.** RoboPRO creates a new renderer with 50000-entry material and texture pools on every reset, which leaks GPU memory in a persistent worker. The pools are capped at 4000, and SAPIEN's cache is cleared every 5 resets.
- **Names.** `cup`, `coaster`, the static furniture (`bookcase`, `wooden_box`), a `coaster_top` region, and automatic registration of any clutter.
- **Seed bank.** Keeps seeds whose scene is stable and that RoboPRO's scripted expert solves (the solvability filter for the base task).
- **Pool supervisor.** Got a thermal guard.
- **Copying the stack.** Only the assets the clean config loads were copied; background textures (11 GB) are used only by randomized configs.

**Infrastructure notes:**
- **The other x86 RoboPRO tree is unusable for this task.** A cluster fallback was considered. It has a different version of this task whose `check_success` has no `abs()`: a one-sided test that also passes cups far past the coaster (ISSUES 1).
- **First compute.** Once compute came back (16:20), the assigned node's GPU failed to initialise (firmware boot error), so it needed a full power cycle by the user.
- **Bring-up probe.** One 3-seed bring-up probe ran on a cluster GB10 in that gap. No numbers of record came from it.

## 3. Bring-up (16:30-16:55)

- **Stack copy:** ~10 GB in 6 parallel streams, 3.4 min. Path fix-up: venv home, the curobo editable path, the robot's curobo configs.
- **Probe of the stock scene:**
  - **The coaster.** It lies flat (10.6 x 10.6 x 0.8 cm), and its origin equals its box centre in xy.
  - **The cup.** Its origin is its bottom centre, with the same xy as its box. So `detect()` coordinates measure what the success check measures.
  - **The expert.** It solves a scene in 4-8 s alone and ends 0.5-1.5 cm from the centre.
- **Seed bank:** 60 of 90 scanned. 24 expert failures (`IK_FAIL`, no valid pre-grasp), 0 unstable.
- **Cost per worker:** ~10 GB of node memory, measured as the MemAvailable delta (host PSS 4.8 GB + 9.5 GB GPU). Three busy workers hold the GPU at ~96%, so the pool stayed at 3 workers.
- **Bug found:** the seed-bank job imported RoboPRO before changing into its root. That killed the workers and orphaned their jobs. Fixed, and workers now record job exceptions instead of dying.
- **Sanity (a)-(e) all pass:**
  - (a) trivial pipeline 0.0;
  - (b) no-op variation rejected;
  - (c) third tuning variation rejected;
  - (d) modified success check rejected, and so is a modified backend file;
  - (e) all 10 edit keys apply.

## 4. Phase 1 (17:00-17:45)

| Eval | Change | Result |
|---|---|---|
| p1a | 8-sample estimates; API grasp (the cup's annotated rim points); API place(); one re-place | 0.1. The rim pinch at depth 0 overlaps the 2-3 mm wall by ~9 mm, so the fingers close fully and the cup slips out on the lift. A loop bug also opened the gripper mid-air |
| debug | Lift test, 6-10 seeds | Rim depth 0: 3/6, depth 0.02: 4/6. Level side grasp: 0/10 (unreachable at table height). Tilted body grasp, 45 or 60 deg: 10/10 |
| p1b | Tilted body grasp + place() | 0.8. The cup pivots 11-18 deg in the hand while place() turns the hand, and is released 3-8 cm off. Backing straight up knocks it over; backing out along the approach keeps it upright |
| p1c | Own placement: translate, align the averaged bottom centre, lower, release, back out | 0.7. Successes now within 1 cm; far-apart layouts unreachable with the hand orientation fixed |
| p1d | Carry with a hand-yaw search; relay through the table to the other arm; 2 mm drop margin | 0.8. The relay estimated the table height from the *held* cup, an 8 cm drop |
| p1e/f | An edit script failed and a file write was refused, so the old version ran again | 0.7 (identical code reads 0.7-0.8: first sign of run-to-run noise) |
| p1g | Arm = the one whose start pose is closest to the farther object; descent fallback; relay height from the coaster | **1.0**, 1.0 on seeds 10-19. **s0 = 1.0** |

`assumptions.md` lists 13 assumptions with the code lines that rely on them, ranked by how likely a real robot is to break them.

## 5. Round 1: v1, a mug-size cup (1.5x) (17:25-17:55)

**Screening** (eval.py --variation, round-0 pipeline):
- walls around the coaster: 0.8;
- the coaster sliding mid-carry: 0.9;
- big cup: **0.0**;
- mug with a handle: 0.5;
- heavy cup: 1.0.

**Gate:** s0 = 1.0, s_break = 0.0.

| Repair | Change | base / v1 |
|---|---|---|
| 1 | Grasp chosen by the cup's width: wide = rim grasp (depth 0.02), else body | 0.9 / 0.7. The API's rim candidates are near-vertical only, unplannable for far cups; the body fallback holds the wide cup, then wedges it out on the lift |
| 2 | Manual tilted rim pinch of the near wall (55-85 deg, 5 azimuths, opening 0.5 so the inner finger clears the far wall) | **1.0 / 0.9** |

## 6. Round 2: v2, four movable boxes 1.5 cm around the cup (17:55-18:55)

**Screening:**
- fixed walls 2.5 cm from the cup: 0.0, but probably unsolvable with 3.7 cm fingers, so not gated;
- movable boxes: **0.0**;
- cup nudged: 0.0.

A screen of 3 x 10 long episodes outlived the remote shell's time limit. The evals kept running; long evals were run detached from then on.

**Gate:** s0 = 1.0, s_break = 0.0.

| Repair | Change | base / v1 / v2 |
|---|---|---|
| 1 | Clear movable neighbours (size-filtered so furniture is never moved; gap < 7 cm), hand side first | 0.9 / 0.9 / 0.2. The first box grasp failed and the loop gave up; every grasp candidate re-opened an open gripper (~1.2 s each) |
| 2 | Try every neighbour; skip redundant gripper motions | All box grasps refused at first: the fingers come 9 mm from the cup, inside the planner's margin |
| 3 | Own top-down box grasp, closing across the axis perpendicular to the cup; touching the cup allowed | A box is removed, but the cup grasp is still refused (9 mm gaps to the other boxes) |
| 4 | Allow brushing the light objects still crowding the cup on the short final approaches | **0.9 / 0.9 / 0.8** |

## 7. Round 3: v3, the cup slides 9 cm during the approach (19:00-19:40)

**Screening needed care:**
- **A plain push tips the cup.** In this simulator a plain constant push tips it over (86-89 deg on 6/6 seeds with the robot idle), because the default friction is high. That would be a knocked-over cup, a different task.
- **Partial fixes didn't work.** An impulse with lower cup friction moves it <= 5 mm. A slow push with cup friction 0.2 still tips it (friction is averaged with the table's).
- **What worked:** cup *and* table at friction 0.1 with a 0.05 m/s push. The cup slides 9.3 cm upright, and screened **0.0**.
- **Control:** the same slick surfaces with no push scored 0.8, so the motion is what breaks the pipeline.

Also screened: a slick coaster 1.0, and the mug 1.0 (round 1's grasp choice generalised).

**Gate:** s0 = 0.9, s_break = 0.0.

| Repair | Change | base / v1 / v2 / v3 |
|---|---|---|
| 1 | Wait until the cup is still (two estimates 0.5 s apart within 1.5 cm); re-check after reaching each pre-grasp and replan if it moved | 0.9 / 0.9 / 0.5 / 0.9. v2 regressed: the left arm cannot reach the right-side boxes from above, and in one attempt the cup was flung |
| 2 | Clear with whichever arm reaches the box | ~20 s per box (place() searches yaws); up to 3 boxes before the first cup attempt |
| 3 | Clear one box, try the cup, repeat; set boxes down with lift-shift-lower | **1.0 / 1.0 / 0.9 / 0.9** |

## 8. Round 4: v4, the cup lying on its side (19:40-22:00), UNSOLVED

**Screening:**
- an object lying on the coaster: 0.8 (the xy-only success check counts a cup resting on it, so this can never be a fair break);
- coaster at the back: 0.8;
- a divider between cup and coaster: 0.9;
- cup on its side: **0.0**. It is carried and set down lying, and its base centre ends 2.7-3.8 cm off.

**Gate:** s0 = 1.0, s_break = 0.0.

| Repair | Change | Result |
|---|---|---|
| 1 | Plan the final upright tilted grasp and map it back through the stand-up rotation | No reachable candidate. With the rim pointing away from the robot, every downward final approach maps to an approach from the far side (geometry, not noise) |
| 2 | Flip: top-down grasp across the lying cup, lift, quarter turn about the closing axis in the reachable direction (on its base or rim, both pass the xy check), set down, regrasp. The axis comes from the detected frame (box extents picked a diameter on a squat cup) | Flip test: 3/3 stood upright. Full run v4 0.5, v3 0.7 |
| 3 | Flip grasps tilted +-20/40 deg and shifted along the cup | v4 0.3, worse. An abort exception escaped the flip (crash). Near-edge cups: ~30 refused descents at ~5 s each |
| 4 | At most 4 refused descents per arm; catch the abort inside the flip | v4 0.4 |
| 5 | Flip only a clearly lying cup; grasp it 6 mm higher | v4 **0.7**, v3 0.8 |
| 6 | Carry the stood-up cup straight to the coaster | v4 0.4 |

Not recovered within 6 repairs, so v4 was marked `unsolved` and the round-3 pipeline restored. The attempt is kept in `unsolved/v4_pipeline_attempt.py`. **Why it failed:**
- the rim-away geometry rules out planning back from an upright grasp;
- flip-and-regrasp roughly doubles the motion, so cups at the back or edge of the table run out of reach or out of the 120 s budget;
- the physics occasionally flung a cup during the turn.

During round 4, v3 read 0.7 in 4 of 5 runs although it had been 0.9: a flip misfire on a wobbling sliding cup, removed by the revert.

## 9. Round 5: v6, four movable boxes around the coaster (22:00-00:40)

**Screening:**
- boxes around the coaster: **0.2**;
- wet heavy cup: 0.8.

The first design, **v5, was rejected by the validity gate**: on 1 seed of 10, a box started inside the furniture and moved 9 cm while settling. The same crowd moved to the middle of the table (v6) passed.

**Gate:** s0 = 1.0, s_break = 0.3.

| Repair | Change | base / v1 / v2 / v3 / v6 |
|---|---|---|
| 1 | Before the pick (hands free), clear up to 2 objects around the coaster with the round-2 routine generalised to any target; brush the rest on the descent | 1.0 / 0.9 / 0.6 / 0.7 / 0.6, nearly all step-budget failures. In several, a placement already inside the 2 cm tolerance was "corrected" by a re-pick that ran out of time with the cup in hand |
| 2 | Re-place only if a 16-sample estimate is > 1.2 cm off AND >= 45 s of the 120 s remain | 1.0 / 0.9 / 0.7 / 0.9 / 0.7 |
| 3 | Release with the gripper opened to the cup's width + 1.5 cm (a full opening swings the fingers into neighbours) | 1.0 / 0.9 / 0.7 / 0.9 / 0.7 |
| 4 | Grasp boxes 4.5 cm below their top and check they are still held after the lift (a shallow pinch slipped and the same box was "cleared" twice); clear up to 3 at the coaster | 1.0 / 0.9 / 1.0 / 0.6 / 0.9 |
| 5 | Body grasp lower (0.15 of the height above centre) so a slick cup swings less | 1.0 / 0.9 / 0.7 / 0.9 / 0.7 |
| 6 | Clear at most 2 at the coaster; no homing between consecutive clearing moves | **1.0 / 0.9 / 0.8 / 0.8 / 1.0** |

## 10. Round 6: v7, cross-table reach (00:40-01:40), UNSOLVED

**Screening:**
- cup on a tray (fixed 5 cm rims 3 cm away): 0.5, at the limit and probably unsolvable, so not gated;
- cross-table: **0.4**.

**Gate:** s0 = 1.0, s_break = 0.5.

**Repair 1:**
- if the straight lift is refused, back away along the approach, then lift;
- give up an arm after 4 refused final approaches.

Result: 0.3.

**Diagnosis: the variation is flawed.** RoboPRO moves the furniture with the seed. On 5 of 10 seeds the pinned cup lay inside the low bookcase's footprint (4-5 cm overlap in x and y), in its lower compartment, under a fixed overhang. Those are exactly 5 of the 7 failures. The validity check accepted it because nothing interpenetrates. Since 0.8 was unreachable by construction, the round was closed as `unsolved` after one repair and the round-5 pipeline restored. Filed as ISSUES 8: a containment check proposal.

The run paused here and the coordinating agent was asked whether to run another round. The answer was to continue: the stop rule (5 accepted variations) was not met. It came with a time box of ~2.5 h and a requirement to validate every design on all 10 seeds before gating. The same message also suggested preferring axes that had already broken and recovered. That was design advice, and it is recorded here. Round 7 was chosen from the remaining untested assumptions, then from compositions of earlier axes.

Earlier, in round 1, the coordinating agent sent one piece of variation-design advice (avoid targets under fixed overhangs) and retracted it minutes later, for independence. It was not used. v7's overhang was unintended: the furniture moves with the seed, which is what the round-6 diagnosis found.

## 11. Round 7: v8, a mug-size cup boxed in (01:45-02:20)

Every pinned layout was first checked on all 10 seeds for validity and furniture-footprint overlap.

**Nine candidates did not break the round-5 pipeline:**
- coaster pulled 12 cm away during the carry or later: 1.0 / 1.0;
- two boxes next to each of cup and coaster: 0.8;
- espresso-size cup: 0.7;
- a box sliding across the carry path: 0.9 (its start overlapped the bookcase on 1 seed, caught by the check);
- coaster near the robot: 1.0;
- big mug: 1.0;
- slick 20 g coaster on a glossy table: 1.0;
- coaster on a book: 1.0.

**Two compositions:**
- sliding cup + boxed-in coaster: 0.8;
- mug-size cup boxed in: **0.3**.

**Gate:** s0 = 1.0, s_break = 0.3.

**Repair 1:** a profile printed the cup width the pipeline used as 0.12 m for a 7.3-8.3 cm cup. The width helper took the world-axis extent of the rotated box, which overestimates a cylinder's diameter by up to 41% at 45 deg yaw. The bug dated from round 1 and was invisible in open scenes. Fixed to the larger non-vertical object-frame extent.

Result: **base 0.9, v1 0.9, v2 0.8, v3 0.8, v6 0.9, v8 0.8.** This is the 5th accepted variation, so the run converged.

## 12. Lessons about running GAH here

- **One `--all` is noisy, about ±0.2 per scene between runs.** With 5-6 scenes, "all >= 0.8 at once" passes partly by chance, and rounds 4-5 used all 6 repairs partly for that reason (ISSUES 9).
- **The time budget is the real constraint on the harder scenes.** Most failures were `StepLimitExceeded`; refused motions that still execute part of a trajectory cost 2-6 s, and each gripper motion ~1.2 s (ISSUES 10).
- **Validate a pinned design on every seed.** The furniture moves with the seed. v5 was caught by the validity gate; v7 was not, because its cup sat in a compartment without interpenetrating.
- **Screen candidates before gating.** It separated breaks from robustness: 32 screened, 20 did not break. The screenings are also evidence of what the final pipeline withstands.
- **Infrastructure.** The renderer memory stayed flat over 575+ episodes per pool, with pools capped at 4000 and the cache cleared every 5 resets. A persistent 3-worker pool worked throughout.
- **My own slips:**
  - a broad process-match pattern killed its own remote command once;
  - an edit script failed silently once, and the old version was re-evaluated;
  - a variable rename shadowed a helper (caught before running).
