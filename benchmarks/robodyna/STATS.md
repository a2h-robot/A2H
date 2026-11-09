# GAH training statistics: RoboDyna (person E)

Five RoboDyna Base tasks in the Opt 1+2 condition. Each run was one Claude Opus 5.5 agent following
`prompts/gah_train.md` (TAU = 0.8, SEEDS = 10, MAX_ROUNDS = 5, MAX_REPAIR = 6). The agents ran
2026-10-23/24 on RoboDyna main, which was fixed during the runs: 5303d554, then 399878ab; see
ISSUES.md item 1.

Run codes used in the harness source map: CV = catch_valley_ball, PA = pick_ripe_apple,
DB = drop_ball_hole, PB = place_block_belt, CS = catch_shelf_marble. The number after the code is the
round in that task's `runs/<task>/reflections.jsonl`.

## Per task

| Task | s0 | Variations designed (gate submissions) | Passed gates | Recovered | Rejected: break gate | Rejected: variance gate | Rejected: integrity / validity (no oracle) | Unsolved | Rounds / stop reason | Agent tokens | Wall clock |
|---|---|---|---|---|---|---|---|---|---|---|---|
| catch_valley_ball (CV) | 0.8 | 7 (7) | 5 | 4 | 2 | 0 | 0 / 0 | 1 | 5 rounds; stopped by the agent after its last two designs failed the break gate. Not converged by the novelty rule (all 5 reflections novel) | 607,570 | 9.9 h |
| pick_ripe_apple (PA) | 1.0 | 6 (7) | 3 | 0 | 3 | 0 | 0 / 1 | 3 | 3 rounds; stopped by the agent, not converged (only clutter still broke the pipeline, and it was unsolved each time) | 534,641 | 2.6 h |
| drop_ball_hole (DB) | 1.0 | 6 (7) | 5 | 4 | 1 | 0 | 1 / 0 | 1 | 5 rounds; stopped at 5 gated rounds (round 5 unsolved) | 432,941 | 4.5 h |
| place_block_belt (PB) | 1.0 | 11 (12) | 5 | 4 | 6 | 0 | 0 / 1 | 1 | 5 rounds; **converged**: two consecutive non-novel rounds (4, 5) | 491,330 | 3.5 h |
| catch_shelf_marble (CS) | 0.8 | 8 (8) | 3 | 1 | 5 | 0 | 0 / 0 | 2 | 3 rounds; stopped by the agent, not converged (the editable axes no longer broke the pipeline) | 464,837 | 3.9 h |
| **Total** | | **38 (41)** | **21** | **13** | **17** | **0** | **1 / 2** | **8** | | **2,531,319** | **24.3 h** |

How to read the columns:
- **Variations designed** counts distinct designs. Gate submissions also count re-submissions: DB v1 was first rejected by the integrity check after a RoboDyna update, and PA v4 and PB v10 were first invalid scenes, redesigned under the same id.
- **Passed gates** is the number of accepted rounds, each with a reflection. **Recovered** is the passed rounds the agent repaired to ≥ 0.8 on every scene. **Unsolved** rounds are marked `unsolved: true` and reverted.
- **Integrity**: DB v1 was rejected because RoboDyna changed mid-run and the branch did not yet have the new INTEGRITY.json. It is a process artefact, not tampering.
- **Agent tokens and wall clock** are the agent harness's own figures at each agent's final hand-back.
  - They include Phase 1 and the sessions blocked by RoboDyna bugs: CV ran a pre-fix session, and PA ran two.
  - Wall clock includes waiting on the shared <sim-host> GPUs, and a worker-timeout stall in CV.
  - The token figures are not a billing total.
- **Logged scene evaluations in `logs/runs.jsonl`** (eval / gate): CV 93 / 12, PA 51 / 9, DB 80 / 11, PB 29 / 16, CS 41 / 10. These include diagnostic `--pipeline` runs.

## Reflections per axis

| Axis | Reflections | Recovered | Unsolved | IDs |
|---|---|---|---|---|
| obstacle | 9 | 6 | 3 | CV2, CV5, PA1, PA2, DB2, DB3, PB2, PB3, CS1 |
| object_motion | 4 | 2 | 2 | CV3, CV4, DB1, CS3 |
| distractor | 4 | 1 | 3 | PA3, DB4, DB5, CS2 |
| lighting | 2 | 2 | 0 | PB1, PB5 |
| object_position | 1 | 1 | 0 | CV1 |
| camera | 1 | 1 | 0 | PB4 |
| orientation, color, size, dynamics | 0 | – | – | designed, but rejected by the break gate (did not break the pipeline) or not available (`object_scale` is unsupported on RoboDyna) |
| **Total** | **21** | **13** | **8** | 18 novel, 3 not novel (PA2, PB4, PB5) |

## Caveats

- **Noisy success rates.** On the same code, a 10-seed rate moved by up to ±0.2 between reruns, and up to 0.5–0.9 on some variations. Part of this is shared-GPU out-of-memory errors at reset, which count as failures. s0 and the break gate are therefore approximate; several recoveries passed at exactly 0.8.
- **Three runs stopped before the convergence rule**: CV at 5 gated rounds but all novel, PA and CS at 3. Their summaries explain why. They can be resumed from their branches (`e-robodyna-<task>`).
- **The planner does not model the gripper or the held object.** Every run hit this. It is the main cause of the unsolved clutter rounds (ISSUES.md), and the harness's first rules address it.
