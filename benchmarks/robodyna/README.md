# RoboDyna backend (person E)

[RoboDyna](https://github.com/EAI-RSM/RoboDyna) is a dual-arm benchmark for dynamic scenes, built on RoboTwin 2.0: SAPIEN 3.0.3 / PhysX, cuRobo, and two UR5 arms with WSG grippers (`ur5-wsg`). Objects move and fall, belts run, fruit ripens, and the scene keeps changing while the robot acts.

The backend is a port of the RoboTwin 2.0 GAH backend v2 (`b-robotwin-hanging_mug`, 11b8b4a). It keeps the same API, scene-edit keys, tools and JSON output. The RoboDyna-specific code lives in `gah_env.py` (task setup, names, live world, success latching) and in small parts of `api.py` and `scene_edits.py`.

## Training tasks

All five run in RoboDyna's **Opt 1+2** Base condition. There is one task per RoboDyna skill category (`interactive/experiment.yml`).

| Task | Category | Opt 1+2 adds | Names for detect() |
|---|---|---|---|
| `catch_valley_ball` | motion prediction | wall rebound, black distractor ball | `red ball`, `blue box`, `black ball`; part `red line` |
| `pick_ripe_apple` | state transition | spoiled apple, oscillating basket | `left apple`, `right apple`, `basket`; part `tree` |
| `drop_ball_hole` | dynamic pattern | sticky surface, decoy hole | `ball`; parts `sorter`, `container floor`, `hole A`, `hole B` |
| `place_block_belt` | dynamic avoidance | moving bowl, belt blocker | `block`, `bowl`, `blocker`; parts `belt`, `place line`, `belt support` |
| `catch_shelf_marble` | spatial reasoning | marble moves at once, oscillating shelf | `red ball`, `bowl`; parts `belt`, `left key`, `right key` |

The names describe what can be seen, never the answer:
- In `pick_ripe_apple` the pipeline must tell the good apple from the spoiled one by colour over time (`ObjectState.rgb`).
- In `drop_ball_hole`, `hole A` and `hole B` are the target hole and the decoy in a random order per seed. The decoy is cut smaller than the ball, so `bbox_extents` tells them apart.

## Results

- **Training runs:** `runs/<task>/`, each with `summary.md`, `reflections.jsonl`, `variations/` and `pipeline.py`. Each run is also on its own branch, `e-robodyna-<task>`.
- **Statistics:** [STATS.md](STATS.md), with per-task gate outcomes, unsolved rounds, tokens and wall clock.

## Setup (<sim-host>)

- **RoboDyna:** `main` at **7931bb6e** or later, at `~/RoboDyna` on <sim-host> (`ROBODYNA_ROOT`).
  - 954af13e merged the expert-free eval gates; 5303d554 and 399878ab fixed `catch_valley_ball` (box stayed kinematic) and `pick_ripe_apple` (picked apples floated or fell through the closing fingers); see the tracker in ISSUES.md (`_sync_eval_gates`) from `dualmode-eval`. Before that, 4 of the 5 tasks only advanced their state inside RoboDyna's scripted expert, so no other program could succeed.
  - Assets: the verified `robodyna-assets-v1` package (sha256 `fe3444f1…`).
- **Conda env `gah_robodyna`** (a clone of the shared `robodyna` env, which is left untouched):
  - SAPIEN 3.0.3, torch 2.4.1+cu121;
  - cuRobo v0.7.7, built with CUDA 12.4 for sm_89 and patched like RoboDyna's installer (world_mesh `clear_cache`);
  - `PYTHONNOUSERSITE=1`, because `~/.local` on the shared account shadows conda packages.
- **Running:** run Claude Code in a local checkout that can push to GitHub, and run the simulation on <sim-host>:

```bash
benchmarks/robodyna/remote.sh python benchmarks/robodyna/tools/eval.py --task T --seeds 10
benchmarks/robodyna/remote.sh python benchmarks/robodyna/tools/gate.py --task T --variation benchmarks/robodyna/runs/T/variations/v1.yaml
```

`remote.sh` does three things:
- syncs the checkout (with its commits) to `<sim-host>:~/gah_runs/<checkout>-<hash>/`;
- runs the command in `gah_robodyna` with `GAH_WORKERS=4` (two per GPU);
- copies `logs/` and `runs/` back.

Commit before gating: the gate reads the committed pipeline.

## Files

| File | Purpose |
|---|---|
| `api.py` | Primitive API (`RobotAPI`). Pipelines define `run(api)` and use only this. |
| `gah_env.py` | `GahEnv(task)`: persistent RoboDyna task env; `reset(seed, variation)` with scene edits; live world (`tick`); step budget; success latching. |
| `scene_edits.py` | Scene-edit keys (see SCENE_EDITS.md) and the validity check. |
| `seeds/eval_seeds_d25d0aeb.yml` | Snapshot of RoboDyna's `task_config/eval_seeds.yml` (its policy-evaluation and human-experiment seeds). GAH uses the Opt 1+2 lists as they are. |
| `tools/eval.py` | Evaluate `runs/<task>/pipeline.py` (`--variation`, `--all`, `--record-s0`, `--video`, `--pipeline`). |
| `tools/gate.py` | Gates, in order: integrity, edits, variance, validity, break. |
| `tools/integrity.py` | Writes or checks `tools/INTEGRITY.json`. |
| `remote.sh` | Runs a tool command on <sim-host>. |
| `examples/` | Trivial pipeline, backend probe, belt smoke test and its diagnostic. |
| `sanity/` | Variations and script for SANITY.md. |
| `SANITY.md`, `SCENE_EDITS.md`, `ISSUES.md` | Sanity results, edit keys, open questions. |

## How an episode works

- **Seeds.** Seed k is entry k of the task's Opt 1+2 list in the seed snapshot. There are 10 per task, so `--seeds` plus `--seed-offset` must stay ≤ 10. Perception noise uses seed `10000 + seed`.
- **Reset.** RoboDyna's policy-evaluator setup is used (`eval_mode`, `is_test`, `demo_dynamic`, Opt 1+2 flags). Scene edits run inside the task's `load_actors()`, before RoboDyna's settle and stability check. The scene is live when `run(api)` starts.
- **Live world.**
  - Every physics step runs RoboDyna's kinematic tasks and expert-free gates. That includes the waits the API does (`wait`, `wait_until`, `track`), not only motion.
  - `api.time()` is sim time.
- **Success.**
  - Like RoboDyna's evaluator, the task's own `check_success()` runs after every physics step, and the first success is latched.
  - After `run(api)` returns, the world keeps running with the arms holding still. It stops at success, after 10 s, or at the budget. This matters, for example, when a released block still has to ride the belt into the bowl.
- **Budget.** 15000 physics steps (60 s at 250 Hz), the same number as RoboDyna's `eval_step_limit`.
- **No solvability oracle.** RoboDyna's scripted expert is not used. A variation the agent cannot recover within MAX_REPAIR is marked unsolved (prompts/gah_train.md).
- **Workers.** Persistent, GPU-pinned and recycled every 8 episodes (the renderer leak). Logs go to `logs/workers/`.
- **Frames.** Start and end head-camera images and the API trace per seed, under `logs/frames/` (not committed).
