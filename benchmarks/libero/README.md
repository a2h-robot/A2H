# LIBERO-Goal backend (person A)

The five training tasks are `open_the_middle_drawer_of_the_cabinet`, `turn_on_the_stove`, `push_the_plate_to_the_front_of_the_stove`, `put_the_wine_bottle_on_the_rack` and `put_the_cream_cheese_in_the_bowl`. They run on a single-arm Panda in robosuite 1.4.1 / MuJoCo 3.2.3, with the OSC_POSE controller at 20 Hz as in the official benchmark.

## Setup

```bash
# Clones LIBERO next to <dir> and creates the conda env `libero` (Python 3.10, torch 2.7.1+cu128).
bash benchmarks/libero/install_libero.sh <dir> ~/miniconda3/etc/profile.d/conda.sh
conda activate libero
```

The installer:
- patches one line of LIBERO, `torch.load(..., weights_only=False)` for the init states, because torch ≥ 2.6 refuses them otherwise;
- writes the LIBERO config inside the env and points `LIBERO_CONFIG_PATH` at it. It never touches `~/.libero`;
- sets `MUJOCO_GL=egl` and `PYTHONNOUSERSITE=1`. On shared accounts, `~/.local` packages would otherwise shadow the env.

Installed on:
- the local machine: `~/Desktop/workspace/LIBERO`, env `libero`;
- `<sim-host>`: `~/LIBERO`, env `libero` in `~/miniforge3`.

## Running on <sim-host>

Run Claude Code in the local checkout, which can push to GitHub, and run the simulation on <sim-host>:

```bash
benchmarks/libero/remote.sh python benchmarks/libero/tools/eval.py --task T --seeds 10
benchmarks/libero/remote.sh python benchmarks/libero/tools/gate.py --task T --variation benchmarks/libero/runs/T/variations/v1.yaml
```

`remote.sh` does three things:
- syncs the checkout (with `.git`) to `<sim-host>:~/gah_runs/<checkout>-<hash>/`;
- runs the command in the `libero` env;
- copies `logs/` and `runs/` back.

Commit locally, then run the gate, because the gate reads the committed pipeline.

## Files

| File | Purpose |
|---|---|
| `api.py` | Primitive API (`RobotAPI`). Pipelines define `run(api)` and use only this. |
| `motion.py` | IK, scene collision checking and RRT-Connect, used by `api.move_to`. |
| `gah_env.py` | `make_env(task, variation, seed)`: LIBERO-Goal env with the scene edits applied, init state `seed % 50`. |
| `scene_edits.py` | Scene-edit keys (see SCENE_EDITS.md). |
| `tools/eval.py` | Evaluate `runs/<task>/pipeline.py` (`--variation`, `--all`, `--record-s0`, `--pipeline`, `--seed-offset`, `--video`: an mp4 per seed under `frames_dir`, head camera, 5 fps = real time). |
| `tools/gate.py` | Integrity, edit, validity, variance and break gates. |
| `tools/integrity.py` | Writes or checks `tools/INTEGRITY.json`. |
| `remote.sh` | Run a tool command on <sim-host> (see above). |
| `examples/` | A trivial pipeline, and a reference pipeline for the held-out task `put_the_bowl_on_the_plate`. |
| `SANITY.md`, `ISSUES.md` | Sanity check results; open questions for the team. |

## Notes

- **Seeds:** seed `s` uses LIBERO's fixed init state `s % 50` (official placements) and perception-noise seed `10000 + s`. `--seed-offset 10` evaluates on unseen init states.
- **Episode:**
  - The scene settles for 1 s (LIBERO init states hold objects about 7 cm above the table).
  - Then the edits are applied, and the scene settles for another 1 s.
  - Then the pipeline runs, with a limit of 3000 control steps (150 s).
  - Success is LIBERO's own BDDL goal check after a 0.5 s hold.
- **Workers:** episodes run in parallel processes (`--workers`, or `GAH_WORKERS`). The default is min(10, CPUs/2).
- **Frozen pipeline:** the gate uses the **committed** `runs/<task>/pipeline.py` (git HEAD) as the pre-round pipeline.
- **Frames:** the final head-camera frame of every seed goes to `logs/frames/<task>/<variation>_<time>/` (not committed).
