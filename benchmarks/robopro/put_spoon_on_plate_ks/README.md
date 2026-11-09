# RoboPRO backend (benchmark D)

Backend for GAH training on [RoboPRO](https://github.com/EAI-RSM/RoboPRO), a RoboTwin 2.0 fork with
household scenes (kitchens, kitchenl, office, study): aloha-agilex (two 6-DoF arms), SAPIEN 3.0.0b1 physics,
curobo motion planning. Pipelines define `run(api)` and use only `api.py`.

The backend is a port of the RoboTwin 2.0 backend (`benchmarks/robotwin`, backend v2 of the
put_object_cabinet branch): same API, same tools, same flags and JSON. What changed is the task layer
(`gah_env.py`: RoboPRO bench tasks, the kitchen furniture as named bodies) and the scene-edit names and
validity rules (`scene_edits.py`, see SCENE_EDITS.md).

## Setup

Needs a RoboPRO checkout (`customized_robotwin/` + `benchmark/` with assets) and the RoboPRO simulator
python (py3.10, SAPIEN 3.0.0b1, curobo; on aarch64 there is no mplib and curobo plans every motion). Put
both paths in `env.local.sh` (not committed):

```bash
export ROBOPRO_ROOT=<RoboPRO checkout>
export SIM_PY=<simulator env>/bin/python
```

Then, on a GPU node (curobo needs CUDA):

```bash
cd benchmarks/robopro && source env.sh
python tools/pool.py seeds --task put_spoon_on_plate_ks --n 40   # once per task -> seeds/<task>.json
bash tools/pool.sh 4 <slurm-node-name>                           # warm worker pool on this node
python tools/pool.py status
```

Without a running pool, `eval.py` / `gate.py` run episodes in `--local N` processes.

## Files

| File | Purpose |
|---|---|
| `api.py` | Primitive API (`RobotAPI`): perception with noise, collision-aware motion, grasp/place, checks. |
| `gah_env.py` | Builds a RoboPRO bench task once per worker, resets it per seed, applies scene edits; task spec (names of objects and furniture), success check, seed oracle (RoboPRO's scripted solution). |
| `scene_edits.py` | Scene-edit keys (see SCENE_EDITS.md) and the scene-validity check. |
| `env.sh` | Environment (paths from env.local.sh, Vulkan, OptiX denoiser, render-pool cap, node-local scratch). |
| `tools/eval.py` | Evaluate `runs/<task>/pipeline.py` (`--variation`, `--all`, `--seeds`, `--seed-offset`, `--record-s0`). |
| `tools/gate.py` | Integrity, edit, validity, variance and break gates. |
| `tools/pool.py`, `tools/pool.sh` | Warm worker pool through files in `.pool/`; `pool.py seeds` writes the seed list. |
| `tools/integrity.py` | Writes or checks `tools/INTEGRITY.json`. |
| `seeds/<task>.json` | Stable, expert-solvable seeds; `--seeds 10 --seed-offset k` uses entries k..k+9. |
| `examples/` | Trivial pipeline used by the sanity checks. |
| `SANITY.md`, `ISSUES.md` | Sanity-check results; open questions for the team. |

## Notes

- **Frozen pipeline:** the gate uses the **committed** `runs/<task>/pipeline.py` (git HEAD), so commit
  after Phase 1 and after every accepted round.
- **Seeds:** RoboPRO's frozen clean seed bank of the task first (`benchmark/eval_seeds/<task>/`), then the
  integers after it; a seed is kept if its scene is stable and RoboPRO's own scripted solution succeeds on
  it, so every base-task seed is solvable. The clean scene config is `bench_demo_kitchens_clean` (no clutter).
- **Episode record:** each seed's record has `trace` (the API calls with results and errors, i.e. the
  execution log) and `final` (ground-truth end state for the developer; the pipeline never sees it).
  Frames: `logs/frames/<task>/<variation>_<time>/seed<k>.png` (head camera after the episode).
- **Speed:** a task object takes ~20 s to warm up (curobo), an episode 15-60 s of wall clock.
- **Sharing GPUs:** workers run outside Slurm at nice 10. The supervisor starts a worker only while
  MemAvailable >= `--start-gb`, stops the newest below `--min-gb`, waits `--stagger` seconds between starts,
  and drops to `--guest-slots` (0 by default in pool.sh) while another user's Slurm job runs on the node.
  Beside an owner's job, size the pool inside the memory the job has not requested and keep the start
  threshold more than one worker's warm-up (about 13.6 GB: 4.2 GB host + 9.3 GB GPU on a GB10) above the
  stop threshold (ISSUES 12).
