# RoboPRO backend

Backend for GAH training on RoboPRO, a RoboTwin 2.0 fork with household scenes (study, office, kitchens).
The robot is aloha-agilex (two 6-DoF arms), with SAPIEN 3 physics and curobo motion planning. Pipelines
define `run(api)` and use only `api.py`.

The backend is a port of the RoboTwin 2.0 GAH backend (backend v2 of `benchmarks/robotwin`): same API,
scene-edit keys, tools and JSON formats. What is RoboPRO-specific lives in `gah_env.py` (task
construction, names, success check) and in the name resolution of `scene_edits.py`.

Training task: `put_cup_on_coaster` (study scene, clean config `bench_demo_study_clean`). Goal:
"Place the cup onto the coaster on the table."

## Setup

You need a RoboPRO checkout (customized_robotwin/ + benchmark/ with assets) and a py3.10 env with
SAPIEN 3.0.0b1, curobo and torch. Set `ROBOPRO_STACK` to a directory that holds `RoboPRO/`, `envs/robotwin_b1/` and
`tooling/vklib/` (or override the pieces with `ROBOPRO_CODE`, `SIM_PY`, `VK_LIB_DIR`, `GAH_SCRATCH`; see `env.sh`).
Run on a GPU node; curobo needs CUDA.

```bash
export ROBOPRO_STACK=/path/to/stack
source benchmarks/robopro/env.sh
cd benchmarks/robopro
bash tools/pool.sh 8                                        # warm worker pool on this GPU node
python tools/pool.py seeds --task put_cup_on_coaster --n 60  # once per task -> seeds/<task>.json
python tools/pool.py status
```

Without a running pool, `eval.py` and `gate.py` run episodes in `--local N` processes.

## Files

| File | Purpose |
|---|---|
| `api.py` | Primitive API (`RobotAPI`): perception with noise, collision-aware motion, grasp/place, checks. |
| `gah_env.py` | Builds a RoboPRO task once per worker, resets it per seed, applies scene edits. Also holds the task spec (names, regions), the success check and the expert (for seeds only). |
| `scene_edits.py` | Scene-edit keys (see SCENE_EDITS.md) and the scene-validity check. |
| `tools/eval.py` | Evaluate `runs/<task>/pipeline.py` (`--variation`, `--all`, `--seeds`, `--seed-offset`, `--record-s0`). |
| `tools/gate.py` | Integrity, edit, validity, variance and break gates. |
| `tools/pool.py`, `tools/pool.sh` | Warm worker pool; jobs, claims and results are files in `.pool/`. |
| `tools/integrity.py` | Writes or checks `tools/INTEGRITY.json`. |
| `seeds/<task>.json` | Seeds whose scene is stable and that RoboPRO's expert solves; `--seeds 10 --seed-offset k` uses entries k..k+9. |
| `examples/` | Trivial pipeline used by the sanity checks. |
| `SANITY.md`, `ISSUES.md` | Sanity-check results; open questions for the team. |

## Notes

- **Frozen pipeline:** the gate uses the **committed** `runs/<task>/pipeline.py` (git HEAD), so commit
  after Phase 1 and after every accepted round.
- **Episode record:** each seed's record has `trace` (the API calls with results and errors, i.e. the
  execution log) and `final` (ground-truth end state for the developer; the pipeline never sees it).
  Frames: `logs/frames/<task>/<variation>_<time>/seed<k>.png` (head camera after the episode).
- **Scene:** the study table is 1.2 x 0.7 m at the origin, top at z = 0.74. The robot base is at
  y = -0.65 and faces +y, and x > 0 is the robot's right. A bookcase stands at the back and a wooden box
  at the front; their sides swap with the seed's furniture layout (`scene_id` 0-2). RoboPRO places the cup
  and the coaster on the half of the table away from the bookcase.
- **Sharing the GPU:** workers run outside Slurm at nice 10. The supervisor starts a worker only while
  MemAvailable >= 40 GB and stops the newest one below 20 GB. It also stops the newest worker when the
  node stays above 89 C for 2 minutes.
