# RoboPRO backend

Backend for GAH training on RoboPRO, a RoboTwin 2.0 fork with kitchen, office and study benchmark scenes:
aloha-agilex (two 6-DoF arms), SAPIEN 3 physics, curobo motion planning. Pipelines define `run(api)` and use
only `api.py`. The tools are the RoboTwin backend's (person B), adapted to RoboPRO's scenes.

## Setup

Use an existing RoboPRO install (RoboPRO `customized_robotwin/` + `benchmark/` with assets, SAPIEN 3.0.0b1,
curobo). Its env script must export `ROBOTWIN_ROOT` (= `customized_robotwin/`), `BENCH_ROOT`
(= `benchmark/`) and `SIM_PY` (the simulator's python). Run on GPU nodes only (curobo needs CUDA).

```bash
export RP_ENV=<install>/env.sh && source $RP_ENV
cd benchmarks/robopro
bash tools/pool.sh 5 [<slurm-node-name>]                            # warm worker pool on each GPU node
python tools/pool.py seeds --task pick_boxdrink_from_basket --n 80   # once per task -> seeds/<task>.json
python tools/pool.py status
```

Without a running pool, `eval.py` / `gate.py` / `oracle.py` run episodes in `--local N` processes.

## Files

| File | Purpose |
|---|---|
| `api.py` | Primitive API (`RobotAPI`): perception with noise, collision-aware motion, grasp/place, checks. |
| `gah_env.py` | Builds a RoboPRO task once per worker, resets it per seed, applies scene edits; task spec (names, containers, regions), success check, scripted expert, backend fixes to RoboPRO's scene setup. |
| `scene_edits.py` | Scene-edit keys (see SCENE_EDITS.md) and the scene-validity check. |
| `tools/eval.py` | Evaluate `runs/<task>/pipeline.py` (`--variation`, `--all`, `--seeds`, `--seed-offset`, `--record-s0`). |
| `tools/gate.py` | Integrity, edit, validity, variance and break gates, then the (informational) expert oracle. |
| `tools/oracle.py` | RoboPRO's scripted expert on a task or variation (solvability evidence). |
| `tools/pool.py`, `tools/pool.sh` | Warm worker pool shared by several GPU nodes through files in `.pool/`; `seeds` builds the seed list. |
| `tools/integrity.py` | Writes or checks `tools/INTEGRITY.json`. |
| `seeds/<task>.json` | RoboPRO clean-config seeds that are stable here and solved by the expert; `--seeds 10 --seed-offset k` uses entries k..k+9. |
| `examples/` | Trivial pipeline used by the sanity checks. |
| `SANITY.md`, `ISSUES.md`, `SCENE_EDITS.md` | Sanity-check results; open questions and backend fixes; edit keys. |

## Notes

- **Frozen pipeline:** the gate uses the **committed** `runs/<task>/pipeline.py` (git HEAD), so commit
  after Phase 1 and after every accepted round.
- **Episode record:** each seed's record has `trace` (the API calls with results and errors, i.e. the
  execution log) and `final` (ground-truth end state for the developer; the pipeline never sees it).
  Frames: `logs/frames/<task>/<variation>_<time>/seed<k>.png` (countertop camera after the episode).
- **One hardware type per task.** RoboPRO's base classes differ between the x86 and the aarch64 installs
  (different revisions and platform patches), so s0, gate and recovery numbers of one task must all come
  from the same hardware.
- **Sharing GPUs:** workers run at nice 10. On a GB10 the node's memory guard kills the largest Slurm job,
  not a guest, so the pool leaves headroom: with no Slurm job it starts a worker only while MemAvailable >=
  `START_GB` and stops the newest below 20 GB; with a Slurm job on the node it runs at most `GUEST_SLOTS`
  workers and keeps MemAvailable >= 60 GB; a job named `bb*` (the backbone study) stops it. A busy worker
  costs ~10 GB of node memory (GPU allocations included).
