# ManiSkill3 backend (person C)

The five tasks are PegInsertionSide-v1, PlugCharger-v1, PullCubeTool-v1, LiftPegUpright-v1 and PlaceSphere-v1. They run on a single-arm Panda in SAPIEN/PhysX.

## Setup (as on jinbang-4080)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
cd GAH && uv venv -p 3.11 .venv && source .venv/bin/activate
uv pip install mani_skill==3.0.1 "numpy<2" pyyaml imageio
export CUDA_VISIBLE_DEVICES=0      # rendering needs Vulkan and the NVIDIA ICD
```

## Files

| File | Purpose |
|---|---|
| `api.py` | Primitive API (`RobotAPI`). Pipelines define `run(api)` and use only this. |
| `gah_env.py` | `make_env(task, variation)`: the task class with scene edits baked in. |
| `scene_edits.py` | Scene-edit keys (see SCENE_EDITS.md). |
| `tools/eval.py` | Evaluate `runs/<task>/pipeline.py` (`--variation`, `--all`, `--record-s0`). |
| `tools/gate.py` | Integrity, validity, variance and break gates, then the oracle. |
| `tools/oracle.py` | Official ManiSkill motion-planning solution as a solvability check. |
| `tools/integrity.py` | Writes or checks `tools/INTEGRITY.json`. |
| `examples/` | A trivial pipeline and a reference LiftPegUpright pipeline (1.0 without noise, 0.7 with the default noise). |
| `SANITY.md`, `ISSUES.md` | Sanity check results; open questions for the team. |

## Notes

- **Workers:** `eval.py` and `gate.py` run episodes in parallel processes (`--workers`, or `GAH_WORKERS`). The default is min(10, CPUs/2).
- **Frozen pipeline:** the gate uses the **committed** `runs/<task>/pipeline.py` (git HEAD) as the pre-round pipeline, so commit after Phase 1 and after every accepted round.
