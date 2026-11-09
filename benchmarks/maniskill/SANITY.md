# Sanity checks: ManiSkill3 backend

Run on jinbang-4080 on 2026-10-23:
- 2x RTX 4080 SUPER, driver 580.178.04, 32 CPUs;
- mani_skill 3.0.1, sapien 3.0.3, mplib 0.1.1, torch 2.14.1+cu130, numpy 1.26.4 (numpy 2 breaks mplib).

The checks ran in a scratch copy of the repo, so this branch's `runs/` stays empty. Commands are relative to the repo root, with `CUDA_VISIBLE_DEVICES=0`.

## Environment

All five tasks load and render (512x512 head camera), and `api.detect` / `task_frames` work for every object.

Official motion-planning solutions (`tools/oracle.py --seeds 10`, seeds 0-9):

| Task | Oracle success |
|---|---|
| PegInsertionSide-v1 | 0.4 |
| PlugCharger-v1 | 0.8 |
| PullCubeTool-v1 | 0.9 |
| LiftPegUpright-v1 | 1.0 |
| PlaceSphere-v1 | 0.6 |

## API check

`examples/lift_peg_upright_pipeline.py` is the official LiftPegUpright solution rewritten against `api.py` only. It scores:
- **1.0** with no perception noise;
- **0.7** with the default noise (1 cm, 3 deg per detection).

It needs `grasp(..., strength=0.6)`. With a full-strength grip the peg cannot pivot in the fingers and ends tilted (success 0).

## (a) Trivial pipeline end to end: PASS

`eval.py --task LiftPegUpright-v1 --seeds 4 --pipeline benchmarks/maniskill/examples/trivial_pipeline.py`

- success_rate 0.0, no errors, about 1.3 s per episode;
- final frames written to `logs/frames/...`;
- one line appended to `logs/runs.jsonl`.

## Gate checks (LiftPegUpright-v1, frozen pipeline = the reference pipeline, s0 = 0.7)

| Check | Variation | Result |
|---|---|---|
| (b) no-op | `edits: {}` | rejected: `no-op variation: no edits` |
| break gate, not broken | `object_color` (peg green) | rejected: `s_break=0.70 > s0/2=0.35` |
| break gate, not broken | `dynamics` (peg mass 3 kg, friction 0.2) | rejected: `s_break=0.60 > s0/2=0.35` |
| pass, oracle cannot solve | `object_motion` (peg slides at 0.25 m/s for 0.6 s) | **pass**, s_break=0.0, oracle 0.0, `needs_agent_solve: true` |
| (c) variance gate | 3rd `parameter_tuning: true` in a row | rejected: `variance gate: third parameter-tuning variation in a row` |
| (c) automatic tuning detection | same as above but labelled `false`; same keys as the previous one, only numbers changed | rejected, `parameter_tuning_auto: true` |
| (d) integrity, tool file | `tools/eval.py` modified | rejected: `integrity: modified ['tools/eval.py']` |
| (d) integrity, success check | stored hash of `LiftPegUpright.evaluate` changed | rejected: `integrity: modified ['evaluate:LiftPegUpright-v1']` |
| (e) impossible variation | peg moved to `xyz [1.3, 0, 0.025]` | gate rejected: `invalid scene: peg is out of reach (1.91 m from the base)`; `oracle.py` reports `oracle_success_rate 0.0`, `solvable: false` |

The (c) streak test seeded `gate_history.jsonl` with two accepted tuning variations, because only accepted variations count toward the streak.

After restoring the files, `tools/integrity.py` reports `{"ok": true}`.
