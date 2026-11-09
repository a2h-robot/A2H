# Sanity checks (RoboPRO backend, put_spoon_on_plate_ks)

Run 2026-10-23 on a GB10 (aarch64, curobo only) with a 3-worker pool. Script `sanity/sanity.sh` (run from the
repo root with `benchmarks/robopro/env.sh` sourced); outputs in `sanity/`.

| Check | Command | Result |
|---|---|---|
| (a) a trivial pipeline runs end to end through eval.py | `eval.py --task put_spoon_on_plate_ks --pipeline examples/trivial_pipeline.py --seeds 3` | PASS: 3 episodes ran with no error, success 0.0 as expected; trace recorded `detect('spoon')` and `move_by -> True` (`sanity/a.json`). Logged to logs/runs.jsonl. |
| (b) gate.py rejects a no-op variation | `gate.py --variation sanity/noop.yaml` (`edits: {}`) | PASS: `"pass": false, "reason": "no-op variation: no edits"` |
| (c) gate.py rejects a third consecutive parameter-tuning variation | history with two `parameter_tuning: true` entries (`sanity/history_two_tuning.jsonl`), then `gate.py --variation sanity/tune3.yaml` | PASS: `"tuning_streak_before": 2`, label and automatic diff both say tuning, `"reason": "variance gate: third parameter-tuning variation in a row"` |
| (d) gate.py rejects when the success check is modified | `eps1 = 0.03 -> 0.05` in `put_spoon_on_plate_ks.check_success`, in a shadow copy of the RoboPRO benchmark tree (`BENCH_ROOT=<shadow>`) | PASS: `"reason": "integrity: modified ['check_success:put_spoon_on_plate_ks']"` |
| (d') gate.py rejects when a protected tool file is modified | one comment line appended to tools/pool.py | PASS: `"reason": "integrity: modified ['tools/pool.py']"`; file restored, `integrity.py` -> ok |
| every scene-edit key applies | `sanity/all_keys.yaml` (all 10 keys in one variation) on the first 3 seeds, plus one episode of the trivial pipeline | PASS: the edits apply and the episode runs; validity ok on seeds 40002 and 40003, and on 40005 it rejects the scene because the rescaled, shifted spoon moved 3.4 cm while settling (the check working as intended) (`sanity/all_keys.out`) |

Found and fixed during the checks: RoboPRO's `create_cylinder` builds dynamic bodies only (no `is_static`), so
`scene_edits` builds cylinders itself.

Also checked while building the backend:
- RoboPRO's scripted solution solves the task on the first seeds (it is the seed filter, see README);
- the Phase-1 pipeline reaches 0.9 on 10 seeds once `detect()` reports the model frame of task objects
  (ISSUES 3).
