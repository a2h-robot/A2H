# Sanity checks (RoboPRO backend, pick_boxdrink_from_basket)

Run 2026-10-23 on one GB10 (aarch64) with a 5-worker pool, `sanity/run_sanity.sh`. Outputs and inputs are
in `sanity/`. The sanity gate decisions are kept out of the training record (`sanity/gate_log_sanity.jsonl`).

| Check | Command | Result |
|---|---|---|
| (a) a trivial pipeline runs end to end through eval.py | `eval.py --task pick_boxdrink_from_basket --pipeline examples/trivial_pipeline.py --seeds 3` | PASS: 3 episodes ran (324-338 steps each, no error), success 0.0 as expected; the trace recorded `detect('boxdrink')` and `move_by(dz=0.05, arm='left') -> True`. Logged to logs/runs.jsonl. |
| (b) gate.py rejects a no-op variation | `gate.py --variation sanity/noop.yaml` (`edits: {}`) | PASS: `"pass": false, "reason": "no-op variation: no edits"` |
| (c) gate.py rejects a third consecutive parameter-tuning variation | history with two `parameter_tuning: true` entries (`sanity/c_fake_gate_history.jsonl`), then `gate.py --variation sanity/tune3.yaml` | PASS: `"pass": false, "tuning_streak_before": 2, "reason": "variance gate: third parameter-tuning variation in a row"` |
| (d) gate.py rejects when the success check is modified | hash of `check_success:pick_boxdrink_from_basket` in tools/INTEGRITY.json replaced, then `gate.py --variation sanity/integ.yaml` | PASS: `"reason": "integrity: modified ['check_success:pick_boxdrink_from_basket']"` |
| (d') gate.py rejects when a protected backend file is modified | one comment line appended to gah_env.py | PASS: `"reason": "integrity: modified ['gah_env.py']"`; file and INTEGRITY.json restored, `integrity.py` -> ok |

The success-check hash covers `check_success` and the helper it calls (`_is_boxdrink_inside_basket`).

Also checked while building the backend:
- Scene setup: without the spawn fix the box drink is thrown out of the basket on every seed tested (it
  starts 2.2 cm inside the basket floor; ISSUES 2). With the 3 cm lift it rests upright in the basket.
- Solvability: RoboPRO's scripted expert solves 32 of 60 candidate seeds (RoboPRO's 20-seed clean bank plus
  40 sequential seeds); those 32 form `seeds/pick_boxdrink_from_basket.json` (16 with the basket on the
  table, 16 with it on the microwave). The expert fails every seed of the third layout (basket in the middle
  of the table), so that layout is not in the seed list (ISSUES 9).
- Episode cost on one GB10: ~13 s first setup per worker, then ~1 s per reset; an expert episode ~15 s
  alone. A busy worker uses ~10 GB of node memory (GPU allocations included); 5 workers hold the GPU at
  ~96%.
