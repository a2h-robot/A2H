# Sanity checks (RoboPRO backend, put_cup_on_coaster)

Run 2026-10-23 on a GB10 (aarch64, one Blackwell GPU) with a 3-worker pool. Inputs and outputs are in `sanity/`
(worker host names in the recorded outputs are redacted to `node-<slot>`).

| Check | Command | Result |
|---|---|---|
| (a) a trivial pipeline runs end to end through eval.py | `eval.py --task put_cup_on_coaster --pipeline examples/trivial_pipeline.py --seeds 3` | PASS: 3 episodes ran (~325 steps each, no error), success 0.0 as expected; the trace records `detect('cup')` and `move_by(dz=0.05, arm='left') -> True`. Logged to logs/runs.jsonl. |
| (b) gate.py rejects a no-op variation | `gate.py --variation sanity/noop.yaml` (`edits: {}`) | PASS: `"pass": false, "reason": "no-op variation: no edits"` |
| (c) gate.py rejects a third consecutive parameter-tuning variation | history with two `parameter_tuning: true` entries (`sanity/c_fake_gate_history.jsonl`), then `gate.py --variation sanity/tune3.yaml` | PASS: `"tuning_streak_before": 2, "reason": "variance gate: third parameter-tuning variation in a row"` |
| (d) gate.py rejects when the success check is modified | hash of `check_success:put_cup_on_coaster` in tools/INTEGRITY.json replaced, then `gate.py --variation sanity/integ.yaml` | PASS: `"reason": "integrity: modified ['check_success:put_cup_on_coaster']"` |
| (d') gate.py rejects when a protected backend file is modified | one comment line appended to gah_env.py | PASS: `"reason": "integrity: modified ['gah_env.py']"`; file and INTEGRITY.json restored, `integrity.py` -> ok |
| (e) every scene-edit key applies and passes validity | `sanity/all_keys.py`: each of the 10 keys alone on bank seed 0 (including `swap_object` cup -> `039_mug`) | PASS: all 10 `valid: true` |

Also checked while bringing the backend up (probe of the stock scene, 3-4 seeds):
- **Task objects match the success check.** The coaster lies flat: its extents are 10.6 x 10.6 x 0.8 cm, and its actor origin equals its bounding-box centre in xy. The cup's origin is its bottom centre, with the same xy as its box. So "cup centred on the coaster" in `detect()` coordinates is what `check_success` measures (ISSUES 2 resolved).
- **The expert works.** RoboPRO's expert solves a scene in 4-8 s of wall time on its own. It ends 0.5-1.5 cm from the coaster centre, inside the 2 cm tolerance.
- **Seed bank.** RoboPRO's expert failed on 24 of 90 scanned seeds (`IK_FAIL`, or no valid pre-grasp pose); none was unstable. `seeds/put_cup_on_coaster.json` keeps the first 60 solved seeds.
- **Cost.** A worker takes ~10 GB of node memory (unified CPU/GPU memory; host PSS 4.8 GB plus 9.5 GB GPU at peak). The first reset takes ~20 s (curobo warm-up), later resets 0.3-0.5 s. Three busy workers already hold the GPU at ~96%.

Found and fixed during the checks: the seed-bank expert job imported RoboPRO's `envs` before changing into the RoboPRO root. That killed the workers and left their claimed jobs orphaned. The import order is fixed, and a worker now records any job exception as a failed episode instead of dying.
