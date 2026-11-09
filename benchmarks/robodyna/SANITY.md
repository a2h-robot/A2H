# Sanity checks (RoboDyna backend)

Run on <sim-host> (RTX 4080 SUPER ×2) on 2026-10-23, with RoboDyna main 954af13e, env `gah_robodyna` and the backend in this commit.

| Check | Command | Result |
|---|---|---|
| Integrity | `tools/integrity.py` | `{"ok": true}`: 21 hashes (backend, tools, seed snapshot, the 5 success checks and task files, RoboDyna `_base_task.py`, `scenario_overrides.py`, `demo_dynamic.yml`) |
| (a) trivial pipeline end to end | `tools/eval.py --task T --seeds 4 --pipeline examples/trivial_pipeline.py` for all 5 tasks | Runs on every task; success 0/4 each, with RoboDyna's own fail reasons (e.g. `ball_not_in_box`, `good apple not in basket`) |
| (b) no-op variation | `sanity/noop.yaml` | rejected: `no-op variation: no edits` |
| (b') unsupported key | `sanity/unsupported.yaml` (`object_scale`) | rejected: `object_scale is not supported on RoboDyna` |
| (c) third parameter-tuning variation | `sanity/tune3.yaml` after `sanity/history_two_tuning.jsonl` | rejected: `variance gate: third parameter-tuning variation in a row` |
| (d) success check modified | `in_bowl` → always true, in a shadow RoboDyna tree | rejected: `integrity: modified ['check_success:place_block_belt', 'task_file:place_block_belt']` |
| (d') tool file modified | a line appended to `tools/eval.py` | rejected: `integrity: modified ['tools/eval.py']` |
| All supported edit keys at once | `sanity/all_keys.yaml` on `place_block_belt` | Scene loads and passes RoboDyna's stability check; distractor and lighting visible in the start frame |

(b)–(d) are scripted in `sanity/sanity.sh`.

## Backend probe (`examples/probe_api.py`)

On each task's first seed:
- **Objects and parts:** every named object and part is detected with a plausible pose, size and colour.
  - `hole A` (4.6 cm, the decoy) and `hole B` (6.0 cm) differ in size, as in RoboDyna (the ball is 5 cm).
  - The `place_block_belt` block reads 4.0 × 4.0 × 3.7 cm. Its extents come from the collision shapes, because its model data has none.
- **Camera:** the head camera renders 320×240.
- **Planner:** a box placed on a reachable target blocks the plan; the same box 40 cm away does not.

## Gripper calibration (ur5-wsg)

| Gripper state | `gripper_opening()` |
|---|---|
| open | 0.929 |
| closed on nothing | 0.186 |
| closed, grazing past the block (a miss) | 0.206 |
| closed on the 4 cm block | 0.676 |

`gripper_holding()` uses a threshold of 0.30.

## A non-expert program can succeed

`examples/belt_smoke_pipeline.py` is a crude scripted `place_block_belt` attempt that uses only the API: grasp, lift, set the block down before the red line in the lane away from the blocker, release, wait.

On seed 10002 (`examples/diag_belt.py`), RoboDyna's own check reports **success**:
1. 6.2 s: the block is released on the belt.
2. The task's latches fire: `_released`, `_belt_active`, `dropped_at_start_left`, `placed_on_belt`.
3. The block rides into the moving bowl (`in_bowl`).

Without 954af13e's expert-free gates, `_released` and `_belt_active` are only set inside RoboDyna's expert, and no other program can succeed.

The crude attempt scores 0/10 on the evaluation seeds. Its grasps miss on some seeds, for example the right arm on mirrored layouts, and it ignores the moving bowl. That is the training loop's job, not the backend's.
