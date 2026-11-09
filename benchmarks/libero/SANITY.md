# Sanity checks: LIBERO-Goal backend

Run on <sim-host> on 2026-10-23:
- 2x RTX 4080 SUPER, driver 590.48.01, 32 CPUs;
- LIBERO 8f1084e, robosuite 1.4.1, mujoco 3.2.3, torch 2.7.1+cu128, numpy 1.26.4, Python 3.10.

The checks ran in a scratch worktree through `remote.sh`, so this branch's `runs/` stays empty. Commands are relative to the repo root.

## Environment: all five tasks load and render

`gah_env.make_env(task, seed=s)` for every training task with seeds 0, 1 and 2:
- every scene is valid (no drift while settling, goal not satisfied at start);
- the head (`agentview`) and wrist (`robot0_eye_in_hand`) cameras render at 256x256.

[`sanity_tasks.png`](sanity_tasks.png) shows seed 0 of each task, one row per task in the order of the README table, head view left and wrist view right.

| Task | LIBERO instruction | Goal (BDDL) |
|---|---|---|
| open_the_middle_drawer_of_the_cabinet | open the middle drawer of the cabinet | `(Open wooden_cabinet_1_middle_region)` |
| turn_on_the_stove | turn on the stove | `(Turnon flat_stove_1)` |
| push_the_plate_to_the_front_of_the_stove | push the plate to the front of the stove | `(On plate_1 main_table_stove_front_region)` |
| put_the_wine_bottle_on_the_rack | put the wine bottle on the rack | `(On wine_bottle_1 wine_rack_1_top_region)` |
| put_the_cream_cheese_in_the_bowl | put the cream cheese in the bowl | `(On cream_cheese_1 akita_black_bowl_1)` |

Timing:
- building an env takes about 1 s, or 2–3 s for the first one in a process;
- a control step takes about 15 ms;
- an episode with a pick and place takes about 6 s.

## API check

| Check | Result |
|---|---|
| `grasp` → lift → `place` → retreat on put_the_cream_cheese_in_the_bowl (seed 1, one-off script) | official success check: **True** |
| `examples/bowl_on_plate_pipeline.py` on the held-out task put_the_bowl_on_the_plate, 10 seeds | **0.8**; the two failures are linear approaches refused by the collision check (noisy plate pose) |
| linear move into the stove knob with `allow_contact=["flat_stove_1"]` | moves until blocked; stopped by the 120 N force limit |
| the same move without `allow_contact` | refused before moving: `linear move would collide with ['flat_stove_1_button']` |
| target outside the workspace bounds | refused |

## Scene edits: all ten keys

Every key was applied to put_the_cream_cheese_in_the_bowl (seed 0); all scenes are valid. [`sanity_edits.png`](sanity_edits.png) shows them row by row, in this order:
1. `object_pose` of the cream cheese;
2. `object_pose` of the cabinet (a fixture);
3. `object_color` (red bowl);
4. `object_scale` (bowl ×1.3, bbox 0.107 → 0.139 m);
5. `swap_object` (cream cheese → butter);
6. `lighting` (0.3, warm);
7. `add_distractor` (a box and a LIBERO ketchup bottle);
8. `add_obstacle`;
9. `object_motion` (0.1 m/s for 1 s: the cheese moved 7.8 cm);
10. `camera_pose`;
11. `dynamics` (mass 1.5 kg, friction 0.2; read back from the model).

Rejected as intended:
- an object moved out of reach: `cream_cheese_1 is out of reach (1.18 m from the robot base)`;
- an unknown key: `UnsupportedEdit: unknown edit key 'teleport'`.

## (a) Trivial pipeline end to end: PASS

`eval.py --task open_the_middle_drawer_of_the_cabinet --seeds 4 --pipeline benchmarks/libero/examples/trivial_pipeline.py`
- success_rate 0.0, no errors, about 3.4 s per episode, 4 workers in parallel;
- final frames written to `logs/frames/...`;
- one line appended to `logs/runs.jsonl`.

## Gate checks (put_the_bowl_on_the_plate, frozen pipeline = the reference pipeline committed as runs/…/pipeline.py, s0 = 0.8)

| Check | Variation | Result |
|---|---|---|
| (b) no-op | `edits: {}` | rejected: `no-op variation: no edits` |
| break gate, not broken | `object_color` (plate green) | rejected: `break gate: s_break=0.80 > s0/2=0.40` |
| break gate, broken | `add_obstacle` (plank 6 cm above the plate) | **pass**, s_break = 0.0 |
| (c) variance gate | 3rd `parameter_tuning: true` in a row (plate pushed further along +x) | rejected: `variance gate: third parameter-tuning variation in a row` |
| (c) automatic tuning detection | same as above but labelled `false`; same keys as the previous one, only numbers changed | rejected, `parameter_tuning_auto: true` |
| (d) integrity, tool file | `tools/eval.py` modified | rejected: `integrity: modified ['tools/eval.py']` |
| (d) integrity, success check | the task's BDDL goal changed to `(On akita_black_bowl_1 main_table_plate_region)` | rejected: `integrity: modified ['success:put_the_bowl_on_the_plate']` |
| (d) integrity, success check code | LIBERO `predicates/base_predicates.py` modified | rejected: `integrity: modified ['success_src:envs/predicates/base_predicates.py']` |

The (c) streak test seeded `gate_history.jsonl` with two accepted tuning variations, because only accepted variations count toward the streak.

After restoring the files, `tools/integrity.py` reports `{"ok": true}`. Every gate call appended a line to `logs/runs.jsonl`: 13 lines over the session.

## Not checked

- No training task was solved here: that is the training agent's job, and pre-solving one would leak into the GAH results.
- The reference pipeline therefore uses the held-out task put_the_bowl_on_the_plate.

## Re-check after the pilot-1 fixes (2026-10-23)

The three fixes are listed in ISSUES.md under "Found during pilot run". Re-run on <sim-host>:

| Check | Result |
|---|---|
| validity, all five tasks × seeds 0–2 (incl. new articulation check) | all valid |
| pilot-1 v1 (cabinet rotated +0.6 rad, drawer blocked by the rack) | rejected: `goal (open wooden_cabinet_1_middle_region) is unreachable: the moving part hits a fixed body before the goal position` |
| cabinet rotated +0.15 rad | valid |
| planned `move_to` top-down → horizontal approach → top-down, seeds 0–2 | 6/6 succeed (each 120–170 steps) |
| an episode held for 1236 control steps | no cutoff |
| (a) trivial pipeline, 4 seeds | runs, 0.0, no errors |
| reference pipeline (put_the_bowl_on_the_plate), 10 seeds | 0.8 (unchanged) |
| `tools/integrity.py` after rewriting INTEGRITY.json | `{"ok": true}` |
