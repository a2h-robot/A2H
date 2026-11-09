# Backend issues: catch_valley_ball (RoboDyna 954af13e, Opt 1+2)

## 1. The blue box is kinematic for the whole episode, so the task cannot be solved (blocking)

**What happens.** On the policy-evaluation path that `gah_env.GahEnv.reset()` uses (`eval_mode=True`,
no scripted expert), the catch box (`blue box`, `env.bowl`, the procedural `custom_catch_box`) is a
**kinematic** rigid body from reset to the end of the episode. No contact can move it, so no program
can push it to the landing point, and `check_success()` can never be true (fail reason
`ball_not_in_box` on every seed).

**Why (RoboDyna `envs/catch_valley_ball.py`).**
- `setup_demo()` -> `_prepare_push_box()` -> `_freeze_box()` makes the box kinematic
  (`set_kinematic(True)`, gravity off).
- The box is handed to PhysX only by `_enable_box_physics()`, which is called in two places:
  - inside the scripted expert (`_push_box_to_landing`, from `play_once`);
  - at the end of `setup_demo()`, but only if `_interactive_robot_mode` or
    `_interactive_universal_controls` is set (the human GUI).
- `catch_valley_ball` does not override `_sync_eval_gates()` (954af13e added it for
  `catch_cup`, `catch_ramp_ball`, `catch_shelf_marble`, `catch_mouse_object_drop`, but not this
  task), so the eval path never enables box physics.
- ISSUES.md item 1 describes the same class of bug for four other tasks; this is a fifth.

**Evidence.**
- `dev/probe1.py` (10 seeds): `rigid.kinematic == True` for the box at t = 0 on all 10 seeds.
- `dev/probe2.py` (3 seeds): with the gripper closed and pressed against the box's robot-facing
  wall, four 5 cm `move_by(dy=+0.05)` pushes all return ok, but the box does not move by even
  0.1 mm and the tcp stays stuck 6 cm behind the box centre (the arm is blocked by an immovable body).
- Official Phase 1 eval of `pipeline.py` (10 seeds): s0 = 0.0, `ball_not_in_box` on every seed
  (see `s0.json`, `logs/runs.jsonl`, frames `logs/frames/catch_valley_ball/base_20261023-212340`;
  the traces show the push stalling at the box's near face and the pipeline giving up).
- **Diagnostic only, not a scored result:** `dev/patched.py` calls RoboDyna's own
  `_enable_box_physics()` at the start (what interactive mode does) and then runs the unchanged
  `pipeline.py`: **8/10** on the same 10 seeds (log line with
  `program: .../dev/patched.py`, frames `logs/frames/catch_valley_ball/base_20261023-212213`).
  So the box physics is the only thing that blocks the task; with it, the pipeline's approach works.

**Suggested fix (RoboDyna side, not done here).** Add to `catch_valley_ball`:

```python
def _sync_eval_gates(self):
    if getattr(self, "_loaded", False) and not self._push_active:
        self._enable_box_physics()
        self._push_active = True
```

(or call `_enable_box_physics()` at the end of `setup_demo()` when `eval_mode` is set, as is done
for interactive mode). Then re-check the expert path is unchanged (`_runs_without_expert()` guards it).

**Effect on this run.** Phase 1 cannot reach tau = 0.8 (s0 = 0.0), so no variation can be gated
(the break gate needs a working pipeline, s_break <= s0/2 is meaningless with s0 = 0). The run
stops at Phase 1 as blocked.

## 2. An arm pressing on the kinematic box makes episodes extremely slow (tooling)

An earlier version of the pipeline kept commanding pushes into the (immovable) box. The 10-seed eval
then took **1803 s of wall clock** instead of ~45 s (workers hit the 900 s job timeout and were
recycled; the log line is `pipeline.py@5e0b13ef829a`, s = 0.0), and the `remote.sh` ssh session
stayed open after the remote eval had exited, until it was killed by hand (the results had already
been written to `logs/runs.jsonl`; worker processes were left on the GPUs and had to be killed).
The committed pipeline now stops pushing after two blocked steps. Worth a guard in the backend
(per-episode wall-clock limit) and in `remote.sh`.

Note: `dev/patched.py` runs import `pipeline.py`, so their `program` hash in `runs.jsonl` does not pin
the pipeline version; only the last patched run (0.8) used the committed pipeline.

## 3. Things a pipeline author has to discover (not bugs, but worth documenting)

- `catch_valley_ball` is randomly **mirrored** per seed (3/10 seeds exit towards -x and need the
  left arm). Nothing in the README says so; the pipeline infers it from where the red ball starts.
- With the gripper open in the home orientation, the WSG fingers open along world x and are
  ~10 cm apart: they **straddle the 9.2 cm box** and pass on both sides of it without contact.
  Rotating the gripper 90 deg about its approach axis (fingers along y) fixes this.
- Open WSG fingertips reach 4.3 cm below `tcp_pose()`; a tcp target lower than ~0.785 is clipped by
  the table (the move returns ok, but the tcp stops at ~0.785).
- The pipeline pushes in 3 cm straight segments (`hold_orientation=True`) so it can stop when the
  pushed object does not give way; whether a single long low `move_to` stays straight was not checked.
- `close_gripper()` costs 1.2 s of sim time, a large share of the ~5.7-6.6 s before the ball leaves
  the ramp.

---

# Update after the fix (RoboDyna 5303d554 and later; rounds 1-5)

Issue 1 is fixed upstream (box handed to PhysX on expert-free runs); Phase 1 re-recorded s0 = 0.8.

## 4. `remote.sh` still hangs sometimes after the remote command has exited

Even with the leftover-process cleanup, the local `ssh` of `remote.sh` stayed open after the remote
gate/eval had finished (seen 3 times: v4 gate, two `--all` evals). Results were complete on <sim-host>;
killing the local ssh let `remote.sh` copy them back. Seen when one or more episodes hit the worker
timeout. Earlier (before the fix) a killed local `remote.sh` left the gate running remotely and its
outputs had to be fetched by hand (rsync of `runs/` and `logs/runs.jsonl`, merged line by line).

## 5. Worker timeouts count as failures (and can hide successes)

Episodes in which the arm presses against an immovable obstacle run very slowly in wall-clock time
and hit the 900 s job timeout (`worker timed out (exit -9)`). In the v2 gate 6/10 break seeds ended
this way (counted as failures, which is fair for the break gate but makes a gate take ~50 min). In
some evals a timed-out seed is reported with `success: true` (success was latched before the
timeout), e.g. v4 seed 10014 and v5 seed 10014.

## 6. Scene edits that RoboDyna silently overrides on the box (from reading the task code; the x part is confirmed by v1)

After `load_actors()` (where edits run) `catch_valley_ball` re-parks and re-tunes the box in
`setup_demo()` -> `_prepare_push_box()` and again in `_enable_box_physics()`:
- `object_pose` on `blue box`: only x survives; y and rotation are reset (`_box_spawn_xy`, `BOX_Q`).
- `dynamics` on `blue box`: mass and friction are reset (`set_mass(bowl_mass)`, `_tune_box_materials`).
So "heavier / slipperier box" and "box starts rotated" cannot be expressed as variations for this
task; the validity check does not flag this (the edit is accepted and has no effect).
