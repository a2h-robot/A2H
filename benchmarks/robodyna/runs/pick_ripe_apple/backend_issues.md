# Backend issues: pick_ripe_apple, expert-free (policy) path

## Issue 1: a picked apple never gets gravity back

**Status:** fixed upstream in RoboDyna 6c90eec2 (merged here as d1b5dc4). Before the fix it blocked the task, and the first training session stopped in Phase 1 because of it.

### Symptom

When the apple is released over the basket, it does not fall. It stays in the air at the height where it was released, at rest, with no gripper contact, for as long as the episode runs. So "drop it into the basket", the task's own strategy and the expert's, cannot succeed. The only base-task success in two 10-seed runs was a fluke: on seed 10016, leftover velocity from the release carried the floating apple slowly down into the basket band.

### Cause (RoboDyna `envs/pick_ripe_apple.py`, read-only here)

- The expert path is `_try_front_grasp`. It calls `_detach_apple(side)`, which runs `_enable_apple_dynamic(gravity=False)` and leaves the apple floating while the jaws close. After `GRASP_SETTLE_STEPS` it calls **`_enable_held_apple_gravity(rigid)`**, which runs `set_disable_gravity(False)`.
- The expert-free path is `_sync_eval_gates`, merged in 954af13e. It only calls `_detach_apple(side=side)` when a closing gripper touches an apple. Nothing in this path ever calls `_enable_held_apple_gravity`. `grep` shows that the only call site is line 943, inside the expert's `_try_front_grasp`.
- As a result, under `eval_mode` (the GAH backend and RoboDyna's own policy evaluator), every detached apple keeps `disable_gravity=True` for the rest of the episode.

### Evidence (probes in `dev/`, which use simulator internals for logging only)

Ground truth was logged every 25 physics steps by `dev/probe_gt.py`, through the normal `eval.py --pipeline` path on the 10 Opt 1+2 seeds:

| Seed / probe | What happened | Ground truth after release |
|---|---|---|
| 10001, `probe_gt.py` (drop) | The apple was picked at r_grasp = 0.771 (inside the window 0.813 ± 0.12) and released at t ≈ 17.8 s, 10 cm above the rim | Apple centre at z = 0.887 from 18.0 s to 22 s (end of log), v = 0, `grav_off=True`, `held=False`. Basket rim at 0.788. |
| 10003, `probe_place.py` (place) | r_grasp in window; released at t = 18.76 s | z = 0.888 constant for 3 s, v = 0, `grav_off=True`, no gripper contact |
| 10016, `probe_place.py` | released at t = 22.15 s | z = 0.905 constant for 3 s, v = 0, `grav_off=True` |
| 10019, `probe_gt.py` | the apple was knocked at detach | it flew off horizontally at constant height (z from 0.898 to 0.80 over 10 s, x to -1.5 m), i.e. a gravity-free, lightly damped body |

The raw logs for issue 1 are in commit ef5999a (`dev/out/probe_10001.json` and `dev/out/place_10003.json`); `probe_10001.json` has since been overwritten by the issue-2 run. In each row, `rigid.grav_off` is the apple's `get_disable_gravity()`.

Everything else on the world side behaves as expected:
- ripeness = sim_time / 14.4 s from the first pipeline step;
- the good apple's colour moves towards red and the spoiled one's towards yellow;
- `_sync_eval_gates` detaches the apple on contact with a closing gripper and latches `r_grasp`;
- the basket ping-pongs between x = ±0.18;
- a held apple is carried by the gripper.

### Suggested fix (applied upstream as 6c90eec2)

In `pick_ripe_apple._sync_eval_gates`, give the apple gravity back the way the expert does, `GRASP_SETTLE_STEPS` (25) steps after the detach. Options:
- keep a per-side countdown in `_eval_gate_state` and call `self._enable_held_apple_gravity(rigid=self._apple_rigids[side])` when it expires;
- or simpler: enable gravity as soon as no gripper contacts the detached apple.

The fix changes `envs/pick_ripe_apple.py`. That changes the `task_file:pick_ripe_apple` hash (not `check_success`), so `tools/INTEGRITY.json` must be regenerated afterwards.

### Workaround not taken

The apple floats wherever it is let go. A pipeline could therefore lower it into the basket's success band (centre z within [base - 0.02, rim + 0.03] and inside the mouth) and open there. The basket's carry code would then drag the floating apple along. Success would then also depend on the carried apple's speed, which is set to the basket speed (0.08–0.16 m/s), against the `< 0.12` settle check. Doing this would tune the pipeline around a world that does not respond physically, which the run instructions forbid. It would also teach rules ("never let go above the target") that come from the bug, not from the task.

## Issue 2 (open, blocks the task): after 6c90eec2, gravity returns 0.1 s after the first finger contact, before any gripper can close

**Status:** open. The run stopped again in Phase 1: s0 was not recorded and no variations were made.

### What 6c90eec2 does, and how that differs from the expert
- **Expert** (`_try_front_grasp`):
  1. `_detach_apple()`;
  2. `move(close_gripper(...))`, a full close of about 1.2 s with the fruit floating;
  3. `GRASP_SETTLE_STEPS` (25) more steps;
  4. `_enable_held_apple_gravity()`.
  The jaws are therefore fully closed and squeezing before gravity returns.
- **Gate** (`_sync_eval_gates`): it detaches at the first physics step where a hand is commanded below 0.8 ("closing") and any gripper link touches the apple. It then turns gravity on **25 steps (0.1 s) after that first contact**, while the fingers are still closing.

### Evidence: 10/10 seeds, `dev/probe_gt.py` with `pipeline.py` at this commit
- **Setup:** horizontal pinch from the front. The fingers are pre-closed to command 0.82 (still "open", so no detach) and rest against the fruit; then `close_gripper(arm)`.
- **The pipeline side is right:** the apple choice is correct on 10/10 seeds, the tcp is within 1.5 cm of the fruit centre, and r_grasp is inside the window on 10/10 seeds.
- **Ground truth (contacts every 5 steps, rigid state every 25 steps):**

| seed | detach (first closing contact) | opening at detach | both fingers touching | gravity on (first row seen) | apple centre below z = 0.80 (fallen) | r_grasp / window |
|---|---|---|---|---|---|---|
| 10001 | 11.24 s | 0.834 | never | <= 11.40 | 11.60 | 0.780 / 0.813 |
| 10002 | 11.12 | 0.834 | never | <= 11.30 | 11.40 | 0.771 / 0.796 |
| 10003 | 11.66 | 0.835 | never | <= 11.80 | 12.00 | 0.809 / 0.789 |
| 10005 | 11.60 | 0.814 | 11.64 (0.795) | <= 11.70 | 11.90 | 0.805 / 0.792 |
| 10006 | 11.76 | 0.823 | 11.82 (0.798) | <= 11.90 | 12.00 | 0.815 / 0.807 |
| 10010 | 11.76 | 0.806 | at detach | <= 11.90 | 12.00 | 0.815 / 0.804 |
| 10016 | 11.76 | 0.839 | never | <= 11.90 | 12.10 | 0.816 / 0.782 |
| 10017 | 11.96 | 0.835 | 12.12 (0.769) | <= 12.10 | 12.20 | 0.829 / 0.785 |
| 10019 | 11.66 | 0.835 | never | <= 11.80 | 11.90 | 0.809 / 0.813 |
| 10023 | 12.30 | 0.819 | 12.34 (0.800) | <= 12.40 | 12.60 | 0.853 / 0.781 |

- **Every apple falls through the still-closing fingers** within 0.2–0.3 s of the detach, and every seed ends "good apple dropped on table".
- **Close timing** (`dev/probe_gripper.py`, ur5-wsg): from first contact (opening about 0.80 on this 5.1 cm fruit) to a tight hold (about 0.53, the hold opening seen with gravity off) takes about 0.3 s, and every close command takes 1.2 s. Even when both fingers already touch at the detach (10010), the squeeze is not built after 0.1 s.
- **The gate cannot be satisfied earlier:** with a command at 0.8 or above ("open"), the fingers can only rest against the fruit, not squeeze it. Any command below 0.8 is "closing", and the first contact then starts the 0.1 s countdown.
- **Earlier v1 runs** with a 0.70 pre-close and no pinch offset showed the same thing.

### Suggested fix (RoboDyna)
Re-enable gravity the way the expert does: only once the closing hand has finished closing, i.e. its gripper is at its commanded value or has stopped moving, plus `GRASP_SETTLE_STEPS`. A simpler option is a longer countdown that covers a full close (about 1.2 s / 300 steps + 25). Either way, a pinch closed at the robot's own speed can then hold the fruit, as the expert's does.

### Workarounds not taken
- **Cradle the fruit on the palm or a lower finger** (tilted, from below), so that it does not fall when gravity returns. A first try with the palm pitched up 30–60 deg was blocked by reach and collisions with the tree.
- **Use the second arm as a tray** under the fruit.
Both work around a stem that breaks at the touch of a still-opening hand and leaves the fruit 0.1 s to be held. They would teach rules that come from this gate's timing rather than from the task.
