# GAH training prompt

Fill in: `{BENCH}` (libero | robotwin | maniskill), `{TASK}`, `{GOAL}`, `{TAU}` = 0.8, `{SEEDS}` = 10, `{MAX_ROUNDS}` = 5, `{MAX_REPAIR}` = 6.

---

You are developing a robot program in simulation for the task **{TASK}**: "{GOAL}".

You play two roles. As the **solver**, you write a program that completes the task. As the **task designer**, you act as a harsh teacher to yourself: you design harder versions of the task that break your own program, so that you learn what your program does not handle. Your final output is not the program. It is the experience you write down, which will later help a fresh agent write a robust program for a different task on a real robot, on its first attempt.

Read `docs/api_spec.md` before you start. Use only that API.

## Phase 1: Solve
1. Write `benchmarks/{BENCH}/runs/{TASK}/pipeline.py`.
2. Run `python benchmarks/{BENCH}/tools/eval.py --task {TASK} --seeds {SEEDS}`. Iterate until the success rate is at least {TAU}.
3. Record this success rate as `s0`. Commit the pipeline.

## Phase 2: Question your pipeline and design a task
1. Read your pipeline line by line. In `benchmarks/{BENCH}/runs/{TASK}/assumptions.md`, list every assumption it makes about the scene. Cover where objects are, how they are oriented, what they look like, whether they move, what is between the gripper and the target, what the camera sees, and how objects respond to contact. For each assumption, give the line of code that relies on it.
2. Pick the assumption that a real robot is most likely to violate and that you have not attacked yet.
3. Write one variation `benchmarks/{BENCH}/runs/{TASK}/variations/vK.yaml` that violates it. Use only the edit keys in `docs/api_spec.md` (Scene edits).
   - Keep the goal and the success check unchanged.
   - The new task must still be physically solvable.
   - Set `parameter_tuning: true` if the variation only changes the magnitude of something the previous variation already changed, such as moving the same object further in the same direction.

## Phase 3: Gates
Run `python benchmarks/{BENCH}/tools/gate.py --task {TASK} --variation benchmarks/{BENCH}/runs/{TASK}/variations/vK.yaml`. The tool runs your pipeline as it was before this round, and it rejects the variation if any of these hold:
- the original pipeline still succeeds more than half as often as `s0` (the variation does not break it);
- it would be the third parameter-tuning variation in a row (a parameter-tuning streak);
- the success check or any tool file changed.

If the tool rejects the variation, go back to Phase 2 and design a different variation. Do not edit `benchmarks/{BENCH}/tools/` or the task's success check.

## Phase 4: Recover
Repair the pipeline until all three hold:
- success on the new variation is at least {TAU};
- success on every earlier accepted variation is at least {TAU};
- success on the original task is at least {TAU}.

Run `python benchmarks/{BENCH}/tools/eval.py --task {TASK} --all --seeds {SEEDS}` to check. If you cannot recover within {MAX_REPAIR} repair iterations, set `unsolved: true` in the variation file, revert the variation, write a reflection that explains why, and return to Phase 2.

## Phase 5: Reflect
Append one JSON line to `benchmarks/{BENCH}/runs/{TASK}/reflections.jsonl` with these fields:
- `weakness`: what your program got wrong, concretely;
- `how_found`: which assumption you questioned and why you suspected it;
- `why_missed`: why you did not handle this when you first wrote the program;
- `fix`: what you changed;
- `general_rule`: one sentence that would have prevented this failure on *any* task. Do not name this task or its objects.
- `novel`: false if an earlier rule already says the same thing.

Also fill in `round`, `variation`, `axis`, `parameter_tuning`, `s0`, `s_break`, and `s_recovered`.

## Phase 6: Continue or stop
- Go back to Phase 2 with the repaired pipeline.
- Stop when one of these holds:
  - two consecutive accepted rounds produced `novel: false`;
  - you have {MAX_ROUNDS} accepted variations.
- When you stop, write `benchmarks/{BENCH}/runs/{TASK}/summary.md`: the variations you designed, which axes broke the program and which did not, and the rules you found.

## Rules
- Prefer variations that a real robot will meet: objects not where expected, objects that move, obstacles near the path or goal, different appearance, different viewpoint, different mass or friction.
- Do not make the task impossible or change what counts as success.
- Do not tune the program to pass the gate. The gate is there to measure the pipeline you had before this round.
