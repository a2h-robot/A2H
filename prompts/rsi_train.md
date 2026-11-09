# RSI baseline training prompt

Fill in: `{TASK}`, `{GOAL}`, `{TAU}` = 0.8, `{SEEDS}` = 10, `{TOKEN_BUDGET}` = the GAH tokens used on this task (or 2M if not yet known).

This prompt matches `gah_train.md` except that the agent **cannot design new tasks**. It improves only from failures on the given task.

---

You are developing a robot program in simulation for the task **{TASK}**: "{GOAL}". Your final output is not the program. It is the experience you write down, which will later help a fresh agent write a robust program for a different task on a real robot, on its first attempt.

Read `docs/api_spec.md`. Use only that API. You may not edit the scene or create variations. Evaluate only with the task's default randomization.

## Loop (until your token budget of {TOKEN_BUDGET} is used)
1. Write or repair `runs_rsi/{TASK}/pipeline.py`.
2. Run `python tools/eval.py --task {TASK} --seeds {SEEDS}`. Look at the failing seeds: frames, logs, and object poses.
3. Diagnose the cause of the failures and fix the pipeline.
4. After each fix, append one JSON line to `runs_rsi/{TASK}/reflections.jsonl` with these fields:
   - `weakness`;
   - `how_found`;
   - `why_missed`;
   - `fix`;
   - `general_rule`: one sentence, with no task or object names;
   - `novel`.
5. If the success rate is at least {TAU} and no seed fails, rerun with new seeds (`--seed-offset`) to look for remaining failures.

Stop when the budget is used or when two consecutive rounds of new seeds show no failure. Then write `runs_rsi/{TASK}/summary.md`.
