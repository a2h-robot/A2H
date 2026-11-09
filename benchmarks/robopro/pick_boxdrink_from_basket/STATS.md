# GAH training statistics: RoboPRO backend

| Task | Variations designed (gated) | Candidates screened | Accepted | Rejected: break gate | Rejected: variance gate | Rejected: integrity / oracle | Unsolved | Rounds to convergence | Total tokens | Wall-clock |
|---|---|---|---|---|---|---|---|---|---|---|
| pick_boxdrink_from_basket | 5 (v1-v5) | 20 | 4 | 0 | 0 | 1 integrity (a stale hash file after a backend change; the gate was rerun and passed) / 0 | 1 (v3) | not converged: stopped after 4 accepted variations | not logged by the tools | ~7 h (phase 1 at 16:57, round 5 accepted at 23:25, Toronto) |

Candidates screened: variations tried against the frozen pipeline on 5 seeds before the gated one was designed.
Of the 20:
- 5 became the gated variations;
- 5 were invalid scenes (the bowl-shaped basket ejects the box at setup);
- 9 were valid but did not break the pipeline;
- 1 was valid and broke it but was not gated: a 30 % larger carton (0.2).

The oracle never rejects a variation in this backend; it is informational.

## Reflections per axis

| Axis | Reflections | Accepted | Unsolved |
|---|---|---|---|
| distractor | 1 | 1 | 0 |
| orientation | 1 | 1 | 0 |
| obstacle | 1 | 0 | 1 |
| object_motion | 1 | 1 | 0 |
| dynamics | 1 | 1 | 0 |
