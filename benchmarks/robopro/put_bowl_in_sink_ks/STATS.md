# GAH training statistics: RoboPRO backend

| Task | Variations designed (gated) | Candidates screened | Accepted | Rejected: break gate | Rejected: variance gate | Rejected: integrity / oracle | Unsolved | Rounds to convergence | Total tokens | Wall-clock |
|---|---|---|---|---|---|---|---|---|---|---|
| pick_boxdrink_from_basket | 5 (v1-v5) | 20 | 4 | 0 | 0 | 1 integrity (a stale hash file after a backend change; the gate was rerun and passed) / 0 | 1 (v3) | not converged: stopped after 4 accepted variations | not logged by the tools | ~7 h (phase 1 at 16:57, round 5 accepted at 23:25, Toronto) |
| put_bowl_in_sink_ks | 3 (v1-v3) | 15 | 3 | 0 | 0 | 0 / 0 (oracle broken: RoboPRO's expert fails every seed) | 0 | not converged: stopped after 3 accepted variations (time) | not logged by the tools | ~1 h 50 min (phase 1 at 16:33, round 3 accepted at 18:02, round-4 screening until 18:25, Toronto), after ~1 h on the abandoned place_bowl_in_dishrack_ks |

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

put_bowl_in_sink_ks, candidates screened (15, rounds 1-4):
- 3 became the gated variations (v1-v3);
- 2 broke it but were not shown solvable: the bowl upside down (the 9 cm gripper cannot hold it), and
  canisters at the bowl's grasp points (a draft repair scored 0/5);
- 2 broke it only partly or for the wrong reason (a 12 cm pot, narrowed to v1; a stand that blocks the
  grasp approach rather than the carry);
- 8 did not break it (heavy 0.4 kg, heavy 0.8 kg, wet and bumped bowl; a bottle and two soap-holder placements
  in the carry path; a 10 cm pan sliding in, which passed for the wrong reason, see ISSUES 15).

| Axis (put_bowl_in_sink_ks) | Reflections | Accepted | Unsolved |
|---|---|---|---|
| obstacle | 1 | 1 | 0 |
| object_position | 1 | 1 | 0 |
| object_motion | 1 | 1 | 0 |
