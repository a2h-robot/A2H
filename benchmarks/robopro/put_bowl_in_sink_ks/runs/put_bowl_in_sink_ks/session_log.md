# Session log: RoboPRO place_bowl_in_dishrack_ks (abandoned) and put_bowl_in_sink_ks

This is a curated record of the agent session (Claude Opus 5.5 in Claude Code, 2026-10-26). It covers what was
built, what was run, what failed and why, and the decisions taken.
- The machine-readable records are in the repo: commits per round, `variations/*.yaml` with their `.gate.json`,
  `gate_log.jsonl`, `gate_history.jsonl`, `reflections.jsonl`, `screening/`, `abandoned_place_bowl_in_dishrack_ks/`
  and `../../logs/runs.jsonl`.
- Times are Toronto time (EDT). The cluster clock is UTC, so gate times and frame folder names are 4 h later.
- Unless noted otherwise, every success rate is over the first 10 seeds of the task's seed file.

Platform of record: the GB10 (aarch64) RoboPRO install, the same tree as the other RoboPRO runs. No number of
this run comes from another platform.

## Requests (relayed by the team's coordinator)

| Request | Outcome |
|---|---|
| Train on `place_bowl_in_dishrack_ks`; if Phase 1 cannot reach tau = 0.8 within about 45 minutes, switch to `put_bowl_in_sink_ks` and report | Abandoned after its Phase 1, with evidence that tau = 0.8 is not honestly reachable; switched. |
| Use a dedicated GB10 node through Slurm | Done: plain Slurm allocations throughout. Two were killed by the node's memory guard (see Compute). |
| Move to faster x86 GPUs / run x86 in parallel | Not done, agreed with the coordinator: the x86 trees carry different task code, and the seed filter and every number must come from one platform. The task was limited by design, not compute. |
| Prove that recording the bowl's start pose does not make success easier | Done: a null policy and a wrong-place policy both score 0/10, and the success check is unchanged. |
| Keep tools out of the training loop; re-check any number measured under a changed harness | Done: a mid-run harness change was reverted and the affected gate re-run (see "Harness incident"). |
| Publish when done: branch, PR (unmerged), clips, dataset | This log is part of that. |

## 1. Abandoned task: place_bowl_in_dishrack_ks (15:10-16:25)

**Backend.** The backend was extended to the small-kitchen scene:
- the bowl is the task object;
- the dish rack is modelled for the planner as a floor slab, four walls and its utensil holder (measured from
  its collision mesh);
- the sink is seen as separate boxes.

RoboPRO's expert solved 23 of 60 candidate seeds (20 clean-bank and 40 sequential).

**What the success check needs.** The rack's floor wires rise only 1-5 cm, but the GB10 check needs the
bowl's origin at least 10.1 cm above the counter. So a bowl lying upright in the rack fails (0.79-0.83 m
against 0.841 m).

Drop tests (356 releases, no robot) found two passing rest states:
- the bowl on edge on the 4.9 cm central ridge, leaning on the utensil holder, 1-10 mm above the threshold;
- nothing else repeatable.

**The expert.**
- Re-run twice on its own 23 seeds, it scored 13/23 both times, with different seeds failing.
- In 16 of its 26 successes the bowl ends at 0.94-0.95 m. Released at exactly that pose, a free bowl never
  stays there (90 releases).
- So those successes are the bowl lying on the robot's open fingers: `play_once` opens the gripper and never
  retreats.

**Attempts.**
- The pinch lets the bowl swing.
- The on-edge pose needs a horizontal hand pointing at the holder, which the right arm cannot reach low enough.

**Decision (16:25).** Abandon and switch. The expert's ceiling of 0.57 bounds tau = 0.8, and the honest target
leaves under a centimetre of margin. The evidence is in `abandoned_place_bowl_in_dishrack_ks/` and in the
summary.

## 2. put_bowl_in_sink_ks: backend and seeds (16:25-16:31)

- The task's check compares the bowl with a start pose that only RoboPRO's expert records. The backend now
  records it at every reset; the check is unchanged.
- RoboPRO's expert fails all 60 candidate seeds (cuRobo: `INVALID_START_STATE_WORLD_COLLISION` on its lift).
  So the seed bank is filtered for a valid start only (60/60), and the pipeline is the only solvability
  evidence.

## 3. Phase 1 (16:31-16:33)

| Eval | Change | Result |
|---|---|---|
| 1 | Rim pinch from above, lift 5 cm over the counter, straight carry over the basin middle, release 3 cm above the floor, straight up | 0.9 |
| s0 | Same pipeline, `--record-s0` | **0.9** (commit 1c38fb2) |

Sanity:
- null policy 0/10;
- wrong-place policy 0/10;
- integrity ok.

## 4. Round 1: v1, a pot in the basin (16:33-16:50)

**Screening.** A 12 cm pot scored 0.2 (5 seeds). A 10 cm pot, leaving a 14 cm strip for the 13.6 cm bowl,
scored 0.2 (10 seeds).

**Gate.**
- s0 = 0.9, s_break = 0.3, scene valid.
- The oracle reads 0.0, from the broken expert.

| Repair | Change | base / v1 |
|---|---|---|
| 1 | Release spot with the most room among the boxes in the basin; carry above them | 1.0 / 0.7 (the first run of this repair crashed on an attribute error in new code and was re-run after the fix). It chose the strip behind the pot, out of reach, and released the bowl on the counter |
| 2 | Spots ranked by room, then by nearness to the robot; next spot if unreachable; release only over the basin | **0.9 / 0.9** (commit 37db142) |

## 5. Round 2: v2, the bowl on the far side of the counter (16:50-17:20)

**Screening.**
- Upside-down bowl: 0.0, but not solvable with this gripper (the foot-ring grasp held 0/18).
- Heavy and slippery bowl: 0.6.
- Bumped bowl: 0.5.
- Soap bottle in the carry path: 0.7.
- Bowl on the far side: 0.0. A draft repair solved 4/5 before gating.

**Gate.** s0 = 1.0, s_break = 0.0.

| Repair | Change | base / v1 / v2 |
|---|---|---|
| 1 | If the holding arm reaches no release spot: set the bowl down between the arms, send that arm home, pick it up with the other arm | **1.0 / 1.0 / 1.0** (commit bbfa283) |

**Harness incident (17:05-17:22).** After GPU out-of-memory failures in a screening evaluation on a shared node,
the hashed tools were changed to re-run infrastructure failures, and `INTEGRITY.json` was regenerated. The v2
gate ran under that change. It was reverted to the Phase-1 tools and hash file (commit 30282b3).

Re-run under the restored harness (from a checkout of the round-1 commit):
- the v2 gate reproduced: s0 1.0, s_break 0.0;
- the round-2 evaluation gave 0.8 / 1.0 / 0.9.

No gate or recovery evaluation of this run contains an infrastructure error.

## 6. Round 3: v3, a pot slides into the release spot (17:22-18:02)

**Screening.**
- A 10 cm pan sliding in scored 1.0, but for the wrong reason: the bowl rested on the pan and passed `below_rim`.
- A 14 cm pot sliding in scored 0.0.

**Gate.** s0 = 1.0, s_break = 0.0.

| Repair | Change | v3 (10 seeds) |
|---|---|---|
| 1 | Before lowering: wait until the basin is still, re-check the spot, choose again if taken | 0.0: the only free part (behind the pot) is out of reach |
| 2 | Turn the hand so the bowl hangs on the far side | 0.0 |
| 3 | Tilted-landing spots (only the bowl's centre over free floor) | 0.3: they put the bowl on the front rim |
| reach map | Plans to TCP poses over the basin's far half | Straight down reaches none of it; pitched 0.5-0.9 rad reaches all of it |
| 4 | Pitch the hand towards far spots | 0.0: the pitch's sign was wrong |
| 5 | Sign fixed | 0.8; `--all` **1.0 / 0.9 / 0.9 / 0.9** (commit b8e3d20) |

## 7. Round 4 screening and stop (18:02-18:25)

Against the round-3 pipeline:
- A fixed 25 cm stand beside the bowl scored 0.2, but it blocks the grasp approach, not the carry.
- A 14 cm soap holder in two placements scored 0.5 and 0.5.
- Wet bowl: 1.0. Heavy bowl (0.8 kg): 1.0.
- Canisters at the bowl's annotated grasp points: 0.0. A draft that pinches the rim elsewhere, lifts over
  nearby items and checks the hold after lifting still scored 0/5.

The run stopped here for time, with 3 accepted variations: convergence not reached.

## Compute

- **Placement.** Plain Slurm allocations on GB10 nodes:
  - one node to start;
  - one other after the first was lost;
  - two nodes for the rest.
- **Lost allocations.** The memory guard killed two allocations: the first with out-of-allocation probes
  beside 8 workers, the second with 8 workers plus two parallel evaluations.
- **Cause.** GPU memory on a GB10 comes out of the node's RAM and is invisible to the job's memory request
  and cgroup. A worker costs 10-13 GB of node memory although its cgroup shows ~3 GB.
- **After that.**
  - all probes and evaluations run inside the run's own allocation;
  - at most 5 workers per node;
  - a worker starts only above 30-35 GB available, and the newest stops below 20 GB.
- **Guests.** Another project's guest processes on the two nodes were drained on request, so evaluations
  were not disturbed.
