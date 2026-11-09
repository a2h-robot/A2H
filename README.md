# A²H: Learning to Anticipate Physical Failures via Agentic Adversarial Harness

Training code and training records for **A²H** (CoRL 2026 Late-Breaking Results).
Project page: <https://a2h-robot.github.io>

A coding agent learns a harness in simulation by designing tasks that break its own robot programs.
For each training task it plays two roles:

- **Solver**: writes a program against a shared robot API until it succeeds on the task (τ = 0.8 over 10 seeds).
- **Task designer**: questions the program's assumptions and edits the scene so that the program fails.

A gate tool, not the agent, decides whether a variation is accepted. The agent then repairs the program on the
new variation and on every earlier one, and writes a reflection. The reflections from all training tasks are
later condensed into one harness that a fresh agent reads before writing a program for an unseen real-robot task.

> Internally the project was called **GAH**; file names such as `gah_env.py` and `prompts/gah_train.md` keep that name.

## What is in this repository

```
docs/workflow.md         Training protocol: loop, gates, logging, variation file format
docs/api_spec.md         Shared robot API and scene-edit keys (same names on every backend)
prompts/gah_train.md     The training prompt (one agent session per task)
prompts/gah_summarize.md The prompt that condenses reflections into a harness
prompts/rsi_train.md     Recursive self-improvement (RSI) baseline prompt
prompts/deploy.md        Deployment prompt template (the harness is inserted at {HARNESS})
benchmarks/<bench>/      Simulator backend, scene edits, eval/gate tools, and one run folder per task
```

### Training tasks (25 tasks in 5 simulators)

| Benchmark | Folder | Tasks |
|---|---|---|
| LIBERO-Goal | `benchmarks/libero/` | open_the_middle_drawer_of_the_cabinet, turn_on_the_stove, push_the_plate_to_the_front_of_the_stove, put_the_wine_bottle_on_the_rack, put_the_cream_cheese_in_the_bowl |
| RoboTwin 2.0 | `benchmarks/robotwin/<task>/` | hanging_mug, put_object_cabinet, scan_object, stack_blocks_three, turn_switch |
| ManiSkill3 | `benchmarks/maniskill/` | LiftPegUpright-v1, PegInsertionSide-v1, PlaceSphere-v1, PlugCharger-v1, PullCubeTool-v1 |
| RoboPRO | `benchmarks/robopro/<task>/` | pick_boxdrink_from_basket, put_bowl_in_sink_ks, put_cup_on_coaster, put_mouse_on_pad, put_spoon_on_plate_ks |
| RoboDyna | `benchmarks/robodyna/` | catch_shelf_marble, catch_valley_ball, drop_ball_hole, pick_ripe_apple, place_block_belt |

LIBERO, ManiSkill3 and RoboDyna share one backend per benchmark. RoboTwin 2.0 and RoboPRO tasks were trained
on backends that diverged per task, so each of those tasks keeps its own backend snapshot next to its run.

### One run folder per task (`.../runs/<task>/`)

| File | Content |
|---|---|
| `pipeline.py` | The final program after all repairs |
| `assumptions.md` | The scene assumptions the agent listed before designing variations |
| `variations/vK.yaml` | Each designed variation (scene edits only; goal and success check unchanged) |
| `variations/vK.gate.json` | The gate tool's verdict for that variation |
| `gate_history.jsonl` | Every gate call, accepted or rejected |
| `reflections.jsonl` | One reflection per accepted round: weakness, how it was found, why it was missed, fix, general rule |
| `s0.json`, `summary.md` | Base success rate and the agent's end-of-run summary |

## The harness

The distilled harness used in the real-robot study is **not released at this time**.
`prompts/gah_summarize.md` shows how it is produced from the reflections in this repository.

## Running a training task

1. Install the simulator for your benchmark (see `benchmarks/<bench>/README.md`; LIBERO has `install_libero.sh`).
   Point the backend at your simulator checkout with the environment variable named there
   (for example `ROBOTWIN_ROOT` or `ROBODYNA_ROOT`).
2. Check the task loads: `python benchmarks/<bench>/tools/eval.py --task <task> --seeds 10`.
3. Start a fresh coding-agent session at the repository root and give it `prompts/gah_train.md` with
   `{BENCH}`, `{TASK}`, `{GOAL}` filled in. The agent writes into `benchmarks/<bench>/runs/<task>/`.

The agent may not edit `tools/` or the task's success check; `gate.py` hashes both and rejects a round if they change.

## Citation

```bibtex
@inproceedings{li2026a2h,
  title     = {Learning to Anticipate Physical Failures via Agentic Adversarial Harness},
  author    = {Li, Zhiyuan and Liu, Dingsheng and Rasouli, Amir},
  booktitle = {Conference on Robot Learning (CoRL), Late-Breaking Results},
  year      = {2026}
}
```
