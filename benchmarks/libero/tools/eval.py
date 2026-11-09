"""Evaluate a pipeline on a LIBERO-Goal task (docs/workflow.md sections 3 and 7).

python benchmarks/libero/tools/eval.py --task open_the_middle_drawer_of_the_cabinet --seeds 10
python benchmarks/libero/tools/eval.py --task T --variation benchmarks/libero/runs/T/variations/v1.yaml
python benchmarks/libero/tools/eval.py --task T --all          # base task + every accepted variation
python benchmarks/libero/tools/eval.py --task T --record-s0    # Phase 1: store s0 in runs/T/s0.json
python benchmarks/libero/tools/eval.py --task T --video        # also an mp4 per seed (head camera, real time)

Seed s uses LIBERO's fixed init state number s % 50 and perception-noise seed 10000 + s.
The pipeline is benchmarks/libero/runs/T/pipeline.py (override with --pipeline) and must
define run(api). Prints JSON; appends one line per evaluated scene to logs/runs.jsonl.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _common as C  # noqa: E402


def evaluate(task, variation_path, seeds, seed_offset, pipeline, workers, frames=True, phase="train", arm="GAH",
             video=False):
    variation = C.load_variation(variation_path)
    vid = C.variation_id(variation_path)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    frames_dir = str(C.LOGS / "frames" / task / f"{vid}_{stamp}") if frames or video else None
    t0 = time.time()
    recs = C.parallel(C.run_episode, [(task, variation, seed_offset + s, str(pipeline), frames_dir, 10, video)
                                      for s in range(seeds)], workers)
    rate = sum(r["success"] for r in recs) / max(1, len(recs))
    out = {"task": task, "variation": vid, "success_rate": rate, "seeds": seeds,
           "seed_offset": seed_offset, "per_seed": recs, "frames_dir": C.rel(frames_dir) if frames_dir else None}
    C.append_log({
        "phase": phase, "arm": arm, "task": task, "variation": vid, "backend": "sim", "attempt": None,
        "success": rate >= 0.8, "success_rate": rate, "seeds": seeds, "tokens_in": None, "tokens_out": None,
        "wall_clock_s": round(time.time() - t0, 1),
        "program": f"{C.rel(pipeline)}@{C.git_blob(pipeline)}", "notes": "eval.py",
    })
    return out


def accepted_variations(task):
    vdir = C.RUNS / task / "variations"
    out = []
    for gate_file in sorted(vdir.glob("*.gate.json")):
        g = json.loads(gate_file.read_text())
        yml = gate_file.with_name(gate_file.name.replace(".gate.json", ".yaml"))
        if not g.get("pass") or not yml.exists():
            continue
        v = C.load_variation(yml) or {}
        if v.get("unsolved"):
            continue
        out.append(yml)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--variation")
    g.add_argument("--all", action="store_true")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--seed-offset", type=int, default=0)
    ap.add_argument("--pipeline")
    ap.add_argument("--workers", type=int, default=C.default_workers())
    ap.add_argument("--record-s0", action="store_true")
    ap.add_argument("--no-frames", action="store_true")
    ap.add_argument("--video", action="store_true",
                    help="also write seed<N>.mp4 per seed under frames_dir (head camera, 5 fps = real time)")
    args = ap.parse_args()
    pipeline = Path(args.pipeline) if args.pipeline else C.RUNS / args.task / "pipeline.py"
    if not pipeline.exists():
        sys.exit(f"pipeline not found: {pipeline}")
    scenes = ([None] + accepted_variations(args.task)) if args.all else [args.variation]
    results = [evaluate(args.task, v, args.seeds, args.seed_offset, pipeline, args.workers,
                        frames=not args.no_frames, video=args.video) for v in scenes]
    if args.record_s0:
        base = results[0]
        (C.RUNS / args.task).mkdir(parents=True, exist_ok=True)
        (C.RUNS / args.task / "s0.json").write_text(json.dumps(
            {"s0": base["success_rate"], "seeds": args.seeds, "pipeline": f"{C.rel(pipeline)}@{C.git_blob(pipeline)}"}, indent=2))
    if len(results) == 1:
        print(json.dumps(results[0], indent=2))
    else:
        summary = {"task": args.task, "min_success_rate": min(r["success_rate"] for r in results),
                   "scenes": {r["variation"]: r["success_rate"] for r in results}, "details": results}
        print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
