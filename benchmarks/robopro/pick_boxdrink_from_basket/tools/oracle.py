"""Solvability oracle: RoboPRO's scripted expert (play_once) on a task or a variation.

python benchmarks/robopro/tools/oracle.py --task T [--variation V.yaml] [--seeds 10] [--seed-offset k]

The expert uses ground-truth poses and RoboPRO's own planner world (bodies added by a variation are not
in it). It is open-loop, so it cannot follow objects that move after the start. Prints JSON
{oracle_success_rate, per_seed}; per_seed holds each seed's start and end state.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _common as C  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--variation")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--seed-offset", type=int, default=0)
    ap.add_argument("--local", type=int, default=0)
    ap.add_argument("--frames", action="store_true")
    args = ap.parse_args()
    variation = C.load_variation(args.variation)
    vid = C.variation_id(args.variation)
    frames_dir = str(C.LOGS / "frames" / args.task / f"oracle_{vid}_{time.strftime('%Y%m%d-%H%M%S')}") \
        if args.frames else None
    recs = C.run_many([{"kind": "oracle", "task": args.task, "variation": variation, "seed": s, "pipeline": None,
                        "frames_dir": frames_dir}
                       for s in C.seed_list(args.task, args.seeds, args.seed_offset)], local_workers=args.local)
    rate = sum(bool(r.get("success")) for r in recs) / max(1, len(recs))
    print(json.dumps({"task": args.task, "variation": vid, "oracle_success_rate": rate,
                      "frames_dir": C.rel(frames_dir) if frames_dir else None, "per_seed": recs}, indent=2))


if __name__ == "__main__":
    main()
