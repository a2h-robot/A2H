"""Solvability oracle: the official ManiSkill motion-planning solution, run on the edited task.

python benchmarks/maniskill/tools/oracle.py --task T [--variation V.yaml] --seeds 10

Added obstacles are passed to the oracle's planner as collision boxes. The oracle plans
open-loop from the initial state, so it cannot solve object_motion variations; gate.py
then requires the agent to show solvability by recovering (see SCENE_EDITS.md).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _common as C  # noqa: E402


def oracle_rate(task, variation, seeds, seed_offset=0, workers=None):
    recs = C.parallel(C.run_oracle_episode, [(task, variation, seed_offset + s) for s in range(seeds)],
                      workers or C.default_workers())
    return sum(r["success"] for r in recs) / max(1, len(recs)), recs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--variation")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--seed-offset", type=int, default=0)
    ap.add_argument("--workers", type=int, default=C.default_workers())
    args = ap.parse_args()
    variation = C.load_variation(args.variation)
    valid, reason = C.validity_episode(args.task, variation, args.seed_offset)
    rate, recs = oracle_rate(args.task, variation, args.seeds, args.seed_offset, args.workers)
    print(json.dumps({"task": args.task, "variation": C.variation_id(args.variation), "valid_scene": valid,
                      "validity_reason": reason, "oracle_success_rate": rate,
                      "solvable": bool(valid and rate > 0), "per_seed": recs}, indent=2))


if __name__ == "__main__":
    main()
