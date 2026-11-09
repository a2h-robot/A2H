"""Variance + break gate for one designed task (docs/workflow.md section 3).

python benchmarks/maniskill/tools/gate.py --task T --variation benchmarks/maniskill/runs/T/variations/vK.yaml

Checks, in order:
  1. integrity  - protected tool files and the task's success check are unchanged (tools/INTEGRITY.json)
  2. edits      - every edit key is supported (scene_edits.py)
  3. validity   - objects in reach, above the table, no interpenetration on reset
  4. streak     - at most 2 parameter-tuning variations in a row (agent label + automatic check)
  5. break      - the frozen pre-round pipeline (git HEAD version of runs/T/pipeline.py)
                  has success <= s0 / 2 on the variation, where s0 is its success on the base task
  6. oracle     - official motion-planning solution on the variation; if it fails (e.g. moving
                  objects, which the open-loop oracle cannot handle), the variation passes with
                  needs_agent_solve = true and only counts once the agent recovers (Phase 4).
Writes runs/T/variations/vK.gate.json, appends passes to runs/T/gate_history.jsonl, prints JSON.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _common as C  # noqa: E402
import eval as E  # noqa: E402
import oracle as O  # noqa: E402


def frozen_pipeline(task) -> tuple[Path | None, str]:
    relpath = f"benchmarks/maniskill/runs/{task}/pipeline.py"
    r = subprocess.run(["git", "show", f"HEAD:{relpath}"], capture_output=True, text=True, cwd=C.REPO)
    if r.returncode != 0:
        return None, "pipeline.py is not committed; commit it before gating"
    tmp = Path(tempfile.mkdtemp(prefix="gah_frozen_")) / "pipeline.py"
    tmp.write_text(r.stdout, encoding="utf-8")
    blob = subprocess.run(["git", "rev-parse", f"HEAD:{relpath}"], capture_output=True, text=True,
                          cwd=C.REPO).stdout.strip()[:12]
    return tmp, blob


def flatten(d, prefix=""):
    out = {}
    if isinstance(d, dict):
        for k, v in d.items():
            out.update(flatten(v, f"{prefix}{k}."))
    elif isinstance(d, list):
        if all(isinstance(x, (int, float)) for x in d):
            out[prefix.rstrip(".")] = d
        else:
            for i, v in enumerate(d):
                out.update(flatten(v, f"{prefix}{i}."))
    else:
        out[prefix.rstrip(".")] = d
    return out


def auto_tuning(edits, prev_edits) -> bool:
    """True if edits has the same structure and the same non-numeric values as prev_edits,
    so that it only changes numeric magnitudes."""
    if prev_edits is None:
        return False
    a, b = flatten(edits), flatten(prev_edits)
    if set(a) != set(b):
        return False
    numeric = lambda x: isinstance(x, (int, float)) or (isinstance(x, list) and all(isinstance(y, (int, float)) for y in x))
    return all(a[k] == b[k] for k in a if not numeric(a[k]))


def load_history(task):
    path = C.RUNS / task / "gate_history.jsonl"
    if not path.exists():
        return []
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--variation", required=True)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--oracle-seeds", type=int, default=10)
    ap.add_argument("--workers", type=int, default=C.default_workers())
    args = ap.parse_args()
    vpath = Path(args.variation)
    variation = C.load_variation(vpath) or {}
    result = {"task": args.task, "variation": vpath.stem, "pass": False, "time": time.strftime("%Y-%m-%d %H:%M:%S")}

    def finish():
        out = vpath.with_name(vpath.stem + ".gate.json")
        out.write_text(json.dumps(result, indent=2))
        if result["pass"]:
            with open(C.RUNS / args.task / "gate_history.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps({k: result.get(k) for k in ("variation", "time", "parameter_tuning", "s0", "s_break",
                                                               "oracle_success_rate", "needs_agent_solve")}
                                   | {"edits": variation.get("edits", {})}) + "\n")
        print(json.dumps(result, indent=2))
        sys.exit(0 if result["pass"] else 1)

    ok, bad = C.integrity_check()
    if not ok:
        result["reason"] = f"integrity: modified {bad}"
        finish()

    import scene_edits
    try:
        scene_edits.normalize(variation.get("edits", {}))
    except scene_edits.UnsupportedEdit as e:
        result["reason"] = f"unsupported edit: {e}"
        finish()
    if not variation.get("edits"):
        result["reason"] = "no-op variation: no edits"
        finish()

    valid, why = C.validity_episode(args.task, variation, 0)
    result["validity"] = why
    if not valid:
        result["reason"] = f"invalid scene: {why}"
        finish()

    history = load_history(args.task)
    label = bool(variation.get("parameter_tuning", False))
    auto = auto_tuning(variation.get("edits", {}), history[-1]["edits"] if history else None)
    is_tuning = label or auto
    streak = 0
    for h in reversed(history):
        if h.get("parameter_tuning"):
            streak += 1
        else:
            break
    result.update(parameter_tuning=is_tuning, parameter_tuning_label=label, parameter_tuning_auto=auto,
                  tuning_streak_before=streak)
    if is_tuning and streak >= 2:
        result["reason"] = "variance gate: third parameter-tuning variation in a row"
        finish()

    frozen, blob = frozen_pipeline(args.task)
    if frozen is None:
        result["reason"] = blob
        finish()
    result["frozen_pipeline"] = blob
    cache_path = C.RUNS / args.task / ".gate_cache.json"
    cache = json.loads(cache_path.read_text()) if cache_path.exists() else {}
    key = f"{blob}:{args.seeds}"
    if key not in cache:
        cache[key] = E.evaluate(args.task, None, args.seeds, 0, frozen, args.workers, frames=False,
                                arm="GAH-gate")["success_rate"]
        cache_path.write_text(json.dumps(cache, indent=2))
    s0 = cache[key]
    s_break = E.evaluate(args.task, str(vpath), args.seeds, 0, frozen, args.workers, frames=True,
                         arm="GAH-gate")["success_rate"]
    result.update(s0=s0, s_break=s_break)
    if s0 <= 0:
        result["reason"] = "break gate: frozen pipeline has s0 = 0 on the base task; recover it first"
        finish()
    if s_break > s0 / 2:
        result["reason"] = f"break gate: s_break={s_break:.2f} > s0/2={s0 / 2:.2f}"
        finish()

    rate, _ = O.oracle_rate(args.task, variation, args.oracle_seeds, 0, args.workers)
    result["oracle_success_rate"] = rate
    result["needs_agent_solve"] = rate == 0
    result["pass"] = True
    result["reason"] = "pass" + (" (oracle could not solve it: counts only after the agent recovers)" if rate == 0 else "")
    finish()


if __name__ == "__main__":
    main()
