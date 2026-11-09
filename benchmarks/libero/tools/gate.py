"""Variance + break gate for one designed task (docs/workflow.md section 3).

python benchmarks/libero/tools/gate.py --task T --variation benchmarks/libero/runs/T/variations/vK.yaml

Checks, in order:
  1. integrity  - protected tool files and the task's success check are unchanged (tools/INTEGRITY.json):
                  benchmarks/libero/{api,motion,gah_env,scene_edits}.py, tools/*.py, the task's BDDL file
                  and the LIBERO sources that evaluate its goal
  2. edits      - every edit key is supported (scene_edits.py) and there is at least one edit
  3. validity   - on 3 seeds: goal objects in reach, nothing below the table, no interpenetration
                  (> 3 cm drift while settling), goal not already satisfied
  4. streak     - at most 2 parameter-tuning variations in a row (agent label + automatic check)
  5. break      - the frozen pre-round pipeline (git HEAD version of runs/T/pipeline.py)
                  has success <= s0 / 2 on the variation, where s0 is its success on the base task
LIBERO has no solvability oracle: a variation that the agent cannot recover within MAX_REPAIR
iterations is marked unsolved (prompts/gah_train.md, Phase 4).
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

VALIDITY_SEEDS = 3


def frozen_pipeline(task) -> tuple[Path | None, str]:
    relpath = f"benchmarks/libero/runs/{task}/pipeline.py"
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
    ap.add_argument("--workers", type=int, default=C.default_workers())
    args = ap.parse_args()
    vpath = Path(args.variation)
    variation = C.load_variation(vpath) or {}
    result = {"task": args.task, "variation": vpath.stem, "pass": False, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    t0 = time.time()

    def finish():
        out = vpath.with_name(vpath.stem + ".gate.json")
        out.write_text(json.dumps(result, indent=2))
        if result["pass"]:
            with open(C.RUNS / args.task / "gate_history.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps({k: result.get(k) for k in ("variation", "time", "parameter_tuning", "s0", "s_break")}
                                   | {"edits": variation.get("edits", {})}) + "\n")
        C.append_log({
            "phase": "train", "arm": "GAH-gate", "task": args.task, "variation": vpath.stem, "backend": "sim",
            "attempt": None, "success": result["pass"], "success_rate": result.get("s_break"),
            "seeds": args.seeds, "tokens_in": None, "tokens_out": None,
            "wall_clock_s": round(time.time() - t0, 1),
            "program": f"benchmarks/libero/runs/{args.task}/pipeline@{result.get('frozen_pipeline')}",
            "notes": f"gate.py: {result.get('reason')}",
        })
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

    checks = C.parallel(C.validity_episode, [(args.task, variation, s) for s in range(VALIDITY_SEEDS)],
                        min(args.workers, VALIDITY_SEEDS))
    bad_seeds = [(s, why) for s, (valid, why) in enumerate(checks) if not valid]
    result["validity"] = "ok" if not bad_seeds else {f"seed{s}": why for s, why in bad_seeds}
    if bad_seeds:
        result["reason"] = f"invalid scene: {bad_seeds[0][1]} (seed {bad_seeds[0][0]})"
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
    result["pass"] = True
    result["reason"] = "pass"
    finish()


if __name__ == "__main__":
    main()
