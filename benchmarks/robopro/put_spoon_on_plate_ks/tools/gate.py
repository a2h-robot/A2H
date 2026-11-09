"""Variance + break gate for one designed task (docs/workflow.md section 3).

python benchmarks/robopro/tools/gate.py --task T --variation benchmarks/robopro/runs/T/variations/vK.yaml

Checks, in order:
  1. integrity  - protected tool files and every task's success check are unchanged (tools/INTEGRITY.json)
  2. edits      - every edit key is supported (scene_edits.py) and the variation edits something
  3. validity   - on the first 3 seeds: objects on the table and in reach, no interpenetration on reset
  4. streak     - at most 2 parameter-tuning variations in a row (agent label OR automatic diff against
                  the previous accepted variation)
  5. break      - the frozen pre-round pipeline (git HEAD version of runs/T/pipeline.py) has
                  success <= s0 / 2 on the variation, where s0 is its success on the base task
There is no solvability oracle: RoboPRO's expert (play_once) plans with its own collision world and does not
see scene edits (added bodies, motions), so it cannot judge a variation (see ISSUES.md). It only picks the
seeds; an unsolvable variation is caught by MAX_REPAIR (marked `unsolved`).
Writes runs/T/variations/vK.gate.json, appends passes to runs/T/gate_history.jsonl, prints JSON.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _common as C  # noqa: E402
import eval as E  # noqa: E402


def frozen_pipeline(task) -> tuple[Path | None, str]:
    relpath = f"benchmarks/robopro/runs/{task}/pipeline.py"
    r = subprocess.run(["git", "show", f"HEAD:{relpath}"], capture_output=True, text=True, cwd=C.REPO)
    if r.returncode != 0:
        return None, "pipeline.py is not committed; commit it before gating"
    blob = subprocess.run(["git", "rev-parse", f"HEAD:{relpath}"], capture_output=True, text=True,
                          cwd=C.REPO).stdout.strip()[:12]
    d = C.BENCH / ".cache" / "frozen"            # shared storage: pool workers on other nodes read it
    d.mkdir(parents=True, exist_ok=True)
    path = d / f"{task}-{blob}.py"
    path.write_text(r.stdout, encoding="utf-8")
    return path, blob


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
    ap.add_argument("--local", type=int, default=0)
    args = ap.parse_args()
    vpath = Path(args.variation).resolve()
    variation = C.load_variation(vpath) or {}
    result = {"task": args.task, "variation": vpath.stem, "pass": False, "time": time.strftime("%Y-%m-%d %H:%M:%S")}

    def finish():
        out = vpath.with_name(vpath.stem + ".gate.json")
        out.write_text(json.dumps(result, indent=2))
        if result["pass"]:
            with open(C.RUNS / args.task / "gate_history.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps({k: result.get(k) for k in ("variation", "time", "parameter_tuning", "s0", "s_break")}
                                   | {"edits": variation.get("edits", {})}) + "\n")
        C.append_log({"phase": "train", "arm": "GAH-gate", "task": args.task, "variation": vpath.stem,
                      "backend": "sim", "attempt": None, "success": bool(result["pass"]),
                      "success_rate": result.get("s_break"), "seeds": args.seeds, "tokens_in": None,
                      "tokens_out": None, "wall_clock_s": None, "program": result.get("frozen_pipeline"),
                      "notes": f"gate.py: {result.get('reason')}"})
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

    checks = C.run_many([{"kind": "validity", "task": args.task, "variation": variation, "seed": s,
                          "pipeline": None} for s in C.seed_list(args.task, 3, 0)], local_workers=args.local)
    bad_v = [c for c in checks if not c.get("valid")]
    result["validity"] = "ok" if not bad_v else bad_v[0].get("why")
    if bad_v:
        result["reason"] = f"invalid scene (seed {bad_v[0]['seed']}): {bad_v[0].get('why')}"
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
    scenes = ([None] if key not in cache else []) + [str(vpath)]
    res = E.evaluate_many(args.task, scenes, args.seeds, 0, frozen, frames=True, arm="GAH-gate", local=args.local,
                          notes="gate.py frozen pipeline")
    if key not in cache:
        cache[key] = res[0]["success_rate"]
        cache_path.write_text(json.dumps(cache, indent=2))
    s0, s_break = cache[key], res[-1]["success_rate"]
    result.update(s0=s0, s_break=s_break, frames_dir=res[-1]["frames_dir"],
                  break_failures=[r.get("last_error") or r.get("error") for r in res[-1]["per_seed"] if not r["success"]][:5])
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
