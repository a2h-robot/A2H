"""Warm worker pool for RoboPRO episodes, shared by several GPU nodes through files on shared storage.

A RoboPRO task object takes ~20 s to build (curobo warm-up), an episode a few seconds, so eval.py and
gate.py hand episodes to long-running workers instead of starting processes. Jobs, claims, results and
heartbeats are files under GAH_POOL_DIR (default benchmarks/robopro/.pool); a claim is an atomic rename.

python tools/pool.py worker --slots 10 [--slurm-node <node-name>]   # on each GPU node (env.sh sourced)
python tools/pool.py status
python tools/pool.py stop [--host H]                                  # workers finish their episode, then exit
python tools/pool.py seeds --task pick_boxdrink_from_basket --n 80    # writes seeds/<task>.json

The supervisor keeps the node usable for others. On a GB10 the node's memory guard kills the largest
*Slurm job* when memory runs low, never an out-of-Slurm guest, so a guest pool must leave headroom for jobs:
  - no Slurm job on the node: start a worker only while MemAvailable >= --start-gb, stop the newest
    worker when MemAvailable < --min-gb;
  - a Slurm job on the node: run at most --guest-slots workers, and no more than fit in the memory the
    jobs have NOT requested (MemTotal - their Slurm memory requests - 5 GB, at --worker-gb each: a job is
    entitled to all it asked for, even while it uses less); also keep MemAvailable >= --hosted-min-gb
    (start only above it + 20 GB: a worker takes ~10 GB as it warms up, so a narrower band oscillates);
  - a job whose name starts with one of --yield-prefixes (default "bb", the backbone study): 0 workers.
The node's Slurm jobs are checked every 30 s.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _common as C  # noqa: E402

HOST = socket.gethostname()


def _mem_available_gb() -> float:
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1e6
    return 0.0


def _mem_gb(s: str) -> float:
    """Slurm memory string (e.g. 96G, 4000M) -> GB."""
    try:
        unit = s[-1].upper()
        v = float(s[:-1]) if unit in "KMGT" else float(s)
        return v * {"K": 1e-6, "M": 1e-3, "G": 1.0, "T": 1e3}.get(unit, 1e-3)
    except (ValueError, IndexError):
        return 0.0


def _mem_total_gb() -> float:
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                return int(line.split()[1]) / 1e6
    return 0.0


def _jobs_on_node(node: str | None) -> list[tuple[str, str, float]]:
    """(user, job name, requested memory GB) of every Slurm job on the node."""
    if not node:
        return []
    try:
        out = subprocess.run(["squeue", "-h", "-w", node, "-o", "%u|%j|%m"], capture_output=True, text=True,
                             timeout=20).stdout.split()
    except Exception:
        return []
    jobs = []
    for l in out:
        parts = l.split("|")
        if len(parts) == 3:
            jobs.append((parts[0], parts[1], _mem_gb(parts[2])))
    return jobs


# --------------------------------------------------------------------------- #
def child(wid: str):
    """One worker: claim a job, run it, write the result, repeat."""
    for d in ("jobs", "claimed", "results", "workers"):
        (C.POOL / d).mkdir(parents=True, exist_ok=True)
    state = {"job": None, "episodes": 0, "stop": False}
    hb = C.POOL / "workers" / f"{wid}.json"

    def beat():
        while not state["stop"]:
            tmp = hb.with_suffix(".tmp")
            tmp.write_text(json.dumps({"wid": wid, "host": HOST, "pid": os.getpid(), "job": state["job"],
                                       "episodes": state["episodes"], "t": time.time()}))
            os.replace(tmp, hb)
            time.sleep(10)

    threading.Thread(target=beat, daemon=True).start()
    signal.signal(signal.SIGTERM, lambda *a: state.update(stop=True))
    stop_file = C.POOL / f"stop-{HOST}"
    backend = ["gah_env.py", "scene_edits.py", "api.py", "tools/_common.py"]
    mtimes = {f: (C.BENCH / f).stat().st_mtime for f in backend}
    while not state["stop"] and not stop_file.exists():
        now = {f: (C.BENCH / f).stat().st_mtime for f in backend}
        if now != mtimes:                     # backend edited: reload it, keep the warm task objects
            import importlib
            envs = C._ENVS
            for name in ("gah_env", "scene_edits", "api"):
                if name in sys.modules:
                    importlib.reload(sys.modules[name])
            importlib.reload(C)
            C._ENVS.update(envs)
            mtimes = now
            print(f"[{wid}] reloaded backend", flush=True)
        claimed = None
        for f in sorted((C.POOL / "jobs").glob("*.json")):
            dst = C.POOL / "claimed" / f"{f.name}.{wid}"
            try:
                os.rename(f, dst)
                claimed = dst
                break
            except (FileNotFoundError, OSError):
                continue
        if claimed is None:
            time.sleep(0.3)
            continue
        job = json.loads(claimed.read_text())
        state["job"] = job["id"]
        rec = C.run_job(job)
        rec["worker"] = wid
        tmp = C.POOL / "results" / f".{job['id']}.tmp"
        tmp.write_text(json.dumps(rec))
        os.replace(tmp, C.POOL / "results" / f"{job['id']}.json")
        claimed.unlink(missing_ok=True)
        state["job"] = None
        state["episodes"] += 1
    state["stop"] = True
    hb.unlink(missing_ok=True)


def supervise(args):
    stop_file = C.POOL / f"stop-{HOST}"
    stop_file.unlink(missing_ok=True)
    procs: dict[int, tuple[str, subprocess.Popen]] = {}
    paused_until = 0.0
    log = C.POOL / "logs"
    log.mkdir(parents=True, exist_ok=True)
    print(f"[pool] {HOST}: target {args.slots} workers, start >= {args.start_gb} GB, stop < {args.min_gb} GB", flush=True)
    last_slurm, jobs = 0.0, []
    prefixes = tuple(x for x in args.yield_prefixes.split(",") if x)
    while not stop_file.exists():
        for slot, (wid, p) in list(procs.items()):
            if p.poll() is not None:
                print(f"[pool] {wid} exited rc={p.returncode}", flush=True)
                del procs[slot]
        if time.time() - last_slurm > 30:
            jobs, last_slurm = _jobs_on_node(args.slurm_node), time.time()
        yield_to = [j for _, j, _ in jobs if prefixes and j.startswith(prefixes)]
        if yield_to:
            target, min_gb, start_gb = 0, args.min_gb, args.start_gb
        elif jobs:
            unclaimed = _mem_total_gb() - sum(m for _, _, m in jobs) - 5.0
            target = max(0, min(args.slots, args.guest_slots, int(unclaimed // args.worker_gb)))
            min_gb, start_gb = max(args.min_gb, args.hosted_min_gb), max(args.start_gb, args.hosted_min_gb + 20)
        else:
            target, min_gb, start_gb = args.slots, args.min_gb, args.start_gb
        mem = _mem_available_gb()
        if procs and (mem < min_gb or len(procs) > target):
            slot = max(procs)                       # newest slot
            wid, p = procs.pop(slot)
            print(f"[pool] stopping {wid}: MemAvailable {mem:.1f} GB (floor {min_gb}), target {target}, "
                  f"slurm jobs {jobs}", flush=True)
            p.kill()
            paused_until = time.time() + (300 if jobs else 120)
        elif len(procs) < target and mem >= start_gb and time.time() >= paused_until:
            slot = min(set(range(args.slots)) - set(procs))
            wid = f"{HOST}-{slot}"
            f = open(log / f"{wid}.log", "a")
            p = subprocess.Popen([sys.executable, __file__, "_child", wid], stdout=f, stderr=subprocess.STDOUT,
                                 cwd=str(C.BENCH))
            procs[slot] = (wid, p)
            print(f"[pool] started {wid} (MemAvailable {mem:.1f} GB)", flush=True)
            time.sleep(args.stagger if not jobs else 75)   # beside a job: let it warm up before the next decision
            continue
        time.sleep(5)
    print("[pool] stop file seen; waiting for workers to finish their episode", flush=True)
    for wid, p in procs.values():
        p.wait()


def status():
    ws = C.live_workers()
    by_host = {}
    for w in ws:
        by_host.setdefault(w["host"], []).append(w)
    for h, lst in sorted(by_host.items()):
        busy = sum(1 for w in lst if w.get("job"))
        print(f"{h}: {len(lst)} workers, {busy} busy, {sum(w['episodes'] for w in lst)} episodes done")
    q = len(list((C.POOL / "jobs").glob("*.json"))) if (C.POOL / "jobs").exists() else 0
    c = len(list((C.POOL / "claimed").glob("*"))) if (C.POOL / "claimed").exists() else 0
    print(f"queue: {q} waiting, {c} running")


def seeds(task, n, local=0, extra=40):
    """Keep RoboPRO's clean-config eval seeds (validated on x86) whose scene is stable here and that the
    scripted expert solves; write the first n to seeds/<task>.json."""
    import gah_env
    scene, config = gah_env.TASKS[task]
    bank = Path(os.environ["BENCH_ROOT"]) / "eval_seeds" / task / f"{config}.txt"
    cands = [int(x) for x in bank.read_text().split()]
    cands += [s for s in range(cands[0], cands[0] + 10 * len(cands)) if s not in cands][:extra]   # beyond the bank
    C.SEEDS_DIR.mkdir(parents=True, exist_ok=True)
    path = C.SEEDS_DIR / f"{task}.json"
    tmp = C.SEEDS_DIR / f".{task}.all.json"          # seed_list() needs a file to read the candidates from
    tmp.write_text(json.dumps({"stable_seeds": cands}))
    os.replace(tmp, path)
    recs = C.run_many([{"kind": "oracle", "task": task, "variation": None, "seed": s, "pipeline": None}
                       for s in cands], local_workers=local, timeout_s=7200)
    good, bad = [], []
    for r in recs:
        start = r.get("start") or {}
        ok = r.get("success") and start.get("box_in_basket_xy", True) and not r.get("error")
        (good if ok else bad).append({"seed": r["seed"], "scene_id": start.get("scene_id"),
                                      "error": r.get("error"), "expert": bool(r.get("success"))})
    path.write_text(json.dumps({"task": task, "source": f"benchmark/eval_seeds/{task}/{config}.txt + {extra} sequential",
                                "stable_seeds": [g["seed"] for g in good][:n],
                                "scene_ids": [g["scene_id"] for g in good][:n],
                                "rejected": bad}, indent=1))
    print(f"{task}: {len(good)}/{len(cands)} seeds solved by the expert -> seeds/{task}.json ({min(n, len(good))} kept)")


def main():
    if len(sys.argv) >= 3 and sys.argv[1] == "_child":
        child(sys.argv[2])
        return
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    w = sub.add_parser("worker")
    w.add_argument("--slots", type=int, default=8)
    w.add_argument("--min-gb", type=float, default=20.0)
    w.add_argument("--start-gb", type=float, default=30.0)
    w.add_argument("--guest-slots", type=int, default=2, help="max workers while a Slurm job is on the node")
    w.add_argument("--hosted-min-gb", type=float, default=60.0, help="MemAvailable floor while a Slurm job is on the node")
    w.add_argument("--yield-prefixes", default="bb", help="comma list of job-name prefixes that stop the pool")
    w.add_argument("--worker-gb", type=float, default=10.0, help="node memory one busy worker costs (GB10: incl. GPU)")
    w.add_argument("--slurm-node", default=None)
    w.add_argument("--stagger", type=float, default=8.0)
    sub.add_parser("status")
    s = sub.add_parser("stop")
    s.add_argument("--host", default=HOST)
    sd = sub.add_parser("seeds")
    sd.add_argument("--task", required=True)
    sd.add_argument("--n", type=int, default=80)
    sd.add_argument("--local", type=int, default=0)
    sd.add_argument("--extra", type=int, default=40, help="sequential seeds tried after RoboPRO's seed bank")
    args = ap.parse_args()
    if args.cmd == "worker":
        supervise(args)
    elif args.cmd == "status":
        status()
    elif args.cmd == "stop":
        (C.POOL / f"stop-{args.host}").parent.mkdir(parents=True, exist_ok=True)
        (C.POOL / f"stop-{args.host}").touch()
        print(f"stop requested for {args.host}")
    elif args.cmd == "seeds":
        seeds(args.task, args.n, args.local, args.extra)


if __name__ == "__main__":
    main()
