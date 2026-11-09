"""Warm worker pool for RoboPRO episodes (ported from the RoboTwin backend).

A RoboPRO task object takes ~20 s to build (curobo warm-up), an episode a few seconds, so eval.py and
gate.py hand episodes to long-running workers instead of starting processes. Jobs, claims, results and
heartbeats are files under GAH_POOL_DIR (default benchmarks/robopro/.pool); a claim is an atomic rename.

python tools/pool.py worker --slots 8 [--slurm-node NODE]   # on each GPU node (env.sh sourced)
python tools/pool.py status
python tools/pool.py stop [--host H]                         # workers finish their episode, then exit
python tools/pool.py seeds --task put_cup_on_coaster --n 60  # writes seeds/<task>.json (expert-solvable)

The supervisor keeps the node usable for others: it starts a worker only while MemAvailable >= --start-gb,
stops the newest worker when MemAvailable < --min-gb or when the hottest thermal zone stays above
--max-temp for --hot-s seconds, and drops to --guest-slots while another user's Slurm job runs on the node.
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


_GPU_TEMP = {"t": 0.0, "v": 0.0}


def _max_temp_c() -> float:
    """Hottest ACPI thermal zone or GPU reading [C] (the GPU is read at most every 30 s)."""
    temps = []
    for p in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            temps.append(int(p.read_text().strip()) / 1000.0)
        except (OSError, ValueError):
            pass
    if time.time() - _GPU_TEMP["t"] > 30:
        _GPU_TEMP["t"] = time.time()
        try:
            out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader,nounits"],
                                 capture_output=True, text=True, timeout=10).stdout.split()
            _GPU_TEMP["v"] = float(out[0]) if out else 0.0
        except Exception:
            _GPU_TEMP["v"] = 0.0
    temps.append(_GPU_TEMP["v"])
    return max(temps)


def _others_on_node(node: str | None) -> list[str]:
    if not node:
        return []
    try:
        out = subprocess.run(["squeue", "-h", "-w", node, "-o", "%u"], capture_output=True, text=True,
                             timeout=20).stdout.split()
    except Exception:
        return []
    me = os.environ.get("USER", "")
    return [u for u in out if u != me]


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
        try:
            rec = C.run_job(job)
        except Exception as e:                # never let one job kill the worker (its claims would orphan)
            import traceback
            rec = {"seed": job.get("seed"), "success": False, "error": f"worker exception {type(e).__name__}: {e}",
                   "traceback": traceback.format_exc(limit=6)}
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
    print(f"[pool] {HOST}: target {args.slots} workers, start >= {args.start_gb} GB, stop < {args.min_gb} GB, "
          f"temp limit {args.max_temp} C for {args.hot_s} s", flush=True)
    last_slurm, others = 0.0, []
    hot_since = None
    while not stop_file.exists():
        for slot, (wid, p) in list(procs.items()):
            if p.poll() is not None:
                print(f"[pool] {wid} exited rc={p.returncode}", flush=True)
                del procs[slot]
        if time.time() - last_slurm > 60:
            others, last_slurm = _others_on_node(args.slurm_node), time.time()
        target = args.slots if not others else min(args.slots, args.guest_slots)
        mem = _mem_available_gb()
        temp = _max_temp_c()
        hot_since = (hot_since or time.time()) if temp > args.max_temp else None
        if procs and hot_since is not None and time.time() - hot_since >= args.hot_s:
            slot = max(procs)
            wid, p = procs.pop(slot)
            print(f"[pool] stopping {wid}: {temp:.0f} C for {time.time() - hot_since:.0f} s", flush=True)
            p.kill()
            hot_since = None
            paused_until = time.time() + 300
        elif procs and (mem < args.min_gb or len(procs) > target):
            slot = max(procs)                       # newest slot
            wid, p = procs.pop(slot)
            print(f"[pool] stopping {wid}: MemAvailable {mem:.1f} GB, target {target}, others {others}", flush=True)
            p.kill()
            paused_until = time.time() + 120
        elif len(procs) < target and mem >= args.start_gb and time.time() >= paused_until and temp <= args.max_temp - 4:
            slot = min(set(range(args.slots)) - set(procs))
            wid = f"{HOST}-{slot}"
            f = open(log / f"{wid}.log", "a")
            p = subprocess.Popen([sys.executable, __file__, "_child", wid], stdout=f, stderr=subprocess.STDOUT,
                                 cwd=str(C.BENCH))
            procs[slot] = (wid, p)
            print(f"[pool] started {wid} (MemAvailable {mem:.1f} GB)", flush=True)
            time.sleep(args.stagger)
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


def seeds(task, n, start=0, scan=None, local=0):
    """Keep the first n seeds whose scene is stable AND that RoboPRO's expert (play_once) solves.
    Scans seeds start .. start+scan-1 in parallel (pool, or --local processes)."""
    scan = scan or 2 * n
    recs = C.run_many([{"kind": "expert", "task": task, "seed": s, "pipeline": None}
                       for s in range(start, start + scan)], local_workers=local, timeout_s=7200)
    recs.sort(key=lambda r: r["seed"])
    good = [r["seed"] for r in recs if r.get("solved")]
    if len(good) < n:
        print(f"only {len(good)} of {scan} seeds usable; scan more (--scan)", file=sys.stderr)
    C.SEEDS_DIR.mkdir(parents=True, exist_ok=True)
    (C.SEEDS_DIR / f"{task}.json").write_text(json.dumps(
        {"task": task, "stable_seeds": good[:n], "filter": "stable scene AND solved by RoboPRO's expert",
         "scanned": [start, start + scan],
         "unstable_seeds_skipped": [r["seed"] for r in recs if not r.get("stable")],
         "expert_failed_skipped": [r["seed"] for r in recs if r.get("stable") and not r.get("solved")]}, indent=1))
    print(f"{task}: {min(n, len(good))} usable seeds of {scan} scanned "
          f"({sum(not r.get('stable') for r in recs)} unstable, "
          f"{sum(bool(r.get('stable')) and not r.get('solved') for r in recs)} expert failures) -> seeds/{task}.json")


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
    w.add_argument("--guest-slots", type=int, default=2)
    w.add_argument("--slurm-node", default=None)
    w.add_argument("--stagger", type=float, default=8.0)
    w.add_argument("--max-temp", type=float, default=89.0)
    w.add_argument("--hot-s", type=float, default=120.0)
    sub.add_parser("status")
    s = sub.add_parser("stop")
    s.add_argument("--host", default=HOST)
    sd = sub.add_parser("seeds")
    sd.add_argument("--task", required=True)
    sd.add_argument("--n", type=int, default=60)
    sd.add_argument("--start", type=int, default=0)
    sd.add_argument("--scan", type=int, default=None)
    sd.add_argument("--local", type=int, default=0)
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
        seeds(args.task, args.n, args.start, args.scan, args.local)


if __name__ == "__main__":
    main()
