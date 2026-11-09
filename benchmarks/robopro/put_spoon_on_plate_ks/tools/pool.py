"""Warm worker pool for RoboPRO episodes, shared by several GPU nodes through files on shared storage.

A RoboPRO task object takes ~20 s to build (curobo warm-up), an episode a few seconds, so eval.py and
gate.py hand episodes to long-running workers instead of starting processes. Jobs, claims, results and
heartbeats are files under GAH_POOL_DIR (default benchmarks/robopro/.pool); a claim is an atomic rename.

python tools/pool.py worker --slots 4 [--slurm-node <node>]          # on each GPU node (env.sh sourced)
python tools/pool.py status
python tools/pool.py stop [--host H]                                  # workers finish their episode, then exit
python tools/pool.py seeds --task put_spoon_on_plate_ks --n 40       # writes seeds/<task>.json

The supervisor keeps the node usable for others: it starts a worker only while MemAvailable >= --start-gb,
stops the newest worker when MemAvailable < --min-gb, and drops to --guest-slots while another user's
Slurm job runs on the node.
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
    last_slurm, others = 0.0, []
    while not stop_file.exists():
        for slot, (wid, p) in list(procs.items()):
            if p.poll() is not None:
                print(f"[pool] {wid} exited rc={p.returncode}", flush=True)
                del procs[slot]
        if time.time() - last_slurm > 60:
            others, last_slurm = _others_on_node(args.slurm_node), time.time()
        target = args.slots if not others else min(args.slots, args.guest_slots)
        mem = _mem_available_gb()
        if procs and (mem < args.min_gb or len(procs) > target):
            slot = max(procs)                       # newest slot
            wid, p = procs.pop(slot)
            print(f"[pool] stopping {wid}: MemAvailable {mem:.1f} GB, target {target}, others {others}", flush=True)
            p.kill()
            paused_until = time.time() + 120
        elif len(procs) < target and mem >= args.start_gb and time.time() >= paused_until:
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


def seeds(task, n, start=None):
    """Stable, expert-solvable seeds: RoboPRO's frozen clean seed bank for the task first, then the
    integers after it. A seed is kept if its scene builds (no UnStableError) and RoboPRO's own scripted
    solution (play_once) succeeds on the unedited scene, so every base-task seed is known to be solvable."""
    os.environ["GAH_CUROBO_MESH"] = "100"          # RoboPRO's own planner world loads the scene meshes
    import gah_env
    gah_env.chdir_root()
    from envs.utils.create_actor import UnStableError
    env = C.get_env(task)
    bank_file = Path(gah_env.BENCH_ROOT) / "eval_seeds" / task / f"{gah_env.TASK_CONFIG[task]}.txt"
    bank = [int(x) for x in bank_file.read_text().split()] if bank_file.exists() else []
    nxt = start if start is not None else (max(bank) + 1 if bank else 0)
    cands = iter(bank + list(range(nxt, nxt + 100000)))
    stable, unstable, expert_failed = [], [], []
    while len(stable) < n:
        s = next(cands)
        try:
            (stable if gah_env.expert_solves(env, s) else expert_failed).append(s)
        except UnStableError:
            unstable.append(s)
        except Exception as e:  # planner errors inside the expert: not a usable seed
            expert_failed.append(s)
            print(f"seed {s}: {type(e).__name__}: {e}", flush=True)
        finally:
            gah_env.close(env)
        print(f"seed {s}: {len(stable)} kept, {len(expert_failed)} expert-failed, {len(unstable)} unstable", flush=True)
    C.SEEDS_DIR.mkdir(parents=True, exist_ok=True)
    (C.SEEDS_DIR / f"{task}.json").write_text(json.dumps(
        {"task": task, "config": gah_env.TASK_CONFIG[task], "stable_seeds": stable,
         "expert_failed_seeds_skipped": expert_failed, "unstable_seeds_skipped": unstable,
         "source": f"RoboPRO eval_seeds/{task}/{gah_env.TASK_CONFIG[task]}.txt, then following integers"},
        indent=1))
    print(f"{task}: {len(stable)} stable expert-solvable seeds ({len(expert_failed)} expert-failed, "
          f"{len(unstable)} unstable skipped) -> seeds/{task}.json")


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
    sub.add_parser("status")
    s = sub.add_parser("stop")
    s.add_argument("--host", default=HOST)
    sd = sub.add_parser("seeds")
    sd.add_argument("--task", required=True)
    sd.add_argument("--n", type=int, default=40)
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
        seeds(args.task, args.n)


if __name__ == "__main__":
    main()
