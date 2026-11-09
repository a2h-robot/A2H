"""Shared code for the LIBERO-Goal GAH tools (eval.py, gate.py, integrity.py)."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import multiprocessing as mp
import os
import subprocess
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor
from contextlib import redirect_stdout
from pathlib import Path

BENCH = Path(__file__).resolve().parents[1]          # benchmarks/libero
REPO = BENCH.parents[1]
RUNS = BENCH / "runs"
LOGS = BENCH / "logs"
TOOLS = BENCH / "tools"
sys.path.insert(0, str(BENCH))
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

PROTECTED = ["api.py", "motion.py", "gah_env.py", "scene_edits.py", "tools/_common.py", "tools/eval.py",
             "tools/gate.py", "tools/integrity.py"]
# LIBERO sources that implement the success check (BDDL goal -> predicates -> object states).
SUCCESS_SOURCES = [
    "envs/predicates/base_predicates.py",
    "envs/object_states/base_object_states.py",
    "envs/objects/articulated_objects.py",
    "envs/objects/site_object.py",
    "envs/objects/target_zones.py",
    "envs/problems/libero_tabletop_manipulation.py",
    "envs/bddl_base_domain.py",
]


def load_variation(path):
    if not path:
        return None
    import yaml
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def variation_id(path):
    return Path(path).stem if path else "base"


def git_blob(path) -> str:
    try:
        return subprocess.run(["git", "hash-object", str(path)], capture_output=True, text=True,
                              cwd=REPO, check=True).stdout.strip()[:12]
    except Exception:
        return hashlib.sha1(Path(path).read_bytes()).hexdigest()[:12]


def rel(path) -> str:
    try:
        return str(Path(path).resolve().relative_to(REPO))
    except ValueError:
        return str(path)


def append_log(record: dict):
    LOGS.mkdir(parents=True, exist_ok=True)
    with open(LOGS / "runs.jsonl", "a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")


# --------------------------------------------------------------------------- #
# Integrity: hashes of protected files and of every task's success check
# --------------------------------------------------------------------------- #
def _libero_root() -> Path:
    spec = importlib.util.find_spec("libero")
    return Path(list(spec.submodule_search_locations)[0]) / "libero"


def integrity_hashes() -> dict:
    with redirect_stdout(sys.stderr):
        import gah_env
    out = {}
    for f in PROTECTED:
        out[f] = hashlib.sha256((BENCH / f).read_bytes()).hexdigest()
    root = _libero_root()
    for f in SUCCESS_SOURCES:
        out[f"success_src:{f}"] = hashlib.sha256((root / f).read_bytes()).hexdigest()
    for task in gah_env.all_tasks():
        out[f"success:{task}"] = hashlib.sha256(Path(gah_env.bddl_path(task)).read_bytes()).hexdigest()
    return out


def integrity_check() -> tuple[bool, list[str]]:
    ref_path = TOOLS / "INTEGRITY.json"
    if not ref_path.exists():
        return False, ["tools/INTEGRITY.json missing"]
    ref = json.loads(ref_path.read_text())
    cur = integrity_hashes()
    bad = [k for k in ref if ref[k] != cur.get(k)]
    return not bad, bad


# --------------------------------------------------------------------------- #
# Episode workers (run in spawned processes)
# --------------------------------------------------------------------------- #
def _load_pipeline(path):
    spec = importlib.util.spec_from_file_location("gah_pipeline", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod, "run"):
        raise AttributeError(f"{path} must define run(api)")
    return mod


VIDEO_EVERY = 4   # control steps per video frame: 0.2 s of sim time, written at 5 fps (real time)


def run_episode(task, variation, seed, pipeline_path, frames_dir=None, settle_steps=10, video=False):
    """Run one episode of a pipeline. Returns a dict (never raises). Prints go to stderr.
    video=True also writes frames_dir/seed<seed>.mp4 (head camera, real time)."""
    with redirect_stdout(sys.stderr):
        return _run_episode(task, variation, seed, pipeline_path, frames_dir, settle_steps, video)


def _run_episode(task, variation, seed, pipeline_path, frames_dir, settle_steps, video=False):
    import gah_env
    from api import RobotAPI
    t0 = time.time()
    rec = {"seed": seed, "success": False, "error": None, "steps": 0}
    env = None
    frames = []
    try:
        env = gah_env.make_env(task, variation, seed=seed)
        api = RobotAPI(env, noise_seed=10_000 + seed)
        if video:
            frames.append(api.camera_frame("head"))
            env_step = env.step

            def step_and_capture(action):
                env_step(action)
                if env.steps % VIDEO_EVERY == 0:
                    frames.append(api.camera_frame("head"))
            env.step = step_and_capture
        mod = _load_pipeline(pipeline_path)
        try:
            mod.run(api)
        except gah_env.StepLimitExceeded as e:
            rec["error"] = f"StepLimitExceeded: {e}"
        try:
            api._hold(settle_steps)
        except gah_env.StepLimitExceeded:
            pass
        rec["success"] = env.success()
        rec["steps"] = env.steps
        if frames_dir:
            import imageio
            Path(frames_dir).mkdir(parents=True, exist_ok=True)
            imageio.imwrite(Path(frames_dir) / f"seed{seed}.png", api.camera_frame("head"))
    except Exception as e:  # pipeline bugs count as failures
        rec["error"] = f"{type(e).__name__}: {e}"
        rec["traceback"] = traceback.format_exc(limit=6)
    finally:
        if frames and frames_dir:  # also on pipeline errors: the video shows what went wrong
            try:
                import imageio
                if env is not None:
                    frames.append(api.camera_frame("head"))
                Path(frames_dir).mkdir(parents=True, exist_ok=True)
                imageio.mimsave(Path(frames_dir) / f"seed{seed}.mp4", frames, fps=1 / (VIDEO_EVERY * 0.05))
            except Exception as e:
                rec["video_error"] = f"{type(e).__name__}: {e}"
        if env is not None:
            env.close()
    rec["wall_clock_s"] = round(time.time() - t0, 2)
    return rec


def validity_episode(task, variation, seed):
    with redirect_stdout(sys.stderr):
        import gah_env
        env = None
        try:
            env = gah_env.make_env(task, variation, seed=seed)
            return env.validity()
        except Exception as e:
            return False, f"{type(e).__name__}: {e}"
        finally:
            if env is not None:
                env.close()


def parallel(fn, arg_list, workers):
    if workers <= 1:
        return [fn(*a) for a in arg_list]
    ctx = mp.get_context("spawn")
    with ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as ex:
        futs = [ex.submit(fn, *a) for a in arg_list]
        return [f.result() for f in futs]


def default_workers():
    return int(os.environ.get("GAH_WORKERS", min(10, (os.cpu_count() or 2) // 2)))
