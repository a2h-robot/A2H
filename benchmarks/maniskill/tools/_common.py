"""Shared code for the ManiSkill3 GAH tools (eval.py, gate.py, oracle.py)."""

from __future__ import annotations

import hashlib
import importlib.util
import inspect
import json
import multiprocessing as mp
import os
import subprocess
import sys
import time
import traceback
from contextlib import redirect_stdout
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

BENCH = Path(__file__).resolve().parents[1]          # benchmarks/maniskill
REPO = BENCH.parents[1]
RUNS = BENCH / "runs"
LOGS = BENCH / "logs"
TOOLS = BENCH / "tools"
sys.path.insert(0, str(BENCH))

PROTECTED = ["api.py", "gah_env.py", "scene_edits.py", "tools/_common.py", "tools/eval.py",
             "tools/gate.py", "tools/oracle.py", "tools/integrity.py"]


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
def integrity_hashes() -> dict:
    import gah_env
    from mani_skill.utils.registration import REGISTERED_ENVS
    out = {}
    for f in PROTECTED:
        out[f] = hashlib.sha256((BENCH / f).read_bytes()).hexdigest()
    for task in gah_env.TASKS:
        src = inspect.getsource(REGISTERED_ENVS[task].cls.evaluate)
        out[f"evaluate:{task}"] = hashlib.sha256(src.encode()).hexdigest()
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


def run_episode(task, variation, seed, pipeline_path, frames_dir=None, settle_steps=10):
    """Run one episode of a pipeline. Returns a dict (never raises). Prints go to stderr."""
    with redirect_stdout(sys.stderr):
        return _run_episode(task, variation, seed, pipeline_path, frames_dir, settle_steps)


def _run_episode(task, variation, seed, pipeline_path, frames_dir, settle_steps):
    import numpy as np
    import gah_env
    from api import RobotAPI
    t0 = time.time()
    rec = {"seed": seed, "success": False, "error": None, "steps": 0}
    env = None
    try:
        env = gah_env.make_env(task, variation)
        env.reset(seed=seed)
        api = RobotAPI(env, noise_seed=10_000 + seed)
        mod = _load_pipeline(pipeline_path)
        try:
            mod.run(api)
        except gah_env.StepLimitExceeded as e:
            rec["error"] = f"StepLimitExceeded: {e}"
        try:
            api._step_hold(settle_steps)
        except gah_env.StepLimitExceeded:
            pass
        rec["success"] = env.gah_success()
        rec["steps"] = env.gah_steps
        if frames_dir:
            import imageio
            Path(frames_dir).mkdir(parents=True, exist_ok=True)
            imageio.imwrite(Path(frames_dir) / f"seed{seed}.png", api.camera_frame())
    except Exception as e:  # pipeline bugs count as failures
        rec["error"] = f"{type(e).__name__}: {e}"
        rec["traceback"] = traceback.format_exc(limit=6)
    finally:
        if env is not None:
            env.close()
    rec["wall_clock_s"] = round(time.time() - t0, 2)
    return rec


def run_oracle_episode(task, variation, seed):
    with redirect_stdout(sys.stderr):
        return _run_oracle_episode(task, variation, seed)


def _run_oracle_episode(task, variation, seed):
    """Run the official ManiSkill motion-planning solution on the (edited) task.
    Added obstacles are given to the oracle's planner as collision boxes."""
    import numpy as np
    import sapien
    import gah_env
    from mani_skill.examples.motionplanning.panda import motionplanner as mpmod
    from mani_skill.examples.motionplanning.panda.run import MP_SOLUTIONS
    from mani_skill.examples.motionplanning.base_motionplanner.utils import get_actor_obb
    t0 = time.time()
    rec = {"seed": seed, "success": False, "error": None}
    env = None
    try:
        env = gah_env.make_env(task, variation)
        orig_init = mpmod.PandaArmMotionPlanningSolver.__init__

        def patched_init(self, *a, **kw):
            orig_init(self, *a, **kw)
            for name, actor in env.gah_extra.items():
                obb = get_actor_obb(actor)
                T = np.asarray(obb.primitive.transform)
                self.add_box_collision(np.asarray(obb.primitive.extents) + 0.02, sapien.Pose(T))

        mpmod.PandaArmMotionPlanningSolver.__init__ = patched_init
        try:
            MP_SOLUTIONS[task](env, seed=seed, debug=False, vis=False)
        except gah_env.StepLimitExceeded as e:
            rec["error"] = f"StepLimitExceeded: {e}"
        finally:
            mpmod.PandaArmMotionPlanningSolver.__init__ = orig_init
        rec["success"] = env.gah_success()
    except Exception as e:
        rec["error"] = f"{type(e).__name__}: {e}"
    finally:
        if env is not None:
            env.close()
    rec["wall_clock_s"] = round(time.time() - t0, 2)
    return rec


def validity_episode(task, variation, seed):
    with redirect_stdout(sys.stderr):
        return _validity_episode(task, variation, seed)


def _validity_episode(task, variation, seed):
    import gah_env
    import scene_edits
    env = None
    try:
        env = gah_env.make_env(task, variation)
        env.reset(seed=seed)
        return scene_edits.scene_validity(env)
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
