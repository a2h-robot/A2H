"""Sanity (e): every scene-edit key applies on seed 0 of the bank and passes the validity check.
Run from benchmarks/robopro with env.sh sourced: $SIM_PY sanity/all_keys.py"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import _common as C  # noqa: E402

TASK = "put_cup_on_coaster"
EDITS = {
    "object_pose": {"name": "coaster", "delta_xyz": [0.05, 0.03, 0.0], "delta_rpy": [0, 0, 0.5]},
    "object_color": {"name": "cup", "rgb": [0.9, 0.1, 0.1]},
    "object_scale": {"name": "cup", "scale": 1.2},
    "swap_object": {"name": "cup", "with": "039_mug"},
    "lighting": {"intensity": 0.4, "color": [1.0, 0.9, 0.8]},
    "add_distractor": {"shape": "box", "size": [0.04, 0.04, 0.06], "pose": [0.0, 0.25, 0.77]},
    "add_obstacle": {"shape": "box", "size": [0.03, 0.03, 0.15], "pose": [0.0, -0.30, 0.815]},
    "object_motion": {"name": "cup", "velocity": [0.05, 0.0, 0.0], "start_time": 1.0},
    "camera_pose": {"view": "head", "delta_xyz": [0.0, 0.0, 0.05], "delta_rpy": [0, 0.1, 0]},
    "dynamics": {"name": "cup", "mass": 0.3, "friction": 0.3},
}
if __name__ == "__main__":
    seed = C.seed_list(TASK, 1, 0)[0]
    jobs = [{"kind": "validity", "task": TASK, "variation": {"edits": {k: v}}, "seed": seed, "pipeline": None}
            for k, v in EDITS.items()]
    recs = C.run_many(jobs)
    out = {k: {"valid": r.get("valid"), "why": r.get("why")} for k, r in zip(EDITS, recs)}
    print(json.dumps(out, indent=1))
