"""DEV ONLY: run pipeline.py unchanged (no extra detections, no callbacks) and dump its LOG to
dev/out/wrap_<seed>.json."""
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def run(api):
    spec = importlib.util.spec_from_file_location("wrapped_pipeline", HERE.parent / "pipeline.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    err = None
    try:
        m.run(api)
    except Exception as e:
        err = f"{type(e).__name__}: {e}"
        raise
    finally:
        (HERE / "out").mkdir(exist_ok=True)
        (HERE / "out" / f"wrap_{api._genv.seed}.json").write_text(json.dumps({"err": err, "log": m.LOG}, default=str))
