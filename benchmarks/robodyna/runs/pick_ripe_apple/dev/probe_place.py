"""DEV ONLY: probe_gt on dev/pipeline_place.py."""
import importlib.util
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
os.environ["GAH_PROBE_PIPELINE"] = str(HERE / "pipeline_place.py")
os.environ["GAH_PROBE_TAG"] = "place"
_s = importlib.util.spec_from_file_location("probe_gt_mod", HERE / "probe_gt.py")
_m = importlib.util.module_from_spec(_s)
_s.loader.exec_module(_m)
run = _m.run
