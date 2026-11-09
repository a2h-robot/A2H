"""Write or check tools/INTEGRITY.json (hashes of protected files and task success checks).

python benchmarks/robopro/tools/integrity.py --write   # maintainers only, after changing tools
python benchmarks/robopro/tools/integrity.py           # check
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _common as C  # noqa: E402

if __name__ == "__main__":
    if "--write" in sys.argv:
        (C.TOOLS / "INTEGRITY.json").write_text(json.dumps(C.integrity_hashes(), indent=2))
        print("wrote", C.rel(C.TOOLS / "INTEGRITY.json"))
    else:
        ok, bad = C.integrity_check()
        print(json.dumps({"ok": ok, "modified": bad}))
        sys.exit(0 if ok else 1)
