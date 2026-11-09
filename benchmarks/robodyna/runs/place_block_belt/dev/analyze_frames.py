"""Dev-only: save frame 0 of dumped frames as PNG, upscaled, with the raw dark-run edges per row marked."""
import sys
from pathlib import Path
import numpy as np
from PIL import Image

out = Path(__file__).resolve().parent / "out"
for seed in sys.argv[1:]:
    d = np.load(out / f"frames_{seed}.npz")
    F = d["frames"]
    img = F[0].copy()
    Image.fromarray(img).resize((640, 480), Image.NEAREST).save(out / f"f0_{seed}.png")
