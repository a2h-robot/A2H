"""Dev-only: contact sheet of an eval mp4 (every k-th frame), saved next to it."""
import sys
from pathlib import Path
import imageio
import numpy as np
from PIL import Image

mp4, k = Path(sys.argv[1]), int(sys.argv[2]) if len(sys.argv) > 2 else 5
start = int(sys.argv[3]) if len(sys.argv) > 3 else 0
fr = [f for i, f in enumerate(imageio.mimread(mp4, memtest=False)) if i >= start and (i - start) % k == 0][:16]
h, w = fr[0].shape[:2]
sheet = np.zeros((h * 4, w * 4, 3), np.uint8)
for i, f in enumerate(fr):
    sheet[(i // 4) * h:(i // 4 + 1) * h, (i % 4) * w:(i % 4 + 1) * w] = f[..., :3]
Image.fromarray(sheet).save(mp4.with_suffix(f".sheet{start}.png"))
print(len(fr), "frames")
