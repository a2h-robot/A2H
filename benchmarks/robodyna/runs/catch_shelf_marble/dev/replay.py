"""DEV ONLY (simulator internals for readout): run a pipeline on seeds (optionally a variation)
and print the true ball / bowl / key state over time. Never used by pipeline.py.
python .../dev/replay.py IDX[,IDX..] [variation.yaml|-] [pipeline.py|-] [delay,delay,..] [quiet]
A delay (s) is waited before run(api) starts, to probe timing sensitivity."""
import sys
from pathlib import Path
BENCH = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(BENCH), str(BENCH / "tools")]
import numpy as np  # noqa
import _common as C  # noqa
import gah_env  # noqa
from api import RobotAPI  # noqa

idxs = [int(a) for a in sys.argv[1].split(",")]
var = C.load_variation(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] != "-" else None
pipe = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3] != "-" else str(BENCH / "runs" / "catch_shelf_marble" / "pipeline.py")
delays = [float(a) for a in sys.argv[4].split(",")] if len(sys.argv) > 4 else [0.0]
quiet = len(sys.argv) > 5
every = 25
g = gah_env.GahEnv("catch_shelf_marble")
for idx in idxs:
    for delay in delays:
        seed = C.task_seeds("catch_shelf_marble")[idx]
        g.reset(seed, var)
        e = g.env
        api = RobotAPI(g, noise_seed=10_000 + seed)
        print(f"##### idx {idx} delay {delay} target", round(e.target_catch_x, 3), "bowl range",
              round(e.bowl_x_min, 3), round(e.bowl_x_max, 3))

        def cb(n, e=e):
            if n % every == 0 and not quiet:
                p = e.ball.get_pose().p
                print(f"t={n / 250:6.2f} ball=({p[0]:+.3f},{p[2]:.3f}) bowl={e.bowl.get_pose().p[0]:+.3f} "
                      f"keys={'L' if e._key_pressed['left'] else '-'}{'R' if e._key_pressed['right'] else '-'} "
                      f"st={e._marble_state} leg={e._leg_idx} tgt={e.target_catch_x:+.3f}", flush=True)

        g.step_callbacks.append(cb)
        try:
            if delay:
                api.wait(delay)
            C._load_pipeline(pipe).run(api)
        except gah_env.StepLimitExceeded as ex:
            print("step limit", ex)
        g.finish()
        print(f"success idx={idx} delay={delay}", g.success(), "at", g.success_step, "result", e._marble_result,
              "ball", np.round(e.ball.get_pose().p, 3).tolist(), "bowl", round(float(e.bowl.get_pose().p[0]), 3))
        if not quiet:
            for t in api.trace[-12:]:
                print("trace", t)
        g.close()
