# Backend issues seen in the place_block_belt run

## 1. `object_motion` (mode constant) does not move a block resting on the table

- Variation `variations/v6.yaml`: `object_motion: {name: block, velocity: [0, -0.03, 0], start_time: 3.0, duration: 6.0, mode: constant}`.
- Probe `dev/motion_probe.py` (true block pose every 1 s, seed 10000): the block stays at
  `[-0.306, 0.002, 0.760]` from t = 0 to t = 11 s, i.e. it never moves.
- `scene_edits.on_step` does call `set_linear_velocity` on the (dynamic, non-kinematic) block every
  physics step inside the window. The likely cause is contact friction: RoboDyna gives the block
  static friction 1.6, so the per-step friction impulse cap (mu * m * g * dt = 1.6 * 0.05 * 9.81 / 250
  ~ 3.1e-3 N s) exceeds the momentum the edit injects (m * v = 1.5e-3 N s), and PhysX cancels the
  velocity within the step. A constant-velocity edit therefore silently does nothing for slow speeds on
  grippy objects; faster speeds would move the object at an unpredictable, friction-reduced speed.
- Consequence here: v6 was rejected by the break gate (s_break = 1.0) because the scene was effectively
  unchanged. This is not a failure of the world to respond to the robot (grasping, lifting and the belt
  all work), so the run continued with other variations.
- Suggestion for the backend: for `constant` mode on a dynamic body, drive it kinematically (or set
  its pose) during the window, or document that the speed is not guaranteed under contact friction.

## 2. CUDA out-of-memory at reset on the shared machine

- In the final `eval.py --all` (after round 5), seed 10005 of v1 and seed 10007 of v5 failed at reset with
  `OutOfMemoryError: CUDA out of memory` (other processes held most of GPU 0); both records are marked
  `invalid: true`, steps 0. The pipeline never ran on them, so v1 and v5 read 0.9 instead of 9/9 runs
  that started. Infrastructure, not the task; GAH_WORKERS=2 was used throughout.
