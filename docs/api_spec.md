# Shared primitive API

The agent writes Python against **one** API that has a sim backend (RoboTwin 2.0) and a real backend. Any function the real robot cannot support should be removed from both backends, so that the harness never relies on something the real robot lacks.

## Perception
- `detect(name) -> ObjectState | None`: pose, bounding box, and confidence for an object named in language. In sim this comes from a perception model rather than ground truth, where feasible. Otherwise add noise matched to the real detector.
- `get_obstacles() -> list[Box]`: occupied regions near the workspace.
- `track(name, duration_s) -> list[ObjectState]`: short pose history, used for moving objects.
- `camera_frame(view="head|wrist") -> image`

## Motion
- `move_to(pose, arm="left|right", speed="slow|normal")`, using a collision-aware planner.
- `grasp(obj, arm, strategy="top|side|auto") -> bool`
- `place(pose, arm) -> bool`
- `open_gripper(arm)`, `close_gripper(arm)`
- `wait_until(predicate, timeout_s) -> bool`

## Checks
- `gripper_holding(arm) -> bool`
- `in_region(obj, region) -> bool`

## Scene edits (training only, sim backend only)

These keys are allowed in `variations/vK.yaml`:

- `object_pose: {name, xyz, rpy}`
- `object_color: {name, rgb}`
- `object_scale: {name, scale}`
- `swap_object: {name, with}`
- `lighting: {intensity, color}`
- `add_distractor: {asset, pose}`
- `add_obstacle: {shape, size, pose}`
- `object_motion: {name, velocity, start_time}`
- `camera_pose: {view, delta_xyz, delta_rpy}`
- `dynamics: {name, mass, friction}`

## Safety limits (enforced in the backend, never in the prompt)

- Velocity and force limits.
- Workspace bounds.
- An emergency stop on the real robot.
