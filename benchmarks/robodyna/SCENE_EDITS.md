# Scene edits: RoboDyna backend

Variations are YAML files in `runs/<task>/variations/vK.yaml` (docs/workflow.md section 8):

```yaml
id: v3
axis: obstacle            # object_position | orientation | color | size | lighting | distractor | obstacle | object_motion | camera | dynamics
parameter_tuning: false
assumption_violated: "the straight path from pre-grasp to grasp is free"
edits:
  add_obstacle: {asset: box, size: [0.05, 0.05, 0.15], pose: [-0.4, -0.1, 0.815]}
```

How edits are applied:
- Each key maps to one spec or a list of specs.
- They run inside the task's own `load_actors()` on every reset, before RoboDyna's settle and stability check. An edit that makes objects fall or interpenetrate therefore fails at reset (`UnStableError`), and the gate rejects it as invalid.
- The code is plain SAPIEN 3 (entities, PhysX components, render materials, lights), ported from the RoboTwin backend.

## Coordinates

- World frame, metres; quaternions are wxyz; `rpy` is in radians about the world axes.
- The table top is at z = 0.74.
- The robot bases are at y ≈ -0.65, facing +y. The left arm works at x < 0, the right arm at x > 0.
- **RoboDyna scenes are dynamic.** A task's own kinematics may keep driving an object every step, for example the bowl in `place_block_belt`, the basket in `pick_ripe_apple` and the sorter in `drop_ball_hole`. On such an object, an `object_pose` or `object_motion` edit can be overridden by the task. Prefer edits on objects that physics moves (a ball, a block, a distractor), or add new bodies.

## Supported keys

| Key | Spec | Notes |
|---|---|---|
| `object_pose` | `{name, xyz?, delta_xyz?, rpy?, delta_rpy?}` | Absolute values replace the task's pose; `delta_*` are added on top. |
| `object_color` | `{name, rgb}` | Flat colour, textures removed. This **does** change perception: `detect().rgb` reads the render colour (judging fruit in `pick_ripe_apple` depends on it). It also changes `camera_frame()`. |
| `lighting` | `{intensity, color?}` | Scales the ambient, directional and point lights. Images only. |
| `add_distractor` | `{name?, asset, size?, pose, rpy?, rgb?, mass?, model_id?}` | Dynamic body. `asset` is `box` \| `sphere` \| `cylinder` (`size` = full extents \| diameter \| `[diameter, height]`), or a model from RoboDyna's `assets/objects` (e.g. `001_bottle`). The default mass is 0.1 kg. It shows up in `objects()`, `detect()` and `get_obstacles()`. |
| `add_obstacle` | as `add_distractor` | A static (immovable) body. |
| `object_motion` | `{name, velocity, start_time, duration=2.0, mode: constant\|impulse}` | `constant` sets the velocity every step during the window; `impulse` sets it once. Time is sim seconds since the pipeline started. Kinematic objects are moved by pose (`constant` only). |
| `camera_pose` | `{view: head, delta_xyz, delta_rpy}` | Moves the head camera. Images only. |
| `dynamics` | `{name, mass?, friction?}` | Mass [kg] for dynamic objects; friction on every collision shape. |

Object names are the ones in README.md. An added object without a name gets `obstacle_i` / `distractor_i`. Parts (belts, keys, holes) can be detected but not edited.

## Not supported

| Key | Why |
|---|---|
| `object_scale` | RoboDyna task objects are procedural and driven by task kinematics; the task's own geometry (catch radii, belt lanes, hole sizes) is computed from their original size, so rescaling them would desynchronise the success check. |
| `swap_object` | Each task's success check is written for its own objects. Use `add_distractor` for a confusable object instead. |
| `camera_pose` for wrist views | The wrist cameras follow the arms. |

## Validity (gate.py)

On every gate seed, a variation is invalid if any of these hold:
- the edited scene fails RoboDyna's stability check;
- a task object is below the table top;
- the goal is already satisfied at reset;
- an added object is off the table or touches the robot at reset.

Objects are not required to sit on the table footprint, because RoboDyna scenes put balls on ramps and shelves. There is no solvability oracle.
