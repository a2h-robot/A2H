# Scene edits: RoboPRO backend

Variations are YAML files in `runs/<task>/variations/vK.yaml`:

```yaml
id: v3
axis: obstacle            # object_position | orientation | color | size | lighting | distractor | obstacle | object_motion | camera | dynamics
parameter_tuning: false   # true if only the magnitude of the previous variation's edit changes
assumption_violated: "the straight path from the cup to the coaster is free"
edits:
  add_obstacle: {name: post, shape: box, size: [0.04, 0.04, 0.20], pose: [0.20, 0.00, 0.84]}
```

Each key maps to one spec or a list of specs. Edits are applied after RoboPRO builds and settles the
seed's scene. The scene then settles again for 1 s.

World frame: robot base at y = -0.65 facing +y; table top z = 0.74, x in [-0.6, 0.6], y in
[-0.35, 0.35]; x > 0 is the robot's right.

## Supported keys

| Key | Spec | Notes |
|---|---|---|
| `object_pose` | `{name, xyz?, delta_xyz?, rpy?, delta_rpy?}` | `xyz` may be partial (`[x, y]`). `delta_rpy` rotates about world axes. Works on the task objects and on the furniture (static bodies are teleported). |
| `object_color` | `{name, rgb}` | Render materials only. Perception is pose-based, so color alone cannot break a pipeline (the break gate will say so). |
| `object_scale` | `{name: cup \| coaster, scale}` | Re-creates the object with its RoboPRO mesh scaled by `scale`; contact points scale too. |
| `swap_object` | `{name: cup \| coaster, with, model_id?}` | Re-creates the object as another RoboPRO asset (e.g. `039_mug` or `mug`) at the same pose. The success check follows it. |
| `lighting` | `{intensity, color}` | Scales ambient, directional and point lights. Affects `camera_frame()` only. |
| `add_distractor` | `{name?, shape: box\|sphere\|cylinder\|asset:<model>, size, pose, rpy?, rgb?, mass?}` | Dynamic body (default 0.1 kg). `size`: full extents (box), diameter (sphere), `[diameter, length]` (cylinder, along x), or a scale factor (asset). `pose`: `[x, y, z]` or `[x, y, z, qw, qx, qy, qz]`. Appears in `objects()`, `detect()` and `get_obstacles()`. |
| `add_obstacle` | same as `add_distractor` | Static (immovable) body. |
| `object_motion` | `{name, velocity: [vx, vy, vz], start_time, duration=2.0, mode: constant\|impulse}` | Times are simulated seconds since the pipeline started (250 Hz). `constant` re-applies the velocity every step in the window; `impulse` applies it once. Dynamic bodies only: the task objects and distractors. |
| `camera_pose` | `{view: head, delta_xyz, delta_rpy}` | Moves the head camera (frames only). |
| `dynamics` | `{name, mass?, friction?}` | Mass [kg] and friction (static = dynamic) of a dynamic body; friction also works on static bodies. |

Names (put_cup_on_coaster):
- task objects: `cup` (also `021_cup`, `object`) and `coaster` (`019_coaster`);
- furniture: `bookcase` (`014_bookcase`) and `wooden_box` (`042_wooden_box`, `box`);
- `table` and `wall`;
- region: `coaster_top` (the space above the coaster).

Added bodies use their `name`, or `distractor_i` / `obstacle_i`.

## Unsupported

| Key / case | Why |
|---|---|
| `dynamics.damping` | Joint damping; the study scene has no articulated objects. |
| `object_motion` of furniture or a static obstacle | Static bodies do not move; use a distractor for a moving obstacle. |
| `object_scale` / `swap_object` of furniture | The furniture layout comes from RoboPRO's scene config; move it with `object_pose` instead. |
| `camera_pose` of a wrist camera | Wrist cameras ride on the arms. |

## Validity (gate.py, first 3 seeds)

A variation is rejected if, after the edits settle:
- the cup or the coaster is below the table top, or outside x in [-0.55, 0.55], y in [-0.33, 0.33];
- a task object or an added body moved more than 3 cm while settling (interpenetration, or dropped from
  the air);
- an added body fell off the table.
