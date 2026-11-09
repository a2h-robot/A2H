# Scene edits: RoboPRO backend

Variations are YAML files in `runs/<task>/variations/vK.yaml`:

```yaml
id: v3
axis: obstacle            # object_position | orientation | color | size | lighting | distractor | obstacle | object_motion | camera | dynamics
parameter_tuning: false   # true if only the magnitude of the previous variation's edit changes
assumption_violated: "the straight path from the spoon to the plate is free"
edits:
  add_obstacle: {name: post, shape: box, size: [0.04, 0.04, 0.30], pose: [0.0, -0.10, 0.89]}
```

Each key maps to one spec or a list of specs. Edits are applied after RoboPRO builds and settles the
seed's scene, then the scene settles again for 1 s. World frame: robot base at y = -0.65 facing +y,
counter top z = 0.74 (x in [-0.6, 0.6], y in [-0.35, 0.35]), x > 0 is the robot's right.

The small-kitchen scene (`kitchens`) has three layouts, picked by the seed: microwave / dish rack / sink at
x = (-0.32, 0.05, 0.42), (-0.32, 0.42, 0.10) or (0.10, -0.32, 0.42), all at the back of the counter
(y = 0.08-0.18). The sink is a hole in the counter.

## Names (put_spoon_on_plate_ks)

| Name | Body | Notes |
|---|---|---|
| `spoon` (also `134_spoon`) | task object, dynamic, 0.05 kg | its model origin is what the success check measures |
| `plate` (also `003_plate`) | target, **static** | the success check reads its pose live, so moving it moves the goal |
| `microwave` | articulated fixture | `object_pose` moves its root |
| `dishrack` | static fixture | |
| `table`, `wall` | counter, back wall | cannot be edited |
| `sink` | basin + rims | part of the counter (a hole in it); cannot be edited |

Added bodies use their `name`, or `distractor_i` / `obstacle_i`.

## Supported keys

| Key | Spec | Notes |
|---|---|---|
| `object_pose` | `{name, xyz?, delta_xyz?, rpy?, delta_rpy?}` | `xyz` may be partial (`[x, y]`). `delta_rpy` rotates about world axes. Works on the spoon, the plate, the dish rack and the microwave. |
| `object_color` | `{name, rgb}` | Render materials only. Perception is pose-based, so color alone cannot break a pipeline (the break gate will say so). |
| `object_scale` | `{name: spoon\|plate, scale}` | Re-creates the object with its RoboPRO mesh scaled by `scale` (contact points scale too). The spoon keeps its mass; the plate stays static. |
| `swap_object` | `{name: spoon\|plate, with, model_id?}` | Re-creates the object as another asset of `benchmark/assets/objects` (`048_stapler` or `stapler`) at the same pose. |
| `lighting` | `{intensity, color}` | Scales ambient, directional and point lights. Affects `camera_frame()` only. |
| `add_distractor` | `{name?, shape: box\|sphere\|cylinder\|asset:<model>, size, pose, rpy?, rgb?, mass?}` | Dynamic body (default 0.1 kg). `size`: full extents (box), diameter (sphere), `[diameter, length]` (cylinder, along x), scale factor (asset). `pose`: `[x, y, z]` or `[x, y, z, qw, qx, qy, qz]`. Appears in `objects()`, `detect()` and `get_obstacles()`. |
| `add_obstacle` | same as `add_distractor` | Static (immovable) body. |
| `object_motion` | `{name, velocity: [vx, vy, vz], start_time, duration=2.0, mode: constant\|impulse}` | Simulated seconds since the pipeline started (250 Hz). `constant` re-applies the velocity every step in the window; `impulse` once. Dynamic bodies get a velocity; static ones (plate, dish rack, static obstacles) and the microwave root are moved kinematically. |
| `camera_pose` | `{view: head, delta_xyz, delta_rpy}` | Moves the head camera (frames only). |
| `dynamics` | `{name, mass?, friction?}` | mass [kg] of a dynamic body; friction (static = dynamic) of any body, e.g. a slippery spoon or plate. |

## Unsupported

| Key / case | Why |
|---|---|
| any edit of `table`, `wall` or `sink` | The sink is a hole cut in the counter; moving either would leave a hole or a floating basin. |
| `object_scale` / `swap_object` of fixtures | Only the task objects are re-created. |
| `dynamics: damping` | The task uses no articulation joint. |
| `camera_pose` of a wrist camera | Wrist cameras ride on the arms. |

## Validity (gate.py, first 3 seeds)

Rejected if, after the edits settle:
- the spoon or the plate is below the counter top (off the counter or in the sink), outside
  x in [-0.58, 0.58], y in [-0.33, 0.33], or (the static plate) over the sink hole;
- the spoon or an added dynamic body moved > 3 cm while settling (interpenetration, or dropped from the air);
- an added body fell off the counter or into the sink.
