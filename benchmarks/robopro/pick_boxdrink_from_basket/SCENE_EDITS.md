# Scene edits: RoboPRO backend

Variations are YAML files in `runs/<task>/variations/vK.yaml`:

```yaml
id: v3
axis: obstacle            # object_position | orientation | color | size | lighting | distractor | obstacle | object_motion | camera | dynamics
parameter_tuning: false   # true if only the magnitude of the previous variation's edit changes
assumption_violated: "the space above the basket is free"
edits:
  add_obstacle: {name: shelf, shape: box, size: [0.30, 0.25, 0.02], pose: [-0.39, 0.06, 1.25]}
```

Each key maps to one spec or a list of specs. Edits are applied after RoboPRO builds and settles the
seed's scene, then the scene settles again for 1 s. World frame: robot base at y = -0.65 facing +y,
table top z = 0.74 (x in [-0.6, 0.6], y in [-0.35, 0.35]), x > 0 is the robot's right.

## Supported keys

| Key | Spec | Notes |
|---|---|---|
| `object_pose` | `{name, xyz?, delta_xyz?, rpy?, delta_rpy?}` | `xyz` may be partial (`[x, y]`). `delta_rpy` rotates about world axes. Moving the `basket` carries the object that lies in it. |
| `object_color` | `{name, rgb}` | Render materials only. Perception is pose-based, so color alone cannot break a pipeline (the break gate will say so). Works on furniture too. |
| `object_scale` | `{name: <task object>, scale}` | Re-creates the task object with its RoboPRO mesh scaled by `scale` (contact points scale too). Its bottom stays where it was. |
| `swap_object` | `{name: <task object>, with, model_id?}` | Re-creates the task object as another RoboPRO asset (`001_bottle` or `bottle`) at the same pose. |
| `lighting` | `{intensity, color}` | Scales ambient, directional and point lights. Affects `camera_frame()` only. |
| `add_distractor` | `{name?, shape: box\|sphere\|cylinder\|asset:<model>, size, pose, rpy?, rgb?, mass?}` | Dynamic body (default 0.1 kg). `size`: full extents (box), diameter (sphere), `[diameter, length]` (cylinder, along x), scale factor (asset). `pose`: `[x, y, z]` or `[x, y, z, qw, qx, qy, qz]` in the world, or `{relative_to: <name>, offset: [dx, dy, dz], z?}` (the named object's origin plus a world-frame offset, optionally with an absolute height `z`; the layouts differ per seed). Appears in `objects()`, `detect()` and `get_obstacles()`. |
| `add_obstacle` | same as `add_distractor` | Static (immovable) body. |
| `object_motion` | `{name, velocity: [vx, vy, vz], start_time, duration=2.0, mode: constant\|impulse}` | Simulated seconds since the pipeline started (250 Hz). `constant` re-applies the velocity every step in the window; `impulse` once. Free (dynamic) objects only: the task object and distractors. |
| `camera_pose` | `{view: head\|front\|countertop, delta_xyz, delta_rpy}` | Moves a fixed camera (frames only). |
| `dynamics` | `{name, mass?, friction?}` | Object mass [kg]; friction (static = dynamic) of any rigid body, including the basket. |

Names (pick_boxdrink_from_basket): `boxdrink` (also `box_drink`, `drink`, `object`), `basket`, `table`,
`wall`, furniture `microwave`, `fridge`, `cabinet` (the wall cabinet); region `basket_interior`. Added bodies
use their `name`, or `distractor_i` / `obstacle_i`.

## Unsupported

| Key / case | Why |
|---|---|
| `object_pose`, `dynamics`, `object_motion` of furniture (`microwave`, `fridge`, `cabinet`) | Fixed URDF articulations that the basket or other objects may stand on; move the basket or add a body instead. |
| `object_motion` of the basket or of a static obstacle | Static bodies; use a distractor for a moving obstacle. |
| `object_scale` / `swap_object` of anything but the task object | The success check refers to the task object and the basket by identity. |
| `camera_pose` of a wrist camera | Wrist cameras ride on the arms. |

## Validity (gate.py, first 3 seeds)

Rejected if, after the edits settle: the task object is below the table top or outside x in [-0.55, 0.55],
y in [-0.33, 0.33]; the basket is off the table or out of that range; the task object or an added body
moved > 3 cm while settling (interpenetration, or dropped from the air); an added body fell off the table.
