# Scene edits: LIBERO-Goal backend

Variations are YAML files in `runs/<task>/variations/vK.yaml`:

```yaml
id: v3
axis: obstacle            # object_position | orientation | color | size | lighting | distractor | obstacle | object_motion | camera | dynamics
parameter_tuning: false   # true if only the magnitude of the previous variation's edit changes
assumption_violated: "the straight path from pre-grasp to grasp is free"
edits:
  add_obstacle: {shape: box, size: [0.04, 0.20, 0.20], pose: [-0.10, 0.07, 1.00]}
```

Each edit key maps to one spec or a list of specs. Edits are applied at three stages:
- **BDDL**, before the env is built: `swap_object`;
- **MJCF**, before the model is compiled: `add_obstacle`, `add_distractor`, `object_scale`;
- **MuJoCo model and data**, after the base scene has settled: everything else.

The BDDL goal, which is the success check, is never changed.

Frame: world, z up. The table top is at z = 0.90 and the robot base at (-0.66, 0, 0.91); +x points away from the robot. `rpy` is extrinsic x-y-z, in radians.

## Supported keys (all ten)

| Key | Spec | Notes |
|---|---|---|
| `object_pose` | `{name, xyz?, delta_xyz?, rpy?, delta_rpy?}` | Absolute values replace the settled pose; `delta_*` are added to it. `delta_rpy` rotates about world axes. Works for movable objects and for fixtures (`wooden_cabinet_1`, `flat_stove_1`, `wine_rack_1`, which move with their regions). |
| `object_color` | `{name, rgb}` | Replaces the textures of the object's visual geoms with a flat color. Perception in this backend is pose-based, so color alone rarely breaks a pipeline (the break gate will say so). |
| `object_scale` | `{name, scale}` | Uniform scale in [0.5, 1.5]. Edits meshes, primitive sizes and body offsets before compilation, so collision geometry, mass (∝ s³) and `bbox_extents` all change. The object is lowered or raised so that it rests on the table. |
| `swap_object` | `{name, with}` | Replaces a movable object's asset with another LIBERO object category (e.g. `cream_cheese_1` with `butter`). The instance name is kept, so the goal is unchanged. The new object rests on the table. |
| `lighting` | `{intensity, color}` | Scales the diffuse, specular and ambient light of all lights. Affects `camera_frame()` only. |
| `add_distractor` | `{name?, shape: box\|sphere\|cylinder \| asset: <LIBERO category>, size, pose, rpy?, rgb?, mass?}` | Movable object. `size` is full extents (box), the diameter (sphere), or `[diameter, length]` with the axis along z (cylinder). `asset` instead loads a LIBERO object (e.g. `ketchup`, `milk`, `alphabet_soup`). `pose` is `[x, y, z]` or `[x, y, z, qw, qx, qy, qz]`. Appears in `objects()` and `get_obstacles()`. |
| `add_obstacle` | same as `add_distractor` | Fixed (immovable) body. |
| `object_motion` | `{name, velocity: [vx, vy, vz], start_time, duration=2.0, mode: constant\|impulse}` | Movable objects only. `constant` sets the linear velocity before every physics substep (500 Hz) during `[start_time, start_time + duration]`. With vz = 0 gravity is kept. `impulse` sets it once. Time is simulated seconds since the pipeline started. |
| `camera_pose` | `{view: head\|wrist, delta_xyz, delta_rpy}` | `head` = `agentview` (world frame); `wrist` = `robot0_eye_in_hand` (hand frame). Affects `camera_frame()` only. |
| `dynamics` | `{name, mass?, friction?, damping?}` | `mass` [kg] is the total over the object's bodies, with inertia scaled to match. `friction` is the sliding friction of all its collision geoms. `damping` (extension) is the joint damping of an articulated object, for example a stiff drawer or knob. |

Object names: `akita_black_bowl_1`, `cream_cheese_1`, `wine_bottle_1`, `plate_1` (movable); `wooden_cabinet_1`, `flat_stove_1`, `wine_rack_1` (fixtures). The `_1` suffix may be omitted. Added objects use their `name`, or `obstacle_i` / `distractor_i` if no name is given.

## Unsupported keys

None. Limits:
- `object_motion` cannot move fixtures, because they have no joint;
- `swap_object` only swaps movable objects, because fixtures are articulated and their joints are part of the init state.

## Validity

`gate.py` rejects a variation if any of these hold on seeds 0, 1 and 2 after the edits and settling:
- an object named in the goal is more than 0.85 m (horizontally) from the robot base;
- any object is more than 5 cm below the table top;
- an object drifts more than 3 cm while settling (interpenetration or an unstable pose); moving objects are exempt;
- the goal is already satisfied;
- an articulated goal (open a drawer, turn a knob) cannot reach a goal-satisfying joint value without the moving part hitting a fixed body.

LIBERO has no motion-planning oracle. Solvability is therefore shown by the agent, who must recover the variation within MAX_REPAIR iterations. Otherwise the variation is marked `unsolved` (prompts/gah_train.md, Phase 4).
