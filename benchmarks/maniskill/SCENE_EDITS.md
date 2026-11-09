# Scene edits: ManiSkill3 backend

Variations are YAML files in `runs/<task>/variations/vK.yaml`:

```yaml
id: v3
axis: obstacle            # object_position | orientation | color | lighting | distractor | obstacle | object_motion | camera | dynamics
parameter_tuning: false   # true if only the magnitude of the previous variation's edit changes
assumption_violated: "the straight path from pre-grasp to grasp is free"
edits:
  add_obstacle: {shape: box, size: [0.05, 0.05, 0.15], pose: [0.0, 0.05, 0.075]}
```

Each edit key maps to one spec or a list of specs. The edits live inside the env (`gah_env.make_env`), so they are re-applied on every reset. They therefore also apply to the official motion-planning oracle.

## Supported keys

| Key | Spec | Notes |
|---|---|---|
| `object_pose` | `{name, xyz?, delta_xyz?, rpy?, delta_rpy?}` | Absolute values replace the task's random pose; `delta_*` are added on top of it. `delta_rpy` rotates about world axes. Moving `receptacle` in PlugCharger also moves its goal. |
| `object_color` | `{name, rgb}` | Changes render materials only. Perception in this backend is pose-based, so color alone rarely breaks a pipeline (the break gate will say so). |
| `lighting` | `{intensity, color}` | Scales the ambient light and both directional lights. Affects `camera_frame()` only. |
| `add_distractor` | `{name?, shape: box\|sphere\|cylinder, size, pose, rgb?, mass?}` | Dynamic object. `size` is full extents (box), the diameter (sphere), or `[diameter, length]` along x (cylinder). `pose` is `[x, y, z]` or `[x, y, z, qw, qx, qy, qz]`, plus optional `rpy`. Appears in `objects()` and `get_obstacles()`. |
| `add_obstacle` | same as `add_distractor` | Kinematic (immovable) object. |
| `object_motion` | `{name, velocity: [vx, vy, vz], start_time, duration=2.0, mode: constant\|impulse}` | `constant` re-applies the velocity every control step during `[start_time, start_time + duration]`; `impulse` applies it once. Time is simulated seconds since reset (20 Hz control). |
| `camera_pose` | `{view: head, delta_xyz, delta_rpy}` | Moves the head (render) camera. |
| `dynamics` | `{name, mass?, friction?}` | Sets mass [kg] and static/dynamic friction on all collision shapes of the object. |

Object names: `peg`, `box_with_hole` (PegInsertionSide); `charger`, `receptacle` (PlugCharger); `cube`, `l_shape_tool` (PullCubeTool); `peg` (LiftPegUpright); `sphere`, `bin` (PlaceSphere). Added objects use their `name`, or `obstacle_i` / `distractor_i` if no name is given.

## Unsupported keys

| Key | Why |
|---|---|
| `object_scale` | Each task builds its objects' collision geometry once, when it loads the scene. Changing their size would need a per-task rebuild and would change the task's own success thresholds. |
| `swap_object` | The task objects and their success checks are defined by the task class. |

As a substitute for both, use `add_distractor` with a similar or confusable shape next to the target object.

## Validity and solvability

`gate.py` rejects a variation if any of these hold after reset:
- a task object is more than 0.85 m (horizontally) from the robot base;
- a task object is below the table;
- any object drifts more than 3 cm while settling (interpenetration).

Moving objects are exempt from the drift check.

The oracle is the official ManiSkill motion-planning solution, with added obstacles as collision boxes. It plans open-loop from the initial state, so it cannot solve `object_motion` variations. When the oracle fails, the variation still passes with `needs_agent_solve: true`. Per docs/workflow.md it then only counts once the agent recovers it in Phase 4; otherwise it is marked `unsolved`.
