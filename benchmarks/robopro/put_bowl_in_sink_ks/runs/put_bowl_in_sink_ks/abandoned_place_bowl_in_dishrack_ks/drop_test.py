"""Drop the bowl into the rack from given poses (no robot) and record where it settles and whether
check_success holds. usage: python drop_test.py <seed> <release_bottom_above_counter_m>"""
import json, os, sys
sys.path.insert(0, os.getcwd())
import numpy as np, sapien, transforms3d as t3d
import gah_env as G

seed, rel = int(sys.argv[1]), float(sys.argv[2])
env = G.make_task("place_bowl_in_dishrack_ks")
G.reset(env, seed)
c = np.asarray(env.dishrack_center, float)
bowl = env.target_obj
comp = G._rigid(bowl.actor)
q_up = np.array([0.5, 0.5, 0.5, 0.5])                    # model y -> world z (as spawned)
R_up = t3d.quaternions.quat2mat(q_up)
def q_axis(world_axis):
    """bowl opening along world_axis"""
    a = np.asarray(world_axis, float); a /= np.linalg.norm(a)
    # rotation taking world z to a, applied after R_up
    z = np.array([0, 0, 1.0]); v = np.cross(z, a); s = np.linalg.norm(v); cth = z @ a
    if s < 1e-9:
        Rz = np.eye(3) if cth > 0 else t3d.axangles.axangle2mat([1, 0, 0], np.pi)
    else:
        Rz = t3d.axangles.axangle2mat(v / s, np.arctan2(s, cth))
    return t3d.quaternions.mat2quat(Rz @ R_up)
ORIENT = {k: v for k, v in {"up": [0, 0, 1], "down": [0, 0, -1], "face_-y": [0, -1, 0], "face_+x": [1, 0, 0],
          "face_-x": [-1, 0, 0], "face_+y": [0, 1, 0], "lean_-y": [0, -1, 0.35], "lean_+y": [0, 1, 0.35], "tilt45_-y": [0, -1, 1], "tilt60_-y": [0, -1, 0.58]}.items()
          if k in os.environ.get("ORIENTS", "up,down,face_-y,face_+x,tilt45_-y,tilt60_-y").split(",")}
DXS = [float(v) for v in os.environ.get("DXS", "-0.06,-0.02,0.02,0.06").split(",")]
DYS = [float(v) for v in os.environ.get("DYS", "-0.05,0,0.05").split(",")]
table = 0.74
for name, ax in ORIENT.items():
    for dx in DXS:
        for dy in DYS:
            G.reset(env, seed)
            bowl = env.target_obj; comp = G._rigid(bowl.actor)
            q = q_axis(ax)
            bowl.actor.set_pose(sapien.Pose([0, 0, 2.0], q))
            lo, hi = G._mesh_world_aabb(comp)
            off = np.array([0, 0, 2.0]) - (lo + hi) / 2                  # origin relative to the AABB centre
            centre = np.array([c[0] + dx, c[1] + dy, table + rel + (hi[2] - lo[2]) / 2])
            bowl.actor.set_pose(sapien.Pose(centre + off, q))
            comp.set_linear_velocity([0, 0, 0]); comp.set_angular_velocity([0, 0, 0])
            for _ in range(750):
                env.scene.step()
            d = G.diagnostics(env)
            print(json.dumps({"o": name, "dx": dx, "dy": dy, "ok": bool(env.check_success()),
                              "z": round(d["bowl_xyz"][2], 3), "d": d["bowl_minus_rack_xyz"], "cos": d["bowl_upright_cos"]}), flush=True)
