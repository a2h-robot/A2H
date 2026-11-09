"""Release the bowl near the expert's perched pose (on edge across the utensil holder's top and the front
wall, opening towards the robot) and see whether it settles there.
usage: python drop_perch.py <seed> <dzs> <dxs> <dys> <tilts>   (offsets in m from the perched origin)"""
import json, os, sys
sys.path.insert(0, os.getcwd())
import numpy as np, sapien, transforms3d as t3d
import gah_env as G
seed = int(sys.argv[1])
DZS, DXS, DYS, TILTS = ([float(v) for v in a.split(",")] for a in sys.argv[2:6])
env = G.make_task("place_bowl_in_dishrack_ks")
R_up = t3d.quaternions.quat2mat([0.5, 0.5, 0.5, 0.5])
def q_axis(a):
    z = np.array([0, 0, 1.0]); v = np.cross(z, a); s = np.linalg.norm(v); cth = z @ a
    Rz = t3d.axangles.axangle2mat(v / s, np.arctan2(s, cth)) if s > 1e-9 else np.eye(3)
    return t3d.quaternions.mat2quat(Rz @ R_up)
PERCH = np.array([-0.063, -0.040, 0.030])
for tilt in TILTS:
    for dz in DZS:
        for dx in DXS:
            for dy in DYS:
                G.reset(env, seed)
                c = np.asarray(env.dishrack_center, float)
                bowl = env.target_obj; comp = G._rigid(bowl.actor)
                a = np.array([0.0, -1.0, tilt]); a /= np.linalg.norm(a)
                bowl.actor.set_pose(sapien.Pose(c + PERCH + [dx, dy, dz], q_axis(a)))
                comp.set_linear_velocity([0, 0, 0]); comp.set_angular_velocity([0, 0, 0])
                for _ in range(750):
                    env.scene.step()
                d = G.diagnostics(env)
                print(json.dumps({"tilt": tilt, "dz": dz, "dx": dx, "dy": dy, "ok": bool(env.check_success()),
                                  "z": round(d["bowl_xyz"][2], 3), "d": d["bowl_minus_rack_xyz"]}), flush=True)
