"""Drop the bowl on edge beside the rack's utensil holder: opening along (cos yaw, sin yaw, tilt), bowl box
centre (H/2 + gap) from the holder's face, rim bottom `rel` above the counter. Prints settle pose + success.
usage: python drop_yaw.py <seed> <rel> <yaws_deg comma> <tilts comma> <dys comma> <gaps comma>"""
import json, os, sys
sys.path.insert(0, os.getcwd())
import numpy as np, sapien, transforms3d as t3d
import gah_env as G
seed, rel = int(sys.argv[1]), float(sys.argv[2])
YAWS, TILTS, DYS, GAPS = ([float(v) for v in a.split(",")] for a in sys.argv[3:7])
env = G.make_task("place_bowl_in_dishrack_ks")
G.reset(env, seed)
c = np.asarray(env.dishrack_center, float)
face = c[0] + G.RACK_CUP[0][1]
R_up = t3d.quaternions.quat2mat([0.5, 0.5, 0.5, 0.5])
def q_axis(a):
    z = np.array([0, 0, 1.0]); v = np.cross(z, a); s = np.linalg.norm(v); cth = z @ a
    Rz = t3d.axangles.axangle2mat(v / s, np.arctan2(s, cth)) if s > 1e-9 else np.eye(3)
    return t3d.quaternions.mat2quat(Rz @ R_up)
for yaw in YAWS:
    for tilt in TILTS:
        for dy in DYS:
            for gap in GAPS:
                G.reset(env, seed)
                bowl = env.target_obj; comp = G._rigid(bowl.actor)
                a = np.array([np.cos(np.deg2rad(yaw)), np.sin(np.deg2rad(yaw)), tilt]); a /= np.linalg.norm(a)
                q = q_axis(a)
                bowl.actor.set_pose(sapien.Pose([0, 0, 2.0], q))
                lo, hi = G._mesh_world_aabb(comp)
                off = np.array([0, 0, 2.0]) - (lo + hi) / 2
                cx = face + (hi[0] - lo[0]) / 2 + gap          # box's -x side `gap` from the holder face
                centre = np.array([cx, c[1] + dy, 0.74 + rel + (hi[2] - lo[2]) / 2])
                bowl.actor.set_pose(sapien.Pose(centre + off, q))
                comp.set_linear_velocity([0, 0, 0]); comp.set_angular_velocity([0, 0, 0])
                for _ in range(750):
                    env.scene.step()
                d = G.diagnostics(env)
                print(json.dumps({"yaw": yaw, "tilt": tilt, "dy": dy, "gap": gap, "ok": bool(env.check_success()),
                                  "z": round(d["bowl_xyz"][2], 3), "d": d["bowl_minus_rack_xyz"], "cos": d["bowl_upright_cos"]}), flush=True)
