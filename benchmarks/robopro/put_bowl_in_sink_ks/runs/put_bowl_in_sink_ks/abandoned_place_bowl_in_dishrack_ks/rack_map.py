"""Height map of the dish rack (world frame, scene 0): for each xy cell, the surfaces a downward ray hits.
usage: python rack_map.py <seed>"""
import os, sys
sys.path.insert(0, os.getcwd())
import numpy as np, trimesh
import gah_env as G

env = G.make_task("place_bowl_in_dishrack_ks")
G.reset(env, int(sys.argv[1]))
rack = G._rigid(G._entity(env.dishrack))
s = rack.get_collision_shapes()[0]
T = (rack.entity.get_pose() * s.get_local_pose()).to_transformation_matrix()
v = np.asarray(s.get_vertices(), float) * np.asarray(s.get_scale(), float)
w = v @ T[:3, :3].T + T[:3, 3]
m = trimesh.Trimesh(w, np.asarray(s.get_triangles()), process=False)
lo, hi = m.bounds
c = env.dishrack_center
print("bounds", np.round(lo, 3), np.round(hi, 3), "center", np.round(c, 3), "tris", len(m.faces))
table = 0.74
xs = np.arange(lo[0] + 0.005, hi[0], 0.015)
ys = np.arange(lo[1] + 0.005, hi[1], 0.015)
print("top height above counter (cm) rows=y (back at top), cols=x;  '.' = no hit")
print("      " + " ".join(f"{x - c[0]:+.2f}"[1:] if False else f"{int(round((x - c[0]) * 100)):+3d}" for x in xs))
for y in ys[::-1]:
    row = []
    for x in xs:
        locs, _, _ = m.ray.intersects_location([[x, y, hi[2] + 0.1]], [[0, 0, -1]])
        row.append(f"{int(round((locs[:, 2].max() - table) * 100)):3d}" if len(locs) else "  .")
    print(f"{int(round((y - c[1]) * 100)):+4d}  " + " ".join(row))
# all hit layers along the centre lines
for x in (c[0],):
    print("layers along x=centre:")
    for y in ys[::-1]:
        locs, _, _ = m.ray.intersects_location([[x, y, hi[2] + 0.1]], [[0, 0, -1]])
        print(f"  dy {y - c[1]:+.3f}: " + ",".join(f"{(z - table) * 100:.1f}" for z in sorted(set(np.round(locs[:, 2], 3)), reverse=True)))
