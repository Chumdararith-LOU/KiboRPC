"""Kibo (JPM) geometry for the offline simulator: distance from the station's collision mesh.

The mesh is JAXA's jpm.dae (scale 1.0858), placed in our (iss_body / dock) frame:
ours = (-3.47656 - u_y, 10.935894 - u_x, 5.643621 - u_z), u = (10.9358938, -2.3364698, 4.8505872) + diag(-1, 1, -1) * s * v.
A distance grid (4 cm) is cached in sim/data/; exact distance and surface normal are computed near the walls.
"""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MESH = os.environ.get("JPM_DAE", "/home/claude/ext/int-ball2_simulator/Int-Ball2_platform_simulator/src/platform_sim/"
                      "description/media/meshes/iss/jpm.dae")
CACHE = os.path.join(HERE, "data", "jpm_distance_grid.npz")
LO = np.array([-0.4, -1.3, -0.6])
HI = np.array([7.8, 1.3, 2.2])
STEP = 0.04
ROBOT_R = 0.100          # Int-Ball2 collision mesh: max radius 0.100 m


class Station(object):
    def __init__(self):
        import trimesh
        m = trimesh.load(MESH, force="mesh")
        v = np.asarray(m.vertices) * 1.0858
        ux, uy, uz = 10.9358938 - v[:, 0], -2.3364698 + v[:, 1], 4.8505872 - v[:, 2]
        ours = np.stack([-3.47656 - uy, 10.935894 - ux, 5.643621 - uz], axis=1)
        self.mesh = trimesh.Trimesh(vertices=ours, faces=m.faces, process=False)
        self.query = trimesh.proximity.ProximityQuery(self.mesh)
        if os.path.exists(CACHE):
            self.grid = np.load(CACHE)["d"]
        else:
            axes = [np.arange(LO[k], HI[k] + 1e-9, STEP) for k in range(3)]
            P = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
            d = np.empty(len(P))
            for i in range(0, len(P), 20000):
                _, d[i:i + 20000], _ = self.query.on_surface(P[i:i + 20000])
            self.grid = d.reshape(len(axes[0]), len(axes[1]), len(axes[2])).astype(np.float32)
            np.savez_compressed(CACHE, d=self.grid)

    def coarse(self, p):
        """Distance from the grid (nearest node): within about 3.5 cm of the true value."""
        i = np.clip(np.round((np.asarray(p) - LO) / STEP).astype(int), 0, np.array(self.grid.shape) - 1)
        return float(self.grid[i[0], i[1], i[2]])

    def exact(self, p):
        """(distance, unit normal pointing from the surface to p)."""
        c, d, _ = self.query.on_surface(np.asarray([p]))
        n = np.asarray(p) - c[0]
        nn = np.linalg.norm(n)
        return float(d[0]), (n / nn if nn > 1e-9 else np.array([0.0, 0.0, 1.0]))

    def clearance(self, p):
        """Distance from the robot's surface to the station (negative = overlap), and the normal."""
        if self.coarse(p) > ROBOT_R + 0.12:
            return self.coarse(p) - ROBOT_R, None
        d, n = self.exact(p)
        return d - ROBOT_R, n
