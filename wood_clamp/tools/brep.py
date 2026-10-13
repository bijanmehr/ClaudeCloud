"""Rebuild a printed part as an exact B-rep solid for FEA meshing.

Every part of the clamp is a plan-view outline extruded through a few Z
bands, so the solid is rebuilt band by band from cross-sections of the
exported mesh using gmsh's OpenCASCADE kernel, then fused. Meshing that
clean B-rep is far more robust than meshing the faceted STL directly.
Returns linear tetrahedra (fea.py adds the mid-edge nodes).
"""
import gmsh
import numpy as np
import trimesh
from shapely.geometry import Polygon

import kinematics as K


def _loop(occ, coords):
    pts = [occ.addPoint(x, y, 0) for x, y in coords[:-1]]
    lines = [occ.addLine(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]
    return occ.addCurveLoop(lines)


def solid_tets(stl_mesh, z_breaks, h_max=1.0, h_min=0.3, simplify=0.03, transform=None):
    m = stl_mesh if isinstance(stl_mesh, trimesh.Trimesh) else trimesh.load(stl_mesh)
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    occ = gmsh.model.occ
    vols = []
    for z0, z1 in zip(z_breaks[:-1], z_breaks[1:]):
        sec = K.section(m, (z0 + z1) / 2)
        for poly in getattr(sec, "geoms", [sec]):
            poly = poly.simplify(simplify)
            if poly.is_empty or poly.area < 1e-3:
                continue
            loops = [_loop(occ, np.array(poly.exterior.coords))]
            loops += [_loop(occ, np.array(h.coords)) for h in poly.interiors]
            s = occ.addPlaneSurface(loops)
            occ.translate([(2, s)], 0, 0, z0)
            ext = occ.extrude([(2, s)], 0, 0, z1 - z0)
            vols += [e for e in ext if e[0] == 3]
    occ.synchronize()
    if len(vols) > 1:
        occ.fuse(vols[:1], vols[1:])
        occ.synchronize()
    occ.removeAllDuplicates()
    occ.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", h_max)
    gmsh.option.setNumber("Mesh.MeshSizeMin", h_min)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)
    gmsh.option.setNumber("Mesh.Algorithm3D", 10)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
    gmsh.model.mesh.generate(3)
    ntags, coords, _ = gmsh.model.mesh.getNodes()
    nodes = coords.reshape(-1, 3)
    idx = {t: i for i, t in enumerate(ntags)}
    et, etags, enodes = gmsh.model.mesh.getElements(3)
    tets = np.array([idx[t] for t in enodes[list(et).index(4)]]).reshape(-1, 4)
    nvol = len(gmsh.model.getEntities(3))
    gmsh.finalize()
    # drop collapsed slivers (zero volume) - they carry no material and break the solver
    v = np.einsum("ij,ij->i", np.cross(nodes[tets[:, 1]] - nodes[tets[:, 0]], nodes[tets[:, 2]] - nodes[tets[:, 0]]),
                  nodes[tets[:, 3]] - nodes[tets[:, 0]]) / 6
    tets = tets[np.abs(v) > 1e-6]
    used = np.unique(tets)
    remap = -np.ones(len(nodes), int)
    remap[used] = np.arange(len(used))
    return nodes[used], remap[tets], nvol
