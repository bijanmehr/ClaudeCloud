"""Linear-elastic finite-element stress check with CalculiX (ccx).

mesh_stl() turns a watertight STL into quadratic tetrahedra (TetGen corners
+ mid-edge nodes in Abaqus C3D10 order); run() writes a ccx deck with
node-set constraints and distributed nodal forces, solves it and returns
nodal stresses. PETG is treated as isotropic for stiffness; strength is
checked separately in-plane (von Mises) and across the printed layers
(sigma_zz tension), because upright FDM prints are weakest between layers.
"""
import os
import re
import subprocess

import numpy as np
import tetgen
import trimesh

E_PETG, NU_PETG = 2000.0, 0.38


def mesh_stl(mesh, max_vol=0.6):
    """Quadratic tets straight from a watertight STL with TetGen (fine for simple shapes)."""
    m = mesh if isinstance(mesh, trimesh.Trimesh) else trimesh.load(mesh)
    tg = tetgen.TetGen(np.asarray(m.vertices, float), np.asarray(m.faces, int))
    nodes, tets = tg.tetrahedralize(switches=f"pq1.6/12a{max_vol}Q")[:2]
    return quadratic(np.asarray(nodes, float), np.asarray(tets, int))


def quadratic(nodes, tets):
    """Orient linear tets for ccx and add mid-edge nodes in Abaqus C3D10 order."""
    tets = tets.copy()
    v = np.einsum("ij,ij->i", np.cross(nodes[tets[:, 1]] - nodes[tets[:, 0]], nodes[tets[:, 2]] - nodes[tets[:, 0]]),
                  nodes[tets[:, 3]] - nodes[tets[:, 0]])
    tets[v < 0] = tets[v < 0][:, [0, 2, 1, 3]]
    edges = np.array([(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)])
    a = tets[:, edges[:, 0]].ravel()
    b = tets[:, edges[:, 1]].ravel()
    key = np.minimum(a, b) * (len(nodes) + 1) + np.maximum(a, b)
    uniq, inv = np.unique(key, return_inverse=True)
    lo, hi = uniq // (len(nodes) + 1), uniq % (len(nodes) + 1)
    mids = (nodes[lo] + nodes[hi]) / 2
    t10 = np.zeros((len(tets), 10), int)
    t10[:, :4] = tets
    t10[:, 4:] = (len(nodes) + inv).reshape(len(tets), 6)
    return np.vstack([nodes, mids]), t10


def _fmt_nodes(ids):
    ids = [int(i) + 1 for i in ids]
    return "\n".join(", ".join(str(x) for x in ids[k:k + 12]) for k in range(0, len(ids), 12))


def run(name, nodes, t10, fixed, loads, workdir, fixed_dofs=(1, 2, 3), solver=None):
    """fixed: array of node indices; loads: list of (node_indices, force_vector_total)."""
    os.makedirs(workdir, exist_ok=True)
    inp = [f"*HEADING\n{name}", "*NODE, NSET=NALL"]
    inp += [f"{i + 1}, {x:.6f}, {y:.6f}, {z:.6f}" for i, (x, y, z) in enumerate(nodes)]
    inp.append("*ELEMENT, TYPE=C3D10, ELSET=EALL")
    inp += [f"{i + 1}, " + ", ".join(str(n + 1) for n in t) for i, t in enumerate(t10)]
    inp += ["*NSET, NSET=NFIX", _fmt_nodes(fixed)]
    inp += ["*MATERIAL, NAME=PETG", "*ELASTIC", f"{E_PETG}, {NU_PETG}",
            "*SOLID SECTION, ELSET=EALL, MATERIAL=PETG", "*STEP",
            "*STATIC" + (f", SOLVER={solver}" if solver else ""),
            "*BOUNDARY"] + [f"NFIX, {d}, {d}" for d in fixed_dofs]
    inp.append("*CLOAD")
    for ids, F in loads:
        F = np.asarray(F, float)
        per_node = F if F.ndim == 2 else np.tile(F / len(ids), (len(ids), 1))
        for i, f in zip(ids, per_node):
            for d in range(3):
                if abs(f[d]) > 0:
                    inp.append(f"{int(i) + 1}, {d + 1}, {f[d]:.6e}")
    inp += ["*NODE FILE", "U, RF", "*EL FILE", "S", "*END STEP"]
    path = os.path.join(workdir, name)
    open(path + ".inp", "w").write("\n".join(inp) + "\n")
    r = subprocess.run(["ccx", "-i", name], cwd=workdir, capture_output=True, text=True)
    if r.returncode != 0 or "ERROR" in r.stdout:
        raise RuntimeError(r.stdout[-3000:])
    return read_frd(path + ".frd", len(nodes))


def read_frd(path, n):
    U = np.zeros((n, 3))
    S = np.zeros((n, 6))
    RF = np.zeros((n, 3))
    block = None
    for line in open(path):
        if line.startswith(" -4"):
            block = line.split()[1]
            continue
        if line.startswith(" -3"):
            block = None
            continue
        if block and line.startswith(" -1"):
            nid = int(line[3:13]) - 1
            vals = [float(line[13 + 12 * k:25 + 12 * k]) for k in range((len(line.rstrip()) - 13) // 12)]
            if block == "DISP":
                U[nid] = vals[:3]
            elif block == "STRESS":
                S[nid] = vals[:6]       # xx yy zz xy yz zx
            elif block == "FORC":
                RF[nid] = vals[:3]
    return U, S, RF


def von_mises(S):
    xx, yy, zz, xy, yz, zx = S.T
    return np.sqrt(0.5 * ((xx - yy) ** 2 + (yy - zz) ** 2 + (zz - xx) ** 2) + 3 * (xy ** 2 + yz ** 2 + zx ** 2))


def summarize(nodes, U, S, exclude=None, pct=99.5):
    vm = von_mises(S)
    szz = S[:, 2]
    keep = np.ones(len(nodes), bool) if exclude is None else ~exclude
    i_vm = np.argmax(np.where(keep, vm, -1))
    i_zz = np.argmax(np.where(keep, szz, -1e9))
    return dict(vm_max=float(vm[i_vm]), vm_at=nodes[i_vm].round(1).tolist(), vm_p=float(np.percentile(vm[keep], pct)),
                szz_max=float(szz[i_zz]), szz_at=nodes[i_zz].round(1).tolist(), szz_p=float(np.percentile(szz[keep], pct)),
                disp_max=float(np.linalg.norm(U, axis=1).max()))
