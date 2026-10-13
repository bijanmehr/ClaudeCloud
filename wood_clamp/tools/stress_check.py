"""Worst-case FEA of the lever and the collar using loads from tools/forces.py.

Load cases (worst case = stiff TPU + wood 0.4 mm oversize unless noted):
  lever_closed  : lever locked, maximum cam force, pin holds it
  lever_peak    : mid-stroke at the peak hand force (cam force + hand push)
  collar_closed : collar with the cam force on ear A, the pin load and the
                  TPU pressure on the bore
Strength limits (printed PETG): 45 MPa in-plane, 20 MPa across layers.
"""
import json
import math
import sys

import numpy as np
import trimesh

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import brep  # noqa: E402
import fea  # noqa: E402

Z_BREAKS = [0, 5.2, 5.8, 15.6, 16.2, 21.4, 22.0, 31.8, 32.4, 37.6, 38.2, 48.0, 48.6, 53.8, 54.4, 64.2, 64.8, 70]


def mesh_part(m, h_max):
    nodes, tets, _ = brep.solid_tets(m, Z_BREAKS, h_max=h_max, h_min=0.35)
    return fea.quadratic(nodes, tets)

MU = 0.30
HANDLE_END = 160.0   # handle tip angle in clamp.scad
IN_PLANE, ACROSS = 45.0, 20.0


def bands(g):
    kh, cb, zg = 5.2, g["cam_band_h"], 0.6
    return [(i * (kh + cb + 2 * zg) + kh + zg, i * (kh + cb + 2 * zg) + kh + zg + cb) for i in range(4)]


def in_bands(z, bl, pad=0.05):
    m = np.zeros(len(z), bool)
    for a, b in bl:
        m |= (z > a - pad) & (z < b + pad)
    return m


def pose_matrix(c, theta, f):
    """4x4 for a lever point (closed B frame) -> global, matching kinematics.lever()."""
    P = c.g["P"]
    a, dr = c.a_f(f)
    def R(deg):
        t = math.radians(deg); M = np.eye(4); M[:2, :2] = [[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]]; return M
    def T(x, y):
        M = np.eye(4); M[0, 3] = x; M[1, 3] = y; return M
    return R(-a) @ T(dr, 0) @ R(c.g["a0"]) @ T(P[0], P[1]) @ R(theta) @ T(-P[0], -P[1])


def lever_case(chk, c, row, hand_T, name, max_vol, handle_len=48.0):
    g = c.g
    m = trimesh.load(f"{chk}/fea_lever_closed.stl")
    m.apply_transform(pose_matrix(c, row["theta"], row["f"]))
    nodes, t10 = mesh_part(m, max_vol)
    P = np.array(row["pivot"]); cp = np.array(row["contact"]); nrm = np.array(row["normal"])
    bl = bands(g)
    rxy = np.linalg.norm(nodes[:, :2] - P, axis=1)
    hole = (np.abs(rxy - g.get("pin_hole_r", 3.45)) < 0.12) & in_bands(nodes[:, 2], bl)
    # friction on the lever opposes its sliding (lever turns +theta = CCW)
    r = cp - P; v = np.array([-r[1], r[0]]); v -= nrm * (v @ nrm); t_hat = v / (np.linalg.norm(v) + 1e-12)
    Fcon = row["Fc"] * nrm - MU * row["Fc"] * t_hat
    loads = []
    cont = (np.linalg.norm(nodes[:, :2] - cp, axis=1) < 1.2) & in_bands(nodes[:, 2], bl, pad=-0.3)
    loads.append((np.where(cont)[0], np.r_[Fcon, 0]))
    Fh = np.zeros(2)
    if hand_T > 0:
        tip_local = np.array([math.cos(math.radians(HANDLE_END - 4)), math.sin(math.radians(HANDLE_END - 4))]) * (g["shell_r"] + 1.2 + 2.5)
        tip = (pose_matrix(c, row["theta"], row["f"]) @ np.r_[tip_local, 0, 1])[:2]
        rt = tip - P; Fh = hand_T / np.linalg.norm(rt) * np.array([-rt[1], rt[0]]) / np.linalg.norm(rt)
        hz = (nodes[:, 2] > 25) & (nodes[:, 2] < 45)
        hn = (np.linalg.norm(nodes[:, :2] - tip, axis=1) < 4.0) & hz
        loads.append((np.where(hn)[0], np.r_[Fh, 0]))
    Rpin = -(Fcon + Fh)
    d = (nodes[:, :2] - P) / (rxy[:, None] + 1e-9)
    bearing = hole & ((d @ (Rpin / np.linalg.norm(Rpin))) > math.cos(math.radians(70)))
    U, S, RF = fea.run(name, nodes, t10, np.where(bearing)[0], loads, f"{chk}/fea", fixed_dofs=(1, 2, 3),
                       solver="ITERATIVE CHOLESKY")
    excl = np.zeros(len(nodes), bool)
    for ids, _ in loads:
        for i in ids:
            excl |= np.linalg.norm(nodes - nodes[i], axis=1) < 0.8      # skip point-load singularities
    out = fea.summarize(nodes, U, S, exclude=excl)
    out.update(elements=len(t10), cam_force=row["Fc"], hand_force=float(np.linalg.norm(Fh)),
               pin_force=float(np.linalg.norm(Rpin)), reaction=float(np.linalg.norm(RF[bearing].sum(0))))
    np.savez_compressed(f"{chk}/fea/{name}.npz", nodes=nodes, S=S, U=U)
    return out


def collar_case(chk, c, row, p_wood, name, max_vol):
    g = c.g
    m = trimesh.load(f"{chk}/fea_collar_closed.stl")
    nodes, t10 = mesh_part(m, max_vol)
    P = np.array(row["pivot"]); cp = np.array(row["contact"]); nrm = np.array(row["normal"])
    bl = bands(g)
    r = cp - P; v = np.array([-r[1], r[0]]); v -= nrm * (v @ nrm); t_hat = v / (np.linalg.norm(v) + 1e-12)
    Fcon = row["Fc"] * nrm - MU * row["Fc"] * t_hat          # force ON the lever
    loads = []
    cont = (np.linalg.norm(nodes[:, :2] - cp, axis=1) < 1.2) & in_bands(nodes[:, 2], bl, pad=-0.3)
    loads.append((np.where(cont)[0], np.r_[-Fcon, 0]))         # lever pushes ear A
    rp = np.linalg.norm(nodes[:, :2] - P, axis=1)
    R = -Fcon                                                 # pin force on the lever
    d = (nodes[:, :2] - P) / (rp[:, None] + 1e-9)
    pin = (np.abs(rp - (g.get("pin_hole_r", 3.45) - 0.45)) < 0.12) & in_bands(nodes[:, 2], bl) & ((d @ (R / np.linalg.norm(R))) > math.cos(math.radians(70)))
    loads.append((np.where(pin)[0], np.r_[-R, 0]))            # lever pushes the pin
    bore_r = g["shell_r"] - 4.0
    rr = np.linalg.norm(nodes[:, :2], axis=1)
    bore = (np.abs(rr - bore_r) < 0.12) & (nodes[:, 2] > 2.5) & (nodes[:, 2] < 67.5)
    p_bore = p_wood * (g["tip_r"] + 0.45) / bore_r
    A = 2 * math.pi * bore_r * 65.0 * (1 - 3 / 360)
    ids = np.where(bore)[0]
    radial = np.c_[nodes[ids, :2] / rr[ids, None], np.zeros(len(ids))]
    loads.append((ids, radial * p_bore * A / len(ids)))
    back = (nodes[:, 0] < -(g["shell_r"] - 0.3)) & (np.abs(nodes[:, 1]) < 3) & (np.abs(nodes[:, 2] - 35) < 3)
    U, S, RF = fea.run(name, nodes, t10, np.where(back)[0], loads, f"{chk}/fea", solver="ITERATIVE CHOLESKY")
    excl = np.zeros(len(nodes), bool)
    for idx in (np.where(cont)[0], np.where(pin)[0], np.where(back)[0]):
        for i in idx:
            excl |= np.linalg.norm(nodes - nodes[i], axis=1) < 0.8
    out = fea.summarize(nodes, U, S, exclude=excl)
    out.update(elements=len(t10), cam_force=row["Fc"], p_bore=p_bore,
               back_reaction=float(np.linalg.norm(RF[back].sum(0))))
    np.savez_compressed(f"{chk}/fea/{name}.npz", nodes=nodes, S=S, U=U)
    return out


if __name__ == "__main__":
    chk = sys.argv[1]
    max_vol = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2   # max element size, mm
    import kinematics as K
    c = K.Clamp(chk)
    forces = json.load(open(f"{chk}/forces.json"))
    worst = forces["WORST: stiff TPU, wood +0.4 mm"]
    rows = worst["rows"]
    closed = rows[-1]
    hand_T = [max(0.0, -r["T_normal"] + r["T_fric"]) if r["Fc"] > 0 else 0.0 for r in rows]
    ip = int(np.argmax(hand_T))
    res = {}
    res["lever_closed"] = lever_case(chk, c, closed, 0.0, "lever_closed", max_vol)
    res["lever_peak"] = lever_case(chk, c, rows[ip], hand_T[ip], "lever_peak", max_vol)
    res["collar_closed"] = collar_case(chk, c, closed, worst["pressure_closed"], "collar_closed", max_vol * 1.25)
    for k, v in res.items():
        sf_vm = IN_PLANE / v["vm_max"]; sf_zz = ACROSS / max(v["szz_max"], 1e-6)
        v.update(sf_in_plane=sf_vm, sf_across_layers=sf_zz)
        print(f"{k:14s} elems {v['elements']:6d} | von Mises max {v['vm_max']:5.1f} MPa at {v['vm_at']} (99.5% {v['vm_p']:.1f}) "
              f"| across-layer tension max {v['szz_max']:5.1f} MPa at {v['szz_at']} | defl {v['disp_max']:.3f} mm "
              f"| safety in-plane {sf_vm:.1f}, across layers {sf_zz:.1f}")
    json.dump(res, open(f"{chk}/fea/results.json", "w"), indent=1)
