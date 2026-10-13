"""3D FEA of the closed collar on the wood: TPU fins as radial springs.

The PETG collar (ring + ear A + arm + hinge knuckles + pin, closed state)
sits on radial springs at the bore (the fins against the wood). The cam
pushes ear A and the lever pushes back on the pin with equal and opposite
forces along the cam line. Linear, so one solve gives the compliance; the
force is then scaled to the cam stroke actually available, and the result
is the squeeze all around the wood plus the stresses.
"""
import json
import math
import os
import subprocess
import sys

import numpy as np
import trimesh

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import brep  # noqa: E402
import fea  # noqa: E402
import kinematics as K  # noqa: E402

Z_BREAKS = [0, 5.2, 5.8, 15.6, 16.2, 21.4, 22.0, 31.8, 32.4, 37.6, 38.2, 48.0, 48.6, 53.8, 54.4, 64.2, 64.8, 70]


def run(chk, kp=0.73, h_max=1.6, stroke_eff=None, label="typical"):
    c = K.Clamp(chk)
    g = c.g
    forces = json.load(open(f"{chk}/forces.json"))
    row = forces[list(forces)[1]]["rows"][-1]          # closed position geometry (contact point, normal, pivot)
    P = np.array(g["P"]); cp = np.array(row["contact"]); nrm = np.array(row["normal"])
    m = trimesh.load(f"{chk}/fea_collar_closed.stl")
    nodes, tets, _ = brep.solid_tets(m, Z_BREAKS, h_max=h_max, h_min=0.4)
    nodes, t10 = fea.quadratic(nodes, tets)
    bore_r = g["shell_r"] - g.get("shell_wall", 4.0)
    rr = np.linalg.norm(nodes[:, :2], axis=1)
    ang = np.degrees(np.arctan2(nodes[:, 1], nodes[:, 0]))
    bore = np.where((np.abs(rr - bore_r) < 0.05) & (np.abs(ang) > 12) & (nodes[:, 2] > 2.5) & (nodes[:, 2] < 67.5))[0]
    wood_r = g["tip_r"] + 0.45
    k_area = kp * wood_r / bore_r                                     # N/mm^3 at the bore
    area = 2 * math.pi * bore_r * (1 - 24 / 360) * 65.0
    k_node = k_area * area / len(bore)
    # loads: unit force pair along the cam line
    F = 100.0
    bl = [(5.8, 15.6), (22.0, 31.8), (38.2, 48.0), (54.4, 64.2)]
    inb = np.zeros(len(nodes), bool)
    for a, b in bl:
        inb |= (nodes[:, 2] > a + 0.3) & (nodes[:, 2] < b - 0.3)
    ear = np.where((np.linalg.norm(nodes[:, :2] - cp, axis=1) < 1.5) & inb)[0]
    rp = np.linalg.norm(nodes[:, :2] - P, axis=1)
    d = (nodes[:, :2] - P) / (rp[:, None] + 1e-9)
    pin = np.where((np.abs(rp - 3.0) < 0.1) & inb & ((d @ -nrm) > math.cos(math.radians(70))))[0]
    # write deck with springs to fixed ground nodes placed radially inside the bore
    work = f"{chk}/fea"
    os.makedirs(work, exist_ok=True)
    name = f"collar_springs_{label}"
    n0 = len(nodes)
    gnd = nodes[bore].copy()
    gnd[:, :2] *= (bore_r - 1.0) / bore_r
    lines = [f"*HEADING\n{name}", "*NODE, NSET=NALL"]
    lines += [f"{i + 1}, {x:.6f}, {y:.6f}, {z:.6f}" for i, (x, y, z) in enumerate(np.vstack([nodes, gnd]))]
    lines.append("*ELEMENT, TYPE=C3D10, ELSET=EALL")
    lines += [f"{i + 1}, " + ", ".join(str(n + 1) for n in t) for i, t in enumerate(t10)]
    e0 = len(t10)
    lines.append("*ELEMENT, TYPE=SPRINGA, ELSET=ESPR")
    lines += [f"{e0 + j + 1}, {b + 1}, {n0 + j + 1}" for j, b in enumerate(bore)]
    lines += ["*NSET, NSET=NGND"] + [", ".join(str(n0 + j + 1) for j in range(k, min(k + 12, len(bore)))) for k in range(0, len(bore), 12)]
    # weak anchor against rigid rotation about Z at the back of the ring
    back = np.where((nodes[:, 0] < -(bore_r + 2.5)) & (np.abs(nodes[:, 1]) < 1.0) & (np.abs(nodes[:, 2] - 35) < 1.0))[0]
    lines += ["*NSET, NSET=NBACK", ", ".join(str(i + 1) for i in back[:6])]
    lines += ["*MATERIAL, NAME=PETG", "*ELASTIC", f"{fea.E_PETG}, {fea.NU_PETG}",
              "*SOLID SECTION, ELSET=EALL, MATERIAL=PETG",
              "*SPRING, ELSET=ESPR", "", f"{k_node:.6f}",
              "*STEP", "*STATIC", "*BOUNDARY", "NGND, 1, 3", "NBACK, 2, 2", "NBACK, 3, 3", "*CLOAD"]
    for ids, vec in ((ear, -nrm * F), (pin, nrm * F)):
        for i in ids:
            fx, fy = vec / len(ids)
            lines.append(f"{i + 1}, 1, {fx:.6e}")
            lines.append(f"{i + 1}, 2, {fy:.6e}")
    lines += ["*NODE FILE", "U", "*EL FILE", "S", "*END STEP"]
    open(f"{work}/{name}.inp", "w").write("\n".join(lines) + "\n")
    r = subprocess.run(["ccx", "-i", name], cwd=work, capture_output=True, text=True)
    if r.returncode != 0 or "ERROR" in r.stdout:
        raise RuntimeError(r.stdout[-2000:])
    U, S, _ = fea.read_frd(f"{work}/{name}.frd", n0 + len(bore))
    U, S = U[:n0], S[:n0]
    closure = float(((U[pin, :2].mean(0) - U[ear, :2].mean(0)) @ nrm))     # pin moves +nrm, ear moves -nrm
    out = dict(label=label, kp=kp, nodes=n0, elements=len(t10), bore_nodes=len(bore), compliance_mm_per_100N=closure)
    if stroke_eff:
        scale = stroke_eff / closure
        U, S = U * scale, S * scale
        sq = -(U[bore, 0] * nodes[bore, 0] + U[bore, 1] * nodes[bore, 1]) / rr[bore]
        vm = fea.von_mises(S)
        out.update(cam_force=F * scale, squeeze_min=float(sq.min()), squeeze_mean=float(sq.mean()),
                   squeeze_max=float(sq.max()), pressure_mean=float(kp * sq.mean()),
                   vm_p995=float(np.percentile(vm, 99.5)), vm_max=float(vm.max()), vm_at=nodes[np.argmax(vm)].round(1).tolist(),
                   szz_max=float(S[:, 2].max()), szz_at=nodes[np.argmax(S[:, 2])].round(1).tolist())
        np.savez_compressed(f"{work}/{name}.npz", nodes=nodes, S=S, U=U, bore=bore, sq=sq)
    return out


if __name__ == "__main__":
    chk = sys.argv[1]
    stroke = float(sys.argv[2])
    res = [run(chk, kp, stroke_eff=stroke, label=lab) for lab, kp in (("typical", 0.73), ("stiff", 1.12))]
    for r in res:
        print(json.dumps(r))
    json.dump(res, open(f"{chk}/fea/collar_springs.json", "w"), indent=1)
