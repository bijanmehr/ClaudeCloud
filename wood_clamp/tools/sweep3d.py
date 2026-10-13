"""Full 3D collision sweep: the real meshes posed at many lever angles.

At each angle the collar closes exactly as far as the cam pushes it (found
on a cam-layer section by tools/kinematics.py). Reports the overlap volume
between the lever and every fixed part, and between ear A and side B.
"""
import math
import sys

import numpy as np
import trimesh

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import kinematics as K  # noqa: E402


def mat2d(rot_deg, tx=0.0, ty=0.0):
    t = math.radians(rot_deg)
    M = np.eye(4)
    M[:2, :2] = [[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]]
    M[0, 3], M[1, 3] = tx, ty
    return M


def poses(c, f):
    a, dr = c.a_f(f)
    A = mat2d(a) @ mat2d(0, dr) @ mat2d(-c.g["a0"])
    B = mat2d(-a) @ mat2d(0, dr) @ mat2d(c.g["a0"])
    return A, B


def main(chk, n=12):
    c = K.Clamp(chk)
    S = c.sections(10.8)
    m = {k: trimesh.load(f"{chk}/{k}.stl") for k in ("lever_closed", "earA_closed", "bgroup_closed", "ring_closed", "ring_open")}
    P = c.g["P"]
    worst = 0.0
    for th in np.linspace(-c.g["open_angle"], 0, n):
        f = c.contact_f(S, th)
        A, B = poses(c, f)
        lev = m["lever_closed"].copy().apply_transform(B @ mat2d(0, P[0], P[1]) @ mat2d(th) @ mat2d(0, -P[0], -P[1]))
        ea = m["earA_closed"].copy().apply_transform(A)
        bg = m["bgroup_closed"].copy().apply_transform(B)
        ring = m["ring_open" if f > 0.5 else "ring_closed"]
        vols = {"lever-earA": lev.intersection(ea).volume, "lever-sideB": lev.intersection(bg).volume,
                "lever-collar": lev.intersection(ring).volume, "earA-sideB": ea.intersection(bg).volume}
        worst = max(worst, *vols.values())
        print(f"lever {th + c.g['open_angle']:5.1f} deg  collar open {f:5.3f}  overlaps (mm3): " +
              "  ".join(f"{k} {v:.3f}" for k, v in vols.items()))
    print("RESULT:", "PASS" if worst < 0.05 else f"FAIL (max overlap {worst:.3f} mm3)")


if __name__ == "__main__":
    main(sys.argv[1])
