"""Kinematic and clearance checks for the print-in-place cam lever.

Works on plan-view cross-sections of the exported pieces (all features are
vertical extrusions in bands), composed into any state in Python:
  f     = how open the collar is (1 = as printed, 0 = closed on the wood)
  theta = lever angle about the pivot (0 = closed, -open_angle = as printed)
"""
import json
import math
import sys

import numpy as np
import trimesh
from shapely import affinity
from shapely.geometry import Polygon
from shapely.ops import unary_union


def section(mesh, z):
    sec = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    planar, _ = sec.to_2D(to_2D=np.eye(4))
    return unary_union([Polygon(p.exterior.coords, [h.coords for h in p.interiors])
                        for p in planar.polygons_full])


class Clamp:
    def __init__(self, chk_dir):
        self.g = json.load(open(f"{chk_dir}/geom.json"))
        self.m = {n: trimesh.load(f"{chk_dir}/{n}.stl") for n in
                  ("earA_closed", "bgroup_closed", "lever_closed", "ring_closed", "ring_open")}
        self.rn = self.g["shell_r"] - 2.0

    def a_f(self, f):
        dr = f * self.g["open_dr"]
        return 180 - (180 - self.g["a0"]) * self.rn / (self.rn + dr), dr

    def pose_A(self, geom, f):
        a, dr = self.a_f(f)
        g = affinity.rotate(geom, -self.g["a0"], origin=(0, 0))
        g = affinity.translate(g, dr, 0)
        return affinity.rotate(g, a, origin=(0, 0))

    def pose_B(self, geom, f):
        a, dr = self.a_f(f)
        g = affinity.rotate(geom, self.g["a0"], origin=(0, 0))
        g = affinity.translate(g, dr, 0)
        return affinity.rotate(g, -a, origin=(0, 0))

    def lever(self, sec, theta, f):
        P = self.g["P"]
        return self.pose_B(affinity.rotate(sec, theta, origin=(P[0], P[1])), f)

    def sections(self, z):
        return {n: section(m, z) for n, m in self.m.items()}

    def contact_f(self, S, theta, tol=1e-3):
        """Most-open collar state at which the cam does not overlap ear A."""
        lev = self.lever(S["lever_closed"], theta, 1.0)
        if lev.intersection(self.pose_A(S["earA_closed"], 1.0)).area < tol:
            return 1.0
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = (lo + hi) / 2
            lev = self.lever(S["lever_closed"], theta, mid)
            if lev.intersection(self.pose_A(S["earA_closed"], mid)).area < tol:
                lo = mid
            else:
                hi = mid
        return lo


def main(chk_dir, cam_z, knuckle_z):
    c = Clamp(chk_dir)
    g = c.g
    ok = True
    for label, z in (("cam layer", cam_z), ("knuckle layer", knuckle_z)):
        S = c.sections(z)
        print(f"== {label} (z = {z}) ==")
        # as printed: everything separate, report the smallest gap
        lev = c.lever(S["lever_closed"], -g["open_angle"], 1.0)
        fixed = {"ear A": c.pose_A(S["earA_closed"], 1.0), "arm/knuckle/pin": c.pose_B(S["bgroup_closed"], 1.0),
                 "collar": S["ring_open"]}
        for n, geo in fixed.items():
            if geo.is_empty:
                continue
            ov = lev.intersection(geo).area
            gap = lev.distance(geo)
            print(f"  printed: lever vs {n:16s} overlap {ov:.4f} mm2, min gap {gap:.3f} mm")
            ok &= ov < 1e-3
        ea, bg = c.pose_A(S["earA_closed"], 1.0), c.pose_B(S["bgroup_closed"], 1.0)
        print(f"  printed: ear A vs arm/knuckle    min gap {ea.distance(bg):.3f} mm")
        ok &= ea.distance(bg) > 0.3
        # sweep the lever closed; the collar closes only as far as the cam pushes it
        rows = []
        for th in np.linspace(-g["open_angle"], 0, 28):
            f = c.contact_f(S, th) if label == "cam layer" else c.f_of[round(th, 3)]
            lev = c.lever(S["lever_closed"], th, f)
            bad = {}
            for n, geo in (("arm/knuckle/pin", c.pose_B(S["bgroup_closed"], f)),
                           ("collar", S["ring_open"] if f > 0.5 else S["ring_closed"])):
                a = lev.intersection(geo).area
                if a > 1e-3:
                    bad[n] = a
            if label != "cam layer":
                a = lev.intersection(c.pose_A(S["earA_closed"], 0.0)).area
                if a > 1e-3:
                    bad["ear A (closed pos)"] = a
            ea_bg = c.pose_A(S["earA_closed"], f).intersection(c.pose_B(S["bgroup_closed"], f)).area
            if ea_bg > 1e-3:
                bad["ear A vs arm"] = ea_bg
            rows.append((th, f, bad))
            ok &= not bad
        for th, f, bad in rows[::3] + [rows[-1]]:
            sq = (1 - f) * g["open_dr"]
            print(f"  theta {th:7.1f}  collar open {f:5.3f}  squeeze {sq:5.3f} mm  {'COLLISION ' + str(bad) if bad else 'ok'}")
        if label == "cam layer":
            c.rows = rows
            c.f_of = {round(th, 3): f for th, f, _ in rows}
    # closed: handle stop and cam on ear A
    S = c.sections(cam_z)
    lev = c.lever(S["lever_closed"], 0, 0.0)
    print(f"closed: lever vs ear A overlap {lev.intersection(S['earA_closed']).area:.4f} mm2 "
          f"gap {lev.distance(S['earA_closed']):.3f}; lever vs collar gap {lev.distance(S['ring_closed']):.3f}")
    # dead-centre check: squeeze must peak just before the closed position
    sq = [(1 - f) * g["open_dr"] for _, f, _ in c.rows]
    print(f"squeeze at closed {sq[-1]:.3f} mm, max during travel {max(sq):.3f} mm "
          f"(over-centre drop {max(sq) - sq[-1]:.4f} mm)")
    print("RESULT:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    d = sys.argv[1]
    sys.exit(0 if main(d, float(sys.argv[2]), float(sys.argv[3])) else 1)
