"""Fast 2D model of the C-collar closing on the TPU fins.

The PETG ring is a chain of frame elements along its neutral axis; the TPU
fins are radial springs on the wood side (compression only). The cam
pushes ear A towards the arm along a line `e` outside the neutral axis,
with rigid offsets to the two ring ends. The cam travel is prescribed
(displacement control); output: squeeze around the wood and cam force.
"""
import math

import numpy as np


def frame_k(x1, y1, x2, y2, EA, EI):
    L = math.hypot(x2 - x1, y2 - y1)
    c, s = (x2 - x1) / L, (y2 - y1) / L
    k = np.array([[EA / L, 0, 0, -EA / L, 0, 0],
                  [0, 12 * EI / L**3, 6 * EI / L**2, 0, -12 * EI / L**3, 6 * EI / L**2],
                  [0, 6 * EI / L**2, 4 * EI / L, 0, -6 * EI / L**2, 2 * EI / L],
                  [-EA / L, 0, 0, EA / L, 0, 0],
                  [0, -12 * EI / L**3, -6 * EI / L**2, 0, 12 * EI / L**3, -6 * EI / L**2],
                  [0, 6 * EI / L**2, 2 * EI / L, 0, -6 * EI / L**2, 4 * EI / L]])
    T = np.zeros((6, 6))
    for b in (0, 3):
        T[b:b + 2, b:b + 2] = [[c, s], [-s, c]]
        T[b + 2, b + 2] = 1
    return T.T @ k @ T


def solve(Rn, EA, EI, kp_line, a_end_deg, e, travel, n=360, fin_from_deg=8.0):
    """kp_line: fin stiffness per mm of arc per mm of radial squeeze (N/mm^2), at the neutral radius.
    e: offset of the cam force line outside the neutral axis (mm). travel: cam stroke (mm)."""
    a0, a1 = math.radians(a_end_deg), math.radians(360 - a_end_deg)
    ang = np.linspace(a0, a1, n + 1)
    X = np.c_[Rn * np.cos(ang), Rn * np.sin(ang)]
    N = len(X)
    K = np.zeros((3 * N, 3 * N))
    for i in range(n):
        ke = frame_k(*X[i], *X[i + 1], EA, EI)
        idx = [3 * i, 3 * i + 1, 3 * i + 2, 3 * i + 3, 3 * i + 4, 3 * i + 5]
        K[np.ix_(idx, idx)] += ke
    ds = Rn * (a1 - a0) / n
    fin = (np.degrees(ang) > a_end_deg + fin_from_deg) & (np.degrees(ang) < 360 - a_end_deg - fin_from_deg)
    active = fin.copy()
    # cam: equal/opposite forces F along the slit tangent (y) at points e outside each end
    def load(F):
        f = np.zeros(3 * N)
        for node, sgn in ((0, -1), (N - 1, +1)):    # end A (y>0) pushed towards -y, end B towards +y
            th = ang[node]
            tdir = np.array([-math.sin(th), math.cos(th)]) * (1 if node == 0 else -1)   # towards the slit
            Fv = -F * tdir                                                                # closing
            f[3 * node:3 * node + 2] += Fv
            arm = np.array([math.cos(th), math.sin(th)]) * e                              # point of action offset
            f[3 * node + 2] += arm[0] * Fv[1] - arm[1] * Fv[0]
        return f
    for _ in range(20):
        Kt = K.copy()
        for i in np.where(active)[0]:
            r = X[i] / Rn
            k = kp_line * ds
            Kt[3 * i:3 * i + 2, 3 * i:3 * i + 2] += k * np.outer(r, r)
        # remove rigid body: fix tangential + rotation at the back node (symmetric problem)
        ib = N // 2
        fixed = [3 * ib + 1, 3 * ib + 2] if abs(X[ib][1]) < 1e-9 else [3 * ib + 1]
        free = np.setdiff1d(np.arange(3 * N), fixed)
        u1 = np.zeros(3 * N)
        u1[free] = np.linalg.solve(Kt[np.ix_(free, free)], load(1.0)[free])
        # closure along the force line per unit F
        def pt_disp(node):
            th = ang[node]; u = u1[3 * node:3 * node + 2]; rot = u1[3 * node + 2]
            arm = np.array([math.cos(th), math.sin(th)]) * e
            return u + rot * np.array([-arm[1], arm[0]])
        dA, dB = pt_disp(0), pt_disp(N - 1)
        closure_per_F = (dB - dA)[1]          # end B moves +y, end A moves -y -> closure
        F = travel / closure_per_F
        u = u1 * F
        radial = -(u[0::3] * X[:, 0] + u[1::3] * X[:, 1]) / Rn      # inward = squeeze
        new_active = fin & (radial > 0)
        if (new_active == active).all():
            break
        active = new_active
    sq = np.where(fin, np.maximum(radial, 0), np.nan)
    return dict(F=F, squeeze=sq, angle=np.degrees(ang), mean=float(np.nanmean(sq)), min=float(np.nanmin(sq)),
                max=float(np.nanmax(sq)), back=float(sq[N // 2]))


def ring_section(wall, height=70.0, flange_depth=0.0, flange_h=0.0, E=2000.0):
    """EA, EI (in-plane bending about Z) and the outward shift of the neutral axis for a wall
    plus optional outward flanges at top and bottom (flange_depth beyond the wall)."""
    parts = [(wall * height, wall / 2, wall, height)]
    if flange_depth > 0:
        parts += [(flange_depth * flange_h, wall + flange_depth / 2, flange_depth, flange_h)] * 2
    A = sum(p[0] for p in parts)
    c = sum(p[0] * p[1] for p in parts) / A
    I = sum(p[3] * p[2] ** 3 / 12 + p[0] * (p[1] - c) ** 2 for p in parts)
    return E * A, E * I, c
