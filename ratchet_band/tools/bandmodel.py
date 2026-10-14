"""2D model of the ratchet band closing on the wood.

The PETG band is a chain of frame elements along its neutral circle, from
the jaw B end (phi = +gap/2) round the top to the jaw A end (phi = -gap/2).
The two jaws are (nearly) rigid; the upper band between them is the thin
PETG band. The TPU fins are radial springs to the wood that only push, and
only after the as-printed clearance at the fin tips is taken up.

Two load cases:
  clicks:  the ratchet is engaged k teeth: jaw B is turned k tooth pitches
           towards jaw A about the band centre, relative to jaw A (rack
           seated on the teeth). Output: squeeze and pressure all round,
           band tension, and the force and moment the ratchet must hold.
  pinch:   no ratchet; equal and opposite forces on the two ears close the
           band. Output: the finger force needed to reach a given click.

Fin stiffness k_p (MPa per mm of squeeze) is scaled from the cantilever-fin
model used for the first clamp (0.42 / 0.73 / 1.12 for TPU 95A with E = 15 /
26 / 40 MPa, fins 1.9 x 1.0 mm at 3.2 mm pitch) by t^3 / (L^3 * pitch).
"""
import json
import math
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
E_PETG = 2000.0


def geom(extra=()):
    r = subprocess.run(["openscad", "-D", 'part="none"', *extra, "-o", "/tmp/_band_geom.csg",
                        os.path.join(ROOT, "band.scad")], capture_output=True, text=True)
    line = [l for l in r.stderr.splitlines() if "GEOM " in l][0]
    return json.loads(line.split("GEOM ", 1)[1].rstrip('"'))


def kp_of(E_tpu, fin_h, fin_t, fin_pitch):
    base = 0.73 * E_tpu / 26.0
    return base * (fin_t / 1.0) ** 3 / (fin_h / 1.9) ** 3 * (3.2 / fin_pitch)


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
    return T.T @ k @ T, L


def P(r, phi_deg):
    a = math.radians(phi_deg)
    return np.array([r * math.sin(a), -r * math.cos(a)])


class Band:
    def __init__(self, g, kp, wood_offset=0.0, n=720):
        self.g = g
        self.Rn = (g["R_i"] + g["R_o"]) / 2
        h = g["band_h"]
        t = g["ring_t"]
        self.EA_u, self.EI_u = E_PETG * t * h, E_PETG * h * t ** 3 / 12
        self.phi = np.linspace(g["g2"], 360 - g["g2"], n + 1)         # B end -> top -> A end
        self.X = np.array([P(self.Rn, p) for p in self.phi])
        self.N = len(self.X)
        jaw = (self.phi < g["jaw_end"]) | (self.phi > 360 - g["jaw_end"])
        K = np.zeros((3 * self.N, 3 * self.N))
        for i in range(n):
            rigid = jaw[i] and jaw[i + 1]
            EA = self.EA_u * (200 if rigid else 1)
            EI = self.EI_u * (2000 if rigid else 1)
            ke, L = frame_k(*self.X[i], *self.X[i + 1], EA, EI)
            idx = list(range(3 * i, 3 * i + 6))
            K[np.ix_(idx, idx)] += ke
        self.K = K
        self.ds = self.Rn * math.radians(self.phi[1] - self.phi[0])
        h_f = h - 1.2                                                   # fins fade at both faces
        self.k_node = kp * h_f * (g["R_tip"] / self.Rn) * self.ds        # N/mm per node
        self.fin = np.abs(((self.phi + 180) % 360) - 180) > g["g2"] + 3.0
        self.clear = g["R_tip"] - (g["wood_r"] + wood_offset)            # gap before a fin touches
        self.kp = kp

    def radial_in(self, u):
        rdir = self.X / self.Rn
        return -(u[0::3] * rdir[:, 0] + u[1::3] * rdir[:, 1])

    def _solve(self, build):
        """build(Kt, f) -> (K_red, f_red, expand). Iterate the fin contact set."""
        active = np.zeros(self.N, bool)
        for _ in range(60):
            Kt = self.K.copy()
            f = np.zeros(3 * self.N)
            rdir = self.X / self.Rn
            for i in np.where(active)[0]:
                kk = self.k_node * np.outer(rdir[i], rdir[i])
                Kt[3 * i:3 * i + 2, 3 * i:3 * i + 2] += kk
                # spring acts only beyond the clearance: preload term
                f[3 * i:3 * i + 2] += -self.k_node * self.clear * rdir[i] * -1 * -1
            u = build(Kt, f)
            sq = self.radial_in(u) - self.clear
            new = self.fin & (sq > 0)
            if (new == active).all():
                return u, sq, active
            active = new
        raise RuntimeError("fin contact did not settle")

    # --- load case 1: ratchet engaged k teeth -------------------------
    def clicks(self, turn_deg):
        """Jaw B turned turn_deg towards jaw A (about the band centre) relative to jaw A."""
        a, b = self.N - 1, 0           # A end node (phi = 360-g2), B end node (phi = g2)
        D = math.radians(turn_deg)
        Rm = np.array([[math.cos(-D), -math.sin(-D)], [math.sin(-D), math.cos(-D)]])
        # turning towards -phi = clockwise in the (x, y) plane = rotation by -D
        xB = self.X[b]
        xBp = Rm @ xB
        dB = xBp - xB
        rel = xBp - self.X[a]
        # u_B = u_A + th_A * k x (xB' - xA) + dB ;  th_B = th_A - D
        nfree = 3 * self.N
        T = np.zeros((nfree, nfree - 3))
        cols = [c for c in range(nfree) if c not in (3 * b, 3 * b + 1, 3 * b + 2)]
        for j, c in enumerate(cols):
            T[c, j] = 1.0
        ja = {c: j for j, c in enumerate(cols)}
        T[3 * b, ja[3 * a]] = 1.0
        T[3 * b, ja[3 * a + 2]] = -rel[1]
        T[3 * b + 1, ja[3 * a + 1]] = 1.0
        T[3 * b + 1, ja[3 * a + 2]] = rel[0]
        T[3 * b + 2, ja[3 * a + 2]] = 1.0
        u0 = np.zeros(nfree)
        u0[3 * b:3 * b + 2] = dB
        u0[3 * b + 2] = -D
        top = int(np.argmin(np.abs(self.phi - 180)))
        fix = [ja[3 * top]]                 # symmetry: no sideways drift of the top

        def build(Kt, f):
            Kr = T.T @ Kt @ T
            fr = T.T @ (f - Kt @ u0)
            free = np.setdiff1d(np.arange(Kr.shape[0]), fix)
            q = np.zeros(Kr.shape[0])
            q[free] = np.linalg.solve(Kr[np.ix_(free, free)], fr[free])
            return T @ q + u0

        u, sq, active = self._solve(build)
        return self._report(u, sq, active, ratchet=True)

    # --- load case 2: pinch the ears ----------------------------------
    def pinch(self, F):
        """Equal and opposite horizontal forces F on the ear tips (no ratchet)."""
        g = self.g
        f_ext = np.zeros(3 * self.N)
        ear_phi = (g["ear_a"][0] + g["ear_a"][1]) / 2 if "ear_a" in g else 52.0
        for s in (1, -1):
            # ear attached to the jaw node nearest its centre; force pushes towards the gap
            p_e = s * ear_phi % 360
            i = int(np.argmin(np.abs(self.phi - p_e)))
            tip = P(g["r_ear"] - 1.0, s * ear_phi)
            Fv = np.array([-s * F, 0.0])
            arm = tip - self.X[i]
            f_ext[3 * i:3 * i + 2] += Fv
            f_ext[3 * i + 2] += arm[0] * Fv[1] - arm[1] * Fv[0]
        top = int(np.argmin(np.abs(self.phi - 180)))
        fix = [3 * top]

        def build(Kt, f):
            free = np.setdiff1d(np.arange(3 * self.N), fix)
            u = np.zeros(3 * self.N)
            # small soft anchor against rigid drift along y (the wood takes it once fins touch)
            Kt = Kt.copy()
            Kt[3 * top + 1, 3 * top + 1] += 1e-3
            u[free] = np.linalg.solve(Kt[np.ix_(free, free)], (f + f_ext)[free])
            return u

        u, sq, active = self._solve(build)
        # relative turn of jaw B vs jaw A about the band centre, measured at the rack radius
        a, b = self.N - 1, 0
        turn = math.degrees(u[3 * a + 2] - u[3 * b + 2])
        return turn, self._report(u, sq, active, ratchet=False)

    def _report(self, u, sq, active, ratchet):
        pres = np.where(self.fin, np.maximum(sq, 0) * self.kp, np.nan)
        sqz = np.where(self.fin, np.maximum(sq, 0), np.nan)
        # band tension at the top (axial force of the element there)
        top = int(np.argmin(np.abs(self.phi - 180)))
        x1, x2 = self.X[top], self.X[top + 1]
        L = np.linalg.norm(x2 - x1)
        e = (x2 - x1) / L
        du = u[3 * top + 3:3 * top + 5] - u[3 * top:3 * top + 2]
        tension_top = self.EA_u * (du @ e) / L
        res = dict(squeeze_mean=float(np.nanmean(sqz)), squeeze_min=float(np.nanmin(sqz)),
                   squeeze_max=float(np.nanmax(sqz)), pressure_mean=float(np.nanmean(pres)),
                   pressure_max=float(np.nanmax(pres)), contact_frac=float(np.mean(active[self.fin])),
                   tension_top=float(tension_top), phi=self.phi.tolist(), squeeze=np.nan_to_num(sqz).tolist())
        # total radial push on the wood -> mean band tension that balances it
        Fr = float(np.nansum(np.maximum(sq, 0)[self.fin]) * self.k_node)
        res["radial_total"] = Fr
        res["tension_equiv"] = Fr / (2 * math.pi)
        return res


def main():
    g = geom()
    # extra GEOM values we need but the SCAD does not echo
    g.setdefault("ear_a", [46, 58])
    out = {}
    for label, E in (("soft TPU (E~15)", 15), ("typical TPU (E~26)", 26), ("stiff TPU (E~40)", 40)):
        kp = kp_of(E, 2.4, 1.2, 3.6)
        for wood in (-0.15, 0.0, 0.15):
            band = Band(g, kp, wood_offset=wood)
            rows = []
            for k in range(0, 9):
                turn = k * g["dA"]
                r = band.clicks(turn)
                rows.append(dict(clicks=k, turn=turn, **{kk: v for kk, v in r.items() if kk not in ("phi", "squeeze")}))
            out[f"{label}, wood radius {wood:+.2f}"] = dict(kp=kp, rows=rows)
            print(f"\n{label}  kp={kp:.2f} MPa/mm  wood radius {wood:+.2f} mm")
            print(" clicks  squeeze min/mean/max (mm)   pressure mean/max (MPa)  contact  band tension (N)")
            for r in rows:
                print(f"  {r['clicks']:2d}     {r['squeeze_min']:.2f} / {r['squeeze_mean']:.2f} / {r['squeeze_max']:.2f}"
                      f"          {r['pressure_mean']:.2f} / {r['pressure_max']:.2f}        {r['contact_frac']*100:4.0f}%    {r['tension_equiv']:6.1f}")
    os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
    json.dump(out, open(os.path.join(ROOT, "build", "bandmodel.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
