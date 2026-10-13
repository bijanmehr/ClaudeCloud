"""Quasi-static force simulation of the cam lever through its whole travel.

For each lever angle the collar closes exactly as far as the cam pushes it
(from tools/kinematics.py). The TPU fins and the bending of the PETG C give
the collar's resistance; virtual work turns that into the cam contact force,
and statics on the lever gives the pin load, the stop load and the force a
hand has to apply at the handle.

Fin stiffness k_p (MPa per mm of radial squeeze) comes from a cantilever-fin
model: ~0.42 / 0.73 / 1.12 for TPU 95A with E = 15 / 26 / 40 MPa.
"""
import json
import math
import sys

import numpy as np
from shapely.geometry import Point
from shapely.ops import nearest_points

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import kinematics as K  # noqa: E402

MU = 0.30            # PETG on PETG, dry
PIN_R = 3.0
WALL = 4.0
E_PETG = 2000.0
CLAMP_H = 70.0
FIN_LEN = 65.0       # fin length minus the tapered ends


def simulate(chk_dir, kp, wood_offset=0.0, n=60, fit_gap=0.15, wall=None):
    c = K.Clamp(chk_dir)
    g = c.g
    S = c.sections(10.8)
    lever_sec, earA_sec = S["lever_closed"], S["earA_closed"]
    P0 = g["P"]
    wood_r = g["tip_r"] + 0.45          # nominal wood radius used by the model
    area = 2 * math.pi * wood_r * FIN_LEN
    wall = wall or (6.0 if g["shell_r"] > 26 else 4.0)
    rn = g["shell_r"] - wall / 2
    I_ring = CLAMP_H * wall ** 3 / 12
    thetas = np.linspace(-g["open_angle"], 0, n)
    fs = np.array([c.contact_f(S, t) for t in thetas])
    s = np.clip((1 - fs) * g["open_dr"] - fit_gap + wood_offset / 2, 0, None)   # squeeze on the wood
    dr = (1 - fs) * g["open_dr"]                                        # collar radius change
    # closure measured along the cam push direction, for d(U)/d(closure)
    closure = (1 - fs) * g.get("draw", 5.698)          # slit closure along the cam line
    U = 0.5 * kp * s ** 2 * area + 0.5 * E_PETG * I_ring * (dr / rn ** 2) ** 2 * 2 * math.pi * rn
    with np.errstate(divide="ignore", invalid="ignore"):
        Fc = np.gradient(U, closure)
    Fc = np.where(np.isfinite(Fc) & (fs < 0.999), np.maximum(Fc, 0), 0)
    rows = []
    for th, f, F in zip(thetas, fs, Fc):
        lev = c.lever(lever_sec, th, f)
        ea = c.pose_A(earA_sec, f)
        pc_l, pc_e = nearest_points(lev, ea)
        a, _ = c.a_f(f)
        nrm = np.array([-math.sin(math.radians(a - g["a0"])), math.cos(math.radians(a - g["a0"]))])  # ear A face normal (towards the cam)
        Pp = c.pose_B(Point(P0), f)
        P = np.array([Pp.x, Pp.y])
        cp = np.array([pc_e.x, pc_e.y])
        rvec = cp - P
        arm_n = rvec[0] * nrm[1] - rvec[1] * nrm[0]          # moment arm of the normal force about the pivot
        t_dir = np.array([-nrm[1], nrm[0]])
        arm_t = abs(rvec[0] * t_dir[1] - rvec[1] * t_dir[0])
        T_normal = F * arm_n                                   # torque from the ear on the lever (+ = towards closed, i.e. locking)
        T_fric = MU * F * (arm_t + PIN_R)                      # friction at cam face and pin, always resists
        rows.append(dict(theta=th, f=f, squeeze=None, Fc=F, arm=arm_n, T_normal=T_normal, T_fric=T_fric,
                         contact=cp.tolist(), normal=nrm.tolist(), pivot=P.tolist()))
    for r, sq in zip(rows, s):
        r["squeeze"] = sq
    return g, rows


def report(chk_dir, handle_len=48.0, stop_r=None):
    out = {}
    for label, kp, off in (("soft TPU (E~15), wood exact", 0.42, 0.0),
                           ("typical TPU (E~26), wood exact", 0.73, 0.0),
                           ("stiff TPU (E~40), wood exact", 1.12, 0.0),
                           ("WORST: stiff TPU, wood +0.4 mm", 1.12, 0.4)):
        g, rows = simulate(chk_dir, kp, off)
        # closing: hand torque = torque needed to keep rotating towards closed
        close_T = [r["T_normal"] * -1 + r["T_fric"] for r in rows]   # closing rotates +theta; ear torque opposes when arm<0
        close_T = [max(0.0, -r["T_normal"] + r["T_fric"]) if r["Fc"] > 0 else 0.0 for r in rows]
        peak = max(range(len(rows)), key=lambda i: close_T[i])
        last = rows[-1]
        lock_T = last["T_normal"]   # >0 means the ear pushes the lever further closed = locked
        Fmax = max(r["Fc"] for r in rows)
        out[label] = dict(kp=kp, wood_offset=off, Fc_max=Fmax, Fc_closed=last["Fc"], squeeze_closed=last["squeeze"],
                          pressure_closed=kp * last["squeeze"], peak_hand_N=close_T[peak] / handle_len,
                          peak_theta=rows[peak]["theta"], lock_torque_Nmm=lock_T,
                          open_hand_N=(lock_T + rows[-1]["T_fric"]) / handle_len,
                          pin_load_max=Fmax * math.sqrt(1 + MU ** 2), rows=rows)
        print(f"{label:34s} squeeze {last['squeeze']:.2f} mm  pressure {kp*last['squeeze']:.2f} MPa  "
              f"cam force max {Fmax:4.0f} N  hand push peak {close_T[peak]/handle_len:3.0f} N (at {rows[peak]['theta']+g['open_angle']:.0f} deg)  "
              f"lock torque {lock_T:+5.0f} N.mm  pull to open {out[label]['open_hand_N']:.0f} N")
    return out


if __name__ == "__main__":
    res = report(sys.argv[1])
    json.dump({k: {kk: vv for kk, vv in v.items() if kk != "rows"} | {"rows": v["rows"]} for k, v in res.items()},
              open(f"{sys.argv[1]}/forces.json", "w"), indent=1, default=float)
