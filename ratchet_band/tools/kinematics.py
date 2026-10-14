"""Closing motion of the ratchet band, checked step by step.

Model: the two jaws are rigid. The thin upper band (between the jaw ends at
+-jaw_end) wraps the wood ever tighter: its length stays the same while its
radius shrinks by dR, so it spans a larger angle and both jaw ends swing
down towards the ratchet. Each jaw follows its end: it turns with the band
and moves in by dR. Relative to jaw A, jaw B (and its rack) turns towards
-phi; every tooth pitch of that turn is one click.

At each step the script reports the fin squeeze on the wood, the clicks,
the overlap of the rack with jaw A's teeth and stop, how far the rack
pushes the spring finger out, and the gap left between the jaw ends.

Usage: python3 tools/kinematics.py [extra openscad -D args]
"""
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
import trimesh
from shapely import affinity
from shapely.geometry import Point
from shapely.ops import unary_union

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCAD = os.path.join(ROOT, "band.scad")


def export(part, extra=()):
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, f"{part}.stl")
        r = subprocess.run(["openscad", "-D", f'part="{part}"', *extra, "-o", out, SCAD],
                           capture_output=True, text=True, check=True)
        geom = None
        for line in r.stderr.splitlines():
            if "GEOM " in line:
                geom = json.loads(line.split("GEOM ", 1)[1].rstrip('"'))
        m = trimesh.load(out)
    sec = m.section(plane_origin=[0, 0, 0.5], plane_normal=[0, 0, 1])
    planar, _ = sec.to_2D(to_2D=np.eye(4))
    return unary_union(planar.polygons_full), geom


def pose(poly, rot_deg, shift):
    """Rotate about the band centre by rot_deg (counter-clockwise), then shift."""
    p = affinity.rotate(poly, rot_deg, origin=(0, 0))
    return affinity.translate(p, *shift)


def radial(phi_deg):
    a = math.radians(phi_deg)
    return np.array([math.sin(a), -math.cos(a)])


def jaw_motion(g, dR):
    R_n = (g["R_i"] + g["R_o"]) / 2
    th_u = 360 - 2 * g["jaw_end"]
    th_u2 = th_u * R_n / (R_n - dR)
    phi_b = 180 - th_u2 / 2                   # where jaw B now meets the band
    delta = phi_b - g["jaw_end"]               # jaw B turns by this (negative)
    shift_b = -dR * radial(phi_b)
    shift_a = -dR * radial(-phi_b)
    return delta, shift_a, shift_b


def main(extra=()):
    A, g = export("A_slab", extra)
    F, _ = export("F_slab", extra)
    B, _ = export("B_slab", extra)
    dA = g["dA"]
    # rack = the part of jaw B below the band end (phi < g2); body = the rest
    wedge_neg = affinity.rotate(Point(0, 0).buffer(60).intersection(
        Point(0, -60).buffer(60)), 0)
    rows, ok = [], True
    print(f"tooth pitch {dA:.3f} deg; fixed teeth {g['ft_lo']:.1f}..{g['ft_hi']:.1f} deg; "
          f"rack tip {g['rk_tip']:.1f} deg; stop {g['stop_a']:.1f} deg")
    print(" dR    turn  clicks  fin tips  squeeze  rack-in-teeth  finger pushed  end gap  rack tip")
    for dR in np.round(np.arange(0.0, 2.01, 0.05), 3):
        delta, sa, sb = jaw_motion(g, dR)
        A2 = pose(A, -delta, sa)
        F2 = pose(F, -delta, sa)
        B2 = pose(B, delta, sb)
        turn = -2 * delta                       # B relative to A, towards -phi
        clicks = turn / dA
        # overlap of jaw B with jaw A's rigid body (teeth, stop, band end)
        ov = B2.intersection(A2).area
        # how far the rack pushes into the finger: largest depth of overlap
        fo = B2.intersection(F2)
        push = 0.0
        if not fo.is_empty:
            for geom in getattr(fo, "geoms", [fo]):
                c = np.array(geom.exterior.coords)
                r = np.linalg.norm(c, axis=1)
                push = max(push, float(g["r_arm_in"] + 0.0 - r.min()) if r.size else 0.0)
        # gap between the jaw ends (inner band corners)
        end_gap = A2.distance(B2) if ov < 1e-6 else 0.0
        # rack tip angle in jaw A's frame
        tip_world = pose(Point(*(g["r_rk_root"] * radial(g["rk_tip"]))), delta, sb)
        tip_in_A = pose(affinity.translate(tip_world, -sa[0], -sa[1]), delta, (0, 0))
        tip_phi = math.degrees(math.atan2(tip_in_A.x, -tip_in_A.y))
        tips = g["R_tip"] - dR
        squeeze = g["wood_r"] - tips
        rows.append(dict(dR=dR, turn=turn, clicks=clicks, tips=tips, squeeze=squeeze, overlap=ov,
                         finger_push=push, end_gap=end_gap, tip_phi=tip_phi))
        print(f"{dR:4.2f}  {turn:5.2f}  {clicks:5.2f}   {tips:6.2f}   {squeeze:6.2f}    "
              f"{ov:8.3f} mm2     {push:5.2f} mm    {end_gap:5.2f}    {tip_phi:6.2f}")
    json.dump(dict(geom=g, rows=rows), open(os.path.join(ROOT, "build", "kinematics.json"), "w"), indent=1)
    return rows, g


if __name__ == "__main__":
    os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
    main(sys.argv[1:])
