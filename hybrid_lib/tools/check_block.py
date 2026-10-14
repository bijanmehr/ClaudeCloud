"""Validate one hybrid block: export, mesh checks, zero-adhesion capture, slice test.

Usage:
  python3 tools/check_block.py blocks/<file>.scad [-D name=value ...] [--petg-bodies N] [--tpu-bodies N]
                               [--out DIR] [--no-slice]
The .scad file must honour `part` = "petg" / "tpu" (see params.scad) for whatever
it shows at top level with the given -D settings.
Prints a JSON report and exits non-zero if a check fails.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "wood_clamp", "tools"))
import capture_check  # noqa: E402


def export(scad, part, defs, out):
    cmd = ["openscad", "-q", "-D", f'part="{part}"'] + [x for d in defs for x in ("-D", d)] + ["-o", out, scad]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr[-2000:])
    return trimesh.load(out)


def floating_islands(petg, tpu, layer=0.2):
    """Regions of a layer (either material) that rest on nothing printed below them.
    Printed without supports, those would be extruded into the air."""
    import numpy as np
    from shapely.ops import unary_union
    top = max(petg.bounds[1][2], tpu.bounds[1][2])
    def sec(z):
        polys = []
        for m in (petg, tpu):
            s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if s is not None:
                pl, _ = s.to_2D(to_2D=np.eye(4))
                polys += list(pl.polygons_full)
        return unary_union(polys).buffer(0) if polys else None
    out, below = [], None
    z = layer / 2
    while z < top:
        cur = sec(z)
        if cur is not None and below is not None and not cur.is_empty:
            for g in getattr(cur, "geoms", [cur]):
                if g.area > 0.05 and g.intersection(below).area < 0.01:
                    c = g.representative_point()
                    out.append(dict(z=round(z, 2), area=round(g.area, 2), at=[round(c.x, 1), round(c.y, 1)]))
        below = cur if cur is not None else below
        z += layer
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scad")
    ap.add_argument("-D", action="append", default=[])
    ap.add_argument("--petg-bodies", type=int, default=None)
    ap.add_argument("--tpu-bodies", type=int, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-slice", action="store_true")
    a = ap.parse_args()
    out = a.out or tempfile.mkdtemp()
    os.makedirs(out, exist_ok=True)
    pp, tp = os.path.join(out, "petg.stl"), os.path.join(out, "tpu.stl")
    petg, tpu = export(a.scad, "petg", a.D, pp), export(a.scad, "tpu", a.D, tp)
    rep = dict(petg_watertight=bool(petg.is_watertight), tpu_watertight=bool(tpu.is_watertight),
               petg_bodies=len(petg.split(only_watertight=False)), tpu_bodies=len(tpu.split(only_watertight=False)),
               petg_cm3=round(float(petg.volume) / 1000, 3), tpu_cm3=round(float(tpu.volume) / 1000, 3),
               overlap_mm3=round(float(petg.intersection(tpu).volume), 4),
               size_mm=[round(float(v), 1) for v in (petg.bounds[1] - petg.bounds[0])],
               min_z=round(float(min(petg.bounds[0][2], tpu.bounds[0][2])), 3))
    rep["floating_islands"] = floating_islands(petg, tpu)
    ok, cap = capture_check.check(petg, tpu)
    rep["captured"] = bool(ok)
    rep["capture"] = [dict(piece=c["piece"], volume=round(c["volume"], 1), captured=c["captured"],
                           escapes=c["escapes"], worst_free_mm=c["worst_free_mm"]) for c in cap]
    fails = []
    if not (rep["petg_watertight"] and rep["tpu_watertight"]):
        fails.append("mesh not watertight")
    if rep["overlap_mm3"] > 0.01:
        fails.append("PETG and TPU overlap")
    if a.petg_bodies is not None and rep["petg_bodies"] != a.petg_bodies:
        fails.append(f"PETG bodies {rep['petg_bodies']} != {a.petg_bodies}")
    if a.tpu_bodies is not None and rep["tpu_bodies"] != a.tpu_bodies:
        fails.append(f"TPU bodies {rep['tpu_bodies']} != {a.tpu_bodies}")
    if not ok:
        fails.append("TPU not captured")
    if rep["floating_islands"]:
        fails.append("regions start in mid-air (need support)")
    if abs(rep["min_z"]) > 1e-3:
        fails.append("part does not sit on z = 0")
    if not a.no_slice:
        import slicecheck
        import threemf
        p3 = os.path.join(out, "slice_test.3mf")
        threemf.write_multimaterial_3mf(p3, "block", [("PETG", petg, 1), ("TPU", tpu, 2)])
        try:
            sl = slicecheck.slice_multimaterial(p3, os.path.join(out, "slice_test.gcode"),
                                                extra=["--bed-shape", "0x0,350x0,350x320,0x320", "--center", "175,160"])
            rep["slice"] = dict(ok=True, layers=sl["layers"], tool_changes=sl["tool_changes"],
                                warnings=[w for w in sl["warnings"] if "=>" not in w][:5])
        except Exception as e:
            rep["slice"] = dict(ok=False, error=str(e)[-600:])
            fails.append("slicing failed")
    rep["fails"] = fails
    rep["ok"] = not fails
    rep["files"] = dict(petg=pp, tpu=tp)
    print(json.dumps(rep, indent=1))
    sys.exit(0 if not fails else 1)


if __name__ == "__main__":
    main()
