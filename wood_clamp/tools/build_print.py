"""Build the print files: STLs, a Bambu Studio 3MF (H2D) and a PrusaSlicer
3MF, for the full clamp and for the short hinge test piece; validate the
meshes and slice-test with the PrusaSlicer CLI.

Usage: python3 tools/build_print.py [extra openscad -D args, e.g. -D wood_d=37.8]
"""
import json
import os
import subprocess
import sys

import math

import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slicecheck  # noqa: E402
import threemf  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "print")
BED_CENTRE = (175.0, 160.0)      # Bambu Lab H2D plate is 350 x 320 mm

# Bambu Studio beam interlocking, saved as object settings in the Bambu 3MF (applied when the
# file is opened as a project). Narrow 0.6 mm beams at depth 1 keep the woven PETG/TPU zone
# inside the 3 mm TPU skin: measured with Bambu Studio's own slicing code on the hinge test,
# PETG beams stay at least 0.65 mm clear of the fin roots and appear on 96 of 107 layers
# (the default boundary avoidance of 2 leaves out the faces, where the PETG lips hold the TPU).
INTERLOCK = {"interlocking_beam": 1, "interlocking_beam_width": 0.6, "interlocking_depth": 1,
             "interlocking_boundary_avoidance": 2, "interlocking_beam_layer_count": 2,
             "interlocking_orientation": 22.5}

VARIANTS = {
    "wood_clamp": [],
    "hinge_test": ["-D", "clamp_h=21.4", "-D", "cam_bands=1"],   # one cam layer between two knuckles
}


def geom(extra):
    r = subprocess.run(["openscad", "-D", 'part="none"', *extra, "-o", "/tmp/_geom.csg", os.path.join(ROOT, "clamp.scad")],
                       capture_output=True, text=True)
    line = [l for l in r.stderr.splitlines() if "GEOM" in l][0]
    return json.loads(line.split("GEOM ", 1)[1].rstrip('"'))


def export(part, path, extra):
    subprocess.run(["openscad", "-q", "-D", f'part="{part}"', *extra, "-o", path, os.path.join(ROOT, "clamp.scad")],
                   check=True, capture_output=True)
    return trimesh.load(path)


def main(user_args):
    os.makedirs(OUT, exist_ok=True)
    report = {}
    for name, extra in VARIANTS.items():
        args = extra + user_args
        petg_p, tpu_p = (os.path.join(OUT, f"{name}_PETG.stl"), os.path.join(OUT, f"{name}_TPU.stl"))
        petg, tpu = export("petg", petg_p, args), export("tpu", tpu_p, args)
        bodies = len(petg.split(only_watertight=False)), len(tpu.split(only_watertight=False))
        overlap = petg.intersection(tpu).volume
        ok = petg.is_watertight and tpu.is_watertight and bodies == (2, 1) and overlap < 0.05
        lo = petg.bounds[0].copy(); hi = petg.bounds[1].copy()
        lo = [min(a, b) for a, b in zip(lo, tpu.bounds[0])]; hi = [max(a, b) for a, b in zip(hi, tpu.bounds[1])]
        offset = (BED_CENTRE[0] - (lo[0] + hi[0]) / 2, BED_CENTRE[1] - (lo[1] + hi[1]) / 2, -lo[2])
        # support blocker around the printed hinge: even with supports switched
        # on, nothing may be generated inside the hinge gaps
        g = geom(args)
        a = 180 - (180 - g["a0"]) * (g["shell_r"] - g["shell_wall"] / 2) / (g["shell_r"] - g["shell_wall"] / 2 + g["open_dr"])
        px, py = g["P"]
        c, s = math.cos(math.radians(g["a0"])), math.sin(math.radians(g["a0"]))
        qx, qy = px * c - py * s + g["open_dr"], px * s + py * c
        c, s = math.cos(math.radians(-a)), math.sin(math.radians(-a))
        pin = (qx * c - qy * s, qx * s + qy * c)
        blocker = trimesh.creation.cylinder(radius=g["lobe"] + 0.45 + 1.8, height=hi[2] - lo[2], sections=64)
        blocker.apply_translation((pin[0], pin[1], (lo[2] + hi[2]) / 2))
        parts = [(f"{name} collar + lever (PETG, left nozzle)", petg, 1),
                 (f"{name} spring liner (TPU 95A, right nozzle)", tpu, 2),
                 ("hinge support blocker", blocker, 1, "support_blocker")]
        bambu = os.path.join(OUT, f"{name}_BambuStudio_H2D.3mf")
        threemf.write_bambu_3mf(bambu, name, parts, offset, settings=INTERLOCK)
        prusa = os.path.join(OUT, f"{name}_PrusaSlicer.3mf")
        threemf.write_multimaterial_3mf(prusa, name, parts, offset)
        sl = slicecheck.slice_multimaterial(prusa, os.path.join("/tmp", f"{name}_mm.gcode"))
        report[name] = dict(ok=bool(ok), petg_watertight=bool(petg.is_watertight), tpu_watertight=bool(tpu.is_watertight),
                            bodies_petg_tpu=list(bodies), petg_tpu_overlap_mm3=round(float(overlap), 4),
                            footprint_mm=[round(float(hi[i] - lo[i]), 1) for i in range(3)],
                            petg_cm3=round(float(petg.volume) / 1000, 1), tpu_cm3=round(float(tpu.volume) / 1000, 1),
                            bambu_3mf_parts=threemf.check_bambu_3mf(bambu),
                            slice_test=dict(filament_mm=sl["filament_mm"], time=sl["time"], layers=sl["layers"],
                                            tool_changes=sl["tool_changes"], warnings=sl["warnings"][:5]))
        print(name, json.dumps(report[name], indent=1))
    json.dump(report, open(os.path.join(OUT, "build_report.json"), "w"), indent=1)
    return all(r["ok"] for r in report.values())


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1:]) else 1)
