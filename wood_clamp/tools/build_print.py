"""Build the print files: STLs, a Bambu Studio 3MF (H2D) and a PrusaSlicer
3MF, for the full clamp and for the short hinge test piece; validate the
meshes and slice-test with the PrusaSlicer CLI.

Usage: python3 tools/build_print.py [extra openscad -D args, e.g. -D wood_d=37.8]
"""
import json
import os
import subprocess
import sys

import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slicecheck  # noqa: E402
import threemf  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "print")
BED_CENTRE = (175.0, 160.0)      # Bambu Lab H2D plate is 350 x 320 mm

VARIANTS = {
    "wood_clamp": [],
    "hinge_test": ["-D", "clamp_h=21.4", "-D", "cam_bands=1"],   # one cam layer between two knuckles
}


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
        parts = [(f"{name} collar + lever (PETG, left nozzle)", petg, 1),
                 (f"{name} spring liner (TPU 95A, right nozzle)", tpu, 2)]
        bambu = os.path.join(OUT, f"{name}_BambuStudio_H2D.3mf")
        threemf.write_bambu_3mf(bambu, name, parts, offset)
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
