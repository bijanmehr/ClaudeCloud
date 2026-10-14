"""Build the PETG + TPU joint test plate for a Bambu Lab H2D.

Two of each coupon type (A-H pull coupons, P peel strip), laid out on one
plate. Writes:
  print/bond_tests_BambuStudio_H2D.3mf   every coupon is its own object (PETG part on
                                         filament 1, TPU part on filament 2); B and H
                                         carry the "beam interlocking" object setting
  print/bond_tests_PETG.stl + _TPU.stl   the same plate as two STLs (import both at once)
  print/build_report.json                mesh checks and a two-extruder slice test

Usage: python3 bond_tests/build.py
"""
import json
import os
import subprocess
import sys
import tempfile

import trimesh

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "..", "wood_clamp", "tools"))
import slicecheck  # noqa: E402
import threemf  # noqa: E402

SCAD = os.path.join(ROOT, "coupons.scad")
OUT = os.path.join(ROOT, "print")

TYPES = {
    "A": "butt joint",
    "B": "butt joint + beam interlocking",
    "C": "lap joint (TPU on top of PETG)",
    "D": "large dovetail",
    "E": "three small dovetails",
    "F": "encapsulated T-anchor",
    "G": "riveted tongue",
    "H": "encapsulated T-anchor + beam interlocking",
    "P": "peel strip",
}
# Bambu's interlocking keeps clear of outer surfaces by `boundary_avoidance` cells (all
# three directions). Measured with Bambu Studio's slicing code: at the default of 2 these
# 3.2 mm coupons would get beams in only 8 of their 16 layers, and joints under about
# 6 mm wide / 2.4 mm tall get none, so it is 0 here. 45 deg makes both beam directions
# cross the joint faces (they lie along Y); depth 3 reaches about 3 mm into each side.
INTERLOCK = {"interlocking_beam": 1, "interlocking_boundary_avoidance": 0,
             "interlocking_orientation": 45, "interlocking_depth": 3}
COLS_X = (100.0, 245.0)        # coupon centres, inside the area both H2D nozzles reach
ROW_Y0, ROW_PITCH = 70.0, 20.0


def export(part, ty):
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "m.stl")
        subprocess.run(["openscad", "-q", "-D", f'part="{part}"', "-D", f'type="{ty}"', "-o", p, SCAD],
                       check=True, capture_output=True)
        return trimesh.load(p)


def main():
    os.makedirs(OUT, exist_ok=True)
    objects, all_petg, all_tpu, report = [], [], [], {}
    for row, (ty, desc) in enumerate(TYPES.items()):
        petg, tpu = export("petg", ty), export("tpu", ty)
        overlap = petg.intersection(tpu).volume
        n_petg = len(petg.split(only_watertight=False))
        ok = petg.is_watertight and tpu.is_watertight and len(tpu.split(only_watertight=False)) == 1 \
            and overlap < 0.01 and n_petg == (1 if ty == "P" else 2)
        report[ty] = dict(description=desc, ok=bool(ok), petg_bodies=n_petg, overlap_mm3=round(float(overlap), 4),
                          petg_cm3=round(float(petg.volume) / 1000, 2), tpu_cm3=round(float(tpu.volume) / 1000, 2))
        lo = [min(a, b) for a, b in zip(petg.bounds[0], tpu.bounds[0])]
        hi = [max(a, b) for a, b in zip(petg.bounds[1], tpu.bounds[1])]
        for col, cx in enumerate(COLS_X):
            off = (cx - (lo[0] + hi[0]) / 2, ROW_Y0 + row * ROW_PITCH - (lo[1] + hi[1]) / 2, -lo[2])
            name = f"{ty}{col + 1} {desc}"
            objects.append(dict(name=name, offset=off,
                                parts=[(f"{ty}{col + 1} PETG", petg, 1), (f"{ty}{col + 1} TPU", tpu, 2)],
                                settings=INTERLOCK if ty in "BH" else {}))
            all_petg.append(petg.copy().apply_translation(off))
            all_tpu.append(tpu.copy().apply_translation(off))
        print(ty, desc, report[ty])
    threemf.write_bambu_3mf_multi(os.path.join(OUT, "bond_tests_BambuStudio_H2D.3mf"), "PETG-TPU joint tests", objects)
    petg_all, tpu_all = trimesh.util.concatenate(all_petg), trimesh.util.concatenate(all_tpu)
    petg_all.export(os.path.join(OUT, "bond_tests_PETG.stl"))
    tpu_all.export(os.path.join(OUT, "bond_tests_TPU.stl"))
    # slice test: whole plate as one two-material object in PrusaSlicer
    prusa = os.path.join(tempfile.gettempdir(), "bond_tests_PrusaSlicer.3mf")
    threemf.write_multimaterial_3mf(prusa, "bond tests", [("PETG", petg_all, 1), ("TPU", tpu_all, 2)])
    sl = slicecheck.slice_multimaterial(prusa, os.path.join(tempfile.gettempdir(), "bond_tests.gcode"),
                                        extra=["--bed-shape", "0x0,350x0,350x320,0x320"])
    report["slice_test"] = dict(time=sl["time"], layers=sl["layers"], tool_changes=sl["tool_changes"],
                                filament_mm=sl["filament_mm"], warnings=sl["warnings"][:4])
    json.dump(report, open(os.path.join(OUT, "build_report.json"), "w"), indent=1)
    print("slice test:", report["slice_test"])
    return all(v["ok"] for k, v in report.items() if k in TYPES)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
