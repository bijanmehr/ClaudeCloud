"""Slice files with the PrusaSlicer CLI as a printability smoke test.

This is not a substitute for your own slicer profile; it only proves the parts
slice cleanly in the intended orientation without support material and
reports filament use and print-time estimates.
"""
import re
import subprocess

_COMMON = [
    "--nozzle-diameter", "0.4", "--filament-diameter", "1.75",
    "--layer-height", "0.2", "--first-layer-height", "0.2",
    "--perimeters", "4", "--fill-density", "40%", "--fill-pattern", "gyroid",
    "--bed-shape", "0x0,250x0,250x210,0x210", "--center", "125,105",
]

PROFILES = {
    "PETG": ["--temperature", "240", "--first-layer-temperature", "240",
             "--bed-temperature", "80", "--filament-type", "PETG"],
    "TPU": ["--temperature", "228", "--first-layer-temperature", "228",
            "--bed-temperature", "50", "--filament-type", "FLEX",
            "--max-volumetric-speed", "3.5", "--retract-length", "0.8"],
}

_MM = ["--nozzle-diameter", "0.4,0.4", "--filament-diameter", "1.75,1.75",
       "--temperature", "240,228", "--first-layer-temperature", "240,228",
       "--bed-temperature", "70,70", "--filament-type", "PETG;FLEX",
       "--layer-height", "0.2", "--first-layer-height", "0.2",
       "--perimeters", "4", "--fill-density", "40%", "--fill-pattern", "gyroid",
       "--bed-shape", "0x0,250x0,250x210,0x210",
       "--wipe-tower", "--use-relative-e-distances"]


def _parse(gcode_path, log):
    text = open(gcode_path, errors="replace").read()
    def grab(key):
        m = re.search(r"^; " + re.escape(key) + r" = (.*)$", text, re.M)
        return m.group(1).strip() if m else None
    warnings = [l for l in log.splitlines()
                if re.search(r"warn|support|overhang|floating|stability|error", l, re.I)
                and "Alert if supports needed" not in l]
    return {
        "filament_mm": grab("filament used [mm]"),
        "filament_g": grab("total filament used [g]") or grab("filament used [g]"),
        "time": grab("estimated printing time (normal mode)"),
        "layers": len(re.findall(r"^;LAYER_CHANGE", text, re.M)),
        "tool_changes": len(re.findall(r"^T[0-9]+\s*$", text, re.M)),
        "warnings": warnings,
    }


def slice_single(model_path, material, gcode_path, extra=()):
    cmd = ["prusa-slicer", "--export-gcode", model_path, *_COMMON,
           *PROFILES[material], *extra, "--output", gcode_path]
    r = subprocess.run(cmd, capture_output=True, text=True)
    log = r.stdout + r.stderr
    if r.returncode != 0:
        raise RuntimeError(f"slicing {model_path} failed:\n{log[-2000:]}")
    return _parse(gcode_path, log)


def slice_multimaterial(model_path, gcode_path, extra=()):
    cmd = ["prusa-slicer", "--export-gcode", model_path, *_MM, *extra,
           "--output", gcode_path]
    r = subprocess.run(cmd, capture_output=True, text=True)
    log = r.stdout + r.stderr
    if r.returncode != 0:
        raise RuntimeError(f"slicing {model_path} failed:\n{log[-2000:]}")
    return _parse(gcode_path, log)
