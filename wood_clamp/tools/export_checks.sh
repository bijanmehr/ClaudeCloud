#!/usr/bin/env bash
# Re-export the meshes used by tools/kinematics.py into build/chk.
# Usage: tools/export_checks.sh [extra openscad -D args...]
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build/chk
for prt in earA_closed bgroup_closed ring_closed ring_open collar_closed collar_open; do
  openscad -q -D "part=\"$prt\"" "$@" -o "build/chk/$prt.stl" clamp.scad
done
openscad -q -D 'part="lever"' -D 'lever_open=0' -D 'lever_theta=0' "$@" -o build/chk/lever_closed.stl clamp.scad
openscad -D 'part="lever"' -D 'lever_open=1' -D 'lever_theta=-135' "$@" -o build/chk/lever_open.stl clamp.scad 2>&1 \
  | grep GEOM | sed 's/.*GEOM //; s/"$//' > build/chk/geom.json
# coarser facets for the FEA rebuild (tools/brep.py, collar_fea.py, stress_check.py)
openscad -q -D 'part="collar_closed"' -D '$fn=48' "$@" -o build/chk/fea_collar_closed.stl clamp.scad
openscad -q -D 'part="lever"' -D 'lever_open=0' -D 'lever_theta=0' -D '$fn=48' "$@" -o build/chk/fea_lever_closed.stl clamp.scad
openscad -q -D 'part="petg"' "$@" -o build/petg.stl clamp.scad
openscad -q -D 'part="tpu"' "$@" -o build/tpu.stl clamp.scad
openscad -q -D 'part="tpu_closed"' "$@" -o build/tpu_closed.stl clamp.scad
# every printed part must be one connected solid: PETG = collar + lever, TPU = liner
python3 - <<'PY'
import sys, trimesh
for f, n in (("build/petg.stl", 2), ("build/tpu.stl", 1), ("build/chk/lever_closed.stl", 1), ("build/chk/bgroup_closed.stl", 1)):
    m = trimesh.load(f); b = len(m.split(only_watertight=False))
    print(f"{f}: {b} solid(s), watertight {m.is_watertight}")
    if b != n or not m.is_watertight:
        sys.exit(f"CONNECTIVITY FAIL: {f} has {b} solids, expected {n}")
PY
