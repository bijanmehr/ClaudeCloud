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
openscad -q -D 'part="petg"' "$@" -o build/petg.stl clamp.scad
openscad -q -D 'part="tpu"' "$@" -o build/tpu.stl clamp.scad
openscad -q -D 'part="tpu_closed"' "$@" -o build/tpu_closed.stl clamp.scad
