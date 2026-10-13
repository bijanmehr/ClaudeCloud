#!/usr/bin/env bash
# Load every file in print/ with Bambu Studio's own loader and print the parts.
# Usage: tools/bambu_load/run_checks.sh path/to/bbs_load
set -euo pipefail
BBS=${1:?usage: run_checks.sh path/to/bbs_load}
cd "$(dirname "$0")/../../print"
for f in wood_clamp_BambuStudio_H2D.3mf hinge_test_BambuStudio_H2D.3mf wood_clamp_PrusaSlicer.3mf hinge_test_PrusaSlicer.3mf; do
  echo "##### $f"
  "$BBS" "$f" | grep -E "===|en_3mf_file_type|objects:|volumes:|volume\[[0-9]+\] name|type=|bbox WORLD|GUI" || true
done
for v in wood_clamp hinge_test; do
  echo "##### ${v}_PETG.stl + ${v}_TPU.stl"
  "$BBS" "${v}_PETG.stl" "${v}_TPU.stl" | sed -n '/answer YES/,$p' | grep -E "answer|name=|type=|delta_min" || true
done
