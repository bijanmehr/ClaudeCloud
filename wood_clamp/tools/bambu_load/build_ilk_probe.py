"""Compile ilk_probe.cpp against an existing bbs_load build tree.

Reuses the compiler flags, include paths and libraries that CMake wrote for
bbs_load (so build bbs_load first, see README.md), then links ilk_probe.

Usage: python3 build_ilk_probe.py /path/to/BambuStudio/build
"""
import glob
import os
import re
import subprocess
import sys

B = os.path.abspath(sys.argv[1])
here = os.path.dirname(os.path.abspath(__file__))
txt = open(os.path.join(B, "build.ninja")).read()


def get(key, s):
    return re.search(r"\n  " + key + r" = (.*)\n", s).group(1)


blk = txt[txt.index("build bbs_load_harness/CMakeFiles/bbs_load.dir/bbs_load.cpp.o"):]
DEF, INC, FL = get("DEFINES", blk), get("INCLUDES", blk), get("FLAGS", blk)
lk = txt[txt.index("# Link build statements for EXECUTABLE target bbs_load"):]
LL, LFL = get("LINK_LIBRARIES", lk), get("FLAGS", lk)
objs = [os.path.relpath(f, B) for f in glob.glob(B + "/bbs_load_harness/**/*.o", recursive=True)
        if not f.endswith("bbs_load.cpp.o")]
obj = os.path.join(B, "ilk_probe.o")
print("compiling...")
subprocess.run(f"cd {B} && c++ {DEF} {INC} {FL} -c {here}/ilk_probe.cpp -o {obj}", shell=True, check=True)
print("linking...")
subprocess.run(f"cd {B} && c++ {LFL} {obj} {' '.join(objs)} -o {B}/ilk_probe {LL}", shell=True, check=True)
print("built", os.path.join(B, "ilk_probe"))
