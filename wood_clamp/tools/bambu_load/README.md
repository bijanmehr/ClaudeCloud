# bbs_load: load the print files with Bambu Studio's own code

A small command-line program linked against Bambu Studio's core library
(libslic3r, built without the GUI). It loads 3MF and STL files through the
same functions Bambu Studio calls when you open or import them
(`Plater::priv::load_files`), then prints what you would end up with: objects,
parts, part type (normal part or support blocker), filament number and
position on the plate. It is used to check the files in `print/`.

Checked against Bambu Studio commit 79df7ae (version 02.08.04.57).

## Usage

```
bbs_load file.3mf            # "Open project" and "Import geometry only"
bbs_load a.stl b.stl         # each alone, then "Load these files as a single
                             # object with multiple parts?" answered No and Yes
bbs_load one.stl             # single STL, plus Split > To parts
./run_checks.sh path/to/bbs_load   # all files in print/
```

Environment: `BBS_BED_X` / `BBS_BED_Y` set the bed centre (default 175 / 160,
the H2D), `BBS_FILAMENTS` sets the number of filaments in the project (default
2), and `BBS_LOG` sets the log level (0 to 5).

## Building (Ubuntu 24.04, about 45 minutes on 4 cores)

1. Install the dependencies:
   ```
   sudo apt-get install build-essential pkg-config ninja-build libboost-all-dev libtbb-dev \
     libcgal-dev libopenvdb-dev libeigen3-dev libexpat1-dev libocct-data-exchange-dev \
     libocct-modeling-algorithms-dev libocct-modeling-data-dev libocct-foundation-dev \
     libocct-ocaf-dev libocct-visualization-dev libnlopt-cxx-dev libnlopt-dev libcereal-dev \
     libglew-dev libglfw3-dev libgl-dev libpng-dev libjpeg-dev libtiff-dev \
     libcurl4-openssl-dev libssl-dev libdraco-dev libnoise-dev zlib1g-dev libgmp-dev \
     libmpfr-dev libqhull-dev libfreetype-dev libfontconfig-dev libblosc-dev libopenexr-dev \
     libimath-dev libudev-dev libdbus-1-dev libopencv-dev libassimp-dev libgtk-3-dev
   ```
2. Get the source and apply the build-system patch. The patch changes CMake
   files only; no Bambu Studio C++ source is modified.
   ```
   git clone https://github.com/bambulab/BambuStudio && cd BambuStudio
   git checkout 79df7ae309be398ed30c9f397c13c9911c2edc57
   git apply /path/to/wood_clamp/tools/bambu_load/bambustudio-build.patch
   ```
   What the patch does:
   - builds against Ubuntu's split OpenCV libraries;
   - makes `libslic3r_cgal` link the shared Boost.Log;
   - adds an option to pin the build time, so re-running CMake does not
     force a full rebuild;
   - adds an option that includes this directory in the build.
3. Configure and build:
   ```
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DFLATPAK=ON \
     -DSLIC3R_GUI=OFF -DSLIC3R_STATIC=0 -DSLIC3R_PCH=OFF -DSLIC3R_BUILD_TESTS=OFF \
     -DSLIC3R_ENABLE_OPENMESHCRAFT_BOOLEAN=OFF -DSLIC3R_ALLOW_MCUT_BOOLEAN=ON \
     -DLIBNOISE_INCLUDE_DIR=/usr/include/libnoise -DLIBNOISE_LIBRARY=/usr/lib/libnoise.so \
     -DOpenCASCADE_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencascade \
     -DBBS_LOAD_HARNESS_DIR=/path/to/wood_clamp/tools/bambu_load -DBBS_LOAD_OUTPUT_DIR=$PWD \
     -DSLIC3R_FIXED_BUILD_TIME=20261013-000000
   cmake --build build --target bbs_load -j4
   ```

`CMakeLists.txt` here also compiles three unmodified Bambu Studio sources:
`src/slic3r/Utils/Http.cpp`, `BBLUtil.cpp`, and the nanosvg implementation.
libslic3r refers to them, and in the real application they come from the GUI
library.

## What it does not cover

Anything that needs the printer and filament presets, such as checking the
vendor of a project's settings or loading embedded presets. Those only matter
for 3MF projects saved by Bambu Studio itself.

# ilk_probe: measure Bambu Studio's beam interlocking

`ilk_probe.cpp` slices test objects through Bambu Studio's own
`PrintObject::slice()` (the same code as the slicer, including
`InterlockingGenerator`) and reports where the interlocking beams go: on which
layers, and how far each material reaches into the other. All the
interlocking numbers in the READMEs and notes came from it or from small
variants of it. `ilk_probe all` gives the built-in cases. The rim widths and
the check that no beam crosses a gap came from its `ILK_DUMP` geometry output.
The grid-position sweep for narrow joints and the gap between a top and a
bottom face came from extra cases that are not built in.

```
python3 build_ilk_probe.py /path/to/BambuStudio/build   # after bbs_load is built
ilk_probe all                     # built-in cases: blocks, gaps, overlaps, thin coupons
ilk_probe --3mf file.3mf [name]   # a real 3MF, with its own object settings
```

- `--3mf` reports the layers with interlocking and how far the PETG and TPU
  beams reach from the bore centre. `ILK_RDEEP=<r>` adds a histogram by
  angle; `ILK_AVOID` / `ILK_BEAM` override boundary avoidance and on/off.
- `ilk_results.txt` is the output of `ilk_probe all` on commit 79df7ae. It is
  the record behind these findings:
  - at the default boundary avoidance (2), joints less than about 2.4 mm high
    or about 5 mm wide get no beams, and the outer 1 to 2.5 mm of every joint
    gets none;
  - a gap of 0.01 mm between the two bodies stops all interlocking (the gap
    tolerance is compared in the wrong unit): no beam crosses the gap. Where
    the file still reports "interlocking" with a gap (boundary avoidance 0 or
    1), `ILK_DUMP` shows only material swapped on each side of it;
  - with overlapping bodies, the later part keeps its full shape and carves
    the earlier one.
