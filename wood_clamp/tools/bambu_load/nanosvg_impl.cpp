// nanosvg implementation: in Bambu Studio this lives in src/slic3r/GUI/BitmapCache.cpp (libslic3r_gui),
// but libslic3r/NSVGUtils.cpp references it. Compile it here for the non-GUI harness.
#include <cstdio>
#include <cstring>
#include <cmath>
#define NANOSVG_IMPLEMENTATION
#include "nanosvg/nanosvg.h"
