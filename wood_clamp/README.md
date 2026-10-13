# Screwless cam-lever clamp for a cracked wooden cylinder

A clamp for a wooden cylinder (38 mm diameter, 70 mm tall) that holds an
axial crack closed and stops it running further. It is **one print with no
screws, pins or other parts**: a PETG collar with a TPU spring lining and a
PETG lever whose hinge is printed already assembled.

| Closed on the wood | Open (slides on and off) | As printed |
|---|---|---|
| ![closed](images/closed_on_wood.png) | ![open](images/open_on_wood.png) | ![printed](images/as_printed.png) |

Motion videos: [3D view](images/motion_3d.mp4), [top view](images/motion_top.mp4).

## How it works

- The **collar** is a C shape with a slit. It is printed slightly open, so it
  slides over the end of the wood with about 0.15 mm of clearance.
- The **lever** turns on a 6 mm pin that is part of the collar. Its four cam
  rings sit between fixed knuckles, so it cannot come off
  ([slice through the hinge](images/hinge_slice.png)).
- Pushing the lever down makes its off-centre cam press the ear beside the
  slit. The slit closes, the collar shrinks, and the **TPU fins** squeeze the
  wood. The fins are springs, so the grip stays steady as the wood swells and
  shrinks with humidity.
- In the last 10° of travel the cam passes dead centre. From there the load
  holds the lever shut, so it cannot spring open.

## Files (`print/`)

| File | What it is |
|---|---|
| `wood_clamp_BambuStudio_H2D.3mf` | **Open this in Bambu Studio.** One object, two parts: PETG = filament 1, TPU = filament 2 |
| `hinge_test_BambuStudio_H2D.3mf` | A 21 mm tall working slice of the clamp (one cam ring). **Print this first.** |
| `*_PETG.stl`, `*_TPU.stl` | The same parts as STLs, in the same position. Select both, drag them in together, and answer **Yes** to "load as a single object with multiple parts", then set the filament for each part |
| `*_PrusaSlicer.3mf` | The same thing for PrusaSlicer (Bambu Studio cannot read this layout) |
| `build_report.json` | Mesh checks and slice test for the files above |

All files are for **38.0 mm wood**. If yours measures differently, see
"Your wood diameter" below.

## Printing on a Bambu Lab H2D

### Filaments and nozzles
- **Filament 1: PETG** (for example Bambu PETG HF or PETG Basic), **left nozzle**. It can feed from the AMS.
- **Filament 2: TPU 95A** (for example Bambu TPU 95A HF), **right nozzle**,
  from the external spool through the TPU bypass, **not through the AMS**.
  On the H2D, TPU only prints from the right nozzle.
- Bambu's harder "TPU for AMS" (68D) is much stiffer. The fins are sized for 95A.
- In Bambu Studio, set **filament grouping to manual**: PETG on the left, TPU on the right.

### Recommended settings (best balance of time and quality)
Start from the **0.20mm Standard @BBL H2D** process and change only these:

| Setting | Value | Why |
|---|---|---|
| Layer height | **0.20 mm, do not change** | The hinge gaps are exactly 3 empty 0.2 mm layers |
| First layer height | 0.20 mm | Same reason |
| Wall loops | **4** | Makes the pin, cam rings and hinge web almost solid |
| Top / bottom shells | 5 / 4 | |
| Sparse infill | 20 % gyroid | The strength comes from the walls; more infill mostly adds time |
| **Supports** | **OFF** | Supports would fill the hinge gaps. Ignore the "floating / long bridge, consider supports" warning: those bridges are intended |
| Thick bridges | **OFF** | Thick bridges eat into the 0.6 mm hinge gaps |
| Bridge / overhang fan | 100 % | Clean first layers over the hinge gaps |
| Seam | Aligned | |
| Outer wall speed | 150 mm/s (PETG) | Truer pin and cam surfaces; leave other speeds at the profile defaults |
| Brim | Outer brim only, 3 mm | Steadies the tall handle; at the bed the lever is 7 mm from the collar, so the brims cannot join |
| Elephant foot compensation | default (0.15 mm) | |
| Prime tower | ON (default) | Needed for the nozzle change |
| Plate | Textured PEI, no glue | |
| Temperatures | filament profile defaults, bed 70 °C | |

To print faster, drop infill to 15 % and keep everything else. To get a
smoother finish, take the outer wall speed down to about 100 mm/s. **Do not
change the layer height** unless you rebuild the files with the matching
`layer_h` (see Customising).

Bambu Studio shows the exact print time. The full clamp uses about 128 cm³ of
PETG (~160 g) and 15 cm³ of TPU (~18 g). The hinge test uses about a third of that.

### After printing
1. Let it cool on the plate, then remove it.
2. **Free the hinge:** hold the collar and work the lever back and forth a few
   times. The layers over the hinge gaps may be lightly stuck at first. They
   should come free with steady thumb pressure; if needed, run a thin blade
   along the gaps.
3. With the **hinge test** check three things: the lever swings freely, it
   pushes over its last 10° and stays shut, and it holds the wood. If the hinge
   is fused, rebuild with `xy_gap` 0.55 and `z_gap_layers` 4. If it feels
   loose, use `xy_gap` 0.35.

## Fitting it to the wood
1. For a proper repair, first work wood glue (PVA) or thin epoxy into the crack.
2. Open the lever and slide the collar over the end of the wood. Turn it so
   **the slit is close to the crack**: the squeeze is strongest within about
   45° of the slit.
3. Push the handle down until it stops against the collar. It is now locked.
   Expect roughly 20–30 N at the handle tip (a firm push with the thumb).
4. To remove it, lift the handle tip and slide the collar off.

## Your wood diameter
Measure the cylinder with calipers in 3–4 places and use the average. Then
rebuild the print files:

```
python3 tools/build_print.py -D wood_d=37.8
```

One print fits wood from about 0.3 mm under to 0.5 mm over the diameter it was
built for. Inside that range the pressure on the wood varies by about ±30 %.

## What was checked (simulation, not yet a real print)
- **Motion:** the lever was swept from open to closed. At each step the collar
  closes exactly as far as the cam pushes it, and collisions were checked in
  2D (`tools/kinematics.py`) and in full 3D on the print meshes
  (`tools/sweep3d.py`). There are no collisions. As printed, the narrowest
  gaps are 0.45 mm (pin), 0.6 mm (between layers), about 0.6 mm (ear to arm)
  and 1.0 mm (cam to ear). The lever locks past dead centre.
- **Forces:** squeeze, cam force and hand force through the whole travel
  (`tools/forces.py`, rigid-part model; `tools/collar_fea.py` adds the
  flexing of the PETG parts).
- **Stress:** 3D finite-element analysis with CalculiX, on the parts rebuilt
  as exact solids (`tools/brep.py`, `tools/fea.py`, `tools/stress_check.py`).
  The solver was validated on a beam against textbook values. With the fins
  modelled as springs (version 3, typical TPU): cam force about 230 N; squeeze
  0.23 / 0.32 / 0.55 mm (min / average / max); average pressure about
  0.23 MPa; collar stress mostly below 10 MPa (PETG is about 45 MPa).
  **Open item:** one corner where the hinge web joins the arm reaches about
  17–22 MPa across the layers in the model. It is being reinforced, and the
  lever's stress check is being repeated for version 3.
- **Print files:** watertight meshes, exactly one collar, one lever and one
  liner, no PETG/TPU overlap. They slice with two extruders in PrusaSlicer, and
  the Bambu 3MF was checked against Bambu Studio's own importer logic.
- **Not yet done:** a real print. Print the hinge test first.

## Customising
Every dimension is a parameter in `clamp.scad`: wood diameter, clamp height,
squeeze, fit gap, fin shape, wall, cam and handle sizes, and the
print-in-place gaps (`xy_gap`, `z_gap_layers`, `layer_h`). `tools/` has the
build, check and simulation scripts:

```
python3 tools/build_print.py [-D name=value ...]      # print files + checks
tools/export_checks.sh && python3 tools/kinematics.py build/chk 10.8 2.75 && python3 tools/sweep3d.py build/chk
python3 tools/forces.py build/chk                     # force simulation
python3 tools/collar_fea.py build/chk 3.8             # collar FEA, fins as springs
python3 tools/animate.py build images/motion_3d.mp4   # motion video
```

Requirements: OpenSCAD 2021+, Python 3 with trimesh, manifold3d, shapely,
numpy, scipy, gmsh, tetgen, matplotlib and Pillow; CalculiX (`ccx`);
PrusaSlicer (optional slice test); ffmpeg (videos).
