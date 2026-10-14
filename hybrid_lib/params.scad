// Shared settings for PETG + TPU hybrid parts (Bambu Lab H2D, 0.4 mm nozzles)
// -----------------------------------------------------------------------
// Conventions used by every block in blocks/:
//  - millimetres; Z is the print direction (the build plate is z = 0);
//  - every block is a module with a `part` argument:
//      "petg" -> the PETG body only, "tpu" -> the TPU body only,
//      "both" -> both, coloured, for previews;
//  - the TPU body is always (TPU region) minus (PETG body), so the two share
//    exact faces and never overlap;
//  - every PETG/TPU interface is held by shape, not by adhesion: the TPU
//    must not be able to slide out of the PETG in any direction
//    (tools/capture_check.py tests this).

layer_h = 0.2;
line_w = 0.42;            // extrusion width
min_tpu = 1.2;            // thinnest TPU wall: 3 lines
min_petg = 1.2;           // thinnest PETG wall: 3 lines
xy_clear = 0.4;           // side clearance between separate moving parts
z_clear = 0.6;            // empty layers between stacked moving parts (3 x 0.2)

// anchor features (PETG reaching into TPU)
anchor_neck = 1.3;        // dovetail neck at the PETG surface
anchor_head = 2.3;        // dovetail head inside the TPU
anchor_depth = 1.2;
anchor_gap_z = 1.2;       // TPU-filled breaks along a rib, so the TPU also wraps over it
rivet_d = 2.4;            // holes through PETG that TPU fills

petg_colour = "#3b6e9c";
tpu_colour = "#e2572b";
eps = 0.01;

// Show/return one material. Children: child(0) = PETG body, child(1) = TPU region.
module hybrid(part) {
    if (part == "petg") children(0);
    else if (part == "tpu") difference() { children(1); children(0); }
    else {
        color(petg_colour) children(0);
        color(tpu_colour) difference() { children(1); children(0); }
    }
}
