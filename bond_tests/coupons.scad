// PETG + TPU joint test coupons
// -----------------------------------------------------------------------
// Pull coupons: PETG grip - TPU strap - PETG grip, printed flat. Every
// coupon type joins the TPU to the PETG a different way (the same way at
// both ends). Hook one grip on something solid, pull the other with a
// luggage scale, and note the peak reading when a joint lets go - or that
// the TPU stretched to about twice its length without the joint failing.
//
// The strap narrows to 8 mm in the middle, so a joint that is as strong as
// the TPU itself shows up as a long stretch instead of a failure.
//
//   A  butt joint, flat face side by side in each layer (baseline)
//   B  butt joint + Bambu Studio "Use beam interlocking" (object setting)
//   C  lap joint: TPU printed on top of a PETG tongue (layer-on-layer)
//   D  one large dovetail through the full thickness
//   E  three small dovetails (finger joint)
//   F  encapsulated T-anchor: a thin PETG tongue fully wrapped in TPU
//   G  riveted tongue: TPU wraps a PETG tongue and fills two holes in it
//   H  encapsulated T-anchor + beam interlocking (best combination)
//   P  peel strip: TPU printed on a PETG plate, peeled at 90 degrees
//      (the worst case for any bonded joint)

part = "petg";      // [petg, tpu]
type = "A";         // [A, B, C, D, E, F, G, H, P]

grip_l = 28;
gauge_l = 50;
shoulder_l = 14;    // full-width TPU next to each joint before the strap narrows
w = 12;
w_gauge = 8;
t = 3.2;            // 16 layers at 0.2 mm
hole_d = 6;
label_depth = 0.6;

tongue_z = [1.0, 2.2];   // embedded tongues (types F, G, H): 1.0 mm of TPU above and below

$fn = 64;
eps = 0.01;
total_l = 2 * grip_l + gauge_l;

// ---------------------------------------------------------------------
// Outline of the whole coupon (dog-bone)
// ---------------------------------------------------------------------
module outline_2d() {
    translate([0, -w / 2]) square([grip_l + shoulder_l, w]);
    translate([total_l - grip_l - shoulder_l, -w / 2]) square([grip_l + shoulder_l, w]);
    // strap with generous transitions so it does not tear at the shoulders
    hull() for (x = [grip_l + shoulder_l + 5, total_l - grip_l - shoulder_l - 5])
        translate([x, 0]) square([eps, w_gauge], center = true);
    for (s = [0, 1]) {
        x0 = s == 0 ? grip_l + shoulder_l : total_l - grip_l - shoulder_l;
        dir = s == 0 ? 1 : -1;
        hull() {
            translate([x0 - dir * eps, 0]) square([eps, w], center = true);
            translate([x0 + dir * 5, 0]) square([eps, w_gauge], center = true);
        }
    }
}

// ---------------------------------------------------------------------
// PETG grip with the joint feature of each type, for the left end
// (the right end is the same, mirrored)
// ---------------------------------------------------------------------
module dovetail_2d(neck, head, depth) polygon([[0, -neck / 2], [depth, -head / 2], [depth, head / 2], [0, neck / 2]]);

module grip_left_3d() difference() {
    union() {
        translate([0, -w / 2, 0]) cube([grip_l, w, t]);
        if (type == "C")
            translate([grip_l - eps, -w / 2, 0]) cube([10 + eps, w, t / 2]);
        if (type == "D")
            translate([grip_l - eps, 0, 0]) linear_extrude(t) dovetail_2d(4.5, 8.5, 7);
        if (type == "E")
            for (y = [-3.6, 0, 3.6]) translate([grip_l - eps, y, 0]) linear_extrude(t) dovetail_2d(1.8, 3.2, 5);
        if (type == "F" || type == "H")
            translate([0, 0, tongue_z[0]]) linear_extrude(tongue_z[1] - tongue_z[0]) {
                translate([grip_l - eps, -3]) square([8 + eps, 6]);
                translate([grip_l + 8, -5]) square([3, 10]);
            }
        if (type == "G")
            translate([0, 0, tongue_z[0]]) linear_extrude(tongue_z[1] - tongue_z[0]) difference() {
                translate([grip_l - eps, -4]) square([11 + eps, 8]);
                for (x = [grip_l + 3.5, grip_l + 8]) translate([x, 0]) circle(d = 2.4);
            }
    }
    translate([9, 0, -1]) cylinder(d = hole_d, h = t + 2);
    // engraved type letter on top of the grip
    translate([20, 0, t - label_depth]) linear_extrude(label_depth + 1)
        text(type, size = 6, font = "DejaVu Sans:style=Bold", halign = "center", valign = "center");
}

module petg_pull_3d() {
    grip_left_3d();
    translate([total_l, 0, 0]) mirror([1, 0, 0]) grip_left_3d();
}

module tpu_pull_3d() difference() {
    linear_extrude(t) outline_2d();
    // keep the TPU off the grips themselves (they are PETG), and out of the PETG features
    translate([-1, -w, -1]) cube([grip_l + 1, 2 * w, t + 2]);
    translate([total_l - grip_l, -w, -1]) cube([grip_l + 1, 2 * w, t + 2]);
    petg_pull_3d();
}

// ---------------------------------------------------------------------
// Peel coupon (P)
// ---------------------------------------------------------------------
peel_base_l = 50; peel_bond_l = 35; peel_base_t = 2.0; peel_strip_t = 1.6; peel_tail_l = 40;
module petg_peel_3d() difference() {
    translate([0, -w / 2, 0]) cube([peel_base_l, w, peel_base_t]);
    translate([8, 0, -1]) cylinder(d = hole_d, h = 5);
}
module tpu_peel_3d() difference() {
    union() {
        // bonded on top of the PETG plate
        translate([peel_base_l - peel_bond_l, -w / 2, peel_base_t]) cube([peel_bond_l, w, peel_strip_t]);
        // down the end of the plate and along the bed, so the tail lies flat
        translate([peel_base_l, -w / 2, 0]) cube([2, w, peel_base_t + peel_strip_t]);
        translate([peel_base_l, -w / 2, 0]) cube([peel_tail_l, w, peel_strip_t]);
    }
    translate([peel_base_l + peel_tail_l - 7, 0, -1]) cylinder(d = hole_d, h = 5);
}

// ---------------------------------------------------------------------
if (type == "P") { if (part == "petg") petg_peel_3d(); else tpu_peel_3d(); }
else { if (part == "petg") petg_pull_3d(); else tpu_pull_3d(); }
