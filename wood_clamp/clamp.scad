// =====================================================================
//  TPU-lined split clamp for a cracked wooden cylinder
//  PETG shell (rigid, carries the bolt load) + TPU spring liner
//  (spreads the pressure evenly, keeps it constant as the wood moves)
//
//  Two identical halves. Print 2x "half" (PETG + TPU co-printed on a
//  dual-nozzle printer such as the Bambu Lab H2D), join with M3 bolts.
//  Tighten until the split faces touch: that positive stop sets the
//  designed liner squeeze, so you cannot over-crush the wood.
//
//  Units: mm. Coordinates: cylinder axis = Z, split plane = YZ (x = 0).
//  Each half is modelled at x >= 0 in its CLOSED (installed) position,
//  standing upright, so the same coordinates are the print orientation.
// =====================================================================

/* [What to export] */
// half_assembly = PETG + TPU together (preview); shell = PETG only; liner = TPU only; assembly = both halves around the wood; exploded = both halves pulled apart
part = "assembly"; // [assembly, exploded, half_assembly, shell, liner, wood, section]

/* [Your wood] */
// Measured diameter of the wooden cylinder (use calipers; average of a few readings)
wood_d = 38.0;
// Height of the wooden cylinder (only used for the preview)
wood_h = 70.0;

/* [Clamp size] */
// Height of the clamp (70 = full length of the cylinder)
clamp_h = 70.0;
// Bolts per side (2 sides)
bolts_per_side = 3;

/* [TPU spring liner] */
// TPU skin bonded to the PETG shell
liner_base_t = 1.0;
// Radial height of the spring fins before squeeze
fin_h = 1.9;
// Fin thickness (1.0 = two ~0.5 mm lines). Thinner = softer.
fin_t = 1.0;
// Fin lean from radial, towards the middle of each half (deg)
fin_lean = 30;
// Fin spacing measured along the wood surface
fin_pitch = 3.2;
// Designed radial squeeze of the fins when the halves touch
squeeze = 0.8;
// Chamfer that softens the pressure at the top/bottom edge of the liner
fin_end_taper = 2.5;

/* [PETG shell] */
shell_wall = 4.0;
// Gap kept between the TPU liners of the two halves at the split
liner_split_gap = 0.6;
// Number of dovetail keys locking the TPU liner into each PETG half
keys_per_half = 3;

/* [Hardware: M3 socket head cap screws + hex nuts] */
bolt_d = 3.4;          // clearance hole
head_d = 6.0;          // counterbore for M3 SHCS head (5.5 mm)
head_depth = 3.5;
nut_af = 5.75;         // across flats for M3 nut (5.5 mm) + print clearance
nut_depth = 3.0;
ear_depth = 7.5;       // ear thickness from split face to outer face

/* [Quality] */
$fn = 160;

// ---------------------------------------------------------------------
// Derived dimensions
// ---------------------------------------------------------------------
wood_r   = wood_d / 2;
fin_tip_r = wood_r - squeeze;                 // fin tips before squeeze
liner_in_r = fin_tip_r + fin_h;               // inner face of TPU skin
bore_r   = liner_in_r + liner_base_t;         // PETG bore = TPU outer face
shell_r  = bore_r + shell_wall;               // PETG outer radius
bolt_y   = shell_r + 0.9;                     // bolt axis distance from centre
ear_y_out = bolt_y + 4.2;                     // outer edge of the ear
eps = 0.01;

function bolt_z(i) = clamp_h * (i + 0.5) / bolts_per_side;

// fin length so the tip of a leaning fin lands on fin_tip_r
fin_len = liner_in_r * cos(fin_lean)
        - sqrt(pow(liner_in_r * cos(fin_lean), 2) - pow(liner_in_r, 2) + pow(fin_tip_r, 2));

// ---------------------------------------------------------------------
// 2D helpers
// ---------------------------------------------------------------------
module half_plane(xmin = 0) {
    translate([xmin, -200]) square([400, 400]);
}

// Dovetail key outline (TPU side), centred on angle a: neck width at the
// bore, wider at the bottom of the groove. Starts 1 mm inside the bore so its
// flanks cross the bore circle cleanly.
key_neck = 2.2;
key_flare = 3.4;
key_depth = 1.4;
module key_2d(a) {
    w0 = key_neck - (key_flare - key_neck) / key_depth;   // width 1 mm inside the bore
    rotate(a)
        polygon([[bore_r - 1, -w0 / 2], [bore_r + key_depth, -key_flare / 2],
                 [bore_r + key_depth, key_flare / 2], [bore_r - 1, w0 / 2]]);
}

function key_angles() =
    keys_per_half == 1 ? [0] :
    [for (i = [0 : keys_per_half - 1]) -55 + 110 * i / (keys_per_half - 1)];

// one bolt ear (s = +1 / -1 side), with a fillet down onto the shell
module ear_2d(s) {
    r = 2.0;
    fx = ear_depth + 4.5;
    mirror([0, s > 0 ? 0 : 1])
        hull() {
            translate([0, bore_r + 0.5]) square([eps, ear_y_out - bore_r - 0.5]);
            translate([ear_depth - r, ear_y_out - r]) circle(r = r, $fn = 40);
            translate([ear_depth - eps, bore_r + 0.5]) square([eps, eps]);
            translate([fx, sqrt(pow(shell_r, 2) - pow(fx, 2)) - 0.5]) circle(r = 0.5, $fn = 16);
        }
}

module shell_2d() {
    difference() {
        intersection() {
            union() {
                circle(r = shell_r);
                // bolt ears, both sides of the split
                for (s = [-1, 1]) ear_2d(s);
            }
            half_plane(0);
        }
        circle(r = bore_r);
        for (a = key_angles()) key_2d(a);
    }
}

// one fin, rooted on the TPU skin at polar angle a, leaning towards the half's centre (a = 0)
module fin_2d(a) {
    s = a >= 0 ? 1 : -1;
    lean = s * fin_lean;
    root = [liner_in_r + 0.3, 0];
    dir = [-cos(lean), -sin(lean)];  // inward, tipped back towards a = 0
    rotate(a)
        hull() {
            translate(root) circle(d = fin_t, $fn = 24);
            translate(root + dir * (fin_len + 0.3 - fin_t / 2)) circle(d = fin_t, $fn = 24);
        }
}

// Fins lean towards the middle of each half (a = 0) so that, as the halves
// close, the wood slides over them in their lean direction and they fold
// smoothly. The first fin each side starts one pitch from the middle so the
// two innermost fins cannot clash when they fold.
function fin_angles() =
    let(step = fin_pitch / fin_tip_r * 180 / PI,
        a_max = 90 - asin((liner_split_gap / 2 + 2.0) / liner_in_r) - 4,
        n = floor(a_max / step))
    [for (i = [1 : n]) i * step, for (i = [1 : n]) -i * step];

module liner_2d() {
    intersection() {
        union() {
            difference() {
                circle(r = bore_r);
                circle(r = liner_in_r);
            }
            for (a = key_angles()) key_2d(a);
            for (a = fin_angles()) fin_2d(a);
        }
        half_plane(liner_split_gap / 2);
    }
}

// ---------------------------------------------------------------------
// 3D parts
// ---------------------------------------------------------------------
// horizontal hole along +X with a 45 deg pointed roof (+Z) so it prints without support
module tear_x(d, len) {
    rotate([0, 90, 0]) linear_extrude(len) hull() {
        circle(d = d);
        translate([-d / 2 * sqrt(2), 0]) square(eps, center = true);
    }
}

module shell() {
    difference() {
        linear_extrude(clamp_h) shell_2d();
        for (i = [0 : bolts_per_side - 1]) {
            z = bolt_z(i);
            // through hole, both ears
            for (s = [-1, 1])
                translate([-1, s * bolt_y, z]) tear_x(bolt_d, ear_depth + 2, $fn = 40);
            // +Y ear: hex nut pocket opening on the outer face (corner up = self-supporting roof)
            translate([ear_depth - nut_depth, bolt_y, z]) rotate([0, 90, 0])
                cylinder(d = nut_af / cos(30), h = nut_depth + 1, $fn = 6);
            // -Y ear: counterbore for the screw head
            translate([ear_depth - head_depth, -bolt_y, z]) tear_x(head_d, head_depth + 1, $fn = 48);
        }
    }
}

module liner() {
    // taper the fins near both ends so the pressure fades out instead of stepping
    intersection() {
        linear_extrude(clamp_h) liner_2d();
        union() {
            translate([0, 0, fin_end_taper]) cylinder(r = bore_r + 5, h = clamp_h - 2 * fin_end_taper);
            cylinder(r1 = liner_in_r, r2 = liner_in_r + fin_end_taper * 2, h = fin_end_taper);
            translate([0, 0, clamp_h - fin_end_taper])
                cylinder(r1 = liner_in_r + fin_end_taper * 2, r2 = liner_in_r, h = fin_end_taper);
            difference() { cylinder(r = bore_r + 5, h = clamp_h); cylinder(r = liner_in_r - eps, h = clamp_h); }
        }
    }
}

module wood() {
    color("BurlyWood") translate([0, 0, (clamp_h - wood_h) / 2]) cylinder(r = wood_r, h = wood_h);
}

module half_assembly() {
    color("SteelBlue") shell();
    color("OrangeRed") liner();
}

if (part == "shell") shell();
else if (part == "liner") liner();
else if (part == "half_assembly") half_assembly();
else if (part == "wood") wood();
else if (part == "assembly") { wood(); half_assembly(); rotate(180) half_assembly(); }
else if (part == "exploded") { wood(); translate([22, 0, 0]) half_assembly(); rotate(180) translate([22, 0, 0]) half_assembly(); }
else if (part == "section") projection(cut = true) translate([0, 0, -bolt_z(0)]) { half_assembly(); rotate(180) half_assembly(); }
