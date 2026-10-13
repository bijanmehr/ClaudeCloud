// =====================================================================
//  Screwless cam-lever clamp for a cracked wooden cylinder
//
//  One print, no hardware: a PETG C-shaped collar with a TPU spring
//  liner, closed by a PETG cam lever that is printed already hinged.
//  Slide it over the wood, push the lever down until it snaps over
//  centre. Lift the lever to release.
//
//  How it works (plan view, slit at +X, side A above the slit):
//    - the collar is a C with a slit; it is printed OPEN (wood slides in)
//    - an "arm" grows from the B side of the slit, reaches over ear A
//      and carries the lever pivot
//    - the lever's eccentric cam pushes ear A towards B, closing the
//      slit; at the end of its travel it passes dead centre and locks
//    - closing the slit shrinks the collar so the TPU fins squeeze the
//      wood by `squeeze`
//
//  Printed upright (Z = cylinder axis) on a dual-nozzle printer
//  (PETG + TPU, e.g. Bambu Lab H2D: TPU must use the right nozzle).
//  All loads act in the XY plane, i.e. along the layers (strong).
//  Units: mm.
// =====================================================================

/* [What to show / export] */
// print = as printed (open); closed = clamped on the wood; open = open on the wood
part = "closed"; // [print, closed, open, petg, tpu]

/* [Your wood] */
// Measured diameter of the wooden cylinder (calipers, average of a few readings)
wood_d = 38.0;
// Height of the wooden cylinder (preview only)
wood_h = 70.0;

/* [Clamp] */
// Height of the clamp (70 = full length of the cylinder)
clamp_h = 70.0;
// Radial squeeze on the wood when the lever is closed
squeeze = 0.45;
// Extra gap between fin tips and wood when open (0 = fins just touch)
fit_gap = 0.0;

/* [TPU spring liner] */
liner_base_t = 1.0;   // TPU skin bonded to the PETG
fin_h = 1.9;          // radial height of the spring fins
fin_t = 1.0;          // fin thickness (thinner = softer)
fin_lean = 30;        // fin lean from radial (deg), towards the back of the C
fin_pitch = 3.2;      // fin spacing along the wood surface
fin_end_taper = 2.5;  // fins fade out over this height at top and bottom
liner_end_gap = 0.6;  // liner stops short of the slit by this much

/* [PETG collar] */
shell_wall = 4.0;
// Slit width at the outer surface when closed
slit_gap = 1.2;

/* [Cam lever] */
earA_w = 6.0;         // thickness of the ear the cam pushes on
earB_w = 6.0;         // thickness of the arm root on the other side
arm_t = 5.0;          // thickness of the arm that reaches over ear A
pin_d = 6.0;          // hinge pin (printed in place)
cam_wall = 1.6;       // cam wall around the pin hole
over_centre = 10;     // degrees past dead centre when closed (the lock)
open_angle = 135;     // lever travel from closed to fully open (as printed)
cam_slack = 0.5;      // cam-to-ear gap when open (also a print clearance)
handle_th = 4.0;      // handle thickness
handle_end = 128;     // handle tip, degrees around the collar from the slit

/* [Print-in-place clearances] */
xy_gap = 0.45;        // side clearance between moving parts
z_gap = 0.5;          // vertical clearance between stacked knuckles
knuckle_h = 5.5;      // height of each fixed knuckle
cam_bands = 4;        // number of cam layers on the lever

/* [Quality] */
$fn = 128;

/* [Hidden] */
lever_open = 1;       // collar opening 0..1, used by the build checks
lever_theta = 0;

// ---------------------------------------------------------------------
// Derived dimensions (the CLOSED state is the reference geometry)
// ---------------------------------------------------------------------
eps = 0.01;
wood_r = wood_d / 2;
tip_r = wood_r - squeeze;              // fin tips, closed
liner_in_r = tip_r + fin_h;            // inner face of the TPU skin, closed
bore_r = liner_in_r + liner_base_t;    // PETG bore, closed
shell_r = bore_r + shell_wall;         // PETG outer radius, closed
neutral_r = bore_r + shell_wall / 2;
open_dr = squeeze + fit_gap;           // collar radius grows by this when open

// Half-angle of the slit (closed / open). The C keeps its length, so a
// bigger radius means a shorter angular span and a wider slit.
a0 = (slit_gap / 2) / neutral_r * 180 / PI;
a0o = 180 - (180 - a0) * neutral_r / (neutral_r + open_dr);

// Rigid motion of features attached to each end of the C, closed -> open
function rot2(p, a) = [p[0] * cos(a) - p[1] * sin(a), p[0] * sin(a) + p[1] * cos(a)];
function TA(p) = rot2(rot2(p, -a0) + [open_dr, 0], a0o);
function TB(p) = rot2(rot2(p, a0) + [open_dr, 0], -a0o);
module move_A(open) rotate(st_a(open)) translate([st_dr(open), 0]) rotate(-a0) children();
module move_B(open) rotate(-st_a(open)) translate([st_dr(open), 0]) rotate(a0) children();

// Cam: the eccentricity is solved so that, from fully open to dead
// centre, the cam travel equals the slit closure needed for `squeeze`
// plus `cam_slack`.
pin_r = pin_d / 2;
pin_hole_r = pin_r + xy_gap;
function cam_r_of(e) = e + pin_hole_r + cam_wall;
function lobe_of(e) = e + cam_r_of(e);
function P_local(e) =                       // pivot, in ear A's frame (x along the slit face, y away from it)
    let(ly = slit_gap / 2 + earA_w + lobe_of(e),
        absP = shell_r + lobe_of(e) + 1.0 + open_dr)
    [sqrt(absP * absP - ly * ly), ly];
function draw_of(e) =
    let(P = P_local(e), C = [P_local(e)[0], slit_gap / 2 + earA_w], n = [0, -1],
        Po = TB(P), Co = TA(C))
    (C - P) * n - (Co - Po) * n;
cam_travel = 1 - cos(open_angle - over_centre);
function solve_e(e, i) = i == 0 ? e : solve_e((draw_of(e) + cam_slack) / cam_travel, i - 1);
cam_e = solve_e(2.8, 8);
cam_r = cam_r_of(cam_e);
lobe = lobe_of(cam_e);
r_clear = lobe + xy_gap;                    // only the lever may enter this radius around the pivot
P = P_local(cam_e);                         // pivot, closed state (moves with side B)
earA_y = slit_gap / 2 + earA_w;             // ear A's back face (the cam pushes here)
earA_top = P[0] + 1.5;                      // ear A covers the cam contact zone

// Lever angles (deg, side B's frame): lobe direction when closed / as printed
lobe_closed = 270 + over_centre;
lobe_open = lobe_closed - open_angle;

// Z bands: fixed knuckles (arm) and cam layers (lever) alternate
cam_band_h = (clamp_h - (cam_bands + 1) * knuckle_h - 2 * cam_bands * z_gap) / cam_bands;
function knuckle_z(i) = i * (knuckle_h + cam_band_h + 2 * z_gap);
function cam_z(i) = knuckle_z(i) + knuckle_h + z_gap;

echo(str("cam_e=", cam_e, " cam_r=", cam_r, " lobe=", lobe, " |P|=", norm(P), " P=", P));
echo(str("slit closure at cam=", draw_of(cam_e), " cam_band_h=", cam_band_h));

// ---------------------------------------------------------------------
// Collar (PETG C-ring) and TPU spring liner, in either state
// ---------------------------------------------------------------------
// `open` is how open the collar is: 0 = closed on the wood, 1 = as printed
function st_dr(open) = open * open_dr;
function st_a(open) = 180 - (180 - a0) * neutral_r / (neutral_r + st_dr(open));

module half_plane_at(a) rotate(a) translate([-200, 0]) square([400, 200]);

// region of the C: everything except the slit wedge (-a < angle < a, x > 0)
module c_sector(a) {
    difference() {
        circle(r = 200);
        polygon([[0, 0], [200 * cos(a), 200 * sin(a)], [200, 0], [200 * cos(a), -200 * sin(a)]]);
    }
}

// Dovetail key outline (TPU side) at angle k: neck at the bore, wider inside
// the PETG. Starts 1 mm inside the bore so its flanks cross the bore cleanly.
key_neck = 2.2; key_flare = 3.4; key_depth = 1.4;
key_angles = [60, 120, 180, 240, 300];
module key_2d(k, br) {
    w0 = key_neck - (key_flare - key_neck) / key_depth;
    rotate(k) polygon([[br - 1, -w0 / 2], [br + key_depth, -key_flare / 2],
                       [br + key_depth, key_flare / 2], [br - 1, w0 / 2]]);
}

module ring_2d(open) {
    dr = st_dr(open);
    intersection() {
        difference() {
            circle(r = shell_r + dr);
            circle(r = bore_r + dr);
            for (k = key_angles) key_2d(k, bore_r + dr);
        }
        c_sector(st_a(open));
    }
}

// Fins lean towards the back of the C (180 deg): as the C closes, its
// material slides towards the slit relative to the wood, which folds the
// fins further in their lean direction instead of buckling them.
function fin_step(open) = fin_pitch / (tip_r + st_dr(open)) * 180 / PI;
function fin_angles(open) =
    let(step = fin_step(open),
        a_min = st_a(open) + (liner_end_gap + 3) / (tip_r + st_dr(open)) * 180 / PI,
        n = floor((180 - a_min) / step) - 1)
    [for (i = [1 : n]) 180 - i * step, for (i = [1 : n]) -(180 - i * step)];

module fin_2d(a, lin_r, tip_rr) {
    s = a >= 0 ? 1 : -1;
    lean = s * fin_lean;
    len = lin_r * cos(fin_lean) - sqrt(pow(lin_r * cos(fin_lean), 2) - pow(lin_r, 2) + pow(tip_rr, 2));
    root = [lin_r + 0.3, 0];
    dir = [-cos(lean), sin(lean)];
    rotate(a) hull() {
        translate(root) circle(d = fin_t, $fn = 24);
        translate(root + dir * (len + 0.3 - fin_t / 2)) circle(d = fin_t, $fn = 24);
    }
}

module liner_2d(open) {
    dr = st_dr(open);
    ae = st_a(open) + liner_end_gap / (bore_r + dr) * 180 / PI;
    intersection() {
        union() {
            difference() { circle(r = bore_r + dr); circle(r = liner_in_r + dr); }
            for (k = key_angles) key_2d(k, bore_r + dr);
            for (a = fin_angles(open)) fin_2d(a, liner_in_r + dr, tip_r + dr);
        }
        c_sector(ae);
    }
}

module liner_3d(open) {
    dr = st_dr(open);
    // fins fade out at both ends so the pressure ramps in instead of stepping
    intersection() {
        linear_extrude(clamp_h) liner_2d(open);
        union() {
            translate([0, 0, fin_end_taper]) cylinder(r = shell_r + 5, h = clamp_h - 2 * fin_end_taper);
            cylinder(r1 = liner_in_r + dr, r2 = liner_in_r + dr + fin_end_taper * 2, h = fin_end_taper);
            translate([0, 0, clamp_h - fin_end_taper])
                cylinder(r1 = liner_in_r + dr + fin_end_taper * 2, r2 = liner_in_r + dr, h = fin_end_taper);
            difference() { cylinder(r = shell_r + 5, h = clamp_h); translate([0, 0, -1]) cylinder(r = liner_in_r + dr - eps, h = clamp_h + 2); }
        }
    }
}

// ---------------------------------------------------------------------
// Mechanism outlines, CLOSED state, plan view
// ---------------------------------------------------------------------
function dir2(a) = [cos(a), sin(a)];
cross_ri = norm([earA_top, earA_y]) + 0.8;     // arm passes this far out over ear A
cross_ro = cross_ri + arm_t;
cross_end = atan2(P[1], P[0]) - asin((r_clear + 0.2) / ((cross_ri + cross_ro) / 2)) - 2;

// Ear A: the block the cam pushes on (side A of the slit)
module earA_2d() {
    intersection() {
        translate([bore_r + 1, 0]) square([earA_top - bore_r - 1, earA_y]);
        c_sector(a0);
    }
}

// Side B: arm root, the arm that reaches over ear A, and (in knuckle
// layers) the web out to the hinge knuckle
module arm_2d() {
    intersection() {
        translate([bore_r + 1, -(slit_gap / 2 + earB_w)]) square([cross_ro - bore_r - 1, slit_gap / 2 + earB_w]);
        c_sector(a0);
    }
    // the part that bridges the slit, clear of ear A
    difference() {
        circle(r = cross_ro);
        circle(r = cross_ri);
        rotate(cross_end) translate([-100, 0]) square([200, 100]);
        rotate(-(slit_gap / 2 + earB_w) / cross_ri * 180 / PI) mirror([0, 1]) translate([-100, 0]) square([200, 100]);
    }
}

// Hinge knuckle (fixed, part of side B): same footprint as the cam as
// printed, so each cam layer prints on top of a knuckle and vice versa
knuckle_c = P + cam_e * dir2(lobe_open);
module knuckle_2d() {
    translate(knuckle_c) circle(r = cam_r);
    // web out to the end of the arm, kept on the outside so it clears the
    // corner of ear A in every collar position
    hull() {
        translate(knuckle_c + (cam_r - 2.5) * dir2(-25)) circle(r = 2.5);
        intersection() {
            difference() { circle(r = cross_ro); circle(r = cross_ri + 2); }
            rotate(cross_end - 4) square([100, 100]);
            rotate(cross_end) mirror([0, 1]) square([100, 100]);
        }
    }
}

module pin_2d() translate(P) circle(r = pin_r);

// ---- lever (closed position; rotated about P for other positions) ----
hr_in = shell_r + 1.2;               // handle inner radius (stop nub closes the gap)
hr_out = hr_in + handle_th;
neck_dir = atan2(P[1], P[0]) + 122;  // from the pivot towards the collar, side A
neck_c = P + (r_clear + handle_th / 2 + 0.3) * dir2(neck_dir);
handle_start = atan2(neck_c[1], neck_c[0]) + 4;

module cam_2d() {
    difference() {
        union() {
            translate(P + cam_e * dir2(lobe_closed)) circle(r = cam_r);
            hull() {   // bridge from the cam to the handle neck
                translate(P + cam_e * dir2(lobe_closed)) circle(r = cam_r * 0.7);
                translate(neck_c) circle(d = handle_th);
            }
        }
        translate(P) circle(r = pin_hole_r);
    }
}

module handle_2d() {
    // neck
    hull() {
        translate(neck_c) circle(d = handle_th);
        rotate(handle_start) translate([(hr_in + hr_out) / 2, 0]) circle(d = handle_th);
    }
    // band hugging the collar
    intersection() {
        difference() { circle(r = hr_out); circle(r = hr_in); }
        rotate(handle_start) square([100, 100]);
        rotate(handle_end) mirror([0, 1]) square([100, 100]);
    }
    // rounded tip with a finger lip
    rotate(handle_end) translate([(hr_in + hr_out) / 2 + 0.8, 0]) circle(d = handle_th + 1.6);
    // stop nub: touches the collar when the cam is just past dead centre
    rotate(handle_end - 10) translate([shell_r + 0.05, -2]) square([hr_in - shell_r, 4]);
}

module lever_at(theta) {
    translate(P) rotate(theta) translate(-P) children();
}

// ---------------------------------------------------------------------
// 3D bodies
// ---------------------------------------------------------------------
module knuckle_layers() for (i = [0 : cam_bands]) translate([0, 0, knuckle_z(i)]) linear_extrude(knuckle_h) children();
module cam_layers() for (i = [0 : cam_bands - 1]) translate([0, 0, cam_z(i)]) linear_extrude(cam_band_h) children();

// fixed PETG: collar, ear A, arm, hinge knuckles and pin
module collar_3d(open) {
    linear_extrude(clamp_h) ring_2d(open);
    move_A(open) linear_extrude(clamp_h) earA_2d();
    move_B(open) {
        linear_extrude(clamp_h) arm_2d();
        knuckle_layers() knuckle_2d();
        linear_extrude(clamp_h) pin_2d();
    }
}

// moving PETG: the lever (handle full height, cam in its own layers)
module lever_3d(open, theta) {
    move_B(open) lever_at(theta) {
        linear_extrude(clamp_h) handle_2d();
        cam_layers() cam_2d();
    }
}

module petg_3d(open, theta) { collar_3d(open); lever_3d(open, theta); }
module tpu_3d(open) liner_3d(open);

module wood_3d() color("BurlyWood") translate([0, 0, (clamp_h - wood_h) / 2]) cylinder(r = wood_r, h = wood_h);

// ---------------------------------------------------------------------
// What to show / export
// ---------------------------------------------------------------------
if (part == "print") { color("SteelBlue") petg_3d(1, -open_angle); color("OrangeRed") tpu_3d(1); }
else if (part == "closed") { wood_3d(); color("SteelBlue") petg_3d(0, 0); color("OrangeRed") tpu_3d(0); }
else if (part == "open") { wood_3d(); color("SteelBlue") petg_3d(1, -open_angle); color("OrangeRed") tpu_3d(1); }
else if (part == "petg") petg_3d(1, -open_angle);
else if (part == "petg_closed") petg_3d(0, 0);
else if (part == "tpu_closed") tpu_3d(0);
else if (part == "tpu") tpu_3d(1);
else if (part == "collar_open") collar_3d(1);
else if (part == "collar_closed") collar_3d(0);
else if (part == "collar_at") collar_3d(lever_open);
else if (part == "tpu_at") tpu_3d(lever_open);
else if (part == "lever") lever_3d(lever_open, lever_theta);
else if (part == "earA_closed") linear_extrude(clamp_h) earA_2d();
else if (part == "bgroup_closed") { linear_extrude(clamp_h) arm_2d(); knuckle_layers() knuckle_2d(); linear_extrude(clamp_h) pin_2d(); }
else if (part == "ring_closed") linear_extrude(clamp_h) ring_2d(0);
else if (part == "ring_open") linear_extrude(clamp_h) ring_2d(1);
echo(str("GEOM ", "{\"P\":[", P[0], ",", P[1], "],\"a0\":", a0, ",\"a0o\":", a0o, ",\"open_dr\":", open_dr,
         ",\"open_angle\":", open_angle, ",\"cam_e\":", cam_e, ",\"cam_r\":", cam_r, ",\"lobe\":", lobe,
         ",\"shell_r\":", shell_r, ",\"tip_r\":", tip_r, ",\"cam_band_h\":", cam_band_h, "}"));
