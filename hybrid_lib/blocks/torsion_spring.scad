include <../params.scad>
// torsion_spring (header written after FEA calibration)

part = "both";   // [petg, tpu, both]
show = "demo";   // [demo, test, section]

ts_fn = 72;      // round parts; divisible by 6 and 8 so hex and 45 deg points are facet vertices

// -----------------------------------------------------------------------
// Defaults (shared by every module, so the parts always agree)
// -----------------------------------------------------------------------
ts_size = 24;      ts_length = 20;    ts_wall = 2.8;
ts_twist = 30;     ts_strain = 0.5;   ts_corner_r = 1.0;  ts_cav_r = 1.5;
ts_relief = 3.0;   ts_core_d = 3.0;
ts_bore = 6.5;     ts_bore_shape = "hex";
ts_drive_h = 4;    ts_lip_t = 1.6;    ts_rim = 1.2;
ts_flange_w = 7;   ts_flange_t = 3;   ts_hole_d = 3.4;

// -----------------------------------------------------------------------
// Geometry (all derived sizes in one place)
// -----------------------------------------------------------------------
// shaft corner-tip radius whose corners, twisted by `twist`, squeeze a cord
// filling the pocket by `strain` (nominal compression under the corner)
function ts_tip_auto(a, twist, strain, corner_r) =
    let(rho = corner_r * (sqrt(2) - 1), s = sin(twist), co = cos(twist))
    (strain * (a - rho) + rho) / (s + co - 1 + strain * (1 - s));
function ts_strain_at(a, c, c_tip, twist) =
    (c_tip * (sin(twist) + cos(twist)) - c) / (a - c + c_tip * sin(twist));

function ts_v(g, k) = g[search([k], g)[0]][1];

function ts_geom(size = ts_size, length = ts_length, wall = ts_wall, shaft = undef, twist = ts_twist,
                 strain = ts_strain, corner_r = ts_corner_r, cav_r = ts_cav_r, relief = ts_relief,
                 core_d = ts_core_d, bore = ts_bore, bore_shape = ts_bore_shape, drive_h = ts_drive_h,
                 lip_t = ts_lip_t, rim = ts_rim) =
    let(a = size / 2 - wall,
        B = is_undef(shaft) ? sqrt(2) * (ts_tip_auto(a, twist, strain, corner_r) + corner_r * (sqrt(2) - 1)) : shaft,
        c = B / sqrt(2),
        c_tip = c - corner_r * (sqrt(2) - 1),
        h = B / 2,
        bore_rc = bore_shape == "hex" ? bore / sqrt(3) : bore_shape == "square" ? bore / sqrt(2) : bore / 2,
        R_n = max(bore_rc + min_petg + 0.3, 2.5),     // neck radius (passes the bottom lip)
        R_f = R_n + xy_clear + min_petg,             // foot radius on the bed (under the lip)
        z_f = R_f - R_n,                             // top of the foot cone
        z_n1 = z_f + rim + 2 * (z_clear - xy_clear), // top of the neck
        z_b1 = z_n1 + c_tip - R_n,                   // full square from here up
        z_lb = z_b1 - z_clear,                       // bottom lip top = cord bottom
        z_lt = length - lip_t,                       // top lip underside = cord top
        z_tpu0 = z_n1 - z_clear + max(h, R_n + xy_clear) - R_n,   // lowest TPU (wedge under the face middle)
        ri = (2 * a - c) * (2 - sqrt(2)) / 2,        // inradius of a corner pocket
        r_core = sqrt(2) * (a - ri),
        D0 = sqrt(2) * a - h - cav_r * (sqrt(2) - 1),
        s1 = h - sqrt(2) * relief)
    [["size", size], ["L", length], ["wall", wall], ["a", a], ["B", B], ["c", c], ["c_tip", c_tip],
     ["h", h], ["R_n", R_n], ["R_f", R_f], ["z_f", z_f], ["z_n1", z_n1], ["z_b1", z_b1], ["z_lb", z_lb],
     ["z_lt", z_lt], ["z_tpu0", z_tpu0], ["Lc", z_lt - z_lb],
     ["corner_r", corner_r], ["cav_r", cav_r], ["relief", relief], ["core_d", core_d], ["ri", ri],
     ["r_core", r_core], ["bore", bore], ["bore_shape", bore_shape], ["bore_rc", bore_rc],
     ["drive_h", drive_h], ["lip_t", lip_t], ["rim", rim], ["twist", twist], ["D0", D0], ["s1", s1],
     ["strain", ts_strain_at(a, c, c_tip, twist)]];

// -----------------------------------------------------------------------
// Estimates (see ESTIMATES in the header)
// -----------------------------------------------------------------------
// initial twist stiffness, N*mm per degree: [total, compression part, drag part]
function ts_k(g, E = 26, G = 9) =
    let(D0 = ts_v(g, "D0"), s1 = ts_v(g, "s1"), h = ts_v(g, "h"), t1 = D0 - s1, lg = ln(D0 / t1),
        Ic = D0 * D0 * lg - D0 * s1 - s1 * s1 / 2,
        Is = 2 * h * h * lg,
        f = ts_core_factor(ts_v(g, "core_d") / (2 * ts_v(g, "ri"))),
        kc = 4 * ts_v(g, "Lc") * (4 / 3) * E * Ic * f * PI / 180,
        ks = 4 * ts_v(g, "Lc") * G * Is * f * PI / 180)
    [kc + ks, kc, ks];
// hollow cords: FEA factor on the solid-cord stiffness vs core_d / (2 * ri)
ts_core_table = [[0, 1], [1, 1]];   // placeholder
function ts_core_factor(q) = q <= 0 ? 1 : lookup(q, ts_core_table);
// torque-per-degree multiplier (secant) vs twist angle, from FEA: progressive
ts_prog_table = [[0, 1], [40, 1]];  // placeholder
function ts_torque(g, deg) = ts_k(g)[0] * deg * lookup(abs(deg), ts_prog_table);

// public helpers
function torsion_spring_geom(size = ts_size, length = ts_length, wall = ts_wall, shaft = undef,
                             relief = ts_relief, core_d = ts_core_d, bore = ts_bore, bore_shape = ts_bore_shape) =
    ts_geom(size, length, wall, shaft, relief = relief, core_d = core_d, bore = bore, bore_shape = bore_shape);
function torsion_spring_k(size = ts_size, length = ts_length, wall = ts_wall, shaft = undef,
                          relief = ts_relief, core_d = ts_core_d) =
    ts_k(ts_geom(size, length, wall, shaft, relief = relief, core_d = core_d))[0];
function torsion_spring_torque(deg, size = ts_size, length = ts_length, wall = ts_wall, shaft = undef,
                               relief = ts_relief, core_d = ts_core_d) =
    ts_torque(ts_geom(size, length, wall, shaft, relief = relief, core_d = core_d), deg);

module ts_checks(g) {
    a = ts_v(g, "a"); h = ts_v(g, "h"); c_tip = ts_v(g, "c_tip");
    assert(ts_v(g, "wall") >= min_petg, "torsion_spring: wall must be >= min_petg (1.2)");
    assert(c_tip + xy_clear <= a, "torsion_spring: shaft too big: its corners must clear the tube by xy_clear");
    assert(ts_v(g, "rim") >= min_petg, "torsion_spring: rim must be >= min_petg");
    assert(ts_v(g, "lip_t") >= 3 * layer_h, "torsion_spring: lip_t must be >= 3 layers");
    assert(ts_v(g, "relief") >= 0.4, "torsion_spring: relief must be >= 0.4");
    assert(ts_v(g, "s1") >= 0.5, "torsion_spring: relief too big: the cords must touch the shaft faces (s1 >= 0.5)");
    assert(a - ts_v(g, "c") + ts_v(g, "relief") >= min_tpu, "torsion_spring: cord tips thinner than min_tpu");
    assert(ts_v(g, "core_d") == 0 || ts_v(g, "core_d") >= 1.6, "torsion_spring: core_d must be 0 or >= 1.6");
    assert(ts_v(g, "ri") - ts_v(g, "core_d") / 2 >= min_tpu, "torsion_spring: core_d too big: TPU round it < min_tpu");
    assert(ts_v(g, "core_d") == 0 || (a - ts_v(g, "ri")) - ts_v(g, "relief") - ts_v(g, "core_d") / 2 >= min_tpu,
           "torsion_spring: core_d + relief too big: TPU between core and cord tip < min_tpu");
    assert(ts_v(g, "bore") >= 1.6, "torsion_spring: bore must be >= 1.6");
    assert(h - ts_v(g, "bore_rc") >= min_petg, "torsion_spring: bore too big for the shaft (wall < 1.2)");
    assert(ts_v(g, "R_n") + xy_clear + 0.6 <= c_tip, "torsion_spring: bore too big for the shaft neck");
    assert(ts_v(g, "Lc") >= 4, str("torsion_spring: length too short, needs >= ", ts_v(g, "z_lb") + ts_v(g, "lip_t") + 4));
}

// -----------------------------------------------------------------------
// 2D sections
// -----------------------------------------------------------------------
module ts_rsq(side, r) offset(r = r, $fn = 32) square(side - 2 * r, center = true);
module ts_cavity_2d(g) ts_rsq(2 * ts_v(g, "a"), ts_v(g, "cav_r"));
module ts_shaft_2d(g, grow = 0) rotate(45) ts_rsq(ts_v(g, "B") + 2 * grow, ts_v(g, "corner_r") + grow);
module ts_relief_2d(g) let(e = ts_v(g, "relief"), w = 2 * ts_v(g, "a") + 2) {
    square([w, 2 * e], center = true);
    square([2 * e, w], center = true);
}
module ts_cores_2d(g) if (ts_v(g, "core_d") > 0)
    for (q = [0 : 3]) rotate(45 + 90 * q) translate([ts_v(g, "r_core"), 0]) circle(d = ts_v(g, "core_d"), $fn = 32);
module ts_cords_2d(g) difference() {
    ts_cavity_2d(g);
    ts_shaft_2d(g);
    ts_relief_2d(g);
    ts_cores_2d(g);
}
module ts_bore_2d(g) let(b = ts_v(g, "bore"), s = ts_v(g, "bore_shape")) {
    if (s == "hex") circle(r = b / sqrt(3), $fn = 6);
    else if (s == "square") rotate(45) square(b, center = true);
    else circle(d = b, $fn = 32);
}
// ribs: a short PETG web from the middle of each tube wall towards the
// shaft corner (outside the circle the corners sweep). They split the air
// strip between two cords, so the top lip bridges at most relief - 0.6.
function ts_has_ribs(g) = ts_v(g, "relief") >= 1.5 && ts_v(g, "a") - ts_v(g, "c_tip") - xy_clear >= 0.6;
module ts_ribs_2d(g) let(a = ts_v(g, "a"), r0 = ts_v(g, "c_tip") + xy_clear)
    if (ts_has_ribs(g)) for (q = [0 : 3]) rotate(90 * q) translate([r0, -min_petg / 2]) square([a - r0 + 0.5, min_petg]);

// -----------------------------------------------------------------------
// 3D bodies
// -----------------------------------------------------------------------
// Bottom lip: a solid of revolution (so its clearance to the turning shaft
// is the same at every angle) with a waist that the shaft's foot and its
// upper cone hook under and over. Profile in (r, z):
function ts_lip_profile(g) =
    let(R_n = ts_v(g, "R_n"), R_f = ts_v(g, "R_f"), Rb = sqrt(2) * ts_v(g, "a") + 2)
    [[R_f + z_clear + 1, -1], [Rb, -1], [Rb, ts_v(g, "z_lb")], [ts_v(g, "c_tip"), ts_v(g, "z_lb")],
     [R_n + xy_clear, ts_v(g, "z_n1") - (z_clear - xy_clear)], [R_n + xy_clear, ts_v(g, "z_f") + (z_clear - xy_clear)]];

// outer tube: walls, ribs, bottom lip (waist), top lip (flat ring over the cords)
module ts_tube_3d(g) {
    L = ts_v(g, "L");
    difference() {
        linear_extrude(L) ts_rsq(ts_v(g, "size"), 1.0);
        difference() {
            translate([0, 0, -1]) linear_extrude(L + 2) difference() { ts_cavity_2d(g); ts_ribs_2d(g); }
            rotate_extrude($fn = ts_fn) polygon(ts_lip_profile(g));
            translate([0, 0, ts_v(g, "z_lt")]) linear_extrude(L) difference() {
                square(2 * ts_v(g, "size"), center = true);
                circle(r = ts_v(g, "c_tip") + xy_clear, $fn = ts_fn);
            }
        }
    }
}

// inner shaft: foot cone - neck - 45 deg cone - square, out through the top
// lip as a square drive; bore all through
module ts_shaft_3d(g) {
    L = ts_v(g, "L"); R_n = ts_v(g, "R_n"); top = L + ts_v(g, "drive_h");
    difference() {
        union() {
            cylinder(r1 = ts_v(g, "R_f"), r2 = R_n, h = ts_v(g, "z_f") + eps, $fn = ts_fn);
            translate([0, 0, ts_v(g, "z_f")]) cylinder(r = R_n, h = ts_v(g, "z_n1") - ts_v(g, "z_f") + eps, $fn = ts_fn);
            hull() {
                translate([0, 0, ts_v(g, "z_n1")]) cylinder(r = R_n, h = eps, $fn = ts_fn);
                translate([0, 0, ts_v(g, "z_b1") - eps]) linear_extrude(eps) ts_shaft_2d(g);
            }
            translate([0, 0, ts_v(g, "z_b1") - eps]) linear_extrude(top - ts_v(g, "z_b1") + eps) ts_shaft_2d(g);
        }
        translate([0, 0, -1]) linear_extrude(top + 2) ts_bore_2d(g);
    }
}

// TPU region (the PETG is taken out of it by hybrid())
module ts_tpu_region_3d(g)
    translate([0, 0, ts_v(g, "z_tpu0")]) linear_extrude(ts_v(g, "z_lt") - ts_v(g, "z_tpu0")) ts_cords_2d(g);

module ts_echo(g) {
    k = ts_k(g);
    echo(str("torsion_spring: size ", ts_v(g, "size"), " x ", ts_v(g, "L"), ", shaft ", round(ts_v(g, "B") * 100) / 100,
             " (h ", round(ts_v(g, "h") * 100) / 100, ", corner tip r ", round(ts_v(g, "c_tip") * 100) / 100,
             "), cord length ", round(ts_v(g, "Lc") * 100) / 100, ", relief ", ts_v(g, "relief"), ", core_d ", ts_v(g, "core_d"),
             ", k0 ~ ", round(k[0]), " N*mm/deg (compression ", round(k[1]), " + drag ", round(k[2]), ")",
             ", torque at 10/20/30 deg ~ ", round(ts_torque(g, 10)) / 1000, " / ", round(ts_torque(g, 20)) / 1000,
             " / ", round(ts_torque(g, 30)) / 1000, " N*m"));
}

// -----------------------------------------------------------------------
// Public modules
// -----------------------------------------------------------------------
// The unit alone (tube, shaft, cords); children() = your own PETG fused to
// the tube's outside.
module torsion_spring_unit(part = "both", size = ts_size, length = ts_length, wall = ts_wall, shaft = undef,
                           twist = ts_twist, strain = ts_strain, corner_r = ts_corner_r, cav_r = ts_cav_r,
                           relief = ts_relief, core_d = ts_core_d, bore = ts_bore, bore_shape = ts_bore_shape,
                           drive_h = ts_drive_h, lip_t = ts_lip_t, rim = ts_rim) {
    g = ts_geom(size, length, wall, shaft, twist, strain, corner_r, cav_r, relief, core_d,
                bore, bore_shape, drive_h, lip_t, rim);
    ts_checks(g);
    ts_echo(g);
    if (part == "both") {
        color(petg_colour) render() ts_hybrid("petg") children();
        color(tpu_colour) render() ts_hybrid("tpu") children();
    } else ts_hybrid(part) children();
    module ts_hybrid(p) hybrid(p) {
        union() { ts_tube_3d(g); ts_shaft_3d(g); children(); }
        ts_tpu_region_3d(g);
    }
}

// The unit with a square mounting flange on the bed (4 holes)
module torsion_spring(part = "both", size = ts_size, length = ts_length, wall = ts_wall, shaft = undef,
                      twist = ts_twist, strain = ts_strain, corner_r = ts_corner_r, cav_r = ts_cav_r,
                      relief = ts_relief, core_d = ts_core_d, bore = ts_bore, bore_shape = ts_bore_shape,
                      drive_h = ts_drive_h, lip_t = ts_lip_t, rim = ts_rim,
                      flange_w = ts_flange_w, flange_t = ts_flange_t, hole_d = ts_hole_d) {
    S = size + 2 * flange_w;
    hp = size / 2 + flange_w / 2;
    assert(flange_w == 0 || flange_w >= hole_d + 2 * min_petg, "torsion_spring: flange_w too small for the holes");
    assert(hole_d >= 1.6, "torsion_spring: hole_d must be >= 1.6");
    torsion_spring_unit(part, size, length, wall, shaft, twist, strain, corner_r, cav_r, relief, core_d,
                        bore, bore_shape, drive_h, lip_t, rim) {
        if (flange_w > 0) difference() {
            linear_extrude(flange_t) ts_rsq(S, 3);
            translate([0, 0, -1]) linear_extrude(flange_t + 2) ts_rsq(size - 0.2, 1);
            for (x = [-hp, hp], y = [-hp, hp]) translate([x, y, -1]) cylinder(d = hole_d, h = flange_t + 2, $fn = 24);
        }
        children();
    }
}

// -----------------------------------------------------------------------
// Test piece: the unit + an angle scale on the flange + a loose lever
// -----------------------------------------------------------------------
ts_scale_r = 64;      // outer radius of the angle scale
ts_lever_r = 50;      // load hole radius on the lever (torque = force x 50 mm)

// lever, printed flat with its square pocket opening upwards (turn it over
// to fit it on the square drive); local frame: pocket on the origin, arm +x
module ts_lever_3d(g) {
    depth = ts_v(g, "drive_h") - 1;    // sits on the drive, 1 mm clear of the tube top
    t = depth + 2.4;
    hub = ts_v(g, "c_tip") + 4.5;
    difference() {
        union() {
            cylinder(r = hub, h = t, $fn = ts_fn);
            hull() {
                translate([hub - 2, -6, 0]) cube([1, 12, t]);
                translate([ts_lever_r + 4, -4, 0]) cube([1, 8, t]);
            }
            linear_extrude(t) polygon([[ts_lever_r + 4, -4], [ts_scale_r - 1, -0.5], [ts_scale_r - 1, 0.5], [ts_lever_r + 4, 4]]);
        }
        translate([0, 0, t - depth]) linear_extrude(depth + 1) ts_shaft_2d(g, 0.2);
        translate([0, 0, -1]) linear_extrude(t + 2) ts_bore_2d(g);       // e.g. a 1/4" hex bit through lever and shaft
        for (x = [30, ts_lever_r]) translate([x, 0, -1]) cylinder(d = 3.4, h = t + 2, $fn = 24);
        // engraved underneath, mirrored: reads right way round once the lever is turned over
        translate([40, 0, -1]) mirror([0, 1, 0]) linear_extrude(1.6)
            text("30   50", size = 3, font = "DejaVu Sans:style=Bold", halign = "center", valign = "center");
    }
}

// flat angle scale (+-40 deg about +x) fused to the flange, ticks engraved
module ts_scale_3d(r0, t) difference() {
    linear_extrude(t) intersection() {
        difference() { circle(r = ts_scale_r, $fn = 180); circle(r = r0, $fn = 90); }
        polygon([[0, 0], [100 * cos(43), -100 * sin(43)], [100, -100], [100, 100], [100 * cos(43), 100 * sin(43)]]);
    }
    for (d = [-40 : 5 : 40]) rotate(d) translate([ts_scale_r - (d % 10 == 0 ? 7 : 4), -0.4, t - 0.6])
        cube([10, 0.8, 1]);
    for (d = [-40 : 10 : 40]) rotate(d) translate([ts_scale_r - 11, 0, t - 0.6]) rotate(-90) linear_extrude(1)
        text(str(abs(d)), size = 3, font = "DejaVu Sans:style=Bold", halign = "center", valign = "center");
}

module torsion_spring_test(part = "both", size = ts_size, length = ts_length, wall = ts_wall, shaft = undef,
                           relief = ts_relief, core_d = ts_core_d, drive_h = ts_drive_h,
                           flange_w = ts_flange_w, flange_t = ts_flange_t) {
    g = ts_geom(size, length, wall, shaft, relief = relief, core_d = core_d, drive_h = drive_h);
    assert(drive_h >= 3, "torsion_spring_test: needs drive_h >= 3");
    torsion_spring(part, size = size, length = length, wall = wall, shaft = shaft, relief = relief,
                   core_d = core_d, drive_h = drive_h, flange_w = flange_w, flange_t = flange_t)
        ts_scale_3d(size / 2 + flange_w - 2, flange_t);
    if (part != "tpu") color(petg_colour) translate([-10, -56, 0]) ts_lever_3d(g);
}

// cut-away preview (not a print): the front half removed along a diagonal
module ts_section(part = "both") {
    for (p = ["petg", "tpu"]) if (part == "both" || part == p)
        color(p == "petg" ? petg_colour : tpu_colour) render() difference() {
            torsion_spring(p);
            translate([0, 0, -1]) rotate(-45) translate([0, -50, 0]) cube([100, 100, 50]);
        }
}

// slice at mid-height (preview only, view from the top)
module ts_slice(part = "both", z = 10) {
    for (p = ["petg", "tpu"]) if (part == "both" || part == p)
        color(p == "petg" ? petg_colour : tpu_colour) render() intersection() {
            torsion_spring(p, flange_w = 0, drive_h = 0);
            translate([-50, -50, z]) cube([100, 100, 1]);
        }
}

if (show == "demo") torsion_spring(part);
else if (show == "test") torsion_spring_test(part);
else if (show == "section") ts_section(part);
else if (show == "slice") ts_slice(part);
