include <../params.scad>
// =======================================================================
// bump_stop: TPU bump stops, landing feet and bumpers held in PETG cups
// =======================================================================
// (header: see the bottom of this comment block for parameters, estimates,
//  uses and limitations)
//SECTION_PLACEHOLDER
// =======================================================================

part = "both";   // [petg, tpu, both]
show = "demo";   // [demo, test, foot]

bs_fn = 48;      // facets on every round outline (same count everywhere, so
                 // the offsets line up vertex for vertex)

// -----------------------------------------------------------------------
// Small helpers
// -----------------------------------------------------------------------
// Outline in the XY plane: a stadium l long (x) and w wide (y), grown by g.
// l == w gives a circle of diameter w + 2 g.
module _bs_o2d(l, w, g = 0) {
    r = w / 2 + g;
    if (l - w < 1e-6) circle(r = r, $fn = bs_fn);
    else hull() for (s = [-1, 1]) translate([s * (l - w) / 2, 0]) circle(r = r, $fn = bs_fn);
}

// flat pad h tall over the outline, 45 deg chamfer c on its top edge
module _bs_flat(l, w, h, c) {
    cc = min(c, h / 2, w / 2 - 1);
    if (cc > 0.05) hull() {
        linear_extrude(h - cc) _bs_o2d(l, w);
        linear_extrude(h) _bs_o2d(l, w, -cc);
    }
    else linear_extrude(h) _bs_o2d(l, w);
}

// rounded pad h tall: straight sides, then a spherical cap t high
// (stadium: the cap is swept along the straight part)
module _bs_dome(l, w, h, t) {
    a = w / 2;
    tt = max(min(t, a, h), 0.4);
    R = (a * a + tt * tt) / (2 * tt);
    hull() for (s = [-1, 1]) translate([s * (l - w) / 2, 0, 0]) {
        cylinder(r = a, h = max(h - tt, 2 * eps), $fn = bs_fn);
        intersection() {
            translate([0, 0, h - R]) sphere(r = R, $fn = bs_fn);
            translate([0, 0, h - tt]) cylinder(r = a + 1, h = tt, $fn = bs_fn);
        }
    }
}

// rivet points: n on a circle of radius r (n = 1: the centre)
function _bs_ring(n, r) = n <= 0 ? [] : n == 1 ? [[0, 0]] :
    [for (i = [0 : n - 1]) [r * cos(90 + 360 * i / n), r * sin(90 + 360 * i / n)]];
// rivet points along the strip axis at `pitch`, as many as fit in the pocket
function _bs_row(l, w, undercut, hd, pitch) =
    pitch <= 0 ? [] :
    let(xm = l / 2 + undercut - hd / 2 - 0.4, n = floor(2 * xm / pitch + 1e-6) + 1)
    [for (i = [0 : n - 1]) [(i - (n - 1) / 2) * pitch, 0]];
// distance from p to the stadium's axis segment
function _bs_axd(p, l, w) = norm([max(abs(p[0]) - (l - w) / 2, 0), p[1]]);
// smallest distance between any two points
function _bs_mind(pts) = len(pts) < 2 ? 1e9 :
    min([for (i = [0 : len(pts) - 2]) for (j = [i + 1 : len(pts) - 1]) norm(pts[i] - pts[j])]);
function _bs_cum(v, i) = i <= 0 ? 0 : _bs_cum(v, i - 1) + v[i - 1];

// -----------------------------------------------------------------------
// Public helper functions
// -----------------------------------------------------------------------
// height of the cup rim above the cup's z = 0 (the TPU stands proud above it)
function bump_stop_rim_z(pocket_depth = 4, floor_t = 1.2, rivets = false, head_h = 1.2, base_h = 0) =
    base_h + (rivets ? head_h : 0) + floor_t + pocket_depth;
// outside size of the PETG cup across (round: its diameter)
function bump_stop_outer(d = 10, undercut = 1.2, wall = 2.0) = d + 2 * (undercut + wall);

// -----------------------------------------------------------------------
// The core: one cup + one plug. Stadium outline (l x w), round when l == w.
// part: "petg" / "tpu" / "both" as in params.scad.
// -----------------------------------------------------------------------
module _bs(part, l, w, h, pocket_depth, shape, undercut, lip_t, wall, floor_t, dome_h, ch,
           edge_ch, core_d, rivet_pts, rivet_dia, head_d, head_h, push_hole, base_h,
           socket_d, socket_depth, cap_edge) {
    rv = len(rivet_pts) > 0;
    hd = head_d > 0 ? head_d : rivet_dia + 2 * min_tpu;
    zh = base_h;                          // rivet heads (TPU) start here
    zf = base_h + (rv ? head_h : 0);      // PETG floor
    zc = zf + floor_t;                    // pocket floor
    zl = zc + pocket_depth - lip_t;       // underside of the lip
    zt = zc + pocket_depth;               // rim top; the TPU stands proud above it
    g = undercut + wall;                  // cup outside = opening grown by g
    sink = base_h > 0 ? 0 : eps;          // reach below z = 0 to fuse with the face below
    dh = dome_h > 0 ? dome_h : min(h, w / 2);

    // ---- design rules ------------------------------------------------
    assert(l >= w - 1e-6, "l must be >= w (round stop: l = w = d)");
    assert(w >= 4 - 1e-6, "opening (d or w) must be >= 4 mm");
    assert(undercut >= 0.8 - 1e-6, "undercut must be >= 0.8 mm (2 lines)");
    assert(lip_t >= 3 * layer_h - 1e-6, "lip_t must be >= 3 layers (0.6 mm)");
    assert(pocket_depth - lip_t >= min_tpu - 1e-6,
           "TPU flange under the lip (pocket_depth - lip_t) must be >= min_tpu (1.2)");
    assert(wall >= min_petg - 1e-6, "wall must be >= min_petg (1.2)");
    assert(floor_t >= 0, "floor_t must be >= 0");
    assert(!rv || floor_t >= min_petg - 1e-6, "with rivets, floor_t must be >= min_petg");
    assert(h >= 0, "h must be >= 0");
    assert(shape == "dome" || shape == "flat" || shape == "cap", "shape must be dome, flat or cap");
    assert(shape != "cap" || (cap_edge >= 3 * layer_h - 1e-6 && h >= cap_edge + 0.4 - 1e-6),
           "cap: need cap_edge >= 0.6 and h >= cap_edge + 0.4");
    assert(edge_ch >= 0 && edge_ch <= lip_t - 3 * layer_h + 1e-6,
           "edge_ch must leave >= 3 layers of lip at the opening edge");
    assert(core_d == 0 || (core_d >= 1.6 - 1e-6 && (w - core_d) / 2 >= min_tpu - 1e-6),
           "core_d: >= 1.6 and leave >= min_tpu of TPU wall");
    assert(!rv || rivet_dia >= 1.6 - 1e-6, "rivet holes must be >= 1.6 mm");
    assert(!rv || hd >= rivet_dia + 1.6 - 1e-6, "rivet head must lap the hole by >= 0.8 mm");
    assert(!rv || head_h >= 3 * layer_h - 1e-6, "head_h must be >= 3 layers");
    for (p = rivet_pts)
        assert(_bs_axd(p, l, w) + hd / 2 <= w / 2 + undercut + 1e-6,
               "a rivet head reaches outside the pocket outline");
    assert(_bs_mind(rivet_pts) >= hd + min_petg - 1e-6,
           "rivet heads closer than head_d + min_petg: fewer rivets or a larger pitch / circle");
    assert(push_hole == 0 || (!rv && push_hole >= 1.6 - 1e-6
                              && push_hole / 2 <= w / 2 + undercut - min_petg + 1e-6),
           "push_hole: no rivets, >= 1.6, and leave >= min_petg of floor round it");
    assert(socket_d == 0 || (socket_d >= 1.6 - 1e-6 && (w + 2 * g - socket_d) / 2 >= min_petg - 1e-6
                             && socket_depth + socket_d / 2 + min_petg <= base_h + 1e-6),
           "socket: too wide for the cup, or base_h too low over its cone");
    if (h > 1.5 * w) echo("WARNING bump_stop: proud height h > 1.5 x width; the TPU column may buckle");

    // ---- geometry ----------------------------------------------------
    // TPU that sits inside the PETG: undercut cavity, neck through the lip,
    // flare at the rim, rivet stems and heads under the floor
    module pocket() union() {
        translate([0, 0, zc]) linear_extrude(zl - zc + eps) _bs_o2d(l, w, undercut);
        translate([0, 0, zl]) linear_extrude(zt - zl) _bs_o2d(l, w);
        if (edge_ch > 0) hull() {
            translate([0, 0, zt - edge_ch]) linear_extrude(eps) _bs_o2d(l, w);
            translate([0, 0, zt - eps]) linear_extrude(eps) _bs_o2d(l, w, edge_ch);
        }
        for (p = rivet_pts) translate([p[0], p[1], zh]) {
            cylinder(d = hd, h = head_h, $fn = 32);
            cylinder(d = rivet_dia, h = head_h + floor_t + eps, $fn = 24);
        }
    }
    // TPU above the rim
    module proud() {
        if (shape == "flat") _bs_flat(l, w, h + eps, ch);
        else if (shape == "dome") _bs_dome(l, w, h + eps, dh);
        else _bs_dome(l + 2 * g, w + 2 * g, h + eps, h - cap_edge);   // cap over the whole rim
    }
    module region() difference() {
        union() {
            pocket();
            if (h > 0) translate([0, 0, zt - eps]) proud();
        }
        if (core_d > 0) translate([0, 0, zt]) cylinder(d = core_d, h = h + 1, $fn = bs_fn);
    }
    module petg() difference() {
        translate([0, 0, -sink]) linear_extrude(zt + sink) _bs_o2d(l, w, g);
        pocket();
        if (push_hole > 0) translate([0, 0, -1]) cylinder(d = push_hole, h = zc + 1 + eps, $fn = bs_fn);
        if (socket_d > 0) {
            translate([0, 0, -1]) cylinder(d = socket_d, h = socket_depth + 1, $fn = bs_fn);
            translate([0, 0, socket_depth - eps]) cylinder(d1 = socket_d, d2 = 0, h = socket_d / 2, $fn = bs_fn);
        }
    }
    hybrid(part) { petg(); region(); }
}

// -----------------------------------------------------------------------
// Public block modules
// -----------------------------------------------------------------------
// Round bump stop / foot: PETG cup standing on z = 0, TPU plug proud by h.
module bump_stop(part, d = 10, h = 6, pocket_depth = 4, shape = "dome", undercut = 1.2,
                 lip_t = 1.6, wall = 2.0, floor_t = 1.2, dome_h = -1, ch = 0.6, edge_ch = 0.4,
                 core_d = 0, rivets = 0, rivet_r = -1, rivet_dia = rivet_d, head_d = -1,
                 head_h = 1.2, push_hole = 0) {
    hd = head_d > 0 ? head_d : rivet_dia + 2 * min_tpu;
    rr = rivet_r >= 0 ? rivet_r : d / 2 + undercut - hd / 2 - 0.4;
    _bs(part, d, d, h, pocket_depth, shape, undercut, lip_t, wall, floor_t, dome_h, ch, edge_ch,
        core_d, _bs_ring(rivets, rr), rivet_dia, hd, head_h, push_hole, 0, 0, 0, 1.2);
}

// Landing foot: round cup, the TPU forms a rounded cap over the whole rim.
// socket_d > 0 makes it a standalone foot with a blind socket for a leg
// tube/rod in its underside (45 deg cone roof, no bridge).
module landing_foot(part, d = 16, h = 6, pocket_depth = 4, undercut = 1.2, lip_t = 1.6, wall = 2.0,
                    floor_t = 1.2, edge_h = 1.2, rivets = 3, rivet_r = -1, rivet_dia = rivet_d,
                    head_d = -1, head_h = 1.2, socket_d = 0, socket_depth = 10) {
    hd = head_d > 0 ? head_d : rivet_dia + 2 * min_tpu;
    rr = rivet_r >= 0 ? rivet_r : d / 2 + undercut - hd / 2 - 0.4;
    base = socket_d > 0 ? socket_depth + socket_d / 2 + min_petg : 0;
    _bs(part, d, d, h, pocket_depth, "cap", undercut, lip_t, wall, floor_t, -1, 0, 0.4,
        0, _bs_ring(rivets, rr), rivet_dia, hd, head_h, 0, base, socket_d, socket_depth, edge_h);
}

// Bumper strip: stadium-shaped pocket l x w along x (centred), rivets along
// its axis every rivet_pitch (0 = none).
module bumper_strip(part, l = 60, w = 10, h = 4, pocket_depth = 4, shape = "dome", undercut = 1.2,
                    lip_t = 1.6, wall = 2.0, floor_t = 1.2, dome_h = -1, ch = 0.6, edge_ch = 0.4,
                    rivet_pitch = 8, rivet_dia = rivet_d, head_d = -1, head_h = 1.2) {
    hd = head_d > 0 ? head_d : rivet_dia + 2 * min_tpu;
    _bs(part, l, w, h, pocket_depth, shape, undercut, lip_t, wall, floor_t, dome_h, ch, edge_ch,
        0, _bs_row(l, w, undercut, hd, rivet_pitch), rivet_dia, hd, head_h, 0, 0, 0, 0, 1.2);
}

// Test plate: three sizes in two rows on a plate_t PETG plate.
//   back row (+y): plain flat plugs for COMPRESSION tests;
//   front row (-y): the same plugs with a loose PETG piston under each one,
//   in a hole through plate and floor, for PUSH-OUT (= pull-out) tests.
module bump_stop_test(part, sizes = [6, 10, 16], h = 4, plate_t = 3, gap = 6, margin = 4) {
    o = [for (s = sizes) bump_stop_outer(s)];
    n = len(sizes);
    tot = _bs_cum(o, n) + (n - 1) * gap;
    xs = [for (i = [0 : n - 1]) -tot / 2 + _bs_cum(o, i) + i * gap + o[i] / 2];
    om = max(o);
    yr = om / 2 + gap / 2;                       // row centre lines at +-yr
    L = tot + 2 * margin;
    W = 2 * om + gap + 2 * margin;
    ph = [for (s = sizes) s - 2];                // push holes: floor ring stays undercut + 1
    if (part != "tpu") color(petg_colour) {
        difference() {
            translate([-L / 2, -W / 2, 0]) cube([L, W, plate_t]);
            for (i = [0 : n - 1]) translate([xs[i], -yr, -1]) cylinder(d = ph[i], h = plate_t + 2, $fn = bs_fn);
        }
        // pistons: 0.4 mm side clearance, flush with the pocket floor
        for (i = [0 : n - 1]) translate([xs[i], -yr, 0])
            cylinder(d = ph[i] - 2 * xy_clear, h = plate_t + 1.2, $fn = bs_fn);
    }
    for (i = [0 : n - 1]) {
        translate([xs[i], yr, plate_t]) bump_stop(part, d = sizes[i], h = h, shape = "flat");
        translate([xs[i], -yr, plate_t]) bump_stop(part, d = sizes[i], h = h, shape = "flat", push_hole = ph[i]);
    }
}

// Demo: a 100 x 60 chassis plate with a dome stop, a riveted flat stop,
// a landing foot and a bumper strip, plus four M3 holes.
module bump_stop_demo(part) {
    T = 3;
    if (part != "tpu") color(petg_colour) difference() {
        linear_extrude(T) offset(r = 4, $fn = 32) square([92, 52], center = true);
        for (x = [-45, 45], y = [-25, 25]) translate([x, y, -1]) cylinder(d = 3.4, h = T + 2, $fn = 24);
    }
    translate([-32, 14, T]) bump_stop(part, d = 10, h = 6, shape = "dome");
    translate([-8, 14, T]) bump_stop(part, d = 10, h = 4, shape = "flat", rivets = 2);
    translate([24, 12, T]) landing_foot(part, d = 16, h = 6, rivets = 3);
    translate([0, -16, T]) bumper_strip(part, l = 80, w = 10, h = 4);
}

// -----------------------------------------------------------------------
if (show == "demo") bump_stop_demo(part);
else if (show == "test") bump_stop_test(part);
else if (show == "foot") landing_foot(part, d = 16, h = 6, rivets = 3, socket_d = 8.3, socket_depth = 10);
