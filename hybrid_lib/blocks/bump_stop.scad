include <../params.scad>
// =======================================================================
// bump_stop: TPU bump stops, landing feet and bumpers held in PETG cups
// =======================================================================
// WHAT IT IS FOR
//   Soft contact points on a PETG rover chassis or drone frame, printed in
//   the same print (H2D: PETG left nozzle, TPU 95A right nozzle), loaded
//   mainly in COMPRESSION: suspension end stops, landing feet, skid feet,
//   bumpers. PETG and TPU do not reliably stick to each other, so every
//   plug is held by SHAPE ONLY - it would stay put with zero adhesion.
//
// HOW IT WORKS (real section through a d 10 dome stop with rivets = 2,
//   cut through the rivets; 1 char = 0.4 mm across, 1 line = 0.6 mm up;
//   # = PETG, T = TPU)
//
//   z mm
//   12.3 |                  TTTTT                  |
//        |              TTTTTTTTTTTTT              |  <- dome: spherical cap
//        |            TTTTTTTTTTTTTTTTT            |     (shape = "dome")
//        |           TTTTTTTTTTTTTTTTTTT           |
//        |          TTTTTTTTTTTTTTTTTTTTT          |
//        |         TTTTTTTTTTTTTTTTTTTTTTT         |
//        |        TTTTTTTTTTTTTTTTTTTTTTTTT        |  <- proud part, h above the
//        |        TTTTTTTTTTTTTTTTTTTTTTTTT        |     rim: the only TPU that
//        |        TTTTTTTTTTTTTTTTTTTTTTTTT        |     can bulge = the spring
//    6.9 |        TTTTTTTTTTTTTTTTTTTTTTTTT        |
//    6.3 |#######TTTTTTTTTTTTTTTTTTTTTTTTTTT#######|  <- rim top, edge_ch bevel
//        |########TTTTTTTTTTTTTTTTTTTTTTTTT########|  <- LIP (lip_t) printed on
//        |########TTTTTTTTTTTTTTTTTTTTTTTTT########|     the TPU; opening d
//        |#####TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT#####|  <- UNDERCUT: pocket is
//        |#####TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT#####|     d + 2 undercut wide;
//        |#####TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT#####|     its TPU flange cannot
//        |#####TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT#####|     pass the lip
//    2.1 |#########TTTTTT###########TTTTTT#########|  <- floor (floor_t) with
//        |#########TTTTTT###########TTTTTT#########|     rivet stems through it
//    0.9 |######TTTTTTTTTTTT#####TTTTTTTTTTTT######|  <- rivet heads (head_h)
//    0.3 |######TTTTTTTTTTTT#####TTTTTTTTTTTT######|     under the floor
//        |<--------- 16.4 = d + 2 (undercut + wall) --------->|
//   Pictures: images/bump_stop.png (demo), images/bump_stop_section.png
//   (cut-away: riveted dome stop, flat stop with hollow core, landing foot
//   with leg socket, bumper strip), images/bump_stop_test.png (test plate).
//
//   (a) UNDERCUT + LIP: the pocket is wider (d + 2 undercut) than its
//       opening (d). The top lip_t of the cup wall - the lip - is printed
//       on top of the TPU flange (which supports it). The flange cannot
//       rise past the lip, the wall stops it sideways, the floor stops it
//       going down. This alone captures the plug (the demo's dome stop has
//       no rivets and passes the 26-direction check).
//   (b) RIVETS (optional, as bond_tests coupon G): TPU stems through holes
//       in the floor end in heads in recesses under it. They hold the middle
//       of a big plug / a long strip down, so a pull or a peel starting at
//       one end must tear stems one by one, and add pull-out strength.
//   (c) COMPRESSION goes straight into the floor in bearing - never through
//       the joint. The TPU inside the cup is confined on all sides and
//       nearly incompressible, so the spring is only the proud part (h);
//       the confined TPU pushes on the wall and lip like a fluid (LIMITS).
//   (d) LANDING FOOT: the TPU cap covers the whole rim top (TPU printed on
//       the rim), rounded, so an angled landing hits TPU, never the PETG
//       edge, and the foot slides instead of digging in.
//   (e) BUMPER STRIP: the same section along a stadium (round-ended) outline;
//       the undercut runs all round, one row of rivets along the axis.
//   The TPU body is (TPU region) minus (PETG), so the faces match exactly.
//   tools/check_block.py: demo (PETG 1, TPU 4), test (PETG 4 = plate + 3
//   loose pistons, TPU 6) and foot (1, 1) are watertight, overlap-free,
//   nothing floats, every plug is captured in all 26 directions (it can
//   slide 0.15 mm at most) and PrusaSlicer slices them with two extruders.
//
// PUBLIC MODULES (mm; every cup stands on z = 0, centred on x = y = 0)
//   bump_stop(part, d, h, pocket_depth, shape, ...)     round stop / foot pad
//   landing_foot(part, d, h, ..., socket_d, socket_depth)  rounded cap over
//       the whole rim; socket_d > 0 = standalone foot with a blind socket
//       for a leg tube/rod in its underside (45 deg cone roof, no bridge).
//   bumper_strip(part, l, w, h, ..., rivet_pitch)       long strip along x
//   bump_stop_test(part, sizes, h, plate_t, gap, margin) test plate
//   bump_stop_demo(part)                                 demo plate
//   functions: bump_stop_rim_z(pocket_depth, floor_t, rivets, head_h,
//       base_h) = rim height above the cup bottom (TPU top = rim + h);
//       bump_stop_outer(d, undercut, wall) = cup outside diameter / width.
//   Put a cup on any flat, upward-facing (+z) PETG face of your part:
//       if (part != "tpu") my_body();                 // top face at z = t
//       translate([x, y, t]) bump_stop(part, d = 10, h = 5);
//   The cup's PETG reaches eps below its z = 0 so it fuses with your body
//   (not when it has its own base, i.e. landing_foot with a socket); its
//   TPU starts at z >= 0, so it never overlaps a body that ends at z <= t.
//   part = "petg" / "tpu" return exactly the PETG cup / the TPU region, so
//   you can cut through both (bump_stop_test does) or sink a cup flush
//   into a thick body (p = cup bottom, rim flush with the body's top):
//       PETG: union() { difference() { my_body(); translate(p) bump_stop("tpu", ...); }
//                       translate(p) bump_stop("petg", ...); }
//       TPU:  translate(p) bump_stop("tpu", ...);
//   Your face must be flat and at least the cup's outside wide.
//
// PARAMETERS (same names in every module; defaults in brackets)
//   d          [10]  opening = diameter of the TPU contact face. >= 4 (a
//                    printable TPU column). Stiffness ~ d^2.
//   h           [6]  TPU proud of the rim = the working zone; usable travel
//                    ~ h / 4. 0 = flush plug. <= 1.5 d (taller columns
//                    buckle sideways; a warning is echoed).
//   pocket_depth[4]  pocket floor to rim top (TPU depth inside the cup).
//                    Rule: pocket_depth - lip_t >= min_tpu, so the TPU
//                    flange under the lip is >= 3 lines thick (2.4 here).
//   shape  ["dome"]  "dome": straight side + spherical cap dome_h high -
//                    soft first touch, progressive; "flat": flat face with a
//                    45 deg chamfer ch - stiffer, linear, for hard end stops.
//                    (landing_foot always uses its own rounded "cap".)
//   undercut  [1.2]  how far the pocket reaches under the lip (radially).
//                    >= 0.8 (2 lines); 1.2 = 3 lines. Pull-out strength
//                    grows with it; for d > 12 consider ~0.12 d.
//   lip_t     [1.6]  lip thickness (8 layers). >= 0.6 (3 layers). Lip
//                    bending stress ~ 1 / lip_t^2; 1.6 keeps it below the
//                    wall's hoop stress (LIMITS).
//   wall      [2.0]  cup wall outside the undercut (5 lines). >= min_petg.
//                    Sets the compression limit (hoop stress ~ 1 / wall).
//   floor_t   [1.2]  PETG floor under the pocket (6 layers). 0 only when the
//                    cup sits on your own PETG face (that face is then the
//                    floor); >= min_petg with rivets.
//   dome_h     [-1]  cap height for "dome"; -1 = min(h, d/2) (hemisphere
//                    when h >= d/2). Clamped to d/2 (no TPU overhang).
//   ch        [0.6]  top-edge chamfer of a "flat" pad.
//   edge_ch   [0.4]  45 deg bevel on the rim's inner top edge, so a column
//                    pushed sideways bends over a bevel, not a sharp PETG
//                    edge. <= lip_t - 0.6.
//   core_d      [0]  hollow core through the proud part (a TPU tube): a bit
//                    softer and lighter. >= 1.6, leaves >= min_tpu of wall.
//   rivets      [0]  TPU rivets through the floor: 1 = centre, n >= 2 on a
//                    circle. Rule: heads inside the pocket outline and >=
//                    head_d + min_petg apart (d 10: max 2; d 16: max 6).
//                    Each raises the rim by head_h. (landing_foot: [3])
//   rivet_r    [-1]  rivet circle radius; -1 = as far out as fits,
//                    d/2 + undercut - head_d/2 - 0.4.
//   rivet_dia [2.4]  stem diameter (params rivet_d); >= 1.6 (hole rule).
//   head_d     [-1]  head diameter; -1 = rivet_dia + 2 min_tpu = 4.8 (the
//                    head laps the hole by 3 lines). >= rivet_dia + 1.6.
//   head_h    [1.2]  head thickness (6 layers). >= 0.6.
//   landing_foot only: d [16], edge_h [1.2] vertical TPU edge where the cap
//       meets the rim's outer edge (>= 0.6; h >= edge_h + 0.4), so the cap
//       has no feather edge; socket_d [0] leg socket bore (tube OD + 0.2 to
//       0.3); socket_depth [10]. With a socket the foot gets a solid base
//       socket_depth + socket_d/2 + min_petg high under the rivet heads.
//       Defaults: 22.4 mm across, rim 6.4, top 12.4 (with socket_d 8.3:
//       27.8 mm tall).
//   bumper_strip only: l [60] opening length along x (>= w; round ends),
//       w [10] opening width, h [4], rivet_pitch [8] rivets along the axis,
//       as many as fit (l 60: 8, l 80: 10); 0 = none; >= head_d + min_petg.
//   bump_stop_test only: sizes [[6, 10, 16]], h [4], plate_t [3], gap [6]
//       between cups, margin [4].
//
// STIFFNESS / STRENGTH ESTIMATE (hand formulas; TPU 95A E ~ 26 MPa,
//   G ~ 9 MPa, nu ~ 0.5 -> E* = E/(1 - nu^2) ~ 35 MPa; printed TPU tensile
//   strength assumed ~ 20 MPa; PETG E ~ 2000 MPa, ~ 45 MPa)
//   FLAT pad, diameter d, free height h, A = pi d^2/4. Shape factor
//     S = loaded area / bulging area = (pi d^2/4) / (pi d h) = d / (4 h).
//     k_c = E_c A / h,  E_c = E (1 + 2 phi S^2),  phi ~ 0.5 for a hard
//     elastomer whose face cannot slide on what it hits (phi = 0 if it
//     slides freely -> lower bound k_c = E A / h).
//       d 10, h 6:  S 0.42 -> k_c ~ 340 .. 400 N/mm
//       d 10, h 4:  S 0.63 -> k_c ~ 510 .. 710 N/mm
//       test plate (h 4): d 6 ~ 180 .. 210, d 10 ~ 510 .. 710,
//                         d 16 ~ 1300 .. 2600 N/mm (S = 1: fat pads 2x)
//     Hollow core c: ring area, S = (d - c) / (4 h); d 10, c 4, h 4:
//       k_c ~ 430 .. 490 N/mm (~30 % softer than solid).
//   DOME, cap radius R = (a^2 + t^2) / (2 t), a = d/2, t = dome_h. Hertz
//     against a flat:  F ~ (4/3) E* sqrt(R) delta^1.5.
//       d 10, h 6 (hemisphere, R 5): F ~ 103 delta^1.5 N (delta in mm):
//       37 N @ 0.5 mm, 103 N @ 1 mm, 190 N @ 1.5 mm; tangent stiffness
//       155 sqrt(delta) N/mm - soft first touch, rising towards the flat
//       value (~350 N/mm) as the cap flattens. ~0.11 J taken up to 1.5 mm.
//   SIDEWAYS (shear of the proud part): k_s ~ G A / h = 9 * 78.5 / 6 ~
//     120 N/mm for d 10, h 6.
//   LANDING FOOT, d 16 (cap over the 22.4 mm rim, h 6, edge_h 1.2 ->
//     R 15.5): F ~ 182 delta^1.5 N. 1 kg drone on 4 feet: 0.3 m/s
//     touchdown -> ~0.5 mm, ~60 N per foot; 1 m/s drop (0.125 J per foot)
//     -> ~1.2 mm, ~250 N per foot (~26 g).
//   BUMPER STRIP, dome, w 10: line contact along the straight part,
//     k ~ (pi/4) E* ~ 27 N/mm per mm of contact (l 60: 50 mm -> ~1.4 kN/mm).
//   => TPU 95A in compression is STIFF: these are end stops, feet and
//     impact skins, not suspension springs. For travel use a bending /
//     torsion TPU spring block and let a bump stop end its stroke.
// LIMITS (what gives first)
//   * Strain: keep deflection <= ~25 % of h for repeated hits (TPU 95A
//     takes a permanent set beyond ~30 %): 1.5 mm for h 6, 1 mm for h 4.
//   * Cup wall in compression: the confined TPU pushes out at p ~ F/A_open.
//     Hoop stress p (d/2 + undercut) / wall <= ~20 MPa (PETG, fatigue
//     margin): F_max ~ 20 * wall / (d/2 + undercut) * A_open
//       = 270 N (d 6), 510 N (d 10), 870 N (d 16) with wall 2.
//     The same pressure lifts the lip: sigma ~ 3 p undercut^2 / lip_t^2
//     = 1.7 p (below the hoop stress). Both are in-layer stresses (strong
//     direction). A STRIP's straight walls are cantilevers instead:
//     sigma ~ 3 p pocket_depth^2 / wall^2 = 12 p -> p <= ~1.7 MPa, ~17 N
//     per mm of strip; sink strips into a thick body (or raise wall) for
//     heavier hits.
//   * Pull-out / peel, no adhesion: the base (d + 2 undercut) must squeeze
//     through the opening d, a radial strain eps = 2 undercut/(d + 2 undercut).
//     Crude estimate F_out ~ 2 E eps A_flange ~ 0.4 kN for d 6 .. 16 with
//     undercut 1.2 (A_flange = ring under the lip: 27 / 42 / 65 mm^2). The
//     neck itself tears at ~ 20 MPa * A_open (570 N d 6, 1.6 kN d 10).
//     Each rivet adds roughly its stem, 20 MPa * pi 2.4^2/4 ~ 90 N.
//     These are order-of-magnitude numbers: measure with the test plate.
//
// RECOMMENDED USES
//   Rover: end stops on rocker-bogie / swing-arm / steering travel (dome
//   = progressive, flat = hard stop), skid feet under the chassis, bumper
//   strips along the nose and corners, soft seats in battery/payload trays.
//   Drone: landing feet on leg ends printed upright (cup on the leg's top
//   face) or socket feet on carbon legs, belly skid pads, frame-edge and
//   arm bumpers, gimbal / camera tilt end stops. Make the contact face
//   point UP (+z) in the print orientation.
//
// TEST PIECE (show = "test", bump_stop_test(part)): 71 x 59 x 3 mm plate,
//   flat plugs d 6 / 10 / 16 with the same h 4, so S = 0.38 / 0.63 / 1.0.
//   Row "C" (COMPRESSION): put the plate on a kitchen/bathroom scale, press
//     a stiff flat plate straight down on one plug, read force against
//     squash (calipers on the remaining proud height or a dial gauge) up to
//     ~1 mm. Compare with k_c above (gives E and phi for your filament).
//   Row "P" (PUSH-OUT = pull-out): a loose PETG piston (3.2 / 7.2 / 13.2 mm,
//     0.4 mm clearance) sits under each plug in a hole through plate and
//     floor - keep them when you take the plate off the bed. Turn the plate
//     over onto a deep socket / pipe / two blocks that clear the cup
//     (12.4 / 16.4 / 22.4 mm), all on the scale, and press the piston with
//     a bolt or punch until the plug pops out through the lip. The peak is
//     the retention force (any adhesion included); compare with F_out.
//
// PRINTING / RULES THIS BLOCK FOLLOWS
//   No supports. The lip and the floor over the rivet heads are PETG
//   printed on TPU (supported by it); the socket roof is a 45 deg cone; the
//   test pistons' 0.4 mm gap is the only bridge (TPU). Walls >= 1.2 mm in
//   both materials, holes >= 1.6 mm. Print the TPU SOLID (100 % infill) -
//   the estimates assume it. Every layer from the floor to the rim holds
//   both materials (PrusaSlicer test: 35 tool changes for the demo, 23 for
//   the test plate). Separate PETG bodies only in the test plate (pistons,
//   0.4 mm side clearance, TPU above them).
//
// LIMITATIONS
//   * Contact faces point +z in print only (the cup opens upward). Side or
//     downward-facing stops need the part printed with that face up.
//   * Round cups and straight stadium strips only; follow a curve with
//     several short strips (or use skin_anchor for thin skins).
//   * The cup stands rim_z proud of your face (5.2 mm; 6.4 with rivets) -
//     sink it into a thick body for a flush stop (see PUBLIC MODULES).
//   * One ring / one row of rivets; strips wider than ~20 mm would want two.
//   * Rigid capture is checked, not soft pull-out: a hard enough pull or
//     peel can still squeeze TPU out. All numbers are hand estimates
//     (+-50 %); TPU brands and print settings differ - use the test plate.
//   * Not a suspension spring (see the estimates).
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
           edge_ch, core_d, rivet_pts, rivet_dia, head_d, head_h, base_h,
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
                 head_h = 1.2) {
    hd = head_d > 0 ? head_d : rivet_dia + 2 * min_tpu;
    rr = rivet_r >= 0 ? rivet_r : d / 2 + undercut - hd / 2 - 0.4;
    _bs(part, d, d, h, pocket_depth, shape, undercut, lip_t, wall, floor_t, dome_h, ch, edge_ch,
        core_d, _bs_ring(rivets, rr), rivet_dia, hd, head_h, 0, 0, 0, 1.2);
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
        0, _bs_ring(rivets, rr), rivet_dia, hd, head_h, base, socket_d, socket_depth, edge_h);
}

// Bumper strip: stadium-shaped pocket l x w along x (centred), rivets along
// its axis every rivet_pitch (0 = none).
module bumper_strip(part, l = 60, w = 10, h = 4, pocket_depth = 4, shape = "dome", undercut = 1.2,
                    lip_t = 1.6, wall = 2.0, floor_t = 1.2, dome_h = -1, ch = 0.6, edge_ch = 0.4,
                    rivet_pitch = 8, rivet_dia = rivet_d, head_d = -1, head_h = 1.2) {
    hd = head_d > 0 ? head_d : rivet_dia + 2 * min_tpu;
    _bs(part, l, w, h, pocket_depth, shape, undercut, lip_t, wall, floor_t, dome_h, ch, edge_ch,
        0, _bs_row(l, w, undercut, hd, rivet_pitch), rivet_dia, hd, head_h, 0, 0, 0, 1.2);
}

// Test plate: three sizes in two rows on a plate_t PETG plate.
//   back row (+y): plain flat plugs for COMPRESSION tests;
//   front row (-y): the same plugs with a loose PETG piston under each one,
//   in a hole through plate and cup floor, for PUSH-OUT (= pull-out) tests.
// (Also an example of composing the block with your own PETG: the holes are
//  cut through plate + cup PETG together, the TPU is the block's own.)
module bump_stop_test(part, sizes = [6, 10, 16], h = 4, plate_t = 3, gap = 6, margin = 4) {
    o = [for (s = sizes) bump_stop_outer(s)];
    n = len(sizes);
    tot = _bs_cum(o, n) + (n - 1) * gap;
    xs = [for (i = [0 : n - 1]) -tot / 2 + _bs_cum(o, i) + i * gap + o[i] / 2];
    om = max(o);
    yr = om / 2 + gap / 2;                       // row centre lines at +-yr
    L = tot + 2 * margin;
    W = 2 * om + gap + 2 * margin;
    ph = [for (s = sizes) s - 2];                // push holes: leaves a floor ring of undercut + 1
    zc = plate_t + 1.2;                          // pocket floor (cup floor_t = 1.2, no rivets)
    module cups(p) for (i = [0 : n - 1]) for (y = [yr, -yr])
        translate([xs[i], y, plate_t]) bump_stop(p, d = sizes[i], h = h, shape = "flat");
    module petg() {
        difference() {
            union() { translate([-L / 2, -W / 2, 0]) cube([L, W, plate_t]); cups("petg"); }
            // push holes, reaching up into the (empty) pocket
            for (i = [0 : n - 1]) translate([xs[i], -yr, -1]) cylinder(d = ph[i], h = zc + 1.5, $fn = bs_fn);
            // row marks, engraved 0.6 mm: C = compression row, P = push-out row
            for (m = [["C", 1], ["P", -1]]) translate([xs[0], m[1] * gap * 0.75, plate_t - 0.6])
                linear_extrude(1) text(m[0], size = 4, font = "DejaVu Sans:style=Bold",
                                       halign = "center", valign = "center");
        }
        // pistons: 0.4 mm side clearance, flush with the pocket floor
        for (i = [0 : n - 1]) translate([xs[i], -yr, 0]) cylinder(d = ph[i] - 2 * xy_clear, h = zc, $fn = 40);
    }
    if (part == "petg") petg();
    else if (part == "tpu") cups("tpu");
    else { color(petg_colour) petg(); color(tpu_colour) cups("tpu"); }
}

// Demo: a 100 x 60 chassis plate with a dome stop, a riveted flat stop with
// a hollow core, a landing foot and a bumper strip, plus four M3 holes.
module bump_stop_demo(part) {
    T = 3;
    if (part != "tpu") color(petg_colour) difference() {
        linear_extrude(T) offset(r = 4, $fn = 32) square([92, 52], center = true);
        for (x = [-45, 45], y = [-25, 25]) translate([x, y, -1]) cylinder(d = 3.4, h = T + 2, $fn = 24);
    }
    translate([-32, 14, T]) bump_stop(part, d = 10, h = 6, shape = "dome");
    translate([-8, 14, T]) bump_stop(part, d = 10, h = 4, shape = "flat", core_d = 4, rivets = 2);
    translate([24, 12, T]) landing_foot(part, d = 16, h = 6, rivets = 3);
    translate([0, -16, T]) bumper_strip(part, l = 80, w = 10, h = 4);
}

// -----------------------------------------------------------------------
if (show == "demo") bump_stop_demo(part);
else if (show == "test") bump_stop_test(part);
else if (show == "foot") landing_foot(part, d = 16, h = 6, rivets = 3, socket_d = 8.3, socket_depth = 10);
