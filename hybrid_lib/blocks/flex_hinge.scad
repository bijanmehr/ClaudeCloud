include <../params.scad>
// =======================================================================
// flex_hinge: a TPU living hinge joining two PETG parts
// =======================================================================
// WHAT IT IS FOR
//   A fold line printed in place, in the same print as the rigid parts:
//   two PETG parts side by side, joined by a short flat TPU web that bends
//   about an axis parallel to their edges (the Y axis here). Nothing to
//   assemble, no pins, no wear, no backlash, and it springs back.
//   On rovers and drones: compliant legs and bumpers, folding arms, sensor
//   and camera flaps, battery and hatch doors, cable covers, gripper fingers.
//
// HOW IT WORKS (every PETG/TPU joint holds by shape, never by adhesion)
//
//   side view (cut along X):
//        plate A          TPU block        web        TPU block         plate B
//   +----------------+---------------+           +---------------+----------------+ z = t
//   |                |               |           |               |                |
//   |      PETG      ==== tongue ==T |           | T== tongue ====      PETG      |
//   |                |                \_________/                |                |
//   +----------------+-------------------------------------------+----------------+ z = 0
//                     <-- block_l --> r<- gap ->r <-- block_l -->   (web: web_t thick, on the bed)
//   plan of one tongue (inside its TPU block; o = TPU-filled rivet holes):
//        plate |===========+--+
//        edge  |  o  o  o  |T |  <- the head, wider than the neck
//              |===========+--+
//
//   - Each plate edge grows a thin PETG tongue at mid-thickness (like bond
//     coupons F and G): a neck with a row of rivet holes and a wider T-head
//     at its end. The tongue is wrapped on every side by a TPU block the
//     full thickness of the plate (skin >= 1.2 mm above and below, side
//     walls beside the head, a tip wall in front of it), and the TPU fills
//     the rivet holes, tying the upper and lower skins through the PETG.
//     Pulled away from the plate, the TPU bears on the back of the T-head
//     and on the rivet holes; pushed, it bears on the plate edge and on the
//     head front; up/down and sideways it is boxed in by its own skins and
//     walls. tools/capture_check.py confirms the TPU cannot slide out in
//     any of 26 directions.
//   - Between the two blocks a thinner TPU web, lying on the bed, is the
//     bending zone. It is printed flat, so the bending strain runs along the
//     extrusion lines inside each layer, never across layer boundaries (no
//     layer peeling). Small fillets (root_r) round the corners where the web
//     meets the blocks, so the strain does not peak at a sharp notch.
//   - The blocks are 2x thicker than the web (8x stiffer in bending) and
//     carry a PETG tongue, so practically all bending happens in the web.
//
// PRINTING (Bambu Lab H2D, 0.4 mm nozzles, 0.2 mm layers, no supports)
//   PETG left nozzle, TPU 95A right nozzle. Lay it flat as modelled (the web
//   on the bed). The tongue's underside is printed on the TPU skin and the
//   TPU skin on top of the tongue is printed on PETG; nothing else hangs.
//   Print the TPU solid (100 % infill, or >= 4 walls and >= 5 top/bottom
//   layers) so the web and the skins have no voids; if your slicer lets you,
//   run the TPU's solid lines across the fold axis (along X).
//
// PARAMETERS (mm, degrees). Defaults and the rule behind each:
//   width     = 20   hinge width along the fold axis (Y). Stiffness and
//                    strength scale linearly with it. >= 10 (anchor fits:
//                    2 * side + 2 * head_over + a 3.6 mm neck);
//                    width <= the plate width.
//   gap       = 6    free web length (X), flat part between the two root
//                    fillets. Rule: bending strain at fold angle theta (rad)
//                       eps = theta * web_t / (2 * gap)
//                    so for eps <= ~25 % at 90 deg:  gap >= 3 * web_t
//                    (gap = 3 * web_t gives pi/12 = 26 %). Asserted against
//                    `fold`: eps(fold) <= 27 %. The fillets add bending
//                    length, so the formula is on the safe side.
//   web_t     = 2.0  web thickness (Z, 10 layers). >= 1.2 (min_tpu). Fold
//                    stiffness grows with web_t^3, strength with web_t.
//   plate     = [25, 20, 4]  plate A [length X, width Y, thickness Z];
//                    plate_b defaults to the same. Thickness t is also the
//                    TPU block height. Rule: t >= 2 * skin + 1.2 = 3.6
//                    (1.2 mm TPU skins over a >= 1.2 mm PETG tongue).
//   fold      = 90   design fold angle used for the strain check.
//   root_r    = 1.0  fillet radius where the web meets each block (<= t - web_t).
//   skin      = 1.2  TPU above and below the tongue (min_tpu, 3 layers+).
//                    The tongue is t - 2 * skin thick (1.6 at t = 4).
//   side      = 1.6  TPU wall beside the T-head (Y). >= min_tpu.
//   neck_l    = 5.0  tongue neck length from the plate edge (X). With rivets
//                    >= rivet_d + 2 * 1.2 = 4.8 (PETG walls round the holes).
//   head_l    = 2.0  T-head length (X). >= min_petg.
//   head_over = 1.6  how far the head sticks out past the neck each side (Y):
//                    the hook that the TPU pocket behind the head bears on.
//                    >= min_tpu so the pocket TPU is 3 lines wide.
//   tip       = 1.6  TPU wall in front of the head (X). >= min_tpu.
//   rivet_d   = 2.4  holes through the neck that the TPU fills (params
//                    rivet_d; >= 1.6). 0 = no rivets.
//   rivet_pitch = 4.0  hole spacing (Y): leaves 1.6 mm PETG bars between
//                    holes; as many as fit with >= 1.2 mm to the neck edge.
//   overlap   = 1.0  how far the anchor reaches back into your part, so the
//                    two fuse into one PETG body.
//   holes_a, holes_b = []  mounting holes in plate A / B as [distance from
//                    the hinge edge, y]; hole_d = 3.4 (M3 clearance).
//   label     = ""   text engraved 0.6 mm into the top of plate B.
//   Derived: block_l = neck_l + head_l + tip = 8.6 (each TPU block, X);
//   the plate edges sit at x = +-e, e = gap/2 + root_r + block_l = 12.6
//   (flex_hinge_edge_x()).
//
// ESTIMATES (defaults: width w = 20, web_t h = 2, gap L = 6;
//            TPU 95A E ~ 26 MPa, G ~ 9 MPa; PETG E ~ 2000 MPa, ~45 MPa)
//   Fold stiffness   k = E * w * h^3 / (12 * L) = 26 * 20 * 8 / 72
//                      = 58 N*mm/rad (~1.0 N*mm per degree)
//   Moment at 90 deg M = k * pi/2 = 91 N*mm: ~3.6 N pushing 25 mm from the
//                      fold. Small-strain beam theory; expect +-30 %.
//   Strain at 90 deg eps = (pi/2) * h / (2 L) = 26 %.
//   Fold-up limit    folding so the top faces close, the blocks touch at
//                      theta ~ (gap + 2 root_r) / (t - web_t/2) = 2.7 rad
//                      (~150 deg); folding the other way is limited only by
//                      the strain.
//   Axial (pull)     k_x = E * w * h / L = 26 * 40 / 6 = 170 N/mm.
//   Yaw (in-plane)   E * (h * w^3 / 12) / L = 5800 N*mm/rad: 100x the fold
//                      stiffness ((w/h)^2), so it folds, it does not skew.
//   Twist (about X)  G * (w * h^3 / 3) / L = 9 * 53 / 6 = 80 N*mm/rad: as
//                      soft as the fold. Use two hinges spaced apart (or one
//                      wide one) where twist matters.
//   Web in tension   section w * h = 40 mm^2; at a working stress of 2.5 MPa
//                      (~10 % strain) 100 N; TPU breaks at ~25-35 MPa
//                      (1000+ N), long after the anchors would let go.
//   Anchor pull-out  (no adhesion) bearing on the back of the T-head
//                      2 * head_over * tongue = 2 * 1.6 * 1.6 = 5.1 mm^2 plus
//                      3 rivets * rivet_d * tongue = 3 * 2.4 * 1.6 = 11.5 mm^2
//                      = 16.6 mm^2; at ~15-20 MPa confined bearing in TPU
//                      roughly 250-330 N per anchor. Measure your own with
//                      the bond-test pull coupons (F/G) or this test coupon.
//   PETG tongue      net section (13.6 - 3 * 2.4) * 1.6 = 10 mm^2 -> 460 N.
//                      Root bending at a 90 deg fold, if the tongue alone
//                      took the moment: 6 M / (b t^2) = 6 * 91 / (13.6 *
//                      1.6^2) = 16 MPa, about a third of PETG strength (the
//                      TPU block also bears on the plate edge, so less).
//   Design rule: the TPU web is the weakest and softest section, so the
//   hinge fails, if ever, by a visible crack across the web, not by the
//   anchors pulling out.
//
// TEST PIECE (show = "test"): flex_hinge_coupon(), a 90 deg flex-fatigue
//   coupon. Plate A (24 mm, two M3 holes) bolts or clamps flat with its
//   edge flush with the bench edge; plate B is a 40 mm lever with an M3
//   hole near its end for a servo horn link or a string. Fold B up to 90 deg
//   and back (by hand, or a servo at ~1 Hz) and count cycles. Every 1000
//   cycles look for: whitening or a crack across the web (record the
//   count), a gap opening between a TPU block and its plate, the head
//   showing through the skin. Then pull it apart with a luggage scale.
//   The web size is engraved on plate B ("T<web_t> L<gap>").
//
// MODULES (all take `part` = "petg" | "tpu" | "both" first)
//   flex_hinge(part, width, gap, web_t, plate, ...)   two plates + hinge
//   flex_hinge_joint(part, width, gap, web_t, t, ...) the hinge alone (both
//       anchors + TPU) between plate edges at x = +-flex_hinge_edge_x();
//       add your own PETG parts at |x| >= e and they fuse with the anchors.
//   flex_hinge_anchor(part, width, t, ...)  one PETG anchor; edge frame:
//       the edge face on x = 0, your part on x < 0, the anchor points +x.
//       Union it with your PETG body.  ("tpu" draws nothing.)
//   flex_hinge_tpu_region(part, width, gap, web_t, t, ...)  the TPU REGION
//       (blocks + web, anchors NOT removed) for child(1) of hybrid().
//       ("petg" draws nothing.)
//   flex_hinge_coupon(part, ...)  the fatigue test coupon.
//   Functions: flex_hinge_edge_x(), flex_hinge_strain(), flex_hinge_min_gap(),
//       flex_hinge_k().
//
// RULES FOR YOUR OWN PARTS
//   - Keep your PETG out of the hinge zone |x| < e, and give each part a
//     solid edge face at x = -e / +e at least `width` wide that covers
//     z = 0..t: the anchor root reaches `overlap` (1 mm) back into it.
//   - Parts may be thicker than t (the hinge sits at the bottom) but not
//     thinner.
//   - The two parts must not touch anywhere else, or they stop being two
//     bodies; give them xy_clear (0.4 mm) where they come close.
//
// LIMITATIONS
//   - Not a precision pivot: the fold axis moves with the bending and the
//     hinge gives a little in tension and twist. Use a pin hinge to locate.
//   - TPU creeps: a hinge held folded for weeks takes a set; one that
//     carries a constant load (e.g. a leg spring under the robot's weight)
//     will sag over time.
//   - Fatigue life of printed TPU is not known in advance; run the coupon.
//   - The capture check treats the TPU as rigid. A large enough pull can
//     still squeeze the TPU past the head; the anchor sizes above keep the
//     web the weakest link, but only a pull test proves it on your printer.
//   - The web lies on the bed, so the hinge is not symmetric: folding so the
//     plate tops close is limited to ~150 deg by the blocks touching.
//     (A mid-thickness web would need a > 5 mm TPU bridge.)
//   - Twist stiffness is low (see above).
// =======================================================================

part = "both";   // [petg, tpu, both]
show = "demo";   // [demo, test, section]

// ---------------------------------------------------------------------
// Defaults shared by all modules (so anchors and TPU always agree)
// ---------------------------------------------------------------------
fh_width = 20;
fh_gap = 6;
fh_web_t = 2.0;
fh_t = 4;
fh_fold = 90;
fh_root_r = 1.0;
fh_skin = min_tpu;          // 1.2
fh_side = 1.6;
fh_neck_l = 5.0;
fh_head_l = 2.0;
fh_head_over = 1.6;
fh_tip = 1.6;
fh_rivet_d = rivet_d;       // 2.4
fh_rivet_pitch = 4.0;
fh_overlap = 1.0;

// ---------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------
// distance from the hinge centre to each plate edge
function flex_hinge_edge_x(gap = fh_gap, root_r = fh_root_r, neck_l = fh_neck_l, head_l = fh_head_l, tip = fh_tip) =
    gap / 2 + root_r + neck_l + head_l + tip;
// peak bending strain (fraction) in the web folded to `angle` degrees
function flex_hinge_strain(gap = fh_gap, web_t = fh_web_t, angle = fh_fold) = angle * PI / 180 * web_t / (2 * gap);
// shortest free web length for a strain limit at a fold angle
function flex_hinge_min_gap(web_t = fh_web_t, angle = fh_fold, strain = 0.25) = angle * PI / 180 * web_t / (2 * strain);
// fold stiffness, N*mm per radian (E in MPa)
function flex_hinge_k(width = fh_width, gap = fh_gap, web_t = fh_web_t, E = 26) = E * width * pow(web_t, 3) / (12 * gap);

// the design rule: web strain at the design fold angle <= ~25 % (27 % allows gap = 3 * web_t)
module fh_check_strain(gap, web_t, fold)
    assert(flex_hinge_strain(gap, web_t, fold) <= 0.27,
           str("flex_hinge: gap too short: strain at ", fold, " deg would be ",
               round(100 * flex_hinge_strain(gap, web_t, fold)), " %; use gap >= ",
               ceil(10 * flex_hinge_min_gap(web_t, fold, 0.25)) / 10, " (or set fold to a smaller angle)"));

// rivet hole positions across the neck: as many as fit with min_petg walls
function fh_rivet_ys(neck_w, d, pitch) =
    let(n = d > 0 ? max(0, floor((neck_w - 2 * min_petg - d) / pitch) + 1) : 0)
    n > 0 ? [for (i = [0 : n - 1]) (i - (n - 1) / 2) * pitch] : [];

// ---------------------------------------------------------------------
// PETG anchor (edge frame: edge face on x = 0, part at x < 0, anchor +x)
// ---------------------------------------------------------------------
module flex_hinge_anchor(part = "petg", width = fh_width, t = fh_t, skin = fh_skin, side = fh_side,
                         neck_l = fh_neck_l, head_l = fh_head_l, head_over = fh_head_over,
                         rivet_d = fh_rivet_d, rivet_pitch = fh_rivet_pitch, overlap = fh_overlap) {
    head_w = width - 2 * side;
    neck_w = head_w - 2 * head_over;
    tongue = t - 2 * skin;
    assert(skin >= min_tpu, "flex_hinge: skin must be >= min_tpu");
    assert(side >= min_tpu, "flex_hinge: side must be >= min_tpu");
    assert(head_over >= min_tpu, "flex_hinge: head_over must be >= min_tpu");
    assert(tongue >= min_petg, str("flex_hinge: t must be >= ", 2 * skin + min_petg, " (two TPU skins + a PETG tongue)"));
    assert(head_l >= min_petg, "flex_hinge: head_l must be >= min_petg");
    assert(neck_w >= 3 * min_petg, str("flex_hinge: width too small, needs >= ", 2 * side + 2 * head_over + 3 * min_petg));
    assert(rivet_d == 0 || rivet_d >= 1.6, "flex_hinge: rivet holes must be >= 1.6 mm");
    assert(rivet_d == 0 || neck_l >= rivet_d + 2 * min_petg, "flex_hinge: neck_l too short for the rivet holes");
    if (part == "petg") fh_anchor_3d();
    else if (part == "both") color(petg_colour) fh_anchor_3d();

    module fh_anchor_3d()
        translate([0, 0, skin]) linear_extrude(tongue) difference() {
            union() {
                translate([-overlap, -neck_w / 2]) square([overlap + neck_l + eps, neck_w]);
                translate([neck_l, -head_w / 2]) square([head_l, head_w]);
            }
            for (y = fh_rivet_ys(neck_w, rivet_d, rivet_pitch))
                translate([neck_l / 2, y]) circle(d = rivet_d, $fn = 24);
        }
}

// ---------------------------------------------------------------------
// TPU region: two blocks + the web with root fillets (hinge frame:
// centred on x = 0, edges at x = +-e, y centred, sits on z = 0).
// This is the REGION: hybrid() subtracts the PETG anchors from it.
// ---------------------------------------------------------------------
module flex_hinge_tpu_region(part = "tpu", width = fh_width, gap = fh_gap, web_t = fh_web_t, t = fh_t,
                             fold = fh_fold, root_r = fh_root_r, neck_l = fh_neck_l, head_l = fh_head_l,
                             tip = fh_tip) {
    e = flex_hinge_edge_x(gap, root_r, neck_l, head_l, tip);
    g2 = gap / 2;
    r = root_r;
    nf = 8;
    assert(web_t >= min_tpu, "flex_hinge: web_t must be >= min_tpu (1.2)");
    assert(tip >= min_tpu, "flex_hinge: tip must be >= min_tpu");
    assert(r >= 0 && web_t + r <= t, "flex_hinge: root_r must be 0 .. t - web_t");
    fh_check_strain(gap, web_t, fold);
    face = web_t + r < t - eps;     // a vertical block face above the fillet?
    profile = concat(
        [[-e, 0], [e, 0], [e, t]],
        face ? [[g2 + r, t]] : [],
        r > 0 ? [for (i = [0 : nf]) let(a = -90 * i / nf) [g2 + r * cos(a), web_t + r + r * sin(a)]]
              : [[g2, web_t]],
        r > 0 ? [for (i = [0 : nf]) let(a = -90 - 90 * i / nf) [-g2 + r * cos(a), web_t + r + r * sin(a)]]
              : [[-g2, web_t]],
        face ? [[-g2 - r, t]] : [],
        [[-e, t]]);
    if (part != "petg")
        rotate([90, 0, 0]) linear_extrude(width, center = true) polygon(profile);
}

// ---------------------------------------------------------------------
// The hinge alone: both anchors + TPU, plate edges at x = +-e
// ---------------------------------------------------------------------
module flex_hinge_joint(part = "both", width = fh_width, gap = fh_gap, web_t = fh_web_t, t = fh_t,
                        fold = fh_fold, root_r = fh_root_r, skin = fh_skin, side = fh_side,
                        neck_l = fh_neck_l, head_l = fh_head_l, head_over = fh_head_over, tip = fh_tip,
                        rivet_d = fh_rivet_d, rivet_pitch = fh_rivet_pitch, overlap = fh_overlap) {
    e = flex_hinge_edge_x(gap, root_r, neck_l, head_l, tip);
    fh_check_strain(gap, web_t, fold);
    if (part == "both") {
        // each material rendered on its own: the F5 preview of the nested
        // difference inside hybrid() otherwise shows z-fighting artifacts
        color(petg_colour) render() fh_hybrid("petg");
        color(tpu_colour) render() fh_hybrid("tpu");
    } else fh_hybrid(part);

    module fh_hybrid(p) hybrid(p) {
        fh_anchors();
        flex_hinge_tpu_region("tpu", width = width, gap = gap, web_t = web_t, t = t, fold = fold,
                              root_r = root_r, neck_l = neck_l, head_l = head_l, tip = tip);
    }
    module fh_anchors() for (s = [-1, 1])
        translate([s * e, 0, 0]) mirror([s > 0 ? 1 : 0, 0, 0])
            flex_hinge_anchor("petg", width = width, t = t, skin = skin, side = side, neck_l = neck_l,
                              head_l = head_l, head_over = head_over, rivet_d = rivet_d,
                              rivet_pitch = rivet_pitch, overlap = overlap);
}

// ---------------------------------------------------------------------
// Complete hinge: two plates (A at -x, B at +x) joined by the hinge
// ---------------------------------------------------------------------
module flex_hinge(part = "both", width = fh_width, gap = fh_gap, web_t = fh_web_t, plate = [25, 20, 4],
                  plate_b = undef, holes_a = [], holes_b = [], hole_d = 3.4, label = "",
                  fold = fh_fold, root_r = fh_root_r) {
    pb = is_undef(plate_b) ? plate : plate_b;
    t = plate[2];
    e = flex_hinge_edge_x(gap, root_r);
    assert(pb[2] == t, "flex_hinge: both plates must be the same thickness");
    assert(width <= plate[1] && width <= pb[1], "flex_hinge: width must be <= the plate widths");
    assert(hole_d >= 1.6, "flex_hinge: holes must be >= 1.6 mm");
    echo(str("flex_hinge: width ", width, ", web ", web_t, " x ", gap, " mm, strain at ", fold, " deg ",
             round(1000 * flex_hinge_strain(gap, web_t, fold)) / 10, " %, fold stiffness ",
             round(flex_hinge_k(width, gap, web_t)), " N*mm/rad, plate edges at x = +-", e));
    if (part != "tpu") fh_colour(part) difference() {
        union() {
            translate([-e - plate[0], -plate[1] / 2, 0]) cube(plate);
            translate([e, -pb[1] / 2, 0]) cube(pb);
        }
        for (h = holes_a) translate([-e - h[0], h[1], -1]) cylinder(d = hole_d, h = t + 2, $fn = 32);
        for (h = holes_b) translate([e + h[0], h[1], -1]) cylinder(d = hole_d, h = t + 2, $fn = 32);
        if (label != "")
            translate([e + pb[0] * 0.45, 0, t - 0.6]) linear_extrude(1)
                text(label, size = min(5, pb[1] * 0.3), font = "DejaVu Sans:style=Bold",
                     halign = "center", valign = "center");
    }
    flex_hinge_joint(part, width = width, gap = gap, web_t = web_t, t = t, fold = fold, root_r = root_r);
}

// ---------------------------------------------------------------------
// Test piece: 90 deg flex-fatigue coupon
// ---------------------------------------------------------------------
module flex_hinge_coupon(part = "both", width = fh_width, gap = fh_gap, web_t = fh_web_t, t = fh_t) {
    flex_hinge(part, width = width, gap = gap, web_t = web_t, plate = [24, width, t], plate_b = [40, width, t],
               holes_a = [[8, 0], [18, 0]], holes_b = [[34, 0]], hole_d = 3.4,
               label = str("T", web_t, " L", gap));
}

// PETG-only helper: shows children for "petg", coloured for "both"
module fh_colour(part) {
    if (part == "both") color(petg_colour) children();
    else if (part == "petg") children();
}

// ---------------------------------------------------------------------
// Demo: a folding arm on a chassis mounting plate, built from your own
// PETG shapes + flex_hinge_joint() (a 30 mm wide hinge)
// ---------------------------------------------------------------------
module flex_hinge_demo(part = "both") {
    w = 30; t = 4;
    e = flex_hinge_edge_x();
    // chassis side: a mounting plate with two M3 holes
    fh_colour(part) difference() {
        translate([-e - 18, -21, 0]) cube([18, 42, t]);
        for (y = [-15, 15]) translate([-e - 9, y, -1]) cylinder(d = 3.4, h = t + 2, $fn = 32);
    }
    // the arm: 30 mm wide at the hinge, tapering to a rounded tip with a hole
    fh_colour(part) difference() {
        hull() {
            translate([e, -w / 2, 0]) cube([1, w, t]);
            translate([e + 52, 0, 0]) cylinder(r = 8, h = t, $fn = 64);
        }
        translate([e + 52, 0, -1]) cylinder(d = 3.4, h = t + 2, $fn = 32);
        hull() for (x = [e + 13, e + 36]) translate([x, 0, -1]) cylinder(d = 7, h = t + 2, $fn = 40);
    }
    flex_hinge_joint(part, width = w, t = t);
}

// Cut-away preview (not a print): the default hinge cut on y = 0 (through
// the middle rivet) and, on plate A's side, down to mid-tongue height to
// show the T-head and the rivet holes in plan.
module flex_hinge_section(part = "both") {
    for (p = ["petg", "tpu"]) if (part == "both" || part == p)
        color(p == "petg" ? petg_colour : tpu_colour) render() difference() {
            flex_hinge(p);
            translate([-100, -50, -1]) cube([200, 50, 10]);
            translate([-100, -eps, fh_t / 2]) cube([100, 50, 10]);
        }
}

// ---------------------------------------------------------------------
if (show == "demo") flex_hinge_demo(part);
else if (show == "test") flex_hinge_coupon(part);
else if (show == "section") flex_hinge_section(part);
