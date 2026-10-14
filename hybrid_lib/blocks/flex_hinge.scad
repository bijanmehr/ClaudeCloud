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
//   |                /               |           |               \                |
//   |      PETG      ==== tongue ==T |           | T== tongue ====      PETG      |
//   |                \                \_________/                /                |
//   +----------------+-------------------------------------------+----------------+ z = 0
//                     <-- block_l --> r<- gap ->r <-- block_l -->   (web: web_t thick, on the bed)
//   plan of one tongue (inside its TPU block; P = TPU pockets the hooks bear on):
//                 |PPP+--+PPP+--+
//        plate    |===|  |===|  |      the neck carries the bending; the barb
//        edge     |===|  |===|  |      and the T-head stick out beside it
//                 |PPP+--+PPP+--+
//                     barb    head
//
//   - Each plate edge grows a thin PETG tongue at mid-thickness (like bond
//     coupon F): a neck ending in a wider T-head, with `barbs` extra pairs
//     of hooks along the neck (a "fir tree", like the staggered ribs of the
//     ratchet band). The tongue is wrapped on every side by a TPU block the
//     full thickness of the plate: skins >= 1.2 mm above and below, side
//     walls beside the hooks, a tip wall in front of the head, and TPU
//     pockets behind every hook.
//     Pulled away from the plate, the TPU bears on the backs of the hooks;
//     pushed, it bears on the plate edge and on the head front; up/down and
//     sideways it is boxed in by its own skins and walls.
//     tools/capture_check.py confirms the TPU cannot slide out in any of 26
//     directions.
//   - The hooks stick out BESIDE the neck, so they do not weaken it. That
//     matters: with no adhesion the PETG tongue carries ~85 % of the fold
//     moment (2D FE, see ESTIMATES), so the neck's net section is the most
//     loaded PETG in every fold cycle. Rivet holes through the neck (coupon G) are
//     therefore optional and off by default; a small 45 deg chamfer at the
//     tongue root takes the sharp corner out of the most stressed spot.
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
//   on the bed). The tongue's underside and its lower root chamfer (45 deg)
//   are printed on the TPU skin, and the TPU skin on top of the tongue is
//   printed on PETG; nothing else hangs.
//   Print the TPU solid (100 % infill, or >= 4 walls and >= 5 top/bottom
//   layers) so the web and the skins have no voids; if your slicer lets you,
//   run the TPU's solid lines across the fold axis (along X).
//
// PARAMETERS (mm, degrees). Defaults and the rule behind each:
//   width     = 20   hinge width along the fold axis (Y). Stiffness and
//                    strength scale linearly with it. >= 2 * side +
//                    2 * head_over + 3 * 1.2 = 10; width <= the plate width.
//   gap       = 6    free web length (X), flat part between the two root
//                    fillets. Rule: bending strain at fold angle theta (rad)
//                       eps = theta * web_t / (2 * gap)
//                    so for eps <= ~25 % at 90 deg:  gap >= 3 * web_t
//                    (gap = 3 * web_t gives pi/12 = 26 %). Asserted against
//                    `fold`: eps(fold) <= pi/12. The fillets and the blocks
//                    add bending length, so the formula is on the safe side
//                    (FE: 21 % at 90 deg for the defaults).
//   web_t     = 2.0  web thickness (Z, 10 layers). >= 1.2 (min_tpu). Fold
//                    stiffness grows with web_t^3, strength with web_t, and
//                    at the strain limit the fold moment grows with web_t^2,
//                    which loads the PETG tongue (see "Tongue" below).
//   plate     = [25, 20, 4]  plate A [length X, width Y, thickness Z];
//                    plate_b defaults to the same. Thickness t is also the
//                    TPU block height. Rule: t >= 2 * skin + 1.2 = 3.6
//                    (1.2 mm TPU skins over a >= 1.2 mm PETG tongue).
//   fold      = 90   design fold angle used for the strain and tongue checks.
//   root_r    = 1.0  fillet radius where the web meets each block (<= t - web_t).
//   skin      = 1.2  TPU above and below the tongue (min_tpu, 6 layers).
//                    The tongue is t - 2 * skin thick (1.6 at t = 4).
//   side      = 1.6  TPU wall beside the hooks (Y). >= min_tpu.
//   neck_l    = 5.0  tongue neck length from the plate edge (X). It holds
//                    2 * barbs + 1 equal segments (pocket, barb, pocket,
//                    ...), each >= 1.2: neck_l >= 1.2 * (2 * barbs + 1).
//   head_l    = 2.0  T-head length (X). >= min_petg.
//   head_over = 1.6  how far the head and barbs stick out past the neck on
//                    each side (Y): the hooks the TPU pockets bear on.
//                    >= min_tpu so the pockets are 3 lines wide.
//   barbs     = 1    extra hook pairs along the neck (0 = T-head only).
//                    Each pair adds as much pull-out bearing as the head.
//   tip       = 1.6  TPU wall in front of the head (X). >= min_tpu.
//   chamfer   = 0.6  45 deg chamfer at the tongue root, top and bottom
//                    (3 layers). Cuts the root-corner stress peak by ~16 %
//                    (FE). <= skin - 0.6, so >= 0.6 mm TPU stays at the face.
//   rivet_d   = 0    optional TPU-filled holes through the neck (coupon G);
//                    >= 1.6 when used (params rivet_d = 2.4). Off by default:
//                    each hole cuts the neck's bending section (3 x 2.4 mm
//                    holes would double the tongue stress, see ESTIMATES).
//                    Use them for links that see pull more than folding.
//   rivet_pitch = 4.0  hole spacing (Y) when rivet_d > 0; as many as fit
//                    with >= 1.2 mm PETG to the neck edges.
//   overlap   = 1.0  how far the anchor reaches back into your part, so the
//                    two fuse into one PETG body.
//   holes_a, holes_b = []  mounting holes in plate A / B as [distance from
//                    the hinge edge, y]; hole_d = 3.4 (M3 clearance).
//   label     = ""   text engraved 0.6 mm into the top of plate B.
//   Derived: block_l = neck_l + head_l + tip = 8.6 (each TPU block, X);
//   the plate edges sit at x = +-e, e = gap/2 + root_r + block_l = 12.6
//   (flex_hinge_edge_x()); neck width = width - 2 * side - 2 * head_over
//   = 13.6.
//
// ESTIMATES (defaults: width w = 20, web_t h = 2, gap L = 6, t = 4;
//            TPU 95A E ~ 26 MPa, G ~ 9 MPa; PETG E ~ 2000 MPa, ~45 MPa).
//   Hand formulas, cross-checked with a 2D plane-strain FE (CalculiX) of
//   the XZ section with no PETG/TPU adhesion on the faces that open.
//   Fold stiffness   k = E * w * h^3 / (12 * L) = 26 * 20 * 8 / 72
//                      = 58 N*mm/rad (~1.0 N*mm per degree)
//                      (flex_hinge_k()). FE: 47 N*mm/rad, the TPU blocks
//                      and fillets add some compliance. Expect 40-60.
//   Moment at 90 deg M = k * pi/2 = 91 N*mm (FE 74): ~3.6 N pushing 25 mm
//                      from the fold. Linear theory; TPU softens a little
//                      at 20 % strain, so the real value is a bit lower.
//   Strain at 90 deg eps = (pi/2) * h / (2 L) = 26 %; FE peak 21 %, at the
//                      toe of the root fillets.
//   Tongue (PETG)    the tongue carries a share s ~ 0.85 of the fold moment
//                      (FE: 0.74 bonded .. 0.86 unbonded), as plate bending
//                      through its net neck section b x tongue:
//                      sigma = 6 * s * M / (b * tongue^2)
//                            = 6 * 0.85 * 91 / (13.6 * 1.6^2) = 13 MPa
//                      (flex_hinge_tongue_stress(); FE: 11 MPa nominal,
//                      17 MPa peak at the chamfer toe, 21 without the
//                      chamfer). About 1/3 of PETG strength, the fatigue
//                      budget: the module echoes a warning above 15 MPa
//                      and refuses above 22.5 MPa (1/2 strength). Three
//                      2.4 mm rivet holes (b = 6.4) would give 28 MPa plus
//                      the holes' own stress peak: a fatigue crack across
//                      the rivet row before the web wears out.
//                      At the strain limit M = E w h^2 eps / 6, so sigma ~
//                      (web_t / tongue)^2: a thicker web needs a thicker
//                      plate (or a smaller fold angle).
//   Fold-up limit    folding so the top faces close, the blocks touch at
//                      theta ~ (gap + 2 root_r) / (t - web_t/2) = 2.7 rad
//                      (~150 deg); folding the other way is limited only by
//                      the strain.
//   Axial (pull)     k_x = E * w * h / L = 26 * 40 / 6 = 170 N/mm (web only).
//   Sideways shear   k_y = G * w * h / L = 9 * 40 / 6 = 60 N/mm.
//   Yaw (in-plane)   E * (h * w^3 / 12) / L = 5800 N*mm/rad: 100x the fold
//                      stiffness ((w/h)^2), so it folds, it does not skew.
//   Twist (about X)  G * (w * h^3 / 3) / L = 9 * 53 / 6 = 80 N*mm/rad: as
//                      soft as the fold. Use two hinges spaced apart (or one
//                      wide one) where twist matters.
//   Web in tension   section w * h = 40 mm^2; at a working stress of 2.5 MPa
//                      (~10 % strain) 100 N; TPU breaks at ~25-35 MPa
//                      (1000+ N) after stretching several times its length.
//   Anchor pull-out  (no adhesion) bearing on the backs of the hooks:
//                      (1 + barbs) * 2 * head_over * tongue = 2 * 2 * 1.6 *
//                      1.6 = 10 mm^2; if the TPU around the hooks squeezes
//                      past them at ~8-12 MPa mean bearing, roughly
//                      80-120 N per anchor (each optional rivet adds
//                      rivet_d * tongue = 3.8 mm^2, ~40 N). A PETG barb (a
//                      1.6 mm cantilever, 1.67 x 1.6 root) breaks at ~40 N,
//                      more than its ~25 N share at that load. Measure your
//                      own with the bond-test pull coupons or this coupon.
//   Weakest link:  in FOLDING (the hinge's job) the TPU web works at its
//   ~25 % strain design limit and the PETG tongue at ~1/3 of its strength
//   every cycle; the coupon's fatigue test tells which goes first on your
//   printer (a whitened web, or a click and a sudden loss of stiffness from
//   a cracked tongue). In a straight PULL
//   the anchors let go (~100 N) long before the web breaks, so do not use
//   the hinge as a tension link above ~30-50 N working load; add a tether.
//
// TEST PIECE (show = "test"): flex_hinge_coupon(), a 90 deg flex-fatigue
//   coupon. Plate A (24 mm, two M3 holes) bolts or clamps flat with its
//   edge flush with the bench edge; plate B is a 40 mm lever with an M3
//   hole near its end for a servo horn link or a string. Fold B up to 90 deg
//   and back (by hand, or a servo at ~1 Hz) and count cycles. Every 1000
//   cycles look for: whitening or a crack across the web (record the
//   count), a gap opening between a TPU block and its plate, a click or
//   a sudden drop in stiffness (a cracked tongue), the head showing
//   through the skin. Then pull it apart with a luggage scale.
//   The web size is engraved on plate B ("T<web_t> L<gap>").
//
// MODULES (all take `part` = "petg" | "tpu" | "both" first)
//   flex_hinge(part, width, gap, web_t, plate, ...)   two plates + hinge
//   flex_hinge_joint(part, width, gap, web_t, t, ...) { your PETG parts }
//       the hinge alone (both anchors + TPU) between plate edges at
//       x = +-flex_hinge_edge_x(). Pass your own PETG parts (at |x| >= e)
//       as children: they are fused with the anchors and subtracted from
//       the TPU, so the two materials never overlap.
//   flex_hinge_anchor(part, width, t, ...)  one PETG anchor; edge frame:
//       the edge face on x = 0, your part on x < 0, the anchor points +x.
//       Union it with your PETG body.  ("tpu" draws nothing.)
//   flex_hinge_tpu_region(part, width, gap, web_t, t, ...)  the TPU REGION
//       (blocks + web, anchors NOT removed) for child(1) of hybrid().
//       ("petg" draws nothing.)
//   flex_hinge_coupon(part, ...)  the fatigue test coupon.
//   Functions: flex_hinge_edge_x(), flex_hinge_strain(), flex_hinge_min_gap(),
//       flex_hinge_k(), flex_hinge_tongue_stress().
//
// RULES FOR YOUR OWN PARTS
//   - Keep your PETG out of the hinge zone |x| < e, and give each part a
//     solid edge face at x = -e / +e at least `width` wide that covers
//     z = 0..t: the anchor root reaches `overlap` (1 mm) back into it.
//   - Parts may be thicker than t (the hinge sits at the bottom) but not
//     thinner.
//   - The two parts must not touch anywhere else, or they stop being two
//     bodies; give them xy_clear (0.4 mm) where they come close.
//   - Pass the same anchor parameters to flex_hinge_anchor() and
//     flex_hinge_tpu_region() if you use them separately.
//
// LIMITATIONS
//   - Not a precision pivot: the fold axis moves with the bending and the
//     hinge gives a little in tension and twist. Use a pin hinge to locate.
//   - TPU creeps: a hinge held folded for weeks takes a set; one that
//     carries a constant load (e.g. a leg spring under the robot's weight)
//     will sag over time.
//   - Fatigue life of printed TPU and of the thin PETG tongue is not known
//     in advance; run the coupon. The estimates above come from hand
//     formulas and a linear 2D FE (one XZ section, no plan-view effects
//     such as the stress raisers at the hook shoulders), not from tests.
//   - At 0.2 mm layers the 0.6 mm root chamfer prints as 2-3 small steps.
//   - Engraved labels leave small PETG islands inside letters such as 0,
//     6 or 8 (< 1.2 mm wide, resting on the plate): cosmetic only.
//   - The capture check treats the TPU as rigid. A large enough pull can
//     still squeeze the TPU past the hooks (see "Anchor pull-out").
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
fh_barbs = 1;
fh_tip = 1.6;
fh_chamfer = 0.6;
fh_rivet_d = 0;             // off; params rivet_d (2.4) when wanted
fh_rivet_pitch = 4.0;
fh_overlap = 1.0;

// design limits
fh_strain_max = PI / 12;    // web strain: gap = 3 * web_t at 90 deg (26 %)
fh_share = 0.85;            // share of the fold moment the PETG tongue carries (FE)
fh_petg_strength = 45;      // MPa
fh_sigma_warn = fh_petg_strength / 3;   // 15 MPa: fatigue budget
fh_sigma_max = fh_petg_strength / 2;    // 22.5 MPa: refused above this

// ---------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------
// distance from the hinge centre to each plate edge
function flex_hinge_edge_x(gap = fh_gap, root_r = fh_root_r, neck_l = fh_neck_l, head_l = fh_head_l, tip = fh_tip) =
    gap / 2 + root_r + neck_l + head_l + tip;
// peak bending strain (fraction) in the web folded to `angle` degrees
function flex_hinge_strain(gap = fh_gap, web_t = fh_web_t, angle = fh_fold) = angle * PI / 180 * web_t / (2 * gap);
// shortest free web length for a strain limit at a fold angle
function flex_hinge_min_gap(web_t = fh_web_t, angle = fh_fold, strain = fh_strain_max) =
    angle * PI / 180 * web_t / (2 * strain);
// fold stiffness, N*mm per radian (E in MPa)
function flex_hinge_k(width = fh_width, gap = fh_gap, web_t = fh_web_t, E = 26) = E * width * pow(web_t, 3) / (12 * gap);
// rivet hole positions across the neck: as many as fit with min_petg walls
function fh_rivet_ys(neck_w, d, pitch) =
    let(n = d > 0 ? max(0, floor((neck_w - 2 * min_petg - d) / pitch) + 1) : 0)
    n > 0 ? [for (i = [0 : n - 1]) (i - (n - 1) / 2) * pitch] : [];
// nominal bending stress (MPa) in the PETG tongue's net neck section with the
// hinge folded to `fold` degrees: 6 * s * k * theta / (b_net * tongue^2)
function flex_hinge_tongue_stress(width = fh_width, gap = fh_gap, web_t = fh_web_t, t = fh_t, fold = fh_fold,
                                  skin = fh_skin, side = fh_side, head_over = fh_head_over,
                                  rivet_d = fh_rivet_d, rivet_pitch = fh_rivet_pitch) =
    let(neck_w = width - 2 * side - 2 * head_over,
        b = neck_w - len(fh_rivet_ys(neck_w, rivet_d, rivet_pitch)) * rivet_d,
        tongue = t - 2 * skin)
    6 * fh_share * flex_hinge_k(width, gap, web_t) * fold * PI / 180 / (b * tongue * tongue);

// the design rule: web strain at the design fold angle <= pi/12 (gap >= 3 * web_t at 90 deg)
module fh_check_strain(gap, web_t, fold)
    assert(flex_hinge_strain(gap, web_t, fold) <= fh_strain_max + 1e-9,
           str("flex_hinge: gap too short: strain at ", fold, " deg would be ",
               round(100 * flex_hinge_strain(gap, web_t, fold)), " % (limit 26 %); use gap >= ",
               ceil(10 * flex_hinge_min_gap(web_t, fold) - 1e-6) / 10, " (or set fold to a smaller angle)"));

// the PETG tongue must not be the part that fails when the hinge folds
module fh_check_tongue(width, gap, web_t, t, fold, skin, side, head_over, rivet_d, rivet_pitch) {
    s = flex_hinge_tongue_stress(width, gap, web_t, t, fold, skin, side, head_over, rivet_d, rivet_pitch);
    hint = " MPa; use a thinner web, a thicker plate (t), fewer or no rivet holes, or a smaller fold";
    assert(s <= fh_sigma_max, str("flex_hinge: PETG tongue overloaded: ~", round(s), " MPa at ", fold,
                                  " deg (limit ", fh_sigma_max, hint));
    if (s > fh_sigma_warn)
        echo(str("WARNING: flex_hinge: PETG tongue at ~", round(s), " MPa at ", fold,
                 " deg, above the fatigue budget of ", fh_sigma_warn, hint));
}

// ---------------------------------------------------------------------
// PETG anchor (edge frame: edge face on x = 0, part at x < 0, anchor +x)
// ---------------------------------------------------------------------
module flex_hinge_anchor(part = "petg", width = fh_width, t = fh_t, skin = fh_skin, side = fh_side,
                         neck_l = fh_neck_l, head_l = fh_head_l, head_over = fh_head_over, barbs = fh_barbs,
                         chamfer = fh_chamfer, rivet_d = fh_rivet_d, rivet_pitch = fh_rivet_pitch,
                         overlap = fh_overlap) {
    head_w = width - 2 * side;
    neck_w = head_w - 2 * head_over;
    tongue = t - 2 * skin;
    seg = neck_l / (2 * barbs + 1);          // pocket, barb, pocket, ... along the neck
    assert(skin >= min_tpu, "flex_hinge: skin must be >= min_tpu");
    assert(side >= min_tpu, "flex_hinge: side must be >= min_tpu");
    assert(head_over >= min_tpu, "flex_hinge: head_over must be >= min_tpu");
    assert(tongue >= min_petg, str("flex_hinge: t must be >= ", 2 * skin + min_petg, " (two TPU skins + a PETG tongue)"));
    assert(head_l >= min_petg, "flex_hinge: head_l must be >= min_petg");
    assert(neck_w >= 3 * min_petg, str("flex_hinge: width too small, needs >= ", 2 * side + 2 * head_over + 3 * min_petg));
    assert(barbs >= 0 && barbs == floor(barbs), "flex_hinge: barbs must be 0, 1, 2, ...");
    assert(seg >= max(min_tpu, min_petg) - 1e-9,
           str("flex_hinge: neck_l too short for ", barbs, " barbs, needs >= ", 1.2 * (2 * barbs + 1)));
    assert(chamfer >= 0 && chamfer <= skin - 3 * layer_h + 1e-9, "flex_hinge: chamfer must be 0 .. skin - 0.6");
    assert(rivet_d == 0 || rivet_d >= 1.6, "flex_hinge: rivet holes must be >= 1.6 mm");
    assert(rivet_d == 0 || neck_l >= rivet_d + 2 * min_petg, "flex_hinge: neck_l too short for the rivet holes");
    if (part == "petg") fh_anchor_3d();
    else if (part == "both") color(petg_colour) fh_anchor_3d();

    module fh_anchor_3d() {
        translate([0, 0, skin]) linear_extrude(tongue) difference() {
            union() {
                translate([-overlap, -neck_w / 2]) square([overlap + neck_l + eps, neck_w]);
                translate([neck_l, -head_w / 2]) square([head_l, head_w]);
                if (barbs > 0) for (i = [1 : barbs])
                    translate([(2 * i - 1) * seg, -head_w / 2]) square([seg, head_w]);
            }
            for (y = fh_rivet_ys(neck_w, rivet_d, rivet_pitch))
                translate([neck_l / 2, y]) circle(d = rivet_d, $fn = 24);
        }
        // 45 deg root chamfers above and below the tongue, across the neck
        if (chamfer > 0)
            rotate([90, 0, 0]) linear_extrude(neck_w, center = true)
                polygon([[-overlap, skin - chamfer], [0, skin - chamfer], [chamfer, skin], [chamfer, t - skin],
                         [0, t - skin + chamfer], [-overlap, t - skin + chamfer]]);
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
// The hinge alone: both anchors + TPU, plate edges at x = +-e.
// Children (optional) = your PETG parts: fused with the anchors and
// subtracted from the TPU.
// ---------------------------------------------------------------------
module flex_hinge_joint(part = "both", width = fh_width, gap = fh_gap, web_t = fh_web_t, t = fh_t,
                        fold = fh_fold, root_r = fh_root_r, skin = fh_skin, side = fh_side,
                        neck_l = fh_neck_l, head_l = fh_head_l, head_over = fh_head_over, barbs = fh_barbs,
                        tip = fh_tip, chamfer = fh_chamfer, rivet_d = fh_rivet_d, rivet_pitch = fh_rivet_pitch,
                        overlap = fh_overlap) {
    e = flex_hinge_edge_x(gap, root_r, neck_l, head_l, tip);
    fh_check_strain(gap, web_t, fold);
    fh_check_tongue(width, gap, web_t, t, fold, skin, side, head_over, rivet_d, rivet_pitch);
    if (part == "both") {
        // each material rendered on its own: the F5 preview of the nested
        // difference inside hybrid() otherwise shows z-fighting artifacts
        color(petg_colour) render() fh_hybrid("petg") children();
        color(tpu_colour) render() fh_hybrid("tpu") children();
    } else fh_hybrid(part) children();

    module fh_hybrid(p) hybrid(p) {
        union() {
            fh_anchors();
            children();
        }
        flex_hinge_tpu_region("tpu", width = width, gap = gap, web_t = web_t, t = t, fold = fold,
                              root_r = root_r, neck_l = neck_l, head_l = head_l, tip = tip);
    }
    module fh_anchors() for (s = [-1, 1])
        translate([s * e, 0, 0]) mirror([s > 0 ? 1 : 0, 0, 0])
            flex_hinge_anchor("petg", width = width, t = t, skin = skin, side = side, neck_l = neck_l,
                              head_l = head_l, head_over = head_over, barbs = barbs, chamfer = chamfer,
                              rivet_d = rivet_d, rivet_pitch = rivet_pitch, overlap = overlap);
}

// ---------------------------------------------------------------------
// Complete hinge: two plates (A at -x, B at +x) joined by the hinge
// ---------------------------------------------------------------------
module flex_hinge(part = "both", width = fh_width, gap = fh_gap, web_t = fh_web_t, plate = [25, 20, 4],
                  plate_b = undef, holes_a = [], holes_b = [], hole_d = 3.4, label = "",
                  fold = fh_fold, root_r = fh_root_r, neck_l = fh_neck_l, barbs = fh_barbs, rivet_d = fh_rivet_d) {
    pb = is_undef(plate_b) ? plate : plate_b;
    t = plate[2];
    e = flex_hinge_edge_x(gap, root_r, neck_l);
    assert(pb[2] == t, "flex_hinge: both plates must be the same thickness");
    assert(width <= plate[1] && width <= pb[1], "flex_hinge: width must be <= the plate widths");
    assert(hole_d >= 1.6, "flex_hinge: holes must be >= 1.6 mm");
    echo(str("flex_hinge: width ", width, ", web ", web_t, " x ", gap, " mm, strain at ", fold, " deg ",
             round(1000 * flex_hinge_strain(gap, web_t, fold)) / 10, " %, fold stiffness ",
             round(flex_hinge_k(width, gap, web_t)), " N*mm/rad, PETG tongue ~",
             round(flex_hinge_tongue_stress(width, gap, web_t, t, fold, rivet_d = rivet_d)),
             " MPa at ", fold, " deg, plate edges at x = +-", e));
    flex_hinge_joint(part, width = width, gap = gap, web_t = web_t, t = t, fold = fold, root_r = root_r,
                     neck_l = neck_l, barbs = barbs, rivet_d = rivet_d)
        difference() {
            union() {
                translate([-e - plate[0], -plate[1] / 2, 0]) cube(plate);
                translate([e, -pb[1] / 2, 0]) cube(pb);
            }
            for (h = holes_a) translate([-e - h[0], h[1], -1]) cylinder(d = hole_d, h = t + 2, $fn = 32);
            for (h = holes_b) translate([e + h[0], h[1], -1]) cylinder(d = hole_d, h = t + 2, $fn = 32);
            if (label != "")
                translate([e + pb[0] * 0.45, 0, t - 0.6]) linear_extrude(1)
                    text(label, size = min(5, pb[1] * 0.3), font = "DejaVu Sans:style=Bold", spacing = 1.15,
                         halign = "center", valign = "center");
        }
}

// ---------------------------------------------------------------------
// Test piece: 90 deg flex-fatigue coupon
// ---------------------------------------------------------------------
module flex_hinge_coupon(part = "both", width = fh_width, gap = fh_gap, web_t = fh_web_t, t = fh_t) {
    flex_hinge(part, width = width, gap = gap, web_t = web_t, plate = [24, width, t], plate_b = [40, width, t],
               holes_a = [[8, 0], [18, 0]], holes_b = [[34, 0]], hole_d = 3.4,
               label = str("T", web_t, " L", gap));
}

// ---------------------------------------------------------------------
// Demo: a folding arm on a chassis mounting plate, built from your own
// PETG shapes + flex_hinge_joint() (a 30 mm wide hinge)
// ---------------------------------------------------------------------
module flex_hinge_demo(part = "both") {
    w = 30; t = 4;
    e = flex_hinge_edge_x();
    flex_hinge_joint(part, width = w, t = t) {
        // chassis side: a mounting plate with two M3 holes
        difference() {
            translate([-e - 18, -21, 0]) cube([18, 42, t]);
            for (y = [-15, 15]) translate([-e - 9, y, -1]) cylinder(d = 3.4, h = t + 2, $fn = 32);
        }
        // the arm: 30 mm wide at the hinge, tapering to a rounded tip with a hole
        difference() {
            hull() {
                translate([e, -w / 2, 0]) cube([1, w, t]);
                translate([e + 52, 0, 0]) cylinder(r = 8, h = t, $fn = 64);
            }
            translate([e + 52, 0, -1]) cylinder(d = 3.4, h = t + 2, $fn = 32);
            hull() for (x = [e + 13, e + 36]) translate([x, 0, -1]) cylinder(d = 7, h = t + 2, $fn = 40);
        }
    }
}

// Cut-away preview (not a print): the default hinge cut on y = 0 and, on
// plate A's side, down to mid-tongue height to show the T-head and the
// barbs in plan.
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
