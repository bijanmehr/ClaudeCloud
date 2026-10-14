include <../params.scad>
// =======================================================================
// skin_anchor: a TPU skin / pad on a PETG surface that cannot peel off
// =======================================================================
// WHAT IT IS FOR
//   Grippy feet, bumpers, gripper-finger pads, anti-slip linings and
//   protective skins on a PETG robot chassis or drone frame, printed in the
//   same print as the PETG (H2D: PETG left nozzle, TPU 95A right nozzle).
//   PETG and TPU do not reliably stick to each other, so the pad is held by
//   SHAPE ONLY - it would stay put with zero adhesion.
//
// HOW IT WORKS (vertical section through one edge of the pad)
//
//                    proud TPU (contact face), top edge chamfered by ch
//           lip_w    ___________________________________________
//          |<--->|  /
//   +------+-----+-/ - - - - - - - - - - - - - - - - - - - - - -  rim top
//   |  rim |  lip|        TPU                     <- lip (lip_t) printed ON the TPU
//   |      +-----+      ______         ______         ______
//   |      |            \    /         \    /         \    /   <- dovetail ribs (rib_h):
//   |      |     TPU     \__/    TPU    \__/    TPU    \__/       wide head, narrow neck
//   +======+===================================================  PETG face (z = 0)
//    rim_w     skin_t = TPU height from the face up to the lip
//
//   (a) RIM + LIP: a PETG wall around the pad; its top layers reach inwards
//       over the TPU edge by lip_w. The lip is printed on top of the TPU,
//       which supports it (no bridge, no support). The TPU flange under the
//       lip cannot lift, and the rim walls stop it sliding sideways.
//   (b) DOVETAIL RIBS: PETG ribs stand up from the face into the TPU, narrow
//       at the face (neck) and wide at the top (head), so the TPU between
//       them is a dovetail key that cannot lift. The ribs are broken into
//       segments with TPU-filled gaps, staggered row to row (brick pattern),
//       so the TPU also wraps over and between them and cannot slide along
//       them. They hold the middle of the pad down, so a peel that starts at
//       an edge has to tear TPU at every rib it reaches.
//   The TPU body is (TPU region) minus (PETG), so the faces match exactly.
//   tools/check_block.py confirms the rigid TPU body cannot slide out in any
//   of 26 directions. Each feature also captures the pad on its own (checked
//   with ribs = false, and with the lip cut away), so either one is a backup
//   for the other. Pictures: images/skin_anchor.png (demo),
//   images/skin_anchor_section.png (quarter cut-away: lip and ribs),
//   images/skin_anchor_test.png (peel coupon).
//
// PUBLIC MODULES (all sizes in mm; pad centred on x = y = 0)
//   skin_pad(part, size, base_t, ...)
//       Complete part: PETG plate (z = 0 .. base_t) with the pad on its top
//       face, optional plate margin and screw holes.
//   skin_anchor(part, size, ...)
//       The pad alone with its rim/lip/ribs, standing on z = 0. Put it on any
//       flat, upward-facing PETG face of your own part:
//           if (part != "tpu") my_body();          // top face at z = h
//           translate([x, y, h]) skin_anchor(part, [30, 15]);
//       (its PETG reaches eps below z = 0 so it fuses with your body; the TPU
//       starts at z = 0, so it never overlaps a body that ends at z <= h).
//   skin_anchor_petg(size, ...)   raw PETG geometry (rim + lip + ribs)
//   skin_tpu_region(size, ...)    raw TPU region (pocket + proud pad)
//       Geometry helpers for params.scad's hybrid(), so they take no `part`:
//           hybrid(part) {
//               union() { my_body(); translate(p) skin_anchor_petg(s); }  // PETG
//               translate(p) skin_tpu_region(s);                         // TPU
//           }
//       Recessed (flush) variant: cut the pocket into a thick body first:
//               union() { difference() { my_body(); translate(p) skin_tpu_region(s); }
//                         translate(p) skin_anchor_petg(s); }
//       (p = a point on the pocket floor, skin_t + lip_t below the body's
//       flat top face; the body itself becomes the rim and lip.)
//   Your face must be flat and at least size + 2 * rim_w across, or the rim
//   would hang over its edge (and start in mid-air).
//   skin_peel_coupon(part, ...)
//       Test piece comparable to bond_tests coupon P: same 12 mm wide,
//       35 mm long pad, a 1.6 mm TPU tail down the plate end and along the
//       bed, 6 mm holes in the plate and the tail. Clamp the plate, pull the
//       tail straight up with a luggage scale, note the peak reading and how
//       it failed; compare with P (bonded only).
//
// PARAMETERS (same names in every module; defaults in brackets)
//   size      [40, 20]  pad footprint inside the rim = TPU area on the PETG
//                       face. Any size >= ~10 x 10; ribs fill what fits.
//   base_t    3         PETG plate thickness (skin_pad / coupon only);
//                       15 layers, stiff enough not to bend when peeled.
//   skin_t    2.0       TPU thickness under the lip (pocket depth, face to
//                       lip). Rule: >= rib_h + 0.6 (3 TPU layers over the
//                       ribs) and >= min_tpu.
//   lip_t     1.2       lip thickness = min_petg (6 layers). Thinner lips
//                       bend open (strength ~ lip_t^2, see below).
//   lip_w     1.6       how far the lip reaches over the TPU edge (4 lines).
//                       More = better capture, less exposed TPU.
//   rim_w     2.0       PETG rim wall width, >= min_petg (5 lines).
//   proud     1.0       TPU standing above the rim top (the contact face).
//                       0 = skin flush with the rim. >= 0.6 if used.
//   ch        0.6       45 deg chamfer on the top edge of the proud TPU, so
//                       edges do not catch and start a peel (clamped to proud).
//   r         3         pocket corner radius; the rim outside is r + rim_w,
//                       the lip opening r - lip_w (keep r >= lip_w).
//   rib_h     anchor_depth (1.2)   rib height above the face.
//   rib_neck  anchor_neck (1.3)    rib width at the face (>= min_petg).
//   rib_head  anchor_head (2.3)    rib width at the top. Flank overhang
//                       atan((head - neck) / 2 / rib_h) = 23 deg (<= 45).
//   rib_pitch 4.0       rib row spacing (along y). Rule: pitch - head >=
//                       min_tpu, so the TPU key between heads is >= 3 lines
//                       (default 1.7 mm); the key is 2.7 mm wide at the face.
//   seg_l     5.0       rib segment length (along x).
//   rib_gap   1.6       TPU gap between segments (>= anchor_gap_z = 1.2;
//                       1.6 gives each TPU post 4 lines). Odd rows are
//                       shifted half a period (brick pattern).
//   rib_inset 2.0       clear TPU between the pocket wall and the ribs
//                       (>= min_tpu; about lip_w, so ribs begin where the
//                       lip stops and the flange under the lip stays skin_t).
//   ribs      true      false = rim + lip only (tiny pads, thin strips).
//   skin_pad only:  margin 0 (scalar or [x, y]) extra plate outside the rim;
//                   holes [] list of [x, y] hole centres (must lie in the
//                   margin, not under the pad); hole_d 3.4 (M3 clearance).
//   coupon only:    grip_l 15 (plate before the pad, holds the 6 mm hole),
//                   tab_t 1.6 and tab_w = size.y (as coupon P), tail_l 40,
//                   tab_overlap 8 (tab fused onto the pad top), tab_wall 2
//                   (TPU wall down the plate end), hole_d 6.
//
// STIFFNESS / STRENGTH ESTIMATE (defaults, 40 x 20 pad)
//   TPU 95A: E ~ 26 MPa, G ~ 9 MPa; printed tensile strength assumed
//   ~20 MPa (conservative). PETG: E ~ 2000 MPa, ~45 MPa strength.
//   Contact area A = lip opening ~ 36.8 x 16.8 ~ 615 mm^2;
//   TPU above the rib tops t = skin_t - rib_h + lip_t + proud = 3.0 mm.
//   * Shear (foot pushed sideways): k_s = G A / t = 9 * 615 / 3.0
//     ~ 1.8 kN/mm -> 10 N sideways moves the face ~ 5 um.
//   * Compression: k_c >= E A / t_total = 26 * 615 / 4.2 ~ 3.8 kN/mm (lower
//     bound; the pocket confines the TPU, so the real value is higher).
//     => the pad is for GRIP and impact/scuff protection, not suspension
//     travel. Use a TPU spring block for compliance.
//   * Lip (PETG cantilever, bent within each layer, so no layer-bond
//     weakness): per mm of edge  F' = sigma_PETG * lip_t^2 / (3 lip_w)
//     = 45 * 1.44 / 4.8 ~ 13 N/mm before it yields -> ~160 N on the 12 mm
//     coupon edge, ~1.6 kN round the 40 x 20 pad.
//   * Rib keys (TPU): to lift the skin the TPU key between two rib heads
//     must tear at its narrow top: F' ~ sigma_TPU * (pitch - head)
//     = 20 * 1.7 ~ 34 N per mm of rib row.
//   * The TPU itself is the weak link: the coupon's 12 x 1.6 mm tail
//     breaks at ~ 20 * 19 ~ 380 N and stretches a lot well before that.
//     Coupon P measures the bonded-only peel of the same 12 mm strip (the
//     owner found that bond unreliable); this coupon should hold far more,
//     and ideally fail by stretching/tearing the tail, not at the anchor.
//   These are rigid-body/hand estimates: soft TPU can still squeeze out
//   from under a lip if pulled hard enough. Measure with the coupon.
//
// RECOMMENDED USES
//   Rover: feet / skid pads under the chassis, bumper faces on the nose and
//   corners, anti-slip lining in battery or payload trays, gripper pads.
//   Drone: landing-gear foot pads, belly skid plate, arm-tip bumpers, soft
//   battery-strap seats. Make the face whose pad you want point UP (+z) in
//   the print orientation.
//
// PRINTING / RULES THIS BLOCK FOLLOWS
//   No supports. The lip's first layer is printed on the TPU (it sticks
//   poorly to it, which does not matter). Rib flanks overhang 23 deg. Every
//   wall >= 1.2 mm, every TPU gap >= 1.2 mm. Every layer from the face to
//   the rim top holds both materials -> about one nozzle change per layer
//   (the slicer alternates the order) over (skin_t + lip_t) / 0.2 = 16
//   layers with the defaults (PrusaSlicer test: 19 changes for the demo).
//
// LIMITATIONS
//   * Flat, rounded-rectangle pads on an upward-facing (+z) face only. Pads
//     on side walls or on the bed-facing side need a different anchor
//     (a lip there would be a PETG overhang without TPU under it).
//   * Ribs run along x only; rotate the pad if the main load runs along y.
//   * The rim stands rim_w around and skin_t + lip_t above the face; the
//     usable TPU face is the lip opening, size - 2 * lip_w.
//   * Rigid capture is checked, not soft pull-out; a hard enough peel can
//     still stretch the TPU out of a small pad. Test with the coupon.
//   * Thin pads (skin_t < rib_h + 0.6) are refused: use ribs = false.
// =======================================================================

part = "both";   // [petg, tpu, both]
show = "demo";   // [demo, test]

skin_fn = 48;

// -----------------------------------------------------------------------
// 2D helpers
// -----------------------------------------------------------------------
module _skin_rrect(s, r) {
    if (r > 0.01) offset(r = r, $fn = skin_fn) square([s[0] - 2 * r, s[1] - 2 * r], center = true);
    else square(s, center = true);
}
module _skin_pocket_2d(size, r) _skin_rrect(size, r);
module _skin_opening_2d(size, lip_w, r) _skin_rrect(size - [2, 2] * lip_w, max(r - lip_w, 0));
module _skin_outer_2d(size, rim_w, r) _skin_rrect(size + [2, 2] * rim_w, r + rim_w);

// rib rows (y positions), centred, inside the pocket by rib_inset
function _skin_rows(size, rib_inset, rib_head, rib_pitch) =
    let(span = size[1] - 2 * rib_inset - rib_head,
        n = span < 0 ? 0 : floor(span / rib_pitch + 1e-6) + 1)
    n == 0 ? [] : [for (j = [0 : n - 1]) (j - (n - 1) / 2) * rib_pitch];

// rib segments [x0, x1] of one row; o = stagger offset; pieces < 2 mm dropped
function _skin_segs(xl, seg_l, gap, o) =
    let(P = seg_l + gap, kmax = ceil((xl + seg_l) / P) + 1)
    [for (k = [-kmax : kmax])
        let(c = o + k * P, a = max(c - seg_l / 2, -xl), b = min(c + seg_l / 2, xl))
        if (b - a >= 2.0) [a, b]];

// one dovetail rib segment along x (narrow neck at z = 0, wide head on top)
module _skin_rib(a, b, y, neck, head, h)
    translate([a, y, 0]) rotate([90, 0, 90]) linear_extrude(b - a)
        polygon([[-neck / 2, -eps], [neck / 2, -eps], [head / 2, h], [-head / 2, h]]);

module _skin_checks(size, skin_t, lip_t, lip_w, rim_w, proud, ch, r,
                    rib_h, rib_neck, rib_head, rib_pitch, seg_l, rib_gap, rib_inset, ribs) {
    assert(skin_t >= min_tpu, "skin_t must be >= min_tpu (1.2 mm)");
    assert(!ribs || skin_t >= rib_h + 3 * layer_h - 1e-6,
           "skin_t must leave >= 3 TPU layers over the ribs (skin_t >= rib_h + 0.6), or set ribs = false");
    assert(lip_t >= 3 * layer_h - 1e-6, "lip_t must be >= 3 layers");
    assert(rim_w >= min_petg - 1e-6, "rim_w must be >= min_petg");
    assert(lip_w > 0 && min(size[0], size[1]) - 2 * lip_w >= min_tpu, "lip opening too small: size - 2 * lip_w < min_tpu");
    assert(r >= 0 && 2 * r < min(size[0], size[1]), "corner radius r too large for size");
    assert(proud >= 0 && ch >= 0, "need proud >= 0 and ch >= 0");
    assert(!ribs || rib_neck >= min_petg - 1e-6, "rib_neck must be >= min_petg");
    assert(!ribs || atan((rib_head - rib_neck) / 2 / rib_h) <= 45, "rib flank overhang > 45 deg");
    assert(!ribs || rib_pitch - rib_head >= min_tpu - 1e-6, "TPU key between rib heads < min_tpu: raise rib_pitch");
    assert(!ribs || rib_gap >= anchor_gap_z - 1e-6, "rib_gap must be >= anchor_gap_z");
    assert(!ribs || rib_inset >= min_tpu - 1e-6, "rib_inset must be >= min_tpu");
}

// -----------------------------------------------------------------------
// Raw geometry helpers (children for hybrid(); no `part` argument)
// -----------------------------------------------------------------------
// PETG anchor features on a face at z = 0: rim, lip over the TPU, ribs.
module skin_anchor_petg(size = [40, 20], skin_t = 2.0, lip_t = 1.2, lip_w = 1.6, rim_w = 2.0,
                        proud = 1.0, ch = 0.6, r = 3,
                        rib_h = anchor_depth, rib_neck = anchor_neck, rib_head = anchor_head,
                        rib_pitch = 4.0, seg_l = 5.0, rib_gap = 1.6, rib_inset = 2.0, ribs = true) {
    _skin_checks(size, skin_t, lip_t, lip_w, rim_w, proud, ch, r,
                 rib_h, rib_neck, rib_head, rib_pitch, seg_l, rib_gap, rib_inset, ribs);
    H = skin_t + lip_t;
    // rim wall
    translate([0, 0, -eps]) linear_extrude(H + eps) difference() {
        _skin_outer_2d(size, rim_w, r);
        _skin_pocket_2d(size, r);
    }
    // lip: the top lip_t of the rim reaches lip_w over the TPU
    translate([0, 0, skin_t]) linear_extrude(lip_t) difference() {
        _skin_outer_2d(size, rim_w, r);
        _skin_opening_2d(size, lip_w, r);
    }
    // staggered dovetail rib segments, kept rib_inset clear of the pocket wall
    if (ribs) {
        rows = _skin_rows(size, rib_inset, rib_head, rib_pitch);
        xl = size[0] / 2 - rib_inset;
        if (len(rows) > 0) intersection() {
            union() for (j = [0 : len(rows) - 1])
                for (s = _skin_segs(xl, seg_l, rib_gap, (j % 2) * (seg_l + rib_gap) / 2))
                    _skin_rib(s[0], s[1], rows[j], rib_neck, rib_head, rib_h);
            translate([0, 0, -1]) linear_extrude(rib_h + 2)
                offset(delta = -rib_inset) _skin_pocket_2d(size, r);
        }
    }
}

// TPU region on a face at z = 0: the pocket under the lip, the lip opening,
// and the proud contact pad with a chamfered top edge.
module skin_tpu_region(size = [40, 20], skin_t = 2.0, lip_t = 1.2, lip_w = 1.6, rim_w = 2.0,
                       proud = 1.0, ch = 0.6, r = 3,
                       rib_h = anchor_depth, rib_neck = anchor_neck, rib_head = anchor_head,
                       rib_pitch = 4.0, seg_l = 5.0, rib_gap = 1.6, rib_inset = 2.0, ribs = true) {
    H = skin_t + lip_t;
    linear_extrude(skin_t) _skin_pocket_2d(size, r);                  // under the lip
    translate([0, 0, skin_t - eps]) linear_extrude(lip_t + 2 * eps)   // through the lip opening
        _skin_opening_2d(size, lip_w, r);
    if (proud > 0) translate([0, 0, H - eps]) {
        c = min(ch, proud);
        if (c > 0) hull() {
            linear_extrude(proud - c + eps) _skin_opening_2d(size, lip_w, r);
            linear_extrude(proud + eps) offset(delta = -c) _skin_opening_2d(size, lip_w, r);
        }
        else linear_extrude(proud + eps) _skin_opening_2d(size, lip_w, r);
    }
}

// -----------------------------------------------------------------------
// Public block modules
// -----------------------------------------------------------------------
// The pad alone (rim + lip + ribs + TPU), standing on a face at z = 0.
module skin_anchor(part, size = [40, 20], skin_t = 2.0, lip_t = 1.2, lip_w = 1.6, rim_w = 2.0,
                   proud = 1.0, ch = 0.6, r = 3,
                   rib_h = anchor_depth, rib_neck = anchor_neck, rib_head = anchor_head,
                   rib_pitch = 4.0, seg_l = 5.0, rib_gap = 1.6, rib_inset = 2.0, ribs = true)
    hybrid(part) {
        skin_anchor_petg(size, skin_t, lip_t, lip_w, rim_w, proud, ch, r,
                         rib_h, rib_neck, rib_head, rib_pitch, seg_l, rib_gap, rib_inset, ribs);
        skin_tpu_region(size, skin_t, lip_t, lip_w, rim_w, proud, ch, r,
                        rib_h, rib_neck, rib_head, rib_pitch, seg_l, rib_gap, rib_inset, ribs);
    }

// PETG plate (z = 0 .. base_t) with a captured TPU pad on its top face.
module skin_pad(part, size = [40, 20], base_t = 3, skin_t = 2.0, lip_t = 1.2, lip_w = 1.6, rim_w = 2.0,
                proud = 1.0, ch = 0.6, r = 3,
                rib_h = anchor_depth, rib_neck = anchor_neck, rib_head = anchor_head,
                rib_pitch = 4.0, seg_l = 5.0, rib_gap = 1.6, rib_inset = 2.0, ribs = true,
                margin = 0, holes = [], hole_d = 3.4) {
    m = is_list(margin) ? margin : [margin, margin];
    assert(base_t >= min_petg, "base_t must be >= min_petg");
    assert(hole_d >= 1.6, "holes must be >= 1.6 mm");
    hybrid(part) {
        difference() {
            union() {
                linear_extrude(base_t)
                    _skin_rrect(size + [2, 2] * rim_w + 2 * m, r + rim_w + min(m[0], m[1]));
                translate([0, 0, base_t])
                    skin_anchor_petg(size, skin_t, lip_t, lip_w, rim_w, proud, ch, r,
                                     rib_h, rib_neck, rib_head, rib_pitch, seg_l, rib_gap, rib_inset, ribs);
            }
            for (h = holes) translate([h[0], h[1], -1]) cylinder(d = hole_d, h = base_t + 2, $fn = 32);
        }
        translate([0, 0, base_t])
            skin_tpu_region(size, skin_t, lip_t, lip_w, rim_w, proud, ch, r,
                            rib_h, rib_neck, rib_head, rib_pitch, seg_l, rib_gap, rib_inset, ribs);
    }
}

// Peel coupon: plate (grip end at x = 0) + pad + TPU tab over the far rim,
// down the plate end and along the bed. Clamp the plate, pull the tail up.
module skin_peel_coupon(part, size = [35, 12], base_t = 3, skin_t = 2.0, lip_t = 1.2, lip_w = 1.6,
                        rim_w = 2.0, proud = 1.0, ch = 0.6, r = 3,
                        rib_h = anchor_depth, rib_neck = anchor_neck, rib_head = anchor_head,
                        rib_pitch = 4.0, seg_l = 5.0, rib_gap = 1.6, rib_inset = 2.0, ribs = true,
                        grip_l = 15, tab_t = 1.6, tail_l = 40, tab_overlap = 8, tab_wall = 2, hole_d = 6) {
    L = grip_l + size[0] + 2 * rim_w;        // plate length
    W = size[1] + 2 * rim_w;                 // plate width
    cx = L - rim_w - size[0] / 2;            // pad centre
    H = skin_t + lip_t;
    tw = size[1];                            // tab width (12 mm, as coupon P)
    top = base_t + H + tab_t;
    x_tab = cx + size[0] / 2 - tab_overlap;  // where the tab joins the pad top
    hybrid(part) {
        difference() {
            union() {
                translate([0, -W / 2, 0]) cube([L, W, base_t]);
                translate([cx, 0, base_t])
                    skin_anchor_petg(size, skin_t, lip_t, lip_w, rim_w, proud, ch, r,
                                     rib_h, rib_neck, rib_head, rib_pitch, seg_l, rib_gap, rib_inset, ribs);
            }
            translate([8, 0, -1]) cylinder(d = hole_d, h = base_t + 2, $fn = 40);
        }
        difference() {
            union() {
                translate([cx, 0, base_t])
                    skin_tpu_region(size, skin_t, lip_t, lip_w, rim_w, proud, ch, r,
                                    rib_h, rib_neck, rib_head, rib_pitch, seg_l, rib_gap, rib_inset, ribs);
                // tab over the far rim (fills down to the plate where the rim corners round off)
                translate([x_tab, -tw / 2, base_t]) cube([L - x_tab + eps, tw, H + tab_t]);
                // down the plate end, then flat along the bed
                translate([L, -tw / 2, 0]) cube([tab_wall, tw, top]);
                translate([L, -tw / 2, 0]) cube([tail_l, tw, tab_t]);
            }
            translate([L + tail_l - 7, 0, -1]) cylinder(d = hole_d, h = tab_t + 2, $fn = 40);
        }
    }
}

// -----------------------------------------------------------------------
// Demo: a grippy foot pad with two M3 holes. Test: the peel coupon.
// -----------------------------------------------------------------------
if (show == "demo")
    skin_pad(part, size = [40, 20], margin = [8, 0], holes = [[-26, 0], [26, 0]]);
else if (show == "test")
    skin_peel_coupon(part);
