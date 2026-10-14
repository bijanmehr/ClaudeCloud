// Ratchet band for a cracked wooden cylinder
// -----------------------------------------------------------------------
// A narrow, low-profile band printed flat in one go: a PETG band with a
// thick TPU spring lining locked into it. Squeeze the two ears together:
// the toothed rack on jaw B slides under the spring finger on jaw A, which
// presses it onto saw teeth cut in jaw A. Each click shrinks the band; the
// teeth hold it. Lift the finger's lip to release. No screws, nothing to
// assemble.
//
// Inspired by the one-piece "Circular Fastener (Hose Clamp)" by
// inhabitworks; drawn from scratch for a rigid wooden core:
//  - the jaws carrying the ratchet are rigid, so the rack stays in mesh
//    and the band closes by wrapping its thin upper part tighter;
//  - the TPU fins give the spring that wood (unlike a hose) does not have,
//    so the grip is steady between clicks and as the wood moves.
//
// Plan-view angles: phi = 0 at the bottom (ratchet), positive towards +x.

part = "print"; // [print, petg, tpu]

/* [Wood] */
wood_d = 38.0;
band_h = 12.0;          // band width along the wood

/* [Spring lining (TPU)] */
fit_gap = 0.5;          // radial clearance at the fin tips as printed (the ratchet takes it up)
liner_base_t = 2.0;     // solid TPU skin
fin_h = 2.4;            // spring fins
fin_t = 1.2;
fin_lean = 30;
fin_pitch = 3.6;
fin_end_taper = 1.2;    // fins fade in over this height at both faces

/* [TPU-to-PETG lock] */
// The TPU is held mechanically, not by adhesion (PETG and TPU bond poorly):
// lips along both faces, dovetail ribs, and walls over the skin ends.
lip_t = 0.6;            // PETG lips over the TPU skin at both faces (3 layers)
lip_depth = 1.0;        // how far the lips reach over the skin
end_wall = 0.8;         // PETG wall over each end of the skin, beside the gap
anchor_depth = 1.2;     // PETG dovetail ribs reaching into the TPU skin
anchor_neck = 1.3;
anchor_head = 2.3;
anchor_step = 10;       // degrees between ribs
anchor_gap_z = 1.2;     // TPU-filled breaks along each rib (locks the skin along the wood axis)
anchor_segs = 3;

/* [Band (PETG)] */
ring_t = 2.2;           // thin upper band: wraps tighter as the ratchet closes
gap = 24;               // degrees of opening at the bottom, as printed
jaw_end = 60;           // jaws span gap/2 .. jaw_end degrees each side

/* [Ratchet] */
tooth_pitch = 1.0;      // along the arc at the tooth roots
tooth_d = 0.8;          // tooth depth
tooth_c = 0.1;          // radial clearance between meshed teeth
tooth_tip = 0.25;       // flat on tooth tips (printability)
undercut = 6;           // locking faces lean this much so load pulls teeth deeper
teeth_n = 6;            // fixed teeth on jaw A
teeth_start = 2.0;      // first fixed tooth, degrees beyond the jaw A end
rack_t = 1.4;           // rack back above its teeth
rack_tip0 = 1.5;        // rack tip, degrees short of the jaw A end, as printed
rack_teeth_n = 9;
arm_t = 1.4;            // spring finger
arm_preload = 0.3;      // finger bump presses the meshed rack in by this much
arm_root = -44;         // where the finger leaves jaw A
arm_tip = -6;           // free end of the finger (release lip beyond it)
bump_at = -19;          // finger bump, centre angle
xy_gap = 0.4;           // print clearance between parts that move

/* [Ears] */
ear_h = 5.5;            // how far the finger ears stand off the band
ear_a = [46, 58];       // angular extent of each ear

$fn = 180;
eps = 0.01;

// ---------------------------------------------------------------------
// Derived sizes
// ---------------------------------------------------------------------
wood_r = wood_d / 2;
R_tip = wood_r + fit_gap;        // fin tips, as printed
R_li = R_tip + fin_h;            // inner face of the TPU skin
R_i = R_li + liner_base_t;       // PETG band, inner
R_o = R_i + ring_t;              // PETG band, outer

r_ft_root = R_o;                        // fixed teeth (jaw A) cut into the band surface
r_ft_tip = r_ft_root + tooth_d;
r_rk_tip = r_ft_root + tooth_c;         // rack teeth (jaw B) point inwards
r_rk_root = r_ft_tip + tooth_c;
r_rk_out = r_rk_root + rack_t;          // rack back
r_arm_in = r_rk_out + xy_gap;           // spring finger
r_arm_out = r_arm_in + arm_t;
r_bump = r_rk_out - arm_preload;
r_stop = r_rk_out - 0.6;                // jaw A beyond the teeth: the rack stops against it
r_ear = R_o + ear_h;

g2 = gap / 2;
dA = tooth_pitch / r_ft_root * 180 / PI;      // tooth pitch in degrees
ft_hi = -(g2 + teeth_start);                  // fixed teeth span [ft_lo, ft_hi]
ft_lo = ft_hi - teeth_n * dA;
stop_a = ft_lo - 0.6 * dA;                    // rack end stop on jaw A
rk_tip = -(g2 - rack_tip0);                   // rack tip, as printed (B frame)
// rack teeth sit on the same angular grid as the fixed teeth, so they mesh
// whenever jaw B has turned a whole number of teeth towards jaw A
rk_first = ft_hi + ceil((rk_tip - ft_hi) / dA + 0.35) * dA;
rk_last = rk_first + (rack_teeth_n - 1) * dA;

function pt(r, a) = [r * sin(a), -r * cos(a)];
function arc(r, a1, a2, n = 0) =
    let(k = n > 0 ? n : max(2, ceil(abs(a2 - a1) / 1.5)))
    [for (i = [0 : k]) pt(r, a1 + (a2 - a1) * i / k)];
module annulus_sector(r1, r2, a1, a2) polygon(concat(arc(r2, a1, a2), arc(r1, a2, a1)));
module wedge(a1, a2, r = 200) polygon(concat([[0, 0]], arc(r, a1, a2)));

// ---------------------------------------------------------------------
// Ratchet teeth (polar saw profiles; locking faces lean by `undercut`)
// ---------------------------------------------------------------------
// Fixed teeth point outwards. Each locking face looks towards -phi, so the
// rack (moving towards -phi as the band closes) rides up the slopes and
// is caught when pulled back.
function fixed_profile() =
    let(t = tooth_tip / r_ft_tip * 180 / PI, u = tooth_d * tan(undercut) / r_ft_root * 180 / PI)
    concat([for (k = [0 : teeth_n - 1]) each [
            pt(r_ft_root, ft_lo + k * dA),
            pt(r_ft_tip, ft_lo + k * dA - u),
            pt(r_ft_tip, ft_lo + k * dA - u + t)]],
           [pt(r_ft_root, ft_hi)]);
module fixed_teeth()
    polygon(concat(fixed_profile(), arc(R_o - 0.3, ft_hi, ft_lo)));

// Rack teeth point inwards and fill the fixed valleys: each rack tooth's
// locking face looks towards +phi and leans like the fixed face it meets,
// so the tips hook under each other when the band pulls the rack back.
function rack_profile() =
    let(t = tooth_tip / r_rk_tip * 180 / PI, u = tooth_d * tan(undercut) / r_ft_root * 180 / PI,
        d = 0.05 / r_rk_tip * 180 / PI)
    [for (k = [0 : rack_teeth_n - 1]) each [
        pt(r_rk_tip, rk_first + k * dA - d - t),
        pt(r_rk_tip, rk_first + k * dA - d),
        pt(r_rk_root, rk_first + k * dA - u - d)]];

// ---------------------------------------------------------------------
// PETG outlines (as printed)
// ---------------------------------------------------------------------
module upper_band() annulus_sector(R_i, R_o, jaw_end - 2, 360 - jaw_end + 2);

module ear(s) {    // s = -1 jaw A (left), +1 jaw B (right)
    a1 = s * ear_a[0]; a2 = s * ear_a[1];
    offset(r = 1.2) offset(delta = -1.2)
        polygon(concat(arc(R_o - 0.5, a2, a1), arc(r_ear, a1, a2)));
}

// Jaw A: band segment with the fixed teeth, rack stop, spring finger, ear
module jaw_A() { jaw_A_body(); spring_finger(); }
module jaw_A_body() {
    annulus_sector(R_i, R_o, -jaw_end, -g2);
    fixed_teeth();
    annulus_sector(R_o - 0.3, r_stop, arm_root - 2, stop_a);       // rack stop
    annulus_sector(R_o - 0.3, r_arm_out, -jaw_end, arm_root);      // finger root
    ear(-1);
}

module spring_finger() {
    lip_a = arm_tip + 3.5;
    annulus_sector(r_arm_in, r_arm_out, arm_root - 1, arm_tip);
    // bump that presses the rack onto the teeth
    hull() {
        polygon(arc(r_arm_in + 0.2, bump_at - 3.5, bump_at + 3.5));
        polygon(arc(r_bump, bump_at - 1.2, bump_at + 1.2));
    }
    // release lip: a low hook beyond the finger end
    hull() {
        translate(pt((r_arm_in + r_arm_out) / 2, arm_tip)) circle(d = arm_t, $fn = 40);
        translate(pt(r_arm_out + 0.9, lip_a)) circle(d = 1.6, $fn = 40);
    }
}

// Jaw B: band segment filled out to the rack back, rack, ear
module jaw_B() {
    annulus_sector(R_i, r_rk_out, g2, jaw_end);
    ear(1);
    rack();
}

module rack() {
    tip_ch = 0.6;    // lead-in chamfer on the rack tip
    lead = tip_ch / r_rk_out * 180 / PI;
    polygon(concat(
        arc(r_rk_out, g2 + 1, rk_tip + lead),
        [pt(r_rk_out - tip_ch, rk_tip), pt(r_rk_root, rk_tip)],
        rack_profile(),
        [pt(r_rk_root, rk_last + 0.2), pt(r_rk_root, g2 + 1)]));
}

module petg_2d() union() { upper_band(); jaw_A(); jaw_B(); }

// ---------------------------------------------------------------------
// TPU lock (PETG side)
// ---------------------------------------------------------------------
lin_end = g2 + end_wall / R_i * 180 / PI;    // skin starts behind the end walls
// end walls: the band turns down over the ends of the TPU skin
module end_walls_2d() for (s = [-1, 1])
    annulus_sector(R_li + 0.3, R_i + 0.05, s * g2, s * lin_end);
// lips: the band reaches over the TPU skin along both faces
module lips_3d() for (z = [0, band_h - lip_t]) translate([0, 0, z]) linear_extrude(lip_t)
    union() { annulus_sector(R_i - lip_depth, R_i + 0.05, lin_end - 0.1, 360 - lin_end + 0.1); end_walls_2d(); }
// dovetail ribs reaching into the skin, broken along the height (staggered
// rib to rib) so the TPU wraps round, between and over every piece
anchor_first = g2 + 4;
anchor_n = floor((180 - anchor_first) / anchor_step);
anchor_phis = concat([for (i = [0 : anchor_n]) anchor_first + i * anchor_step],
                     [for (i = [0 : anchor_n]) -(anchor_first + i * anchor_step)]);
module anchor_2d(a) rotate(a) polygon([
    [-anchor_neck / 2, -(R_i + 0.3)], [anchor_neck / 2, -(R_i + 0.3)],
    [anchor_head / 2, -(R_i - anchor_depth)], [-anchor_head / 2, -(R_i - anchor_depth)]]);
inner_h = band_h - 2 * lip_t;
seg_h = (inner_h - (anchor_segs + 1) * anchor_gap_z) / anchor_segs;
module anchors_3d()
    for (i = [0 : len(anchor_phis) - 1]) {
        a = anchor_phis[i];
        sh = (i % 2) * (seg_h + anchor_gap_z) / 2;      // stagger every other rib
        if (abs(a) <= 180)
            for (k = [0 : anchor_segs - 1]) {
                z0 = lip_t + anchor_gap_z + k * (seg_h + anchor_gap_z) + sh;
                z1 = min(z0 + seg_h, band_h - lip_t - anchor_gap_z);
                if (z1 - z0 > 0.6) translate([0, 0, z0]) linear_extrude(z1 - z0) anchor_2d(a);
            }
    }

module petg_3d() union() {
    linear_extrude(band_h) union() { petg_2d(); end_walls_2d(); }
    lips_3d();
    anchors_3d();
}

// ---------------------------------------------------------------------
// TPU lining: skin + fins, filling round the PETG ribs
// ---------------------------------------------------------------------
fin_step = fin_pitch / R_tip * 180 / PI;
fin_first = lin_end + 2.6 / R_tip * 180 / PI;
fin_n = floor((180 - fin_first) / fin_step);
// fins lean with their tips towards the top: as the band tightens its
// material slides towards the gap relative to the wood, folding the fins
// further the way they already lean
fin_phis = concat([for (i = [0 : fin_n]) fin_first + i * fin_step],
                  [for (i = [0 : fin_n]) -(fin_first + i * fin_step)]);
module fin_2d(a) {
    s = a >= 0 ? 1 : -1;
    lean = s * fin_lean;
    len = R_li * cos(fin_lean) - sqrt(pow(R_li * cos(fin_lean), 2) - pow(R_li, 2) + pow(R_tip, 2));
    root = [0, -(R_li + 0.3)];
    dir = [sin(lean), cos(lean)];           // inwards, tipped towards the top
    rotate(a) hull() {
        translate(root) circle(d = fin_t, $fn = 24);
        translate(root + dir * (len + 0.3 - fin_t / 2)) circle(d = fin_t, $fn = 24);
    }
}
module liner_region_2d() intersection() {
    union() {
        annulus_sector(R_li, R_i + 0.05, lin_end, 360 - lin_end);
        for (a = fin_phis) fin_2d(a);
    }
    union() { wedge(lin_end, 180); wedge(-180, -lin_end); }
}

module liner_3d() difference() {
    linear_extrude(band_h) liner_region_2d();
    petg_3d();       // exact shared boundary with the band, no overlap
    translate([0, 0, -eps]) cylinder(r1 = R_li, r2 = R_tip - 0.5, h = fin_end_taper + eps);
    translate([0, 0, band_h - fin_end_taper]) cylinder(r1 = R_tip - 0.5, r2 = R_li, h = fin_end_taper + eps);
}

// ---------------------------------------------------------------------
if (part == "print") { color("SteelBlue") petg_3d(); color("OrangeRed") liner_3d(); }
else if (part == "petg") petg_3d();
else if (part == "tpu") liner_3d();
// one-millimetre slabs of each rigid piece, for the motion checks
else if (part == "A_slab") linear_extrude(1) jaw_A_body();
else if (part == "F_slab") linear_extrude(1) spring_finger();
else if (part == "B_slab") linear_extrude(1) jaw_B();

echo(str("GEOM {\"R_tip\":", R_tip, ",\"R_li\":", R_li, ",\"R_i\":", R_i, ",\"R_o\":", R_o,
         ",\"r_ft_root\":", r_ft_root, ",\"r_ft_tip\":", r_ft_tip, ",\"r_rk_tip\":", r_rk_tip,
         ",\"r_rk_root\":", r_rk_root, ",\"r_rk_out\":", r_rk_out, ",\"r_arm_in\":", r_arm_in,
         ",\"r_arm_out\":", r_arm_out, ",\"r_bump\":", r_bump, ",\"r_stop\":", r_stop, ",\"r_ear\":", r_ear,
         ",\"g2\":", g2, ",\"dA\":", dA, ",\"ft_lo\":", ft_lo, ",\"ft_hi\":", ft_hi,
         ",\"stop_a\":", stop_a, ",\"rk_tip\":", rk_tip, ",\"rk_first\":", rk_first, ",\"rk_last\":", rk_last,
         ",\"jaw_end\":", jaw_end, ",\"arm_root\":", arm_root, ",\"arm_tip\":", arm_tip, ",\"bump_at\":", bump_at,
         ",\"band_h\":", band_h, ",\"wood_r\":", wood_r, ",\"fin_h\":", fin_h, ",\"liner_base_t\":", liner_base_t,
         ",\"ring_t\":", ring_t, "}"));
