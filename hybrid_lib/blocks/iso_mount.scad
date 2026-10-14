include <../params.scad>
// =======================================================================
// iso_mount: fail-safe vibration isolation mount, printed in place
// =======================================================================
// WHAT IT IS FOR
//   Soft-mounting a flight controller, gyro/IMU board, camera or a small
//   motor/gearbox on a PETG frame plate, in the same print as the plate
//   (H2D: PETG left nozzle, TPU 95A right nozzle). Nothing to assemble: the
//   PETG hub that takes the screw is printed inside its PETG ring, joined
//   only by a thin TPU web. If the TPU tears - or never stuck in the first
//   place - the hub still cannot leave the ring (fail-safe), and hard stops
//   limit how far it can move, so a crash cannot overstretch the web.
//
// HOW IT WORKS (half section through a spoke, from the real geometry:
//   r to the right, z up, 1 char ~ 0.4 mm, 1 line ~ 0.8 mm;
//   H = hub, # = ring, T = TPU, blank = air)
//
//   z mm   axis
//   14.8 |    HHHHHHH                     <- hub top = board seat, `lift` above the ring top
//        |    HHHHHHH           H
//        |    HHHHHHH         HHH         <- F2: top flange, a 45 deg cone shell (cup)
//   11.8 |    HHHHHHH       HHHH  ######  <- ring top
//        |    HHHHHHH     HHHH  ########
//        |    HHHHHHH   HHHH  ##########  <- lip top: 45 deg funnel, `gap` under F2
//        |    HHHHHHH HHHH   ###########     (down stop)
//        |    HHHHHHHHHH     ###########
//        |    HHHHHHHH       ###########
//        |    HHHHHHH        ###########
//    6.4 |    HHHHTTTTTTTTTTTTTTT#######  <- TPU web: rims in grooves in hub and lip,
//    5.2 |    HHHHTTTTTTTTTTTTTTT#######     `span` of air between them (spokes)
//        |    HHHHHHH        ###########
//        |    HHHHHHH         ##########  <- lip underside: 45 deg chamfer, `gap`
//        |    HHHHHHH           ########     over F1's rim (up stop)
//        |    HHHHHHH             ######
//    1.6 |    HHHHHHHHHHHHHHHHHHH  #####  <- F1: bottom flange on the bed, `gap` to
//    0   |    HHHHHHHHHHHHHHHHHHH  #####     the bore (side stop)
//        +----+------+------+---+-+----+---> r
//            r_h    r_t    r_l r_f r_i r_o
//   Pictures: images/iso_mount.png (demo plate), images/iso_mount_section.png
//   (cut-away), images/iso_mount_test.png (test piece).
//
//   - HUB (PETG, its own body): a tube for the screw, a flat bottom flange
//     F1 on the bed and a conical top flange F2. Both flanges reach past
//     the edge of the ring's LIP by `overlap`: F1 below it, F2 above it.
//     The hub cannot come out up or down even with no TPU at all.
//   - RING (PETG, part of the plate): bore r_i, an inward lip whose
//     underside is a 45 deg chamfer (prints without support over F1) and
//     whose top is a 45 deg funnel parallel to F2.
//   - TPU WEB: a horizontal disc, web_t thick, spanning `span` mm of air from
//     the hub tube to the lip edge. Its inner rim sits in a groove round the
//     hub (hexagonal floor), its outer rim in a groove in the lip edge
//     (8-sided floor). The groove shoulders above and below hold the rims;
//     the rims are closed rings round the hub / inside the lip, so pushes and
//     pulls in every direction are carried by bearing, never by adhesion,
//     and the polygon floors key the rims so they cannot slip round the hub
//     or the lip. tools/capture_check.py confirms the TPU cannot slide out
//     in any of 26 directions - against the hub alone and against the ring
//     alone too, and the hub cannot leave the ring with the TPU removed.
//   - `spokes` windows cut through the web leave that many TPU spokes
//     (spoke_w wide) between the two rims: the spring rate is set by the
//     spokes (bending + shear, see ESTIMATES). spokes = 0 keeps a full,
//     much stiffer annulus (for heavy payloads).
//   - STOPS / FAIL-SAFE: every hub-ring clearance is `gap` (0.8 mm):
//     up - F1 meets the lip chamfer; down - F2 meets the funnel;
//     sideways - F1 and F2 meet the bore. Vibration amplitudes are
//     micrometres, so the hub never touches the ring in normal use, but a
//     crash or a yank stops PETG-on-PETG after 0.8 mm, so the TPU never sees
//     more than ~30 % strain. Air (not TPU) fills those gaps on purpose: TPU
//     packed between flange and lip would be a compression pad ~100x
//     stiffer than the bending web (resonance in the kHz, no isolation).
//
// PRINTING (no supports; z is the print direction as modelled)
//   The web is a TPU bridge of `span` mm (<= 5) between PETG shoulders; the
//   groove shoulders above the web are PETG printed on TPU; the lip chamfer
//   and the F2 cone are 45 deg PETG overhangs; nothing starts in mid-air.
//   Export part = "petg" and part = "tpu" to two STLs, load them as one
//   object with two parts and give each its filament (PETG left, TPU right).
//   Print the TPU solid (100 % infill or >= 3 walls + solid layers); slow
//   TPU bridges (~15-20 mm/s) with the fan on. 4 PETG walls round the hub
//   tube. Keep the 0.8 mm gaps clean (no "elephant foot" on F1, low
//   z-hop/ooze); if F1 fuses to the ring, cut it free with a knife round
//   the bore before use.
//
// PARAMETERS (mm). Defaults and the rule behind each:
//   hole_d    3.2  screw hole through the hub (M3 clearance; 2.2 for M2).
//                  >= 1.6. For a heat-set insert use its hole size and
//                  hub_wall >= 2.
//   web_t     1.2  TPU web thickness (6 layers) = min_tpu. Spring rate
//                  ~ web_t^3 (bending): 1.6 -> ~2x stiffer, 2.0 -> ~3x.
//   span      3.0  free web length from hub tube to lip edge. Spring rate
//                  ~ 1/span^3 (bending) - the strongest tuning knob. Rule:
//                  min_tpu <= span <= 5 (the web is printed as a bridge).
//   spokes    4    TPU spokes left by cutting windows into the web
//                  (0 = full annulus; otherwise >= 3 so the hub cannot rock
//                  about one spoke line). Spring rate ~ spokes * spoke_w.
//   spoke_w   1.6  spoke width (4 lines). >= min_tpu.
//   gap       0.8  every hub-ring clearance = travel to the hard stops.
//                  >= z_clear (0.6, vertical rule) and xy_clear; 0.8 so
//                  PETG print-in-place parts do not fuse (the 45 deg faces
//                  are then 0.57 mm apart).
//   overlap   1.6  how far F1/F2 reach past the lip edge (the fail-safe
//                  bearing). >= min_petg; 1.6 keeps >= 0.8 of bearing even
//                  with the hub pushed sideways to its stop.
//   flange_t  1.6  F1 thickness (8 layers); its bending sets the fail-safe
//                  pull-out strength (~ flange_t^2). >= min_petg.
//   shoulder  1.2  PETG below the lip groove at the lip edge. >= min_petg.
//   groove    1.2  TPU rim depth in each groove (at the shallowest point of
//                  the polygon floors). >= min_tpu.
//   hub_wall  1.2  PETG between the hole and the hub groove floor. >= min_petg.
//   cap_t     1.2  F2 cone shell thickness (normal to the cone). >= min_petg.
//   ring_wall 2.0  ring wall outside the bore (standalone mount / plate boss).
//   lift      3.0  hub top above the ring top: the board's clearance over the
//                  ring (2.2 at the down stop). The rings reach ~12 mm out
//                  from each board hole, under the board's corners; raise
//                  `lift` for taller parts on the underside of the board.
//   Derived (defaults, M3): hub tube r_t = 4.43 (hex groove floor: 2.8 to
//   the flats, 3.23 to the corners, + groove), lip edge r_l = r_t + span =
//   7.43, flanges r_f = 9.03, bore r_i = 9.83, ring OD 23.7 mm, ring top
//   11.8, hub top 14.8 above the bed (iso_mount_dims()). Heights:
//   ring top = flange_t + 2 gap + 2 overlap + shoulder + web_t + span.
//   Hub ~1.8 g, TPU 0.15 cm^3 per mount.
//
// ESTIMATES (defaults; TPU 95A E ~ 26 MPa, G ~ 9 MPa, nu ~ 0.45, shear
//            factor 5/6; PETG E ~ 2000 MPa, ~45 MPa)
//   The spokes are guided beams (hub moves, both ends stay square):
//   AXIAL (z), per mount, W = spokes * spoke_w = 6.4, L = span = 3, t = web_t:
//     bending  k_b = E W t^3 / L^3 = 26 * 6.4 * 1.728 / 27 = 10.6 N/mm
//     shear    k_s = (5/6) G W t / L = 0.833 * 9 * 7.68 / 3 = 19.1 N/mm
//     k_ax = 1 / (1/k_b + 1/k_s) = 6.8 N/mm
//   RADIAL (x/y), per mount: each spoke is stretched along its length and
//   bent across it; summed over the spokes:
//     k_r = spokes/2 * (E spoke_w t / L + k_across)
//         = 2 * (16.6 + 2.4) = 38 N/mm      (k_across = bending E t w^3/L^3
//           = 4.7 in series with shear 4.8 per spoke)
//     With zero adhesion the spokes on the pulled side only act through the
//     soft TPU rims (bearing goes round the ring), so expect 25-38 N/mm.
//   Full annulus (spokes = 0): same strip formula with W = 2 pi r_mean =
//     37 mm -> k_ax ~ 40 N/mm (axisymmetric Mindlin-plate FE: 46); radial
//     ~ pi r_mean t (E/(1-nu^2) + G) / L ~ 310 N/mm (plane-stress FE: 290).
//   NATURAL FREQUENCY, 30 g board on 4 mounts (+ 4 hubs ~ 1.8 g each):
//     f = (1/2 pi) sqrt(4 k / m),  m = 37 g
//     vertical  f_z  = sqrt(4 * 6850 N/m / 0.037 kg) / 2 pi ~ 135 Hz
//     sideways  f_xy = sqrt(4 * 38000 / 0.037) / 2 pi ~ 320 Hz
//                      (~260 Hz at k_r = 25 N/mm)
//     roll/pitch rocking ~ 1.5 f_z ~ 200 Hz (uniform 36 mm board)
//   iso_mount_f(m_g, n, ...) evaluates this for other masses / settings.
//   Expect +-30-50 % (printed TPU modulus, groove compliance, end effects);
//   measure with the test piece. TPU damping (loss factor ~0.1-0.2) keeps
//   the resonance peak to ~5-10x. Isolation starts above 1.4 f and is ~-15
//   to -20 dB at 3 f: with the defaults that is the motor/prop harmonics and
//   ESC/frame noise above ~300-1000 Hz, which is what corrupts gyros.
//   TUNING (f ~ sqrt(k / m); aim f <= half the lowest frequency to isolate):
//     knobs: k_ax ~ spokes * spoke_w * web_t^3 / span^3 while bending
//     dominates (short, thick spokes become shear-limited: ~ web_t / span).
//     softer: span 4 -> k_ax 3.4, f_z ~95 Hz (OD +2 mm, 1 mm taller);
//             3 spokes x 1.2 -> k_ax 3.9, f_z ~100 Hz; or a heavier board.
//     stiffer: web_t 1.6 -> k_ax 12.7 (f_z ~185 Hz); spoke_w 2.4 -> 10.3
//             (f_z ~165 Hz); spokes = 0 (full web, k_ax ~40) for 60-150 g
//             cameras / motors (100 g on 4 full webs: f_z ~195 Hz,
//             f_xy ~540 Hz; web_t 2.0: k_ax ~113 for ~150-300 g).
//     static sag under 1 g = 9.81/(2 pi f)^2: 14 um at 135 Hz - never an
//     issue; the 0.8 mm stops are reached at ~ k * 0.8 = 5.5 N axial /
//     30 N radial per mount (~60 g / 300 g on a 37 g isolated mass).
//   STRENGTH (TPU gone - fail-safe):
//     Pull-out (hub up): F1 bending on the lip, cantilever ring estimate
//       P = sigma t^2 2 pi r_t / (6 (r_f - r_t)) = 45 * 2.56 * 27.9 / 27.6
//       ~ 115 N per mount (ignores hoop stiffening, so conservative), in-
//       plane stress, so no layer-bond weakness; the hub tube across its
//       layers (19 mm^2 at the groove, ~20 MPa) ~ 380 N.
//     Push-through (hub down): the F2 cone bears on the funnel: >> 200 N.
//     TPU web strain at the stops: 3 t gap / L^2 = 32 % (bending, axial),
//       gap / L = 27 % (radial) - well inside TPU 95A's range.
//   Design rule: in normal use only the TPU carries load; anything above a
//   few newtons per mount goes PETG-to-PETG through the stops.
//
// RECOMMENDED USES (rovers / drones)
//   Flight controller / IMU / gyro boards (30.5 x 30.5 or 20 x 20 stacks,
//   iso_plate()); FPV / action camera cradles (spokes = 0 or wider spokes);
//   lidar, GPS mast and compass boards; small gear motors or pumps on a
//   rover deck (spokes = 0, web_t 1.6-2.0); electronics trays that must
//   survive crashes (the stops keep them captive).
//
// MODULES (all take `part` = "petg" | "tpu" | "both" first; sizes in mm)
//   iso_mount(part, hole_d, web_t, span, spokes, spoke_w, gap, overlap, ...)
//       one complete mount, axis on z, standing on z = 0: ring (OD
//       2 * r_o, height ring top) + hub + TPU web.
//   iso_mount_ring(part, ...) / iso_mount_hub(part, ...)   the PETG pieces
//       ("tpu" draws nothing).
//   iso_mount_tpu(part, ...)   the TPU web body (TPU region minus PETG;
//       "petg" draws nothing).
//   iso_mount_cut(part, ...)   keep-out cylinder (r_o, full height) to
//       subtract from YOUR plate before adding iso_mount() (drawn for any
//       part, for use inside difference()):
//           if (part != "tpu") difference() { my_plate(); translate(p) iso_mount_cut(part); }
//           translate(p) iso_mount(part);      // the ring fuses with the plate
//   iso_plate(part, pattern = 30.5, plate_t = 3, ...)   the demo: a PETG
//       plate with 4 mounts on a pattern x pattern square (30.5 or 20 for
//       flight-controller stacks), 4 M3 holes to bolt it to the frame and a
//       central cable hole (when it fits). For pattern < 25 it defaults to
//       M2 hubs (hole_d 2.2) so the 4 rings fit 20 mm apart (M3 there:
//       hole_d = 3.2, span = 2.5). Rings that overlap are clipped out of
//       each other's bores. board_g (30 / 15 g) only feeds the echoed
//       frequency estimate.
//   iso_mount_test(part, ...)   test piece: one mount in a 48 x 34 x 4 frame
//       with two M3 holes (see TEST PIECE).
//   Functions: iso_mount_dims(...) = [r_h, r_gi, r_gc, r_t, r_l, r_lg, r_f,
//       r_i, r_o, z_l0, z_w0, z_w1, z_c0, z_rt, z_ht, h_n];
//       iso_mount_od(...), iso_mount_top(...) (hub top z),
//       iso_mount_k(...) = [k_axial, k_radial] N/mm per mount,
//       iso_mount_f(m_g, n, ...) = [f_z, f_xy] Hz for m_g grams on n mounts.
//
// RULES FOR YOUR OWN PARTS
//   - Subtract iso_mount_cut() (radius r_o) from your PETG before adding
//     the mount; nothing of yours may enter r < r_o (above the ring top
//     too: that is where the hub moves).
//   - The ring must stand on the bed (z = 0) - the hub's F1 is printed on
//     the bed. Your plate can be any thickness <= the ring top.
//   - Two hubs must not touch: mount centres >= 2 * r_i + min_petg apart
//     (20.9 mm with the M3 defaults; 19.7 with M2).
//   - Screw the board to the hub top only; nothing else may connect the
//     board to the frame (cables slack), or the isolation is bypassed.
//   - The hub has no rotation stop and the spokes are soft in twist
//     (~6 N*mm per degree): tighten with the nut held by a wrench under F1
//     and turn the screw from above (the board, held by the other mounts,
//     then takes the torque), not the other way round. Use a nylon-insert
//     nut. With a heat-set insert in the hub, hold the hub's cup while
//     tightening.
//
// TEST PIECE (show = "test"): iso_mount_test(). Bolt the frame to a bench
//   or a shaker (two M3 holes, 36 mm apart). Static: hang weights from an
//   M3 screw in the hub and read the deflection with a dial gauge -> k_ax
//   (expect ~7 N/mm, i.e. 0.14 mm per 100 g, linear up to the stop at
//   0.8 mm ~ 550 g). Tap test: screw a known mass (e.g. 8 g) to the hub, tap
//   the frame, record with a phone accelerometer app or a microphone and
//   read the peak: f = sqrt(k/m)/2pi with m = mass + hub (1.8 g); ~130 Hz
//   expected for 8 g. Fail-safe: cut all spokes with a knife and pull the
//   hub with a luggage scale - it must stop on the lip after 0.8 mm and hold
//   > 100 N. The label on the frame gives spokes x width, web_t, span.
//
// LIMITATIONS
//   - TPU 95A is ~15x stiffer than the silicone of commercial FC gummies:
//     even the soft settings put the vertical resonance near 100-150 Hz and
//     the sideways one 2-2.5x higher. It removes high-frequency noise; it
//     does not isolate low-frequency (< ~200 Hz) wobble. Radial stiffness
//     is 4-6x the axial (spokes in tension), so sideways isolation starts
//     higher.
//   - Big for its job: 23.7 mm OD and ~15 mm tall with M3 defaults; the
//     funnel/cone geometry is what makes it supportless and fail-safe.
//   - The web is a 3 mm TPU bridge: needs a reasonable TPU bridge setting.
//   - TPU creeps: a mount that carries a constant load (a heavy camera
//     hanging under it) sags slowly; keep static loads to a few N per mount.
//   - Fatigue life of printed TPU spokes is unknown; inspect after crashes.
//   - The estimates are hand formulas (+-30-50 %); the capture check is a
//     rigid-body test - measure your own print with the test piece.
// =======================================================================

part = "both";   // [petg, tpu, both]
show = "demo";   // [demo, demo20, test, section]

// ---------------------------------------------------------------------
// Defaults shared by all modules
// ---------------------------------------------------------------------
im_hole_d = 3.2;
im_web_t = min_tpu;       // 1.2
im_span = 3.0;
im_spokes = 4;
im_spoke_w = 1.6;
im_gap = 0.8;
im_overlap = 1.6;
im_flange_t = 1.6;
im_shoulder = min_petg;   // 1.2
im_groove = min_tpu;      // 1.2
im_hub_wall = min_petg;   // 1.2
im_cap_t = min_petg;      // 1.2
im_ring_wall = 2.0;
im_lift = 3.0;
im_lip_sides = 8;         // lip groove floor polygon (anti-turn key)
im_fn = 72;               // facets round the axis (same for every round part)

E_tpu = 26; G_tpu = 9;    // MPa, for the estimate functions

// ---------------------------------------------------------------------
// Derived dimensions (see the section in the header)
// ---------------------------------------------------------------------
function iso_mount_dims(hole_d = im_hole_d, web_t = im_web_t, span = im_span, gap = im_gap,
                        overlap = im_overlap, flange_t = im_flange_t, shoulder = im_shoulder,
                        groove = im_groove, hub_wall = im_hub_wall, cap_t = im_cap_t,
                        ring_wall = im_ring_wall, lift = im_lift) =
    let(r_h = hole_d / 2,
        r_gi = r_h + hub_wall,              // hub groove floor: hexagon, across flats / 2
        r_gc = r_gi / cos(30),              //   ... its corners
        r_t = r_gc + groove,                // hub tube (and groove mouth)
        r_l = r_t + span,                   // lip edge
        r_lg = r_l + groove,                // lip groove floor (polygon, across flats / 2)
        r_f = r_l + overlap,                // F1 / F2 radius
        r_i = r_f + gap,                    // ring bore
        r_o = r_i + ring_wall,
        z_l0 = flange_t + gap + overlap,    // bottom of the lip edge (top of the chamfer)
        z_w0 = z_l0 + shoulder,             // web bottom
        z_w1 = z_w0 + web_t,                // web top
        z_c0 = z_w1 + gap,                  // F2 cone underside at the hub tube
        z_rt = z_c0 + (r_i - r_t) - gap,    // ring top (end of the funnel)
        h_n = cap_t * sqrt(2),              // vertical thickness of the F2 shell
        z_ht = max(z_c0 + (r_f - r_t) + h_n, z_rt + lift))   // hub top
    [r_h, r_gi, r_gc, r_t, r_l, r_lg, r_f, r_i, r_o, z_l0, z_w0, z_w1, z_c0, z_rt, z_ht, h_n];

function iso_mount_od(hole_d = im_hole_d, span = im_span, gap = im_gap, overlap = im_overlap,
                      ring_wall = im_ring_wall) =
    2 * iso_mount_dims(hole_d = hole_d, span = span, gap = gap, overlap = overlap, ring_wall = ring_wall)[8];
function iso_mount_top(hole_d = im_hole_d, web_t = im_web_t, span = im_span, gap = im_gap,
                       overlap = im_overlap, flange_t = im_flange_t, lift = im_lift) =
    iso_mount_dims(hole_d = hole_d, web_t = web_t, span = span, gap = gap, overlap = overlap,
                   flange_t = flange_t, lift = lift)[14];

// spring rates per mount [axial, radial] in N/mm (hand formulas, header)
function iso_mount_k(web_t = im_web_t, span = im_span, spokes = im_spokes, spoke_w = im_spoke_w,
                     hole_d = im_hole_d) =
    let(d = iso_mount_dims(hole_d = hole_d, web_t = web_t, span = span),
        t = web_t, L = span, ks = 5 / 6,
        W = spokes > 0 ? spokes * spoke_w : PI * (d[3] + d[4]),
        k_ax = 1 / (L^3 / (E_tpu * W * t^3) + L / (ks * G_tpu * W * t)),
        k_across = 1 / (L^3 / (E_tpu * t * spoke_w^3) + L / (ks * G_tpu * spoke_w * t)),
        k_r = spokes > 0 ? spokes / 2 * (E_tpu * spoke_w * t / L + k_across)
                         : PI * (d[3] + d[4]) / 2 * t * (E_tpu / (1 - 0.45^2) + G_tpu) / L)
    [k_ax, k_r];
// natural frequencies [vertical, sideways] in Hz of m_g grams on n mounts
// (hub mass ~1.8 g each added)
function iso_mount_f(m_g = 30, n = 4, web_t = im_web_t, span = im_span, spokes = im_spokes,
                     spoke_w = im_spoke_w, hole_d = im_hole_d, hub_g = 1.8) =
    let(k = iso_mount_k(web_t, span, spokes, spoke_w, hole_d), m = (m_g + n * hub_g) / 1000)
    [sqrt(n * k[0] * 1000 / m) / (2 * PI), sqrt(n * k[1] * 1000 / m) / (2 * PI)];

// ---------------------------------------------------------------------
// Checks
// ---------------------------------------------------------------------
module im_check(hole_d, web_t, span, spokes, spoke_w, gap, overlap, flange_t, shoulder, groove,
                hub_wall, cap_t, ring_wall, lift) {
    d = iso_mount_dims(hole_d, web_t, span, gap, overlap, flange_t, shoulder, groove, hub_wall,
                       cap_t, ring_wall, lift);
    assert(hole_d >= 1.6, "iso_mount: hole_d must be >= 1.6");
    assert(web_t >= min_tpu, "iso_mount: web_t must be >= min_tpu (1.2)");
    assert(span >= min_tpu && span <= 5, "iso_mount: span must be 1.2 .. 5 (the web is printed as a bridge)");
    assert(spokes == 0 || spokes >= 3, "iso_mount: spokes must be 0 (full web) or >= 3");
    assert(spokes == 0 || spoke_w >= min_tpu, "iso_mount: spoke_w must be >= min_tpu (1.2)");
    assert(spokes == 0 || spokes * (spoke_w + min_tpu) <= 2 * PI * d[3],
           "iso_mount: too many / too wide spokes for the hub circumference");
    assert(gap >= z_clear && gap >= xy_clear, "iso_mount: gap must be >= 0.6");
    assert(overlap >= min_petg, "iso_mount: overlap must be >= min_petg (1.2)");
    assert(flange_t >= min_petg && shoulder >= min_petg && hub_wall >= min_petg && cap_t >= min_petg,
           "iso_mount: flange_t, shoulder, hub_wall, cap_t must be >= min_petg (1.2)");
    assert(groove >= min_tpu, "iso_mount: groove must be >= min_tpu (1.2)");
    assert(ring_wall >= min_petg, "iso_mount: ring_wall must be >= min_petg");
    assert(lift >= 0, "iso_mount: lift must be >= 0");
}

// ---------------------------------------------------------------------
// Geometry (raw, uncoloured). d = iso_mount_dims(...)
// ---------------------------------------------------------------------
module im_ring_3d(d, web_t, span, flange_t) {
    r_l = d[4]; r_lg = d[5]; r_i = d[7]; r_o = d[8];
    z_l0 = d[9]; z_w0 = d[10]; z_w1 = d[11]; z_rt = d[13];
    difference() {
        rotate_extrude($fn = im_fn) polygon([
            [r_i, 0], [r_o, 0], [r_o, z_rt], [r_i, z_rt],
            [r_l, z_w1 + span],           // funnel, 45 deg, `gap` under the F2 cone
            [r_l, z_l0],                  // lip edge
            [r_i, flange_t]]);            // chamfer, 45 deg, `gap` over F1's rim
        // lip groove for the web's outer rim (8-sided floor = anti-turn key)
        translate([0, 0, z_w0]) linear_extrude(web_t) im_lip_floor_2d(r_lg);
    }
}

module im_hub_3d(d, web_t, flange_t) {
    r_h = d[0]; r_gc = d[2]; r_t = d[3]; r_f = d[6];
    z_w0 = d[10]; z_c0 = d[12]; z_ht = d[14]; h_n = d[15];
    z_cf = z_c0 + (r_f - r_t);           // F2 cone underside at its rim
    difference() {
        rotate_extrude($fn = im_fn) polygon([
            [r_h, 0], [r_f, 0], [r_f, flange_t], [r_t, flange_t],     // F1
            [r_t, z_c0], [r_f, z_cf], [r_f, z_cf + h_n],             // F2 cone shell
            [r_t, z_c0 + h_n], [r_t, z_ht], [r_h, z_ht]]);           // tube
        // hub groove for the web's inner rim (hexagonal floor = anti-turn key)
        translate([0, 0, z_w0]) linear_extrude(web_t)
            difference() { circle(r = r_t + 1, $fn = im_fn); circle(r = r_gc, $fn = 6); }
    }
}

module im_lip_floor_2d(r_lg) circle(r = r_lg / cos(180 / im_lip_sides), $fn = im_lip_sides);

// TPU REGION (before the PETG is removed): the web slab between the two
// groove floors, minus the windows between the spokes
module im_tpu_region_3d(d, web_t, spokes, spoke_w) {
    r_gc = d[2]; r_t = d[3]; r_l = d[4]; r_lg = d[5]; z_w0 = d[10];
    translate([0, 0, z_w0]) linear_extrude(web_t) difference() {
        im_lip_floor_2d(r_lg);
        circle(r = r_gc, $fn = 6);
        if (spokes > 0) difference() {
            // windows span exactly hub tube .. lip edge, so the rims stay whole
            difference() { circle(r = r_l, $fn = im_fn); circle(r = r_t, $fn = im_fn); }
            for (k = [0 : spokes - 1]) rotate(90 + k * 360 / spokes)
                translate([0, -spoke_w / 2]) square([r_l + 1, spoke_w]);
        }
    }
}

module im_tpu_3d(d, web_t, span, spokes, spoke_w, flange_t)
    difference() {
        im_tpu_region_3d(d, web_t, spokes, spoke_w);
        im_hub_3d(d, web_t, flange_t);
        im_ring_3d(d, web_t, span, flange_t);
    }

// ---------------------------------------------------------------------
// Public modules
// ---------------------------------------------------------------------
module iso_mount(part = "both", hole_d = im_hole_d, web_t = im_web_t, span = im_span,
                 spokes = im_spokes, spoke_w = im_spoke_w, gap = im_gap, overlap = im_overlap,
                 flange_t = im_flange_t, shoulder = im_shoulder, groove = im_groove,
                 hub_wall = im_hub_wall, cap_t = im_cap_t, ring_wall = im_ring_wall, lift = im_lift) {
    im_check(hole_d, web_t, span, spokes, spoke_w, gap, overlap, flange_t, shoulder, groove,
             hub_wall, cap_t, ring_wall, lift);
    d = iso_mount_dims(hole_d, web_t, span, gap, overlap, flange_t, shoulder, groove, hub_wall,
                       cap_t, ring_wall, lift);
    if (part == "petg" || part == "both")
        im_colour(part, "petg") { im_ring_3d(d, web_t, span, flange_t); im_hub_3d(d, web_t, flange_t); }
    if (part == "tpu" || part == "both")
        im_colour(part, "tpu") im_tpu_3d(d, web_t, span, spokes, spoke_w, flange_t);
}

module iso_mount_ring(part = "petg", hole_d = im_hole_d, web_t = im_web_t, span = im_span,
                      gap = im_gap, overlap = im_overlap, flange_t = im_flange_t, shoulder = im_shoulder,
                      groove = im_groove, hub_wall = im_hub_wall, ring_wall = im_ring_wall) {
    d = iso_mount_dims(hole_d, web_t, span, gap, overlap, flange_t, shoulder, groove, hub_wall,
                       im_cap_t, ring_wall, im_lift);
    if (part != "tpu") im_colour(part, "petg") im_ring_3d(d, web_t, span, flange_t);
}

module iso_mount_hub(part = "petg", hole_d = im_hole_d, web_t = im_web_t, span = im_span,
                     gap = im_gap, overlap = im_overlap, flange_t = im_flange_t, shoulder = im_shoulder,
                     groove = im_groove, hub_wall = im_hub_wall, cap_t = im_cap_t, lift = im_lift) {
    d = iso_mount_dims(hole_d, web_t, span, gap, overlap, flange_t, shoulder, groove, hub_wall,
                       cap_t, im_ring_wall, lift);
    if (part != "tpu") im_colour(part, "petg") im_hub_3d(d, web_t, flange_t);
}

module iso_mount_tpu(part = "tpu", hole_d = im_hole_d, web_t = im_web_t, span = im_span,
                     spokes = im_spokes, spoke_w = im_spoke_w, gap = im_gap, overlap = im_overlap,
                     flange_t = im_flange_t, shoulder = im_shoulder, groove = im_groove,
                     hub_wall = im_hub_wall, cap_t = im_cap_t, ring_wall = im_ring_wall, lift = im_lift) {
    im_check(hole_d, web_t, span, spokes, spoke_w, gap, overlap, flange_t, shoulder, groove,
             hub_wall, cap_t, ring_wall, lift);
    d = iso_mount_dims(hole_d, web_t, span, gap, overlap, flange_t, shoulder, groove, hub_wall,
                       cap_t, ring_wall, lift);
    if (part != "petg") im_colour(part, "tpu") im_tpu_3d(d, web_t, span, spokes, spoke_w, flange_t);
}

// keep-out for your own PETG (drawn for every `part`; use inside difference())
module iso_mount_cut(part = "petg", hole_d = im_hole_d, web_t = im_web_t, span = im_span,
                     gap = im_gap, overlap = im_overlap, flange_t = im_flange_t,
                     ring_wall = im_ring_wall, lift = im_lift) {
    d = iso_mount_dims(hole_d = hole_d, web_t = web_t, span = span, gap = gap, overlap = overlap,
                       flange_t = flange_t, ring_wall = ring_wall, lift = lift);
    translate([0, 0, -1]) cylinder(r = d[8] - eps, h = d[14] + 2, $fn = im_fn);
}

// "both" -> coloured and rendered on its own (avoids preview z-fighting)
module im_colour(part, mat) {
    if (part == "both") color(mat == "petg" ? petg_colour : tpu_colour) render() children();
    else children();
}

// ---------------------------------------------------------------------
// Demo: flight-controller plate, 4 mounts on a pattern x pattern square
// ---------------------------------------------------------------------
module iso_plate(part = "both", pattern = 30.5, plate_t = 3, hole_d = undef, web_t = im_web_t,
                 span = im_span, spokes = im_spokes, spoke_w = im_spoke_w, gap = im_gap,
                 overlap = im_overlap, lift = im_lift, frame_hole_d = 3.4, board_g = undef) {
    hd = is_undef(hole_d) ? (pattern >= 25 ? im_hole_d : 2.2) : hole_d;
    bg = is_undef(board_g) ? (pattern >= 25 ? 30 : 15) : board_g;   // for the echoed estimate
    im_check(hd, web_t, span, spokes, spoke_w, gap, overlap, im_flange_t, im_shoulder, im_groove,
             im_hub_wall, im_cap_t, im_ring_wall, lift);
    d = iso_mount_dims(hole_d = hd, web_t = web_t, span = span, gap = gap, overlap = overlap, lift = lift);
    r_i = d[7]; r_o = d[8]; z_rt = d[13];
    p2 = pattern / 2;
    assert(pattern >= 2 * r_i + min_petg,
           str("iso_plate: mounts too close for pattern ", pattern, ": need >= ", 2 * r_i + min_petg,
               " (smaller hole_d / span / overlap)"));
    assert(plate_t >= min_petg && plate_t <= z_rt, "iso_plate: plate_t must be min_petg .. ring top");
    // frame holes on the edge midlines, clear of the rings
    h_min = p2 + sqrt(max(0, pow(r_o + frame_hole_d / 2 + min_petg, 2) - p2 * p2));
    half = max(p2 + r_o + 2, h_min + 4);
    hx = half - 4;
    r_c = min(8, pattern * sqrt(0.5) - r_o - 2);      // central cable hole
    mounts = [for (x = [-p2, p2], y = [-p2, p2]) [x, y]];
    k = iso_mount_k(web_t, span, spokes, spoke_w, hd);
    f = iso_mount_f(bg, 4, web_t, span, spokes, spoke_w, hd, hd < 3 ? 1.5 : 1.8);
    echo(str("iso_plate: pattern ", pattern, ", M", hd < 3 ? 2 : 3, " hubs, ring OD ", 2 * r_o,
             ", hub top ", d[14], " mm; per mount k_ax ", round(k[0] * 10) / 10, " N/mm, k_r ",
             round(k[1]), " N/mm; ", bg, " g board: f_z ", round(f[0]), " Hz, f_xy ", round(f[1]), " Hz"));
    if (part != "tpu") im_colour(part, "petg") union() {
        difference() {
            translate([-half, -half, 0]) linear_extrude(plate_t)
                offset(r = 4, $fn = 32) offset(delta = -4) square(2 * half);
            for (m = mounts) translate([m[0], m[1], 0]) iso_mount_cut("petg", hole_d = hd, web_t = web_t,
                span = span, gap = gap, overlap = overlap, lift = lift);
            for (a = [0 : 90 : 270]) rotate(a) translate([hx, 0, -1])
                cylinder(d = frame_hole_d, h = plate_t + 2, $fn = 32);
            if (r_c >= 3) translate([0, 0, -1]) cylinder(r = r_c, h = plate_t + 2, $fn = 64);
        }
        for (i = [0 : 3]) {
            // each ring, clipped out of the other mounts' bores (rings overlap
            // on tight patterns such as 20 x 20)
            difference() {
                translate([mounts[i][0], mounts[i][1], 0])
                    iso_mount_ring("petg", hole_d = hd, web_t = web_t, span = span, gap = gap, overlap = overlap);
                for (j = [0 : 3]) if (j != i) translate([mounts[j][0], mounts[j][1], -1])
                    cylinder(r = r_i, h = d[14] + 2, $fn = im_fn);
            }
            translate([mounts[i][0], mounts[i][1], 0])
                iso_mount_hub("petg", hole_d = hd, web_t = web_t, span = span, gap = gap, overlap = overlap,
                              lift = lift);
        }
    }
    if (part != "petg") for (m = mounts) translate([m[0], m[1], 0])
        iso_mount_tpu(part, hole_d = hd, web_t = web_t, span = span, spokes = spokes, spoke_w = spoke_w,
                      gap = gap, overlap = overlap, lift = lift);
}

// ---------------------------------------------------------------------
// Test piece: one default mount in a 48 x 34 x 4 frame, 2 M3 holes 36 apart
// ---------------------------------------------------------------------
module iso_mount_test(part = "both", hole_d = im_hole_d, web_t = im_web_t, span = im_span,
                      spokes = im_spokes, spoke_w = im_spoke_w, gap = im_gap, overlap = im_overlap,
                      size = [48, 34], plate_t = 4) {
    im_check(hole_d, web_t, span, spokes, spoke_w, gap, overlap, im_flange_t, im_shoulder, im_groove,
             im_hub_wall, im_cap_t, im_ring_wall, im_lift);
    d = iso_mount_dims(hole_d = hole_d, web_t = web_t, span = span, gap = gap, overlap = overlap);
    r_o = d[8];
    label = str(spokes > 0 ? str(spokes, "x", spoke_w) : "FULL", " T", web_t, " S", span);
    assert(size[1] / 2 >= r_o + 3.5, "iso_mount_test: frame too narrow for the label");
    k = iso_mount_k(web_t, span, spokes, spoke_w, hole_d);
    echo(str("iso_mount_test: ", label, ": k_ax ", round(k[0] * 10) / 10, " N/mm, k_r ", round(k[1]),
             " N/mm; with 8 g on the hub f_z ~ ", round(sqrt(k[0] * 1000 / 0.0098) / (2 * PI)), " Hz"));
    if (part != "tpu") im_colour(part, "petg") union() {
        difference() {
            translate([-size[0] / 2, -size[1] / 2, 0]) linear_extrude(plate_t)
                offset(r = 3, $fn = 24) offset(delta = -3) square(size);
            iso_mount_cut("petg", hole_d = hole_d, web_t = web_t, span = span, gap = gap, overlap = overlap);
            for (x = [-18, 18]) translate([x, 0, -1]) cylinder(d = 3.4, h = plate_t + 2, $fn = 32);
            translate([0, -(r_o + size[1] / 2) / 2, plate_t - 0.6]) linear_extrude(1)
                text(label, size = 2.6, font = "DejaVu Sans:style=Bold", halign = "center", valign = "center");
        }
        iso_mount_ring("petg", hole_d = hole_d, web_t = web_t, span = span, gap = gap, overlap = overlap);
        iso_mount_hub("petg", hole_d = hole_d, web_t = web_t, span = span, gap = gap, overlap = overlap);
    }
    if (part != "petg")
        iso_mount_tpu(part, hole_d = hole_d, web_t = web_t, span = span, spokes = spokes,
                      spoke_w = spoke_w, gap = gap, overlap = overlap);
}

// Cut-away preview (not a print): one default mount cut on the x-z plane
// (front half removed); the cut runs along two spokes (spokes sit at
// 90 + k * 360 / spokes deg: on the x and y axes for 4 spokes).
// The hub is drawn a lighter blue than the ring (both are PETG).
module iso_mount_section(part = "both") {
    module cut() difference() { children(); translate([-50, -100, -1]) cube([100, 100, 50]); }
    if (part != "tpu") {
        color(petg_colour) render() cut() iso_mount_ring("petg");
        color("#8fb4d6") render() cut() iso_mount_hub("petg");
    }
    if (part != "petg") color(tpu_colour) render() cut() iso_mount_tpu("tpu");
}

// ---------------------------------------------------------------------
if (show == "demo") iso_plate(part);
else if (show == "demo20") iso_plate(part, pattern = 20);
else if (show == "test") iso_mount_test(part);
else if (show == "section") iso_mount_section(part);
