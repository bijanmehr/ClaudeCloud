include <../params.scad>
// =======================================================================
// torsion_spring: Rosta-style rubber torsion unit with TPU cords in compression
// =======================================================================
// WHAT IT IS FOR
//   A compact rotary spring that is also a bushing, printed in one go: an
//   outer PETG square tube, an inner PETG square shaft turned 45 deg inside
//   it, and four TPU cords in the four corner pockets between them. Twist the
//   shaft and its faces squeeze and drag the cords against the tube walls;
//   let go and it springs back to centre. No pins, no metal spring, nothing
//   to assemble, some damping from the TPU, and it also carries radial load.
//   On rovers and drones: swing-arm / rocker / bogie suspension pivots,
//   landing-gear legs that fold back on touchdown, spring-centred steering or
//   camera tilt, pivoting bumpers and tail skids, compliant leg and gripper
//   joints, belt / track tensioners.
//
// HOW IT WORKS (every PETG/TPU joint holds by shape, never by adhesion)
//
//   section at mid-height (images/torsion_spring_slice.png):
//        +-------------------------------+   tube (PETG)
//        | TTTTTTTT :   :   : TTTTTTTT   |   T = TPU cord, one per tube corner
//        | TT  o  TT:   /\  :TT  o  TT   |   o = hollow core (core_d)
//        | TTTTTTTTT:  /  \ :TTTTTTTTT   |   : = air strip at each shaft corner
//        |         : /shaft\ :           |       (2 x relief wide), split by a
//        |-|       :<  PETG  >:       |-|       short PETG rib (-|) on the wall
//        |         : \      / :          |   the shaft is the square turned
//        | TTTTTTTTT:  \  / :TTTTTTTTT   |   45 deg; its corners point at the
//        | TT  o  TT:   \/  :TT  o  TT   |   middle of the tube walls
//        | TTTTTTTT :   :   : TTTTTTTT   |
//        +-------------------------------+
//   section through the axis and two tube corners (images/torsion_spring_section.png):
//                    |  drive  |              square drive (the shaft itself)
//        +-----------+         +-----------+  z = length
//        | top lip   |         |   top lip |  flat ring printed ON the cords
//        |  TTTT     |  shaft  |     TTTT  |
//        |  T  T     |  (PETG, |     T  T  |  cords (TPU), hollow core
//        |  TTTT    /|  bore)  |\    TTTT  |
//        | bottom  / |         | \  bottom |  bottom lip: a waist the shaft
//        |  lip   <  |  neck   |  >   lip  |  hooks over (45 deg cone) and
//        |         \ |         | /         |  under (its foot, 45 deg cone)
//   =====+==========\|__foot___|/==========+=====  flange on the bed, z = 0
//
//   - The cords: in the cross-section each cord is boxed in by two tube
//     walls and a shaft face (a triangle-like pocket: no in-plane direction
//     is free); along the axis the bottom lip is under it and the top lip is
//     printed on top of it. tools/capture_check.py confirms none of the four
//     cords can slide out in any of 26 directions with zero adhesion.
//   - The shaft is a separate PETG body that touches only TPU. At the bottom
//     it narrows to a round neck through a round hole in the bottom lip and
//     widens again below it into a foot on the bed: the lip sits in that
//     waist (0.6 mm vertical, >= 0.4 mm side clearance, 45 deg cones on both
//     sides), so the shaft cannot move up or down more than 0.6 mm. All the
//     clearances are solids of revolution, so they are the same at every
//     twist angle (without the cords the shaft would spin freely).
//   - The top lip only covers the cords; its hole (corner radius + 0.4)
//     lets the full square shaft out of the top as a square drive: the
//     strongest torque path out of the unit. The hex bore runs all through
//     (a 1/4" hex shaft or bit, or a bolt).
//   - When the shaft turns by theta, each face (a) rocks about its middle,
//     squeezing its cord on one side and lifting off on the other, and (b)
//     slides by theta * h along the cord, dragging it (TPU on PETG has high
//     friction). Both are resisted by the TPU between the face and the tube
//     corner. Past ~asin(relief / corner radius) (23 deg at the defaults)
//     the shaft corners also start to press into the cord tips, so the
//     spring gets progressively stiffer towards the design twist (a soft
//     bump stop). The air strips and the hollow cores give the nearly
//     incompressible TPU room to move; filled pockets would be ~15x stiffer.
//
// PRINTING (Bambu Lab H2D: PETG left nozzle, TPU 95A right; no supports)
//   As modelled, flange on the bed. Shaft foot and bottom lip start on the
//   bed; the shaft's cones overhang 45 deg over the lip with 0.6 mm air; the
//   cords start on the bottom lip; the top lip is printed on the cords (PETG
//   on TPU) and bridges <= 2.1 mm over the air strips (thanks to the ribs)
//   and the cores. Print the TPU solid (100 % infill, or >= 4 walls) for the
//   stiffness below; lower TPU infill makes it softer and less predictable.
//   Break the shaft free after printing by twisting it +-30 deg a few times.
//
// PARAMETERS (mm, degrees; defaults and the rule behind each)
//   size     = 24    tube outside width (square). Everything scales from it.
//   length   = 20    tube length along the axis (Z), not counting the drive.
//                    Torque grows with the cord length Lc = length - z_lb -
//                    lip_t (13.25 at the defaults); needs Lc >= 4.
//   wall     = 2.8   tube wall (>= 1.2). Sized so the walls, bent by the cord
//                    pressure, stay below ~15 MPa at 2 N*m (see ESTIMATES).
//   shaft    = auto  shaft square side across flats (11.62 at the defaults).
//                    Rule: the shaft corners, twisted by `twist`, would
//                    squeeze a cord filling the pocket by at most `strain`:
//                    corner-tip radius c_tip = (eps (a - rho) + rho) /
//                    (sin t + cos t - 1 + eps (1 - sin t)), a = size/2 - wall,
//                    rho = 0.414 corner_r, t = twist, eps = strain.
//   twist    = 30    design twist each way (used by the shaft rule).
//   strain   = 0.5   allowed nominal compression under a shaft corner at
//                    `twist` (TPU 95A takes 50 % locally, repeatedly).
//   corner_r = 1.0   shaft corner rounding (blunt corners do not cut TPU).
//   cav_r    = 1.5   tube cavity corner rounding.
//   relief   = auto  half-width of the air strip at each shaft corner
//                    (auto = 0.52 h rounded to 0.1 = 3.0 at the defaults;
//                    h = shaft / 2). The cords touch the shaft faces only
//                    over +-s1 = h - 1.414 relief around each face middle
//                    (s1 = 0.27 h): the main stiffness knob (bigger =
//                    softer and a later bump stop). Needs s1 >= 0.5;
//                    relief >= 1.5 adds the PETG ribs.
//   core_d   = auto  hollow core in each cord, centred on the pocket's
//                    inscribed circle (radius ri, 2.98 at the defaults);
//                    auto = the biggest with ~1.25 mm of TPU round it
//                    (3.4 at the defaults); 0 = solid cords (~2.2x stiffer).
//                    Needs >= 1.2 of TPU round it, also to the cord tips.
//   bore     = 6.5,  bore_shape = "hex": through bore. 6.5 hex = 1/4" hex
//                    shaft or screwdriver bit; 5.6 hex = M3 nut / M3 hex
//                    standoff; "square"; "round" (3.4 = M3 bolt). Needs
//                    >= 1.2 of PETG round it in the shaft and in the neck.
//   drive_h  = 4     how far the square shaft sticks out of the top as the
//                    drive for your arm (square socket, +0.2 mm per side).
//                    0 = flush.
//   lip_t    = 1.6   top lip thickness (8 layers).
//   rim      = 1.2   bottom lip thickness at its hole (>= min_petg).
//   flange_w = 7, flange_t = 3, hole_d = 3.4: square mounting flange round
//                    the tube base, 4 holes on a (size + flange_w) square
//                    (M3 on 31 mm at the defaults). flange_w = 0: no flange.
//   Derived (echoed): neck radius R_n = bore circumradius + 1.5, foot radius
//   R_f = R_n + 1.6, bottom lip / cord bottom z_lb = 5.15, cord top
//   z_lt = length - lip_t.
//
// ESTIMATES (TPU 95A E ~ 26 MPa, G ~ 9 MPa; PETG E ~ 2000 MPa, ~45 MPa)
//   Twist stiffness, simple compression model (one cord; h = shaft/2 = the
//   face's distance from the axis; s = position along the face from its
//   middle; the cord touches the face over |s| <= s1 = h - 1.414 relief;
//   pocket depth normal to the face D0 = 1.414 a - h - 0.414 cav_r at the
//   middle, falling off as t(s) = D0 - |s| towards the tube walls;
//   t1 = D0 - s1; Lc = cord length):
//   (a) compression: the face rocks about its middle and presses the cord
//       by theta*s: p = Ec theta s / t(s), moment p*s, one half of the face:
//         Mc = Ec Lc theta [D0^2 ln(D0/t1) - D0 s1 - s1^2/2]
//       Ec = 4/3 E = 35 MPa (long cord: plane strain, nearly incompressible)
//   (b) drag: the face also slides theta*h along the cord, which sticks to
//       it (TPU on PETG grips) and shears across its depth: tau = G theta h /
//       t(s), moment tau*h, whole contact:
//         Ms = 2 G Lc theta h^2 ln(D0/t1)
//   k0 = 4 (Mc + Ms) / theta x C_contact(s1/h) x C_core(core_d / 2 ri), where
//   the C factors come from 2D FEA (below; ri = pocket inradius) and are
//   1 for solid cords with a long contact.
//   Defaults: a = 9.2, h = 5.81, D0 = 6.58, s1 = 1.57, t1 = 5.01,
//     ln(D0/t1) = 0.272, Lc = 13.25:
//     per cord and mm: Ec x 0.24 mm^3 = 8 (compression) + G x 18.4 mm^3 = 165
//     (drag) N*mm/rad -> solid cords 4 x 13.25 x 174 = 9200 N*mm/rad
//     = 161 N*mm/deg; x C_contact(0.27) = 0.78 x C_core(0.57) = 0.47
//     -> k0 = 59 N*mm per degree (0.059 N*m/deg) at small angles.
//   So the drag does most of the work: the spring relies on the TPU
//   gripping the shaft faces. Frictionless (wet, greasy, dusty) FEA gives
//   ~1/4 of the torque at small angles.
//   FEA check (CalculiX, 2D plane strain, Neo-Hooke TPU G = 8.8 MPa,
//   rigid PETG, contact with friction 0.6 and no adhesion), default unit:
//@@FEA@@
//   The hand model matched FEA within 4-8 % for solid cords with a long
//   contact (relief 1 / 2.5: 18.0 vs 17.2, 4.6 vs 5.0 N*mm/deg per mm and
//   cord); the C factors cover shorter contacts and the hollow cores.
//   Expect +-30 % on a real print (TPU brand, infill, friction): measure
//   yours with the test piece and scale.
//   Stiffness knobs (FEA at 1 deg): length (linear); relief 2.5 -> ~2.1x,
//   3.5 -> ~0.4x (same cores); core_d 0 (solid) -> ~2.2x, 3.0 -> ~1.2x;
//   size: ~size^3 at equal proportions.
//   Strength (PETG, default unit):
//   - square drive / shaft in torsion (11.62 square, 6.5 hex bore; Saint-
//     Venant, finite differences): tau = T / 300 mm^3 -> 7 MPa at 2 N*m,
//     12 MPa at 3.5 N*m. The layer planes carry this shear, so keep
//     working torque <= ~2 N*m, peaks <= ~4 N*m.
//   - tube walls: each cord pushes its wall with F ~ T / (4 x 5.3 mm)
//     (FEA), ~4 mm off the wall middle; as a clamped beam across the cavity
//     M ~ 0.145 F (2a), sigma = 6 M / (Lc wall^2) -> 12 MPa at 1.7 N*m
//     (20 deg), ~20 MPa at 3 N*m (the lips at both ends roughly halve this).
//   - axial: the foot / upper cone bear on the bottom lip's waist; lip rim
//     in shear 2 pi (R_n + 0.4) rim = 43 mm^2 -> ~600 N to fail; neck in
//     tension (layers) ~1000 N. Use ~300 N working.
//   - radial (as a bushing): ~120 N/mm (two cords squeezed, four dragged);
//     after a - c_tip = 1.4 mm the shaft corners touch the tube walls
//     (PETG on PETG plain bearing).
//   - flange: 4 x M3 on a 31 mm square: 34 N per screw at 3 N*m.
//   TPU strain (FEA): peak stretch at the cord corners 11 % at 5 deg, 24 %
//   at 10, 36 % at 15, 54 % at 20 deg (peak squeeze 37 %); 95 % of the
//   cord stays below 20 % at 20 deg. Use ~+-20 deg as the working range
//   and +-30 deg as the limit.
//   Sizing example: rover of 4 kg on 4 swing arms 80 mm long: 9.8 N x 80 =
//   0.78 N*m per pivot -> ~11 deg static sag, ~20 deg left for bumps.
//   Drone of 1.5 kg landing at 3 g on 4 legs 60 mm long: 0.66 N*m -> ~10 deg.
//
// TEST PIECE (show = "test"): torsion_spring_test() - the default unit with
//   a +-40 deg angle scale on its flange (ticks every 5 deg) and a loose
//   lever printed beside it. Turn the lever over and push its square pocket
//   onto the drive (it sits 1 mm clear of the tube top; a 1/4" hex bit
//   through lever and shaft takes the torque off the drive if you want).
//   Screw the flange down with the axis vertical, pull the lever's 50 mm
//   hole sideways with a luggage scale or spring scale held at right angles
//   to the lever, and note the force at 5, 10, 20, 30 deg both ways, then
//   on the way back (hysteresis), and the angle it returns to (set).
//   Torque [N*mm] = force [N] x 50. Compare with the table above; the ratio
//   tells you your TPU / friction factor for torsion_spring_k().
//
// MODULES (all take `part` = "petg" | "tpu" | "both" first)
//   torsion_spring(part, size, length, wall, shaft, twist, strain, corner_r,
//       cav_r, relief, core_d, bore, bore_shape, drive_h, lip_t, rim,
//       flange_w, flange_t, hole_d) { your PETG }
//       the unit with its flange; children() are fused to the PETG tube.
//   torsion_spring_unit(part, size, length, ... same, no flange ...) { your PETG }
//       the unit alone (tube outside = size square, z = 0 .. length) - build
//       it into your own bracket, chassis wall or arm: union your PETG with
//       the tube's outside as children().
//   torsion_spring_test(part, size, length, wall, shaft, relief, core_d,
//       drive_h, flange_w, flange_t)   the test piece above.
//   Functions: torsion_spring_k(size, length, wall, shaft, relief, core_d)
//       initial stiffness (N*mm per deg); torsion_spring_torque(deg, ...)
//       estimated torque (N*mm) including the progressive rise;
//       torsion_spring_geom(...) all derived sizes (read with ts_v(g, key)).
//
// RULES FOR YOUR OWN PARTS
//   - Fuse PETG only to the outside of the tube (|x| or |y| >= size/2 - wall)
//     and keep it out of the region above the top lip within the drive's
//     swept circle (radius c_tip + 0.4, 8.2 at the defaults).
//   - Your arm/lever goes on the square drive (socket = shaft + 0.4, corner
//     radius corner_r + 0.2) and must stay >= 0.6 mm clear of the tube top
//     (a socket depth of drive_h - 1 does that), or put a washer between.
//   - Clamp the arm axially with a bolt through the bore + washer under the
//     foot, if it must not pull off; the shaft takes ~300 N axially.
//   - Limit the swing with your own stop at about +-30 deg; past that the
//     TPU strain and the PETG stresses climb fast.
//
// LIMITATIONS
//   - Friction matters: most of the torque comes from the cords gripping
//     the shaft faces. Keep oil, grease and grit out (the annular gap
//     round the drive and the air strips are open at the top). A slipping
//     cord also gives some hysteresis (the shaft may not return the last
//     degree or two to exactly zero).
//   - TPU creeps: under a constant torque (a vehicle's weight) the unit
//     sags a few degrees more over days and takes a set. Design the static
//     sag <= ~10-15 deg and keep the rest for bumps.
//   - The estimate is +-30 % (TPU brand and infill, friction, print
//     squish). Calibrate with the test piece. FEA with friction converged
//     to 20 deg; beyond that the numbers above are the frictionless shape
//     scaled (see the table).
//   - PETG torque limit ~2 N*m working / ~4 N*m peak at the default size
//     (walls and the drive's layer planes); bigger units scale ~size^3.
//   - A bushing, not a precision bearing: 0.6 mm axial play, ~1/120 mm
//     per N radially, and it tilts under a bending moment. Keep the arm
//     close to the tube top, or carry big side loads on a separate pin.
//   - TPU stiffens in the cold (much below 0 C) and softens when hot;
//     PETG is fine to ~70 C.
//   - The capture check treats the TPU as rigid. A very large twist could
//     roll a cord partly out of its pocket at the open top (between the
//     drive and the top lip); stop the swing at ~+-30 deg.
// =======================================================================

part = "both";   // [petg, tpu, both]
show = "demo";   // [demo, test, section, slice]

ts_fn = 72;      // round parts; divisible by 6 and 8 so hex and 45 deg points are facet vertices

// -----------------------------------------------------------------------
// Defaults (shared by every module, so the parts always agree)
// -----------------------------------------------------------------------
ts_size = 24;      ts_length = 20;    ts_wall = 2.8;
ts_twist = 30;     ts_strain = 0.5;   ts_corner_r = 1.0;  ts_cav_r = 1.5;
ts_relief = undef; ts_core_d = undef;   // undef = auto (see PARAMETERS)
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
        ri = (2 * a - c) * (2 - sqrt(2)) / 2,        // inradius of a corner pocket
        relief = is_undef(relief) ? round(10 * 0.52 * h) / 10 : relief,
        core_auto = floor(10 * 2 * (ri - 1.25)) / 10,
        core_d = is_undef(core_d) ? (core_auto >= 1.6 ? core_auto : 0) : core_d,
        bore_rc = bore_shape == "hex" ? bore / sqrt(3) : bore_shape == "square" ? bore / sqrt(2) : bore / 2,
        R_n = max(bore_rc + min_petg + 0.3, 2.5),     // neck radius (passes the bottom lip)
        R_f = R_n + xy_clear + min_petg,             // foot radius on the bed (under the lip)
        z_f = R_f - R_n,                             // top of the foot cone
        z_n1 = z_f + rim + 2 * (z_clear - xy_clear), // top of the neck
        z_b1 = z_n1 + c_tip - R_n,                   // full square from here up
        z_lb = z_b1 - z_clear,                       // bottom lip top = cord bottom
        z_lt = length - lip_t,                       // top lip underside = cord top
        z_tpu0 = z_n1 - z_clear + max(h, R_n + xy_clear) - R_n,   // lowest TPU (wedge under the face middle)
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
// (simple compression model x FEA calibration; see ESTIMATES)
function ts_k(g, E = 26, G = 9) =
    let(D0 = ts_v(g, "D0"), s1 = ts_v(g, "s1"), h = ts_v(g, "h"), t1 = D0 - s1, lg = ln(D0 / t1),
        Ic = D0 * D0 * lg - D0 * s1 - s1 * s1 / 2,
        Is = 2 * h * h * lg,
        f = lookup(s1 / h, ts_contact_table) * lookup(ts_v(g, "core_d") / (2 * ts_v(g, "ri")), ts_core_table),
        kc = 4 * ts_v(g, "Lc") * (4 / 3) * E * Ic * f * PI / 180,
        ks = 4 * ts_v(g, "Lc") * G * Is * f * PI / 180)
    [kc + ks, kc, ks];
// hand model / FEA (2D plane strain, stick friction), solid cords, vs the contact ratio s1 / h
ts_contact_table = [[0, 0.45], [0.15, 0.59], [0.27, 0.78], [0.39, 0.93], [0.76, 1.04], [1, 1.05]];
// hollow cords: FEA factor vs core_d / (2 * ri)  (0.6 and up extrapolated)
ts_core_table = [[0, 1], [0.5, 0.56], [0.57, 0.47], [0.7, 0.33], [1, 0.2]];
// secant torque-per-degree multiplier vs twist (FEA of the default unit, stick friction)
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
