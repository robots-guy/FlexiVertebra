// ===================================================================
// PARAMETRIC FLEXI VERTBRAE ROBOT WITH BLADE FLEXURES + TENDON HOLES
// ===================================================================
// Features:
// Separate tendon holes for Head+Spine and Tail
// Tail tendon holes support tapering (diameter reduces toward tip)
// Independent offset control for head+spine vs tail
// Head+Spine tendon tapering follows spine_tapering_factor
// Tail tendon tapering follows tail_tapering_factor
//
// ================
// FLEXURE METRICS
// ================
//
// Three geometry-only, material-agnostic design indices are
// computed and displayed on the metrics plate.
//
// ── S_geo = b·t³ / L  (Geometric Stiffness Index) ────────────
//   The Pseudo-Rigid-Body Model (PRBM) represents a small-length
//   flexural pivot as a torsional spring with constant:
//     K_θ = K_Θ · E · I / L = K_Θ · E · (b·t³/12) / L
//   where K_Θ ≈ 2.65 (fixed-pinned) or 12 (fixed-fixed, as here).
//   S_geo = b·t³/L is the pure geometric factor. Multiply by E·K_Θ/12
//   to recover actual rotational stiffness in N·mm/rad.
//   K_Θ and E cancel when computing the Spine/Tail ratio, so the
//   ratio column is material-independent and boundary-condition-independent.
//   PRBM accuracy is highest when t/L < 0.1 (i.e., FR > 10).
//   For FR = 2–6 (typical here) it is a validated first-order approximation,
//   the standard tool used throughout soft robotics for design guidance.
//   The t³ cubic dominance means doubling t → 8× stiffer.
//   b (robot_thickness, the out-of-plane width) scales stiffness linearly.
//
// ── FR = L / t  (Flex Ratio) ─────────────────────────────────
//   Euler-Bernoulli beam theory: ε = t·θ/(2L)
//   Maximum safe bending angle before overstress:
//     θ_max = 2·ε_allow·(L/t) = 2·ε_allow·FR
//   For TPU 80A (ε_allow ≈ 500%) and TPU 95A (ε_allow ≈ 400%), the
//   difference in ε_allow (~20%) is small vs. the FR range 2–15 exposed
//   by the sliders, making FR highly material-agnostic in practice.
//   Proposed Practical thresholds:
//     FR < 2  → nearly rigid, minimal angular travel
//     FR 2–4  → firm flex
//     FR 4–8  → moderate flex, typical working range
//     FR > 8  → highly flexible, monitor AR for buckling
//
// ── AR = t / b  (Aspect Ratio, Lateral Buckling Flag) ─────────
//   The ratio of weak-axis to strong-axis second moment of area is:
//     I_weak / I_strong = (b·t³/12) / (t·b³/12) = (t/b)² = AR²
//   Low AR → in-plane stiffness >> lateral stiffness → lateral-torsional
//   buckling risk: the joint twists sideways instead of bending cleanly.
//   AR thresholds (conservative, for tendon-loaded TPU blade flexures):
//     AR < 0.10 → WARN: high lateral buckling risk
//     AR 0.10–0.25 → LOW: moderate risk, monitor
//     AR > 0.25 → OK: low buckling risk
//   Note: full LTB critical moment also depends on L; AR is a fast flag.
//
// ── Spine/Tail ratio columns ──────────────────────────────────
//   Ratio > 1 means tail joint is stiffer / more flexible / higher risk
//   than the spine joint. The goal for a lizard-like gait is typically
//   ratio_S_geo < 1 (tail softer) and ratio_FR > 1 (tail more flexible).
//
// =====================================================
// Material Recommendation: TPU 95A 100% infill
// Units: millimeters (mm)
// =====================================================

/* [00 - Basic: Quick Setup] */

// Add head segment
enable_head = true; // [true, false]

// Add tail segments
enable_tail = true; // [true, false]

// Add limbs
enable_limbs = true; // [true, false]

// Add tendon/cable channels (holes)
enable_tendon_holes = true; // [true, false]

// Overall thickness / Z height (mm)
robot_thickness = 12; // [2:0.5:20]

// Show metrics plate below robot
enable_info_plate = false; // [true, false]

/* [01 - Spine Parameters] */

// Spine profile shape
spine_shape = "hexagon"; // ["rounded_rect","hexagon"]

// Number of spine segments
spine_num_bones = 6; // [3:1:30]

// Spine segment base length (mm)
spine_bone_length = 15; // [1:1:40]

// Spine bone width (mm)
spine_bone_width = 25; // [10:1:45]

// Spine taper — values below 1 shrink bones toward the tail
spine_tapering_factor = 0.95; // [0.85:0.01:1.00]

// Spine flexure thickness — stiffer when thicker (mm)
spine_joint_thickness = 2; // [0.8:0.1:3.5]

// Spine flexure length — longer allows more bend (mm)
spine_joint_length = 6; // [3:1:25]

/* [02 - Tail Parameters] */

// Tail shape
tail_shape = "rounded_rect"; // ["rounded_rect","hexagon"]

// Number of tail segments
tail_num_bones = 8; // [3:1:40]

// Tail segment base length (mm)
tail_bone_length = 5; // [1:1:30]

// Tail bone width (mm)
tail_bone_width = 20; // [5:1:30]

// Tail taper — values below 1 shrink bones toward the tip
tail_tapering_factor = 0.94; // [0.85:0.01:1.00]

// Tail flexure thickness — stiffer when thicker (mm)
tail_joint_thickness = 2; // [0.8:0.1:3.5]

// Tail flexure length — longer allows more bend (mm)
tail_joint_length = 8; // [3:1:25]

/* [03 - Head Parameters] */

// Head shape
head_shape = "hexagon"; // ["rounded_rect","hexagon"]

// Head bone length (mm)
head_length = 20; // [2:1:40]

// Head bone width (mm)
head_width = 28; // [10:1:40]

/* [04 - Limb Geometry] */

// Limb length in Y direction (mm)
limb_length = 22; // [10:1:60]

// Limb width in X direction (mm)
limb_width = 5; // [4:1:25]

// Limb thickness / Z height (mm)
limb_thickness = 3; // [2:0.5:12]

// Limb flexure length (mm)
limb_joint_length = 10; // [3:1:18]

// Limb flexure width (mm)
limb_joint_width = 2; // [1:0.5:12]

// Leg angles (degrees)
front_leg_splay_deg = 0;  // [-45:1:45]
rear_leg_splay_deg  = 0;  // [-45:1:45]

/* [05 - Tendon Channels] */

// Head and spine tendon offset from centerline +/-Y (mm)
head_spine_tendon_offset_y = 6.0; // [1:0.5:30]

// Head and spine tendon channel diameter (mm)
head_spine_tendon_diameter = 2.5; // [0.8:0.1:12]

// Tail tendon offset from centerline +/-Y (mm)
tail_tendon_offset_y = 4.0; // [1:0.5:30]

// Tail tendon channel diameter (mm)
tail_tendon_diameter = 2.4; // [0.8:0.1:12]

/* [06 - Advanced: Rounding and Fillets] */

head_bone_fillet  = 2.0; // [0:0.1:8]
spine_bone_fillet = 2.0; // [0:0.1:8]
tail_bone_fillet  = 1.5; // [0:0.1:8]



/* [Hidden] */


// Plate style
plate_mode = "embossed"; // ["embossed","engraved"]

// Plate width (mm)
plate_width = 190; // [80:1:220]

// Plate height (mm)
plate_height = 54; // [35:1:100]

// Plate thickness (mm)
plate_thickness = 1.6; // [0.8:0.1:4]

// Text size (mm)
plate_text_size = 4.2; // [2:0.25:8]

// Text relief height — emboss height or engrave depth (mm)
plate_text_height = 0.6; // [0.2:0.1:3]

// Gap between robot bottom edge and plate top (mm)
plate_margin_y = 10; // [0:1:30]

// Plate base color (hex). Visible in OpenSCAD + Bambu .3mf export.
plate_body_color = "#faf5f5";

// Text color (hex). Visible in OpenSCAD + Bambu .3mf export.
plate_text_color = "#ff6600";

head_rect_radius   = head_bone_fillet;
head_width_fillet  = head_bone_fillet;
head_hex_radius    = head_bone_fillet;

spine_rect_radius  = spine_bone_fillet;
spine_width_fillet = spine_bone_fillet;
spine_hex_radius   = spine_bone_fillet;

tail_rect_radius   = tail_bone_fillet;
tail_width_fillet  = tail_bone_fillet;
tail_hex_radius    = tail_bone_fillet;

overlap_mm = 1.0;
FAST_HOLES_PREVIEW = true;
WORK_MODE = true; // [true, false]
tail_tendon_tapering_factor = tail_tapering_factor;

$fa = (WORK_MODE && $preview) ? 25 : 6;
$fs = (WORK_MODE && $preview) ? 1.5 : 0.25;
$fn = (WORK_MODE && $preview) ? 10  : 48;


// =====================================================
// GEOMETRY HELPERS
// =====================================================
function sum(v, i=0, s=0) = i < len(v) ? sum(v, i+1, s+v[i]) : s;

function get_spine_bone_length(index) =
    spine_bone_length * pow(spine_tapering_factor, index);

function get_spine_bone_width(index) =
    spine_bone_width * pow(spine_tapering_factor, index);

function get_spine_cumulative_position(index) =
    (index == 0) ? 0 :
    get_spine_cumulative_position(index - 1) +
    get_spine_bone_length(index - 1) +
    (spine_joint_length - 2*overlap_mm);

function calculate_spine_length() =
    let(
        bone_lengths = [for (i = [0:spine_num_bones-1]) get_spine_bone_length(i)],
        total_bone_length = sum([for (i = bone_lengths) i]),
        total_joint_length = (spine_num_bones - 1) * (spine_joint_length - 2*overlap_mm)
    )
    total_bone_length + total_joint_length;

function get_tail_bone_length(index) =
    tail_bone_length * pow(tail_tapering_factor, index);

function get_tail_cumulative_position(index) =
    (index == 0) ? 0 :
    get_tail_cumulative_position(index - 1) +
    get_tail_bone_length(index - 1) +
    (tail_joint_length - 2*overlap_mm);

function calculate_tail_length() =
    let(
        bone_lengths = [for (i = [0:tail_num_bones-1]) get_tail_bone_length(i)],
        total_bone_length = sum([for (i = bone_lengths) i]),
        total_joint_length = (tail_num_bones - 1) * (tail_joint_length - 2*overlap_mm)
    )
    total_bone_length + total_joint_length;

function calculate_robot_length() =
    head_length +
    (spine_joint_length - 2*overlap_mm) +
    calculate_spine_length() +
    (tail_joint_length - 2*overlap_mm) +
    calculate_tail_length();


// =====================================================
// FLEXURE METRIC FUNCTIONS
// =====================================================
//
// S_geo(t, L, b) — Howell & Midha (1994) PRBM geometric factor
//   K_θ = E · K_Θ · S_geo / 12   [N·mm/rad]
//   K_Θ ≈ 12 for double-clamped blade (fixed-fixed BC)
//   t³ dominance: 2× t → 8× stiffer
//
function s_geo(t, L, b) =
    (t <= 0 || L <= 0 || b <= 0) ? 0 : (b * t*t*t) / L;

// FR(t, L) — Flex Ratio, Howell (2001) §5.2.5
//   θ_max = 2 · ε_allow · FR   (Euler-Bernoulli bending strain)
//   Higher FR → more angular travel before overstress
//
function flex_ratio(t, L) =
    (t <= 0 || L <= 0) ? 0 : L / t;

// AR(t, b) — Aspect Ratio, Timoshenko & Gere (1961) LTB criterion
//   AR² = I_weak / I_strong = (t/b)²
//   Low AR → high lateral-torsional buckling risk under tendon load
//
function aspect_ratio(t, b) =
    (t <= 0 || b <= 0) ? 0 : t / b;

// Safe ratio helper (avoids division-by-zero in display)
function safe_ratio(num, den) = (den <= 0) ? 0 : num / den;

// Format to 2 decimal places
function fmt2(x) = str(round(x * 100) / 100);

// ---- Qualitative labels ----
// S_geo — joint stiffness (how hard the joint is to bend).
// Calibrated for robot_thickness 10-15mm, typical TPU blade joints.
function stiff_label(sg) =
    (sg < 1.5) ? "SOFT"   :
    (sg < 6)   ? "MEDIUM" :
    (sg < 18)  ? "FIRM"   : "STIFF";

// FR — bending range label (how far the joint can travel per unit tendon pull).
// Uses distinct vocabulary from stiff_label so the two scales cannot be confused.
// Howell (2001) ss5.2.5: theta_max proportional to FR.
function flex_label(fr) =
    (fr < 2)  ? "NO-FLEX"  :
    (fr < 4)  ? "LOW-FLEX" :
    (fr < 8)  ? "MID-FLEX" : "HI-FLEX";

// AR — lateral buckling risk (Timoshenko and Gere 1961 LTB criterion).
// CAUTION/SAFE unambiguous to non-engineers; avoids LOW/OK confusion.
function ar_label(ar) =
    (ar < 0.10) ? "BUCKLE-RISK" :
    (ar < 0.25) ? "CAUTION"     : "SAFE";


// =====================================================
// FILLET HELPERS
// =====================================================

module prism_fillet_x_edges_only(length, width, thickness, fillet_r=1, q=24) {
    r = max(0, min(fillet_r, min(width, thickness)/2 - 0.01));
    if (r <= 0) {
        translate([-length/2, -width/2, 0]) cube([length, width, thickness], center=false);
    } else {
        translate([0, 0, thickness/2])
            rotate([0, 90, 0])
                let($fn=q, $fa=6, $fs=0.25)
                    linear_extrude(height=length, center=true, convexity=10)
                        offset(r=r)
                            square([thickness - 2*r, width - 2*r], center=true);
    }
}

module prism_fillet_y_edges_only(length, width, thickness, fillet_r=1, q=24) {
    r = max(0, min(fillet_r, min(length, thickness)/2 - 0.01));
    if (r <= 0) {
        translate([-length/2, -width/2, 0]) cube([length, width, thickness], center=false);
    } else {
        translate([0, 0, thickness/2])
            rotate([-90, 0, 0])
                let($fn=q, $fa=6, $fs=0.25)
                    linear_extrude(height=width, center=true, convexity=10)
                        offset(r=r)
                            square([length - 2*r, thickness - 2*r], center=true);
    }
}


// =====================================================
// PRIMITIVES
// =====================================================

module rounded_rect_prism(length, width, thickness, radius=1, width_fillet=0) {
    rx  = max(0, min(radius, min(width, thickness)/2 - 0.01));
    rxy = max(0, min(width_fillet, min(length, width)/2 - 0.01));
    q   = $preview ? 18 : 48;

    module base_xy_rounded() {
        q_xy = $preview ? 32 : 80;
        if (rxy <= 0) {
            translate([-length/2, -width/2, 0])
                cube([length, width, thickness], center=false);
        } else {
            linear_extrude(height=thickness, center=false, convexity=10)
                let($fn=q_xy, $fa=4, $fs=0.2)
                    offset(r=rxy)
                        square([length - 2*rxy, width - 2*rxy], center=true);
        }
    }
    if (rx <= 0) {
        base_xy_rounded();
    } else {
        intersection() {
            base_xy_rounded();
            prism_fillet_x_edges_only(length, width, thickness, fillet_r=rx, q=q);
        }
    }
}

module hex_prism(length, width, thickness, corner_r=0) {
    sx = length / 2;
    sy = width  / sqrt(3);
    cr = max(0, min(corner_r, min(width, length)*0.20));
    linear_extrude(height=thickness, center=false, convexity=10) {
        if (cr > 0) {
            offset(r=cr) offset(delta=-cr)
                polygon(points=hex_points_scaled(sx, sy));
        } else {
            polygon(points=hex_points_scaled(sx, sy));
        }
    }
}

function hex_points_scaled(sx, sy) =
    [ for (k=[0:5]) [cos(k*60)*sx, sin(k*60)*sy] ];

module segment_primitive(length, width, thickness, rect_r, shape, hex_r, width_fillet=0) {
    if (shape == "hexagon")
        hex_prism(length, width, thickness, corner_r=hex_r);
    else
        rounded_rect_prism(length, width, thickness, radius=rect_r, width_fillet=width_fillet);
}


// =====================================================
// BODY PARTS
// =====================================================
module head() {
    segment_primitive(head_length, head_width, robot_thickness,
                      rect_r=head_rect_radius, shape=head_shape, hex_r=head_hex_radius,
                      width_fillet=head_width_fillet);
}

module spine_bone(bone_index=0) {
    taper_mult  = pow(spine_tapering_factor, bone_index);
    bone_length = spine_bone_length * taper_mult;
    bone_width  = spine_bone_width  * taper_mult;
    segment_primitive(bone_length, bone_width, robot_thickness,
                      rect_r=spine_rect_radius, shape=spine_shape, hex_r=spine_hex_radius,
                      width_fillet=spine_width_fillet);
}

module tail_bone(bone_index) {
    taper_mult        = pow(tail_tapering_factor, bone_index);
    final_bone_length = tail_bone_length * taper_mult;
    final_bone_width  = tail_bone_width  * taper_mult;
    segment_primitive(final_bone_length, final_bone_width, robot_thickness,
                      rect_r=tail_rect_radius, shape=tail_shape, hex_r=tail_hex_radius,
                      width_fillet=tail_width_fillet);
}


// =====================================================
// FLEXURES
// =====================================================
module flexure_joint(joint_thickness, joint_length, width) {
    cube([joint_length + 2*overlap_mm, joint_thickness, robot_thickness], center=true);
}


// =====================================================
// LIMBS
// =====================================================
module limb() {
    rounded_rect_prism(limb_width, limb_length, limb_thickness, radius=1, width_fillet=0.2);
}

module limb_flexure() {
    cube([limb_joint_width, limb_joint_length, limb_thickness], center=true);
}

module limb_with_joint(side_multiplier, bone_width, pair_splay_deg=0) {
    clearance = -2;
    flexure_start_y = side_multiplier * (bone_width/2 + clearance);
    yaw = side_multiplier * (is_undef(pair_splay_deg) ? 0 : pair_splay_deg);

    translate([0, flexure_start_y, 0])
        rotate([0, 0, yaw])
            translate([0, -flexure_start_y, 0]) {
                flexure_center_y = flexure_start_y + side_multiplier * (limb_joint_length/2);
                translate([0, flexure_center_y, limb_thickness/2])
                    limb_flexure();
                limb_start_y  = flexure_start_y + side_multiplier * (limb_joint_length + clearance);
                limb_center_y = limb_start_y    + side_multiplier * (limb_length/2);
                translate([0, limb_center_y, 0]) limb();
            }
}


// =====================================================
// ASSEMBLIES
// =====================================================
module spine_assembly() {
    union() {
        for (i = [0:spine_num_bones-1]) {
            bone_length    = get_spine_bone_length(i);
            bone_width     = get_spine_bone_width(i);
            cumulative_pos = get_spine_cumulative_position(i);
            bone_pos       = cumulative_pos + bone_length/2;

            translate([bone_pos, 0, 0]) spine_bone(i);

            if (enable_limbs) {
                if (spine_num_bones >= 4) {
                    if (i == 1 || i == spine_num_bones - 2) {
                        pair_splay = (i == 1) ? front_leg_splay_deg : rear_leg_splay_deg;
                        translate([bone_pos, 0, 0]) {
                            limb_with_joint( 1, bone_width, pair_splay);
                            limb_with_joint(-1, bone_width, pair_splay);
                        }
                    }
                } else if (spine_num_bones >= 2) {
                    if (i == 0 || i == spine_num_bones - 1) {
                        pair_splay = (i == 0) ? front_leg_splay_deg : rear_leg_splay_deg;
                        translate([bone_pos, 0, 0]) {
                            limb_with_joint( 1, bone_width, pair_splay);
                            limb_with_joint(-1, bone_width, pair_splay);
                        }
                    }
                }
            }

            if (i < spine_num_bones - 1) {
                joint_center = cumulative_pos + bone_length + (spine_joint_length - 2*overlap_mm)/2;
                translate([joint_center, 0, robot_thickness/2])
                    flexure_joint(spine_joint_thickness, spine_joint_length, bone_width);
            }
        }
    }
}

module tail_assembly() {
    union() {
        for (i = [0:tail_num_bones-1]) {
            bone_length    = get_tail_bone_length(i);
            cumulative_pos = get_tail_cumulative_position(i);
            bone_pos       = cumulative_pos + bone_length/2;
            translate([bone_pos, 0, 0]) tail_bone(i);
            if (i < tail_num_bones - 1) {
                joint_center = cumulative_pos + bone_length + (tail_joint_length - 2*overlap_mm)/2;
                translate([joint_center, 0, robot_thickness/2])
                    flexure_joint(tail_joint_thickness, tail_joint_length, 0);
            }
        }
    }
}

module lizard_robot_solid() {
    union() {
        if (enable_head) head();

        head_end_x = head_length/2;
        head_to_spine_joint_center = head_end_x + (spine_joint_length - 2*overlap_mm)/2;
        translate([head_to_spine_joint_center, 0, robot_thickness/2])
            flexure_joint(spine_joint_thickness, spine_joint_length, head_width);

        spine_start_pos = head_end_x + (spine_joint_length - 2*overlap_mm);
        translate([spine_start_pos, 0, 0]) spine_assembly();

        spine_end_pos = spine_start_pos + calculate_spine_length();

        spine_to_tail_joint_center = spine_end_pos + (tail_joint_length - 2*overlap_mm)/2;
        translate([spine_to_tail_joint_center, 0, robot_thickness/2])
            flexure_joint(tail_joint_thickness, tail_joint_length,
                          get_spine_bone_width(spine_num_bones-1));

        tail_start_pos = spine_end_pos + (tail_joint_length - 2*overlap_mm);
        if (enable_tail) {
            translate([tail_start_pos, 0, 0]) tail_assembly();
        }
    }
}


// =====================================================
// TENDON HOLES
// =====================================================

module teardrop_2d_pointX(d) {
    r         = d/2;
    tip       = d*0.1;
    tip_round = max(d*0.1, 0.25);
    hull() {
        circle(r=r, $fn=40);
        translate([r + tip, 0]) circle(r=tip_round, $fn=32);
    }
}

module tendon_channel(length, d) {
    rotate([180,0,0])
    rotate([0,90,0])
        linear_extrude(height=length, center=false, convexity=10)
            teardrop_2d_pointX(d);
}

module tendon_channel_tapered(length, d, offset_start, tapering_factor) {
    offset_end  = offset_start * tapering_factor;
    n_segments  = max(10, ceil(length / 2));
    for (i = [0:n_segments-1]) {
        x_pos          = (i / n_segments) * length;
        seg_len        = length / n_segments;
        t_param        = i / n_segments;
        current_offset = offset_start + (offset_end - offset_start) * t_param;
        translate([x_pos, current_offset, 0])
            tendon_channel(seg_len * 1.01, d);
    }
}

module head_tendon_holes() {
    zc           = robot_thickness/2;
    margin_start = 20;
    x_start      = -head_length/2 - margin_start;
    x_end        = head_length/2;
    L            = x_end - x_start + 2;
    for (s = [-1, 1])
        translate([x_start, s*head_spine_tendon_offset_y, zc])
            tendon_channel(L, head_spine_tendon_diameter);
}

module spine_tendon_holes(spine_start_x, spine_end_x) {
    zc      = robot_thickness/2;
    x_start = spine_start_x - 2;
    L       = spine_end_x - spine_start_x + 3;
    for (s = [-1, 1]) {
        translate([x_start, 0, zc]) {
            if (spine_tapering_factor < 1.0) {
                tendon_channel_tapered(L, head_spine_tendon_diameter,
                                       s * head_spine_tendon_offset_y,
                                       spine_tapering_factor);
            } else {
                translate([0, s * head_spine_tendon_offset_y, 0])
                    tendon_channel(L, head_spine_tendon_diameter);
            }
        }
    }
}

module tail_tendon_holes(tail_start_x, tail_len) {
    zc         = robot_thickness/2;
    x_start    = tail_start_x - 2;
    margin_end = 20;
    L          = tail_len + margin_end;
    for (s = [-1, 1]) {
        translate([x_start, 0, zc]) {
            if (tail_tendon_tapering_factor < 1.0) {
                tendon_channel_tapered(L, tail_tendon_diameter,
                                       s * tail_tendon_offset_y,
                                       tail_tendon_tapering_factor);
            } else {
                translate([0, s*tail_tendon_offset_y, 0])
                    tendon_channel(L, tail_tendon_diameter);
            }
        }
    }
}


// =====================================================
// METRICS PLATE
// =====================================================
// color() has NO $preview gate — required for Bambu 3MF color.
// In Bambu Studio: export from OpenSCAD as .3mf, not .stl.
// STL has no color channel; .3mf preserves color() data.
// =====================================================

module _plate_text_lines(lines, size, height, pad) {
    lh = size * 1.56;
    translate([-plate_width/2 + pad, plate_height/2 - pad - size, 0])
        for (i = [0:len(lines)-1])
            translate([0, -i * lh, 0])
                linear_extrude(height=height, center=false, convexity=10)
                    text(lines[i], size=size, halign="left", valign="top");
}

module metrics_plate(lines) {

    module base_slab() {
        translate([0, 0, plate_thickness/2])
            cube([plate_width, plate_height, plate_thickness], center=true);
    }

    module text_geo() {
        _plate_text_lines(lines, plate_text_size, plate_text_height, pad=3.5);
    }

    if (plate_mode == "engraved") {
        color(plate_body_color)
            difference() {
                base_slab();
                translate([0, 0, plate_thickness - plate_text_height])
                    text_geo();
            }
        color(plate_text_color)
            translate([0, 0, plate_thickness - plate_text_height])
                text_geo();
    } else { // embossed
        color(plate_body_color) base_slab();
        color(plate_text_color)
            translate([0, 0, plate_thickness])
                text_geo();
    }
}


// =====================================================
// MAIN
// =====================================================
robot_len = calculate_robot_length();

head_end_x    = head_length/2;
spine_start_x = head_end_x + (spine_joint_length - 2*overlap_mm);
spine_end_x   = spine_start_x + calculate_spine_length();
tail_start_x  = spine_end_x + (tail_joint_length - 2*overlap_mm);
tail_length   = calculate_tail_length();

// ---- Compute the three validated indices ----
// b = robot_thickness = out-of-plane blade width, shared by all joints
b = robot_thickness;

sp_Sgeo = s_geo(spine_joint_thickness, spine_joint_length, b);
tl_Sgeo = s_geo(tail_joint_thickness,  tail_joint_length,  b);

sp_FR   = flex_ratio(spine_joint_thickness, spine_joint_length);
tl_FR   = flex_ratio(tail_joint_thickness,  tail_joint_length);

sp_AR   = aspect_ratio(spine_joint_thickness, b);
tl_AR   = aspect_ratio(tail_joint_thickness,  b);

// Spine/Tail ratios  (> 1 = tail joint has higher value)
ratio_Sgeo = safe_ratio(tl_Sgeo, sp_Sgeo);
ratio_FR   = safe_ratio(tl_FR,   sp_FR);

// ---- Console echo ----
echo("===========================================");
echo("ROBOT PARAMETERS");
echo("===========================================");
echo(str("Total length:   ", robot_len, " mm"));
echo(str("Head: ",  enable_head  ? "ON" : "OFF",
         "  Tail: ", enable_tail  ? "ON" : "OFF",
         "  Limbs: ", enable_limbs ? "ON" : "OFF"));
echo(str("Tendons: ", enable_tendon_holes ? "ON" : "OFF"));
echo("-------------------------------------------");
echo("FLEXURE METRICS (geometry-only, material-agnostic)");
echo("  Sources: Howell & Midha ASME JMD 1994; Howell 2001; Timoshenko & Gere 1961");
echo(str("  b = robot_thickness = ", b, " mm  (shared out-of-plane width for all joints)"));
echo("  Formulas:  S_geo = b*t^3/L  (geom. stiffness factor, prop. to EI/L)");
echo("             FR    = L/t      (flex ratio, prop. to max safe bend angle)");
echo("             AR    = t/b      (aspect ratio, lateral buckling risk flag)");
echo("  Tl/Sp = Tail value / Spine value  (>1 = tail joint has higher value)");
echo(str("  Spine joint:  t=", spine_joint_thickness, "mm  L=", spine_joint_length, "mm"));
echo(str("    S_geo=", sp_Sgeo, " [", stiff_label(sp_Sgeo), "]",
         "   FR=", sp_FR, " [", flex_label(sp_FR), "]",
         "   AR=", sp_AR, " [", ar_label(sp_AR), "]"));
echo(str("  Tail joint:   t=", tail_joint_thickness,  "mm  L=", tail_joint_length,  "mm"));
echo(str("    S_geo=", tl_Sgeo, " [", stiff_label(tl_Sgeo), "]",
         "   FR=", tl_FR, " [", flex_label(tl_FR), "]",
         "   AR=", tl_AR, " [", ar_label(tl_AR), "]"));
echo(str("  Tl/Sp ratios:  S_geo=", ratio_Sgeo, "  FR=", ratio_FR));
echo("  AR < 0.10 = WARN: lateral buckling risk (Timoshenko LTB criterion)");
echo("  PRBM most accurate when FR > 10; FR 2-6 = validated first-order approx");
echo("===========================================");

do_holes = enable_tendon_holes && !(WORK_MODE && $preview && FAST_HOLES_PREVIEW);

// Y extent for plate placement
body_half_w    = max(head_width, spine_bone_width, tail_bone_width) / 2;
limb_clearance = 2;
limb_extra     = enable_limbs ? (limb_clearance + limb_joint_length + limb_length) : 0;
approx_half_y  = body_half_w + limb_extra;

// ---- Plate text lines ----
// Line 0: title, total length, shared blade width b
// Line 1: spine joint — plain-English parameter names
// Line 2: tail joint  — plain-English parameter names
// Line 3: Stiffness (S_geo = b*t^3/L) — how hard the joint is to bend
// Line 4: Bending Range (FR = L/t)    — how far the joint can travel
// Line 5: Buckling Risk (AR = t/b)    — lateral stability flag
plate_lines = [
    str("FlexVertebra  Length = ", fmt2(robot_len), "mm  Thickness = ", b, "mm"),
    str("Spine joint: thick = ", spine_joint_thickness, "mm",
        "  gap = ",        spine_joint_length,    "mm",
        "  segments = ",   spine_num_bones),
    str("Tail joint:  thick = ", tail_joint_thickness,  "mm",
        "  gap = ",        tail_joint_length,     "mm",
        "  segments = ",   tail_num_bones),
    str("Metrics:"),
    str("Stiffness:  Spine = ", fmt2(sp_Sgeo), " [", stiff_label(sp_Sgeo), "],",
        "  Tail = ",           fmt2(tl_Sgeo), " [", stiff_label(tl_Sgeo), "],",
        "  Ratio=",          fmt2(ratio_Sgeo)),
    str("Bend Range: Spine = ", fmt2(sp_FR),   " [", flex_label(sp_FR),    "],",
        "  Tail = ",           fmt2(tl_FR),   " [", flex_label(tl_FR),    "],",
        "  Ratio=",          fmt2(ratio_FR)),
    str("Buckling:   Spine =", fmt2(sp_AR),   " [", ar_label(sp_AR),      "],",
        "  Tail = ",           fmt2(tl_AR),   " [", ar_label(tl_AR),      "]")
];

// ---- Render ----
translate([-robot_len/2 + head_length/2, 0, 0]) {

    if (do_holes) {
        difference() {
            lizard_robot_solid();
            if (enable_head) head_tendon_holes();
            spine_tendon_holes(spine_start_x, spine_end_x);
            if (enable_tail) tail_tendon_holes(tail_start_x, tail_length);
        }
    } else {
        lizard_robot_solid();
    }

    if (enable_info_plate) {
        plate_x = robot_len/2 - head_length/2;
        plate_y = -(approx_half_y + plate_margin_y + plate_height/2);
        translate([plate_x, plate_y, 0])
            metrics_plate(plate_lines);
    }
}
