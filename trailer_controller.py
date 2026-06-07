"""
trailer_controller.py
=====================
Path planner, lookahead helper, and cascaded trailer controller.
No main() — import this from run_experiments.py or any other script.

Public API
----------
    trailer_path_planner(mode)          -> list of (x, y)
    trailer_controller(path, lookahead,
                       robot, speed,
                       dt, limit_curvature)  -> (v, omega)
"""

import numpy as np
import math


# ==========================================================
# PATH PLANNER
# ==========================================================

def trailer_path_planner(mode="figure8"):
    """
    Generate a test path for the trailer.

    Modes
    -----
    "figure8"  — Lissajous figure-8
    "line"     — Straight horizontal line
    "vertical" — Straight vertical line
    "circle"   — Constant-curvature loop
    "square"   — Piecewise square loop
    "triangle" — Sharp triangle with no corner smoothing (forces fixup x3/lap)
    "arc"      — Smooth cubic-Bezier S-curve, bottom-left → top-right
    "demo"     — Slalom climb + tight loop-de-loop
    "hairpin"  — Straight approach + tangent-continuous CW loop
    """

    if mode == "figure8":
        t = np.linspace(0, 2 * np.pi, 400)
        return list(zip(1.0 + 0.5 * np.sin(t),
                        1.0 + 0.35 * np.sin(2 * t)))

    elif mode == "line":
        x = np.linspace(0.3, 1.7, 300)
        return list(zip(x, np.ones_like(x) * 1.0))

    elif mode == "vertical":
        y = np.linspace(0.3, 1.7, 300)
        return list(zip(np.ones_like(y) * 1.0, y))

    elif mode == "circle":
        t = np.linspace(0, 2 * np.pi, 400)
        return list(zip(1.0 + 0.4 * np.cos(t),
                        1.0 + 0.4 * np.sin(t)))

    elif mode == "triangle":
        """
        Sharp equilateral-ish triangle — no corner smoothing.
        The hard heading discontinuity at each vertex forces the
        controller to saturate, demonstrating the fixup trigger.
        Vertices: A=(0.35,0.35) B=(1.65,0.35) C=(1.00,1.65)
        """
        A = np.array([0.35, 0.35])
        B = np.array([1.65, 0.35])
        C = np.array([1.00, 1.65])
        path = []
        for p1, p2 in [(A, B), (B, C), (C, A)]:
            for k in range(150):
                t = k / 150
                path.append(tuple((1 - t) * p1 + t * p2))
        return path

    elif mode == "arc":
        """
        Cubic Bezier S-curve from (0.30, 0.30) to (1.70, 1.70).
        Smooth curvature — good for stable-tracking demonstration.
        """
        p0 = np.array([0.30, 0.30])
        p1 = np.array([0.30, 1.10])
        p2 = np.array([1.70, 0.90])
        p3 = np.array([1.70, 1.70])
        path = []
        for t in np.linspace(0, 1, 400):
            u = 1 - t
            pt = (u**3*p0 + 3*u**2*t*p1 + 3*u*t**2*p2 + t**3*p3)
            path.append(tuple(pt))
        return path

    elif mode == "demo":
        """
        Phase 1: broad slalom upward (hitch angle visibly active mid-range).
        Phase 2: tight loop-de-loop (forces fixup).
        Phase 3: straight exit rightward.
        """
        path = []
        for t in np.linspace(0, 1, 250):
            path.append((0.55 + 0.22 * np.sin(t * 2 * np.pi * 1.5),
                         0.25 + t * 1.10))

        loop_cx, loop_cy, loop_r = 0.55, 1.47, 0.10
        for t in np.linspace(np.pi/2, np.pi/2 - 2*np.pi, 100):
            path.append((loop_cx + loop_r*np.cos(t),
                         loop_cy + loop_r*np.sin(t)))

        loop_exit = np.array([loop_cx, loop_cy + loop_r])
        for t in np.linspace(0, 1, 150):
            path.append(tuple((1-t)*loop_exit + t*np.array([1.70, loop_cy+loop_r])))
        return path

    elif mode == "hairpin":
        """
        Straight diagonal approach, then a tangent-continuous CW loop
        (r=0.12) — the required hitch angle exceeds 65° almost immediately,
        so the jackknife fixup triggers within the first quarter-turn.
        """
        path = []
        start      = np.array([0.30, 0.30])
        loop_entry = np.array([0.85, 1.10])
        loop_r     = 0.12

        ah  = np.arctan2(loop_entry[1]-start[1], loop_entry[0]-start[0])
        rh  = ah - np.pi/2
        lcx = loop_entry[0] + loop_r * np.cos(rh)
        lcy = loop_entry[1] + loop_r * np.sin(rh)
        ea  = np.arctan2(loop_entry[1]-lcy, loop_entry[0]-lcx)

        for t in np.linspace(0, 1, 220):
            path.append(tuple((1-t)*start + t*loop_entry))
        for t in np.linspace(ea, ea - 2*np.pi, 140):
            path.append((lcx + loop_r*np.cos(t), lcy + loop_r*np.sin(t)))

        exit_pt = np.array([1.20, 1.10])
        end     = np.array([1.70, 1.10])
        for t in np.linspace(0, 1, 120):
            path.append(tuple((1-t)*loop_entry + t*exit_pt))
        for t in np.linspace(0, 1, 100):
            path.append(tuple((1-t)*exit_pt + t*end))
        return path

    elif mode == "square":
        path = []
        corners = [(0.4,0.4),(1.6,0.4),(1.6,1.6),(0.4,1.6)]
        for i in range(len(corners)):
            p1 = np.array(corners[i])
            p2 = np.array(corners[(i+1) % len(corners)])
            for k in range(80):
                path.append(tuple(p1 + (k/80)*(p2-p1)))
        return path

    else:
        raise ValueError(f"Unknown path mode: {mode!r}")


# ==========================================================
# LOOKAHEAD
# ==========================================================

def _find_lookahead(path, cx, cy, lookahead, path_index):
    """
    Index-ahead lookahead: find the closest path point within a
    forward search window, then step N_AHEAD indices forward.

    Uses index stepping rather than circle-crossing interpolation so
    that sharp corners cause an immediate heading-target change —
    important for the frustration trigger to fire at corners.
    """
    N_AHEAD = 12
    n       = len(path)
    pos     = np.array([cx, cy])

    best_idx  = path_index
    best_dist = np.linalg.norm(np.array(path[path_index]) - pos)

    for offset in range(1, min(n, 60)):
        i = (path_index + offset) % n
        d = np.linalg.norm(np.array(path[i]) - pos)
        if d < best_dist:
            best_dist = d
            best_idx  = i
        elif d > best_dist + 0.10:
            break

    target_idx = (best_idx + N_AHEAD) % n
    return path[target_idx], best_idx


# ==========================================================
# FIXUP HELPERS
# ==========================================================

def _enter_fixup(robot, hitch, lookahead_target, min_dist, max_dist):
    """
    Compute a forward fixup waypoint and enter fixup mode.

    The waypoint is placed in the direction opposite the lookahead target
    so that driving forward naturally unwinds the hitch angle and
    reorients the system for the next reverse attempt.
    Fixup distance shrinks for larger hitch angles to avoid over-committing.
    """
    to_x = lookahead_target[0] - robot.x
    to_y = lookahead_target[1] - robot.y
    dist = np.hypot(to_x, to_y)

    if dist > 1e-6:
        dx, dy = -to_x/dist, -to_y/dist
    else:
        dx, dy = np.cos(robot.theta), np.sin(robot.theta)

    hitch_ratio = np.clip(abs(hitch) / np.deg2rad(90), 0.0, 1.0)
    fix_dist    = max_dist - hitch_ratio * (max_dist - min_dist)

    robot.recovery_target       = (robot.x + fix_dist*dx,
                                   robot.y + fix_dist*dy)
    robot.debug_recovery_target = robot.recovery_target
    robot.control_mode          = "fixup"
    robot.fixup_frustration_timer = 0.0


def _execute_fixup(robot, hitch, max_omega, fixup_speed):
    """
    Drive forward toward the fixup waypoint.
    Exits when close enough OR when hitch angle has recovered.
    """
    tx, ty = robot.recovery_target
    dx, dy = tx - robot.x, ty - robot.y
    dist   = np.hypot(dx, dy)

    if dist < 0.04 or abs(hitch) < np.deg2rad(20):
        robot.control_mode          = "normal"
        robot.recovery_target       = None
        robot.debug_recovery_target = None
        robot.fixup_frustration_timer = 0.0
        robot.fixup_post_cooldown   = 1.5
        robot.debug_info = {
            "mode": "FIXUP→NORMAL",
            "hitch": hitch, "v": 0.0, "omega": 0.0,
            "local_y": 0.0, "curvature": 0.0,
            "phi_des": 0.0, "phi_err": 0.0,
            "frustration": 0.0,
            "limit_curvature": robot._limit_curvature,
        }
        return 0.0, 0.0

    err   = np.arctan2(np.sin(np.arctan2(dy, dx) - robot.theta),
                       np.cos(np.arctan2(dy, dx) - robot.theta))
    omega = 3.0 * err

    robot.debug_info = {
        "mode": "FIXUP",
        "hitch": hitch, "v": fixup_speed, "omega": omega,
        "local_y": dist, "curvature": 0.0,
        "phi_des": 0.0, "phi_err": hitch,
        "frustration": 0.0,
        "limit_curvature": robot._limit_curvature,
    }
    return fixup_speed, np.clip(omega, -max_omega, max_omega)


# ==========================================================
# TRAILER CONTROLLER
# ==========================================================

def trailer_controller(path, lookahead, robot, speed, dt=0.05,
                       limit_curvature=True):
    """
    Cascaded trailer controller.

    Parameters
    ----------
    path            : list of (x, y) waypoints
    lookahead       : pure-pursuit lookahead radius (m)
    robot           : Robot instance
    speed           : signed speed (m/s) — negative = reverse
    dt              : timestep (s)
    limit_curvature : if True, caps arcsin argument at ±0.9 (~64° hitch,
                      effective r_min ≈ 0.11 m).  If False, only a hard
                      NaN guard (±0.999) is applied.

    Forward mode
    ------------
    Trailer-aware blended pure pursuit.  Pure pursuit dominates when the
    hitch angle is small; a hitch-correction term blends in as it grows,
    preventing the trailer folding into a dangerous angle mid-corner.

    Reverse mode
    ------------
    Cascaded hitch-angle controller:
      Outer loop: path curvature κ → desired hitch angle φ_des
                  via kinematic inversion  sin(φ) = L · κ
      Inner loop: PD on hitch error → robot ω

    Fixup trigger (reverse only, described as a recommended extension)
    ------------------------------------------------------------------
    Fires when ALL THREE conditions hold simultaneously for ≥ FRUSTRATION_TIMEOUT s:
      1. |local_y| > HEADING_ERR_THRESHOLD   (large lateral error)
      2. dist_to_path < DIST_ERR_THRESHOLD    (near the path — not just approaching)
      3. |ω_raw| ≥ OMEGA_SAT_FRACTION × max_ω (controller saturated)
    Hard jackknife (|hitch| > 65°) fires immediately without the timeout.
    """

    robot._limit_curvature = limit_curvature

    # ── Gains ──────────────────────────────────────────────
    L           = robot.trailer_length
    k_fwd_path  = 1.8
    k_fwd_phi   = 1.5
    k_fwd_damp  = 1.2
    k_rev_phi   = 2.5
    k_rev_damp  = 2.5
    max_omega   = 3.0

    # ── Fixup parameters ───────────────────────────────────
    HITCH_JACKKNIFE       = np.deg2rad(65)
    HEADING_ERR_THRESHOLD = 0.15
    DIST_ERR_THRESHOLD    = 0.60
    OMEGA_SAT_FRACTION    = 0.80
    FRUSTRATION_TIMEOUT   = 0.2
    FIXUP_SPEED           = 0.18
    FIXUP_MIN_DIST        = 0.12
    FIXUP_MAX_DIST        = 0.28

    arcsin_clip = 0.9 if limit_curvature else 0.999

    hitch = robot.get_hitch_angle()

    if robot.fixup_post_cooldown > 0.0:
        robot.fixup_post_cooldown = max(0.0, robot.fixup_post_cooldown - dt)

    # ── Fixup execution ────────────────────────────────────
    if robot.control_mode == "fixup":
        return _execute_fixup(robot, hitch, max_omega, FIXUP_SPEED)

    # ── Control point ──────────────────────────────────────
    if speed >= 0:
        cx, cy, theta = robot.x, robot.y, robot.theta
    else:
        (cx, cy), theta = robot.get_trailer_position(), robot.get_trailer_heading()

    # ── Lookahead ──────────────────────────────────────────
    target, robot.path_index = _find_lookahead(
        path, cx, cy, lookahead, robot.path_index
    )
    robot.debug_lookahead = target

    # ── Local frame ────────────────────────────────────────
    dx, dy  = target[0] - cx, target[1] - cy
    local_x =  np.cos(theta)*dx + np.sin(theta)*dy
    local_y = -np.sin(theta)*dx + np.cos(theta)*dy
    local_x = max(local_x, 1e-3)   # prevent donut-causing sign flip

    kappa = 2.0 * local_y / (lookahead ** 2)
    v     = speed

    # ── Reverse ────────────────────────────────────────────
    if speed < 0:
        # Pure kinematic inversion — gain belongs in omega step, not arcsin
        arg_rev = np.clip(L * kappa, -arcsin_clip, arcsin_clip)
        phi_des = np.arcsin(arg_rev)
        phi_err = phi_des - hitch

        omega_rev         = k_rev_phi * phi_err - k_rev_damp * hitch
        omega_rev_clipped = np.clip(omega_rev, -max_omega, max_omega)
        is_saturated      = abs(omega_rev) >= OMEGA_SAT_FRACTION * max_omega

        # Local window dist check (O(1) vs O(n))
        window       = range(max(0, robot.path_index - 20),
                             min(len(path), robot.path_index + 20))
        dist_to_path = min(np.hypot(path[j][0]-cx, path[j][1]-cy) for j in window)

        frustrated = (abs(local_y) > HEADING_ERR_THRESHOLD and
                      dist_to_path  < DIST_ERR_THRESHOLD    and
                      is_saturated)

        if frustrated and robot.fixup_post_cooldown <= 0.0:
            robot.fixup_frustration_timer += dt
        else:
            robot.fixup_frustration_timer = max(
                0.0, robot.fixup_frustration_timer - dt * 2.0
            )

        hard_jackknife = abs(hitch) > HITCH_JACKKNIFE
        timed_out      = (robot.fixup_frustration_timer >= FRUSTRATION_TIMEOUT
                          and robot.fixup_post_cooldown <= 0.0)

        if hard_jackknife or timed_out:
            _enter_fixup(robot, hitch, target, FIXUP_MIN_DIST, FIXUP_MAX_DIST)
            robot.debug_info = {
                "mode": "→FIXUP" + (" (JACKKNIFE)" if hard_jackknife else " (TIMEOUT)"),
                "hitch": hitch, "v": 0.0, "omega": 0.0,
                "local_y": local_y, "curvature": kappa,
                "phi_des": phi_des, "phi_err": phi_err,
                "frustration": robot.fixup_frustration_timer,
                "limit_curvature": limit_curvature,
            }
            return 0.0, 0.0

        robot.debug_info = {
            "mode": f"REVERSE  frust={robot.fixup_frustration_timer:.1f}s",
            "hitch": hitch, "v": v, "omega": omega_rev_clipped,
            "local_y": local_y, "curvature": kappa,
            "phi_des": phi_des, "phi_err": phi_err,
            "frustration": robot.fixup_frustration_timer,
            "limit_curvature": limit_curvature,
        }
        return v, omega_rev_clipped

    # ── Forward ────────────────────────────────────────────
    arg_fwd = np.clip(L * kappa * k_fwd_path, -arcsin_clip, arcsin_clip)
    phi_des = np.arcsin(arg_fwd)
    phi_err = phi_des - hitch

    omega_pp    = v * kappa * k_fwd_path
    omega_hitch = k_fwd_phi * phi_err - k_fwd_damp * hitch
    blend       = np.clip(abs(hitch) / np.deg2rad(30), 0.0, 1.0)
    omega       = (1.0 - blend) * omega_pp + blend * omega_hitch

    robot.fixup_frustration_timer = 0.0

    robot.debug_info = {
        "mode": "FORWARD",
        "hitch": hitch, "v": v, "omega": omega,
        "local_y": local_y, "curvature": kappa,
        "phi_des": phi_des, "phi_err": phi_err,
        "frustration": 0.0,
        "limit_curvature": limit_curvature,
    }
    return v, np.clip(omega, -max_omega, max_omega)