import numpy as np
import time, math

from base_classes import Robot, Sim


# ==========================================================
# PATH PLANNER
# ==========================================================

def trailer_path_planner(mode="figure8"):
    """
    Generate a test path for the trailer.

    Modes:
        "figure8"  — Lissajous figure-8, good for testing cornering
        "line"     — Straight horizontal line, good for stability testing
        "vertical" — Straight vertical line
        "circle"   — Constant curvature, good for steady-state testing
        "square"   — Piecewise square loop
        "demo"     — Recording path: wobble up, sharp corner, wobble right
        "arc"      — Smooth S-curve from bottom-left to top-right
    """
    if mode == "figure8":
        t = np.linspace(0, 2 * np.pi, 400)
        x = 1.0 + 0.5 * np.sin(t)
        y = 1.0 + 0.35 * np.sin(2 * t)
        return list(zip(x, y))

    elif mode == "line":
        x = np.linspace(0.3, 1.7, 300)
        y = np.ones_like(x) * 1.0
        return list(zip(x, y))

    elif mode == "vertical":
        y = np.linspace(0.3, 1.7, 300)
        x = np.ones_like(y) * 1.0
        return list(zip(x, y))

    elif mode == "circle":
        t = np.linspace(0, 2 * np.pi, 400)
        x = 1.0 + 0.4 * np.cos(t)
        y = 1.0 + 0.4 * np.sin(t)
        return list(zip(x, y))

    elif mode == "demo":
        """
        Demo recording path — designed for a robot starting at bottom-left
        facing into the corner (theta0 = -3*pi/4, i.e. southwest):

          Phase 1: Wobble upward along the left side of the arena.
                   Sinusoidal lateral wobble in X shows trailer tracking.

          Phase 2: Sharp 90-degree elbow at the top-left. The tight radius
                   is intentionally aggressive to trigger the fixup mechanic.

          Phase 3: Wobble rightward across the top of the arena.
                   Sinusoidal lateral wobble in Y mirrors Phase 1.
        """
        def wobble_segment(p1, p2, n, amp, perp):
            """
            Linearly interpolate p1→p2 with sinusoidal wobble of amplitude
            `amp` added in the perpendicular direction `perp`.
            Uses 2.5 cycles so the segment neither starts nor ends flat,
            giving a natural, organic-looking oscillation.
            """
            pts = []
            for i, t in enumerate(np.linspace(0, 1, n)):
                base = p1 + t * (p2 - p1)
                wave = amp * np.sin(i / n * 2 * np.pi * 2.5)
                pts.append(tuple(base + wave * perp))
            return pts

        path = []

        # Key waypoints
        start      = np.array([0.30, 0.30])   # bottom-left
        top_left   = np.array([0.30, 1.55])   # top of vertical run
        elbow_tip  = np.array([0.38, 1.68])   # apex of the sharp corner
        elbow_exit = np.array([0.55, 1.68])   # exit heading into horizontal run
        end        = np.array([1.70, 1.68])   # bottom-right of horizontal run

        # ── Phase 1: wobble upward ──────────────────────────────────────
        # Perpendicular to upward travel is the X axis.
        path += wobble_segment(start, top_left, 200, 0.06, np.array([1.0, 0.0]))

        # ── Phase 2: sharp elbow at top-left ────────────────────────────
        # Two straight segments meeting at a tight apex — intentionally
        # aggressive so the fixup mechanic has to intervene.
        for t in np.linspace(0, 1, 25):
            path.append(tuple((1 - t) * top_left + t * elbow_tip))
        for t in np.linspace(0, 1, 25):
            path.append(tuple((1 - t) * elbow_tip + t * elbow_exit))

        # ── Phase 3: wobble rightward ────────────────────────────────────
        # Perpendicular to rightward travel is the Y axis.
        path += wobble_segment(elbow_exit, end, 200, 0.06, np.array([0.0, 1.0]))

        return path

    elif mode == "arc":
        """
        Smooth S-curve from bottom-left (0.30, 0.30) to top-right (1.70, 1.70).
        Same start position and theta0 as "demo".

        Built as a cubic Bezier so the entry and exit tangents are controlled:
          - Entry tangent: northeast (matches a robot reversing SW → trailer
            leads NE up the path)
          - Exit tangent: northeast, symmetric

        The two interior control points are offset to create a gentle S-shape
        rather than a straight diagonal, giving the trailer something
        interesting to track without any sharp corners.
        """
        p0 = np.array([0.30, 0.30])   # start — bottom-left
        p1 = np.array([0.30, 1.10])   # pull upward first (S upper lobe)
        p2 = np.array([1.70, 0.90])   # pull rightward (S lower lobe)
        p3 = np.array([1.70, 1.70])   # end — top-right

        n = 400
        path = []
        for t in np.linspace(0, 1, n):
            u = 1 - t
            pt = (u**3 * p0
                  + 3 * u**2 * t * p1
                  + 3 * u * t**2 * p2
                  + t**3 * p3)
            path.append(tuple(pt))
        return path

    elif mode == "square":
        path = []
        corners = [(0.4, 0.4), (1.6, 0.4), (1.6, 1.6), (0.4, 1.6)]
        for i in range(len(corners)):
            p1 = np.array(corners[i])
            p2 = np.array(corners[(i + 1) % len(corners)])
            for t in np.linspace(0, 1, 80):
                path.append(tuple(p1 + t * (p2 - p1)))
        return path

    else:
        raise ValueError(f"Unknown path mode: {mode!r}")


# ==========================================================
# HELPER: LOOKAHEAD POINT
# ==========================================================

def _find_lookahead(path, cx, cy, lookahead, path_index):
    """
    Circle-crossing lookahead: find where the lookahead circle first
    crosses the path, searching forward from path_index.
    Wraps around for looping paths.

    Returns (target_point, new_path_index).
    """
    n = len(path)

    for offset in range(n):
        i = (path_index + offset) % n
        j = (i + 1) % n

        p1  = np.array(path[i])
        p2  = np.array(path[j])
        pos = np.array([cx, cy])

        d1 = np.linalg.norm(p1 - pos)
        d2 = np.linalg.norm(p2 - pos)

        if d1 < lookahead and d2 >= lookahead:
            ratio  = (lookahead - d1) / (d2 - d1 + 1e-9)
            target = tuple(p1 + ratio * (p2 - p1))
            return target, i

    # fallback: just step ahead
    idx = (path_index + 5) % n
    return path[idx], idx


# ==========================================================
# FIXUP HELPERS
# ==========================================================

def _enter_fixup(robot, hitch, lookahead_target, min_dist, max_dist):
    """
    Compute a forward fixup waypoint and enter fixup mode.

    The waypoint is placed opposite the direction toward the lookahead
    target, so driving forward to it naturally reduces hitch angle and
    reorients the system for the next reverse attempt.

    Fixup distance is scaled by hitch severity — larger hitch angles
    use a shorter, more conservative maneuver.
    """
    to_x = lookahead_target[0] - robot.x
    to_y = lookahead_target[1] - robot.y
    dist = np.hypot(to_x, to_y)

    if dist > 1e-6:
        dx = -to_x / dist
        dy = -to_y / dist
    else:
        dx = np.cos(robot.theta)
        dy = np.sin(robot.theta)

    # Larger hitch → shorter fixup (avoid over-committing)
    hitch_ratio = np.clip(abs(hitch) / np.deg2rad(90), 0.0, 1.0)
    fix_dist    = max_dist - hitch_ratio * (max_dist - min_dist)

    robot.recovery_target        = (robot.x + fix_dist * dx,
                                    robot.y + fix_dist * dy)
    robot.debug_recovery_target  = robot.recovery_target
    robot.control_mode           = "fixup"

    # Reset frustration timer so we don't immediately re-enter fixup
    robot.fixup_frustration_timer = 0.0


def _execute_fixup(robot, hitch, max_omega, fixup_speed):
    """
    Execute the fixup maneuver: drive forward toward the fixup waypoint.

    Exits when either:
      1. Close enough to the waypoint (distance-based)
      2. Hitch angle has recovered sufficiently (angle-based)
    """
    tx, ty = robot.recovery_target

    dx   = tx - robot.x
    dy   = ty - robot.y
    dist = np.hypot(dx, dy)

    reached  = dist < 0.04
    hitch_ok = abs(hitch) < np.deg2rad(20)

    if reached or hitch_ok:
        robot.control_mode           = "normal"
        robot.recovery_target        = None
        robot.debug_recovery_target  = None
        robot.fixup_frustration_timer = 0.0
        robot.fixup_post_cooldown    = 1.5   # brief cooldown before re-triggering
        robot.debug_info = {
            "mode": "FIXUP→NORMAL",
            "hitch": hitch, "v": 0.0, "omega": 0.0,
            "local_y": 0.0, "curvature": 0.0,
            "phi_des": 0.0, "phi_err": 0.0,
            "frustration": 0.0,
        }
        return 0.0, 0.0

    desired_heading = np.arctan2(dy, dx)
    heading_error   = desired_heading - robot.theta
    heading_error   = np.arctan2(np.sin(heading_error), np.cos(heading_error))

    v     = fixup_speed
    omega = 3.0 * heading_error

    robot.debug_info = {
        "mode": "FIXUP",
        "hitch": hitch, "v": v, "omega": omega,
        "local_y": dist, "curvature": 0.0,
        "phi_des": 0.0, "phi_err": hitch,
        "frustration": 0.0,
    }
    return v, np.clip(omega, -max_omega, max_omega)


# ==========================================================
# TRAILER CONTROLLER
# ==========================================================

def trailer_controller(path, lookahead, robot, speed, dt=0.05):
    """
    Cascaded trailer controller with timeout-based fixup maneuver system.

    ── State machine ──────────────────────────────────────────────────────
    "normal"   Standard path tracking (forward or reverse)
    "fixup"    Forward repositioning maneuver to escape tight corner

    ── Forward mode ───────────────────────────────────────────────────────
    Trailer-aware pure pursuit. Blends standard curvature tracking with a
    hitch-angle correction that activates as hitch builds up, preventing
    the trailer from folding into a dangerous angle mid-corner.

    ── Reverse mode ───────────────────────────────────────────────────────
    Cascaded hitch-angle controller:
      Outer: path curvature → desired hitch angle  (kinematic inversion)
      Inner: PD on hitch error → robot ω

    ── Fixup trigger (reverse only) ───────────────────────────────────────
    A fixup is triggered only when ALL THREE conditions hold simultaneously:

      1. HEADING ERROR large  — |local_y| > HEADING_ERR_THRESHOLD
             The trailer is significantly off the path laterally.

      2. DISTANCE ERROR small — distance from control point to nearest
             path point < DIST_ERR_THRESHOLD
             The robot IS near the path, so the error is genuinely a
             heading problem, not just a long-range approach.

      3. SATURATED TIMEOUT    — the controller has been outputting max
             omega (saturated) continuously for ≥ FRUSTRATION_TIMEOUT s
             This proves the controller has genuinely tried and failed
             to fix the heading — it's not just a transient.

    A post-fixup cooldown (fixup_post_cooldown) prevents immediate
    re-entry after a fixup completes.

    Hard jackknife (|hitch| > HITCH_JACKKNIFE) triggers immediately,
    bypassing the timeout, because that state is physically dangerous.
    """

    # ==========================
    # PARAMETERS
    # ==========================
    L = robot.trailer_length

    # Forward gains (trailer-aware blend)
    k_fwd_path  = 2.5
    k_fwd_phi   = 2.0
    k_fwd_damp  = 0.8

    # Reverse gains (cascaded hitch controller)
    k_rev_phi   = 4.5
    k_rev_damp  = 1.5

    max_omega   = 3.0   # rad/s — also used as saturation reference

    # Hard jackknife limit — immediate fixup, no timeout
    HITCH_JACKKNIFE = np.deg2rad(65)

    # Timeout-based fixup trigger thresholds
    HEADING_ERR_THRESHOLD = 0.1    # local_y (m) — significant lateral error
    DIST_ERR_THRESHOLD    = 0.60    # m — control point close to path
    OMEGA_SAT_FRACTION    = 0.80    # fraction of max_omega to count as "saturated"
    FRUSTRATION_TIMEOUT   = 0.2     # seconds to persist before fixup fires

    # Post-fixup cooldown (s) before frustration timer can build again
    FIXUP_POST_COOLDOWN   = 1.5

    # Fixup maneuver geometry
    FIXUP_SPEED     = 0.18
    FIXUP_MIN_DIST  = 0.12
    FIXUP_MAX_DIST  = 0.28

    hitch = robot.get_hitch_angle()

    # Decay post-fixup cooldown each call
    if robot.fixup_post_cooldown > 0.0:
        robot.fixup_post_cooldown = max(0.0, robot.fixup_post_cooldown - dt)

    # ==========================
    # FIXUP MODE
    # ==========================
    if robot.control_mode == "fixup":
        return _execute_fixup(robot, hitch, max_omega, FIXUP_SPEED)

    # ==========================
    # CONTROL POINT SELECTION
    # ==========================
    if speed >= 0:
        cx, cy = robot.x, robot.y
        theta  = robot.theta
    else:
        cx, cy = robot.get_trailer_position()
        theta  = robot.get_trailer_heading()

    # ==========================
    # LOOKAHEAD
    # ==========================
    target, robot.path_index = _find_lookahead(
        path, cx, cy, lookahead, robot.path_index
    )
    robot.debug_lookahead = target

    # ==========================
    # LOCAL FRAME
    # ==========================
    dx = target[0] - cx
    dy = target[1] - cy

    local_x =  np.cos(theta) * dx + np.sin(theta) * dy
    local_y = -np.sin(theta) * dx + np.cos(theta) * dy

    # Prevent donut-causing sign flip when target drifts behind control point
    local_x = max(local_x, 1e-3)

    # ==========================
    # PURE PURSUIT CURVATURE  κ = 2y / Ld²
    # ==========================
    kappa = 2.0 * local_y / (lookahead ** 2)

    # Desired hitch angle from kinematic inversion: sin(φ) = L · κ_path
    arg_rev = np.clip(L * kappa * k_rev_phi / k_rev_phi, -0.9, 0.9)  # placeholder, computed properly below
    arg     = np.clip(L * kappa * k_fwd_path, -0.9, 0.9)
    phi_des = np.arcsin(arg)
    phi_err = phi_des - hitch

    v = speed

    # ==========================
    # REVERSE: COMPUTE OMEGA CANDIDATE
    # (needed to check saturation before deciding to trigger fixup)
    # ==========================
    if speed < 0:
        omega_rev = k_rev_phi * phi_err - k_rev_damp * hitch
        omega_rev_clipped = np.clip(omega_rev, -max_omega, max_omega)
        is_saturated = abs(omega_rev) >= OMEGA_SAT_FRACTION * max_omega

        # Distance from control point to nearest path point
        dists_to_path = [np.hypot(px - cx, py - cy) for px, py in path]
        dist_to_path  = min(dists_to_path)

        # Check all three frustration conditions
        heading_err_large = abs(local_y) > HEADING_ERR_THRESHOLD
        dist_err_small    = dist_to_path < DIST_ERR_THRESHOLD
        frustrated        = heading_err_large and dist_err_small and is_saturated

        if frustrated and robot.fixup_post_cooldown <= 0.0:
            robot.fixup_frustration_timer += dt
        else:
            # Any condition not met → reset timer
            robot.fixup_frustration_timer = max(
                0.0, robot.fixup_frustration_timer - dt * 2.0
            )

        # Hard jackknife — immediate, no timeout required
        hard_jackknife = abs(hitch) > HITCH_JACKKNIFE

        if hard_jackknife or (robot.fixup_frustration_timer >= FRUSTRATION_TIMEOUT
                               and robot.fixup_post_cooldown <= 0.0):
            _enter_fixup(robot, hitch, target, FIXUP_MIN_DIST, FIXUP_MAX_DIST)
            robot.debug_info = {
                "mode": "ENTERING FIXUP" + (" (JACKKNIFE)" if hard_jackknife else " (TIMEOUT)"),
                "hitch": hitch, "v": 0.0, "omega": 0.0,
                "local_y": local_y, "curvature": kappa,
                "phi_des": phi_des, "phi_err": phi_err,
                "frustration": robot.fixup_frustration_timer,
            }
            return 0.0, 0.0

        # Normal reverse control
        robot.debug_info = {
            "mode": f"REVERSE (frust {robot.fixup_frustration_timer:.1f}s)",
            "hitch": hitch, "v": v, "omega": omega_rev_clipped,
            "local_y": local_y, "curvature": kappa,
            "phi_des": phi_des, "phi_err": phi_err,
            "frustration": robot.fixup_frustration_timer,
        }
        return v, omega_rev_clipped

    # ==========================
    # FORWARD — trailer-aware blended controller
    # ==========================
    # Pure pursuit drives path tracking; hitch correction blends in as
    # the trailer angle builds up, keeping it from folding in corners.
    omega_pp    = v * kappa * k_fwd_path
    omega_hitch = k_fwd_phi * phi_err - k_fwd_damp * hitch

    blend = np.clip(abs(hitch) / np.deg2rad(30), 0.0, 1.0)
    omega = (1.0 - blend) * omega_pp + blend * omega_hitch

    # Reset frustration when not reversing
    robot.fixup_frustration_timer = 0.0

    robot.debug_info = {
        "mode": "FORWARD",
        "hitch": hitch, "v": v, "omega": omega,
        "local_y": local_y, "curvature": kappa,
        "phi_des": phi_des, "phi_err": phi_err,
        "frustration": 0.0,
    }
    return v, np.clip(omega, -max_omega, max_omega)


# ==========================================================
# MAIN
# ==========================================================

def main():
    path = trailer_path_planner(mode="arc")

    dt    = 0.05
    speed = -0.2   # negative = reverse (trailer leads)

    x0, y0 = path[0]
    x1, y1 = path[1]

    # Align robot to the first path segment.
    # In reverse the robot drives backward (trailer leads), so theta
    # points opposite the path direction — robot faces away from path[1].
    path_heading = math.atan2(y1 - y0, x1 - x0)
    theta0 = path_heading + (math.pi if speed < 0 else 0.0)

    trailer_length = 0.1

    if speed < 0:
        # Control point is the trailer, which sits trailer_length behind
        # the robot center. Offset the robot forward so the trailer starts
        # exactly on path[0], eliminating initial tracking error.
        x0 = x0 + trailer_length * math.cos(theta0)
        y0 = y0 + trailer_length * math.sin(theta0)

    robot = Robot(
        mode="sim",
        x0=x0, y0=y0,
        theta0=theta0,
        has_trailer=True,
        trailer_length=trailer_length,
    )

    sim = Sim([robot], bounds=[[0, 0], [2, 2]], path=path)

    while True:
        robot.update(dt=dt)

        v, w = trailer_controller(path, lookahead=0.15, robot=robot, speed=speed, dt=dt)
        robot.set_velocity(v, w)

        sim.update()
        time.sleep(dt)


if __name__ == "__main__":
    main()