import numpy as np
import time
from base_classes import Robot, Sim

import numpy as np



def generate_test_path(mode="figure8"):
    """
    Path generator for testing different behaviours.

    mode:
        - "figure8"
        - "line"
        - "circle"
    """

    if mode == "figure8":
        t = np.linspace(0, 2*np.pi, 400)
        x = 1.0 + 0.5 * np.sin(t)
        y = 1.0 + 0.35 * np.sin(2*t)
        return list(zip(x, y))

    elif mode == "line":
        # Straight horizontal line (perfect for reverse testing)
        x = np.linspace(0.3, 1.7, 200)
        y = np.ones_like(x) * 1.0
        return list(zip(x, y))

    elif mode == "vertical":
        # Straight vertical line
        y = np.linspace(0.3, 1.7, 200)
        x = np.ones_like(y) * 1.0
        return list(zip(x, y))

    elif mode == "circle":
        t = np.linspace(0, 2*np.pi, 300)
        x = 1.0 + 0.4 * np.cos(t)
        y = 1.0 + 0.4 * np.sin(t)
        return list(zip(x, y))

    else:
        raise ValueError("Unknown path mode")



def trailer_controller(path, lookahead, robot, speed):

    MAX_FIX = 0.25
    MIN_FIX = 0.12

    # =====================
    # RECOVERY MODE
    # =====================
    if robot.control_mode == "recovery":

        if robot.recovery_target is None:
            robot.control_mode = "normal"
            return 0.0, 0.0

        v_rec, w_rec = robot.handle_recovery()

        dx = robot.recovery_target[0] - robot.x
        dy = robot.recovery_target[1] - robot.y
        dist = np.hypot(dx, dy)

        if dist < 0.035:
            robot.control_mode = "normal"
            robot.recovery_origin = (robot.x, robot.y)
            robot.recovery_target = None
            robot.debug_recovery_target = None

        return v_rec, w_rec

    # =====================
    # DISTANCE COOLDOWN
    # =====================
    cooldown_active = False
    if robot.recovery_origin is not None:
        moved = np.hypot(robot.x - robot.recovery_origin[0],
                         robot.y - robot.recovery_origin[1])
        if moved < robot.recovery_distance_cooldown:
            cooldown_active = True
        else:
            robot.recovery_origin = None

    # =====================
    # CONTROL POINT
    # =====================
    (cx, cy), ctheta = robot.get_control_point(speed)

    # =====================
    # NEAREST POINT
    # =====================
    dists = np.array([np.hypot(px - cx, py - cy) for px, py in path])

    search_start = max(0, robot.path_index - 10)
    candidates = np.argsort(dists[search_start:])[:15] + search_start

    idx = candidates[np.argmin(dists[candidates])]
    robot.path_index = idx

    # =====================
    # LOOKAHEAD
    # =====================
    target = None
    for j in range(idx, len(path)):
        if np.hypot(path[j][0] - cx, path[j][1] - cy) > lookahead:
            target = path[j]
            break

    if target is None:
        target = path[0]
        robot.path_index = 0

    robot.debug_lookahead = target

    dx = target[0] - cx
    dy = target[1] - cy

    desired_heading = np.arctan2(dy, dx)

    heading_error = desired_heading - ctheta
    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

    # 🔥 FIX: prevent 180° flip behaviour
    MAX_TURN = np.deg2rad(90)
    if heading_error > MAX_TURN:
        heading_error -= np.pi
    elif heading_error < -MAX_TURN:
        heading_error += np.pi

    # 🔥 clamp for reversing stability
    if speed < 0:
        heading_error = np.clip(heading_error, -np.deg2rad(60), np.deg2rad(60))

    # =====================
    # RECOVERY
    # =====================
    if speed < 0 and robot.is_jackknifed() and not cooldown_active:

        dx = target[0] - cx
        dy = target[1] - cy

        dx *= -1
        dy *= -1

        dist = np.hypot(dx, dy)
        if dist > 1e-6:
            dx /= dist
            dy /= dist

        fix_dist = np.clip(dist, MIN_FIX, MAX_FIX)

        robot.recovery_target = (
            cx + fix_dist * dx,
            cy + fix_dist * dy
        )

        robot.debug_recovery_target = robot.recovery_target
        robot.control_mode = "recovery"

        return 0.0, 0.0

    # =====================
    # CONTROL
    # =====================
    v = speed

    if v >= 0:
        omega = 2.2 * heading_error
        return v, omega

    # reverse
    k_heading = 2.2
    k_phi = 2.0
    k_damp = 0.5

    kappa = k_heading * heading_error
    omega_t_des = v * kappa

    arg = (robot.trailer_length * omega_t_des) / v
    arg = np.clip(arg, -0.8, 0.8)

    phi_des = np.arcsin(arg)
    phi = robot.get_hitch_angle()

    omega = k_phi * (phi_des - phi) - k_damp * phi
    omega = np.clip(omega, -2.5, 2.5)

    return v, omega



def trailer_controller_curvature(path, lookahead, robot, speed):

    (cx, cy), ctheta = robot.get_control_point(speed)

    # =====================
    # NEAREST POINT
    # =====================
    dists = np.array([np.hypot(px - cx, py - cy) for px, py in path])
    idx = np.argmin(dists)
    robot.path_index = idx

    # =====================
    # LOOKAHEAD
    # =====================
    for j in range(idx, len(path)):
        if np.hypot(path[j][0] - cx, path[j][1] - cy) > lookahead:
            target = path[j]
            break
    else:
        target = path[0]

    robot.debug_lookahead = target

    dx = target[0] - cx
    dy = target[1] - cy

    # transform into local frame
    local_x =  np.cos(ctheta) * dx + np.sin(ctheta) * dy
    local_y = -np.sin(ctheta) * dx + np.cos(ctheta) * dy

    # =====================
    # CURVATURE
    # =====================
    if abs(local_x) < 1e-6:
        return 0.0, 0.0

    curvature = 2 * local_y / (lookahead ** 2)

    v = speed
    omega = v * curvature

    # =====================
    # TRAILER STABILISATION (reverse only)
    # =====================
    if v < 0:
        phi = robot.get_hitch_angle()
        omega -= 1.2 * phi   # damping

    omega = np.clip(omega, -2.5, 2.5)

    return v, omega



def trailer_controller_trailer_aware(path, lookahead, robot, speed):

    v = speed
    if abs(v) < 1e-5:
        return 0.0, 0.0

    # =====================
    # CONTROL POINT (IMPORTANT)
    # =====================
    (cx, cy), ctheta = robot.get_control_point(v)

    # =====================
    # FIND NEAREST POINT
    # =====================
    dists = np.array([np.hypot(px - cx, py - cy) for px, py in path])
    idx = np.argmin(dists)
    robot.path_index = idx

    # =====================
    # LOOKAHEAD
    # =====================
    target = None

    for j in range(idx, len(path)):
        dx = path[j][0] - cx
        dy = path[j][1] - cy

        local_x =  np.cos(ctheta)*dx + np.sin(ctheta)*dy

        if local_x > 0:  # 🔥 MUST be in front
            if np.hypot(dx, dy) > lookahead:
                target = path[j]
                break

    # fallback
    if target is None:
        target = path[(idx + 20) % len(path)]

    robot.debug_lookahead = target

    # =====================
    # LOCAL FRAME (CRITICAL)
    # =====================
    dx = target[0] - cx
    dy = target[1] - cy

    local_x =  np.cos(ctheta) * dx + np.sin(ctheta) * dy
    local_y = -np.sin(ctheta) * dx + np.cos(ctheta) * dy

    # =====================
    # TRAILER CURVATURE
    # =====================
    curvature = 2 * local_y / (lookahead ** 2)

    # desired trailer angular velocity
    omega_trailer_des = v * curvature

    # =====================
    # CONVERT → HITCH ANGLE
    # =====================
    L = robot.trailer_length

    arg = (L * omega_trailer_des) / v
    arg = np.clip(arg, -0.9, 0.9)   # avoid invalid arcsin

    phi_des = np.arcsin(arg)

    # =====================
    # CONTROL LAW
    # =====================
    phi = robot.get_hitch_angle()

    # 🔥 tuned gains (important)
    k_phi = 3.0
    k_damp = 0.8

    omega = k_phi * (phi_des - phi) - k_damp * phi

    # =====================
    # SAFETY
    # =====================
    omega = np.clip(omega, -2.5, 2.5)

    return v, omega



def trailer_controller_clean(path, lookahead, robot, speed):

    v = speed
    if abs(v) < 1e-5:
        return 0.0, 0.0

    # --- ALWAYS use trailer ---
    cx, cy = robot.get_trailer_position()
    theta = robot.get_trailer_heading()

    # --- nearest ---
    dists = np.array([np.hypot(px - cx, py - cy) for px, py in path])
    idx = np.argmin(dists)

    # --- lookahead ---
    target = None
    for j in range(idx, len(path)):
        if np.hypot(path[j][0]-cx, path[j][1]-cy) > lookahead:
            target = path[j]
            break
    if target is None:
        target = path[0]

    # --- local frame ---
    dx = target[0] - cx
    dy = target[1] - cy

    local_x =  np.cos(theta)*dx + np.sin(theta)*dy
    local_y = -np.sin(theta)*dx + np.cos(theta)*dy

    # --- curvature ---
    curvature = 2 * local_y / (lookahead**2)
    omega_trailer = v * curvature

    # --- convert to hitch ---
    L = robot.trailer_length
    arg = (L * omega_trailer) / v
    arg = np.clip(arg, -0.9, 0.9)

    phi_des = np.arcsin(arg)

    # --- hitch control ---
    phi = robot.get_hitch_angle()

    k_phi = 2.5
    k_damp = 0.8

    omega = k_phi * (phi_des - phi) - k_damp * phi

    return v, np.clip(omega, -2.5, 2.5)



def main():
    robot = Robot(mode="sim", x0=1.0, y0=1.0, has_trailer=True)

    path = generate_test_path("figure8")
    sim = Sim([robot], [[0, 0], [2, 2]], path=path)

    while True:
        robot.update()

        #v, w = trailer_controller(path, lookahead=0.26, robot=robot, speed=-0.22)
        #v, w = trailer_controller_curvature(path, lookahead=0.26, robot=robot, speed=-0.22)
        #v, w = trailer_controller_trailer_aware(path, lookahead=0.26, robot=robot, speed=-0.1)
        v, w = trailer_controller_clean(path, lookahead=0.26, robot=robot, speed=0.22)
        robot.set_velocity(v, w)

        sim.update()
        time.sleep(0.05)


if __name__ == "__main__":
    main()