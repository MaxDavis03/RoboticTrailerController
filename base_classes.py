import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D


class Robot:
    def __init__(
        self,
        mode="sim",
        x0=0.0, y0=0.0, theta0=0.0,
        wheel_base=0.15,
        has_trailer=False,
        trailer_length=0.1,
    ):
        self.mode = mode

        # geometry
        self.wheel_base = wheel_base
        self.trailer_length = trailer_length
        self.has_trailer = has_trailer

        # 🔥 hitch is NOT at axle
        self.hitch_offset = 0.06

        # state
        self.x = x0
        self.y = y0
        self.theta = theta0

        self.v = 0.0
        self.omega = 0.0

        if has_trailer:
            self.trailer_theta = theta0

        # commands
        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        # control
        self.control_mode = "normal"
        self.recovery_target = None

        # cooldown
        self.recovery_origin = None
        self.recovery_distance_cooldown = 0.2

        # tracking
        self.path_index = 0

        # debug
        self.debug_lookahead = None
        self.debug_recovery_target = None

        # history
        self.history = []
        self.trailer_history = []

    # =====================
    # BASIC
    # =====================

    def set_velocity(self, v, omega):
        self.v_cmd = v
        self.omega_cmd = omega

    def update(self, dt=0.05):
        self._update_sim(dt)

        self.history.append((self.x, self.y))

        if self.has_trailer:
            tx, ty = self.get_trailer_position()
            self.trailer_history.append((tx, ty))

    def get_pose(self):
        return self.x, self.y, self.theta

    # =====================
    # HITCH + TRAILER
    # =====================

    def get_hitch_position(self):
        hx = self.x - self.hitch_offset * np.cos(self.theta)
        hy = self.y - self.hitch_offset * np.sin(self.theta)
        return hx, hy

    def get_trailer_position(self):
        hx, hy = self.get_hitch_position()

        tx = hx - self.trailer_length * np.cos(self.trailer_theta)
        ty = hy - self.trailer_length * np.sin(self.trailer_theta)

        return tx, ty

    def get_trailer_heading(self):
        return self.trailer_theta

    def get_hitch_angle(self):
        angle = self.theta - self.trailer_theta
        return np.arctan2(np.sin(angle), np.cos(angle))  # 🔥 wrapped

    def get_control_point(self, speed):
        if speed >= 0:
            return (self.x, self.y), self.theta
        else:
            return self.get_trailer_position(), self.trailer_theta

    def is_jackknifed(self):
        return abs(self.get_hitch_angle()) > np.deg2rad(85)

    # =====================
    # SIMULATION
    # =====================

    def _update_sim(self, dt):
        self.v = self.v_cmd
        self.omega = self.omega_cmd

        # --- robot motion ---
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt

        self.theta += self.omega * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # --- trailer motion (🔥 FIXED PROPERLY) ---
        if self.has_trailer:

            # 1. hitch position
            hx = self.x - self.hitch_offset * np.cos(self.theta)
            hy = self.y - self.hitch_offset * np.sin(self.theta)

            # 2. hitch velocity (KEY FIX)
            vx = self.v * np.cos(self.theta)
            vy = self.v * np.sin(self.theta)

            # rotational velocity component due to offset
            vx += -self.omega * self.hitch_offset * np.sin(self.theta)
            vy +=  self.omega * self.hitch_offset * np.cos(self.theta)

            # 3. project velocity onto trailer axis
            t_hat = np.array([
                np.cos(self.trailer_theta),
                np.sin(self.trailer_theta)
            ])

            v_trailer = vx * t_hat[0] + vy * t_hat[1]

            # 4. trailer angular velocity
            delta = self.theta - self.trailer_theta
            theta_dot = (v_trailer / self.trailer_length) * np.sin(delta)

            self.trailer_theta += theta_dot * dt
            self.trailer_theta = np.arctan2(np.sin(self.trailer_theta), np.cos(self.trailer_theta))

    # =====================
    # RECOVERY
    # =====================

    def handle_recovery(self):
        if self.recovery_target is None:
            return 0.0, 0.0

        dx = self.recovery_target[0] - self.x
        dy = self.recovery_target[1] - self.y

        target_heading = np.arctan2(dy, dx)

        heading_error = target_heading - self.theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        v = 0.1
        omega = 2.5 * heading_error

        return v, omega


# =====================
# SIMULATOR
# =====================

class Sim:
    def __init__(self, robots, bounds, path=None):
        self.robots = robots
        self.path = path

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(bounds[0][0], bounds[1][0])
        self.ax.set_ylim(bounds[0][1], bounds[1][1])
        self.ax.set_aspect('equal')

        # draw path
        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            self.ax.plot(px, py, 'k-', linewidth=2)

        self.robot_patches = []
        self.trailer_patches = []
        self.paths = []
        self.trailer_paths = []

        self.lookahead_markers = []
        self.recovery_markers = []
        self.recovery_lines = []

        for robot in robots:
            # robot
            rect = patches.Rectangle(
                (-0.06, -0.04), 0.12, 0.08,
                edgecolor='blue', facecolor='none', lw=2
            )
            self.ax.add_patch(rect)
            self.robot_patches.append(rect)

            # trailer
            if robot.has_trailer:
                L = robot.trailer_length
                W = 0.08

                tri = np.array([
                    [0.0, 0.0],       # hitch
                    [-L, W/2],
                    [-L, -W/2],
                ])

                poly = patches.Polygon(
                    tri, closed=True,
                    edgecolor='red', facecolor='none', lw=2
                )
                self.ax.add_patch(poly)
            else:
                poly = None

            self.trailer_patches.append(poly)

            # paths
            path_line, = self.ax.plot([], [], '--', color='blue', alpha=0.5)
            self.paths.append(path_line)

            t_path_line, = self.ax.plot([], [], '--', color='red', alpha=0.5)
            self.trailer_paths.append(t_path_line)

            # debug markers
            la, = self.ax.plot([], [], 'go', markersize=6)
            self.lookahead_markers.append(la)

            rm, = self.ax.plot([], [], 'ro', markersize=6)
            self.recovery_markers.append(rm)

            rl, = self.ax.plot([], [], 'r--', linewidth=1)
            self.recovery_lines.append(rl)

        plt.ion()
        plt.show()

    def _transform(self, patch, x, y, theta):
        t = Affine2D().rotate(theta).translate(x, y) + self.ax.transData
        patch.set_transform(t)

    def update(self):
        for i, robot in enumerate(self.robots):
            x, y, theta = robot.get_pose()

            # robot
            self._transform(self.robot_patches[i], x, y, theta)

            # trailer (correctly attached at hitch)
            if robot.has_trailer:
                hx, hy = robot.get_hitch_position()
                tt = robot.get_trailer_heading()
                self._transform(self.trailer_patches[i], hx, hy, tt)

            # history
            if robot.history:
                hx, hy = zip(*robot.history)
                self.paths[i].set_data(hx, hy)

            if robot.has_trailer and robot.trailer_history:
                txh, tyh = zip(*robot.trailer_history)
                self.trailer_paths[i].set_data(txh, tyh)

            # lookahead
            if robot.debug_lookahead is not None:
                lx, ly = robot.debug_lookahead
                self.lookahead_markers[i].set_data([lx], [ly])
            else:
                self.lookahead_markers[i].set_data([], [])

            # recovery
            if robot.control_mode == "recovery" and robot.debug_recovery_target is not None:
                rx, ry = robot.debug_recovery_target
                self.recovery_markers[i].set_data([rx], [ry])
                self.recovery_lines[i].set_data([robot.x, rx], [robot.y, ry])
            else:
                self.recovery_markers[i].set_data([], [])
                self.recovery_lines[i].set_data([], [])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()