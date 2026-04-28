import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D


class Robot:
    def __init__(
        self,
        mode="sim",
        pibot=None,
        group_num=None,

        x0=0.0, y0=0.0, theta0=0.0,

        wheel_base=0.15,
        hitch_offset=0.06,      # distance from robot axle to hitch point (rear of robot)

        has_trailer=False,
        trailer_length=0.1,
    ):
        self.mode = mode

        # Physical parameters
        self.wheel_base = wheel_base
        self.hitch_offset = hitch_offset
        self.has_trailer = has_trailer
        self.trailer_length = trailer_length

        # Robot state (axle center)
        self.x = x0
        self.y = y0
        self.theta = theta0

        # Trailer state (axle)
        if has_trailer:
            self.trailer_theta = theta0

        # Commanded velocities
        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        # Actual velocities
        self.v = 0.0
        self.omega = 0.0

        # Control state machine
        self.control_mode = "normal"   # "normal" | "fixup"
        self.recovery_target = None

        # Path tracking
        self.path_index = 0

        # Fixup frustration tracker
        # Accumulates time spent saturated at max omega while heading error
        # remains large and distance error is small — triggers fixup when
        # the controller has genuinely tried and failed for long enough.
        self.fixup_frustration_timer = 0.0   # seconds spent in frustrated state
        self.fixup_post_cooldown     = 0.0   # seconds since last fixup exit (prevents immediate re-entry)

        # Debug
        self.debug_info = {}
        self.debug_lookahead = None
        self.debug_recovery_target = None

        # History
        self.history = []
        self.trailer_history = []

        # Real robot
        if mode == "real":
            if pibot is None:
                raise ValueError("Real mode requires a PiBot instance")
            self.bot = pibot
            self.group_num = group_num

    # ==========================================================
    # BASIC INTERFACE
    # ==========================================================

    def set_velocity(self, v, omega):
        self.v_cmd = v
        self.omega_cmd = omega

    def update(self, dt=0.05):
        if self.mode == "real":
            self._update_real()
        else:
            self._update_sim(dt)

        self.history.append((self.x, self.y))

        if self.has_trailer:
            tx, ty = self.get_trailer_position()
            self.trailer_history.append((tx, ty))

    def get_pose(self):
        return self.x, self.y, self.theta

    # ==========================================================
    # GEOMETRY
    # ==========================================================

    def get_hitch_position(self):
        """Hitch point — offset behind robot axle center."""
        hx = self.x - self.hitch_offset * np.cos(self.theta)
        hy = self.y - self.hitch_offset * np.sin(self.theta)
        return hx, hy

    def get_trailer_position(self):
        """Trailer axle position, computed from hitch."""
        hx, hy = self.get_hitch_position()
        tx = hx - self.trailer_length * np.cos(self.trailer_theta)
        ty = hy - self.trailer_length * np.sin(self.trailer_theta)
        return tx, ty

    def get_trailer_heading(self):
        return self.trailer_theta

    def get_hitch_angle(self):
        """Signed hitch angle, wrapped to [-pi, pi]."""
        angle = self.theta - self.trailer_theta
        return np.arctan2(np.sin(angle), np.cos(angle))

    def is_jackknifed(self, threshold_deg=70.0):
        return abs(self.get_hitch_angle()) > np.deg2rad(threshold_deg)

    # ==========================================================
    # SIMULATION
    # ==========================================================

    def _update_sim(self, dt):
        self.v = self.v_cmd
        self.omega = self.omega_cmd

        # Robot kinematics
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # Trailer kinematics — must use hitch velocity, not axle velocity
        if self.has_trailer:
            # Velocity at hitch point (includes rotational component due to offset)
            vx = self.v * np.cos(self.theta) - self.omega * self.hitch_offset * np.sin(self.theta)
            vy = self.v * np.sin(self.theta) + self.omega * self.hitch_offset * np.cos(self.theta)

            # Project hitch velocity onto trailer heading
            t_hat_x = np.cos(self.trailer_theta)
            t_hat_y = np.sin(self.trailer_theta)
            v_trailer = vx * t_hat_x + vy * t_hat_y

            # Trailer angular velocity
            delta = self.theta - self.trailer_theta
            theta_dot = (v_trailer / self.trailer_length) * np.sin(delta)
            self.trailer_theta += theta_dot * dt
            self.trailer_theta = np.arctan2(np.sin(self.trailer_theta), np.cos(self.trailer_theta))

    # ==========================================================
    # REAL ROBOT
    # ==========================================================

    def _update_real(self):
        left, right = self._vw_to_skid(self.v_cmd, self.omega_cmd)
        self.bot.setVelocity(left, right)

        pose = self.bot.getLocalizerPose(self.group_num)
        if pose:
            self.x, self.y, self.theta = pose

        self.v = self.v_cmd
        self.omega = self.omega_cmd

    def _vw_to_skid(self, v, omega):
        scale = 100
        v_l = v - (self.wheel_base / 2.0) * omega
        v_r = v + (self.wheel_base / 2.0) * omega
        return int(scale * v_l), int(scale * v_r)


# ==========================================================
# SIMULATOR / VISUALISER
# ==========================================================

class Sim:
    def __init__(self, robots, bounds, path=None):
        self.robots = robots
        self.path = path

        self.fig, (self.ax, self.ax_info) = plt.subplots(
            1, 2, gridspec_kw={'width_ratios': [3, 1]}, figsize=(10, 6)
        )

        self.ax.set_xlim(bounds[0][0], bounds[1][0])
        self.ax.set_ylim(bounds[0][1], bounds[1][1])
        self.ax.set_aspect('equal')
        self.ax.set_title("Arena")

        self.ax_info.axis('off')
        self.ax_info.set_title("Telemetry")
        self.info_text = self.ax_info.text(
            0.05, 0.95, "", transform=self.ax_info.transAxes,
            fontsize=9, verticalalignment='top', fontfamily='monospace'
        )

        # Draw reference path
        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            self.ax.plot(px, py, 'k-', lw=2, label="Path", zorder=1)

        self.robot_patches = []
        self.trailer_patches = []
        self.robot_path_lines = []
        self.trailer_path_lines = []
        self.lookahead_markers = []
        self.recovery_markers = []
        self.recovery_lines = []

        colors = ['blue', 'green', 'purple', 'orange']

        for idx, robot in enumerate(robots):
            col = colors[idx % len(colors)]

            # Robot rectangle (centered at axle)
            rect = patches.Rectangle(
                (-self.ax.get_xlim()[1] * 0.03, -self.ax.get_xlim()[1] * 0.02),
                self.ax.get_xlim()[1] * 0.06,
                self.ax.get_xlim()[1] * 0.04,
                edgecolor=col, facecolor='none', lw=2, zorder=3
            )
            # Fixed size rectangles
            rect = patches.Rectangle(
                (-0.06, -0.04), 0.12, 0.08,
                edgecolor=col, facecolor='none', lw=2, zorder=3
            )
            self.ax.add_patch(rect)
            self.robot_patches.append(rect)

            # Trailer triangle (tip = hitch, base = axle)
            if robot.has_trailer:
                L = robot.trailer_length
                W = 0.08
                tri = np.array([
                    [0.0,   0.0],   # hitch (tip)
                    [-L,  W/2],     # left axle
                    [-L, -W/2],     # right axle
                ])
                poly = patches.Polygon(
                    tri, closed=True,
                    edgecolor='red', facecolor='none', lw=2, zorder=3
                )
                self.ax.add_patch(poly)
            else:
                poly = None
            self.trailer_patches.append(poly)

            # History lines
            rline, = self.ax.plot([], [], '--', color=col, alpha=0.4, lw=1, zorder=2)
            self.robot_path_lines.append(rline)

            tline, = self.ax.plot([], [], '--', color='red', alpha=0.4, lw=1, zorder=2)
            self.trailer_path_lines.append(tline)

            # Lookahead marker
            la, = self.ax.plot([], [], 'go', markersize=7, zorder=4, label="Lookahead")
            self.lookahead_markers.append(la)

            # Recovery marker and line
            rm, = self.ax.plot([], [], 'rx', markersize=10, zorder=4, label="Recovery target")
            self.recovery_markers.append(rm)

            rl, = self.ax.plot([], [], 'r-', lw=1.5, alpha=0.7, zorder=3)
            self.recovery_lines.append(rl)

        self.ax.legend(loc='upper right', fontsize=8)
        plt.tight_layout()
        plt.ion()
        plt.show()

    def _transform(self, patch, x, y, theta):
        t = Affine2D().rotate(theta).translate(x, y) + self.ax.transData
        patch.set_transform(t)

    def update(self):
        for i, robot in enumerate(self.robots):
            x, y, theta = robot.get_pose()

            # Robot body
            self._transform(self.robot_patches[i], x, y, theta)

            # Trailer — placed at hitch point
            if robot.has_trailer:
                hx, hy = robot.get_hitch_position()
                tt = robot.get_trailer_heading()
                self._transform(self.trailer_patches[i], hx, hy, tt)

            # Robot history
            if robot.history:
                hxs, hys = zip(*robot.history)
                self.robot_path_lines[i].set_data(hxs, hys)

            # Trailer history
            if robot.has_trailer and robot.trailer_history:
                txs, tys = zip(*robot.trailer_history)
                self.trailer_path_lines[i].set_data(txs, tys)

            # Lookahead marker
            if robot.debug_lookahead is not None:
                lx, ly = robot.debug_lookahead
                self.lookahead_markers[i].set_data([lx], [ly])
            else:
                self.lookahead_markers[i].set_data([], [])

            # Fixup marker + line (shown during fixup mode)
            if robot.control_mode == "fixup" and robot.recovery_target is not None:
                rx, ry = robot.recovery_target
                self.recovery_markers[i].set_data([rx], [ry])
                self.recovery_lines[i].set_data([robot.x, rx], [robot.y, ry])
            else:
                self.recovery_markers[i].set_data([], [])
                self.recovery_lines[i].set_data([], [])

            # Telemetry panel (first robot only)
            if i == 0 and robot.debug_info:
                info = robot.debug_info
                hitch_deg    = np.rad2deg(info.get('hitch', 0.0))
                phi_des_deg  = np.rad2deg(info.get('phi_des', 0.0))
                phi_err_deg  = np.rad2deg(info.get('phi_err', 0.0))
                frustration  = info.get('frustration', 0.0)
                frust_pct    = np.clip(frustration / 1.0, 0.0, 1.0)  # fraction of 1 s timeout
                frust_bar    = "█" * int(frust_pct * 10) + "░" * (10 - int(frust_pct * 10))

                text = (
                    f"Mode:     {info.get('mode', '?')}\n"
                    f"\n"
                    f"Hitch:    {hitch_deg:+.1f}°\n"
                    f"\n"
                    f"v:        {info.get('v', 0.0):+.3f} m/s\n"
                    f"ω:        {info.get('omega', 0.0):+.3f} rad/s\n"
                    f"\n"
                    f"--- Controller Errors ---\n"
                    f"local_y:  {info.get('local_y', 0.0):+.4f} m\n"
                    f"Curvature:{info.get('curvature', 0.0):+.4f}\n"
                    f"φ_des:    {phi_des_deg:+.1f}°\n"
                    f"φ_err:    {phi_err_deg:+.1f}°\n"
                    f"\n"
                    f"--- Fixup Trigger ---\n"
                    f"Frust:    {frustration:.2f}s\n"
                    f"          [{frust_bar}]\n"
                )
                self.info_text.set_text(text)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()