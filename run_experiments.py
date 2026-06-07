"""
run_experiments.py
==================
Sequenced experiment runner for the EGB439 video recording.

Runs four experiments back-to-back, each for a fixed duration.
At the end of each experiment the sim window title and telemetry
banner update automatically so there is a clear on-screen label
for every section of the recording.

Experiment sequence
-------------------
  1. FORWARD TRACKING     — figure-8, speed +0.2
       Shows the trailer passively following the robot with no special
       control needed.  Clean, stable, makes the forward case obvious.

  2. REVERSE — STABLE     — arc path, speed -0.2
       Shows the cascaded hitch-angle controller tracking a smooth
       reversing path.  Hitch angle oscillates in mid-range (~15-35°).
       Demonstrates that controlled reverse is achievable.

  3. REVERSE — UNSTABLE   — arc path, speed -0.2, hitch control OFF
       Same path, same speed, but the inner PD loop is disabled so
       the hitch angle is unconstrained.  The trailer jackknifes
       within a few seconds, confirming why active control is needed.

  4. OBSTACLE FIELD       — A*-planned path, speed -0.2
       Shows the complete system: planned smooth path through an
       obstacle field, tracked in reverse with the hitch controller.
       Fixup maneuvers (described as a recommended extension) may
       or may not trigger depending on path curvature.

Usage
-----
    python run_experiments.py

Press Ctrl-C to skip to the next experiment early.
The sim window stays open between experiments so the trail is
visible when the next one starts.
"""

import numpy as np
import math
import time
import signal
import sys

import matplotlib
matplotlib.use("TkAgg")          # use interactive backend
import matplotlib.pyplot as plt

from base_classes import Robot, Sim
from trailer_controller import trailer_controller, trailer_path_planner

# Path planner only needed for experiment 4
try:
    from path_planner import plan_path, CircleObstacle, DEFAULT_OBSTACLES
    PLANNER_AVAILABLE = True
except ImportError:
    PLANNER_AVAILABLE = False
    print("path_planner.py not found — experiment 4 will be skipped")


# ==========================================================
# EXPERIMENT DEFINITIONS
# ==========================================================

EXPERIMENTS = [
    {
        "title":       "Experiment 1 — Forward Tracking",
        "subtitle":    "figure-8 path | speed +0.20 m/s | hitch control active",
        "path_mode":   "figure8",
        "speed":       +0.20,
        "lookahead":   0.25,
        "duration":    25,        # seconds
        "limit_curv":  True,
        "obstacles":   None,
    },
    {
        "title":       "Experiment 2 — Controlled Reverse",
        "subtitle":    "arc path | speed -0.20 m/s | cascaded hitch controller",
        "path_mode":   "arc",
        "speed":       -0.20,
        "lookahead":   0.20,
        "duration":    35,
        "limit_curv":  True,
        "obstacles":   None,
    },
    {
        "title":       "Experiment 3 — Reverse Without Hitch Control (UNSTABLE)",
        "subtitle":    "arc path | speed -0.20 m/s | hitch damping = 0 (naive reverse)",
        "path_mode":   "arc",
        "speed":       -0.20,
        "lookahead":   0.20,
        "duration":    15,        # short — it jackknifes quickly
        "limit_curv":  False,     # remove curvature cap so instability is immediate
        "no_hitch_ctrl": True,    # special flag — zeroes inner PD gains
        "obstacles":   None,
    },
    {
        "title":       "Experiment 4 — Obstacle Field Planning",
        "subtitle":    "A* planned path | speed -0.20 m/s | obstacle field",
        "path_mode":   "obstacle",
        "speed":       -0.20,
        "lookahead":   0.20,
        "duration":    60,
        "limit_curv":  True,
        "obstacles":   "default",
    },
]


# ==========================================================
# HELPERS
# ==========================================================

DT             = 0.05
TRAILER_LENGTH = 0.10
BOUNDS         = [[0, 0], [2, 2]]


def _make_robot(path, speed):
    """Spawn robot with trailer at the correct pose for the given path and speed."""
    x0, y0 = path[0]
    x1, y1 = path[1]

    path_heading = math.atan2(y1 - y0, x1 - x0)
    theta0       = path_heading + (math.pi if speed < 0 else 0.0)

    if speed < 0:
        # Offset robot so trailer starts exactly on path[0]
        x0 += TRAILER_LENGTH * math.cos(theta0)
        y0 += TRAILER_LENGTH * math.sin(theta0)

    return Robot(
        mode="sim",
        x0=x0, y0=y0, theta0=theta0,
        has_trailer=True,
        trailer_length=TRAILER_LENGTH,
    )


def _make_path(exp):
    """Build path for an experiment, including A* for the obstacle experiment."""
    if exp["path_mode"] == "obstacle":
        if not PLANNER_AVAILABLE:
            return None, None
        obstacles = [CircleObstacle(cx, cy, r) for cx, cy, r in DEFAULT_OBSTACLES]
        print("  Planning path through obstacle field …")
        path, obstacles = plan_path(
            start=(0.25, 0.25),
            goal=(1.75, 1.75),
            obstacles=obstacles,
        )
        print(f"  Path found: {len(path)} points")
        return path, obstacles
    else:
        return trailer_path_planner(mode=exp["path_mode"]), None


_skip_requested = False

def _signal_handler(sig, frame):
    global _skip_requested
    _skip_requested = True

signal.signal(signal.SIGINT, _signal_handler)


# ==========================================================
# EXPERIMENT RUNNER
# ==========================================================

def run_experiment(exp, sim, path, obstacles):
    """
    Run a single experiment on an existing Sim window.
    Rebuilds the robot and replaces the sim's robot list and path display.
    Returns when duration elapses or Ctrl-C is pressed.
    """
    global _skip_requested
    _skip_requested = False

    speed = exp["speed"]
    robot = _make_robot(path, speed)

    # Patch the sim with the new robot and path
    sim.robots = [robot]

    # Clear all artists and redraw path + obstacles
    sim.ax.cla()
    sim.ax.set_xlim(BOUNDS[0][0], BOUNDS[1][0])
    sim.ax.set_ylim(BOUNDS[0][1], BOUNDS[1][1])
    sim.ax.set_aspect("equal")
    sim.ax.set_title(exp["title"], fontsize=10, fontweight="bold")

    # Obstacles
    if obstacles:
        MARGIN = 0.12
        for obs in obstacles:
            sim.ax.add_patch(plt.Circle((obs.cx, obs.cy), obs.r,
                                        color="#555555", alpha=0.75, zorder=2))
            sim.ax.add_patch(plt.Circle((obs.cx, obs.cy), obs.r + MARGIN,
                                        color="#888888", alpha=0.25,
                                        fill=False, linestyle="--", lw=1, zorder=2))

    # Path
    px = [p[0] for p in path]
    py = [p[1] for p in path]
    sim.ax.plot(px, py, "k-", lw=2, label="Path", zorder=3)

    # Rebuild patches
    sim._rebuild_artists(robot)

    # Subtitle in telemetry pane
    sim.ax_info.set_title(exp["subtitle"], fontsize=7, wrap=True)

    t_start = time.time()
    elapsed = 0.0

    # Override gains for experiment 3 (no hitch control)
    no_hitch = exp.get("no_hitch_ctrl", False)

    while elapsed < exp["duration"] and not _skip_requested:
        robot.update(dt=DT)

        v, w = trailer_controller(
            path,
            lookahead=exp["lookahead"],
            robot=robot,
            speed=speed,
            dt=DT,
            limit_curvature=exp["limit_curv"],
        )

        # Experiment 3: zero the angular output so there is no hitch correction
        # (the controller still computes phi_des for telemetry, but w is forced to
        # pure curvature pursuit with no damping — this rapidly destabilises)
        if no_hitch:
            # Pure naive reverse: just set omega proportional to local_y directly,
            # bypassing the hitch PD completely
            if robot.debug_info:
                kappa  = robot.debug_info.get("curvature", 0.0)
                w      = np.clip(speed * kappa * 2.0, -3.0, 3.0)
                robot.debug_info["mode"] = "REVERSE (NO HITCH CTRL)"

        robot.set_velocity(v, w)
        sim.update()
        time.sleep(DT)

        elapsed = time.time() - t_start

    if _skip_requested:
        print("  Skipped by user")


# ==========================================================
# MAIN
# ==========================================================

def main():
    print("EGB439 — Experiment sequence starting\n")
    print("Press Ctrl-C to skip to the next experiment.\n")

    # Create one Sim window that persists across all experiments
    # Start with a dummy robot just to initialise the figure
    dummy_path = trailer_path_planner("line")
    dummy_robot = _make_robot(dummy_path, speed=-0.2)

    sim = Sim([dummy_robot], bounds=BOUNDS, path=dummy_path)

    # Monkey-patch a method to rebuild artists when robot changes
    def _rebuild_artists(self, robot):
        import matplotlib.patches as patches
        from matplotlib.transforms import Affine2D

        self.robot_patches   = []
        self.trailer_patches = []
        self.robot_path_lines   = []
        self.trailer_path_lines = []
        self.lookahead_markers  = []
        self.recovery_markers   = []
        self.recovery_lines     = []

        rect = patches.Rectangle((-0.06, -0.04), 0.12, 0.08,
                                  edgecolor="blue", facecolor="none", lw=2, zorder=5)
        self.ax.add_patch(rect)
        self.robot_patches.append(rect)

        L = robot.trailer_length
        tri = np.array([[0.0, 0.0], [-L, 0.04], [-L, -0.04]])
        poly = patches.Polygon(tri, closed=True,
                                edgecolor="red", facecolor="none", lw=2, zorder=5)
        self.ax.add_patch(poly)
        self.trailer_patches.append(poly)

        rline, = self.ax.plot([], [], "--", color="blue",  alpha=0.4, lw=1, zorder=4)
        tline, = self.ax.plot([], [], "--", color="red",   alpha=0.4, lw=1, zorder=4)
        self.robot_path_lines.append(rline)
        self.trailer_path_lines.append(tline)

        la, = self.ax.plot([], [], "go", ms=7, zorder=6, label="Lookahead")
        self.lookahead_markers.append(la)

        rm, = self.ax.plot([], [], "rx", ms=10, zorder=6, label="Fixup target")
        rl, = self.ax.plot([], [], "r-", lw=1.5, alpha=0.7, zorder=5)
        self.recovery_markers.append(rm)
        self.recovery_lines.append(rl)

        self.ax.legend(loc="upper right", fontsize=7)

    import types
    sim._rebuild_artists = types.MethodType(_rebuild_artists, sim)

    # Run each experiment
    for idx, exp in enumerate(EXPERIMENTS):
        print(f"\n{'='*60}")
        print(f"  {exp['title']}")
        print(f"  {exp['subtitle']}")
        print(f"  Duration: {exp['duration']} s")
        print(f"{'='*60}")

        if exp["path_mode"] == "obstacle" and not PLANNER_AVAILABLE:
            print("  Skipping — path_planner.py not found")
            continue

        path, obstacles = _make_path(exp)
        if path is None:
            print("  Skipping — path planning failed")
            continue

        # Countdown
        for i in range(3, 0, -1):
            print(f"  Starting in {i} …")
            time.sleep(1.0)

        run_experiment(exp, sim, path, obstacles)
        print(f"  Experiment {idx+1} complete")

        # Pause between experiments
        if idx < len(EXPERIMENTS) - 1:
            print("  Pausing 3 s before next experiment …")
            time.sleep(3.0)

    print("\nAll experiments complete.")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()