"""
path_planner.py
===============
Grid-based A* path planner through a circular obstacle field,
with path shortcutting and cubic spline smoothing.

Usage
-----
    from path_planner import plan_path, DEFAULT_OBSTACLES, CircleObstacle

    path, obstacles = plan_path(start=(0.25, 0.25), goal=(1.75, 1.75))

    # pass obstacles to Sim for visualisation:
    sim = Sim([robot], bounds=[[0,0],[2,2]], path=path, obstacles=obstacles)
"""

import numpy as np
import heapq
from scipy import interpolate


# ==========================================================
# DEFAULT OBSTACLE LAYOUT
# ==========================================================
# Each entry: (centre_x, centre_y, radius)  [metres]
# Placed to create a natural slalom corridor from bottom-left
# to top-right in a 2 × 2 m arena.

DEFAULT_OBSTACLES = [
    (0.70, 0.60, 0.09),   # obs 0 — lower-left cluster
    (1.40, 0.55, 0.09),   # obs 1 — lower-right cluster
    (0.40, 1.05, 0.09),   # obs 2 — mid-left
    (1.00, 1.00, 0.10),   # obs 3 — centre
    (1.60, 1.05, 0.09),   # obs 4 — mid-right
    (0.65, 1.50, 0.09),   # obs 5 — upper-left cluster
    (1.35, 1.50, 0.09),   # obs 6 — upper-right cluster
]


# ==========================================================
# OBSTACLE CLASS
# ==========================================================

class CircleObstacle:
    """Circular obstacle with collision helpers."""

    def __init__(self, cx, cy, r):
        self.cx = cx
        self.cy = cy
        self.r  = r

    def point_collides(self, x, y, margin=0.0):
        return np.hypot(x - self.cx, y - self.cy) < self.r + margin

    def segment_min_dist(self, p1, p2):
        """Minimum distance from obstacle centre to line segment p1→p2."""
        p1 = np.array(p1, dtype=float)
        p2 = np.array(p2, dtype=float)
        d  = p2 - p1
        dlen2 = np.dot(d, d)
        if dlen2 < 1e-12:
            return np.hypot(p1[0] - self.cx, p1[1] - self.cy)
        t  = np.clip(np.dot(np.array([self.cx, self.cy]) - p1, d) / dlen2, 0.0, 1.0)
        cl = p1 + t * d
        return np.hypot(cl[0] - self.cx, cl[1] - self.cy)

    def segment_collides(self, p1, p2, margin=0.0):
        return self.segment_min_dist(p1, p2) < self.r + margin


# ==========================================================
# OCCUPANCY GRID
# ==========================================================

def _build_grid(obstacles, bounds, resolution, margin):
    """
    Build a 2-D boolean occupancy grid.
    True = occupied (obstacle inflated by margin, or too close to boundary).
    """
    x_min, y_min = bounds[0]
    x_max, y_max = bounds[1]

    nx = int((x_max - x_min) / resolution) + 1
    ny = int((y_max - y_min) / resolution) + 1

    grid = np.zeros((nx, ny), dtype=bool)

    for ix in range(nx):
        for iy in range(ny):
            x = x_min + ix * resolution
            y = y_min + iy * resolution

            if (x < x_min + margin or x > x_max - margin or
                    y < y_min + margin or y > y_max - margin):
                grid[ix, iy] = True
                continue

            for obs in obstacles:
                if obs.point_collides(x, y, margin):
                    grid[ix, iy] = True
                    break

    return grid, (x_min, y_min)


def _world_to_grid(x, y, origin, res):
    return (int(round((x - origin[0]) / res)),
            int(round((y - origin[1]) / res)))


def _grid_to_world(ix, iy, origin, res):
    return (origin[0] + ix * res, origin[1] + iy * res)


# ==========================================================
# A* SEARCH
# ==========================================================

def _astar(grid, start, goal):
    """
    8-connected A* on a boolean grid.
    Returns list of (ix, iy) from start to goal, or None if no path.
    """
    nx, ny = grid.shape

    def h(a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1])

    open_set = []
    heapq.heappush(open_set, (h(start, goal), 0.0, start))

    came_from = {}
    g_score   = {start: 0.0}

    moves = [(-1,-1),(-1, 0),(-1, 1),
             ( 0,-1),         ( 0, 1),
             ( 1,-1),( 1, 0),( 1, 1)]

    while open_set:
        _, g, cur = heapq.heappop(open_set)

        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            path.reverse()
            return path

        if g > g_score.get(cur, float('inf')) + 1e-9:
            continue

        for dx, dy in moves:
            nb = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nb[0] < nx and 0 <= nb[1] < ny):
                continue
            if grid[nb]:
                continue

            ng = g_score[cur] + np.hypot(dx, dy)
            if ng < g_score.get(nb, float('inf')):
                came_from[nb] = cur
                g_score[nb]   = ng
                heapq.heappush(open_set, (ng + h(nb, goal), ng, nb))

    return None


# ==========================================================
# PATH SHORTCUTTING
# ==========================================================

def _shortcut(waypoints, obstacles, margin):
    """
    Greedily remove redundant waypoints.

    Uses margin * 1.5 for the shortcut test so that the spline fitted
    through the remaining waypoints cannot bulge back through obstacles
    (spline curves outward between waypoints by a small amount).
    """
    if len(waypoints) <= 2:
        return waypoints

    check_margin = margin * 1.5

    i = 0
    result = [waypoints[0]]

    while i < len(waypoints) - 1:
        j = len(waypoints) - 1
        while j > i + 1:
            p1 = np.array(waypoints[i])
            p2 = np.array(waypoints[j])
            if all(not obs.segment_collides(p1, p2, check_margin)
                   for obs in obstacles):
                break
            j -= 1
        result.append(waypoints[j])
        i = j

    return result


# ==========================================================
# SPLINE SMOOTHING
# ==========================================================

def _smooth_spline(waypoints, obstacles, margin, n_points=400):
    """
    Fit a parametric cubic spline through waypoints.
    Falls back to densified piecewise-linear if the spline hits an obstacle.
    """
    pts = np.array(waypoints, dtype=float)

    if len(pts) < 4:
        return _densify(waypoints, n_points)

    # Parameterise by cumulative arc length
    diffs    = np.diff(pts, axis=0)
    seg_lens = np.linalg.norm(diffs, axis=1)
    t        = np.concatenate([[0.0], np.cumsum(seg_lens)])
    t        = t / t[-1]

    try:
        tck, _ = interpolate.splprep([pts[:, 0], pts[:, 1]], u=t, s=0, k=3)
        t_fine = np.linspace(0.0, 1.0, n_points)
        xs, ys = interpolate.splev(t_fine, tck)
    except Exception:
        return _densify(waypoints, n_points)

    smooth = list(zip(xs.tolist(), ys.tolist()))

    # Collision check — if spline clips anything, fall back to densified segments
    for i in range(len(smooth) - 1):
        p1 = np.array(smooth[i])
        p2 = np.array(smooth[i + 1])
        for obs in obstacles:
            if obs.segment_collides(p1, p2, margin):
                return _densify(waypoints, n_points)

    return smooth


def _densify(waypoints, n_points):
    """Evenly densify a piecewise-linear path to approximately n_points total."""
    pts      = np.array(waypoints, dtype=float)
    diffs    = np.diff(pts, axis=0)
    seg_lens = np.linalg.norm(diffs, axis=1)
    total    = max(seg_lens.sum(), 1e-9)

    path = []
    for i in range(len(pts) - 1):
        n_seg = max(2, int(round(n_points * seg_lens[i] / total)))
        for k in range(n_seg):
            t = k / n_seg
            path.append(tuple((1.0 - t) * pts[i] + t * pts[i + 1]))
    path.append(tuple(pts[-1]))
    return path


# ==========================================================
# PUBLIC API
# ==========================================================

def plan_path(start, goal, obstacles=None, bounds=None,
              robot_margin=0.12, resolution=0.025, n_smooth=400):
    """
    Plan a smooth, collision-free path from *start* to *goal*.

    Parameters
    ----------
    start, goal   : (x, y)
    obstacles     : list[CircleObstacle] or None → use DEFAULT_OBSTACLES
    bounds        : [[xmin, ymin], [xmax, ymax]], default [[0,0],[2,2]]
    robot_margin  : robot footprint radius for collision checking (m)
    resolution    : A* grid cell size (m)
    n_smooth      : number of points in the returned smooth path

    Returns
    -------
    path      : list of (x, y) — smooth, densely sampled, collision-free
    obstacles : list[CircleObstacle] that was used (for Sim visualisation)
    """
    if bounds is None:
        bounds = [[0.0, 0.0], [2.0, 2.0]]

    if obstacles is None:
        obstacles = [CircleObstacle(cx, cy, r) for cx, cy, r in DEFAULT_OBSTACLES]

    # Build occupancy grid
    grid, origin = _build_grid(obstacles, bounds, resolution, robot_margin)
    nx, ny = grid.shape

    # Convert start / goal to grid indices
    si = _world_to_grid(start[0], start[1], origin, resolution)
    gi = _world_to_grid(goal[0],  goal[1],  origin, resolution)

    si = (max(0, min(nx-1, si[0])), max(0, min(ny-1, si[1])))
    gi = (max(0, min(nx-1, gi[0])), max(0, min(ny-1, gi[1])))

    if grid[si]:
        raise ValueError(f"Start {start} is inside an obstacle / margin")
    if grid[gi]:
        raise ValueError(f"Goal {goal} is inside an obstacle / margin")

    # A* search
    grid_path = _astar(grid, si, gi)
    if grid_path is None:
        raise RuntimeError("A* could not find a path — obstacle layout may be too dense")

    # Back to world coordinates
    waypoints = [_grid_to_world(ix, iy, origin, resolution)
                 for ix, iy in grid_path]

    # Shortcut → remove redundant waypoints
    waypoints = _shortcut(waypoints, obstacles, robot_margin)

    # Smooth with spline (falls back to densified linear if spline clips)
    path = _smooth_spline(waypoints, obstacles, robot_margin, n_points=n_smooth)

    return path, obstacles