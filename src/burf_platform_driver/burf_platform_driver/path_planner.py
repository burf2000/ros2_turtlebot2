"""
Grid path planner — homegrown A* on an OccupancyGrid with obstacle inflation.

Sits between the blind GoToController (drives straight at the goal, hits walls)
and full Nav2 (heavyweight, hard to install). For a known mostly-static
environment — house, office, lab — this is enough to plan around walls and
furniture, at a fraction of Nav2's CPU and setup cost.

The algorithm Simon described independently:
  1. Treat the SLAM map as a grid of robot-sized cells.
  2. Mark cells with anything in them as blocked (occupied).
  3. Inflate the blocked region by the robot's radius so the planner can
     treat the robot as a point.
  4. A* from the robot's current cell to the goal cell.
  5. Simplify the cell path into long straight segments via line-of-sight.
  6. Drive each segment with the existing GoToController.

Pure Python + numpy + (optionally) scipy.ndimage. No ROS imports.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    from scipy.ndimage import binary_dilation
    _HAVE_SCIPY = True
except ImportError:
    _HAVE_SCIPY = False


@dataclass
class GridInfo:
    """Geometry of the OccupancyGrid in world (ROS) coords.

    The driver caches this alongside the int8 numpy array straight from the
    nav_msgs/OccupancyGrid message — *before* the image-y-down flip applied
    for browser rendering. Planning works in ROS +y-up convention end to end.
    """
    width: int            # cells across
    height: int           # cells tall
    resolution: float     # metres per cell
    origin_x: float       # world-frame x of bottom-left cell (cell [0, 0])
    origin_y: float       # world-frame y of bottom-left cell (cell [0, 0])


class GridPathPlanner:
    """A* planner on an inflated OccupancyGrid."""

    OCCUPIED_THRESHOLD = 50    # cells with cost >=50 are obstacles
    UNKNOWN_AS_BLOCKED = True  # treat unknown cells (-1) as blocked too — safer

    def __init__(self, robot_radius_m: float = 0.18,
                 inflation_padding_m: float = 0.05):
        """robot_radius_m: half the robot's longest dimension.
           inflation_padding_m: extra buffer beyond robot_radius. Bumps the
           planner away from hugging walls."""
        self.robot_radius_m = float(robot_radius_m)
        self.inflation_padding_m = float(inflation_padding_m)

    # ─── World <-> grid coordinate conversion ────────────────────────────

    def world_to_cell(self, x: float, y: float, info: GridInfo) -> tuple[int, int]:
        cx = int((x - info.origin_x) / info.resolution)
        cy = int((y - info.origin_y) / info.resolution)
        return cx, cy

    def cell_to_world(self, cx: int, cy: int, info: GridInfo) -> tuple[float, float]:
        x = info.origin_x + (cx + 0.5) * info.resolution
        y = info.origin_y + (cy + 0.5) * info.resolution
        return x, y

    # ─── Inflation ────────────────────────────────────────────────────────

    def _inflate(self, occ: np.ndarray, info: GridInfo) -> np.ndarray:
        """Return a boolean grid where every cell within robot_radius +
        inflation_padding of an obstacle is True (= blocked)."""
        cells_radius = max(
            1, int(math.ceil((self.robot_radius_m + self.inflation_padding_m)
                             / info.resolution))
        )
        if _HAVE_SCIPY:
            return binary_dilation(occ, iterations=cells_radius)
        # Pure-numpy fallback: stack shifts of the occupancy mask in every
        # direction within the radius. Slower than scipy but no extra dep.
        h, w = occ.shape
        inflated = occ.copy()
        for dy in range(-cells_radius, cells_radius + 1):
            for dx in range(-cells_radius, cells_radius + 1):
                if dx * dx + dy * dy > cells_radius * cells_radius:
                    continue
                # shift occ by (dy, dx) and OR into inflated
                y0_src = max(0, -dy);  y1_src = h - max(0, dy)
                x0_src = max(0, -dx);  x1_src = w - max(0, dx)
                y0_dst = max(0, dy);   y1_dst = h - max(0, -dy)
                x0_dst = max(0, dx);   x1_dst = w - max(0, -dx)
                inflated[y0_dst:y1_dst, x0_dst:x1_dst] |= occ[y0_src:y1_src, x0_src:x1_src]
        return inflated

    def build_blocked(self, grid: np.ndarray, info: GridInfo) -> np.ndarray:
        """Take the raw int8 OccupancyGrid (-1 unknown, 0 free, 100 occupied)
        and produce a boolean grid where True = "robot cannot stand here"."""
        occ = grid >= self.OCCUPIED_THRESHOLD
        if self.UNKNOWN_AS_BLOCKED:
            occ = occ | (grid < 0)
        return self._inflate(occ, info)

    # ─── A* ───────────────────────────────────────────────────────────────

    @staticmethod
    def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        # Octile distance — admissible for 8-connected grid with diagonal cost
        dx = abs(a[0] - b[0]); dy = abs(a[1] - b[1])
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    _NEIGHBORS = [
        (-1,  0, 1.0), (1,  0, 1.0), (0, -1, 1.0), (0,  1, 1.0),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        ( 1, -1, math.sqrt(2)), ( 1, 1, math.sqrt(2)),
    ]

    def _astar(self, blocked: np.ndarray,
               start: tuple[int, int],
               goal: tuple[int, int]) -> Optional[list[tuple[int, int]]]:
        h, w = blocked.shape
        sx, sy = start
        gx, gy = goal
        if not (0 <= sx < w and 0 <= sy < h):
            return None
        if not (0 <= gx < w and 0 <= gy < h):
            return None
        if blocked[gy, gx]:
            # Goal cell itself is blocked. Try to find the nearest free cell
            # within ~1 m and use that — better than failing outright when
            # the user clicked slightly into a wall by accident.
            free = self._nearest_free(blocked, gx, gy,
                                      max_dist_cells=int(round(1.0 / max(1, 1))))
            if free is None:
                return None
            gx, gy = free
            goal = (gx, gy)
        if blocked[sy, sx]:
            # Start cell shouldn't be blocked, but if it is (bad pose or
            # inflated into a wall) snap to the nearest free cell.
            free = self._nearest_free(blocked, sx, sy, max_dist_cells=10)
            if free is None:
                return None
            sx, sy = free
            start = (sx, sy)

        open_heap: list = []
        heapq.heappush(open_heap, (0.0, start))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            cx, cy = current
            cg = g_score[current]
            for dx, dy, cost in self._NEIGHBORS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                if blocked[ny, nx]:
                    continue
                # For diagonals, don't allow squeezing between two diagonal
                # walls (the "corner-cut" problem)
                if dx != 0 and dy != 0:
                    if blocked[cy, cx + dx] or blocked[cy + dy, cx]:
                        continue
                tentative = cg + cost
                neighbor = (nx, ny)
                if tentative < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score = tentative + self._heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_score, neighbor))
        return None

    @staticmethod
    def _nearest_free(blocked: np.ndarray, cx: int, cy: int,
                      max_dist_cells: int = 10) -> Optional[tuple[int, int]]:
        """BFS-ish outward search for a non-blocked cell near (cx, cy)."""
        h, w = blocked.shape
        for r in range(1, max_dist_cells + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h and not blocked[ny, nx]:
                        return nx, ny
        return None

    # ─── Path simplification (line-of-sight) ─────────────────────────────

    @staticmethod
    def _has_los(blocked: np.ndarray,
                 a: tuple[int, int], b: tuple[int, int]) -> bool:
        """Return True if a straight line of cells from a to b is unblocked.
        Bresenham."""
        x0, y0 = a; x1, y1 = b
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            if blocked[y0, x0]:
                return False
            if (x0, y0) == (x1, y1):
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def simplify(self, path: list[tuple[int, int]],
                 blocked: np.ndarray) -> list[tuple[int, int]]:
        """Reduce a jagged cell-by-cell A* path to a small list of corner
        waypoints using line-of-sight. Drop any waypoint that's visible
        from the previous corner."""
        if len(path) <= 2:
            return list(path)
        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._has_los(blocked, path[i], path[j]):
                j -= 1
            simplified.append(path[j])
            i = j
        return simplified

    # ─── Public API ──────────────────────────────────────────────────────

    def plan(self, start_world: tuple[float, float],
             goal_world: tuple[float, float],
             grid: np.ndarray, info: GridInfo
             ) -> Optional[list[tuple[float, float]]]:
        """Plan a path from start_world to goal_world. Returns a list of
        (x, y) world-frame waypoints from start to goal (inclusive),
        simplified to line-of-sight corners; None if no path exists."""
        blocked = self.build_blocked(grid, info)
        start = self.world_to_cell(*start_world, info)
        goal = self.world_to_cell(*goal_world, info)
        cell_path = self._astar(blocked, start, goal)
        if cell_path is None:
            return None
        corners = self.simplify(cell_path, blocked)
        return [self.cell_to_world(cx, cy, info) for cx, cy in corners]
