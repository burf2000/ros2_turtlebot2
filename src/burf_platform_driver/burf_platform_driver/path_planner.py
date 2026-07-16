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

    OCCUPIED_THRESHOLD = 50     # cells with cost >=50 are obstacles
    # Unknown cells (-1) are UNEXPLORED, not obstacles. For EXPLORE we plan
    # THROUGH unknown so the planner can route toward frontiers (which sit at the
    # edge of unknown); blocking them there stalls exploration. For a MANUAL goto
    # the opposite is true — routing through unmapped space drives the robot blind
    # into walls that were never seen. So unknown-handling is a per-call decision
    # (plan(allow_unknown=...)); this class default is the safe one (block).
    UNKNOWN_AS_BLOCKED = True
    # Occupied blobs this size or smaller are treated as SLAM salt-and-pepper
    # noise and dropped before inflation. A real wall is a large connected run of
    # cells (tens to hundreds); a 1-3 cell speckle in open space is almost always
    # a spurious return. Left in, each gets inflated by the robot radius and
    # pinches doorways/passages shut, making the far side of the map unreachable.
    MIN_OCCUPIED_COMPONENT = 3
    # For a manual goto we don't route through UNMAPPED space (drive blind) — but
    # blocking ALL unknown is too strict: it refuses a perfectly mapped-free goal
    # whose direct route merely clips the ragged frontier edge. So we block only
    # DEEP unknown — cells more than UNKNOWN_REACH_M from any mapped-free cell.
    # Unknown within this rind of free space stays passable (the robot only ever
    # noses a short, bounded distance into the unmapped, and SLAM fills it in as
    # it approaches); a genuine open void has an interior beyond the rind → blocked.
    UNKNOWN_REACH_M = 0.30

    def __init__(self, robot_radius_m: float = 0.18,
                 inflation_padding_m: float = 0.05,
                 unknown_reach_m: float = UNKNOWN_REACH_M):
        """robot_radius_m: half the robot's longest dimension.
           inflation_padding_m: extra buffer beyond robot_radius. Bumps the
           planner away from hugging walls.
           unknown_reach_m: how far a manual goto may nose into unmapped space
           beyond the last mapped-free cell (deep unknown past this is blocked)."""
        self.robot_radius_m = float(robot_radius_m)
        self.inflation_padding_m = float(inflation_padding_m)
        self.unknown_reach_m = float(unknown_reach_m)

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

    @classmethod
    def _denoise(cls, occ: np.ndarray) -> np.ndarray:
        """Drop small isolated occupied blobs — salt-and-pepper SLAM noise. A
        real wall is a large connected run of cells; a 1-3 cell speckle in open
        space is almost always a spurious return. Left in, each gets inflated by
        the robot radius and pinches doorways/passages shut, which makes the far
        side of the map unreachable. Dropping every connected occupied component
        of size <= MIN_OCCUPIED_COMPONENT keeps a real safety margin at genuine
        walls while letting the robot through passages 'blocked' only by noise."""
        if not occ.any():
            return occ
        try:
            from scipy.ndimage import label
            # 8-connectivity: label every occupied blob, drop the small ones.
            lab, n = label(occ, structure=np.ones((3, 3), np.int32))
            if n == 0:
                return occ
            sizes = np.bincount(lab.ravel())      # sizes[0] = background
            keep = sizes > cls.MIN_OCCUPIED_COMPONENT
            keep[0] = False                        # background is never occupied
            return keep[lab]
        except ImportError:
            # Pure-numpy fallback (no scipy): can only cheaply remove fully
            # isolated cells (component size 1), not larger blobs. Count occupied
            # 8-neighbours and keep cells that have at least one.
            h, w = occ.shape
            cnt = np.zeros((h, w), dtype=np.int32)
            oi = occ.astype(np.int32)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    ys0, ys1 = max(0, -dy), h - max(0, dy)
                    xs0, xs1 = max(0, -dx), w - max(0, dx)
                    yd0, yd1 = max(0, dy), h - max(0, -dy)
                    xd0, xd1 = max(0, dx), w - max(0, -dx)
                    cnt[yd0:yd1, xd0:xd1] += oi[ys0:ys1, xs0:xs1]
            return occ & (cnt >= 1)

    def _deep_unknown(self, grid: np.ndarray, info: GridInfo) -> np.ndarray:
        """Boolean grid of unknown cells that are DEEP in an unmapped void — more
        than unknown_reach_m from any mapped-free cell. Unknown within that rind
        of free space is NOT flagged (a manual goto may nose that far into the
        unmapped). Used only when blocking unknown for goto."""
        unknown = grid < 0
        free = grid == 0
        if not unknown.any():
            return np.zeros_like(unknown)
        if not free.any():
            return unknown          # nothing mapped free → treat all unknown as void
        reach = max(1, int(round(self.unknown_reach_m / info.resolution)))
        try:
            from scipy.ndimage import distance_transform_edt
            # distance (in cells) from every cell to the nearest mapped-free cell
            dist_to_free = distance_transform_edt(~free)
            return unknown & (dist_to_free > reach)
        except ImportError:
            # No scipy: fall back to the strict rule (block all unknown). Safe,
            # just fussier — only hit in a dev env without scipy.
            return unknown

    def build_blocked(self, grid: np.ndarray, info: GridInfo,
                      allow_unknown: Optional[bool] = None) -> np.ndarray:
        """Take the raw int8 OccupancyGrid (-1 unknown, 0 free, 100 occupied)
        and produce a boolean grid where True = "robot cannot stand here".

        allow_unknown: if True, unknown (-1) cells are traversable (EXPLORE —
        route toward frontiers). If False, only DEEP unknown is blocked (manual
        GOTO — reach mapped-free goals across a thin frontier rind, but never
        plunge across an open void). None falls back to the class default."""
        block_unknown = (self.UNKNOWN_AS_BLOCKED if allow_unknown is None
                         else not allow_unknown)
        occ = self._denoise(grid >= self.OCCUPIED_THRESHOLD)
        # Inflate the REAL obstacles by the robot radius for wall clearance.
        blocked = self._inflate(occ, info)
        if block_unknown:
            # Add deep unmapped void, un-inflated: the reach rind already provides
            # the margin, and inflating it would eat back into the passable rind.
            blocked = blocked | self._deep_unknown(grid, info)
        return blocked

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
               goal: tuple[int, int],
               resolution: float) -> Optional[list[tuple[int, int]]]:
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
                                      max_dist_cells=max(1, int(round(1.0 / resolution))))
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
    def _clear_disc(blocked: np.ndarray, center: tuple[int, int],
                    radius_m: float, resolution: float) -> None:
        """Force a disc of cells around `center` to be unblocked, in place.
        Used to carve out the robot's own footprint so a phantom occupied cell
        beside the robot can never make the robot's start position unplannable."""
        r = max(1, int(round(radius_m / resolution)))
        cx, cy = center
        h, w = blocked.shape
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    x, y = cx + dx, cy + dy
                    if 0 <= x < w and 0 <= y < h:
                        blocked[y, x] = False

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
             grid: np.ndarray, info: GridInfo,
             allow_unknown: Optional[bool] = None
             ) -> Optional[list[tuple[float, float]]]:
        """Plan a path from start_world to goal_world. Returns a list of
        (x, y) world-frame waypoints from start to goal (inclusive),
        simplified to line-of-sight corners; None if no path exists.

        allow_unknown: True for exploration (plan through unmapped space toward
        frontiers); False for manual goto (refuse to route through unknown so the
        robot never drives blind); None uses the class default."""
        blocked = self.build_blocked(grid, info, allow_unknown=allow_unknown)
        start = self.world_to_cell(*start_world, info)
        goal = self.world_to_cell(*goal_world, info)
        # The robot is physically AT `start`, so its own footprint cannot be an
        # obstacle — yet SLAM often marks a cell occupied right next to the robot
        # (sensor noise, the robot seeing part of itself, a person alongside it).
        # That inflates over the robot's cell and boxes the planner in, so EVERY
        # goal fails with "no path". Clear a robot-radius disc around the start
        # so planning can always begin from where the robot actually stands.
        self._clear_disc(blocked, start, self.robot_radius_m, info.resolution)
        cell_path = self._astar(blocked, start, goal, info.resolution)
        if cell_path is None:
            return None
        corners = self.simplify(cell_path, blocked)
        return [self.cell_to_world(cx, cy, info) for cx, cy in corners]
