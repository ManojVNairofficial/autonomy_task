import math
import random
import numpy as np


class RRTNode:
    __slots__ = ('x', 'y', 'parent', 'cost')

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None   
        self.cost = 0.0      


class RRTPlanner:
    """

    Grid value convention (nav_msgs/OccupancyGrid):
        0   = free
        100 = occupied
        -1  = unknown  (treated as occupied).
    
    """

    def __init__(
        self,
        step_size: float = 0.1,
        max_iter: int = 5000,
        goal_bias: float = 0.1,
        goal_threshold: float = 0.2,
    ):
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_bias = goal_bias
        self.goal_threshold = goal_threshold

        self._grid: np.ndarray | None = None
        self._width = 0
        self._height = 0
        self._resolution = 0.05
        self._origin_x = 0.0
        self._origin_y = 0.0

    
    # ---------------------------------------------------------------------------- #
    #                                 Map ingestion                                #
    # ---------------------------------------------------------------------------- #
    

    def set_map(
        self,
        grid_data: list,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> None:
        self._grid = np.array(grid_data, dtype=np.int8).reshape(height, width)
        self._width = width
        self._height = height
        self._resolution = resolution
        self._origin_x = origin_x
        self._origin_y = origin_y

    # ---------------------------------------------------------------------------- #
    #                              Coordinate helpers                              #
    # ---------------------------------------------------------------------------- #


    def world_to_grid(self, wx: float, wy: float):
        col = int((wx - self._origin_x) / self._resolution)
        row = int((wy - self._origin_y) / self._resolution)
        return col, row

    def grid_to_world(self, col: int, row: int):
        wx = col * self._resolution + self._origin_x + self._resolution * 0.5
        wy = row * self._resolution + self._origin_y + self._resolution * 0.5
        return wx, wy


    # ---------------------------------------------------------------------------- #
    #                         Collision / free-space checks                        #
    # ---------------------------------------------------------------------------- #

    def _in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self._width and 0 <= row < self._height

    def _cell_free(self, col: int, row: int) -> bool:
        if not self._in_bounds(col, row):
            return False
        return int(self._grid[row, col]) == 0

    def _collision_free(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        dist = math.hypot(x2 - x1, y2 - y1)
        # --------- Step at half-cell resolution to avoid skipping thin walls -------- #
        steps = max(int(dist / (self._resolution * 0.5)), 2)
        for i in range(steps + 1):
            t = i / steps
            col, row = self.world_to_grid(x1 + t * (x2 - x1), y1 + t * (y2 - y1))
            if not self._cell_free(col, row):
                return False
        return True


    # ------------------------------ RRT primitives ------------------------------ #

    def _sample(self, gx: float, gy: float):
        if random.random() < self.goal_bias:
            return gx, gy
        x = random.uniform(self._origin_x,
                           self._origin_x + self._width  * self._resolution)
        y = random.uniform(self._origin_y,
                           self._origin_y + self._height * self._resolution)
        return x, y

    @staticmethod
    def _nearest(nodes: list, x: float, y: float) -> int:
        best_idx, best_d = 0, float('inf')
        for i, n in enumerate(nodes):
            d = (n.x - x) ** 2 + (n.y - y) ** 2
            if d < best_d:
                best_d, best_idx = d, i
        return best_idx

    def _steer(self, from_node: RRTNode, tx: float, ty: float):
        d = math.hypot(tx - from_node.x, ty - from_node.y)
        if d <= self.step_size:
            return tx, ty
        theta = math.atan2(ty - from_node.y, tx - from_node.x)
        return (from_node.x + self.step_size * math.cos(theta),
                from_node.y + self.step_size * math.sin(theta))

    @staticmethod
    def _extract_path(nodes: list) -> list:
        path = []
        idx = len(nodes) - 1
        while idx is not None:
            n = nodes[idx]
            path.append((n.x, n.y))
            idx = n.parent
        path.reverse()
        return path


    # ---------------------------------------------------------------------------- #
    #                          Public planning entry point                         #
    # ---------------------------------------------------------------------------- #

    def plan(self, start_x: float, start_y: float, goal_x: float, goal_y: float):
        """
        Returns (path, all_nodes).

        path  — list of (x, y) world-frame waypoints, or None on failure.
        nodes — full RRT tree for visualisation.
        """
        if self._grid is None:
            raise RuntimeError('Map not set. Call set_map() first.')

        nodes = [RRTNode(start_x, start_y)]

        for _ in range(self.max_iter):
            rx, ry = self._sample(goal_x, goal_y)
            ni = self._nearest(nodes, rx, ry)
            nearest = nodes[ni]
            nx, ny = self._steer(nearest, rx, ry)

            if not self._collision_free(nearest.x, nearest.y, nx, ny):
                continue

            new_node = RRTNode(nx, ny)
            new_node.parent = ni
            new_node.cost = nearest.cost + math.hypot(nx - nearest.x, ny - nearest.y)
            nodes.append(new_node)

            if math.hypot(nx - goal_x, ny - goal_y) <= self.goal_threshold:
                if self._collision_free(nx, ny, goal_x, goal_y):
                    goal_node = RRTNode(goal_x, goal_y)
                    goal_node.parent = len(nodes) - 1
                    nodes.append(goal_node)
                    return self._extract_path(nodes), nodes

        return None, nodes
