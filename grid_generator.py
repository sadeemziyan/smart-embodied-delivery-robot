import numpy as np
import random
from collections import deque


class GridGenerator:
    """
    Procedural grid generator for simulation testing.
    Creates occupancy grids with random walls and ensures start-goal connectivity.
    """

    def __init__(self, width=15, height=15, cell_size=0.35):
        """
        Initialize grid generator.

        Args:
            width: Number of cells in x direction
            height: Number of cells in y direction
            cell_size: Size of each cell in meters
        """
        self.width = width
        self.height = height
        self.cell_size = cell_size

    def generate(self, obstacle_density=0.15, seed=None):
        """
        Generate a grid with random obstacles and ensure start-goal connectivity.

        Args:
            obstacle_density: Probability of a cell being a wall (0-1)
            seed: Random seed for reproducibility

        Returns:
            dict with keys:
                - 'grid': 2D numpy array (width, height) with values 0=free, 1=wall, 3=stair
                - 'start_pos': (gx, gy) tuple in grid indices
                - 'goal_pos': (gx, gy) tuple in grid indices
                - 'metadata': dict with num_obstacles, solvable bool
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        # Initialize grid with free cells (0)
        grid = np.zeros((self.width, self.height), dtype=np.int8)

        # Place random walls based on obstacle density
        obstacle_mask = np.random.random((self.width, self.height)) < obstacle_density
        grid[obstacle_mask] = 1

        # Ensure border cells are free for robot safety
        grid[0, :] = 0
        grid[-1, :] = 0
        grid[:, 0] = 0
        grid[:, -1] = 0

        # Enforce minimum free space to prevent over-cluttering
        grid = self._enforce_minimum_free_space(grid, min_free_ratio=0.4)

        # Sample start and goal positions with 1-cell border
        free_cells = np.argwhere(grid == 0)
        if len(free_cells) < 2:
            raise RuntimeError("Not enough free cells to place start and goal")

        start_idx = random.choice(range(len(free_cells)))
        start_pos = tuple(free_cells[start_idx])

        # Remove start from available cells to ensure different goal
        remaining_cells = np.delete(free_cells, start_idx, axis=0)
        goal_idx = random.choice(range(len(remaining_cells)))
        goal_pos = tuple(remaining_cells[goal_idx])

        # Verify connectivity using BFS
        if not self._is_connected(grid, start_pos, goal_pos):
            grid = self._carve_path(grid, start_pos, goal_pos)

        metadata = {
            'num_obstacles': int(np.sum(grid == 1)),
            'solvable': self._is_connected(grid, start_pos, goal_pos)
        }

        return {
            'grid': grid,
            'start_pos': start_pos,
            'goal_pos': goal_pos,
            'metadata': metadata
        }

    def _enforce_minimum_free_space(self, grid, min_free_ratio=0.4):
        """
        Remove random walls until at least min_free_ratio of cells
        are in the largest connected free component.

        Args:
            grid: 2D numpy array (modified in-place)
            min_free_ratio: Minimum fraction of cells that must be free and connected
        Returns:
            Modified grid
        """
        while True:
            largest = self._largest_free_component(grid)
            total = grid.shape[0] * grid.shape[1]
            if largest / total >= min_free_ratio:
                break
            # Remove a random wall cell
            wall_cells = np.argwhere(grid == 1)
            if len(wall_cells) == 0:
                break
            idx = np.random.randint(len(wall_cells))
            grid[wall_cells[idx][0], wall_cells[idx][1]] = 0
        return grid

    def _largest_free_component(self, grid) -> int:
        """Return size of largest connected free component using BFS."""
        visited = np.zeros_like(grid, dtype=bool)
        best = 0
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if grid[i, j] == 0 and not visited[i, j]:
                    size = 0
                    queue = deque([(i, j)])
                    visited[i, j] = True
                    while queue:
                        cx, cy = queue.popleft()
                        size += 1
                        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                            nx, ny = cx+dx, cy+dy
                            if (0 <= nx < grid.shape[0] and
                                0 <= ny < grid.shape[1] and
                                not visited[nx, ny] and
                                grid[nx, ny] == 0):
                                visited[nx, ny] = True
                                queue.append((nx, ny))
                    best = max(best, size)
        return best

    def _is_connected(self, grid, start, goal):
        """
        Check if there is a path from start to goal using BFS.

        Args:
            grid: 2D numpy array with 0=free, 1=wall
            start: (x, y) tuple
            goal: (x, y) tuple

        Returns:
            bool: True if path exists
        """
        if start == goal:
            return True

        width, height = grid.shape
        visited = np.zeros((width, height), dtype=bool)
        queue = deque([start])
        visited[start] = True

        # 4-directional movement (Manhattan)
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        while queue:
            current = queue.popleft()
            if current == goal:
                return True

            cx, cy = current
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < width and 0 <= ny < height and
                    not visited[nx, ny] and grid[nx, ny] == 0):
                    visited[nx, ny] = True
                    queue.append((nx, ny))

        return False

    def _carve_path(self, grid, start, goal):
        """
        Carve a Manhattan path between start and goal.

        Args:
            grid: 2D numpy array to modify
            start: (x, y) tuple
            goal: (x, y) tuple

        Returns:
            Modified grid with a carved path
        """
        sx, sy = start
        gx, gy = goal

        # Create path: horizontal first, then vertical
        x_dir = 1 if gx > sx else -1
        y_dir = 1 if gy > sy else -1

        # Carve horizontal segment
        for x in range(sx, gx + x_dir, x_dir):
            grid[x, sy] = 0

        # Carve vertical segment
        for y in range(sy, gy + y_dir, y_dir):
            grid[gx, y] = 0

        return grid
