"""Implementation of the navigation map component for the delivery robot project.

This component processes raw occupancy grids from GridGenerator, inflates obstacles
for robot clearance, provides coordinate transformations, and supports path planning.
"""

import numpy as np
from scipy.ndimage import binary_dilation
from typing import Tuple, List, Optional, Union
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import os


class NavigationMap:
    """Component 1 of 3: Navigation map processing based on Cell Generator."""

    def __init__(self, robot_radius_m: float = 0.4):
        """
        Initialize navigation map.

        Args:
            robot_radius_m: Robot clearance radius in meters (default 0.4)
        """
        self.robot_radius_m = robot_radius_m
        self.cell_size_m = 0.0                      # meters per cell
        self.width_cells = 0                        # grid width in cells
        self.height_cells = 0                       # grid height in cells
        self.origin_m = np.array([0.0, 0.0])        # world coordinates of (0,0)
        self.raw_grid_np: Optional[np.ndarray] = None       # raw grid from generator
        self.inflated_grid_np: Optional[np.ndarray] = None  # binary inflated grid
        self._sim_start_grid: Optional[Tuple[int, int]] = None   # start position in grid indices
        self._sim_goal_grid: Optional[Tuple[int, int]] = None    # goal position in grid indices
        self.cost_map_obj: Optional['_CostMap'] = None

    def _create_cost_map(self) -> '_CostMap':
        """Create internal cost map representation from raw grid."""
        if self.raw_grid_np is None:
            raise RuntimeError("Raw grid not set - cannot create cost map")
        return _CostMap(self.raw_grid_np)

    def build(self, generator, obstacle_density: float = 0.12,
              max_retries: int = 20) -> None:
        """
        Build the navigation map from a GridGenerator instance.

        This processes the raw grid to create an inflated occupancy map suitable for
        path planning while ensuring start and goal positions remain in free space
        and are connected on the inflated grid.

        Args:
            generator: GridGenerator instance that provides raw grid data
            obstacle_density: Density passed to generator.generate()
            max_retries: Maximum number of seeds to try for connectivity
        """
        for attempt in range(max_retries):
            seed = 42 + attempt
            generator_output = generator.generate(
                obstacle_density=obstacle_density,
                seed=seed
            )
            self.raw_grid_np = generator_output['grid']
            self.cell_size_m = generator.cell_size
            self.width_cells = generator.width
            self.height_cells = generator.height

            self.cost_map_obj = self._create_cost_map()

            # Inflation parameters: use half radius to avoid over-blocking small grids
            effective_radius = self.robot_radius_m * 0.5
            inflate_cells = max(1, int(np.round(effective_radius / self.cell_size_m)))
            self.inflated_grid_np = binary_dilation(
                self.cost_map_obj._obstacles,
                iterations=inflate_cells,
                output=np.zeros_like(self.cost_map_obj._obstacles)
            )

            # Determine start and goal positions (grid indices)
            self._sim_start_grid = generator_output['start_pos']
            self._sim_goal_grid = generator_output['goal_pos']

            # Verify start and goal are in free space after inflation
            self._verify_or_adjust_positions()

            # Check connectivity on inflated grid
            if self._are_connected_in_inflated(
                    self._sim_start_grid, self._sim_goal_grid):
                print(f"[NavigationMap] Connected on attempt "
                      f"{attempt+1} (seed={seed})")
                self._check_inflation_sanity()
                self._print_build_stats()
                return

            print(f"[NavigationMap] Attempt {attempt+1}: "
                  f"start/goal disconnected in inflated grid, "
                  f"retrying with seed={seed+1}...")

        raise RuntimeError(
            f"[NavigationMap] Could not find connected map in "
            f"{max_retries} attempts. "
            f"Try reducing obstacle_density or robot_radius_m."
        )

    def _verify_or_adjust_positions(self) -> None:
        """Ensure start and goal positions are free after inflation."""
        def find_nearest_free(target_pos):
            """Find nearest free cell using BFS."""
            visited = np.zeros_like(self.inflated_grid_np, dtype=bool)
            queue = deque([target_pos])
            visited[target_pos] = True
            while queue:
                cx, cy = queue.popleft()
                if not self.inflated_grid_np[cx, cy]:  # free cell
                    return (cx, cy)
                for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                    nx, ny = cx+dx, cy+dy
                    if 0 <= nx < self.width_cells and 0 <= ny < self.height_cells and not visited[nx, ny]:
                        visited[nx, ny] = True
                        queue.append((nx, ny))
            raise RuntimeError(f"No free cell found near {target_pos}")

        try:
            self._sim_start_grid = find_nearest_free(self._sim_start_grid)
            self._sim_goal_grid = find_nearest_free(self._sim_goal_grid)
        except RuntimeError as e:
            raise RuntimeError(f"Failed to find valid start/goal positions: {e}")

    def _check_inflation_sanity(self) -> None:
        """
        Warn if inflation has blocked too much of the grid.
        Raises RuntimeError if less than 20% of cells are free.
        """
        total = self.width_cells * self.height_cells
        blocked = int(np.sum(self.inflated_grid_np))
        free = total - blocked
        free_ratio = free / total

        print(f"[NavigationMap] Inflation result: {blocked}/{total} blocked ({free_ratio*100:.1f}% free)")

        if free_ratio < 0.30:
            raise RuntimeError(
                f"[NavigationMap] Inflation blocked {blocked}/{total} cells ({free_ratio*100:.1f}% free). "
                f"Reduce obstacle_density or robot_radius_m. "
                f"Current: obstacle_density see generator, "
                f"radius={self.robot_radius_m}m, "
                f"cell_size={self.cell_size_m}m, "
                f"effective_radius={self.robot_radius_m*0.5}m"
            )

    def _are_connected_in_inflated(self, start: Tuple[int, int],
                                    goal: Tuple[int, int]) -> bool:
        """
        BFS on the inflated grid to check start-goal connectivity.

        Args:
            start: (gx, gy) grid coordinates
            goal: (gx, gy) grid coordinates
        Returns:
            bool: True if connected path exists
        """
        from collections import deque
        if not self.is_free(*start) or not self.is_free(*goal):
            return False
        if start == goal:
            return True

        visited = set()
        queue = deque([start])
        visited.add(start)

        while queue:
            cx, cy = queue.popleft()
            if (cx, cy) == goal:
                return True
            # 8-directional movement to allow diagonal planning
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1),
                           (1,1), (1,-1), (-1,1), (-1,-1)]:
                nx, ny = cx + dx, cy + dy
                if ((nx, ny) not in visited and
                    0 <= nx < self.width_cells and
                    0 <= ny < self.height_cells and
                    self.is_free(nx, ny)):
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        return False

    def _print_build_stats(self) -> None:
        """Print helpful build statistics."""
        total = self.width_cells * self.height_cells
        blocked = int(np.sum(self.inflated_grid_np))
        free = total - blocked
        sw = self.get_simulated_start_world()
        gw = self.get_simulated_goal_world()
        print(f"[NavigationMap] Grid: {self.width_cells}x{self.height_cells} cells at {self.cell_size_m}m/cell")
        print(f"[NavigationMap] Free: {free}/{total} ({100*free/total:.1f}%)")
        print(f"[NavigationMap] Start: {sw}  Goal: {gw}")

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid indices.

        Args:
            world_x: World x coordinate (meters)
            world_y: World y coordinate (meters)
        Returns:
            Tuple[int, int]: (grid_x, grid_y) indices
        """
        grid_x = int(np.round((world_x - self.origin_m[0]) / self.cell_size_m))
        grid_y = int(np.round((world_y - self.origin_m[1]) / self.cell_size_m))

        # Clamp to grid boundaries
        grid_x = np.clip(grid_x, 0, self.width_cells - 1)
        grid_y = np.clip(grid_y, 0, self.height_cells - 1)
        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Convert grid indices to world coordinates.

        Args:
            grid_x: Grid x index
            grid_y: Grid y index
        Returns:
            Tuple[float, float]: (world_x, world_y) in meters
        """
        world_x = self.origin_m[0] + grid_x * self.cell_size_m
        world_y = self.origin_m[1] + grid_y * self.cell_size_m
        return world_x, world_y

    def is_free(self, grid_x: int, grid_y: int) -> bool:
        """
        Check if a grid cell is free (unoccupied).

        Args:
            grid_x: Grid x index
            grid_y: Grid y index
        Returns:
            bool: True if cell is free
        """
        if (self.inflated_grid_np is None):
            raise RuntimeError("Map not built yet")
        if not (0 <= grid_x < self.width_cells and 0 <= grid_y < self.height_cells):
            return False
        return not bool(self.inflated_grid_np[grid_x, grid_y])

    def get_inflated_grid(self) -> np.ndarray:
        """
        Get the inflated occupancy grid.

        Returns:
            np.ndarray: Binary array (1=obstacle, 0=free)
        """
        if self.inflated_grid_np is None:
            raise RuntimeError("Map not built yet")
        return self.inflated_grid_np.copy()

    def get_raw_grid(self) -> np.ndarray:
        """
        Get the raw occupancy grid from generator.

        Returns:
            np.ndarray: Raw grid data
        """
        if self.raw_grid_np is None:
            raise RuntimeError("Map not built yet")
        return self.raw_grid_np.copy()

    def get_start_position(self) -> Tuple[int, int]:
        """Return start position in grid coordinates."""
        if self._sim_start_grid is None:
            raise RuntimeError("Start position not set")
        return self._sim_start_grid

    def get_goal_position(self) -> Tuple[int, int]:
        """Return goal position in grid coordinates."""
        if self._sim_goal_grid is None:
            raise RuntimeError("Goal position not set")
        return self._sim_goal_grid

    def get_simulated_start_world(self) -> Tuple[float, float]:
        """Get start position in world coordinates (meters)."""
        gx, gy = self.get_start_position()
        return self.grid_to_world(gx, gy)

    def get_simulated_goal_world(self) -> Tuple[float, float]:
        """Get goal position in world coordinates (meters)."""
        gx, gy = self.get_goal_position()
        return self.grid_to_world(gx, gy)

    def get_grid_dimensions(self) -> Tuple[int, int]:
        """Return grid dimensions."""
        return self.width_cells, self.height_cells

    def get_cell_size(self) -> float:
        """Return cell size in meters."""
        return self.cell_size_m

    def get_origin(self) -> Tuple[float, float]:
        """Return world origin coordinates."""
        return tuple(self.origin_m)

    def set_origin(self, wx: float, wy: float) -> None:
        """Set world origin coordinates."""
        self.origin_m = np.array([wx, wy])

    def get_world_size(self) -> Tuple[float, float]:
        """Return world dimensions."""
        return self.width_cells * self.cell_size_m, self.height_cells * self.cell_size_m

    def verify_alignment(self, robot_world_pos: Tuple[float, float],
                        goal_world_pos: Optional[Tuple[float, float]] = None) -> Optional[str]:
        """
        Generate debug visualization showing map alignment.

        Args:
            robot_world_pos: (x, y) world coordinates of robot
            goal_world_pos: Optional goal world coordinates
        Returns:
            Path to saved debug image file
        """
        # Print free space ratio
        total = self.width_cells * self.height_cells
        free = total - np.sum(self.inflated_grid_np)
        free_ratio = free / total
        print(f"[NavigationMap] Free space after inflation: {free_ratio*100:.1f}%")
        if free_ratio < 0.30:
            print(f"[NavigationMap] WARNING: <30% free. Reduce obstacle_density or robot_radius_m.")

        # Create visualization
        fig, ax = plt.subplots(figsize=(8, 8), dpi=150)

        # Convert grid to world coordinates for plotting
        gx_range = range(self.width_cells)
        gy_range = range(self.height_cells)
        xx, yy = np.meshgrid(gx_range, gy_range)
        wx_grid = self.origin_m[0] + xx * self.cell_size_m
        wy_grid = self.origin_m[1] + yy * self.cell_size_m

        # Plot occupancy
        ax.pcolormesh(wx_grid, wy_grid, self.inflated_grid_np, cmap='Greys', alpha=0.7)

        # Plot robot position
        ax.scatter(robot_world_pos[0], robot_world_pos[1],
                  color='red', s=100, marker='o', label='Robot Position')

        # Plot goal if provided
        if goal_world_pos:
            ax.scatter(goal_world_pos[0], goal_world_pos[1],
                      color='blue', s=150, marker='*', label='Goal Position')

        # Add labels and legend
        ax.set_xlabel('World X (m)')
        ax.set_ylabel('World Y (m)')
        ax.set_title('Navigation Map Alignment')
        ax.legend()
        ax.grid(True)

        # Save and return path
        debug_image_path = 'map_debug.png'
        plt.tight_layout()
        plt.savefig(debug_image_path, bbox_inches='tight')
        plt.close(fig)
        return debug_image_path


class _CostMap:
    """Internal helper class for processing raw grid data."""

    def __init__(self, raw_grid: np.ndarray):
        """
        Create cost map from raw grid.

        Args:
            raw_grid: Raw occupancy grid from generator
        """
        self._raw_grid = raw_grid.astype(np.int32)

        # Free = 0, Wall = 1, Stair = 3
        self._free = raw_grid == 0
        self._wall = raw_grid == 1
        self._stair = raw_grid == 3

        # Binary occupancy for dilation: true where obstacle (wall or stair)
        self._obstacles = np.logical_or(self._wall, self._stair)


def main():
    """Test script for basic NavigationMap functionality."""
    from grid_generator import GridGenerator
    import tempfile
    import shutil

    # Clean up any existing debug images
    for img in ['map_debug.png']:
        if os.path.exists(img):
            os.remove(img)

    try:
        # Generate test map
        generator = GridGenerator(width=15, height=15, cell_size=0.35)
        nav_map = NavigationMap(robot_radius_m=0.4)
        raw_output = nav_map.build(generator)

        # Test coordinate conversions
        start_world = nav_map.get_simulated_start_world()
        goal_world = nav_map.get_simulated_goal_world()
        print(f"Start world: {start_world}")
        print(f"Goal world: {goal_world}")

        # Test boundary conversion
        test_gridPos = nav_map.world_to_grid(*start_world)
        test_worldPos = nav_map.grid_to_world(*test_gridPos)
        print(f"Converted back: {test_worldPos}")

        # Generate debug image
        debug_path = nav_map.verify_alignment(start_world)
        print(f"Debug image saved to: {debug_path}")

        return 0

    except Exception as e:
        print(f"Error during test: {e}")
        return 1


if __name__ == '__main__':
    exit(main())