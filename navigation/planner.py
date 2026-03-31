"""
A* path planning module for the delivery robot navigation stack.

This module implements the AStarPlanner class that plans paths on an
inflated occupancy grid using 8-directional movement with Euclidean
heuristic and corner cutting prevention.
"""

import heapq
import math
from typing import List, Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt


class AStarPlanner:
    """
    A* path planner for navigation on an inflated occupancy grid.

    The planner uses:
      - 8-directional movement (cardinal and diagonal)
      - Euclidean heuristic in world coordinates
      - Corner-cutting prevention for diagonal moves
      - Inflation-safe path (all cells on path are guaranteed free)
    """

    def __init__(self, nav_map):
        """
        Initialize A* planner.

        Args:
            nav_map: NavigationMap instance with inflated grid ready
        """
        self.nav_map = nav_map
        self.cell_size = nav_map.cell_size_m

        # Precompute movement costs
        self._cardinal_cost = self.cell_size
        self._diagonal_cost = self.cell_size * math.sqrt(2)

    def plan(self, start_world: Tuple[float, float],
             goal_world: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Plan a path from start to goal.

        Args:
            start_world: (x, y) world coordinates in meters
            goal_world: (x, y) world coordinates in meters

        Returns:
            List of waypoints in world coordinates.
            Returns empty list if planning fails.
        """
        # Convert to grid coordinates
        start_gx, start_gy = self.nav_map.world_to_grid(*start_world)
        goal_gx, goal_gy = self.nav_map.world_to_grid(*goal_world)

        # Trivial case: already at goal
        if (start_gx, start_gy) == (goal_gx, goal_gy):
            print("[AStarPlanner] Start equals goal, trivial path")
            return [goal_world]

        # Validate that positions are free
        if not self.nav_map.is_free(start_gx, start_gy):
            print("[AStarPlanner] Start position is not free!")
            return []
        if not self.nav_map.is_free(goal_gx, goal_gy):
            print("[AStarPlanner] Goal position is not free!")
            return []

        # Run A* search
        path_grid = self._astar_search(start_gx, start_gy, goal_gx, goal_gy)
        if not path_grid:
            print("[AStarPlanner] No path found")
            return []

        # Convert grid path to world coordinates
        world_path = [self.nav_map.grid_to_world(gx, gy) for gx, gy in path_grid]

        # Compute total path length (meters)
        total_length_m = 0.0
        for i in range(1, len(world_path)):
            dx = world_path[i][0] - world_path[i-1][0]
            dy = world_path[i][1] - world_path[i-1][1]
            total_length_m += math.sqrt(dx*dx + dy*dy)

        print(f"[AStarPlanner] Found path: {len(world_path)} waypoints, "
              f"length = {total_length_m:.2f}m")
        return world_path

    def _astar_search(self, sx: int, sy: int, gx: int, gy: int) -> List[Tuple[int, int]]:
        """
        Perform A* search on the inflated occupancy grid.

        Args:
            sx, sy: Start grid coordinates
            gx, gy: Goal grid coordinates

        Returns:
            List of grid coordinates from start to goal, or empty if no path.
        """
        width, height = self.nav_map.width_cells, self.nav_map.height_cells
        inflated = self.nav_map.inflated_grid_np

        # Movement directions: 8-connected
        directions = [
            (1, 0, self._cardinal_cost), (-1, 0, self._cardinal_cost),
            (0, 1, self._cardinal_cost), (0, -1, self._cardinal_cost),
            (1, 1, self._diagonal_cost), (1, -1, self._diagonal_cost),
            (-1, 1, self._diagonal_cost), (-1, -1, self._diagonal_cost)
        ]

        # Heuristic: Euclidean distance in world units
        def heuristic(x, y):
            # Convert cell difference to world meters
            dx_cells = abs(x - gx)
            dy_cells = abs(y - gy)
            # Euclidean distance in meters
            return math.hypot(dx_cells * self.cell_size, dy_cells * self.cell_size)

        open_heap = []
        # Format: (f, g, x, y)
        heapq.heappush(open_heap, (heuristic(sx, sy), 0.0, sx, sy))

        came_from = {}   # For reconstructing path: (x,y) -> (parent_x, parent_y)
        g_score = {(sx, sy): 0.0}
        closed_set = set()

        while open_heap:
            current_f, current_g, cx, cy = heapq.heappop(open_heap)
            current_node = (cx, cy)

            # Goal check
            if (cx, cy) == (gx, gy):
                # Reconstruct path
                path = [current_node]
                while current_node in came_from:
                    current_node = came_from[current_node]
                    path.append(current_node)
                path.reverse()
                return path

            if current_node in closed_set:
                continue
            closed_set.add(current_node)

            # Explore neighbors
            for dx, dy, move_cost in directions:
                nx, ny = cx + dx, cy + dy

                # Boundary check
                if not (0 <= nx < width and 0 <= ny < height):
                    continue

                # Obstacle check
                if inflated[nx, ny]:
                    continue

                # Corner-cutting check for diagonal moves
                if dx != 0 and dy != 0:
                    # Both intermediate cardinal cells must be free
                    if inflated[cx + dx, cy] or inflated[cx, cy + dy]:
                        continue

                tentative_g = current_g + move_cost

                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + heuristic(nx, ny)
                    heapq.heappush(open_heap, (f, tentative_g, nx, ny))
                    came_from[(nx, ny)] = (cx, cy)

        # Exhausted search - no path
        return []

    def visualize(self, world_path: List[Tuple[float, float]],
                  save_path: str = 'planner_debug.png') -> Optional[str]:
        """
        Save a debug visualization of the planned path.

        Args:
            world_path: List of (x, y) world coordinates
            save_path: Output image file path
        Returns:
            The save_path if successful, None otherwise
        """
        try:
            fig, ax = plt.subplots(figsize=(8, 8), dpi=150)

            # Plot inflated occupancy grid
            inflated = self.nav_map.inflated_grid_np
            width_cells = self.nav_map.width_cells
            height_cells = self.nav_map.height_cells
            cell_size = self.cell_size

            # Create world extents for pcolormesh
            x = np.arange(0, width_cells + 1) * cell_size + self.nav_map.origin_m[0]
            y = np.arange(0, height_cells + 1) * cell_size + self.nav_map.origin_m[1]
            X, Y = np.meshgrid(x, y)
            Z = inflated.T  # transpose to match (x,y) orientation
            ax.pcolormesh(X, Y, Z, cmap='Greys', alpha=0.7)

            # Extract path coordinates
            if world_path:
                xs = [p[0] for p in world_path]
                ys = [p[1] for p in world_path]
                ax.plot(xs, ys, '-', color='cyan', linewidth=2, label='Path')
                ax.scatter(xs[0], ys[0], color='green', s=100, marker='o', label='Start')
                ax.scatter(xs[-1], ys[-1], color='blue', s=150, marker='*', label='Goal')

            ax.set_xlabel('World X (m)')
            ax.set_ylabel('World Y (m)')
            ax.set_title('A* Planned Path')
            ax.legend()
            ax.grid(True, linestyle=':')
            ax.set_aspect('equal')

            plt.tight_layout()
            plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
            plt.close(fig)
            return save_path

        except Exception as e:
            print(f"[AStarPlanner] Failed to save visualization: {e}")
            return None

    def get_planner_info(self) -> dict:
        """
        Return information about the planner configuration.
        """
        return {
            'cell_size_m': self.cell_size,
            'cardinal_cost': self._cardinal_cost,
            'diagonal_cost': self._diagonal_cost,
            'grid_width': self.nav_map.width_cells,
            'grid_height': self.nav_map.height_cells,
        }


# Simple self-test when run directly
if __name__ == '__main__':
    import sys
    from grid_generator import GridGenerator
    from navigation.map import NavigationMap

    print("Testing AStarPlanner...")

    # Build map
    gen = GridGenerator(seed=42)
    nav_map = NavigationMap(robot_radius_m=0.4)
    nav_map.build(gen)

    start_world = nav_map.get_simulated_start_world()
    goal_world = nav_map.get_simulated_goal_world()

    planner = AStarPlanner(nav_map)
    path = planner.plan(start_world, goal_world)

    if path:
        print(f"Planner test: success, {len(path)} waypoints")
        planner.visualize(path, 'planner_debug.png')
        print("Saved planner_debug.png")
        sys.exit(0)
    else:
        print("Planner test: no path found")
        sys.exit(1)
